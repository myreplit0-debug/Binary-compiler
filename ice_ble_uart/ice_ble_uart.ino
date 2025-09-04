/*
  SMOKE Unified (ESP32-S3) — Receiver/Bridge v2.2 (room-aware, TX/RX debug)
  (patched for ESP-IDF 5.x send-callback signature)

  HOW TO USE (same as before)
  Room 1:
    setroom 1   ; tail off, esn on
    tail off
    esn on
    channel 6
    save

  Room 2:
    setroom 2   ; tail on, esn on
    tail on
    esn on
    channel 6
    save

  Debug toggles:
    dbg hex on|off
    dbg parse on|off
    dbg names on|off
*/

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>   // <-- for ESP_IDF_VERSION feature test

#include <map>
#include <unordered_map>
#include <vector>

// ======================= CONFIG =======================
#define UART_BAUD_ICE  921600
#define ICE_UART_RX    16
#define ICE_UART_TX    17
#define ICE_UART_RXBUF 16384
#define ICE_UART_TXBUF 2048

// ESPNOW payload max on ESP32 is 250 bytes
#define ESN_PAYLOAD_MAX   250
#define ESN_FRAG_DATA_MAX 200

// ============== MESSAGE FORMAT FROM ICE ===============
enum MsgType : uint8_t { MT_DATA=0, MT_END=1 };

// ============== CHAIN WRAPPER (ESP-NOW) ===============
struct __attribute__((packed)) EsnFrag {
  uint8_t  magic0; uint8_t  magic1; uint8_t  ver; uint8_t  src_room;
  uint8_t  total;  uint8_t  index;  uint8_t  origin[6];
  uint32_t batch;  uint16_t seq;    uint16_t frag_len;
  uint8_t  data[];
};

// ======================= STATE ========================
static uint8_t  g_room    = 1;
static bool     g_is_tail = true;
static bool     g_esn_on  = false;
static uint8_t  g_channel = 6;

// Debug toggles
static bool DBG_HEX   = true;
static bool DBG_PARSE = true;
static bool DBG_NAMES = true;

struct MacKey { uint8_t b[6]; bool operator==(const MacKey& o) const { for(int i=0;i<6;i++) if(b[i]!=o.b[i]) return false; return true; } };
struct MacHash { size_t operator()(const MacKey& k) const { uint32_t h=(k.b[0]<<24)^(k.b[1]<<16)^(k.b[2]<<8)^k.b[3]^(k.b[4]<<4)^k.b[5]; return h; } };

static std::unordered_map<MacKey, uint16_t, MacHash> mac2dev;
static std::map<String, uint16_t> name2dev;

struct RoomDevKey { uint8_t room; uint16_t dev; bool operator==(const RoomDevKey& o) const { return room==o.room && dev==o.dev; } };
struct RoomDevHash { size_t operator()(const RoomDevKey& k) const { return ((size_t)k.room<<16) ^ (size_t)k.dev; } };
static std::unordered_map<RoomDevKey,int8_t,RoomDevHash> bestRssi;

static std::vector<String> seenNames; static size_t g_seen_max = 128;

struct SeenKey { uint8_t origin[6]; uint32_t batch; uint16_t seq; uint8_t src_room; };
static const size_t SEEN_CACHE_MAX = 256;
static std::vector<SeenKey> seenCache;

static uint8_t ESN_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// UART frame buffers (COBS)
static const size_t FRAME_MAX  = 2000;
static uint8_t frameBuf[FRAME_MAX]; static size_t frameLen=0;
static const size_t DECODE_MAX = 2000;
static uint8_t decodeBuf[DECODE_MAX];

// Reassembly
struct ReasmBuf {
  uint8_t  src_room; uint8_t  origin[6]; uint32_t batch; uint16_t seq; uint8_t total;
  std::vector<bool> have;
  std::vector<uint16_t> frag_len;
  std::vector<uint8_t> data;
  uint32_t last_ms;
};
static std::vector<ReasmBuf> reasmList;

// ======================= UTILS ========================
static void macToStr(const uint8_t* m, char out[18]) { sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); }
static bool parseMac(const String& s, uint8_t out[6]){ if(s.length()!=17) return false; int v[6]; if(sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5])!=6) return false; for(int i=0;i<6;i++) out[i]=(uint8_t)v[i]; return true; }
static String trimQuotes(String s){ s.trim(); if(s.length()>=2 && ((s.startsWith("\"")&&s.endsWith("\""))||(s.startsWith("'")&&s.endsWith("'")))) return s.substring(1,s.length()-1); return s; }
static String sanitizePrefix(const String& in){ String out; out.reserve(in.length()); for(size_t i=0;i<in.length();++i){ uint8_t c=(uint8_t)in[i]; if(c<32||c>126) break; out+=(char)c; } out.trim(); String collapsed; collapsed.reserve(out.length()); bool lastSpace=false; for(size_t i=0;i<out.length(); ++i){ char c=out[i]; if(c==' '||c=='\t'){ if(!lastSpace){ collapsed+=' '; lastSpace=true; } } else { collapsed+=c; lastSpace=false; } } return collapsed; }
static String sanitizeForCsv(const String& s){ String o=s; o.replace(","," "); o.replace("\r"," "); o.replace("\n"," "); return o; }
static uint16_t crc16_ccitt(const uint8_t* d,size_t n,uint16_t crc=0xFFFF){ while(n--){ crc^=(uint16_t)(*d++)<<8; for(int i=0;i<8;i++) crc=(crc&0x8000)?((crc<<1)^0x1021):(crc<<1);} return crc; }
static size_t cobs_decode(const uint8_t* in,size_t len,uint8_t* out,size_t outMax){ if(!len) return 0; const uint8_t* ip=in; const uint8_t* end=in+len; uint8_t* op=out; while(ip<end){ uint8_t code=*ip++; if(code==0 || ip+(code-1)>end) return 0; for(uint8_t i=1;i<code;i++){ if((size_t)(op-out)>=outMax) return 0; *op++=*ip++; } if(code!=0xFF && ip<end){ if((size_t)(op-out)>=outMax) return 0; *op++=0; } } return (size_t)(op-out); }
static void dumpHex(const char* tag, const uint8_t* p, size_t n){
  if (!DBG_HEX) return;
  Serial.printf("%s len=%u\n", tag, (unsigned)n);
  const size_t cols=16;
  for (size_t i=0;i<n;i+=cols){
    Serial.printf("  %04u: ", (unsigned)i);
    for (size_t j=0;j<cols && i+j<n; ++j) Serial.printf("%02X ", p[i+j]);
    Serial.print(" | ");
    for (size_t j=0;j<cols && i+j<n; ++j){ char c=(char)p[i+j]; Serial.print((c>=32&&c<=126)?c:'.'); }
    Serial.println();
  }
}

// =================== PERSISTENCE (SPIFFS) ===================
static void saveRoom(){ File f=SPIFFS.open("/room.txt", FILE_WRITE, true); if(f){ f.printf("%u\n",(unsigned)g_room); f.close(); } }
static void loadRoom(){ File f=SPIFFS.open("/room.txt", FILE_READ); if(f){ g_room=(uint8_t)f.readStringUntil('\n').toInt(); f.close(); if(!g_room) g_room=1; } }
static void saveTail(){ File f=SPIFFS.open("/tail.txt", FILE_WRITE, true); if(f){ f.printf("%d\n", g_is_tail?1:0); f.close(); } }
static void loadTail(){ File f=SPIFFS.open("/tail.txt", FILE_READ); if(f){ g_is_tail=(f.readStringUntil('\n').toInt()!=0); f.close(); } }
static void saveEsn(){ File f=SPIFFS.open("/esn.txt", FILE_WRITE, true); if(f){ f.printf("%d,%u\n", g_esn_on?1:0, (unsigned)g_channel); f.close(); } }
static void loadEsn(){ File f=SPIFFS.open("/esn.txt", FILE_READ); if(f){ String line=f.readStringUntil('\n'); f.close(); int comma=line.indexOf(','); if(comma>0){ g_esn_on=line.substring(0,comma).toInt()!=0; uint16_t ch=line.substring(comma+1).toInt(); if(ch>=1&&ch<=13) g_channel=(uint8_t)ch; } } }
static void saveBinds(){ File f=SPIFFS.open("/binds.csv", FILE_WRITE, true); if(!f) return; for(auto& kv: mac2dev){ char mac[18]; macToStr(kv.first.b,mac); f.printf("%s,%u\n",mac,(unsigned)kv.second);} f.close(); }
static void loadBinds(){ mac2dev.clear(); File f=SPIFFS.open("/binds.csv", FILE_READ); if(!f) return; while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue; int c=line.indexOf(','); if(c<0) continue; String smac=line.substring(0,c), sn=line.substring(c+1); uint8_t m[6]; if(!parseMac(smac,m)) continue; MacKey k; memcpy(k.b,m,6); mac2dev[k]=(uint16_t)sn.toInt(); } f.close(); }
static void saveNameBinds(){ File f=SPIFFS.open("/namebinds.csv", FILE_WRITE, true); if(!f) return; for(auto& kv:name2dev) f.printf("%s,%u\n", sanitizeForCsv(kv.first).c_str(), (unsigned)kv.second); f.close(); }
static void loadNameBinds(){ name2dev.clear(); File f=SPIFFS.open("/namebinds.csv", FILE_READ); if(!f) return; while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue; int c=line.lastIndexOf(','); if(c<0) continue; String nm=line.substring(0,c); uint16_t id=(uint16_t)line.substring(c+1).toInt(); name2dev[nm]=id; } f.close(); }

// =================== USB COMMANDS (unchanged except dbg) ===================
static void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  setroom <n>  | tail on|off | esn on|off | channel <1..13>"));
  Serial.println(F("  bind <MAC> <num> | bindname <name...> <num> | seen | bindseen <idx> <num>"));
  Serial.println(F("  show room|tail|binds|namebinds|esn|channel | save"));
  Serial.println(F("  dbg hex on|off | dbg parse on|off | dbg names on|off"));
  Serial.println(F("  help"));
}

static void handleCommand(const String& s){
  int sp=s.indexOf(' '); String cmd=(sp<0)?s:s.substring(0,sp); String rest=(sp<0)?"":s.substring(sp+1); cmd.toLowerCase();
  if(cmd=="help"||cmd=="?"){ printHelp(); }
  else if(cmd=="setroom"){ uint16_t n=(uint16_t)rest.toInt(); if(n<1||n>255){ Serial.println(F("ERR: room 1..255")); return; } g_room=(uint8_t)n; saveRoom(); Serial.printf("OK room=%u\n", g_room); }
  else if(cmd=="tail"){ String r=rest; r.toLowerCase(); if(r=="on"||r=="1"||r=="true") g_is_tail=true; else if(r=="off"||r=="0"||r=="false") g_is_tail=false; else { Serial.println(F("ERR: tail on|off")); return; } saveTail(); Serial.printf("OK tail=%d\n",(int)g_is_tail); }
  else if(cmd=="esn"){ String r=rest; r.toLowerCase(); if(r=="on"||r=="1"||r=="true") g_esn_on=true; else if(r=="off"||r=="0"||r=="false") g_esn_on=false; else { Serial.println(F("ERR: esn on|off")); return; } saveEsn(); Serial.printf("OK esn=%d\n",(int)g_esn_on); }
  else if(cmd=="channel"){ uint16_t ch=(uint16_t)rest.toInt(); if(ch<1||ch>13){ Serial.println(F("ERR: channel 1..13")); return; } g_channel=(uint8_t)ch; saveEsn(); Serial.printf("OK channel=%u\n",(unsigned)g_channel); }
  else if(cmd=="bind"){ int sp2=rest.indexOf(' '); if(sp2<0){ Serial.println(F("ERR: bind <MAC> <num>")); return; } String smac=rest.substring(0,sp2); String sn=rest.substring(sp2+1); uint8_t m[6]; if(!parseMac(smac,m)){ Serial.println(F("ERR: bad MAC")); return; } MacKey k; memcpy(k.b,m,6); mac2dev[k]=(uint16_t)sn.toInt(); Serial.printf("OK bind %s -> %u\n", smac.c_str(), (unsigned)mac2dev[k]); }
  else if(cmd=="bindname"){ int spLast=rest.lastIndexOf(' '); if(spLast<0){ Serial.println(F("ERR: bindname <name...> <num>")); return; } String raw=trimQuotes(rest.substring(0,spLast)); uint16_t id=(uint16_t)rest.substring(spLast+1).toInt(); if(!id){ Serial.println(F("ERR: invalid number")); return; } String san=sanitizePrefix(raw); if(!san.length()){ Serial.println(F("ERR: sanitized name empty")); return; } name2dev[san]=id; Serial.printf("OK bindname RAW=\"%s\" SAN=\"%s\" -> %u\n", raw.c_str(), san.c_str(), (unsigned)id); }
  else if(cmd=="seen"){ for(size_t i=0;i<seenNames.size();++i) Serial.printf("[%u] %s\n",(unsigned)i, seenNames[i].c_str()); }
  else if(cmd=="bindseen"){ int sp2=rest.indexOf(' '); if(sp2<0){ Serial.println(F("ERR: bindseen <idx> <num>")); return; } int idx=rest.substring(0,sp2).toInt(); uint16_t id=(uint16_t)rest.substring(sp2+1).toInt(); if(idx<0||(size_t)idx>=seenNames.size()){ Serial.println(F("ERR: idx out of range")); return; } String san=seenNames[(size_t)idx]; name2dev[san]=id; Serial.printf("OK bindseen [%d] \"%s\" -> %u\n", idx, san.c_str(), (unsigned)id); }
  else if(cmd=="show"){ String r=rest; r.toLowerCase(); if(r=="room") Serial.printf("room=%u\n", g_room); else if(r=="tail") Serial.printf("tail=%d\n",(int)g_is_tail); else if(r=="esn") Serial.printf("esn=%d\n",(int)g_esn_on); else if(r=="channel") Serial.printf("channel=%u\n",(unsigned)g_channel); else if(r=="binds"){ for(auto& kv:mac2dev){ char mac[18]; macToStr(kv.first.b,mac); Serial.printf("%s -> %u\n", mac, (unsigned)kv.second);} } else if(r=="namebinds"){ for(auto& kv:name2dev){ Serial.printf("\"%s\" -> %u\n", kv.first.c_str(), (unsigned)kv.second);} } else Serial.println(F("ERR: show room|tail|binds|namebinds|esn|channel")); }
  else if(cmd=="dbg"){ int sp2=rest.indexOf(' '); String what=sp2<0?rest:rest.substring(0,sp2); String val=sp2<0?"":rest.substring(sp2+1); what.trim(); what.toLowerCase(); val.trim(); val.toLowerCase(); bool on=(val=="on"||val=="1"||val=="true"); if(what=="hex"){ DBG_HEX=on; Serial.printf("OK dbg hex=%d\n",(int)DBG_HEX);} else if(what=="parse"){ DBG_PARSE=on; Serial.printf("OK dbg parse=%d\n",(int)DBG_PARSE);} else if(what=="names"){ DBG_NAMES=on; Serial.printf("OK dbg names=%d\n",(int)DBG_NAMES);} else Serial.println(F("ERR: dbg hex|parse|names on|off")); }
  else if(cmd=="save"){ saveBinds(); saveNameBinds(); saveEsn(); Serial.println(F("OK saved")); }
  else if(cmd.length()){ Serial.println(F("ERR: unknown. Type 'help'.")); }
}

static void pollUsb(){
  while(Serial.available()){
    static String line; char c=(char)Serial.read();
    if(c=='\r'||c=='\n'){ line.trim(); if(line.length()) handleCommand(line); line=""; }
    else line += c;
  }
}

// forward declaration
void processDecodedMessage(const uint8_t* m, size_t n, uint8_t src_room, bool from_esn);

// =================== ESPNOW LAYER ===================
static bool esnInited=false;

static void esn_teardown(){ if(!esnInited) return; esp_now_deinit(); WiFi.mode(WIFI_OFF); esnInited=false; }

static bool esn_setup(){
  if(!g_esn_on){ esn_teardown(); return false; }
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if(esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); return false; }
  esnInited=true;

  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, ESN_BROADCAST, 6); peer.channel=g_channel; peer.encrypt=false; esp_now_add_peer(&peer);

  // ---------- RECV ----------
  esp_now_register_recv_cb([](const esp_now_recv_info_t* info, const uint8_t* data, int len){
    (void)info;
    if(len<(int)sizeof(EsnFrag)) return;
    const EsnFrag* h=(const EsnFrag*)data;
    if(h->magic0!='S'||h->magic1!='M'||h->ver!=1) return;
    if((int)sizeof(EsnFrag)+(int)h->frag_len>len) return;
    if(!(h->src_room==g_room||h->src_room==(uint8_t)(g_room-1))) return;

    uint32_t now=millis();
    int slot=-1;
    for(size_t i=0;i<reasmList.size();++i){
      ReasmBuf& rb=reasmList[i];
      if(rb.src_room==h->src_room && rb.batch==h->batch && rb.seq==h->seq && memcmp(rb.origin,h->origin,6)==0){ slot=(int)i; break; }
      if(now-rb.last_ms>3000){ reasmList.erase(reasmList.begin()+i); --i; }
    }
    if(slot<0){
      ReasmBuf rb{};
      rb.src_room=h->src_room; memcpy(rb.origin,h->origin,6);
      rb.batch=h->batch; rb.seq=h->seq; rb.total=h->total;
      rb.have.assign(h->total,false);
      rb.frag_len.assign(h->total,0);
      rb.data.assign((size_t)h->total*(size_t)ESN_FRAG_DATA_MAX,0);
      rb.last_ms=now;
      reasmList.push_back(rb);
      slot=(int)reasmList.size()-1;
    }
    ReasmBuf& rb=reasmList[slot];
    if(h->index>=rb.total) return;
    if(!rb.have[h->index]){
      rb.have[h->index]=true;
      rb.frag_len[h->index]=h->frag_len;
      size_t off=(size_t)h->index*(size_t)ESN_FRAG_DATA_MAX;
      memcpy(&rb.data[off], h->data, h->frag_len);
      rb.last_ms=now;
    }
    bool complete=true; for(bool b:rb.have) if(!b){ complete=false; break; }
    if(complete){
      size_t total_len=0; for(uint8_t i=0;i<rb.total;i++) total_len += rb.frag_len[i];
      std::vector<uint8_t> msg; msg.reserve(total_len);
      for(uint8_t i=0;i<rb.total;i++){ size_t off=(size_t)i*(size_t)ESN_FRAG_DATA_MAX; msg.insert(msg.end(), rb.data.begin()+off, rb.data.begin()+off+rb.frag_len[i]); }

      dumpHex("[ESN-RX RAW] reassembled", msg.data(), msg.size());

      SeenKey sk{}; memcpy(sk.origin, rb.origin, 6); sk.batch=rb.batch; sk.seq=rb.seq; sk.src_room=rb.src_room;
      bool duplicate=false; for(auto& k:seenCache){ if(k.src_room==sk.src_room && k.batch==sk.batch && k.seq==sk.seq && memcmp(k.origin,sk.origin,6)==0){ duplicate=true; break; } }
      if(!duplicate){ if(seenCache.size()>=SEEN_CACHE_MAX) seenCache.erase(seenCache.begin()); seenCache.push_back(sk); processDecodedMessage(msg.data(), msg.size(), rb.src_room, true); }
      reasmList.erase(reasmList.begin()+slot);
    }
  });

  // ---------- SEND (IDF-conditional signature) ----------
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
  esp_now_register_send_cb([](const wifi_tx_info_t* /*info*/, esp_now_send_status_t status){
    if(DBG_PARSE) Serial.printf("[ESN-SEND] status=%s\n", status==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
  });
#else
  esp_now_register_send_cb([](const uint8_t* /*mac*/, esp_now_send_status_t status){
    if(DBG_PARSE) Serial.printf("[ESN-SEND] status=%s\n", status==ESP_NOW_SEND_SUCCESS?"OK":"FAIL");
  });
#endif

  return true;
}

static void esn_send_frag(const EsnFrag* hdr, size_t bytes){ if(!esnInited) return; esp_now_send(ESN_BROADCAST, (const uint8_t*)hdr, bytes); }

static void forward_upstream(uint8_t src_room, const uint8_t* decoded, size_t n, const uint8_t* origin, uint32_t batch, uint16_t seq){
  if(!g_esn_on || g_is_tail) return;
  if(!(src_room==g_room || src_room==(uint8_t)(g_room-1))) return;

  dumpHex("[ESN-TX RAW] about to send", decoded, n);

  uint8_t total=(uint8_t)((n+(size_t)ESN_FRAG_DATA_MAX-1)/(size_t)ESN_FRAG_DATA_MAX); if(total==0) total=1;
  for(uint8_t idx=0; idx<total; ++idx){
    size_t off=(size_t)idx*(size_t)ESN_FRAG_DATA_MAX; size_t left=n-off;
    size_t take=(left<(size_t)ESN_FRAG_DATA_MAX)?left:(size_t)ESN_FRAG_DATA_MAX;
    uint8_t buf[sizeof(EsnFrag)+ESN_FRAG_DATA_MAX]; EsnFrag* h=(EsnFrag*)buf;
    h->magic0='S'; h->magic1='M'; h->ver=1; h->src_room=src_room; h->total=total; h->index=idx;
    memcpy(h->origin,origin,6); h->batch=batch; h->seq=seq; h->frag_len=(uint16_t)take;
    memcpy(h->data, decoded+off, take);
    esn_send_frag(h, sizeof(EsnFrag)+take);
  }
}

// =================== PARSER / PIPELINE ===================
static void addSeen(const String& san){ if(!san.length()) return; for(size_t i=0;i<seenNames.size();++i){ if(seenNames[i]==san){ if(i!=0){ seenNames.erase(seenNames.begin()+i); seenNames.insert(seenNames.begin(), san);} return; } } seenNames.insert(seenNames.begin(), san); if(seenNames.size()>g_seen_max) seenNames.pop_back(); }

static void endOfScanFlush(uint8_t flush_room){
  std::vector<RoomDevKey> toErase;
  for(auto &kv:bestRssi){
    if(kv.first.room==flush_room){
      Serial.printf("%u.%u@%d\n",(unsigned)kv.first.room,(unsigned)kv.first.dev,(int)kv.second);
      toErase.push_back(kv.first);
    }
  }
  for(auto &k:toErase) bestRssi.erase(k);
  Serial.println("--- END OF SCAN ---");
}

static void parseAndMaybeRecord(const uint8_t* m, size_t n, uint8_t src_room){
  uint16_t count=(uint16_t)m[14] | ((uint16_t)m[15]<<8);
  size_t p=1+1+6+4+2+2, end=n-2;
  if(DBG_PARSE) Serial.printf("[PARSE] typ=%u src_room=%u count=%u\n",(unsigned)m[1],(unsigned)src_room,(unsigned)count);

  for(uint16_t i=0; i<count && p+2<=end; ++i){
    uint8_t T=m[p++], L=m[p++]; if(!(T==1 && L==6) || p+6>end) break;
    MacKey mk; memcpy(mk.b,&m[p],6); p+=6;
    if(p+2>end) break; T=m[p++]; L=m[p++]; if(!(T==2 && L==1) || p+1>end) break; int8_t rssi=(int8_t)m[p++];
    if(p+2>end) break; T=m[p++]; L=m[p++]; if(!(T==3 && L==1) || p+1>end) break; p++;
    String nameRaw, nameSan;
    if(p+2<=end && m[p]==4){ T=m[p++]; L=m[p++]; if(p+L<=end){ nameRaw=String((const char*)&m[p], L); p+=L; } }
    if(nameRaw.length()){ nameSan=sanitizePrefix(nameRaw); addSeen(nameSan); }
    uint16_t devNum=0xFFFF; if(nameSan.length()){ auto itn=name2dev.find(nameSan); if(itn!=name2dev.end()) devNum=itn->second; }
    if(devNum==0xFFFF){ auto itm=mac2dev.find(mk); if(itm!=mac2dev.end()) devNum=itm->second; }
    char macStr[18]; macToStr(mk.b,macStr);
    if(DBG_PARSE && (!DBG_NAMES || nameRaw.length())){
      Serial.printf("[REC] room=%u mac=%s rssi=%d RAW=\"%s\" SAN=\"%s\" -> %s\n",
        (unsigned)src_room, macStr, (int)rssi, nameRaw.c_str(), nameSan.c_str(),
        (devNum!=0xFFFF? String("dev "+String(devNum)).c_str() : "(no bind)"));
    }
    if(devNum!=0xFFFF){ RoomDevKey k{src_room,devNum}; auto it=bestRssi.find(k); if(it==bestRssi.end() || rssi>it->second) bestRssi[k]=rssi; }
  }
}

void processDecodedMessage(const uint8_t* m, size_t n, uint8_t src_room, bool /*from_esn*/){
  if(n<1+1+6+4+2+2+2) return;
  uint16_t given=(uint16_t)m[n-2] | ((uint16_t)m[n-1]<<8);
  uint16_t calc =crc16_ccitt(m, n-2);
  if(given!=calc){ if(DBG_PARSE) Serial.println("[PARSE] CRC FAIL"); return; }

  uint8_t typ=m[1];
  const uint8_t* origin=&m[2];
  uint32_t batch=(uint32_t)m[8] | ((uint32_t)m[9]<<8) | ((uint32_t)m[10]<<16) | ((uint32_t)m[11]<<24);
  uint16_t seq=(uint16_t)m[12] | ((uint16_t)m[13]<<8);

  SeenKey sk{}; memcpy(sk.origin,origin,6); sk.batch=batch; sk.seq=seq; sk.src_room=src_room;
  for(auto &k:seenCache){ if(k.src_room==sk.src_room && k.batch==sk.batch && k.seq==sk.seq && memcmp(k.origin,sk.origin,6)==0) goto maybe_forward; }
  if(seenCache.size()>=SEEN_CACHE_MAX) seenCache.erase(seenCache.begin());
  seenCache.push_back(sk);

  if(typ==MT_END){ endOfScanFlush(src_room); }
  else { parseAndMaybeRecord(m,n,src_room); }

maybe_forward:
  // Forward only on non-tail
  if (g_esn_on && !g_is_tail &&
      (src_room == g_room || src_room == (uint8_t)(g_room-1))) {
    forward_upstream(src_room, m, n, origin, batch, seq);
  }
}

// =================== UART1 (ICE) ===================
static void pollICE(){
  while(Serial1.available()){
    uint8_t b = (uint8_t)Serial1.read();
    if (b == 0x00){
      if (frameLen > 0){
        size_t dec = cobs_decode(frameBuf, frameLen, decodeBuf, DECODE_MAX);
        if (dec > 0) {
          // Always show what came from ICE so we can compare with ESN-TX RAW.
          dumpHex("[ICE RX] decoded", decodeBuf, dec);
          processDecodedMessage(decodeBuf, dec, g_room, false);
        }
        frameLen = 0;
      }
    } else {
      // NOTE: previous snippet accidentally had a ternary here; this is the correct line:
      if (frameLen < FRAME_MAX) frameBuf[frameLen++] = b; else frameLen = 0;
    }
  }
}

// =================== SETUP / LOOP ===================
void setup(){
  Serial.begin(115200);
  delay(50);

  if (!SPIFFS.begin(true)) Serial.println("SPIFFS mount failed");
  loadRoom(); loadTail(); loadEsn(); loadBinds(); loadNameBinds();

  Serial.printf("SMOKE ready. room=%u tail=%d esn=%d ch=%u\n",
                (unsigned)g_room, (int)g_is_tail, (int)g_esn_on, (unsigned)g_channel);
  printHelp();

  Serial1.setRxBufferSize(ICE_UART_RXBUF);
  Serial1.setTxBufferSize(ICE_UART_TXBUF);
  Serial1.begin(UART_BAUD_ICE, SERIAL_8N1, ICE_UART_RX, ICE_UART_TX);

  if (g_esn_on) esn_setup();
}

void loop(){
  pollICE();
  pollUsb();

  // Re-init ESPNOW if config changed at runtime
  static bool last_esn = false;
  static uint8_t last_ch = 0;
  if (last_esn != g_esn_on || last_ch != g_channel){
    last_esn = g_esn_on;
    last_ch  = g_channel;
    esn_teardown();
    esn_setup();
  }
}
