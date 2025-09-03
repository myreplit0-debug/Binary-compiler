/*
  SMOKE Unified (ESP32-S3) — Receiver/Bridge v1.1
  - Name-based binding with ASCII sanitization
  - Matching order: sanitized NAME bind > MAC bind
  - Verbose RX shows RAW and SAN names for clarity
*/

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <map>
#include <unordered_map>

// ====== CONFIG ======
#define VERBOSE_RX 1
#define ICE_UART_BAUD  921600
#define ICE_UART_RX    16
#define ICE_UART_TX    17
#define ICE_UART_RXBUF 16384
#define ICE_UART_TXBUF 2048

enum MsgType : uint8_t { MT_DATA=0, MT_END=1 };

// ====== STATE / PERSISTED CONFIG ======
static uint8_t  g_room = 1;
static bool     g_is_tail = true;

struct MacKey { uint8_t b[6]; bool operator==(const MacKey& o) const { for(int i=0;i<6;i++) if(b[i]!=o.b[i]) return false; return true; } };
struct MacHash { size_t operator()(const MacKey& k) const { uint32_t h=(k.b[0]<<24)^(k.b[1]<<16)^(k.b[2]<<8)^k.b[3]^(k.b[4]<<4)^k.b[5]; return h; } };

// MAC -> device#
static std::unordered_map<MacKey, uint16_t, MacHash> mac2dev;
// device# -> friendly label (optional)
static std::map<uint16_t, String> devNames;
// *** Sanitized Name -> device#  (ordered map; Arduino String has no std::hash) ***
static std::map<String, uint16_t> name2dev;

// Per-scan strongest RSSI per device
static std::unordered_map<uint16_t, int8_t> bestRssi;

// ====== UTILS ======
static void macToStr(const uint8_t* m, char out[18]) { sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); }
static bool parseMac(const String& s, uint8_t out[6]) {
  if (s.length()!=17) return false; int v[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x",&v[0],&v[1],&v[2],&v[3],&v[4],&v[5])!=6) return false;
  for(int i=0;i<6;i++) out[i]=(uint8_t)v[i]; return true;
}
static String trimQuotes(String s){ s.trim(); if(s.length()>=2 && ((s.startsWith("\"")&&s.endsWith("\""))||(s.startsWith("'")&&s.endsWith("'")))) return s.substring(1,s.length()-1); return s; }

// Sanitize to printable ASCII (32..126). Replace others with '.', collapse multiple spaces, trim ends.
static String sanitizeAscii(const String& in){
  String out; out.reserve(in.length());
  bool lastSpace=false;
  for (size_t i=0;i<in.length();++i){
    uint8_t c=(uint8_t)in[i];
    if (c>=32 && c<=126){
      if (c==' '){ if(!lastSpace) out += ' '; lastSpace=true; }
      else { out += (char)c; lastSpace=false; }
    } else {
      if(!lastSpace){ out += '.'; lastSpace=true; }
    }
  }
  // trim leading/trailing spaces or dots
  while(out.length() && (out[0]==' '||out[0]=='.')) out.remove(0,1);
  while(out.length() && (out[out.length()-1]==' '||out[out.length()-1]=='.')) out.remove(out.length()-1);
  return out;
}

static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF){
  while(len--){ crc ^= (uint16_t)(*data++)<<8; for (int i=0;i<8;i++) crc = (crc&0x8000)?((crc<<1)^0x1021):(crc<<1); }
  return crc;
}
static size_t cobs_decode(const uint8_t* in,size_t len,uint8_t* out,size_t outMax){
  if(!len) return 0; const uint8_t* ip=in; const uint8_t* end=in+len; uint8_t* op=out;
  while(ip<end){ uint8_t code=*ip++; if(code==0 || ip+(code-1)>end) return 0;
    for(uint8_t i=1;i<code;i++){ if((size_t)(op-out)>=outMax) return 0; *op++=*ip++; }
    if(code!=0xFF && ip<end){ if((size_t)(op-out)>=outMax) return 0; *op++=0; }
  } return (size_t)(op-out);
}

// ====== SPIFFS I/O ======
static void saveRoom(){ File f=SPIFFS.open("/room.txt", FILE_WRITE, true); if(f){ f.printf("%u\n",(unsigned)g_room); f.close(); } }
static void loadRoom(){ File f=SPIFFS.open("/room.txt", FILE_READ); if(f){ g_room=(uint8_t)f.readStringUntil('\n').toInt(); f.close(); if(!g_room) g_room=1; } }
static void saveTail(){ File f=SPIFFS.open("/tail.txt", FILE_WRITE, true); if(f){ f.printf("%d\n", g_is_tail?1:0); f.close(); } }
static void loadTail(){ File f=SPIFFS.open("/tail.txt", FILE_READ); if(f){ g_is_tail=(f.readStringUntil('\n').toInt()!=0); f.close(); } }

static void saveBinds(){ File f=SPIFFS.open("/binds.csv", FILE_WRITE, true); if(!f) return;
  for(auto& kv: mac2dev){ char mac[18]; macToStr(kv.first.b,mac); f.printf("%s,%u\n",mac,(unsigned)kv.second);} f.close();}
static void loadBinds(){ mac2dev.clear(); File f=SPIFFS.open("/binds.csv", FILE_READ); if(!f) return;
  while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.indexOf(','); if(c<0) continue; String smac=line.substring(0,c), sn=line.substring(c+1);
    uint8_t m[6]; if(!parseMac(smac,m)) continue; MacKey k; memcpy(k.b,m,6); mac2dev[k]=(uint16_t)sn.toInt(); }
  f.close(); }

static String sanitizeForCsv(const String& s){ String o=s; o.replace(","," "); o.replace("\r"," "); o.replace("\n"," "); return o; }
static void saveNames(){ File f=SPIFFS.open("/names.csv", FILE_WRITE, true); if(!f) return;
  for(auto& kv: devNames) f.printf("%u,%s\n",(unsigned)kv.first,sanitizeForCsv(kv.second).c_str()); f.close();}
static void loadNames(){ devNames.clear(); File f=SPIFFS.open("/names.csv", FILE_READ); if(!f) return;
  while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.indexOf(','); if(c<0) continue; uint16_t id=(uint16_t)line.substring(0,c).toInt(); String nm=line.substring(c+1); devNames[id]=nm; }
  f.close(); }

static void saveNameBinds(){ File f=SPIFFS.open("/namebinds.csv", FILE_WRITE, true); if(!f) return;
  for(auto& kv: name2dev) f.printf("%s,%u\n", sanitizeForCsv(kv.first).c_str(), (unsigned)kv.second); f.close(); }
static void loadNameBinds(){ name2dev.clear(); File f=SPIFFS.open("/namebinds.csv", FILE_READ); if(!f) return;
  while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.lastIndexOf(','); if(c<0) continue; String nm=line.substring(0,c); uint16_t id=(uint16_t)line.substring(c+1).toInt(); name2dev[nm]=id; }
  f.close(); }

// ====== USB COMMANDS ======
static String inLine;
static void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  setroom <n>                 - set this receiver's room number (1..255)"));
  Serial.println(F("  tail on|off                 - mark this receiver as tail (USB output)"));
  Serial.println(F("  bind <MAC> <num>            - map BLE MAC to device number"));
  Serial.println(F("  bindname <name...> <num>    - map Device Name (sanitized) to device number"));
  Serial.println(F("  name <num> <text...>        - set friendly label"));
  Serial.println(F("  show room|tail|binds|names|namebinds"));
  Serial.println(F("  save                        - persist binds & names"));
  Serial.println(F("Note: names are sanitized to printable ASCII before matching."));
}
static void handleCommand(const String& s){
  int sp=s.indexOf(' '); String cmd=(sp<0)?s:s.substring(0,sp); String rest=(sp<0)?"":s.substring(sp+1); cmd.toLowerCase();

  if(cmd=="help"||cmd=="?"){ printHelp(); }
  else if(cmd=="setroom"){ uint16_t n=(uint16_t)rest.toInt(); if(n<1||n>255){ Serial.println(F("ERR: room 1..255")); return; } g_room=(uint8_t)n; saveRoom(); Serial.printf("OK room=%u\n", g_room); }
  else if(cmd=="tail"){ String r=rest; r.toLowerCase(); if(r=="on"||r=="1"||r=="true") g_is_tail=true; else if(r=="off"||r=="0"||r=="false") g_is_tail=false; else { Serial.println(F("ERR: tail on|off")); return; } saveTail(); Serial.printf("OK tail=%d\n",(int)g_is_tail); }
  else if(cmd=="bind"){
    int sp2=rest.indexOf(' '); if(sp2<0){ Serial.println(F("ERR: bind <MAC> <num>")); return; }
    String smac=rest.substring(0,sp2); String sn=rest.substring(sp2+1);
    uint8_t m[6]; if(!parseMac(smac,m)){ Serial.println(F("ERR: bad MAC")); return; }
    MacKey k; memcpy(k.b,m,6); mac2dev[k]=(uint16_t)sn.toInt();
    Serial.printf("OK bind %s -> %u\n", smac.c_str(), (unsigned)mac2dev[k]);
  }
  else if(cmd=="bindname"){
    int spLast=rest.lastIndexOf(' '); if(spLast<0){ Serial.println(F("ERR: bindname <name...> <num>")); return; }
    String raw = trimQuotes(rest.substring(0, spLast));
    uint16_t id = (uint16_t)rest.substring(spLast+1).toInt(); if(!id){ Serial.println(F("ERR: invalid number")); return; }
    String san = sanitizeAscii(raw);
    name2dev[san] = id;
    Serial.printf("OK bindname RAW=\"%s\" SAN=\"%s\" -> %u\n", raw.c_str(), san.c_str(), (unsigned)id);
  }
  else if(cmd=="name"){
    int sp2=rest.indexOf(' '); if(sp2<0){ Serial.println(F("ERR: name <num> <text...>")); return; }
    uint16_t id=(uint16_t)rest.substring(0,sp2).toInt(); String nm=rest.substring(sp2+1);
    devNames[id]=nm; Serial.printf("OK name %u=\"%s\"\n",(unsigned)id,nm.c_str());
  }
  else if(cmd=="save"){ saveBinds(); saveNames(); saveNameBinds(); Serial.println(F("OK saved")); }
  else if(cmd=="show"){
    String r=rest; r.toLowerCase();
    if(r=="room") Serial.printf("room=%u\n", g_room);
    else if(r=="tail") Serial.printf("tail=%d\n", (int)g_is_tail);
    else if(r=="binds"){ for(auto& kv: mac2dev){ char mac[18]; macToStr(kv.first.b,mac); Serial.printf("%s -> %u\n", mac, (unsigned)kv.second);} }
    else if(r=="names"){ for(auto& kv: devNames){ Serial.printf("%u,%s\n", (unsigned)kv.first, kv.second.c_str()); } }
    else if(r=="namebinds"){ for(auto& kv: name2dev){ Serial.printf("\"%s\" -> %u\n", kv.first.c_str(), (unsigned)kv.second); } }
    else Serial.println(F("ERR: show room|tail|binds|names|namebinds"));
  }
  else if(cmd.length()){ Serial.println(F("ERR: unknown. Type 'help'.")); }
}
static void pollUsb(){
  while(Serial.available()){
    static String line; char c=(char)Serial.read();
    if(c=='\r'||c=='\n'){ line.trim(); if(line.length()) handleCommand(line); line=""; }
    else line += c;
  }
}

// ====== UART1 / COBS ======
static const size_t FRAME_MAX  = 2000;
static uint8_t frameBuf[FRAME_MAX]; static size_t frameLen=0;
static const size_t DECODE_MAX = 2000;
static uint8_t decodeBuf[DECODE_MAX];

static size_t cobs_decode(const uint8_t*, size_t, uint8_t*, size_t); // fwd decl above
static void processDecodedMessage(const uint8_t* m, size_t n);

static void pollICE(){
  while(Serial1.available()){
    uint8_t b=(uint8_t)Serial1.read();
    if(b==0x00){
      if(frameLen>0){
        size_t dec=cobs_decode(frameBuf, frameLen, decodeBuf, DECODE_MAX);
        if(dec>0) processDecodedMessage(decodeBuf, dec);
        frameLen=0;
      }
    } else {
      if(frameLen<FRAME_MAX) frameBuf[frameLen++]=b; else frameLen=0;
    }
  }
}

// ====== PARSER ======
static void endOfScanFlush(){
  for(auto &kv: bestRssi){
    Serial.printf("%u.%u@%d\n", (unsigned)g_room, (unsigned)kv.first, (int)kv.second);
  }
  bestRssi.clear();
  Serial.println("--- END OF SCAN ---");
}

static void processDecodedMessage(const uint8_t* m, size_t n){
  if(n < 1+1+6+4+2+2+2) return;
  uint16_t given=(uint16_t)m[n-2]|((uint16_t)m[n-1]<<8);
  uint16_t calc =crc16_ccitt(m,n-2);
  if(given!=calc) return;

  uint8_t typ=m[1];
  if(typ==MT_END){ endOfScanFlush(); return; }

  uint16_t count=(uint16_t)m[14]|((uint16_t)m[15]<<8);
  size_t p=1+1+6+4+2+2, end=n-2;

  for(uint16_t i=0;i<count && p+2<=end;++i){
    // MAC
    if (p+2>end) break; uint8_t T=m[p++], L=m[p++]; if(!(T==1 && L==6) || p+6>end) break;
    MacKey mk; memcpy(mk.b,&m[p],6); p+=6;
    // RSSI
    if (p+2>end) break; T=m[p++]; L=m[p++]; if(!(T==2 && L==1) || p+1>end) break;
    int8_t rssi=(int8_t)m[p++];
    // FLAGS
    if (p+2>end) break; T=m[p++]; L=m[p++]; if(!(T==3 && L==1) || p+1>end) break; p++;
    // NAME
    String nameRaw; String nameSan;
    if (p+2<=end && m[p]==4){ T=m[p++]; L=m[p++]; if(p+L<=end){ nameRaw = String((const char*)&m[p], L); p+=L; } }
    if (nameRaw.length()) nameSan = sanitizeAscii(nameRaw);

    // choose device number: NAME (sanitized) then MAC
    uint16_t devNum = 0xFFFF;
    if (nameSan.length()){
      auto itn = name2dev.find(nameSan);
      if (itn!=name2dev.end()) devNum = itn->second;
    }
    if (devNum==0xFFFF){
      auto itm = mac2dev.find(mk);
      if (itm!=mac2dev.end()) devNum = itm->second;
    }

#if VERBOSE_RX
    {
      char macStr[18]; macToStr(mk.b,macStr);
      Serial.print("RX "); Serial.print(macStr);
      Serial.print(" RSSI="); Serial.print((int)rssi);
      if (nameRaw.length()){
        Serial.print(" NameRAW=\""); Serial.print(nameRaw); Serial.print("\"");
        if (nameSan != nameRaw){
          Serial.print(" SAN=\""); Serial.print(nameSan); Serial.print("\"");
        }
      }
      if (devNum!=0xFFFF){ Serial.print(" -> dev "); Serial.println(devNum); }
      else               { Serial.println(" -> (no bind)"); }
    }
#endif

    if (devNum!=0xFFFF){
      auto br=bestRssi.find(devNum);
      if (br==bestRssi.end() || rssi>br->second) bestRssi[devNum]=rssi;
    }
  }
}

// ====== SETUP / LOOP ======
void setup(){
  Serial.begin(115200); delay(50);
  if(!SPIFFS.begin(true)) Serial.println("SPIFFS mount failed");
  else { loadRoom(); loadTail(); loadBinds(); loadNames(); loadNameBinds(); }
  Serial.printf("SMOKE ready. room=%u tail=%d\n",(unsigned)g_room,(int)g_is_tail);
  printHelp();

  Serial1.setRxBufferSize(ICE_UART_RXBUF);
  Serial1.setTxBufferSize(ICE_UART_TXBUF);
  Serial1.begin(ICE_UART_BAUD, SERIAL_8N1, ICE_UART_RX, ICE_UART_TX);
}
void loop(){ pollICE(); pollUsb(); }
