// SMOKE chain node (ESP32-S3)
// - Filters to registered tags only (MAP from UI).
// - Aggregates local+downstream best per tag; forwards in TDMA slot.
// - REPORT tokens carry room that observed the best signal.
// - Improvements: dynamic active set, chunked REPORT frames, and amTail fallback.
//
// Build FQBN: esp32:esp32:esp32s3

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <unordered_map>
#include <vector>

#define ESPNOW_CH 6
#define MAX_TAG_ID 2048          // up to 2048 tag IDs
#define FRAME_MS   800           // TDMA frame
#define MIN_SLOT   30            // ms, lower bound
#define MAX_REPORT_BYTES 220     // safe ESP-NOW payload budget per REPORT chunk

// ICE UART1 pins on S3
#define ICE_RX1 16
#define ICE_TX1 17
HardwareSerial &SICE = Serial1;

// ---------- utils ----------
static uint8_t BCAST[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static bool addPeer(const uint8_t m[6]){ if(esp_now_is_peer_exist(m)) return true; esp_now_peer_info_t p={}; memcpy(p.peer_addr,m,6); p.channel=ESPNOW_CH; p.encrypt=false; return esp_now_add_peer(&p)==ESP_OK; }
static void sendNow(const uint8_t* mac, const String& s){ addPeer(mac); esp_now_send(mac,(const uint8_t*)s.c_str(), s.length()); }
static bool macIsZero(const uint8_t m[6]){ for(int i=0;i<6;i++) if(m[i]) return false; return true; }
static void macToStr(const uint8_t m[6], char out[18]){ sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); }
static bool strToMac(const String& s, uint8_t m[6]){ if(s.length()!=17) return false; int b[6]; if(sscanf(s.c_str(),"%2x:%2x:%2x:%2x:%2x:%2x",&b[0],&b[1],&b[2],&b[3],&b[4],&b[5])!=6) return false; for(int i=0;i<6;i++) m[i]=(uint8_t)b[i]; return true; }
static uint8_t rssiToStrength(int rssi){ if(rssi>-30) rssi=-30; if(rssi<-100) rssi=-100; int v=(int)((rssi+100)*(100.0/70.0)+0.5); if(v<0)v=0; if(v>100)v=100; return (uint8_t)v; }

// ---------- prefs ----------
Preferences prefs;
static uint8_t nextMac[6]={0};   // upstream hop toward tail
static uint8_t idx=1;            // chain slot index (1..N)
static uint8_t slots=8;          // total slots (from UI)
static uint8_t roomNo=0;         // 0..255
static bool    isTail=false;

static void loadPrefs(){
  prefs.begin("smoke", true);
  String n=prefs.getString("next",""); if(n.length()==17) strToMac(n,nextMac); else memset(nextMac,0,6);
  idx   = prefs.getUChar("idx",1);
  slots = prefs.getUChar("slots",8); if(!slots) slots=1;
  roomNo= prefs.getUChar("room",0);
  isTail= prefs.getBool("tail",false);
  prefs.end();
}
static void savePrefs(){
  char n[18]; macToStr(nextMac,n);
  prefs.begin("smoke", false);
  prefs.putString("next",n);
  prefs.putUChar("idx",idx);
  prefs.putUChar("slots",slots);
  prefs.putUChar("room",roomNo);
  prefs.putBool("tail",isTail);
  prefs.end();
}

// ---------- registry & aggregation ----------
static std::unordered_map<String,uint16_t> raw2id; // RAW name -> tag id

static uint8_t bestStr [MAX_TAG_ID+1]; // 0..100
static uint8_t bestRoom[MAX_TAG_ID+1]; // 0..255 (room that produced bestStr)

// Active IDs tracking (dynamic)
static bool inActive[MAX_TAG_ID+1];
static std::vector<uint16_t> activeIds;

static inline void touchActive(uint16_t id){
  if (!inActive[id]){ inActive[id]=true; activeIds.push_back(id); }
}
static inline void updateBest(uint16_t id, uint8_t str, uint8_t room){
  if (id==0 || id>MAX_TAG_ID || str==0) return;
  if (str >= bestStr[id]) { bestStr[id]=str; bestRoom[id]=room; touchActive(id); }
}
static void decayAndCompact(){
  if (activeIds.empty()) return;
  std::vector<uint16_t> keep; keep.reserve(activeIds.size());
  for (uint16_t id : activeIds){
    uint8_t s = bestStr[id];
    if (s) { s = (s>2)? (uint8_t)(s-2) : 0; bestStr[id]=s; }
    if (s){ keep.push_back(id); }
    else { inActive[id]=false; bestRoom[id]=0; }
  }
  activeIds.swap(keep);
}

// ---------- messaging ----------
static uint8_t uiMac[6]={0}; static bool uiKnown=false;

static void sendHello(){
  uint8_t me[6]; WiFi.macAddress(me); char my[18]; macToStr(me,my);
  char nxt[18]; macToStr(nextMac,nxt);
  String s="HELLO,mac="; s+=my;
  s += ",idx="+String(idx)+",room="+String(roomNo)+",tail="+String(isTail?1:0)+",next="+String(nxt);
  sendNow(BCAST,s); if(uiKnown) sendNow(uiMac,s);
}

// Chunked REPORT with room.tag@str tokens
static void sendReport(){
  if (activeIds.empty()) return;

  auto flushChunk = [&](String& out){
    if (out.length()<=7) return; // "REPORT:" only
    bool amTail = isTail || macIsZero(nextMac); // **amTail improvement**
    if (amTail){ if (uiKnown) sendNow(uiMac,out); }
    else if (!macIsZero(nextMac)){ sendNow(nextMac,out); }
    else { sendNow(BCAST,out); }
    out = "REPORT:";
  };

  String out="REPORT:"; bool first=true;
  for (uint16_t id: activeIds){
    uint8_t s = bestStr[id]; if (!s) continue;
    uint8_t r = bestRoom[id] ? bestRoom[id] : roomNo;
    String tok = String((int)r)+"."+String((int)id)+'@'+String((int)s);
    int add = (first?0:1) + tok.length();
    if (out.length() + add > MAX_REPORT_BYTES) { flushChunk(out); first=true; }
    if (!first) out+=','; first=false; out+=tok;
  }
  flushChunk(out);
}

static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  String msg; msg.reserve(len+1); for (int i=0;i<len;i++) msg+=(char)data[i];
  const uint8_t* src=info->src_addr;

  if (msg=="HELLO?" || msg=="PING?"){ memcpy(uiMac,src,6); uiKnown=true; sendNow(src,"INFO:ok"); return; }

  if (msg.startsWith("CFG:")){
    int p=4; while(p<(int)msg.length()){
      int c=msg.indexOf(',',p); if(c<0) c=msg.length();
      String kv=msg.substring(p,c); kv.trim();
      int e=kv.indexOf('='); String k=(e>0?kv.substring(0,e):kv), v=(e>0?kv.substring(e+1):"1");
      if (k=="room"){ int r=v.toInt(); if(r>=0 && r<=255) roomNo=(uint8_t)r; }
      else if (k=="idx"){ int i=v.toInt(); if(i>=1 && i<=255) idx=(uint8_t)i; }
      else if (k=="slots"){ int s=v.toInt(); if(s<1) s=1; if(s>255) s=255; slots=(uint8_t)s; }
      else if (k=="next"){ if(!strToMac(v,nextMac)) memset(nextMac,0,6); }
      else if (k=="tail"){ isTail=(v.toInt()!=0); }
      p=c+1;
    }
    savePrefs(); sendNow(src,"ACK:CFG"); return;
  }

  if (msg.startsWith("MAP,")){
    int ni=msg.indexOf("name="), ii=msg.indexOf("id=");
    if (ni>0 && ii>0){
      int nc=msg.indexOf(',',ni+5); if(nc<0) nc=msg.length();
      String name=msg.substring(ni+5,nc); name.trim();
      uint16_t id=(uint16_t)msg.substring(ii+3).toInt();
      if (id>=1 && id<=MAX_TAG_ID && name.length()){ raw2id[name]=id; sendNow(src,"ACK:MAP"); }
    }
    return;
  }

  if (msg.startsWith("REG,")){ sendNow(src,"ACK:REG"); return; }

  if (msg.startsWith("REPORT:")){
    // Merge downstream maxima into our best[] (dedupe across rooms)
    String pl=msg.substring(7); int p=0;
    while(p<(int)pl.length()){
      int c=pl.indexOf(',',p); if(c<0) c=pl.length();
      String tok=pl.substring(p,c); tok.trim();
      int dot=tok.indexOf('.'); int at=tok.indexOf('@');
      if (dot>0 && at>dot){
        int room=tok.substring(0,dot).toInt();
        int id  =tok.substring(dot+1,at).toInt();
        int str =tok.substring(at+1).toInt();
        if (id>=1 && id<=MAX_TAG_ID){ if(str<0) str=0; if(str>100) str=100; updateBest((uint16_t)id,(uint8_t)str,(uint8_t)room); }
      }
      p=c+1;
    }
    return;
  }
}

// ---------- ICE UART ----------
static String line;
static void pumpICE(){
  while (SICE.available()){
    char ch=(char)SICE.read();
    if (ch=='\n'){
      String s=line; line=""; s.trim(); if(!s.length()) continue;
      // RAW,<name>,<rssi>  OR  MAC,<aa:bb:..>,<rssi> (MAC mapping optional)
      if (s.startsWith("RAW,")){
        int a=s.indexOf(',',4); if(a>0){
          String name=s.substring(4,a); name.trim();
          int rssi=s.substring(a+1).toInt();
          auto it=raw2id.find(name);
          if (it!=raw2id.end()){
            uint16_t id=it->second; uint8_t st=rssiToStrength(rssi);
            updateBest(id, st, roomNo);
          }
        }
      }
      // else if (s.startsWith("MAC,")) { /* add MAC->id mapping if you want */ }
    } else if (ch!='\r'){
      line+=ch; if(line.length()>256) line.remove(0,64);
    }
  }
}

// ---------- setup/loop ----------
void setup(){
  Serial.begin(115200);
  loadPrefs();
  SICE.begin(115200, SERIAL_8N1, ICE_RX1, ICE_TX1);

  WiFi.mode(WIFI_STA); esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); delay(3000); ESP.restart(); }
  esp_now_register_recv_cb(onRecv); addPeer(BCAST);

  memset(bestStr,0,sizeof(bestStr)); memset(bestRoom,0,sizeof(bestRoom));
  memset(inActive,0,sizeof(inActive)); activeIds.clear();

  uint8_t me[6]; WiFi.macAddress(me); char mm[18]; macToStr(me,mm);
  Serial.printf("[SMOKE] boot mac=%s idx=%u slots=%u room=%u tail=%u\n", mm, idx, slots, roomNo, isTail?1:0);
}

void loop(){
  pumpICE();

  static uint32_t lastHello=0; if (millis()-lastHello>2000){ lastHello=millis(); sendHello(); }

  static uint32_t nextFrame=0;
  uint16_t slotMs = (uint16_t)max((int)MIN_SLOT, (int)(FRAME_MS / max((int)slots,1)));
  if ((int32_t)(millis()-nextFrame) >= 0){
    uint32_t now=millis();
    uint32_t frameStart = now - (now % FRAME_MS);
    nextFrame = frameStart + FRAME_MS;

    uint32_t slotAt = frameStart + ((uint32_t)(idx-1) * slotMs);
    if ((int32_t)(slotAt - now) < 0) slotAt += FRAME_MS;
    while ((int32_t)(millis()-slotAt) < 0) delay(1);

    sendReport();
    decayAndCompact();
  }
}
