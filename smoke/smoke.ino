/*
  smoke_chain_s3.ino
  SMOKE node for chained key-tracker (ESP32-S3), 100% UI-driven.

  Boot behavior:
   - Start ESP-NOW on ch6.
   - Broadcast HELLO every 3s (room=0 until UI sets it).
   - No USB/Serial setup needed; UI assigns rooms/order/tail via the tail.

  Data flow:
   - Mid nodes: ICE -> UART1 (RX1=16, TX1=17). Parse "RAW|RSSI" or "123|RSSI".
                Map RAW->tag via MAP from UI; keep best strength per round.
                Send one summary E-frame to nextPeer during its slot.
   - Tail node: UART1 <-> UI (no ICE). Rebroadcasts UI control lines over RF.
                Starts rounds, aggregates all E-frames, prints single
                "UI: r.t@S, ..." line to UI on UART1 each second.

  UI control lines (ASCII):
    MAP,name=<RAW>,id=<N>
    SET,room=<N>[,mac=<AA:BB:CC:DD:EE:FF>]
    SET,next=<AA:BB:CC:DD:EE:FF|clear>[,mac=<...>]
    SET,tail=0|1[,mac=<...>]
    SET,chan=<1..13>[,mac=<...>]
    SET,reset=1[,mac=<...>]
    HELLO?

  Build target: ESP32S3 Dev Module (esp32:esp32:esp32s3)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ---------------- Pins ----------------
#define RX1_PIN 16
#define TX1_PIN 17

// ---------------- Limits / timing ----------------
static const int     MAX_ROOMS   = 25;
static const int     MAX_TAGS    = 1024;
static const int     MAX_MAP     = 220;     // RAW->tag alias entries
static const int     MAX_NAME    = 32;
static const uint8_t DEFAULT_CH  = 6;

static const uint32_t ROUND_PERIOD_MS = 1000; // tail emits every 1s
static const uint32_t COLLECT_MS      = 500;  // collect ICE hits
static const uint32_t SLOT_MS         = 25;   // per-room send offset
static const uint32_t GUARD_MS        = 250;  // guard before UI emit

static const uint32_t HELLO_DISC_MS   = 3000; // HELLO freq while unassigned
static const uint32_t HELLO_STEADY_MS = 15000;// HELLO freq once assigned

// ---------------- Node state (persisted) ----------------
static uint8_t myMac[6] = {0};
static int     myRoom   = 0;       // 0 until UI assigns
static bool    isTail   = false;
static uint8_t nextPeer[6] = {0};
static uint8_t radioCh  = DEFAULT_CH;

Preferences prefs;

// ---------------- Alias table (RAM) ----------------
struct MapEntry { char name[MAX_NAME]; uint16_t id; bool used; };
MapEntry aliasTab[MAX_MAP];

// ---------------- Per-round ----------------
uint8_t bestStr[MAX_TAGS + 1];          // 0..100
bool    seenOrigin[MAX_ROOMS + 1];      // received upstream cache per room
String  cacheByRoom[MAX_ROOMS + 1];     // E-frames cached
uint32_t roundStartMs = 0;
int      roundId = 0;

// ---------------- Flags / buffers ----------------
bool dbg_rf=false, dbg_ble=false, dbg_tail=false;
String u1Q; // UART1 queue

// ===================================================
// Helpers
// ===================================================
static inline void macToStr(const uint8_t* m, char* out, bool lower=false){
  if(lower) sprintf(out,"%02x:%02x:%02x:%02x:%02x:%02x", m[0],m[1],m[2],m[3],m[4],m[5]);
  else      sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X", m[0],m[1],m[2],m[3],m[4],m[5]);
}
static inline bool macIsZero(const uint8_t* m){ for(int i=0;i<6;i++) if(m[i]) return false; return true; }

static bool parseMac(const String& s, uint8_t out[6]){
  int parts[6]={0}; int n=sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &parts[0],&parts[1],&parts[2],&parts[3],&parts[4],&parts[5]);
  if(n!=6) return false;
  for(int i=0;i<6;i++) out[i]=(uint8_t)parts[i];
  return true;
}

static inline void addOrReplacePeer(const uint8_t* mac, int chan){
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  esp_now_peer_info_t p{}; memcpy(p.peer_addr, mac, 6);
  p.channel = chan; p.encrypt = false;
  esp_now_add_peer(&p);
}
static inline void sendUnicastOrBcast(const uint8_t* mac, const String& s){
  if(mac && !macIsZero(mac)) esp_now_send(mac, (const uint8_t*)s.c_str(), s.length());
  else {
    uint8_t bcast[6]; memset(bcast,0xFF,6);
    if(!esp_now_is_peer_exist(bcast)) addOrReplacePeer(bcast, radioCh);
    esp_now_send(bcast,(const uint8_t*)s.c_str(),s.length());
  }
  if(dbg_rf){ char m[20]; macToStr(mac?mac:nextPeer,m); Serial.printf("[RF TX] %s -> %s\n", s.c_str(), macIsZero(mac?mac:nextPeer)?"BCAST":m); }
}

static inline String myMacStr(bool lower=false){ char m[20]; macToStr(myMac,m,lower); return String(m); }
static inline bool looksNumeric(const String& s){
  if(!s.length()) return false; for(size_t i=0;i<s.length();++i) if(s[i]<'0'||s[i]>'9') return false; return true;
}
static inline int rssiToStrength(int rssi){
  if(rssi>-35) rssi=-35; if(rssi<-100) rssi=-100;
  int s=(rssi+100)*100/65; if(s<0) s=0; if(s>100) s=100; return s;
}
static inline int mapNameToId(const String& nm){
  for(int i=0;i<MAX_MAP;i++) if(aliasTab[i].used && nm.equals(aliasTab[i].name)) return aliasTab[i].id;
  return 0;
}
static inline void aliasSet(const String& raw, uint16_t id){
  int freeIdx=-1;
  for(int i=0;i<MAX_MAP;i++){
    if(aliasTab[i].used && raw.equals(aliasTab[i].name)){ aliasTab[i].id=id; return; }
    if(!aliasTab[i].used && freeIdx<0) freeIdx=i;
  }
  if(freeIdx>=0){
    aliasTab[freeIdx].used=true;
    strncpy(aliasTab[freeIdx].name, raw.c_str(), MAX_NAME-1);
    aliasTab[freeIdx].name[MAX_NAME-1]=0;
    aliasTab[freeIdx].id=id;
  }
}

// ===================================================
// Persistence
// ===================================================
void loadConfig(){
  prefs.begin("smoke", true);
  myRoom  = prefs.getUChar("room", 0);
  isTail  = prefs.getBool("tail", false);
  radioCh = prefs.getUChar("chan", DEFAULT_CH);
  size_t n = prefs.getBytes("next", nextPeer, 6); if(n!=6) memset(nextPeer,0,6);
  prefs.end();
}
void saveConfig(){
  prefs.begin("smoke", false);
  prefs.putUChar("room", (uint8_t)myRoom);
  prefs.putBool("tail", isTail);
  prefs.putUChar("chan", radioCh);
  prefs.putBytes("next", nextPeer, 6);
  prefs.end();
}
void clearConfig(){
  prefs.begin("smoke", false);
  prefs.clear();
  prefs.end();
}

// ===================================================
// Control handling (MAP/SET/HELLO)
// ===================================================
static String fieldOf(const String& s, const char* key){
  String k=String(key)+"="; int p=s.indexOf(k); if(p<0) return "";
  int e=s.indexOf(',', p+k.length()); if(e<0) e=s.length();
  String v=s.substring(p+k.length(), e); v.trim(); return v;
}

void sendHELLO(){
  String h = "HELLO,mac=" + myMacStr() + ",room=" + String(myRoom);
  // Broadcast so tail (and others) see it
  sendUnicastOrBcast(nullptr, h);
  // If we are tail, also echo to UI (UART1) so the web UI sees peers
  if(isTail) Serial1.println(h);
  if(dbg_rf) Serial.printf("[HELLO OUT] %s\n", h.c_str());
}

void applySET(const String& s, bool rebroadcast){
  // Optional target filter
  String tmac = fieldOf(s, "mac");
  if(tmac.length()){
    String me = myMacStr(true); tmac.toLowerCase();
    if(me != tmac){ if(rebroadcast) sendUnicastOrBcast(nullptr, s); return; }
  }

  bool changed=false;
  String sv = fieldOf(s,"room");
  if(sv.length()){ myRoom = sv.toInt(); changed=true; }

  sv = fieldOf(s,"next");
  if(sv.length()){
    if(sv=="clear" || sv=="none"){ memset(nextPeer,0,6); changed=true; }
    else {
      uint8_t m6[6]; if(parseMac(sv,m6)){ memcpy(nextPeer,m6,6); addOrReplacePeer(nextPeer, radioCh); changed=true; }
    }
  }

  sv = fieldOf(s,"tail");
  if(sv.length()){ isTail = (sv.toInt()!=0); changed=true; }

  sv = fieldOf(s,"chan");
  if(sv.length()){
    int c=sv.toInt(); if(c>=1 && c<=13){
      radioCh=(uint8_t)c;
      esp_wifi_set_channel(radioCh, WIFI_SECOND_CHAN_NONE);
      changed=true;
    }
  }

  sv = fieldOf(s,"reset");
  if(sv=="1"){
    clearConfig();
    delay(50);
    ESP.restart();
  }

  if(changed) saveConfig();
  if(rebroadcast) sendUnicastOrBcast(nullptr, s);
}

void handleControl(const String& s, bool fromRF, bool rebroadcastAllowed){
  if(s.startsWith("MAP,")){
    // MAP,name=<RAW>,id=<N>
    String nm = fieldOf(s,"name");
    String id = fieldOf(s,"id");
    if(nm.length() && id.length()) aliasSet(nm, (uint16_t)id.toInt());
    if(rebroadcastAllowed && fromRF==false) sendUnicastOrBcast(nullptr, s); // tail echo to RF
  } else if(s.startsWith("SET,")){
    applySET(s, rebroadcastAllowed && !fromRF); // only tail rebroadcasts
  } else if(s.startsWith("HELLO?")){
    sendHELLO();
    if(rebroadcastAllowed && !fromRF) sendUnicastOrBcast(nullptr, s);
  }
}

// ===================================================
// ESP-NOW
// ===================================================
static void onSent(const uint8_t* mac, esp_now_send_status_t st){
  if(dbg_rf){ char m[20]; macToStr(mac,m); Serial.printf("[RF SENT] %s -> %s\n", m, st==ESP_NOW_SEND_SUCCESS?"OK":"FAIL"); }
}
static void onRecv(const uint8_t* mac, const uint8_t* data, int len){
  // ASCII lines
  String s; s.reserve(len+1);
  for(int i=0;i<len;i++){ char c=(char)data[i]; if(c>=32 && c<=126) s+=c; }

  if(dbg_rf){ char m[20]; macToStr(mac,m); Serial.printf("[RF RX] %s  from %s\n", s.c_str(), m); }

  if(s.startsWith("ROUND,")){
    int i=s.indexOf("id="); if(i>=0) roundId = s.substring(i+3).toInt();
    memset(bestStr,0,sizeof(bestStr));
    for(int r=0;r<=MAX_ROOMS;r++){ seenOrigin[r]=false; cacheByRoom[r]=""; }
    roundStartMs = millis();
  }
  else if(s.startsWith("E:")){
    // Cache upstream by origin room; forward if not tail
    int rpos = s.indexOf(",R:");
    if(rpos>1){
      int origin = s.substring(rpos+3).toInt();
      if(origin>=1 && origin<=MAX_ROOMS && !seenOrigin[origin]){
        seenOrigin[origin]=true; cacheByRoom[origin]=s;
        if(!isTail && !macIsZero(nextPeer)) esp_now_send(nextPeer,(const uint8_t*)s.c_str(), s.length());
      }
    }
  }
  else if(s.startsWith("HELLO,")){
    // Echo to UI if we are tail so the UI can list peers
    if(isTail) Serial1.println(s);
  }
  else if(s.startsWith("MAP,") || s.startsWith("SET,") || s.startsWith("HELLO?")){
    handleControl(s, /*fromRF=*/true, /*rebroadcastAllowed=*/false);
  }
}

// ===================================================
// UART1 pumps
//   - Mid nodes: ICE -> UART1 -> SMOKE
//   - Tail: UI  -> UART1 -> SMOKE (control out), and SMOKE -> UI (HELLO/UI lines)
// ===================================================
static inline void handleICEline(String line){
  line.trim(); if(!line.length()) return;
  int bar=line.indexOf('|');
  if(bar<0){ if(dbg_ble) Serial.printf("[ICE] %s\n", line.c_str()); return; }

  String nm = line.substring(0,bar); nm.trim();
  int rssi  = line.substring(bar+1).toInt();
  int id = looksNumeric(nm) ? nm.toInt() : mapNameToId(nm);
  if(id>0 && id<=MAX_TAGS){
    uint8_t s=(uint8_t)rssiToStrength(rssi);
    if(s>bestStr[id]) bestStr[id]=s;
    if(dbg_ble) Serial.printf("[ICE] %s -> tag%d @%d\n", nm.c_str(), id, s);
  } else {
    if(dbg_ble) Serial.printf("[ICE] UNMAPPED %s\n", nm.c_str());
  }
}

static void pumpUART1_midFromICE(){
  while(Serial1.available()){
    char c=(char)Serial1.read();
    if(c=='\n'){ String line=u1Q; u1Q=""; handleICEline(line); }
    else if(c!='\r'){ u1Q += c; if(u1Q.length()>4096) u1Q.remove(0,1024); }
  }
}

static void pumpUART1_tailFromUI(){
  while(Serial1.available()){
    char c=(char)Serial1.read();
    if(c=='\n'){
      String line=u1Q; u1Q="";
      line.trim(); if(!line.length()) continue;
      // Accept only control from UI; tail rebroadcasts
      if(line.startsWith("MAP,")||line.startsWith("SET,")||line.startsWith("HELLO?")){
        handleControl(line, /*fromRF=*/false, /*rebroadcastAllowed=*/true);
        if(dbg_tail) Serial.printf("[UI->RF] %s\n", line.c_str());
      }
    } else if(c!='\r'){
      u1Q += c; if(u1Q.length()>4096) u1Q.remove(0,1024);
    }
  }
}

// ===================================================
// Round logic
// ===================================================
static void sendMySummary(){
  if(macIsZero(nextPeer) || myRoom<=0) return; // not configured yet
  String s = "E:" + String(roundId) + ",R:" + String(myRoom) + ",";
  for(int id=1; id<=MAX_TAGS; id++){
    uint8_t st=bestStr[id];
    if(st){
      String p = String(id) + "@" + String(st) + ",";
      if(s.length()+p.length() < 235) s += p; else break;
    }
  }
  esp_now_send(nextPeer, (const uint8_t*)s.c_str(), s.length());
  if(dbg_rf) Serial.printf("[E FRAME] %s\n", s.c_str());
}

static void tailAggregateAndEmitUI(){
  if(!isTail) return;

  uint8_t  finalStr[MAX_TAGS+1]; memset(finalStr,0,sizeof(finalStr));
  uint16_t finalRoom[MAX_TAGS+1]; memset(finalRoom,0,sizeof(finalRoom));

  // include self
  for(int id=1; id<=MAX_TAGS; id++) if(bestStr[id]){ finalStr[id]=bestStr[id]; finalRoom[id]=myRoom; }

  auto parseE = [&](const String& fr){
    int rpos=fr.indexOf(",R:"); if(rpos<0) return;
    int origin=fr.substring(rpos+3).toInt();
    int idx=fr.indexOf(',', rpos+3); if(idx<0) idx=rpos+3; else idx+=1;
    while(idx<fr.length()){
      int at=fr.indexOf('@', idx); if(at<0) break;
      int c2=fr.indexOf(',', at);
      int id=fr.substring(idx,at).toInt();
      int st=(c2>0? fr.substring(at+1,c2).toInt() : fr.substring(at+1).toInt());
      if(id>0 && id<=MAX_TAGS && st>finalStr[id]){ finalStr[id]=st; finalRoom[id]=origin; }
      if(c2<0) break; idx=c2+1;
    }
  };
  for(int r=1;r<=MAX_ROOMS;r++) if(seenOrigin[r]) parseE(cacheByRoom[r]);

  String ui = "UI: ";
  for(int id=1; id<=MAX_TAGS; id++){
    if(finalStr[id] && finalRoom[id]){
      String part = String(finalRoom[id]) + "." + String(id) + "@" + String(finalStr[id]) + ", ";
      if(ui.length()+part.length()<500) ui += part; else break;
    }
  }
  Serial1.println(ui);
  if(dbg_tail) Serial.printf("[UI OUT] %s\n", ui.c_str());
}

// ===================================================
// Setup / Loop
// ===================================================
void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

  // Get MAC
#if __has_include(<esp_mac.h>)
  esp_read_mac(myMac, ESP_MAC_WIFI_STA);
#else
  esp_wifi_get_mac(WIFI_IF_STA, myMac);
#endif

  // Load persisted config
  loadConfig();

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(radioCh, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if(esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); while(1) delay(1000); }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  // Add broadcast peer so control/HELLO works before nextPeer assignment
  uint8_t bcast[6]; memset(bcast,0xFF,6);
  addOrReplacePeer(bcast, radioCh);

  // Clear per-round & aliases
  memset(bestStr,0,sizeof(bestStr));
  for(int r=0;r<=MAX_ROOMS;r++){ seenOrigin[r]=false; cacheByRoom[r]=""; }
  for(int i=0;i<MAX_MAP;i++) aliasTab[i].used=false;

  // Announce presence
  sendHELLO();

  Serial.printf("SMOKE up. mac=%s room=%d tail=%s ch=%d\n",
                myMacStr().c_str(), myRoom, isTail?"on":"off", radioCh);
}

void loop(){
  // Tail listens to UI, mids listen to ICE
  if(isTail) pumpUART1_tailFromUI();
  else       pumpUART1_midFromICE();

  uint32_t now=millis();

  // Periodic HELLO (UI discovery)
  static uint32_t lastHello=0;
  uint32_t helloIv = (myRoom==0 ? HELLO_DISC_MS : HELLO_STEADY_MS);
  if(now - lastHello >= helloIv){ lastHello=now; sendHELLO(); }

  if(isTail){
    // Tail: drive rounds
    static uint32_t lastRound=0;
    if(now - lastRound >= ROUND_PERIOD_MS){
      lastRound = now; roundId++;
      memset(bestStr,0,sizeof(bestStr));
      for(int r=0;r<=MAX_ROOMS;r++){ seenOrigin[r]=false; cacheByRoom[r]=""; }
      roundStartMs = now;
      String r = "ROUND,id=" + String(roundId);
      sendUnicastOrBcast(nullptr, r);  // broadcast
      if(dbg_tail) Serial.printf("[ROUND] %s\n", r.c_str());
    }
    if(roundStartMs && now >= roundStartMs + COLLECT_MS + (MAX_ROOMS*SLOT_MS) + GUARD_MS){
      tailAggregateAndEmitUI();
      roundStartMs=0;
    }
  } else {
    // Mid: send summary once per round, at its slot, only if configured
    static int sentRid=0;
    if(roundId && sentRid!=roundId && myRoom>0){
      if(now >= roundStartMs + COLLECT_MS + (uint32_t)(max(0,myRoom-1))*SLOT_MS){
        sendMySummary();
        // forward any cached upstream frames we already picked up (first-time only)
        if(!macIsZero(nextPeer)){
          for(int r=1;r<=MAX_ROOMS;r++) if(seenOrigin[r])
            esp_now_send(nextPeer,(const uint8_t*)cacheByRoom[r].c_str(), cacheByRoom[r].length());
        }
        sentRid=roundId;
      }
    }
  }
}
