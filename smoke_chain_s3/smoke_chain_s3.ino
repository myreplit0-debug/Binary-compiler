/*
  smoke_chain_s3.ino  —  SMOKE node (ESP32-S3) • Arduino-ESP32 core 3.x

  Chain:
    ICE --UART1--> SMOKE(1) --ESP-NOW ch6--> ... --> SMOKE(tail) --UART1--> UI S3

  UART1:
    • Non-tail: RX from ICE (raw BLE lines)
    • Tail:     TX to UI (“UI: r.t@s, r.t@s, …”) and HELLO/CFG passthrough

  UI config (sent over ESPNOW; also accepted on UART1 for bench):
    CFG:room=<n>      (0..25)
    CFG:tail=0|1
    CFG:next=AA:BB:CC:DD:EE:FF
    CFG:clear

  All UI assignments happen in the UI. Nodes boot unassigned (room=0).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>

// === Forward declare to satisfy Arduino's auto-prototype generator ===
struct Agg;

// ---------- Pins ----------
#define RX1_PIN    18
#define TX1_PIN    17
#define UART_BAUD  115200

// ---------- ESPNOW ----------
#define ESPNOW_CH  6
static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- Limits & timing ----------
#define MAX_ITEMS        160
#define TAG_TTL_MS       2000
#define CYCLE_MS          800
#define FWD_DEBOUNCE_MS    30
#define HELLO_MS         3000

// ---------- Persisted config ----------
Preferences prefs;
struct Config {
  uint8_t room = 0;        // 0 = unassigned
  bool    isTail = false;
  bool    hasNext = false;
  uint8_t next[6] = {0};
} CFG;

static void saveCfg(){
  prefs.begin("smoke", false);
  prefs.putUChar("room", CFG.room);
  prefs.putBool("tail", CFG.isTail);
  prefs.putBool("hasNext", CFG.hasNext);
  prefs.putBytes("next", CFG.next, 6);
  prefs.end();
}
static void loadCfg(){
  prefs.begin("smoke", true);
  CFG.room    = prefs.getUChar("room", 0);
  CFG.isTail  = prefs.getBool("tail", false);
  CFG.hasNext = prefs.getBool("hasNext", false);
  size_t n = prefs.getBytesLength("next");
  if(n==6) prefs.getBytes("next", CFG.next, 6);
  prefs.end();
}

// ---------- Small helpers ----------
static inline uint8_t clampU8(int v){ if(v<0) v=0; if(v>100) v=100; return (uint8_t)v; }

static bool macFromStr(const String& s, uint8_t out[6]){
  int p=0, b=0;
  while(b<6){
    if(p+2 > (int)s.length()) return false;
    int v = strtol(s.substring(p,p+2).c_str(), nullptr, 16);
    out[b++] = (uint8_t)v;
    p += 2;
    if(b<6){ if(p >= (int)s.length() || s[p] != ':') return false; p++; }
  }
  return true;
}
static String macToStr(const uint8_t m[6]){
  char buf[18]; sprintf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); return String(buf);
}
static uint8_t MYMAC[6];

// ---------- Local dedup (per node) ----------
struct Item { uint16_t tag; uint8_t str; uint32_t seen; };
static Item table_[MAX_ITEMS]; static int nItems=0;

static uint16_t hash16(const String& s){
  uint32_t h=2166136261u;
  for(size_t i=0;i<s.length();++i){ h^=(uint8_t)s[i]; h*=16777619u; }
  return (uint16_t)((h>>16)^(h&0xFFFF));
}
static uint8_t rssiToStrength(int rssi){
  if(rssi<-95) rssi=-95; if(rssi>-35) rssi=-35;
  int v=(rssi+95)*100/60; return clampU8(v);
}
static int findTag(uint16_t t){ for(int i=0;i<nItems;i++) if(table_[i].tag==t) return i; return -1; }
static void upsert(uint16_t tag, uint8_t str){
  uint32_t now=millis();
  int i=findTag(tag);
  if(i<0){
    if(nItems<MAX_ITEMS) table_[nItems++]={tag,str,now};
    else table_[random(nItems)]={tag,str,now};
  }else{
    if(str>table_[i].str) table_[i].str=str;
    table_[i].seen=now;
  }
}
static void decay(){
  uint32_t now=millis();
  for(int i=0;i<nItems;){
    if(now - table_[i].seen > TAG_TTL_MS) table_[i]=table_[--nItems];
    else i++;
  }
}

// Ingest ICE UART line → update local table
static void parseIceLine(String s){
  s.trim(); if(!s.length()) return;
  String name; long id=-1; int rssi=0; bool have=false;

  int c=s.indexOf(','); String a=(c>=0?s.substring(0,c):s), b=(c>=0?s.substring(c+1):"");
  if(b.length()){
    int p=b.indexOf("rssi=");
    if(p>=0) rssi=b.substring(p+5).toInt();
    else     rssi=b.toInt();
    have=true;
  }
  if(a.startsWith("ID=")||a.startsWith("id=")) id=a.substring(3).toInt();
  else{
    int pn=a.indexOf("name=");
    name=(pn>=0? a.substring(pn+5):a);
  }
  uint16_t tag=(id>=0)?(uint16_t)id:hash16(name);
  if(have && tag>0) upsert(tag, rssiToStrength(rssi));
}

// ---------- Per-cycle aggregator ----------
struct Agg {
  int      cyc = -1;
  uint16_t tag[MAX_ITEMS];
  uint8_t  str[MAX_ITEMS];
  int      n = 0;
  uint32_t updated = 0;
};

static void aggReset(Agg& A, int cyc){ A.cyc=cyc; A.n=0; A.updated=millis(); }
static int  aggFind(const Agg& A, uint16_t t){ for(int i=0;i<A.n;i++) if(A.tag[i]==t) return i; return -1; }
static void aggMergeOne(Agg& A, uint16_t t, uint8_t s){
  int i=aggFind(A,t);
  if(i<0){ if(A.n<MAX_ITEMS){ A.tag[A.n]=t; A.str[A.n]=s; A.n++; } }
  else if(s>A.str[i]) A.str[i]=s;
  A.updated=millis();
}
static void aggMergeList(Agg& A, const String& payload, int room){
  int i=0;
  while(i < (int)payload.length()){
    int comma = payload.indexOf(',', i); if(comma<0) comma=payload.length();
    String tok=payload.substring(i, comma); tok.trim();
    int d=tok.indexOf('.'); int at=tok.indexOf('@');
    if(d>0 && at>d){
      int r = tok.substring(0,d).toInt();
      uint16_t t = (uint16_t)tok.substring(d+1,at).toInt();
      int sval = tok.substring(at+1).toInt();
      uint8_t s = clampU8(sval);
      if(room==0 || r==room) aggMergeOne(A,t,s);
    }
    i = comma+1;
  }
}
static void aggMergeLocal(Agg& A){
  decay();
  if(CFG.room==0) return;
  for(int i=0;i<nItems;i++) aggMergeOne(A, table_[i].tag, table_[i].str);
}
static String aggToPayload(const Agg& A, int room){
  String o; bool first=true;
  for(int i=0;i<A.n;i++){
    if(!first) o += ", ";
    first=false;
    o += String((int)room) + "." + String((int)A.tag[i]) + "@" + String((int)A.str[i]);
  }
  return o;
}

// Working sets + pacing
static Agg FWD, UIA;
static bool fwdArmed=false; static uint32_t fwdDue=0;
static int  lastPrintedCyc=-1;

// ---------- ESPNOW callbacks (core 3.x) ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  (void)info;
  if(!data || len<=0) return;

  // Config frames
  if(len>=4 && data[0]=='C' && data[1]=='F' && data[2]=='G' && data[3]==':'){
    String s((const char*)data, len); s.trim();
    if(s.indexOf("clear")>=0){
      CFG.room=0; CFG.isTail=false; CFG.hasNext=false; memset(CFG.next,0,6); saveCfg();
      return;
    }
    int p;
    if((p=s.indexOf("room="))>=0){ CFG.room = (uint8_t)constrain(s.substring(p+5).toInt(),0,25); saveCfg(); }
    if((p=s.indexOf("tail="))>=0){ CFG.isTail = (s.substring(p+5).toInt()!=0); saveCfg(); }
    if((p=s.indexOf("next="))>=0){
      String mac = s.substring(p+5); mac.trim();
      uint8_t tmp[6]; if(macFromStr(mac,tmp)){ memcpy(CFG.next,tmp,6); CFG.hasNext=true; saveCfg(); }
    }
    return;
  }

  // Upstream data U#C:<list>
  if(len>=3 && data[0]=='U' && data[1]=='#'){
    int i=2; int j=i; while(j<len && data[j]!=':') j++;
    if(j>=len) return;
    int cyc = String((const char*)data+i, j-i).toInt();
    String pay((const char*)data+j+1, len-j-1);

    if(!CFG.isTail){
      if(FWD.cyc!=cyc) aggReset(FWD,cyc);
      aggMergeList(FWD, pay, CFG.room);
      aggMergeLocal(FWD);
      fwdArmed=true; fwdDue=millis()+FWD_DEBOUNCE_MS;
    }else{
      if(UIA.cyc!=cyc) aggReset(UIA,cyc);
      aggMergeList(UIA, pay, CFG.room);
    }
    return;
  }

  // HELLO passthrough to UART (tail only)
  if(len>=5 && data[0]=='H' && data[1]=='E'){
    if(CFG.isTail) { Serial1.write(data, len); Serial1.write('\n'); }
  }
}

static void onSent(const uint8_t* mac_addr, esp_now_send_status_t status){
  (void)mac_addr; (void)status;
}

// ---------- ESPNOW init ----------
static bool addPeerIfMissing(const uint8_t mac[6]){
  if(esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p={};
  memcpy(p.peer_addr, mac, 6);
  p.ifidx   = WIFI_IF_STA;
  p.channel = ESPNOW_CH;
  p.encrypt = false;
  return esp_now_add_peer(&p)==ESP_OK;
}
static bool espnowInit(){
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  if(esp_now_init()!=ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb((esp_now_send_cb_t)onSent);   // explicit cast
  addPeerIfMissing(BCAST);
  if(CFG.hasNext) addPeerIfMissing(CFG.next);
  esp_wifi_get_mac(WIFI_IF_STA, MYMAC);
  return true;
}
static void sendStr(const uint8_t mac[6], const String& s){
  addPeerIfMissing(mac);
  esp_now_send(mac, (const uint8_t*)s.c_str(), s.length());
}
static void broadcastStr(const String& s){ sendStr(BCAST, s); }

// ---------- Periodics ----------
static uint32_t tHello=0, tCycleTick=0;
static String uartBuf;

static void sendHello(){
  String s = "HELLO,mac="+macToStr(MYMAC)+",room="+String((int)CFG.room)+",tail="+String((int)CFG.isTail);
  if(CFG.hasNext) s += ",next="+macToStr(CFG.next);
  broadcastStr(s);
  if(CFG.isTail) { Serial1.println(s); }
}

static void originateCycle(){
  if(CFG.isTail || !CFG.hasNext) return;
  int cyc = (int)(millis()/CYCLE_MS);
  Agg A; aggReset(A, cyc);
  aggMergeLocal(A);
  String payload = aggToPayload(A, CFG.room);
  if(payload.length()){
    String frame = String("U#")+String(cyc)+": "+payload;
    sendStr(CFG.next, frame);
  }
}

static void forwardIfDue(){
  if(CFG.isTail || !CFG.hasNext) return;
  if(!fwdArmed || millis()<fwdDue) return;
  fwdArmed=false;
  String payload = aggToPayload(FWD, CFG.room);
  if(payload.length()){
    String frame = String("U#")+String(FWD.cyc)+": "+payload;
    sendStr(CFG.next, frame);
  }
}

static void tailPrintIfReady(){
  if(!CFG.isTail) return;
  int cyc = (int)(millis()/CYCLE_MS);
  if(UIA.cyc!=cyc || lastPrintedCyc==cyc) return;
  aggMergeLocal(UIA);
  String pay = aggToPayload(UIA, CFG.room);
  if(pay.length()){
    Serial1.print("UI: "); Serial1.println(pay);
    lastPrintedCyc = cyc;
  }
}

// ---------- Setup / Loop ----------
void setup(){
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX1_PIN, TX1_PIN);
  delay(50);

  loadCfg();
  Serial.printf("[CFG] room=%u tail=%d hasNext=%d next=%s\n",
      CFG.room, (int)CFG.isTail, (int)CFG.hasNext, CFG.hasNext? macToStr(CFG.next).c_str():"--");

  if(!espnowInit()) Serial.println("[ERR] esp_now_init failed"); else Serial.println("[OK] ESPNOW ch6");
  sendHello();
}

void loop(){
  // UART1 ingest (ICE lines or bench CFG: commands)
  while(Serial1.available()){
    char c=(char)Serial1.read();
    if(c=='\n'){
      String s=uartBuf; uartBuf=""; s.trim();
      if(s.startsWith("CFG:")){
        if(s.indexOf("clear")>=0){ CFG.room=0; CFG.isTail=false; CFG.hasNext=false; memset(CFG.next,0,6); saveCfg(); }
        int p;
        if((p=s.indexOf("room="))>=0){ CFG.room=(uint8_t)constrain(s.substring(p+5).toInt(),0,25); saveCfg(); }
        if((p=s.indexOf("tail="))>=0){ CFG.isTail=(s.substring(p+5).toInt()!=0); saveCfg(); }
        if((p=s.indexOf("next="))>=0){
          String mac = s.substring(p+5); mac.trim();
          uint8_t tmp[6]; if(macFromStr(mac,tmp)){ memcpy(CFG.next,tmp,6); CFG.hasNext=true; saveCfg(); addPeerIfMissing(CFG.next); }
        }
      }else if(s.length()){
        parseIceLine(s);
      }
    }else if(c!='\r'){
      uartBuf += c;
      if(uartBuf.length()>600) uartBuf.remove(0,200);
    }
  }

  decay();

  uint32_t now=millis();
  if(now - tHello     >= HELLO_MS) { tHello=now; sendHello(); }
  if(now - tCycleTick >= CYCLE_MS) { tCycleTick=now; originateCycle(); }
  forwardIfDue();
  tailPrintIfReady();
}
