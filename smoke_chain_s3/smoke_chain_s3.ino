/*
  smoke_chain_s3.ino  —  SMOKE node (ESP32-S3)  •  Core 3.x / IDF 5.x friendly

  Topology:
    ICE  --UART1-->  SMOKE(1) --ESP-NOW--> SMOKE(2) --...--> TAIL(SMOKE) --UART1--> UI S3

  Chain behavior (per ~800 ms cycle):
    - Each non-tail SMOKE originates U#C frames (its local tags) to its next hop.
    - Any SMOKE that receives U#C merges (max strength per tag) and forwards one merged U#C upstream.
    - Tail merges everything for that cycle and prints a single:
         UI: <room>.<tag>@<str>, <room>.<tag>@<str>, ...
      on UART1 for the UI device.

  UI-side configuration (over ESP-NOW or UART1 for bench):
    CFG:room=<n>        (0..25)
    CFG:tail=0|1
    CFG:next=AA:BB:CC:DD:EE:FF
    CFG:clear

  Also broadcasts HELLO every few seconds:
    HELLO,mac=AA:..,room=N,tail=0/1,next=AA:..  (tail also prints this to UART1)

  Pins (edit to suit):
    UART1 RX/TX → ICE (tail’s UART1 goes to UI S3)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>        // wifi_tx_info_t
#include <Preferences.h>

// ---------------- Pins ----------------
#define RX1_PIN      18
#define TX1_PIN      17
#define UART_BAUD    115200

// ---------------- ESPNOW ----------------
#define ESPNOW_CH        6
static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------------- Limits & timings ----------------
#define MAX_ITEMS        160
#define TAG_TTL_MS       2000
#define CYCLE_MS          800
#define FWD_DEBOUNCE_MS    30
#define HELLO_MS         3000

// ---------------- Persisted config ----------------
Preferences prefs;
struct Config {
  uint8_t room = 0;        // 0 = unassigned
  bool    isTail = false;
  bool    hasNext = false;
  uint8_t next[6] = {0};   // next hop MAC (toward tail)
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

// ---------------- Helpers ----------------
static bool macFromStr(const String& s, uint8_t out[6]){
  int p=0, b=0;
  while(b<6){
    if(p+2 > (int)s.length()) return false;
    int hi = strtol(s.substring(p,p+2).c_str(), nullptr, 16);
    out[b++] = (uint8_t)hi;
    p += 2;
    if(b<6){
      if(p >= (int)s.length() || s[p] != ':') return false;
      p++;
    }
  }
  return true;
}
static String macToStr(const uint8_t m[6]){
  char buf[18];
  sprintf(buf,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]);
  return String(buf);
}
static uint8_t MYMAC[6];

// ---------------- Local dedup table (live tags from ICE) ----------------
struct Item { uint16_t tag; uint8_t str; uint32_t seen; };
static Item table[MAX_ITEMS]; static int nItems=0;

static uint16_t hash16(const String& s){
  uint32_t h=2166136261u;
  for(size_t i=0;i<s.length();++i){ h^=(uint8_t)s[i]; h*=16777619u; }
  return (uint16_t)((h>>16)^(h&0xFFFF));
}
static uint8_t rssiToStrength(int rssi){
  if(rssi<-95) rssi=-95; if(rssi>-35) rssi=-35;
  int v=(rssi+95)*100/60; if(v<0) v=0; if(v>100) v=100; return (uint8_t)v;
}
static int findTag(uint16_t t){ for(int i=0;i<nItems;i++) if(table[i].tag==t) return i; return -1; }
static void upsert(uint16_t tag, uint8_t str){
  uint32_t now=millis();
  int i=findTag(tag);
  if(i<0){
    if(nItems<MAX_ITEMS) table[nItems++]={tag,str,now};
    else table[random(nItems)]={tag,str,now};           // crude fallback if full
  }else{
    if(str>table[i].str) table[i].str=str;
    table[i].seen=now;
  }
}
static void decay(){
  uint32_t now=millis();
  for(int i=0;i<nItems;){
    if(now - table[i].seen > TAG_TTL_MS) table[i]=table[--nItems];
    else i++;
  }
}

// ingest lines from ICE on UART1 (many shapes tolerated)
static void parseIceLine(String s){
  s.trim(); if(!s.length()) return;
  String name; int id=-1; int rssi=0; bool have=false;

  int c=s.indexOf(','); String a=(c>=0?s.substring(0,c):s), b=(c>=0?s.substring(c+1):"");
  if(b.length()){ int p=b.indexOf("rssi="); if(p>=0){ rssi=b.substring(p+5).toInt(); have=true; } else { rssi=b.toInt(); have=true; } }
  if(a.startsWith("ID=")||a.startsWith("id=")) id=a.substring(3).toInt();
  else{ int pn=a.indexOf("name="); name=(pn>=0? a.substring(pn+5):a); }

  uint16_t tag=(id>=0)?(uint16_t)id:hash16(name);
  if(have && tag>0) upsert(tag, rssiToStrength(rssi));
}

// ---------------- Cycle aggregator ----------------
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
static void aggMergeList(Agg& A, const String& payload /* "room.tag@str, ..." */, int room){
  int i=0;
  while(i < (int)payload.length()){
    int comma = payload.indexOf(',', i); if(comma<0) comma=payload.length();
    String tok=payload.substring(i, comma); tok.trim();
    int d=tok.indexOf('.'); int at=tok.indexOf('@');
    if(d>0 && at>d){
      int r = tok.substring(0,d).toInt();
      uint16_t t = (uint16_t)tok.substring(d+1,at).toInt();
      uint8_t s = (uint8_t)max(0,min(100,tok.substring(at+1).toInt()));
      if(room==0 || r==room) aggMergeOne(A,t,s);
    }
    i = comma+1;
  }
}
static void aggMergeLocal(Agg& A){
  decay();
  if(CFG.room==0) return;
  for(int i=0;i<nItems;i++) aggMergeOne(A, table[i].tag, table[i].str);
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

// Two aggregators:
//  - FWD: non-tail merges incoming (and local) for a cycle then forwards upstream once
//  - UI : tail merges everything and prints one UI: line
static Agg FWD, UIA;
static bool fwdArmed=false; static uint32_t fwdDue=0;
static int  lastPrintedCyc=-1;

// ---------------- ESPNOW callbacks (IDF 5.x signatures) ----------------
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  if(!data || len<=0) return;

  // --- Config frames ---
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

  // --- U#C: <list> ---
  if(len>=3 && data[0]=='U' && data[1]=='#'){
    // parse cycle
    int i=2; int j=i; while(j<len && data[j]!=':') j++;
    if(j>=len) return;
    int cyc = String((const char*)data+i, j-i).toInt();
    String pay((const char*)data+j+1, len-j-1);

    // Merge into forward aggregator
    if(!CFG.isTail){
      if(FWD.cyc!=cyc) aggReset(FWD,cyc);
      aggMergeList(FWD, pay, CFG.room);
      aggMergeLocal(FWD);
      fwdArmed=true; fwdDue=millis()+FWD_DEBOUNCE_MS;   // debounce multiple sources
    }

    // Merge into UI aggregator (tail only)
    if(CFG.isTail){
      if(UIA.cyc!=cyc){ aggReset(UIA,cyc); }
      aggMergeList(UIA, pay, CFG.room);   // room filter; all frames should carry proper room
      // local will be merged before print
    }
    return;
  }

  // --- HELLO passthrough to UI (tail) ---
  if(len>=5 && data[0]=='H' && data[1]=='E'){
    if(CFG.isTail && info){
      Serial1.write(data, len); Serial1.write('\n');
    }
  }
}

void onSent(const uint8_t* mac, wifi_tx_info_t* tx, esp_now_send_status_t st){
  (void)mac; (void)tx; (void)st;
}

// ---------------- ESPNOW init ----------------
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
  esp_now_register_send_cb(onSent);
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

// ---------------- Periodics ----------------
static uint32_t tHello=0, tCycleTick=0;
static String uartBuf;

static void sendHello(){
  String s = "HELLO,mac="+macToStr(MYMAC)+",room="+String((int)CFG.room)+",tail="+String((int)CFG.isTail);
  if(CFG.hasNext) s += ",next="+macToStr(CFG.next);
  broadcastStr(s);
  if(CFG.isTail) { Serial1.println(s); }
}

static void originateCycle(){
  // build U#C from local only and send to next
  if(CFG.isTail || !CFG.hasNext) return;
  int cyc = (int)(millis()/CYCLE_MS);
  Agg A; aggReset(A, cyc);
  aggMergeLocal(A);
  String payload = aggToPayload(A, CFG.room);
  String frame = String("U#")+String(cyc)+": "+payload;
  if(payload.length()) sendStr(CFG.next, frame);
}

static void forwardIfDue(){
  if(CFG.isTail || !CFG.hasNext) return;
  if(!fwdArmed || millis()<fwdDue) return;
  fwdArmed=false;
  String payload = aggToPayload(FWD, CFG.room);
  String frame = String("U#")+String(FWD.cyc)+": "+payload;
  if(payload.length()) sendStr(CFG.next, frame);
}

static void tailPrintIfReady(){
  if(!CFG.isTail) return;
  int cyc = (int)(millis()/CYCLE_MS);
  if(UIA.cyc!=cyc) return;             // not this cycle yet
  if(lastPrintedCyc==cyc) return;      // already printed

  // include local before printing
  aggMergeLocal(UIA);

  String pay = aggToPayload(UIA, CFG.room);
  if(pay.length()){
    Serial1.print("UI: "); Serial1.println(pay);
    lastPrintedCyc = cyc;
  }
}

// ---------------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX1_PIN, TX1_PIN);
  delay(50);

  loadCfg();
  Serial.printf("[CFG] room=%u tail=%d hasNext=%d next=%s\n",
      CFG.room, (int)CFG.isTail, (int)CFG.hasNext, CFG.hasNext? macToStr(CFG.next).c_str():"--");

  if(!espnowInit()) Serial.println("[ERR] esp_now_init failed"); else Serial.println("[OK] ESPNOW ch6");

  // First hello so UI can see us
  sendHello();
}

void loop(){
  // UART1 ingest (ICE… or bench config)
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
        parseIceLine(s);              // ICE line
      }
    }else if(c!='\r'){
      uartBuf += c;
      if(uartBuf.length()>600) uartBuf.remove(0,200);
    }
  }

  // Housekeeping
  decay();

  uint32_t now=millis();
  if(now - tHello >= HELLO_MS){ tHello=now; sendHello(); }

  // Each node originates its own cycle frame (non-tail)
  if(now - tCycleTick >= CYCLE_MS){ tCycleTick=now; originateCycle(); }

  // Forward merged frames when debounce expires
  forwardIfDue();

  // Tail prints one UI line per cycle
  tailPrintIfReady();
}
