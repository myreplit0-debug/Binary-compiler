// UI controller (ESP32-S3) — ESP-NOW only, Serial CLI
// - Prints REPORTs from tail to USB Serial
// - Manages chain (idx/next/slots, tail), rooms, and tag registry
// - Works with SMOKE/ICE sketches below
//
// Build FQBN: esp32:esp32:esp32s3

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <unordered_map>
#include <vector>
#include <array>

#define ESPNOW_CH 6

// ---------- utils ----------
static uint8_t BCAST[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static bool addPeer(const uint8_t m[6]) {
  if (esp_now_is_peer_exist(m)) return true;
  esp_now_peer_info_t p={}; memcpy(p.peer_addr,m,6); p.channel=ESPNOW_CH; p.encrypt=false;
  return esp_now_add_peer(&p)==ESP_OK;
}
static void sendNow(const uint8_t* mac, const String& s){ addPeer(mac); esp_now_send(mac,(const uint8_t*)s.c_str(), s.length()); }
static void sendBcast(const String& s){ addPeer(BCAST); esp_now_send(BCAST,(const uint8_t*)s.c_str(), s.length()); }
static void macToStr(const uint8_t m[6], char out[18]){ sprintf(out,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); }
static bool strToMac(const String& s, uint8_t m[6]){
  if (s.length()!=17) return false; int b[6];
  if (sscanf(s.c_str(),"%2x:%2x:%2x:%2x:%2x:%2x",&b[0],&b[1],&b[2],&b[3],&b[4],&b[5])!=6) return false;
  for(int i=0;i<6;i++) m[i]=(uint8_t)b[i]; return true;
}

// ---------- UI state ----------
struct Peer { uint8_t mac[6]; uint8_t idx=0; uint8_t room=0; bool tail=false; String next; uint32_t last=0; };
static std::vector<Peer> peers;
static uint8_t tailMac[6]={0};
static bool haveTail=false;

static std::unordered_map<String,uint16_t> aliasRawToId; // RAW name -> tag id (1..2048)
static std::unordered_map<uint16_t,String> regByTag;     // tag id -> reg text

// ---------- RX ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  String msg; msg.reserve(len+1); for(int i=0;i<len;i++) msg+=(char)data[i];
  uint8_t src[6]; memcpy(src, info->src_addr, 6);
  char smac[18]; macToStr(src, smac);

  if (msg.startsWith("HELLO,")){
    Peer p; memcpy(p.mac, src, 6);
    int ii=msg.indexOf("idx="), ri=msg.indexOf("room="), ti=msg.indexOf("tail="), ni=msg.indexOf("next=");
    if (ii>0){ int c=msg.indexOf(',',ii); p.idx=(uint8_t)msg.substring(ii+4, c<0?msg.length():c).toInt(); }
    if (ri>0){ int c=msg.indexOf(',',ri); p.room=(uint8_t)msg.substring(ri+5, c<0?msg.length():c).toInt(); }
    if (ti>0){ int c=msg.indexOf(',',ti); p.tail=(msg.substring(ti+5, c<0?msg.length():c).toInt()!=0); }
    if (ni>0){ int c=msg.indexOf(',',ni); p.next=msg.substring(ni+5, c<0?msg.length():c); p.next.trim(); }
    p.last=millis();
    bool found=false; for(auto& q:peers){ if(!memcmp(q.mac,src,6)){ q=p; found=true; break; } }
    if(!found) peers.push_back(p);
    if (p.tail){ memcpy(tailMac, src, 6); haveTail=true; }
    return;
  }

  if (msg.startsWith("REPORT:")){
    String payload = msg.substring(7);
    int p=0; while(p<(int)payload.length()){
      int c=payload.indexOf(',',p); if(c<0) c=payload.length();
      String tok=payload.substring(p,c); tok.trim();
      int dot=tok.indexOf('.'); int at=tok.indexOf('@');
      if (dot>0 && at>dot){
        int room = tok.substring(0,dot).toInt();
        int id   = tok.substring(dot+1,at).toInt();
        int str  = tok.substring(at+1).toInt();
        String reg = regByTag.count(id)? regByTag[id] : String("-");
        Serial.printf("[REPORT] tag=%d reg=%s room=%d s=%d\n", id, reg.c_str(), room, str);
      }
      p=c+1;
    }
    return;
  }

  if (msg.startsWith("ACK:"))  { Serial.printf("[ACK]  %s %s\n", smac, msg.c_str()); return; }
  if (msg.startsWith("INFO:")) { Serial.printf("[INFO] %s\n", msg.c_str()+5); return; }
}

// ---------- CLI ----------
static void help(){
  Serial.println(F(
    "\nUI CLI:\n"
    "  peers                         - list nodes\n"
    "  chain MAC1,MAC2,...           - linear chain; idx 1..N, next hops, tail=last, slots=N\n"
    "  tail  <MAC>                   - set tail on node; demote others\n"
    "  room  <MAC> <N>               - set room number (0..255)\n"
    "  idx   <MAC> <I>               - set chain index (1..N)\n"
    "  next  <MAC> <NEXTMAC>         - set upstream hop\n"
    "  slots <N>                     - broadcast total slots for TDMA\n"
    "  map   <RAW_NAME> <TAG_ID>     - RAW->ID (1..2048); broadcast\n"
    "  reg   <TAG_ID>    <REG_TEXT>  - label for printing (UI only)\n"
    "  ping                          - nodes reply INFO:ok\n"
  ));
}
static void listPeers(){
  Serial.println(F("Peers:"));
  for(const auto& p:peers){
    char m[18]; macToStr(p.mac,m);
    Serial.printf("  %s  idx=%u room=%u tail=%u next=%s ago=%lus\n",
      m,p.idx,p.room,p.tail?1:0,p.next.c_str(),(unsigned)((millis()-p.last)/1000));
  }
}
static void cfgKV_target(const uint8_t* mac, const String& kv){ sendNow(mac, "CFG:"+kv); }
static void bcastKV(const String& kv){ sendBcast("CFG:"+kv); }

static void doChain(const String& csv){
  std::vector<std::array<uint8_t,6>> list;
  int p=0; while(p<(int)csv.length()){
    int c=csv.indexOf(',',p); if(c<0) c=csv.length();
    String t=csv.substring(p,c); t.trim(); if(t.length()){ std::array<uint8_t,6>a; if(strToMac(t,a.data())) list.push_back(a); }
    p=c+1;
  }
  if(list.empty()){ Serial.println("chain: no valid MACs"); return; }
  for(size_t i=0;i<list.size();++i){
    char nxt[18]; if(i+1<list.size()) macToStr(list[i+1].data(),nxt); else strcpy(nxt,"00:00:00:00:00:00");
    cfgKV_target(list[i].data(), "idx="+String((int)(i+1)));
    cfgKV_target(list[i].data(), String("next=")+nxt);
  }
  bcastKV("slots="+String((int)list.size()));
  memcpy(tailMac, list.back().data(),6); haveTail=true;
  cfgKV_target(tailMac, "tail=1"); bcastKV("tail=0");
  Serial.printf("chain: %u nodes, tail set to last, slots broadcast\n",(unsigned)list.size());
}

static void handleCLI(const String& line){
  if (!line.length()) return;
  std::vector<String> v; int p=0; while(p<(int)line.length()){ int s=line.indexOf(' ',p); if(s<0)s=line.length(); String t=line.substring(p,s); if(t.length()) v.push_back(t); p=s+1; }
  if (v.empty()) return;

  if (v[0]=="help"){ help(); return; }
  if (v[0]=="peers"){ listPeers(); return; }
  if (v[0]=="ping"){ sendBcast("PING?"); return; }

  if (v[0]=="tail" && v.size()>=2){
    if (!strToMac(v[1], tailMac)){ Serial.println("bad MAC"); return; }
    haveTail=true; cfgKV_target(tailMac,"tail=1"); bcastKV("tail=0"); Serial.println("tail set"); return;
  }
  if (v[0]=="room" && v.size()>=3){
    uint8_t m[6]; if(!strToMac(v[1],m)){ Serial.println("bad MAC"); return; }
    cfgKV_target(m, "room="+String(v[2].toInt())); return;
  }
  if (v[0]=="idx" && v.size()>=3){
    uint8_t m[6]; if(!strToMac(v[1],m)){ Serial.println("bad MAC"); return; }
    cfgKV_target(m, "idx="+String(v[2].toInt())); return;
  }
  if (v[0]=="next" && v.size()>=3){
    uint8_t m[6],n[6]; if(!strToMac(v[1],m)||!strToMac(v[2],n)){ Serial.println("bad MAC"); return; }
    char nxt[18]; macToStr(n,nxt); cfgKV_target(m, String("next=")+nxt); return;
  }
  if (v[0]=="slots" && v.size()>=2){ bcastKV("slots="+String(v[1].toInt())); Serial.println("slots broadcast"); return; }

  if (v[0]=="map" && v.size()>=3){
    uint16_t id=(uint16_t)v[2].toInt(); aliasRawToId[v[1]]=id;
    String msg=String("MAP,name=")+v[1]+",id="+String(id);
    if (haveTail) sendNow(tailMac,msg); else sendBcast(msg);
    Serial.println("map saved+broadcast"); return;
  }
  if (v[0]=="reg" && v.size()>=3){
    uint16_t id=(uint16_t)v[1].toInt(); String rs=v[2]; regByTag[id]=rs;
    String msg=String("REG,id=")+String(id)+",reg="+rs;
    if (haveTail) sendNow(tailMac,msg); else sendBcast(msg);
    Serial.println("reg saved+broadcast"); return;
  }
  if (v[0]=="chain" && v.size()>=2){ doChain(v[1]); return; }

  Serial.println("unknown cmd; try 'help'");
}

// ---------- setup/loop ----------
void setup(){
  Serial.begin(115200); delay(50);
  WiFi.mode(WIFI_STA); esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); delay(3000); ESP.restart(); }
  esp_now_register_recv_cb(onRecv); addPeer(BCAST);
  Serial.println("\n[UI] ready. Type 'help'.");
}
void loop(){
  static String buf;
  while(Serial.available()){
    char c=(char)Serial.read();
    if (c=='\n'||c=='\r'){ buf.trim(); if(buf.length()) handleCLI(buf); buf=""; }
    else buf+=c;
  }
  static uint32_t last=0; if (millis()-last>2000){ last=millis(); sendBcast("HELLO?"); }
}
