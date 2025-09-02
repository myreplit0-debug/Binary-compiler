#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>

#define ESPNOW_CH        6
#define ICE_RX1          16
#define ICE_TX1          17
#define MAX_TAG_ID       1024
#define HELLO_PERIOD_MS  2000
#define FLUSH_PERIOD_MS  600
#define ICE_READ_BUDGET  48

Preferences prefs;
bool     isTail  = false;
uint8_t  roomNo  = 0;
uint8_t  nextMac[6] = {0};

HardwareSerial &SICE = Serial1;
String iceLine;

static uint8_t  bestStr[MAX_TAG_ID + 1];
static uint32_t lastHello = 0;
static uint32_t lastFlush = 0;

// ---------- utils ----------
static inline void macToStr(const uint8_t mac[6], char out[18]) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
static inline bool strToMac(const String &s, uint8_t mac[6]) {
  if (s.length() != 17) return false;
  int b[6];
  if (sscanf(s.c_str(), "%2x:%2x:%2x:%2x:%2x:%2x",
             &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)b[i];
  return true;
}
static inline uint8_t rssiToStrength(int rssi) {
  if (rssi > -30) rssi = -30;
  if (rssi < -100) rssi = -100;
  int v = (rssi + 100) * (100.0 / 70.0) + 0.5;
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (uint8_t)v;
}
static inline bool macIsZero(const uint8_t mac[6]) {
  for (int i=0;i<6;i++) if (mac[i]) return false;
  return true;
}

// ---------- prefs ----------
void loadPrefs() {
  prefs.begin("smoke", true);
  isTail = prefs.getBool("tail", false);
  roomNo = prefs.getUChar("room", 0);
  String ns = prefs.getString("next", "");
  prefs.end();
  if (ns.length()==17) strToMac(ns, nextMac);
  else memset(nextMac, 0, 6);
}
void savePrefs() {
  char nbuf[18]; macToStr(nextMac, nbuf);
  prefs.begin("smoke", false);
  prefs.putBool("tail", isTail);
  prefs.putUChar("room", roomNo);
  prefs.putString("next", nbuf);
  prefs.end();
}

// ---------- esp-now ----------
static uint8_t bcastAddr[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void sendNow(const uint8_t* mac, const char* s) {
  esp_now_send(mac, (const uint8_t*)s, strlen(s));
}
static bool addPeer(const uint8_t mac[6]) {
  if (macIsZero(mac)) return false;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = ESPNOW_CH;
  p.encrypt = false;
  if (esp_now_is_peer_exist(mac)) return true;
  return esp_now_add_peer(&p) == ESP_OK;
}
static void ensurePeers() {
  addPeer(bcastAddr);
  if (!macIsZero(nextMac)) addPeer(nextMac);
}

// ---------- hello ----------
static void sendHello() {
  uint8_t my[6]; WiFi.macAddress(my);
  char mstr[18]; macToStr(my, mstr);
  char nstr[18]; macToStr(nextMac, nstr);
  char buf[128];
  snprintf(buf, sizeof(buf),
           "HELLO,mac=%s,room=%u,tail=%u,next=%s",
           mstr, (unsigned)roomNo, (unsigned)(isTail ? 1 : 0), nstr);
  sendNow(bcastAddr, buf);
}

// ---------- aggregation ----------
static void forwardAggOrPrint() {
  String out; out.reserve(256);
  out += "AGG:";
  bool first = true;
  for (int i=1;i<=MAX_TAG_ID;i++) {
    uint8_t s = bestStr[i];
    if (s) {
      if (!first) out += ',';
      first = false;
      out += String((int)roomNo); out += '.';
      out += String(i); out += '@';
      out += String((int)s);
    }
  }

  if (isTail) {
    if (out.length() > 4) {
      String ui = "UI: " + out.substring(4);
      SICE.println(ui);
    }
  } else if (!macIsZero(nextMac)) {
    ensurePeers();
    sendNow(nextMac, out.c_str());
  }

  for (int i=1;i<=MAX_TAG_ID;i++) {
    if (bestStr[i]) bestStr[i] = (bestStr[i] > 2) ? bestStr[i]-2 : 0;
  }
}

// ---------- uart ----------
static bool parseTagLine(const String& line, int &tag, int &rssi) {
  if (line.length() < 5) return false;
  if (line[0]=='T') {
    int a=line.indexOf(','); if (a<0) return false;
    int b=line.indexOf(',', a+1); if (b<0) return false;
    tag = line.substring(a+1, b).toInt();
    rssi= line.substring(b+1).toInt();
    return tag>0;
  } else if (line.startsWith("TAG ")) {
    int a=line.indexOf(' ',4); if (a<0) return false;
    tag = line.substring(4, a).toInt();
    rssi= line.substring(a+1).toInt();
    return tag>0;
  }
  return false;
}
static void pumpUART1() {
  int budget = ICE_READ_BUDGET;
  while (budget-- > 0 && SICE.available()) {
    char c = (char)SICE.read();
    if (c=='\n') {
      String s = iceLine; iceLine = ""; s.trim();
      if (!s.length()) continue;

      if (isTail) {
        if (s.startsWith("CFG:")) { ensurePeers(); sendNow(bcastAddr, s.c_str()); }
      } else {
        int tag, rssi;
        if (parseTagLine(s, tag, rssi)) {
          uint8_t st = rssiToStrength(rssi);
          if (tag>=1 && tag<=MAX_TAG_ID && st>bestStr[tag]) bestStr[tag] = st;
        }
      }
    } else if (c!='\r') {
      iceLine += c;
      if (iceLine.length() > 256) iceLine.remove(0, 64);
    }
  }
}

// ---------- callbacks ----------
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  if (!data || len<=0) return;

  String msg; msg.reserve(len+1);
  for (int i=0;i<len;i++) msg += (char)data[i];

  if (msg.startsWith("CFG:")) {
    int macPos = msg.indexOf("mac=");
    if (macPos >= 0) {
      String m = msg.substring(macPos+4);
      int comma = m.indexOf(','); if (comma>0) m = m.substring(0, comma);
      m.trim();
      uint8_t tmac[6];
      if (strToMac(m, tmac)) {
        uint8_t my[6]; WiFi.macAddress(my);
        if (memcmp(tmac, my, 6) != 0) return;
      }
    }
    int p = 4;
    while (p < msg.length()) {
      int c = msg.indexOf(',', p); if (c<0) c = msg.length();
      String kv = msg.substring(p, c); kv.trim();
      int eq = kv.indexOf('=');
      String k = (eq>0)? kv.substring(0,eq) : kv;
      String v = (eq>0)? kv.substring(eq+1) : "1";
      if      (k=="room")  { int r=v.toInt(); if (r>=0 && r<=255) roomNo=(uint8_t)r; }
      else if (k=="tail")  { isTail = (v.toInt()!=0); }
      else if (k=="next")  { if (!strToMac(v, nextMac)) memset(nextMac,0,6); }
      else if (k=="clear") { isTail=false; roomNo=0; memset(nextMac,0,6); }
      p = c+1;
    }
    savePrefs();
    return;
  }

  if (msg.startsWith("AGG:")) {
    int p = 4;
    while (p < msg.length()) {
      int c = msg.indexOf(',', p); if (c<0) c = msg.length();
      String tok = msg.substring(p, c); tok.trim();
      int dot = tok.indexOf('.'); int at = tok.indexOf('@');
      if (dot>0 && at>dot) {
        int tag = tok.substring(dot+1,at).toInt();
        int str = tok.substring(at+1).toInt();
        if (tag>=1 && tag<=MAX_TAG_ID) {
          if (str<0) str=0; if (str>100) str=100;
          if (str>bestStr[tag]) bestStr[tag] = (uint8_t)str;
        }
      }
      p = c+1;
    }
  }
}

// *** send callback expected by your core: 2 args, first is wifi_tx_info_t* ***
static void onSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info; (void)status;
}

// ---------- setup/loop ----------
void setup() {
  Serial.begin(115200);
  SICE.begin(115200, SERIAL_8N1, ICE_RX1, ICE_TX1);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CH, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); delay(3000); ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  loadPrefs();
  ensurePeers();
  memset(bestStr, 0, sizeof(bestStr));

  uint8_t me[6]; WiFi.macAddress(me);
  char macs[18]; macToStr(me, macs);
  Serial.printf("[SMOKE] boot mac=%s room=%u tail=%u\n", macs, roomNo, isTail?1:0);
}

void loop() {
  pumpUART1();
  uint32_t now = millis();
  if (now - lastHello >= HELLO_PERIOD_MS) { lastHello = now; sendHello(); }
  if (now - lastFlush >= FLUSH_PERIOD_MS) { lastFlush = now; forwardAggOrPrint(); }
}
