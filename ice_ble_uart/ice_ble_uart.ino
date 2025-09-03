/*
  SMOKE Unified (ESP32-S3) — Receiver/Bridge v0
  - Single firmware for all receivers (flash-and-go)
  - Today: standalone "tail" mode (USB output) + config over USB + SPIFFS persistence
  - Input: COBS-framed ICE messages on UART1 (RX=16, TX=17)
  - Output: compact "room.device@rssi" lines on USB at END_OF_SCAN, strongest RSSI per device

  Next steps (same firmware; config-driven):
  - ESP-NOW upstream/downstream with fragment+ACK to chain R1->R2->R3
*/

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>

// ---- UART1 for ICE ----
#define ICE_UART_BAUD  921600
#define ICE_UART_RX    16
#define ICE_UART_TX    17
#define ICE_UART_RXBUF 8192
#define ICE_UART_TXBUF 2048

// ---- Message format from ICE ----
// VER(1) TYPE(1) ORIGIN(6) BATCH(4) SEQ(2) COUNT(2) PAYLOAD(...) CRC16(2)
// Device TLVs in PAYLOAD: T=1 MAC(6), T=2 RSSI(1,i8), T=3 FLAGS(1), T=4 NAME(n)
enum MsgType : uint8_t { MT_DATA=0, MT_END=1 };

// ---- Config persistence ----
static uint8_t  g_room = 1;   // 1..255
static bool     g_is_tail = true;

// Maps: BLE MAC -> device number; device number -> friendly name
#include <map>
#include <unordered_map>

struct MacKey {
  uint8_t b[6];
  bool operator==(const MacKey& o) const {
    for (int i=0;i<6;i++) if (b[i]!=o.b[i]) return false; return true;
  }
};
struct MacHash {
  size_t operator()(const MacKey& k) const {
    // simple 48-bit fold
    uint32_t h = (k.b[0]<<24) ^ (k.b[1]<<16) ^ (k.b[2]<<8) ^ k.b[3] ^ (k.b[4]<<4) ^ k.b[5];
    return h;
  }
};
static std::unordered_map<MacKey, uint16_t, MacHash> mac2dev;
static std::map<uint16_t, String> devNames;

// Per-scan best RSSI by device number
static std::unordered_map<uint16_t, int8_t> bestRssi;

// ---- Utilities ----
static void macToStr(const uint8_t* m, char out[18]) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X", m[0],m[1],m[2],m[3],m[4],m[5]);
}
static bool parseMac(const String& s, uint8_t out[6]) {
  if (s.length()!=17) return false;
  int vals[6];
  if (sscanf(s.c_str(), "%x:%x:%x:%x:%x:%x", &vals[0],&vals[1],&vals[2],&vals[3],&vals[4],&vals[5])!=6) return false;
  for(int i=0;i<6;i++) out[i] = (uint8_t)vals[i];
  return true;
}

// ---- CRC16-CCITT ----
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i=0;i<8;i++) crc = (crc & 0x8000) ? ((crc<<1) ^ 0x1021) : (crc<<1);
  }
  return crc;
}

// ---- COBS decode (returns decoded length, or 0 on error) ----
static size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out, size_t outMax) {
  if (!len) return 0;
  const uint8_t* ip = in;
  const uint8_t* end = in + len;
  uint8_t* op = out;
  while (ip < end) {
    uint8_t code = *ip++;
    if (code == 0 || ip + (code-1) > end) return 0;
    for (uint8_t i=1; i<code; ++i) {
      if ((size_t)(op - out) >= outMax) return 0;
      *op++ = *ip++;
    }
    if (code != 0xFF && ip < end) {
      if ((size_t)(op - out) >= outMax) return 0;
      *op++ = 0;
    }
  }
  return (size_t)(op - out);
}

// ---- SPIFFS I/O ----
static void saveRoom() {
  File f = SPIFFS.open("/room.txt", FILE_WRITE, true);
  if (f) { f.printf("%u\n", (unsigned)g_room); f.close(); }
}
static void loadRoom() {
  File f = SPIFFS.open("/room.txt", FILE_READ);
  if (f) { g_room = (uint8_t)f.readStringUntil('\n').toInt(); f.close(); if (g_room==0) g_room=1; }
}
static void saveTail() {
  File f = SPIFFS.open("/tail.txt", FILE_WRITE, true);
  if (f) { f.printf("%d\n", g_is_tail?1:0); f.close(); }
}
static void loadTail() {
  File f = SPIFFS.open("/tail.txt", FILE_READ);
  if (f) { g_is_tail = (f.readStringUntil('\n').toInt()!=0); f.close(); }
}
static void saveBinds() {
  File f = SPIFFS.open("/binds.csv", FILE_WRITE, true);
  if (!f) return;
  for (auto& kv : mac2dev) {
    char mac[18]; macToStr(kv.first.b, mac);
    f.printf("%s,%u\n", mac, (unsigned)kv.second);
  }
  f.close();
}
static void loadBinds() {
  mac2dev.clear();
  File f = SPIFFS.open("/binds.csv", FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (!line.length()) continue;
    int comma = line.indexOf(',');
    if (comma<0) continue;
    String smac = line.substring(0, comma);
    String snum = line.substring(comma+1);
    uint8_t m[6]; if (!parseMac(smac, m)) continue;
    MacKey k; memcpy(k.b, m, 6);
    mac2dev[k] = (uint16_t)snum.toInt();
  }
  f.close();
}
static void saveNames() {
  File f = SPIFFS.open("/names.csv", FILE_WRITE, true);
  if (!f) return;
  for (auto& kv : devNames) {
    f.printf("%u,%s\n", (unsigned)kv.first, kv.second.c_str());
  }
  f.close();
}
static void loadNames() {
  devNames.clear();
  File f = SPIFFS.open("/names.csv", FILE_READ);
  if (!f) return;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (!line.length()) continue;
    int comma = line.indexOf(',');
    if (comma<0) continue;
    uint16_t id = (uint16_t) line.substring(0, comma).toInt();
    String nm = line.substring(comma+1);
    devNames[id] = nm;
  }
  f.close();
}

// ---- Command parser over USB ----
static String inLine;
static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  setroom <n>           - set this receiver's room number (1..255)"));
  Serial.println(F("  tail on|off           - mark this receiver as tail (USB output)"));
  Serial.println(F("  bind <MAC> <num>      - map BLE MAC to device number (e.g. bind AA:BB:CC:DD:EE:FF 12)"));
  Serial.println(F("  name <num> <text...>  - set friendly device name"));
  Serial.println(F("  show room|tail|binds|names"));
  Serial.println(F("  save                  - persist binds & names (room/tail save on set)"));
  Serial.println(F("  help"));
}
static void handleCommand(const String& s) {
  // split first token
  int sp = s.indexOf(' ');
  String cmd = (sp<0) ? s : s.substring(0,sp);
  String rest = (sp<0) ? "" : s.substring(sp+1);

  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") {
    printHelp();
  } else if (cmd == "setroom") {
    uint16_t n = (uint16_t)rest.toInt();
    if (n==0 || n>255) { Serial.println(F("ERR: room must be 1..255")); return; }
    g_room = (uint8_t)n; saveRoom();
    Serial.printf("OK room=%u\n", g_room);
  } else if (cmd == "tail") {
    rest.toLowerCase();
    if (rest=="on" || rest=="1" || rest=="true") g_is_tail = true;
    else if (rest=="off" || rest=="0" || rest=="false") g_is_tail = false;
    else { Serial.println(F("ERR: usage tail on|off")); return; }
    saveTail();
    Serial.printf("OK tail=%d\n", (int)g_is_tail);
  } else if (cmd == "bind") {
    int sp2 = rest.indexOf(' ');
    if (sp2<0) { Serial.println(F("ERR: bind <MAC> <num>")); return; }
    String smac = rest.substring(0, sp2);
    String snum = rest.substring(sp2+1);
    uint8_t m[6];
    if (!parseMac(smac, m)) { Serial.println(F("ERR: bad MAC")); return; }
    uint16_t id = (uint16_t)snum.toInt();
    MacKey k; memcpy(k.b, m, 6);
    mac2dev[k] = id;
    Serial.printf("OK bind %s -> %u\n", smac.c_str(), (unsigned)id);
  } else if (cmd == "name") {
    int sp2 = rest.indexOf(' ');
    if (sp2<0) { Serial.println(F("ERR: name <num> <text...>")); return; }
    uint16_t id = (uint16_t)rest.substring(0, sp2).toInt();
    String nm = rest.substring(sp2+1);
    devNames[id] = nm;
    Serial.printf("OK name %u=\"%s\"\n", (unsigned)id, nm.c_str());
  } else if (cmd == "save") {
    saveBinds(); saveNames();
    Serial.println(F("OK saved"));
  } else if (cmd == "show") {
    rest.toLowerCase();
    if (rest=="room") Serial.printf("room=%u\n", g_room);
    else if (rest=="tail") Serial.printf("tail=%d\n", (int)g_is_tail);
    else if (rest=="binds") {
      for (auto& kv : mac2dev) {
        char mac[18]; macToStr(kv.first.b, mac);
        Serial.printf("%s -> %u\n", mac, (unsigned)kv.second);
      }
    } else if (rest=="names") {
      for (auto& kv : devNames) {
        Serial.printf("%u,%s\n", (unsigned)kv.first, kv.second.c_str());
      }
    } else Serial.println(F("ERR: show what? room|tail|binds|names"));
  } else if (cmd.length()) {
    Serial.println(F("ERR: unknown. Type 'help'."));
  }
}

// ---- UART1/COBS frame reader from ICE ----
static const size_t FRAME_MAX = 1600;
static uint8_t frameBuf[FRAME_MAX];
static size_t  frameLen = 0;

static const size_t DECODE_MAX = 1600;
static uint8_t decodeBuf[DECODE_MAX];

static void processDecodedMessage(const uint8_t* m, size_t n);

// read bytes, split by 0x00 delimiter, then cobs-decode and process
static void pollICE() {
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (b == 0x00) {
      if (frameLen > 0) {
        size_t dec = cobs_decode(frameBuf, frameLen, decodeBuf, DECODE_MAX);
        if (dec > 0) processDecodedMessage(decodeBuf, dec);
        frameLen = 0;
      }
    } else {
      if (frameLen < FRAME_MAX) frameBuf[frameLen++] = b;
      else frameLen = 0; // overflow guard: reset
    }
  }
}

// ---- Parse ICE message and collect strongest RSSI per device ----
static uint32_t curBatch = 0;
static uint32_t lastSeenBatch = 0;

static void endOfScanFlush() {
  // Print compact lines: room.device@rssi
  if (!g_is_tail) {
    // In the next version we'll forward upstream here via ESP-NOW.
    // For now still print if configured tail=false to let you test.
  }
  for (auto &kv : bestRssi) {
    uint16_t dev = kv.first;
    int8_t rssi = kv.second;
    Serial.printf("%u.%u@%d\n", (unsigned)g_room, (unsigned)dev, (int)rssi);
  }
  bestRssi.clear();
  Serial.println("--- END OF SCAN ---");
}

static void processDecodedMessage(const uint8_t* m, size_t n) {
  if (n < 1+1+6+4+2+2+2) return; // min header + CRC
  uint8_t ver = m[0];
  uint8_t typ = m[1];
  (void)ver;
  // origin(6) at m+2 (we don't need it for tail mode)
  const uint8_t* origin = m+2;
  uint32_t batch = (uint32_t)m[8] | ((uint32_t)m[9]<<8) | ((uint32_t)m[10]<<16) | ((uint32_t)m[11]<<24);
  uint16_t seq   = (uint16_t)m[12] | ((uint16_t)m[13]<<8);
  uint16_t count = (uint16_t)m[14] | ((uint16_t)m[15]<<8);

  size_t pay_off = 1+1+6+4+2+2;
  size_t pay_len = n - pay_off - 2;
  if ((int)pay_len < 0) return;

  // CRC check
  uint16_t given = (uint16_t)m[n-2] | ((uint16_t)m[n-1]<<8);
  uint16_t calc  = crc16_ccitt(m, n-2);
  if (given != calc) return;

  if (typ == MT_END) {
    endOfScanFlush();
    lastSeenBatch = batch;
    return;
  }

  // DATA: iterate TLVs for each device record
  size_t p = pay_off;
  for (uint16_t i=0; i<count; ++i) {
    // Expect T1 MAC(6), T2 RSSI(1), T3 FLAGS(1), optional T4 NAME
    if (p+2 > n-2) break;
    uint8_t T = m[p++], L = m[p++];
    if (!(T==1 && L==6) || p+6 > n-2) break;
    MacKey mk; memcpy(mk.b, &m[p], 6); p += 6;

    if (p+2 > n-2) break;
    T = m[p++]; L = m[p++];
    if (!(T==2 && L==1) || p+1 > n-2) break;
    int8_t rssi = (int8_t)m[p++];

    if (p+2 > n-2) break;
    T = m[p++]; L = m[p++];
    if (!(T==3 && L==1) || p+1 > n-2) break;
    uint8_t flags = m[p++]; (void)flags;

    // optional NAME
    if (p+2 <= n-2 && m[p]==4) {
      T = m[p++]; L = m[p++];
      if (p+L <= n-2) p += L; // skip
    }

    // Map MAC -> device number
    uint16_t devNum = 0xFFFF; // unknown
    auto it = mac2dev.find(mk);
    if (it != mac2dev.end()) devNum = it->second;

    if (devNum != 0xFFFF) {
      auto br = bestRssi.find(devNum);
      if (br == bestRssi.end() || rssi > br->second) {
        bestRssi[devNum] = rssi; // keep strongest
      }
    } else {
      // Unknown devices are ignored for compact mode.
      // (We can add a flag later to show them as 0.<hash>@rssi or similar.)
    }
  }
}

// ---- Setup/Loop ----
void setup() {
  Serial.begin(115200);
  delay(50);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  } else {
    loadRoom();
    loadTail();
    loadBinds();
    loadNames();
  }
  Serial.printf("SMOKE ready. room=%u tail=%d\n", (unsigned)g_room, (int)g_is_tail);
  printHelp();

  Serial1.setRxBufferSize(ICE_UART_RXBUF);
  Serial1.setTxBufferSize(ICE_UART_TXBUF);
  Serial1.begin(ICE_UART_BAUD, SERIAL_8N1, ICE_UART_RX, ICE_UART_TX);
}

static uint32_t lastCmdPoll = 0;
void loop() {
  // Poll ICE UART for frames
  pollICE();

  // Read user commands on USB
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) handleCommand(s);
  }

  // (Future) ESP-NOW poll here
}
