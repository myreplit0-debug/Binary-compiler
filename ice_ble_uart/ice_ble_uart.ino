/*
  SMOKE v3 — Room Chain (ESP32-S3)  (strongest-per-scan, multi-room winners)

  - UART1 (GPIO16 RX, GPIO17 TX)
    * Non-tail: read "NAME@RSSI" lines from ICE and track best per device id
    * Tail: write final results to UART1 for the UI
  - ESP-NOW compact binary chain: strongest-per-device bubbles up 1->2->3...
  - Strongest-per-scan is enforced via batch id:
      • Batch starts at Room 1 on ICE “END” (or timeout) and increments g_batch
      • Rooms 2..N reset per-scan tables when they see a new batch id
      • Tail prints once per batch when the last fragment for that batch arrives
  - Slot delay per room to reduce collisions
*/

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <map>
#include <unordered_map>
#include <vector>
#include <algorithm>

// ======================= CONFIG =======================
#define UART_BAUD_UI_ICE 921600
#define UART_RX_PIN      16
#define UART_TX_PIN      17
#define UART_RXBUF       16384
#define UART_TXBUF       2048

// ESPNOW payload max on ESP32 is 250 bytes
#define ESN_PAYLOAD_MAX   250
#define ESN_FRAG_DATA_MAX 200

// Chain timing to avoid collisions
#define SLOT_MS           80     // ~80ms per room; tune as needed
#define MERGE_DELAY_MS    10     // short delay before merge/forward

// ======================= STATE ========================
static uint8_t  g_room    = 1;     // 1..255
static bool     g_is_tail = false; // tail -> prints final, doesn’t forward
static bool     g_esn_on  = true;
static uint8_t  g_channel = 6;

// Debug toggles
static bool DBG_HEX   = false;
static bool DBG_PARSE = true;
static bool DBG_NAMES = true;

// Batch id (originates in room 1 on each ICE “END”)
static uint32_t g_batch  = 1;

// RX-side batch tracking (for reset/print gating)
static uint32_t g_rx_batch  = 0;   // current batch we’re accumulating
static bool     g_rx_printed = false; // tail: printed winners for g_rx_batch

// ===================== DATA STRUCTS ====================
struct MacKey {
  uint8_t b[6];
  bool operator==(const MacKey& o) const {
    for (int i=0;i<6;i++) if (b[i]!=o.b[i]) return false;
    return true;
  }
};
struct MacHash {
  size_t operator()(const MacKey& k) const {
    uint32_t h=(k.b[0]<<24)^(k.b[1]<<16)^(k.b[2]<<8)^k.b[3]^(k.b[4]<<4)^k.b[5];
    return h;
  }
};

// name -> device id (1..1000)
static std::map<String, uint16_t> name2dev;

// strongest in THIS room (source is ICE lines)
static std::map<uint16_t, int8_t> localBest;  // dev -> rssi

// strongest so far in the chain that we’ll forward
struct BestVal { int8_t rssi; uint8_t room; };
static std::map<uint16_t, BestVal> chainBest; // dev -> {rssi, room}

// “seen” names (sanitized) to help binding from USB
static std::vector<String> seenNames; size_t g_seen_max = 128;

// ESPNOW peer (broadcast)
static uint8_t ESN_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF};

// ===================== PACKING FORMAT ==================
struct __attribute__((packed)) EsnHdr {
  uint8_t  magic0;     // 'S'
  uint8_t  magic1;     // 'M'
  uint8_t  ver;        // 2
  uint8_t  src_room;   // who sends this fragment
  uint8_t  total;      // total fragments
  uint8_t  index;      // 0..total-1
  uint8_t  count;      // # entries in this fragment
  uint32_t batch;      // chain batch id
  // then entries...
};
struct __attribute__((packed)) Entry {
  uint16_t dev;
  int8_t   rssi;
  uint8_t  room;
};

// ======================= UTILS =========================
static String trimQuotes(String s){
  s.trim();
  if (s.length()>=2 && ((s.startsWith("\"")&&s.endsWith("\""))||(s.startsWith("'")&&s.endsWith("'"))))
    return s.substring(1, s.length()-1);
  return s;
}
// ASCII, trim, collapse spaces
static String sanitizePrefix(const String& in){
  String out; out.reserve(in.length());
  for (size_t i=0;i<in.length(); ++i){
    uint8_t c=(uint8_t)in[i];
    if (c<32 || c>126) break;
    out += (char)c;
  }
  out.trim();
  String collapsed; collapsed.reserve(out.length()); bool lastSpace=false;
  for (size_t i=0;i<out.length(); ++i){
    char c=out[i];
    if (c==' '||c=='t'){ if(!lastSpace){ collapsed+=' '; lastSpace=true; } }
    else { collapsed+=c; lastSpace=false; }
  }
  return collapsed;
}
static String sanitizeForCsv(const String& s){ String o=s; o.replace(","," "); o.replace("\r"," "); o.replace("\n"," "); return o; }

static void addSeen(const String& san){
  if (!san.length()) return;
  for (size_t i=0;i<seenNames.size(); ++i){
    if (seenNames[i]==san){
      if (i!=0){ seenNames.erase(seenNames.begin()+i); seenNames.insert(seenNames.begin(), san); }
      return;
    }
  }
  seenNames.insert(seenNames.begin(), san);
  if (seenNames.size()>g_seen_max) seenNames.pop_back();
}

static void dumpHex(const char* tag, const uint8_t* p, size_t n){
  if (!DBG_HEX) return;
  Serial.printf("%s len=%u\n", tag, (unsigned)n);
  const size_t cols=16;
  for (size_t i=0;i<n;i+=cols){
    Serial.printf("  %04u: ", (unsigned)i);
    for (size_t j=0;j<cols && i+j<n; ++j) Serial.printf("%02X ", p[i+j]);
    Serial.print(" | ");
    for (size_t j=0;j<cols && i+j<n; ++j){
      char c = (char)p[i+j];
      Serial.print((c>=32 && c<=126) ? c : '.');
    }
    Serial.println();
  }
}

// =================== PERSISTENCE (SPIFFS) ==============
static void saveRoom(){ File f=SPIFFS.open("/room.txt", FILE_WRITE, true); if(f){ f.printf("%u\n",(unsigned)g_room); f.close(); } }
static void loadRoom(){ File f=SPIFFS.open("/room.txt", FILE_READ); if(f){ g_room=(uint8_t)f.readStringUntil('\n').toInt(); f.close(); if(!g_room) g_room=1; } }
static void saveTail(){ File f=SPIFFS.open("/tail.txt", FILE_WRITE, true); if(f){ f.printf("%d\n", g_is_tail?1:0); f.close(); } }
static void loadTail(){ File f=SPIFFS.open("/tail.txt", FILE_READ); if(f){ g_is_tail=(f.readStringUntil('\n').toInt()!=0); f.close(); } }

static void saveEsn(){ File f=SPIFFS.open("/esn.txt", FILE_WRITE, true); if(f){ f.printf("%d,%u\n", g_esn_on?1:0, (unsigned)g_channel); f.close(); } }
static void loadEsn(){ File f=SPIFFS.open("/esn.txt", FILE_READ); if(f){ String line=f.readStringUntil('\n'); f.close(); int comma=line.indexOf(','); if(comma>0){ g_esn_on = line.substring(0,comma).toInt()!=0; uint16_t ch=line.substring(comma+1).toInt(); if(ch>=1 && ch<=13) g_channel=(uint8_t)ch; } } }

static void saveNameBinds(){ File f=SPIFFS.open("/namebinds.csv", FILE_WRITE, true); if(!f) return;
  for(auto& kv: name2dev) f.printf("%s,%u\n", sanitizeForCsv(kv.first).c_str(), (unsigned)kv.second); f.close(); }
static void loadNameBinds(){ name2dev.clear(); File f=SPIFFS.open("/namebinds.csv", FILE_READ); if(!f) return;
  while(f.available()){ String line=f.readStringUntil('\n'); line.trim(); if(!line.length()) continue;
    int c=line.lastIndexOf(','); if(c<0) continue; String nm=line.substring(0,c); uint16_t id=(uint16_t)line.substring(c+1).toInt(); name2dev[nm]=id; }
  f.close(); }

// =================== USB COMMANDS ======================
static void printHelp(){
  Serial.println(F("Commands:"));
  Serial.println(F("  setroom <n>              ; set this receiver's room number (1..255)"));
  Serial.println(F("  tail on|off              ; make this receiver the tail (prints final)"));
  Serial.println(F("  esn on|off               ; enable/disable ESP-NOW"));
  Serial.println(F("  channel <1..13>          ; set Wi-Fi channel (all nodes must match)"));
  Serial.println(F("  bindname <name...> <num> ; map device name (sanitized) to device number"));
  Serial.println(F("  seen                     ; list recent sanitized names"));
  Serial.println(F("  bindseen <idx> <num>     ; bind a seen name to number"));
  Serial.println(F("  show room|tail|esn|channel|namebinds"));
  Serial.println(F("  save                     ; persist namebinds and settings"));
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
  else if(cmd=="bindname"){
    int spLast=rest.lastIndexOf(' '); if(spLast<0){ Serial.println(F("ERR: bindname <name...> <num>")); return; }
    String raw = trimQuotes(rest.substring(0, spLast));
    uint16_t id = (uint16_t)rest.substring(spLast+1).toInt(); if(!id){ Serial.println(F("ERR: invalid number")); return; }
    String san = sanitizePrefix(raw); if(!san.length()){ Serial.println(F("ERR: sanitized name empty")); return; }
    name2dev[san] = id;
    Serial.printf("OK bindname RAW=\"%s\" SAN=\"%s\" -> %u\n", raw.c_str(), san.c_str(), (unsigned)id);
  }
  else if(cmd=="seen"){
    for (size_t i=0; i<seenNames.size(); ++i) Serial.printf("[%u] %s\n", (unsigned)i, seenNames[i].c_str());
  }
  else if(cmd=="bindseen"){
    int sp2=rest.indexOf(' '); if(sp2<0){ Serial.println(F("ERR: bindseen <idx> <num>")); return; }
    int idx = rest.substring(0, sp2).toInt();
    uint16_t id = (uint16_t)rest.substring(sp2+1).toInt();
    if (idx < 0 || (size_t)idx >= seenNames.size()){ Serial.println(F("ERR: idx out of range")); return; }
    String san = seenNames[(size_t)idx];
    name2dev[san] = id;
    Serial.printf("OK bindseen [%d] \"%s\" -> %u\n", idx, san.c_str(), (unsigned)id);
  }
  else if(cmd=="show"){
    String r=rest; r.toLowerCase();
    if(r=="room") Serial.printf("room=%u\n", g_room);
    else if(r=="tail") Serial.printf("tail=%d\n", (int)g_is_tail);
    else if(r=="esn") Serial.printf("esn=%d\n", (int)g_esn_on);
    else if(r=="channel") Serial.printf("channel=%u\n", (unsigned)g_channel);
    else if(r=="namebinds"){ for(auto& kv: name2dev){ Serial.printf("\"%s\" -> %u\n", kv.first.c_str(), (unsigned)kv.second); } }
    else Serial.println(F("ERR: show room|tail|esn|channel|namebinds"));
  }
  else if(cmd=="dbg"){
    int sp2=rest.indexOf(' ');
    String what = sp2<0 ? rest : rest.substring(0, sp2);
    String val  = sp2<0 ? ""   : rest.substring(sp2+1);
    what.trim(); what.toLowerCase(); val.trim(); val.toLowerCase();
    bool on = (val=="on"||val=="1"||val=="true");
    if (what=="hex"){ DBG_HEX=on; Serial.printf("OK dbg hex=%d\n",(int)DBG_HEX); }
    else if (what=="parse"){ DBG_PARSE=on; Serial.printf("OK dbg parse=%d\n",(int)DBG_PARSE); }
    else if (what=="names"){ DBG_NAMES=on; Serial.printf("OK dbg names=%d\n",(int)DBG_NAMES); }
    else Serial.println(F("ERR: dbg hex|parse|names on|off"));
  }
  else if(cmd=="save"){ saveNameBinds(); saveEsn(); Serial.println(F("OK saved")); }
  else if(cmd.length()){ Serial.println(F("ERR: unknown. Type 'help'.")); }
}

static void pollUsb(){
  while(Serial.available()){
    static String line; char c=(char)Serial.read();
    if(c=='\r'||c=='\n'){ line.trim(); if(line.length()) handleCommand(line); line=""; }
    else line += c;
  }
}

// =================== ESP-NOW LAYER =====================
static bool esnInited=false;

static void esn_teardown(){
  if(!esnInited) return;
  esp_now_deinit();
  WiFi.mode(WIFI_OFF);
  esnInited=false;
}

// forward decl
struct Entry;
static void esn_send_entries(const std::vector<struct Entry>& vec, uint32_t batch);

static bool esn_setup(){
  if(!g_esn_on){ esn_teardown(); return false; }
  WiFi.mode(WIFI_STA);

  // lock channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(g_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init()!=ESP_OK){ Serial.println("ESP-NOW init failed"); return false; }
  esnInited = true;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ESN_BROADCAST, 6);
  peer.channel = g_channel;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // RX
  esp_now_register_recv_cb([](const esp_now_recv_info_t* info, const uint8_t* data, int len){
    (void)info;
    if (len < (int)sizeof(EsnHdr)) return;
    const EsnHdr* h = (const EsnHdr*)data;
    if (h->magic0!='S' || h->magic1!='M' || h->ver!=2) return;

    // Only accept from previous room or self (rebroadcast)
    if (!(h->src_room==g_room || h->src_room==(uint8_t)(g_room-1))) return;

    // New batch? reset winners AND this room's localBest (patch)
    if (g_rx_batch != h->batch){
      if (DBG_PARSE) Serial.printf("[RX] new batch=%lu (was %lu) -> reset winners\n",
                                   (unsigned long)h->batch, (unsigned long)g_rx_batch);
      g_rx_batch   = h->batch;
      g_rx_printed = false;
      chainBest.clear();
      localBest.clear();              // <-- important: reset this room’s scan cache
    }

    // hex dump fragment
    dumpHex("[ESN-RX] frag", data, len);

    // Pull entries
    size_t need = sizeof(EsnHdr) + (size_t)h->count * sizeof(Entry);
    if (need > (size_t)len) return;

    const Entry* ents = (const Entry*)(data + sizeof(EsnHdr));

    // Merge into winners-for-this-batch: keep strongest RSSI (propagated sources)
    for (uint8_t i=0;i<h->count;i++){
      uint16_t dev = ents[i].dev; int8_t rssi = ents[i].rssi; uint8_t room = ents[i].room;
      auto it = chainBest.find(dev);
      if (it==chainBest.end() || rssi > it->second.rssi){
        chainBest[dev] = { rssi, room };
      }
    }

    // On last fragment: merge this room's own localBest, then forward or print
    if (h->index == h->total - 1){
      if (!g_is_tail){
        // (1) fold localBest into chainBest for this *batch*
        for (auto &kv : localBest){
          uint16_t dev  = kv.first;
          int8_t   rssi = kv.second;
          auto it = chainBest.find(dev);
          if (it == chainBest.end() || rssi > it->second.rssi){
            chainBest[dev] = { rssi, g_room };   // tag with THIS room
          }
        }

        // (2) forward once per batch
        delay((unsigned long)SLOT_MS * (unsigned long)g_room + MERGE_DELAY_MS);
        std::vector<Entry> all; all.reserve(chainBest.size());
        for (auto &kv : chainBest) all.push_back(Entry{ kv.first, kv.second.rssi, kv.second.room });
        esn_send_entries(all, g_rx_batch);

        // (3) clear for next batch window
        localBest.clear();
      } else {
        // Tail prints exactly once per batch on the last fragment
        if (!g_rx_printed){
          g_rx_printed = true;
          Serial.println("--- FINAL ---");
          for (auto &kv : chainBest){
            Serial.printf("%u.%u@%d\n", (unsigned)kv.second.room, (unsigned)kv.first, (int)kv.second.rssi);
            Serial1.printf("%u.%u@%d\n", (unsigned)kv.second.room, (unsigned)kv.first, (int)kv.second.rssi);
          }
          Serial.println("--------------");
        }
      }
    }
  });

  return true;
}

// helper to TX one vector of entries with a given batch id
static void esn_send_entries(const std::vector<Entry>& vec, uint32_t batch){
  if (!esnInited) return;
  const size_t perFrag = (ESN_FRAG_DATA_MAX) / sizeof(Entry);
  uint8_t total = (uint8_t)((vec.size() + perFrag - 1) / perFrag); if (!total) total=1;
  for (uint8_t idx=0; idx<total; ++idx){
    size_t off  = (size_t)idx * perFrag;
    size_t take = std::min(perFrag, vec.size() - off);
    uint8_t buf[sizeof(EsnHdr) + ESN_FRAG_DATA_MAX];
    EsnHdr* h = (EsnHdr*)buf;
    h->magic0='S'; h->magic1='M'; h->ver=2; h->src_room=g_room;
    h->total=total; h->index=idx; h->count=(uint8_t)take; h->batch=batch;
    memcpy(buf+sizeof(EsnHdr), &vec[off], take*sizeof(Entry));
    size_t bytes = sizeof(EsnHdr) + take*sizeof(Entry);
    dumpHex("[ESN-TX] frag", buf, bytes);
    // slot delay before each frag to spread airtime by room index
    delay((unsigned long)SLOT_MS * (unsigned long)g_room);
    esp_now_send(ESN_BROADCAST, buf, bytes);
  }
}

// =================== UART (ICE or UI) ==================
static uint32_t lastLineMs = 0;

static void startNewBatchIfRoom1(){
  if (g_room==1){
    g_batch++;
    chainBest.clear();  // winners reset for the new scan
    if (DBG_PARSE) Serial.printf("[BATCH] start %lu\n", (unsigned long)g_batch);
  }
}

static void flushAndSendIfNeeded(){
  // Room 1 originates sending after END (or timeout)
  if (g_room!=1 || g_is_tail) return;

  // Merge room1 localBest into chainBest (room tag = this room)
  for (auto &kv : localBest){
    auto it = chainBest.find(kv.first);
    if (it==chainBest.end() || kv.second > it->second.rssi){
      chainBest[kv.first] = { kv.second, g_room };
    }
  }

  // Build vector
  std::vector<Entry> start;
  start.reserve(chainBest.size());
  for (auto &kv : chainBest){
    start.push_back(Entry{ kv.first, kv.second.rssi, kv.second.room });
  }
  if (start.empty()){
    if (DBG_PARSE) Serial.println("[BATCH] nothing to send");
    return;
  }
  // short merge delay
  delay(MERGE_DELAY_MS);
  esn_send_entries(start, g_batch);
}

static void pollUART(){
  while (Serial1.available()){
    char c = (char)Serial1.read();
    static String line;
    if (c=='\r' || c=='\n'){
      line.trim();
      if (line.length()){
        // Recognize END markers
        if (line=="--- END ---" || line=="--- END OF SCAN ---"){
          if (DBG_PARSE) Serial.println("[ICE] END");
          // Room1: start new batch and send starters
          startNewBatchIfRoom1();
          flushAndSendIfNeeded();
          // clear localBest for next round
          localBest.clear();
        } else {
          // Expect "NAME@RSSI"
          int at = line.lastIndexOf('@');
          if (at>0){
            String raw = line.substring(0, at);
            String san = sanitizePrefix(raw);
            addSeen(san);
            int rssi = line.substring(at+1).toInt();
            auto it = name2dev.find(san);
            if (it!=name2dev.end()){
              uint16_t dev = it->second;
              int8_t v = (int8_t)rssi;
              auto lb = localBest.find(dev);
              if (lb==localBest.end() || v > lb->second) localBest[dev]=v;
              if (DBG_PARSE && DBG_NAMES){
                Serial.printf("[ICE] RAW=\"%s\" SAN=\"%s\" -> %u RSSI=%d\n", raw.c_str(), san.c_str(), (unsigned)dev, (int)v);
              }
            } else {
              if (DBG_PARSE && DBG_NAMES) Serial.printf("[ICE] RAW=\"%s\" SAN=\"%s\" -> (unbound)\n", raw.c_str(), san.c_str());
            }
          } else {
            if (DBG_PARSE) Serial.printf("[ICE] skip: %s\n", line.c_str());
          }
        }
      }
      line="";
      lastLineMs = millis();
    } else {
      line += c;
      lastLineMs = millis();
    }
  }

  // Timeout flush: if we saw lines recently but no END, treat as end after 1s
  if (!g_is_tail && g_room==1){
    if (millis() - lastLineMs > 1000 && !localBest.empty()){
      if (DBG_PARSE) Serial.println("[ICE] timeout -> flush");
      startNewBatchIfRoom1();
      flushAndSendIfNeeded();
      localBest.clear();
    }
  }
}

// =================== SETUP / LOOP ======================
void setup(){
  Serial.begin(115200); delay(50);
  Serial.println("\nSMOKE v3 ready (strongest-per-scan)");

  if (!SPIFFS.begin(true)) Serial.println("SPIFFS mount failed");
  loadRoom(); loadTail(); loadEsn(); loadNameBinds();

  Serial.printf("room=%u tail=%d esn=%d ch=%u\n",
                (unsigned)g_room, (int)g_is_tail, (int)g_esn_on, (unsigned)g_channel);
  printHelp();

  Serial1.setRxBufferSize(UART_RXBUF);
  Serial1.setTxBufferSize(UART_TXBUF);
  Serial1.begin(UART_BAUD_UI_ICE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  if (g_esn_on) esn_setup();
}

void loop(){
  pollUART();
  pollUsb();

  // Re-init ESPNOW if config changed at runtime
  static bool last_esn=false; static uint8_t last_ch=0;
  if (last_esn != g_esn_on || last_ch != g_channel){
    last_esn = g_esn_on; last_ch = g_channel;
    esn_teardown(); esn_setup();
  }
}
