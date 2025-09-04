/*
  ICE — BLE scanner (ESP32-S3)
  Minimal stream: only Name@RSSI

  - Keeps ONLY named devices
  - Sanitizes name to printable ASCII 0x20..0x7E, trims, collapses spaces
  - USB prints:   Name@-RSSI
  - UART1 sends:  Name@-RSSI\n   (plain text)
    * Optional: COBS+CRC framing (off by default)

  GPIO:
    UART1 RX=16, TX=17  @ 921600 baud
*/

#include <Arduino.h>
#include <NimBLEDevice.h>

// ============== UART (to SMOKE) ==============
#define ICE_UART_RX    16
#define ICE_UART_TX    17
#define UART_BAUD_ICE  921600

// ============== Debug toggles =================
static bool DBG_USB    = true;   // print Name@RSSI to USB
static bool DBG_DROP   = true;   // log drops (no name / sanitized empty)
static bool DBG_HEX    = false;  // hex-dump TX frames (COBS mode)
static int  DBG_SAMPLE = 1;      // print 1 out of N matches

// ============== Output mode ===================
enum OutMode { OUT_TEXT, OUT_COBS };
static OutMode g_out_mode = OUT_TEXT;  // default = plain text line

// ============== Scan params ===================
static uint16_t SCAN_INTERVAL_MS = 120;   // BLE units are 0.625ms; NimBLE takes ms
static uint16_t SCAN_WINDOW_MS   = 80;
static bool     SCAN_ACTIVE      = true;

// ------------- helpers ------------------------
static String sanitize_name(const String& in) {
  String out; out.reserve(in.length());
  for (size_t i=0;i<in.length();++i) {
    uint8_t c = (uint8_t)in[i];
    if (c >= 32 && c <= 126) out += (char)c;
  }
  out.trim();
  String col; col.reserve(out.length());
  bool sp=false;
  for (size_t i=0;i<out.length(); ++i) {
    char c = out[i];
    if (c==' ' || c=='\t') { if (!sp) { col+=' '; sp=true; } }
    else { col+=c; sp=false; }
  }
  return col;
}

static bool sampler() {
  static uint32_t ctr=0;
  if (DBG_SAMPLE <= 1) return true;
  return (++ctr % (uint32_t)DBG_SAMPLE) == 0;
}

// CRC16-CCITT
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF){
  while(len--){ crc ^= (uint16_t)(*data++)<<8; for(int i=0;i<8;i++) crc = (crc&0x8000)?((crc<<1)^0x1021):(crc<<1); }
  return crc;
}

// COBS encode (for optional framed mode)
static size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out, size_t outMax){
  if (outMax==0) return 0;
  const uint8_t* ip=in; const uint8_t* end=in+len;
  uint8_t* op=out; uint8_t* code=op++; uint8_t code_val=1;
  while (ip<end){
    uint8_t b=*ip++;
    if (b==0){
      *code=code_val; code=op++; code_val=1;
      if ((size_t)(op-out)>=outMax) return 0;
    } else {
      if ((size_t)(op-out)>=outMax) return 0;
      *op++=b; code_val++;
      if (code_val==0xFF){ *code=code_val; code=op++; code_val=1; if ((size_t)(op-out)>=outMax) return 0; }
    }
  }
  *code=code_val;
  if ((size_t)(op-out)>=outMax) return 0;
  *op++=0x00;
  return (size_t)(op-out);
}

static void dump_hex(const char* tag, const uint8_t* p, size_t n){
  if (!DBG_HEX) return;
  Serial.printf("%s len=%u\n", tag, (unsigned)n);
  for (size_t i=0;i<n;i+=16){
    Serial.printf("%04u: ", (unsigned)i);
    for (size_t j=0;j<16 && i+j<n; ++j) Serial.printf("%02X ", p[i+j]);
    Serial.print(" | ");
    for (size_t j=0;j<16 && i+j<n; ++j){
      char c=(char)p[i+j]; Serial.print((c>=32 && c<=126)?c:'.');
    }
    Serial.println();
  }
}

// ------------- UART send ----------------------
static void send_line_uart(const String& line){
  if (g_out_mode == OUT_TEXT) {
    Serial1.print(line);
    Serial1.print('\n');
  } else {
    // frame = ASCII line + '\n' + CRC16 (little endian), then COBS
    uint8_t buf[512]; size_t p=0;
    size_t L = min((size_t)300, line.length());
    memcpy(&buf[p], line.c_str(), L); p += L;
    buf[p++] = '\n';
    uint16_t crc = crc16_ccitt(buf, p);
    buf[p++] = (uint8_t)(crc);
    buf[p++] = (uint8_t)(crc>>8);

    if (DBG_HEX) dump_hex("[ICE->SMOKE RAW]", buf, p);

    uint8_t out[600];
    size_t outLen = cobs_encode(buf, p, out, sizeof(out));
    if (outLen) Serial1.write(out, outLen);
  }
}

// ------------- BLE callback -------------------
class AdvCB : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    // Get the name if present (either scan response or adv)
    std::string sname = dev->getName();  // NimBLE gives std::string (may be empty)
    String raw = String(sname.c_str());
    String name = sanitize_name(raw);

    if (!name.length()) {
      if (DBG_DROP) Serial.println(F("[DROP] empty or non-ASCII name"));
      return;
    }

    int rssi = dev->getRSSI();

    // USB print
    if (DBG_USB && sampler()) {
      Serial.printf("%s@%d\n", name.c_str(), (int)rssi);
    }

    // UART1 send (exactly Name@-RSSI\n by default)
    String line = name + "@" + String(rssi);
    send_line_uart(line);
  }
};

// ------------- CLI / USB ----------------------
static void print_help(){
  Serial.println(F("Commands:"));
  Serial.println(F("  dbg usb on|off        ; print Name@RSSI to USB"));
  Serial.println(F("  dbg drop on|off       ; log drop reasons"));
  Serial.println(F("  dbg hex on|off        ; hex-dump framed payloads"));
  Serial.println(F("  dbg sample <N>        ; print 1 of every N matches (USB only)"));
  Serial.println(F("  out text|cobs         ; UART1 output mode (default text)"));
  Serial.println(F("  scan on|off           ; start/stop BLE scan"));
  Serial.println(F("  scan cfg <interval_ms> <window_ms>"));
  Serial.println(F("  help"));
}

static void handle_cli(const String& s){
  int sp=s.indexOf(' ');
  String cmd=(sp<0)?s:s.substring(0,sp); cmd.trim(); cmd.toLowerCase();
  String rest=(sp<0)?"":s.substring(sp+1); rest.trim();

  if (cmd=="help" || cmd=="?"){ print_help(); }
  else if (cmd=="dbg"){
    int sp2=rest.indexOf(' ');
    String what=(sp2<0)?rest:rest.substring(0,sp2); what.toLowerCase();
    String val =(sp2<0)?"":rest.substring(sp2+1);  val.toLowerCase();
    bool on=(val=="on"||val=="1"||val=="true");
    if (what=="usb"){ DBG_USB=on; Serial.printf("OK dbg usb=%d\n",(int)DBG_USB); }
    else if (what=="drop"){ DBG_DROP=on; Serial.printf("OK dbg drop=%d\n",(int)DBG_DROP); }
    else if (what=="hex"){ DBG_HEX=on; Serial.printf("OK dbg hex=%d\n",(int)DBG_HEX); }
    else if (what=="sample"){ int n=val.toInt(); if(n<1)n=1; DBG_SAMPLE=n; Serial.printf("OK dbg sample=%d\n",DBG_SAMPLE); }
    else Serial.println(F("ERR: dbg usb|drop|hex|sample ..."));
  }
  else if (cmd=="out"){
    rest.toLowerCase();
    if (rest=="text"){ g_out_mode=OUT_TEXT; Serial.println(F("OK out=text")); }
    else if (rest=="cobs"){ g_out_mode=OUT_COBS; Serial.println(F("OK out=cobs")); }
    else Serial.println(F("ERR: out text|cobs"));
  }
  else if (cmd=="scan"){
    if (rest.startsWith("on")) { SCAN_ACTIVE=true; NimBLEDevice::getScan()->start(0, nullptr, false); Serial.println(F("OK scan=on")); }
    else if (rest.startsWith("off")){ SCAN_ACTIVE=false; NimBLEDevice::getScan()->stop(); Serial.println(F("OK scan=off")); }
    else if (rest.startsWith("cfg")){
      // scan cfg <interval_ms> <window_ms>
      int sp2=rest.indexOf(' ');
      int sp3=rest.indexOf(' ', sp2+1);
      if (sp2>0 && sp3>sp2){
        SCAN_INTERVAL_MS = (uint16_t)rest.substring(sp2+1, sp3).toInt();
        SCAN_WINDOW_MS   = (uint16_t)rest.substring(sp3+1).toInt();
        NimBLEScan* sc = NimBLEDevice::getScan();
        sc->setInterval(SCAN_INTERVAL_MS);
        sc->setWindow(SCAN_WINDOW_MS);
        Serial.printf("OK scan cfg interval=%u window=%u\n", (unsigned)SCAN_INTERVAL_MS, (unsigned)SCAN_WINDOW_MS);
      } else Serial.println(F("ERR: scan cfg <interval_ms> <window_ms>"));
    } else {
      Serial.println(F("ERR: scan on|off|cfg ..."));
    }
  }
  else {
    Serial.println(F("ERR: unknown. Type 'help'."));
  }
}

static void pollUsb(){
  while(Serial.available()){
    static String line;
    char c=(char)Serial.read();
    if (c=='\r' || c=='\n'){
      line.trim(); if(line.length()) handle_cli(line); line="";
    } else line += c;
  }
}

// ------------- Setup / Loop -------------------
void setup(){
  Serial.begin(115200);
  Serial1.begin(UART_BAUD_ICE, SERIAL_8N1, ICE_UART_RX, ICE_UART_TX);

  // BLE init
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max RX sensitivity for scanning
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  pScan->setInterval(SCAN_INTERVAL_MS);
  pScan->setWindow(SCAN_WINDOW_MS);
  pScan->setActiveScan(true);

  if (SCAN_ACTIVE) pScan->start(0, nullptr, false);

  Serial.println("ICE ready. Type 'help' for commands.");
  Serial.println("Default: print & send Name@RSSI as plain text.");
}

void loop(){
  pollUsb();
  // NimBLE handles scanning in background
}
