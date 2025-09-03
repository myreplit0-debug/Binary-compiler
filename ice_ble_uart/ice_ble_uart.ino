/*
  ICE (ESP32-S3 N16R8) — BLE Scanner -> UART1 framed sender + USB debug

  UART1 (to SMOKE):
    RX = GPIO16, TX = GPIO17
    Baud: 921600
    Framing: COBS + trailing 0x00

  Message bytes (LE where multi-byte):
    VER(1) TYPE(1) ORIGIN(6) BATCH(4) SEQ(2) COUNT(2) PAYLOAD(...) CRC16(2)
    TYPE: 0=DATA, 1=END_OF_SCAN

  Device TLVs in PAYLOAD (order as appended):
    T=1 L=6  MAC6
    T=2 L=1  RSSI (int8)
    T=3 L=1  ADV_FLAGS (u8, 0 if unknown)
    T=4 L=n  NAME (<=32B, optional)
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// -------- Config --------
#define ICE_UART_BAUD        921600
#define ICE_UART_TX          17
#define ICE_UART_RX          16
#define ICE_UART_RXBUF       8192
#define ICE_UART_TXBUF       8192
#define SCAN_WINDOW_MS       5000
#define NAME_MAX_LEN         32
#define UART_PAYLOAD_TARGET  900

// -------- CRC16-CCITT (0x1021, init 0xFFFF) --------
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i=0; i<8; ++i) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// -------- COBS encode --------
static size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  const uint8_t* ip = in;
  const uint8_t* end = in + len;
  uint8_t* op = out;
  uint8_t* code_ptr = op++;
  uint8_t code = 1;
  while (ip < end) {
    uint8_t b = *ip++;
    if (b == 0) { *code_ptr = code; code_ptr = op++; code = 1; }
    else {
      *op++ = b; code++;
      if (code == 0xFF) { *code_ptr = code; code_ptr = op++; code = 1; }
    }
  }
  *code_ptr = code;
  return (size_t)(op - out);
}

// -------- Byte helpers --------
static inline void put_u16_le(uint8_t* p, uint16_t v){ p[0]=uint8_t(v); p[1]=uint8_t(v>>8); }
static inline void put_u32_le(uint8_t* p, uint32_t v){ p[0]=uint8_t(v); p[1]=uint8_t(v>>8); p[2]=uint8_t(v>>16); p[3]=uint8_t(v>>24); }

// -------- Globals --------
static uint8_t ORIGIN[6];
static uint32_t g_batch = 0;
static uint16_t g_seq   = 0;

static const size_t MSG_MAX  = 1400;
static uint8_t msg_buf[MSG_MAX];
static size_t  msg_len = 0;
static size_t  payload_off = 0;
static size_t  payload_len = 0;

static const size_t COBS_MAX = MSG_MAX + (MSG_MAX/254) + 2;
static uint8_t cobs_buf[COBS_MAX];

BLEScan* g_scan = nullptr;

// ---------- Forward decl ----------
static void debug_dump_payload(uint16_t count);

// ---------- Message builders ----------
static void begin_message(uint8_t type) {
  msg_len = 0;
  msg_buf[msg_len++] = 1;                // VER
  msg_buf[msg_len++] = type;             // TYPE
  for (int i=0;i<6;i++) msg_buf[msg_len++] = ORIGIN[i];
  put_u32_le(&msg_buf[msg_len], g_batch); msg_len += 4;
  put_u16_le(&msg_buf[msg_len], g_seq);   msg_len += 2;
  put_u16_le(&msg_buf[msg_len], 0);       msg_len += 2; // COUNT (later)
  payload_off = msg_len;
  payload_len = 0;
}

static void finish_and_send_message(uint16_t count) {
  // Fill COUNT
  put_u16_le(&msg_buf[1+1+6+4+2], count);
  msg_len = payload_off + payload_len;

  // CRC over VER..PAYLOAD
  uint16_t crc = crc16_ccitt(msg_buf, msg_len);
  msg_buf[msg_len++] = uint8_t(crc);
  msg_buf[msg_len++] = uint8_t(crc >> 8);

  // COBS + 0x00 out UART1 (binary for SMOKE)
  size_t enc = cobs_encode(msg_buf, msg_len, cobs_buf);
  cobs_buf[enc++] = 0x00;
  Serial1.write(cobs_buf, enc);
  Serial1.flush();

  // ---- Human-readable USB debug ----
  const uint8_t type = msg_buf[1];
  if (type == 0) { // DATA
    Serial.print("[batch="); Serial.print(g_batch);
    Serial.print(" seq="); Serial.print(g_seq);
    Serial.print(" count="); Serial.print(count);
    Serial.println("]");
    debug_dump_payload(count);
  } else if (type == 1) { // END_OF_SCAN
    Serial.println("--- END OF SCAN ---");
  }

  g_seq++;
}

static bool tlv_append(uint8_t T, const uint8_t* data, uint16_t L) {
  if (L > 255) return false; // 1-byte L
  if (payload_off + payload_len + 2 + L >= MSG_MAX - 2) return false; // leave room for CRC
  msg_buf[payload_off + payload_len++] = T;
  msg_buf[payload_off + payload_len++] = (uint8_t)L;
  memcpy(&msg_buf[payload_off + payload_len], data, L);
  payload_len += L;
  return true;
}

static bool append_device_record(BLEAdvertisedDevice& d) {
  String name = d.getName();
  if (name.length() > NAME_MAX_LEN) name = name.substring(0, NAME_MAX_LEN);
  size_t need = (2+6) + (2+1) + (2+1) + (name.length() ? (2+name.length()) : 0);
  if (payload_off + payload_len + need >= MSG_MAX - 2) return false;

  uint8_t mac6[6];
  memcpy(mac6, d.getAddress().getNative(), 6);
  int8_t  rssi_i8 = (int8_t)d.getRSSI();
  uint8_t rssi_u8 = (uint8_t)rssi_i8;
  uint8_t flags   = 0;

  tlv_append(1, mac6, 6);
  tlv_append(2, &rssi_u8, 1);
  tlv_append(3, &flags, 1);
  if (name.length()) tlv_append(4, (const uint8_t*)name.c_str(), (uint8_t)name.length());
  return true;
}

// ---------- BLE callback ----------
class IceScanCB : public BLEAdvertisedDeviceCallbacks {
public:
  uint16_t pending_count = 0;
  void onResult(BLEAdvertisedDevice d) override {
    if (payload_len == 0) { begin_message(0); pending_count = 0; }
    if (!append_device_record(d)) {
      finish_and_send_message(pending_count);
      begin_message(0);
      pending_count = 0;
      (void)append_device_record(d); // must fit in fresh buffer
    }
    pending_count++;
    if (payload_len >= UART_PAYLOAD_TARGET) {
      finish_and_send_message(pending_count);
      begin_message(0);
      pending_count = 0;
    }
  }
};
static IceScanCB g_cb;

// ---------- Debug dump (USB) ----------
static void mac_to_str(const uint8_t* m, char* out18) {
  sprintf(out18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

// Parse TLVs in the known order for 'count' device records and print them.
static void debug_dump_payload(uint16_t count) {
  size_t p = payload_off;
  const size_t end = payload_off + payload_len;

  for (uint16_t i = 0; i < count && p + 2 <= end; ++i) {
    // Expect MAC TLV
    if (p + 2 > end) break;
    uint8_t T = msg_buf[p++], L = msg_buf[p++];
    if (!(T == 1 && L == 6) || p + 6 > end) break;
    char macStr[18];
    mac_to_str(&msg_buf[p], macStr);
    p += 6;

    // RSSI TLV
    if (p + 2 > end) break;
    T = msg_buf[p++]; L = msg_buf[p++];
    int8_t rssi = 0;
    if (T == 2 && L == 1 && p + 1 <= end) { rssi = (int8_t)msg_buf[p++]; }
    else { break; }

    // FLAGS TLV
    if (p + 2 > end) break;
    T = msg_buf[p++]; L = msg_buf[p++];
    if (!(T == 3 && L == 1) || p + 1 > end) break;
    uint8_t flags = msg_buf[p++]; (void)flags;

    // Optional NAME TLV
    String name;
    if (p + 2 <= end && msg_buf[p] == 4) {
      T = msg_buf[p++]; L = msg_buf[p++];
      if (p + L <= end) {
        name = String((const char*)&msg_buf[p], L);
        p += L;
      }
    }

    Serial.print("MAC="); Serial.print(macStr);
    Serial.print(" RSSI="); Serial.print(rssi);
    if (name.length()) { Serial.print(" Name=\""); Serial.print(name); Serial.print("\""); }
    Serial.println();
  }
}

// ---------- Setup / Loop ----------
static void fill_origin_mac() {
  uint64_t mac = ESP.getEfuseMac();
  ORIGIN[0] = (uint8_t)(mac >> 40);
  ORIGIN[1] = (uint8_t)(mac >> 32);
  ORIGIN[2] = (uint8_t)(mac >> 24);
  ORIGIN[3] = (uint8_t)(mac >> 16);
  ORIGIN[4] = (uint8_t)(mac >>  8);
  ORIGIN[5] = (uint8_t)(mac >>  0);
}

void setup() {
  // USB CDC for human-readable testing
  Serial.begin(115200);

  // Hardware UART1 — binary framed output for SMOKE (later)
  Serial1.setRxBufferSize(ICE_UART_RXBUF);
  Serial1.setTxBufferSize(ICE_UART_TXBUF);
  Serial1.begin(ICE_UART_BAUD, SERIAL_8N1, ICE_UART_RX, ICE_UART_TX);

  fill_origin_mac();

  // BLE init (Arduino-ESP32 uses NimBLE on S3)
  BLEDevice::init("");
  g_scan = BLEDevice::getScan();
  g_scan->setAdvertisedDeviceCallbacks(&g_cb, true); // wantDuplicates
  g_scan->setActiveScan(true);
  g_scan->setInterval(160); // 100ms
  g_scan->setWindow(120);   // 75ms
}

void loop() {
  g_seq = 0; // reset per batch

  // Run scan window (seconds, float)
  g_scan->start(SCAN_WINDOW_MS / 1000.0, false);

  // Flush any partial message left by callback
  if (payload_len > 0 && g_cb.pending_count > 0) {
    finish_and_send_message(g_cb.pending_count);
    g_cb.pending_count = 0;
  }

  // END marker
  begin_message(1);
  finish_and_send_message(0);

  g_batch++;
  delay(100);
}
