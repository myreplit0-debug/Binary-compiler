// ICE BLE scanner (ESP32-S3) — streams ALL adverts to SMOKE over UART1
// Lines: RAW,<name>,<rssi>   OR   MAC,<aa:bb:cc:dd:ee:ff>,<rssi>
//
// Build FQBN: esp32:esp32:esp32s3
// Uses NimBLE (default in Arduino-ESP32 for S3)

#include <Arduino.h>
#include <NimBLEDevice.h>

#define RX1 16
#define TX1 17
HardwareSerial &S = Serial1;

class AdvCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* d) override {
    std::string name = d->haveName() ? d->getName() : "";
    int rssi = d->getRSSI();
    if (!name.empty()){
      String line = "RAW," + String(name.c_str()) + "," + String(rssi) + "\n";
      S.print(line);
    } else {
      uint8_t const* a = d->getAddress().getNative();
      char mac[18];
      sprintf(mac,"%02X:%02X:%02X:%02X:%02X:%02X", a[5],a[4],a[3],a[2],a[1],a[0]);
      String line = "MAC," + String(mac) + "," + String(rssi) + "\n";
      S.print(line);
    }
  }
};

NimBLEScan* pScan;

void setup(){
  Serial.begin(115200);
  S.begin(115200, SERIAL_8N1, RX1, TX1);

  NimBLEDevice::init("");
  pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCallbacks(), false /* wantDuplicates */);
  pScan->setActiveScan(true);
  pScan->setInterval(320);
  pScan->setWindow(160);
  Serial.println("[ICE] BLE scanner ready");
}

void loop(){
  // Repeated short scans to keep callbacks flowing
  pScan->start(2, false); // seconds, is_continue
  delay(50);
  pScan->stop();
}
