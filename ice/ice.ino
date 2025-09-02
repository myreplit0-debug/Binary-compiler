// ICE BLE scanner (ESP32 classic) — streams ALL adverts to SMOKE over UART1
// Lines: RAW,<name>,<rssi>   OR   MAC,<aa:bb:cc:dd:ee:ff>,<rssi>
//
// Build FQBN: esp32:esp32:esp32

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define RX1 16
#define TX1 17
HardwareSerial &S = Serial1;
BLEScan* pScan;

class AdvCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    std::string name = d.haveName()? d.getName() : "";
    int rssi = d.getRSSI();
    if (!name.empty()){
      String line = "RAW," + String(name.c_str()) + "," + String(rssi) + "\n";
      S.print(line);
    } else {
      char mac[18];
      sprintf(mac,"%02X:%02X:%02X:%02X:%02X:%02X",
        d.getAddress().getNative()[5], d.getAddress().getNative()[4],
        d.getAddress().getNative()[3], d.getAddress().getNative()[2],
        d.getAddress().getNative()[1], d.getAddress().getNative()[0]);
      String line = "MAC," + String(mac) + "," + String(rssi) + "\n";
      S.print(line);
    }
  }
};

void setup(){
  Serial.begin(115200);
  S.begin(115200, SERIAL_8N1, RX1, TX1);

  BLEDevice::init("");
  pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new AdvCallbacks());
  pScan->setActiveScan(true);
  pScan->setInterval(320);
  pScan->setWindow(160);
  Serial.println("[ICE] BLE scanner ready");
}

void loop(){
  pScan->start(2 /*sec*/, false);
  delay(50);
  pScan->stop();
}
