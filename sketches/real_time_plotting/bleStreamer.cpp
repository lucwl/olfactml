#include "bleStreamer.h"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Connected to central");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Disconnected from central");
  }
};

void bleInit() {
  Serial.println("Initialising BLE service");

  BLEDevice::init("OlfactML_Edge");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("88aadeff-64a4-47ae-8798-7d7e51b24e55");

  pCharacteristic = pService->createCharacteristic(
                      "88aadeff-64a4-47ae-8798-7d7e51b24e56",
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("Waiting for client...");
}

void bleStreamRow(uint8_t sensorIndex, uint32_t fingerprintIndex,
                  uint8_t position, uint16_t plateTemperature,
                  uint16_t heaterDuration, float temperature, float pressure,
                  float humidity, float gasResistance, const String &label) {
  if (!deviceConnected) {
    return;
  }

  char row[128];

  snprintf(row, sizeof(row), "%u,%u,%u,%u,%u,%f,%f,%f,%f,%s", sensorIndex,
           fingerprintIndex, position, plateTemperature, heaterDuration,
           temperature, pressure, humidity, gasResistance, label.c_str());

  pCharacteristic->setValue(row);
  pCharacteristic->notify();
}
