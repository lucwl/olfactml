#include "ble.h"

#define DEVICE_NAME "OlfactML_Edge"
#define SERVICE_UUID "88aadeff-64a4-47ae-8798-7d7e51b24e55"
#define CHARACTERISTIC_UUID "88aadeff-64a4-47ae-8798-7d7e51b24e56"
#define CHARACTERISTIC_PROPS                                                   \
  (BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY)

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class CentralCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Connected to central");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Disconnected from central");
  }
};

void bleInit() {
  Serial.println("Initialising BLE service");

  BLEDevice::init(DEVICE_NAME);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new CentralCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic =
      pService->createCharacteristic(CHARACTERISTIC_UUID, CHARACTERISTIC_PROPS);

  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();

  Serial.println("Waiting for client...");
}

void bleTransmit(JsonDocument doc) {
  char buffer[512];
  serializeJson(doc, buffer);

  pCharacteristic->setValue(buffer);
  pCharacteristic->notify();
}
