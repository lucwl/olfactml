#include "bleStreamer.h"

BLEService dataService("88aadeff-64a4-47ae-8798-7d7e51b24e55");
BLEStringCharacteristic
    streamCharacteristic("88aadeff-64a4-47ae-8798-7d7e51b24e56", BLENotify, 20);
BLEDevice central;

void bleInit() {
  BLE.begin();
  BLE.setLocalName("OlfactML_Edge");
  BLE.setAdvertisedService(dataService);
  BLE.addService(dataService);
  BLE.advertise();

  while (1) {
    central = BLE.central();
    if (central.connected() && streamCharacteristic.subscribed()) {
      return;
    }
  }
}

void bleStreamRow(uint8_t sensorIndex, uint32_t fingerprintIndex,
                  uint8_t position, uint16_t plateTemperature,
                  uint16_t heaterDuration, float temperature, float pressure,
                  float humidity, float gasResistance, const String &label) {
  char row[128];

  snprintf(row, sizeof(row), "%u,%u,%u,%u,%u,%f,%f,%f,%f,%s", sensorIndex,
           fingerprintIndex, position, plateTemperature, heaterDuration,
           temperature, pressure, humidity, gasResistance, label);

  if (!central.connected()) {
    central = BLE.central();
  }

  streamCharacteristic.writeValue(row);
}
