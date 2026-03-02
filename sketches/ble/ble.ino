/**
 * BLE
 * 
 * Connects to a server via BLE and sends a sequence of integers 
 */
#include <ArduinoBLE.h>

BLEService customService("88aadeff-64a4-47ae-8798-7d7e51b24e55");
BLEIntCharacteristic dataCharacteristic("88aadeff-64a4-47ae-8798-7d7e51b24e56", BLERead | BLEWrite | BLENotify);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Set up BLE service
  BLE.begin();

  Serial.print("Device address: ");
  Serial.println(BLE.address());

  BLE.setLocalName("Nano33BLE");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(dataCharacteristic);
  BLE.addService(customService);
  BLE.advertise();
}

// Wait for connections
void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    uint32_t i = 0;
    while (central.connected()) {
      if (!dataCharacteristic.subscribed()) continue;
      dataCharacteristic.writeValue(i);
      Serial.print("Sent: ");
      Serial.println(i);
      i++;
      delay(1000);

    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
