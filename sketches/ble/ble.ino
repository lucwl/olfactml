/**
 * BLE
 * 
 * Connects to a server via BLE and sends a single byte
 */
#include <ArduinoBLE.h>

BLEService customService("88aadeff-64a4-47ae-8798-7d7e51b24e55");
BLECharacteristic dataCharacteristic("88aadeff-64a4-47ae-8798-7d7e51b24e55", BLERead | BLEWrite, 20);

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

    while (central.connected()) {
      if (dataCharacteristic.written()) {
        const uint8_t* data = dataCharacteristic.value();
        int length = dataCharacteristic.valueLength();

        Serial.print("Received ");
        Serial.print(length);
        Serial.print(" bytes: ");

        // Output bytes in transmission
        for (int i = 0; i < length; i++) {
          Serial.print((char)data[i]);
        }
        Serial.println();
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
