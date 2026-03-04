#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

void bleInit();

void bleStreamRow(uint8_t sensorIndex, uint32_t fingerprintIndex,
                  uint8_t position, uint16_t plateTemperature,
                  uint16_t heaterDuration, float temperature, float pressure,
                  float humidity, float gasResistance, const String &label);
