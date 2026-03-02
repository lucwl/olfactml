/**
 * SD Card Logger — BME688 Dev Kit
 * Implementation — see sdLogger.h for the public API.
 */

#include "sdLogger.h"
#include <SD.h>
#include <SPI.h>

/* ── Board-specific pin ────────────────────────────────────────────────
 * Adjust SD_CS_PIN to match your hardware.
 * The SD card shares the SPI bus with the BME sensors; the commMux
 * I2C-expander deselects all sensor CS lines between transactions, so
 * the SD card's dedicated GPIO CS is safe to use on the same bus.
 * ────────────────────────────────────────────────────────────────────── */
#define SD_CS_PIN 33

static String currentPath;   /* full SD path, e.g. "/lavender.csv" */
static bool   sdAvailable = false;

/* ═══════════════════════════════════════════════════════════════════════
 * sdInit
 * ═══════════════════════════════════════════════════════════════════════ */

bool sdInit() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD: No card detected or initialisation failed.");
    sdAvailable = false;
    return false;
  }
  sdAvailable = true;
  uint32_t sizeMB = (uint32_t)(SD.cardSize() / (1024ULL * 1024ULL));
  Serial.println("SD: Card initialised  (" + String(sizeMB) + " MB).");
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * sdOpenFile
 * ═══════════════════════════════════════════════════════════════════════ */

bool sdOpenFile(const String &filename) {
  if (!sdAvailable) {
    Serial.println("SD: Card not available — cannot open file.");
    return false;
  }

  /* Build the full path */
  currentPath  = "/";
  currentPath += filename;
  if (!currentPath.endsWith(".csv")) currentPath += ".csv";

  bool exists = SD.exists(currentPath.c_str());

  File f = SD.open(currentPath.c_str(), FILE_APPEND);
  if (!f) {
    Serial.println("SD: Could not open " + currentPath);
    return false;
  }

  if (!exists) {
    /* Write CSV header for a brand-new file */
    f.println("sensor_index,fingerprint_index,position,"
              "plate_temperature,heater_duration,"
              "temperature,pressure,humidity,gas_resistance,label");
    Serial.println("SD: Created new file " + currentPath);
  } else {
    Serial.println("SD: Appending to existing file " + currentPath);
  }

  f.close();
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * sdLogRow
 * ═══════════════════════════════════════════════════════════════════════ */

void sdLogRow(uint8_t       sensorIndex,
              uint32_t      fingerprintIndex,
              uint8_t       position,
              uint16_t      plateTemperature,
              uint16_t      heaterDuration,
              float         temperature,
              float         pressure,
              float         humidity,
              float         gasResistance,
              const String &label) {
  if (!sdAvailable || currentPath.length() == 0) return;

  File f = SD.open(currentPath.c_str(), FILE_APPEND);
  if (!f) return;

  f.print(sensorIndex);       f.print(',');
  f.print(fingerprintIndex);  f.print(',');
  f.print(position);          f.print(',');
  f.print(plateTemperature);  f.print(',');
  f.print(heaterDuration);    f.print(',');
  f.print(temperature, 2);    f.print(',');
  f.print(pressure, 2);       f.print(',');
  f.print(humidity, 2);       f.print(',');
  f.print(gasResistance, 0);  f.print(',');
  f.println(label);

  f.close();   /* close after each row — protects data on power loss */
}
