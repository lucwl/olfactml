/**
 * Inference Input Logger — implementation.
 * See inferenceLogger.h for the public API.
 */

#include "inferenceLogger.h"
#include <SD.h>
#include <SPI.h>

/* ── SD chip-select pin ────────────────────────────────────────────────
 * Must match the GPIO wired to the SD card's CS line on your board.
 * The SD card shares the SPI bus with the BME sensors; the commMux
 * I2C-expander deselects all sensor CS lines between transactions so
 * the SD's dedicated CS pin is safe to use on the same bus. */
#define SD_CS_PIN 33

static bool     sdReady   = false;  /* SD card was successfully initialised */
static bool     logActive = false;  /* a log file is currently open */
static String   filePath;           /* full SD path, e.g. "/debug_run1.csv"  */
static uint32_t rowIndex  = 0;      /* inference counter; reset on ilOpen()  */

/* ═══════════════════════════════════════════════════════════════════════
 * ilInit
 * ═══════════════════════════════════════════════════════════════════════ */

bool ilInit() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[InferLog] SD init failed — logging unavailable.");
    sdReady = false;
    return false;
  }
  sdReady = true;
  uint32_t sizeMB = (uint32_t)(SD.cardSize() / (1024ULL * 1024ULL));
  Serial.println("[InferLog] SD card OK (" + String(sizeMB) + " MB).");
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * ilOpen
 * ═══════════════════════════════════════════════════════════════════════ */

bool ilOpen(const String &filename, int numInputs) {
  if (!sdReady) {
    Serial.println("[InferLog] SD card not available.");
    return false;
  }

  filePath  = "/";
  filePath += filename;
  if (!filePath.endsWith(".csv")) filePath += ".csv";

  bool exists = SD.exists(filePath.c_str());

  File f = SD.open(filePath.c_str(), FILE_APPEND);
  if (!f) {
    Serial.println("[InferLog] Cannot open " + filePath);
    return false;
  }

  if (!exists) {
    /* Write the CSV header for a brand-new file. */
    f.print("inference_index,label");
    for (int i = 0; i < numInputs; i++) {
      f.print(",input_");
      f.print(i);
    }
    f.println();
    Serial.println("[InferLog] Created " + filePath +
                   "  (" + String(numInputs) + " input columns)");
  } else {
    Serial.println("[InferLog] Appending to " + filePath);
  }

  f.close();
  rowIndex  = 0;
  logActive = true;
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * ilLog
 * ═══════════════════════════════════════════════════════════════════════ */

void ilLog(const float *inputs, int numInputs, const char *label) {
  if (!logActive || !sdReady || filePath.length() == 0) return;

  File f = SD.open(filePath.c_str(), FILE_APPEND);
  if (!f) return;

  f.print(rowIndex++);
  f.print(',');
  f.print(label);
  for (int i = 0; i < numInputs; i++) {
    f.print(',');
    f.print(inputs[i], 6);   /* 6 decimal places — adequate for z-scored floats */
  }
  f.println();
  f.close();   /* close after every row to protect data on power loss */
}

/* ═══════════════════════════════════════════════════════════════════════
 * ilClose
 * ═══════════════════════════════════════════════════════════════════════ */

void ilClose() {
  if (!logActive) return;
  logActive = false;
  Serial.println("[InferLog] Stopped. " + String(rowIndex) +
                 " row(s) written to " + filePath);
}
