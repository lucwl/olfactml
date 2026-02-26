/**
 * Real-Time Inference — BME688 Dev Kit + LiteRT (TFLite Micro)
 *
 * Collects measurements from one BME688 sensor using the heating-profile
 * API mirrored from real_time_plotting, validates the collected scan
 * against the profile length, then runs inference.
 *
 * Supported scan modes:
 *   Forced mode     — single heater step; inference runs immediately after
 *                     each valid reading.
 *   Parallel mode   — full scan cycle (up to 10 steps) collected before
 *                     inference. (Multi-step model pending.)
 *   Sequential mode — full scan cycle collected before inference.
 *                     (Multi-step model pending.)
 *
 * Current model (forced mode):
 *   Input [1, 6] → Dense(16, ReLU) → Dense(16, ReLU) → Dense(3, Softmax)
 *   Feature order: plate_temperature, heater_duration,
 *                  temperature (°C), pressure (hPa), humidity (%), gas_resistance (Ω)
 *   Classes: 0 — air | 1 — eucalyptus | 2 — lavender
 *
 * Serial commands:
 *   run               — start continuous inference
 *   stop              — stop inference
 *   profile <N>       — (stopped) set heating profile
 *   profiles          — list all available profiles
 *   status            — show current state
 *
 * Hardware: Bosch BME688 Development Kit (Adafruit ESP32 Feather +
 *           8-sensor SPI board via I2C GPIO expander).
 *
 * Required libraries:
 *   • bme68xLibrary  (Bosch Sensortec)
 *   • TFLite Micro: espressif__esp-tflite-micro, bundled with ESP32 Arduino core 3.3.7+
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"
#include "model.h"
#include "statistics.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* Both bits must be set for a gas reading to be considered valid at the
 * intended heater temperature. */
#define GAS_VALID_MSK (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)

/* ═══════════════════════════════════════════════════════════════════════
 * Heating profile definitions  (mirrored from real_time_plotting)
 * ═══════════════════════════════════════════════════════════════════════ */

enum ScanMode { SCAN_FORCED, SCAN_PARALLEL, SCAN_SEQUENTIAL };

struct HeatingProfile {
  const char *name;
  const char *description;
  ScanMode    scanMode;
  /* Forced mode */
  uint16_t    forcedTemp;   // target temperature, °C
  uint16_t    forcedDur;    // heater duration, ms
  /* Parallel / Sequential mode */
  uint8_t     profileLen;   // number of heater steps (max 10)
  uint16_t    tempProf[10]; // heater temperatures, °C
  uint16_t    mulProf[10];  // parallel: duration multipliers; sequential: durations (ms)
  /* Sequential mode only */
  uint8_t     seqSleep;     // ODR constant, e.g. BME68X_ODR_250_MS
};

const HeatingProfile PROFILES[] = {

  /* ── Forced mode ──────────────────────────────────────────────────── */
  { "Forced_Std",    "[F] 320 C / 150 ms  (default)",
    SCAN_FORCED, 320, 150, 0, {0}, {0}, 0 },
  { "Forced_Low",    "[F] 200 C / 150 ms",
    SCAN_FORCED, 200, 150, 0, {0}, {0}, 0 },
  { "Forced_High",   "[F] 400 C / 200 ms",
    SCAN_FORCED, 400, 200, 0, {0}, {0}, 0 },

  /* ── Parallel mode ────────────────────────────────────────────────── */
  { "Par_Bosch",     "[P] Bosch standard 10-step: 320,100,100,100,200,200,200,320,320,320 C",
    SCAN_PARALLEL, 0, 0, 10,
    {320, 100, 100, 100, 200, 200, 200, 320, 320, 320},
    {  5,   2,  10,  30,   5,   5,   5,   5,   5,   5}, 0 },
  { "Par_LinSweep",  "[P] Linear sweep  200 -> 400 C  (10 equal steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {200, 222, 244, 267, 289, 311, 333, 356, 378, 400},
    {  5,   5,   5,   5,   5,   5,   5,   5,   5,   5}, 0 },
  { "Par_WideSweep", "[P] Wide sweep    100 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100, 150, 200, 250, 300, 325, 350, 380, 415, 450},
    { 10,   8,   7,   6,   5,   5,   5,   5,   5,   5}, 0 },
  { "Par_HighFocus", "[P] High-temp     300 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {300, 317, 333, 350, 367, 383, 400, 417, 433, 450},
    {  5,   5,   5,   5,   5,   5,   5,   5,   5,   5}, 0 },
  { "Par_LowFocus",  "[P] Low-temp      100 -> 250 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100, 117, 133, 150, 167, 183, 200, 217, 233, 250},
    { 10,   9,   8,   8,   7,   7,   6,   6,   6,   5}, 0 },

  /* ── Sequential mode ──────────────────────────────────────────────── */
  { "Seq_Std",       "[S] 100,200,320 C / 150 ms each, 250 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 3,
    {100, 200, 320}, {150, 150, 150}, BME68X_ODR_250_MS },
  { "Seq_LinSweep",  "[S] Linear sweep 200->400 C (10 steps / 150 ms each, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {200, 222, 244, 267, 289, 311, 333, 356, 378, 400},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150}, BME68X_ODR_500_MS },
  { "Seq_Bosch",     "[S] Bosch 10-step: 320,100x3,200x3,320x3 C / 150 ms each, 500 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {320, 100, 100, 100, 200, 200, 200, 320, 320, 320},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150}, BME68X_ODR_500_MS },
  { "Seq_WideSweep", "[S] Wide sweep 100->450 C (10 steps, longer dwell at low temps, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {100, 150, 200, 250, 300, 325, 350, 380, 415, 450},
    {200, 180, 160, 150, 150, 140, 140, 140, 140, 140}, BME68X_ODR_500_MS },
};

const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

/* ═══════════════════════════════════════════════════════════════════════
 * Model / class configuration
 * ═══════════════════════════════════════════════════════════════════════ */

/* Tensor arena — must be large enough for all intermediate tensors.
 * This model (6→16→16→3 float32) fits comfortably within 10 kB. */
constexpr int kTensorArenaSize = 10 * 1024;

constexpr int NUM_CLASSES  = 2;
constexpr int NUM_FEATURES = 6;   // plate_temp, heater_dur, temp, pressure, hum, gas

const char* const CLASS_LABELS[NUM_CLASSES] = { "air", "eucalyptus"}; //"lavender" };

/* ═══════════════════════════════════════════════════════════════════════
 * Globals
 * ═══════════════════════════════════════════════════════════════════════ */

Bme68x  bme;
commMux commSetup;

uint8_t  sensorIdx     = 0;  // 0-based sensor index on the dev kit
uint8_t  activeProfile = 0;  // index into PROFILES[]
uint16_t sharedHeatrDur = 0; // parallel mode: shared heater duration (ms)

bool running = false;

/* Scan-cycle accumulation (parallel / sequential modes) */
float   gasBuffer[10];
bool    gasReceived[10];
float   lastTemp = 0, lastHum = 0, lastPres = 0;
int8_t  lastGasIdx  = -1;   // gas_index of the last received field (-1 = none yet)
bool    cycleHasData = false; // true when ≥1 valid gas step received this cycle

String cmdBuffer = "";

/* TFLite Micro objects in static storage (required by the runtime) */
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
static tflite::MicroMutableOpResolver<2> resolver;
static tflite::MicroInterpreter* interpreter = nullptr;

/* ═══════════════════════════════════════════════════════════════════════
 * Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/* z-score:  z = (x − μ) / σ
 * Only called for features with a non-zero std dev. */
inline float standardise(float value, int idx) {
  return (value - FEATURE_MEAN[idx]) / FEATURE_STD[idx];
}

void resetGasBuffer(uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    gasBuffer[i]   = 0.0f;
    gasReceived[i] = false;
  }
  lastGasIdx = -1;
}

/* Checks that every step in [0, profileLen) received a valid gas reading.
 * Returns true if the scan is complete; warns and returns false otherwise. */
bool validateScanSize(uint8_t profileLen) {
  uint8_t got = 0;
  for (uint8_t i = 0; i < profileLen; i++) {
    if (gasReceived[i]) got++;
  }
  if (got != profileLen) {
    Serial.print("[WARN] Incomplete scan: ");
    Serial.print(got);
    Serial.print("/");
    Serial.print(profileLen);
    Serial.println(" valid steps. Dropping cycle.");
    return false;
  }
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Inference  (forced mode)
 * ═══════════════════════════════════════════════════════════════════════ */

void runForcedInference(const HeatingProfile &prof, const bme68xData &data) {
  /* Build raw feature vector matching training feature order. */
  const float raw[NUM_FEATURES] = {
    (float)prof.forcedTemp,    // plate_temperature (passed as-is)
    (float)prof.forcedDur,     // heater_duration   (passed as-is)
    data.temperature,          // standardised
    data.pressure / 100.0f,    // standardised
    data.humidity,             // standardised
    data.gas_resistance,       // standardised
  };

  TfLiteTensor* inp = interpreter->input(0);
  for (int i = 0; i < NUM_FEATURES; i++) {
    /* Standardise only features that have a non-zero training std dev
     * (indices 0 and 1 have FEATURE_STD == 0 and are passed as-is). */
    inp->data.f[i] = (FEATURE_STD[i] != 0.0f) ? standardise(raw[i], i) : raw[i];
  }

  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERR: Invoke() failed.");
    return;
  }

  TfLiteTensor* out = interpreter->output(0);
  int   predicted = 0;
  float best      = out->data.f[0];
  for (int i = 1; i < NUM_CLASSES; i++) {
    if (out->data.f[i] > best) { best = out->data.f[i]; predicted = i; }
  }

  Serial.print("Prediction: ");
  Serial.print(CLASS_LABELS[predicted]);
  Serial.print("  (");
  Serial.print(best * 100.0f, 1);
  Serial.print("%)  scores: [");
  for (int i = 0; i < NUM_CLASSES; i++) {
    Serial.print(CLASS_LABELS[i]);
    Serial.print("=");
    Serial.print(out->data.f[i] * 100.0f, 1);
    Serial.print("%");
    if (i < NUM_CLASSES - 1) Serial.print("  ");
  }
  Serial.println("]");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Sensor initialisation
 * ═══════════════════════════════════════════════════════════════════════ */

/* Configures the sensor according to activeProfile.
 * Returns true on success, false if the sensor reports an error. */
bool initSensor() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  commSetup = commMuxSetConfig(Wire, SPI, sensorIdx, commSetup);
  bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup);

  if (bme.checkStatus()) {
    Serial.println("ERR: Sensor init failed — " + bme.statusString());
    return false;
  }

  bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);
  bme.setFilter(BME68X_FILTER_SIZE_3);

  if (prof.scanMode == SCAN_PARALLEL) {
    uint16_t tArr[10], mArr[10];
    memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
    memcpy(mArr, prof.mulProf,  prof.profileLen * sizeof(uint16_t));
    uint16_t sharedDur = 140 - (bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);
    bme.setHeaterProf(tArr, mArr, sharedDur, prof.profileLen);
    sharedHeatrDur = sharedDur;
  } else if (prof.scanMode == SCAN_SEQUENTIAL) {
    uint16_t tArr[10], dArr[10];
    memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
    memcpy(dArr, prof.mulProf,  prof.profileLen * sizeof(uint16_t));
    bme.setSeqSleep(prof.seqSleep);
    bme.setHeaterProf(tArr, dArr, prof.profileLen);
    sharedHeatrDur = 0;
  } else {
    bme.setHeaterProf(prof.forcedTemp, prof.forcedDur);
    sharedHeatrDur = 0;
  }

  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Measurement functions
 * ═══════════════════════════════════════════════════════════════════════ */

/* Trigger one forced-mode measurement, validate, and run inference. */
void takeForcedMeasurement() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  bme.setOpMode(BME68X_FORCED_MODE);
  delayMicroseconds(bme.getMeasDur(BME68X_FORCED_MODE));

  bme68xData data;
  if (!bme.fetchData()) return;
  bme.getData(data);

  if (!(data.status & BME68X_NEW_DATA_MSK))   return;
  if (!(data.status & BME68X_GASM_VALID_MSK)) return;
  if (!(data.status & BME68X_HEAT_STAB_MSK))  return;

  Serial.print("T="); Serial.print(data.temperature, 1);
  Serial.print(" P="); Serial.print(data.pressure / 100.0f, 1);
  Serial.print(" H="); Serial.print(data.humidity, 1);
  Serial.print(" G="); Serial.println(data.gas_resistance, 0);

  runForcedInference(prof, data);
}

/* Poll for new parallel-mode samples.
 * Cycle boundary detected by gas_index wrapping backwards — fires reliably on
 * the first reading of a new cycle regardless of whether the previous cycle's
 * last step had valid gas. At boundary: validate complete scan, then reset. */
void pollParallelMeasurement() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Wrap-around boundary detection (same logic as real_time_plotting) */
    if (lastGasIdx >= 0 && (int8_t)gi < lastGasIdx) {
      if (cycleHasData && validateScanSize(prof.profileLen)) {
        Serial.println("[Parallel] Complete " + String(prof.profileLen) +
                       "-step scan — parallel-mode inference not yet implemented.");
      }
      cycleHasData = false;
      resetGasBuffer(prof.profileLen);
    }

    lastTemp = data.temperature;
    lastHum  = data.humidity;
    lastPres = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      gasBuffer[gi]   = data.gas_resistance;
      gasReceived[gi] = true;
      cycleHasData    = true;
    }

    lastGasIdx = (int8_t)gi;
  }
}

/* Poll for new sequential-mode samples.
 * Same wrap-around boundary detection as parallel mode.
 * mulProf holds actual heater durations (ms) rather than multipliers. */
void pollSequentialMeasurement() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Same wrap-around boundary detection as parallel mode */
    if (lastGasIdx >= 0 && (int8_t)gi < lastGasIdx) {
      if (cycleHasData && validateScanSize(prof.profileLen)) {
        Serial.println("[Sequential] Complete " + String(prof.profileLen) +
                       "-step scan — sequential-mode inference not yet implemented.");
      }
      cycleHasData = false;
      resetGasBuffer(prof.profileLen);
    }

    lastTemp = data.temperature;
    lastHum  = data.humidity;
    lastPres = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      gasBuffer[gi]   = data.gas_resistance;
      gasReceived[gi] = true;
      cycleHasData    = true;
    }

    lastGasIdx = (int8_t)gi;
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Command handling
 * ═══════════════════════════════════════════════════════════════════════ */

void handleCommand(const String &cmd) {

  /* ── run ───────────────────────────────────────────────────────────── */
  if (cmd == "run") {
    if (running) {
      Serial.println(">> Already running. Send 'stop' first.");
      return;
    }
    if (!initSensor()) {
      Serial.println(">> Sensor init failed — cannot start.");
      return;
    }
    const HeatingProfile &prof = PROFILES[activeProfile];
    cycleHasData = false;
    if (prof.profileLen > 0) resetGasBuffer(prof.profileLen);  // also resets lastGasIdx

    if (prof.scanMode == SCAN_PARALLEL) {
      bme.setOpMode(BME68X_PARALLEL_MODE);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    }
    /* Forced mode: op-mode is set per measurement in takeForcedMeasurement(). */

    running = true;
    Serial.println(">> Running  profile " + String(activeProfile + 1) +
                   " [" + String(prof.name) + "]");

  /* ── stop ──────────────────────────────────────────────────────────── */
  } else if (cmd == "stop") {
    if (!running) {
      Serial.println(">> Not running.");
      return;
    }
    bme.setOpMode(BME68X_SLEEP_MODE);
    running = false;
    Serial.println(">> Stopped.");

  /* ── profile <N> ───────────────────────────────────────────────────── */
  } else if (cmd.startsWith("profile ")) {
    if (running) {
      Serial.println(">> Stop before changing profile.");
      return;
    }
    int n = cmd.substring(8).toInt();
    if (n < 1 || n > (int)NUM_PROFILES) {
      Serial.println(">> Profile must be 1–" + String(NUM_PROFILES) +
                     ". Send 'profiles' to list.");
    } else {
      activeProfile = (uint8_t)(n - 1);
      Serial.println(">> Profile set to " + String(n) +
                     " [" + String(PROFILES[activeProfile].name) + "]");
    }

  /* ── profiles ──────────────────────────────────────────────────────── */
  } else if (cmd == "profiles") {
    Serial.println("--- Available heating profiles ---");
    for (uint8_t i = 0; i < NUM_PROFILES; i++) {
      Serial.println("  " + String(i + 1) + ": " +
                     String(PROFILES[i].name) + "  " +
                     String(PROFILES[i].description));
    }
    Serial.println("----------------------------------");

  /* ── status ────────────────────────────────────────────────────────── */
  } else if (cmd == "status") {
    const HeatingProfile &prof = PROFILES[activeProfile];
    Serial.println("Running : " + String(running ? "yes" : "no"));
    Serial.println("Sensor  : " + String(sensorIdx + 1));
    Serial.println("Profile : " + String(activeProfile + 1) +
                   " [" + String(prof.name) + "] — " +
                   String(prof.description));

  /* ── unknown ───────────────────────────────────────────────────────── */
  } else {
    Serial.println(">> Unknown: '" + cmd + "'");
    Serial.println("   Commands: run | stop | profile <N> | profiles | status");
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Setup
 * ═══════════════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(921600);
  Serial.println("=== Real-Time Inference — BME688 + LiteRT ===");

  commMuxBegin(Wire, SPI);
  delay(100);

  if (!initSensor()) {
    Serial.println("Halting.");
    while (1);
  }
  Serial.println("Sensor OK (index " + String(sensorIdx + 1) + ")");

  /* ── TFLite Micro ────────────────────────────────────────────────── */
  const tflite::Model* tfl_model = tflite::GetModel(model);

  if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERR: Model schema version mismatch ("
                   + String(tfl_model->version()) + " vs expected "
                   + String(TFLITE_SCHEMA_VERSION) + ")");
    while (1);
  }

  resolver.AddFullyConnected();
  resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(
      tfl_model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERR: AllocateTensors() failed — arena may be too small.");
    while (1);
  }

  TfLiteTensor* inp = interpreter->input(0);
  TfLiteTensor* out = interpreter->output(0);
  Serial.println("Model OK  |  input "
                 + String(inp->dims->data[0]) + "x" + String(inp->dims->data[1])
                 + "  output "
                 + String(out->dims->data[0]) + "x" + String(out->dims->data[1]));
  Serial.println("Arena used: " + String(interpreter->arena_used_bytes()) +
                 " / " + String(kTensorArenaSize) + " bytes");
  Serial.println("Active profile: 1 [" + String(PROFILES[0].name) + "]");
  Serial.println("Commands: run | stop | profile <N> | profiles | status");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Main loop
 * ═══════════════════════════════════════════════════════════════════════ */

void loop() {
  /* Non-blocking serial command reader */
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuffer.trim();
      if (cmdBuffer.length() > 0) {
        handleCommand(cmdBuffer);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }

  if (!running) return;

  const HeatingProfile &prof = PROFILES[activeProfile];

  if (prof.scanMode == SCAN_PARALLEL) {
    pollParallelMeasurement();
    delay(10);
  } else if (prof.scanMode == SCAN_SEQUENTIAL) {
    pollSequentialMeasurement();
    delay(10);
  } else {
    takeForcedMeasurement();
    delay(500);
  }
}
