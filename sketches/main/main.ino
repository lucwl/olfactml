/**
 * Real-Time Inference — BME688 Dev Kit + LiteRT (TFLite Micro)
 *
 * Collects measurements from one or more BME688 sensors in parallel using the
 * heating-profile API. All active sensors record independently; once every
 * sensor has a complete fingerprint the per-sensor fingerprints are z-score
 * scaled and averaged together, then inference is run on the result.
 *
 * Supported scan modes:
 *   Forced mode     — single heater step; inference runs immediately after
 *                     each valid reading.
 *   Sequential mode — full scan cycle (up to 10 steps) collected before
 *                     inference.
 *
 * Supported models (swap model.h to switch):
 *   Forced mode  — flat model: Input [1, 6] → Dense→Dense→Softmax
 *   Sequential   — CNN model:  Input [1, SEQ_LEN, 8] → Conv→Mean→Dense→Softmax
 *
 * Feature order (per timestep):
 *   temperature, pressure (hPa), humidity (%),
 *   gas_resistance (Ω), Δtemperature, Δpressure, Δhumidity, Δgas_resistance
 *
 * Serial commands:
 *   run               — start continuous inference
 *   stop              — stop inference / abort cleaning
 *   clean [<sec>]     — burn off accumulated compounds at 400 C (default 30 s,
 *                       5% duty cycle: 200 ms on / 3800 ms off)
 *   profile <N>       — (stopped) set heating profile
 *   profiles          — list all available profiles
 *   sensors [list]    — show or set active sensors, e.g. "sensors 1,2,3"
 *   status            — show current state
 *   logfile [<name>]  — show or set the SD log filename (default: inference_log)
 *   logstart          — begin logging model inputs to SD card CSV
 *   logstop           — stop SD logging (inference continues unaffected)
 *   config [<N>]      — collect N measurements (default 5) and store their
 *                       average as the baseline (overwrites any previous baseline)
 *
 * Hardware: Bosch BME688 Development Kit (Adafruit ESP32 Feather +
 *           8-sensor SPI board via I2C GPIO expander).
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"
#include "model.h"
#include "statistics.h"

#include "inferenceLogger.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* Both bits must be set for a gas reading to be considered valid at the
 * intended heater temperature. */
#define GAS_VALID_MSK (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)

/* ─── Cleaning cycle parameters ────────────────────────────────────── */
/* Duty cycle = CLEAN_DUR_MS / (CLEAN_DUR_MS + CLEAN_GAP_MS) = 200/4000 = 5%
 * Max recommended by Bosch datasheet: ~10%. One pulse every 4 s. */
constexpr uint16_t CLEAN_TEMP_C = 400;  /* heater setpoint during clean (°C) */
constexpr uint16_t CLEAN_DUR_MS = 200;  /* heater on-time per forced pulse (ms) */
constexpr uint32_t CLEAN_GAP_MS = 3800; /* cool-down gap between pulses (ms) */
constexpr uint32_t CLEAN_DEF_SEC = 30;  /* default cleaning duration (s) */

/* ═══════════════════════════════════════════════════════════════════════
 * Heating profile definitions
 * ═══════════════════════════════════════════════════════════════════════ */

enum ScanMode { SCAN_FORCED,
                SCAN_PARALLEL,
                SCAN_SEQUENTIAL };

struct HeatingProfile {
  const char *name;
  const char *description;
  ScanMode scanMode;
  /* Forced mode */
  uint16_t forcedTemp;
  uint16_t forcedDur;
  /* Parallel / Sequential mode */
  uint8_t profileLen;
  uint16_t tempProf[10];
  uint16_t mulProf[10];
  /* Sequential mode only */
  uint8_t seqSleep;
};

const HeatingProfile PROFILES[] = {

  /* ── Forced mode ──────────────────────────────────────────────────── */
  { "Forced_Std", "[F] 320 C / 150 ms  (default)", SCAN_FORCED, 320, 150, 0, { 0 }, { 0 }, 0 },
  { "Forced_Low", "[F] 200 C / 150 ms", SCAN_FORCED, 200, 150, 0, { 0 }, { 0 }, 0 },
  { "Forced_High", "[F] 400 C / 200 ms", SCAN_FORCED, 400, 200, 0, { 0 }, { 0 }, 0 },

  /* ── Parallel mode ────────────────────────────────────────────────── */
  { "Par_Bosch", "[P] Bosch standard 10-step: 320,100,100,100,200,200,200,320,320,320 C", SCAN_PARALLEL, 0, 0, 10, { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 }, { 5, 2, 10, 30, 5, 5, 5, 5, 5, 5 }, 0 },
  { "Par_LinSweep", "[P] Linear sweep  200 -> 400 C  (10 equal steps)", SCAN_PARALLEL, 0, 0, 10, { 200, 222, 244, 267, 289, 311, 333, 356, 378, 400 }, { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 }, 0 },
  { "Par_WideSweep", "[P] Wide sweep    100 -> 450 C  (10 steps)", SCAN_PARALLEL, 0, 0, 10, { 100, 150, 200, 250, 300, 325, 350, 380, 415, 450 }, { 10, 8, 7, 6, 5, 5, 5, 5, 5, 5 }, 0 },
  { "Par_HighFocus", "[P] High-temp     300 -> 450 C  (10 steps)", SCAN_PARALLEL, 0, 0, 10, { 300, 317, 333, 350, 367, 383, 400, 417, 433, 450 }, { 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 }, 0 },
  { "Par_LowFocus", "[P] Low-temp      100 -> 250 C  (10 steps)", SCAN_PARALLEL, 0, 0, 10, { 100, 117, 133, 150, 167, 183, 200, 217, 233, 250 }, { 10, 9, 8, 8, 7, 7, 6, 6, 6, 5 }, 0 },

  /* ── Sequential mode ──────────────────────────────────────────────── */
  { "Seq_Std", "[S] 100,200,320 C / 150 ms each, 250 ms ODR sleep", SCAN_SEQUENTIAL, 0, 0, 3, { 100, 200, 320 }, { 150, 150, 150 }, BME68X_ODR_250_MS },
  { "Seq_LinSweep", "[S] Linear sweep 200->400 C (10 steps / 150 ms each, 500 ms ODR sleep)", SCAN_SEQUENTIAL, 0, 0, 10, { 200, 222, 244, 267, 289, 311, 333, 356, 378, 400 }, { 150, 150, 150, 150, 150, 150, 150, 150, 150, 150 }, BME68X_ODR_500_MS },
  { "Seq_Bosch", "[S] Bosch 10-step: 320,100x3,200x3,320x3 C / 150 ms each, 500 ms ODR sleep", SCAN_SEQUENTIAL, 0, 0, 10, { 320, 100, 100, 100, 200, 200, 200, 320, 320, 320 }, { 150, 150, 150, 150, 150, 150, 150, 150, 150, 150 }, BME68X_ODR_500_MS },
  { "Seq_WideSweep", "[S] Wide sweep 100->450 C (10 steps, longer dwell at low temps, 500 ms ODR sleep)", SCAN_SEQUENTIAL, 0, 0, 10, { 100, 150, 200, 250, 300, 325, 350, 380, 415, 450 }, { 200, 180, 160, 150, 150, 140, 140, 140, 140, 140 }, BME68X_ODR_500_MS },
  { "Seq_Uni", "[S] 100,100,100 C / 150 ms each, 250 ms ODR sleep", SCAN_SEQUENTIAL, 0, 0, 3, { 100, 100, 100 }, { 150, 150, 150 }, BME68X_ODR_250_MS },
};

const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

/* ═══════════════════════════════════════════════════════════════════════
 * Model / class configuration
 * ═══════════════════════════════════════════════════════════════════════ */

constexpr int kTensorArenaSize = 32 * 1024;

constexpr int NUM_CLASSES = 4;
constexpr int NUM_FEATURES = 4;

const char *const CLASS_LABELS[NUM_CLASSES] = { "air", "eucalyptus", "grapefruit", "lavender" };
const char *const MODEL_TYPE = "CNN";
const char *const PREPROCESSING_TYPE = "BASELINE_STD";

/* ═══════════════════════════════════════════════════════════════════════
 * Multi-sensor state
 * ═══════════════════════════════════════════════════════════════════════ */

const uint8_t MAX_ACTIVE = 8;

/* User-configurable list of sensor indices (0-based, matching dev kit labels 1-8). */
uint8_t activeSensors[MAX_ACTIVE] = { 2 };
uint8_t numActiveSensors = 1;

/* All per-sensor runtime state is held in this struct so the array of
 * active sensors can be managed without changing any other code paths. */
struct SensorState {
  Bme68x bme;
  commMux comm;  // for I2C communication

  /* Ongoing scan-cycle accumulation */
  float gasBuffer[10];
  float gasDiffBuffer[10];
  bool gasReceived[10];
  float lastTemp, lastHum, lastPres;
  float diffTemp, diffHum, diffPres;
  int8_t lastGasIdx;
  bool cycleHasData;

  /* Frozen fingerprint from the most recently completed, validated cycle.
   * Written at wrap-around; read only after fingerprintReady == true. */
  float fpGas[10];
  float fpGasDiff[10];
  float fpTemp, fpHum, fpPres;
  float fpDiffTemp, fpDiffHum, fpDiffPres;
  bool fingerprintReady;
};

SensorState sensorStates[MAX_ACTIVE];

uint8_t activeProfile = 10;
uint16_t sharedHeatrDur = 0;
bool running = false;
bool cleaning = false;
uint32_t cleanStart = 0;
uint32_t cleanDuration = CLEAN_DEF_SEC * 1000UL;
uint32_t cleanLastPulse = 0; /* millis() of the last heater pulse */

/* Timestamp (millis) recorded at start of each collection window so we can
 * measure how long it takes all sensors to produce a complete fingerprint. */
uint32_t collectStart = 0;

/* ─── Label / accuracy tracking ───────────────────────────────────────
 * currentLabel == -1  →  no label set (unlabelled run, no accuracy tracking).
 * currentLabel 0..NUM_CLASSES-1  →  true class for the current sample.
 * labelTotal / labelCorrect accumulate across the session. */
int8_t currentLabel = -1;
uint16_t labelTotal[NUM_CLASSES] = { 0 };
uint16_t labelCorrect[NUM_CLASSES] = { 0 };

/* ─── Timing statistics ────────────────────────────────────────────────
 * Running totals for computing averages; reset with 'timing reset'. */
uint32_t timingCount = 0;
uint64_t timingCollectSum = 0;  // ms
uint64_t timingInferSum = 0;    // us

/* ─── SD inference log ─────────────────────────────────────────────────
 * Debugging feature: logs the exact averaged & standardised input tensor
 * that is fed to the model at each inference.  Entirely optional — set or
 * clear ilLogging with the 'logstart' / 'logstop' commands.  Inference
 * runs unchanged regardless of whether logging is active. */
bool ilLogging = false;
String ilFilename = "inference_log";

/* ─── Imputation ───────────────────────────────────────────────────────
 * When true, missing gas steps at wrap-around are filled from peer sensors
 * (or from the historical training mean when no peer data exists) instead
 * of dropping the cycle.  Toggle with 'impute on' / 'impute off'. */
bool imputationEnabled = false;

/* ─── Baseline configuration ───────────────────────────────────────────
 * 'config [<N>]' collects N complete measurements, averages the raw
 * sensor readings per heater-step, and stores the result in baselineFp[].
 * A new 'config' call overwrites the stored baseline. */
bool configuring = false;
uint8_t configTarget = 10;
uint8_t configCount = 0;
bool baselineReady = false;
float baselineFp[10 * NUM_FEATURES] = { 0 };
float configAccum[10 * NUM_FEATURES] = { 0 };
uint8_t baselineSeqLen = 0;

String cmdBuffer = "";


/* TFLite Micro */
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
static tflite::MicroMutableOpResolver<5> resolver;
static tflite::MicroInterpreter *interpreter = nullptr;

/* ═══════════════════════════════════════════════════════════════════════
 * Preprocessing
 * ═══════════════════════════════════════════════════════════════════════ */

inline float standardise(float value, int step, int feat) {
  return (value - FEATURE_MEAN[step][feat]) / FEATURE_STD[step][feat];
}

inline float fractional_baseline_correction(float value, int step, int feat) {
  int base = step * NUM_FEATURES;
  return (value - baselineFp[base + feat]) / (baselineFp[base + feat] + 1e-6f);
}



/* ═══════════════════════════════════════════════════════════════════════
 * Helpers
 * ═══════════════════════════════════════════════════════════════════════ */

void resetSensorGasBuffer(uint8_t s, uint8_t len) {
  SensorState &ss = sensorStates[s];
  for (uint8_t i = 0; i < len; i++) {
    ss.gasBuffer[i] = 0.0f;
    ss.gasReceived[i] = false;
  }
  ss.lastGasIdx = -1;
}

bool validateSensorScan(uint8_t s, uint8_t profileLen) {
  SensorState &ss = sensorStates[s];
  uint8_t got = 0;
  for (uint8_t i = 0; i < profileLen; i++) {
    if (ss.gasReceived[i]) got++;
  }
  if (got != profileLen) {
    Serial.print("[WARN] Sensor ");
    Serial.print(activeSensors[s] + 1);
    Serial.print(" incomplete scan: ");
    Serial.print(got);
    Serial.print("/");
    Serial.print(profileLen);
    Serial.println(" valid steps. Dropping cycle.");
    return false;
  }
  return true;
}

/* Fill any missing gas steps for sensor s by averaging live data from peer
 * sensors still on the same cycle, then freeze the fingerprint normally.
 *
 * Priority:
 *   1. Mean of sensorStates[o].gasBuffer[t] / gasDiffBuffer[t] for peers o
 *      that still carry valid (not-yet-reset) data for step t.
 *   2. Historical training mean FEATURE_MEAN[t][feat] (statistics.h) as
 *      a z-score = 0 neutral fallback when no peer data is available.
 *
 * Correctness note: resetSensorGasBuffer() sets gasReceived[] to false, so
 * peers that have already wrapped are automatically excluded by the
 * gasReceived[t] check — no extra synchronisation is needed. */
void imputeMissingSensorSteps(uint8_t s, uint8_t profileLen) {
  SensorState &ss = sensorStates[s];
  uint8_t imputedCount = 0;

  for (uint8_t t = 0; t < profileLen; t++) {
    if (ss.gasReceived[t]) continue;

    float gasSum = 0.0f, diffSum = 0.0f;
    uint8_t validPeers = 0;
    for (uint8_t o = 0; o < numActiveSensors; o++) {
      if (o == s) continue;
      if (sensorStates[o].gasReceived[t]) {
        gasSum += sensorStates[o].gasBuffer[t];
        diffSum += sensorStates[o].gasDiffBuffer[t];
        validPeers++;
      }
    }

    if (validPeers > 0) {
      ss.gasBuffer[t] = gasSum / validPeers;
      ss.gasDiffBuffer[t] = diffSum / validPeers;
    } else {
      ss.gasBuffer[t] = FEATURE_MEAN[t][0];      // gas_resistance mean
      ss.gasDiffBuffer[t] = FEATURE_MEAN[t][1];  // gas_resistance_diff mean
    }
    ss.gasReceived[t] = true;
    imputedCount++;
  }

  if (imputedCount > 0) {
    Serial.print("[IMPUTE] Sensor ");
    Serial.print(activeSensors[s] + 1);
    Serial.print(": imputed ");
    Serial.print(imputedCount);
    Serial.print("/");
    Serial.print(profileLen);
    Serial.println(" step(s).");
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Label / accuracy helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/* Called after every inference with the predicted class index.
 * If a label is set, prints the comparison and updates running totals. */
void recordResult(int predicted) {
  if (currentLabel < 0) return;
  bool correct = (predicted == currentLabel);
  labelTotal[currentLabel]++;
  if (correct) labelCorrect[currentLabel]++;
  if (correct) {
    Serial.println("  [OK]");
  } else {
    Serial.println("  [WRONG: expected " + String(CLASS_LABELS[currentLabel]) + "]");
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Inference  (sequential / parallel mode — averaged across all sensors)
 * ═══════════════════════════════════════════════════════════════════════ */

/* For each heater step t and feature i:
 *   1. Standardise / baseline-correct the frozen fingerprint value for every active sensor.
 *   2. Average the standardised values across sensors.
 *   3. Run inference on the averaged input tensor. */
void runAveragedInference() {
  const HeatingProfile &prof = PROFILES[activeProfile];
  TfLiteTensor *inp = interpreter->input(0);

  if (inp->dims->size != 3) {
    Serial.println("[WARN] runAveragedInference: model is not a sequence model "
                   "(expected 3-D input).");
    return;
  }
  const int modelSeqLen = inp->dims->data[1];
  if ((int)prof.profileLen != modelSeqLen) {
    Serial.print("[WARN] Profile has ");
    Serial.print(prof.profileLen);
    Serial.print(" steps but model expects SEQ_LEN=");
    Serial.print(modelSeqLen);
    Serial.println(". Dropping cycle — select a matching profile.");
    return;
  }

  /* Zero the input tensor and the sum-of-squares scratch buffer. */
  const int totalElems = prof.profileLen * NUM_FEATURES;
  for (int i = 0; i < totalElems; i++) inp->data.f[i] = 0.0f;
  float sumSq[10 * NUM_FEATURES] = { 0 };

  /* Accumulate standardised values (and their squares) from each sensor. */
  for (uint8_t s = 0; s < numActiveSensors; s++) {
    SensorState &ss = sensorStates[s];
    for (uint8_t t = 0; t < prof.profileLen; t++) {
      const float raw[NUM_FEATURES] = {
        // ss.fpTemp,
        // ss.fpPres,
        ss.fpHum,
        ss.fpGas[t],
        // ss.fpDiffTemp,
        // ss.fpDiffPres,
        ss.fpDiffHum,
        ss.fpGasDiff[t],
      };
      int base = t * NUM_FEATURES;
      for (int i = 0; i < NUM_FEATURES; i++) {
        float v;
        if (PREPROCESSING_TYPE == "STANDARD") {
          v = (FEATURE_STD[t][i] != 0.0f) ? standardise(raw[i], t, i) : raw[i];
        } else if (PREPROCESSING_TYPE == "BASELINE") {
          v = fractional_baseline_correction(raw[i], t, i);
        } else if (PREPROCESSING_TYPE == "BASELINE_STD") {
          v = fractional_baseline_correction(raw[i], t, i);
          v = (FEATURE_STD[t][i] != 0.0f) ? standardise(v, t, i) : v;
        } else {
          v = raw[i];
        }
        inp->data.f[base + i] += v;
        sumSq[base + i] += v * v;
      }
    }
  }

  /* Average. */
  for (int i = 0; i < totalElems; i++) {
    inp->data.f[i] /= (float)numActiveSensors;
  }

  /* SD debug log — record the averaged, standardised input tensor. */
  if (ilLogging) {
    const char *lbl = (currentLabel >= 0) ? CLASS_LABELS[currentLabel] : "none";
    ilLog(inp->data.f, totalElems, lbl);
  }

  /* Debug: print averaged input and per-step sensor variance for each feature. */
  for (uint8_t t = 0; t < prof.profileLen; t++) {
    Serial.print("Input[");
    Serial.print(t);
    Serial.print("]:");
    int base = t * NUM_FEATURES;
    for (int i = 0; i < NUM_FEATURES; i++) {
      Serial.print(" ");
      Serial.print(inp->data.f[base + i]);
    }
    Serial.print("  Var:");
    for (int i = 0; i < NUM_FEATURES; i++) {
      float mean = inp->data.f[base + i];
      float var = sumSq[base + i] / (float)numActiveSensors - mean * mean;
      Serial.print(" ");
      // Serial.print(max(0.0f, var), 4);
      Serial.print(var, 4);
    }
    Serial.println();
  }

  /* Time the inference call with microsecond precision. */
  uint32_t t_inf0 = micros();
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERR: Invoke() failed.");
    return;
  }
  uint32_t t_inf1 = micros();

  TfLiteTensor *out = interpreter->output(0);
  int predicted = 0;
  float best = out->data.f[0];
  for (int i = 1; i < NUM_CLASSES; i++) {
    if (out->data.f[i] > best) {
      best = out->data.f[i];
      predicted = i;
    }
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
  recordResult(predicted);
  uint32_t collectMs = millis() - collectStart;
  uint32_t inferUs = t_inf1 - t_inf0;
  timingCollectSum += collectMs;
  timingInferSum += inferUs;
  timingCount++;
  Serial.print("Timing — collect: ");
  Serial.print(collectMs);
  Serial.print(" ms  |  inference: ");
  Serial.print(inferUs);
  Serial.println(" us");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Baseline capture  (sequential / parallel mode)
 * ═══════════════════════════════════════════════════════════════════════ */

/* Accumulates one sensor-averaged measurement into configAccum[].
 * When configTarget measurements have been collected, finalises baselineFp[]
 * and exits config mode. */
void captureBaselineSample() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  for (uint8_t t = 0; t < prof.profileLen; t++) {
    float stepSum[NUM_FEATURES] = { 0 };
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      SensorState &ss = sensorStates[s];
      const float raw[NUM_FEATURES] = {
        // ss.fpTemp,
        // ss.fpPres,
        ss.fpHum,
        ss.fpGas[t],
        // ss.fpDiffTemp,
        // ss.fpDiffPres,
        ss.fpDiffHum,
        ss.fpGasDiff[t],
      };
      for (int i = 0; i < NUM_FEATURES; i++) stepSum[i] += raw[i];
    }
    int base = t * NUM_FEATURES;
    for (int i = 0; i < NUM_FEATURES; i++)
      configAccum[base + i] += stepSum[i] / (float)numActiveSensors;
  }

  configCount++;
  Serial.println("[CONFIG] " + String(configCount) + "/" + String(configTarget) + " measurement(s) collected.");

  if (configCount < configTarget) return;

  /* Finalise: divide sums by number of measurements. */
  const int totalElems = prof.profileLen * NUM_FEATURES;
  for (int i = 0; i < totalElems; i++)
    baselineFp[i] = configAccum[i] / (float)configTarget;
  baselineSeqLen = prof.profileLen;
  baselineReady = true;
  configuring = false;

  for (uint8_t s = 0; s < numActiveSensors; s++)
    sensorStates[s].bme.setOpMode(BME68X_SLEEP_MODE);

  Serial.println("[CONFIG] Baseline saved (" + String(configTarget) + " measurements, " + String(prof.profileLen) + " steps):");
  for (uint8_t t = 0; t < prof.profileLen; t++) {
    Serial.print("  Step[");
    Serial.print(t);
    Serial.print("]:");
    int base = t * NUM_FEATURES;
    for (int i = 0; i < NUM_FEATURES; i++) {
      Serial.print(" ");
      Serial.print(baselineFp[base + i]);
    }
    Serial.println();
  }
  Serial.println(">> Baseline ready. Send 'run' to start inference.");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Per-sensor polling
 * ═══════════════════════════════════════════════════════════════════════ */

/* Polls one sensor for new sequential-mode samples.
 * When a complete, validated cycle is detected it freezes the fingerprint
 * into the fp* fields and sets fingerprintReady = true.
 * If fingerprintReady is already true (waiting for other sensors to catch up)
 * the current cycle is still accumulated into gasBuffer but no new freeze
 * is attempted, so the previous fingerprint remains intact. */
void pollSensor(uint8_t s) {
  const HeatingProfile &prof = PROFILES[activeProfile];
  SensorState &ss = sensorStates[s];

  uint8_t nFields = ss.bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    ss.bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Wrap-around boundary: a new cycle has just begun. */
    if (ss.lastGasIdx >= 0 && (int8_t)gi < ss.lastGasIdx) {
      /* Only freeze if we haven't already frozen a fingerprint waiting for
       * other sensors. If fingerprintReady is already set, discard this
       * intermediate cycle without overwriting the frozen data. */
      if (ss.cycleHasData && !ss.fingerprintReady) {
        bool ok;
        if (imputationEnabled) {
          imputeMissingSensorSteps(s, prof.profileLen);
          ok = true;
        } else {
          ok = validateSensorScan(s, prof.profileLen);
        }
        if (ok) {
          memcpy(ss.fpGas, ss.gasBuffer, prof.profileLen * sizeof(float));
          memcpy(ss.fpGasDiff, ss.gasDiffBuffer, prof.profileLen * sizeof(float));
          ss.fpTemp = ss.lastTemp;
          ss.fpHum = ss.lastHum;
          ss.fpPres = ss.lastPres;
          ss.fpDiffTemp = ss.diffTemp;
          ss.fpDiffHum = ss.diffHum;
          ss.fpDiffPres = ss.diffPres;
          ss.fingerprintReady = true;
        }
      }
      ss.cycleHasData = false;
      resetSensorGasBuffer(s, prof.profileLen);
    }

    ss.diffTemp = (ss.lastTemp == 0.0f) ? 0.0f : (data.temperature - ss.lastTemp);
    ss.diffHum = (ss.lastHum == 0.0f) ? 0.0f : (data.humidity - ss.lastHum);
    ss.diffPres = (ss.lastPres == 0.0f) ? 0.0f : (data.pressure / 100.0f - ss.lastPres);

    ss.lastTemp = data.temperature;
    ss.lastHum = data.humidity;
    ss.lastPres = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      ss.gasDiffBuffer[gi] = (gi == 0) ? 0.0f
                                       : (ss.gasReceived[gi - 1]
                                            ? (data.gas_resistance - ss.gasBuffer[gi - 1])
                                            : 0.0f);
      ss.gasBuffer[gi] = data.gas_resistance;
      ss.gasReceived[gi] = true;
      ss.cycleHasData = true;
    }

    ss.lastGasIdx = (int8_t)gi;
  }
}

/* Poll all active sensors. When every sensor has a ready fingerprint,
 * run averaged inference and begin the next collection window. */
void pollAllSensors() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  for (uint8_t s = 0; s < numActiveSensors; s++) {
    pollSensor(s);
  }

  bool allReady = true;
  for (uint8_t s = 0; s < numActiveSensors; s++) {
    if (!sensorStates[s].fingerprintReady) {
      allReady = false;
      break;
    }
  }

  if (!allReady) return;

  if (configuring) {
    captureBaselineSample();
  } else {
    Serial.print("[");
    Serial.print(prof.name);
    Serial.print("] ");
    Serial.print(numActiveSensors);
    Serial.print(" sensor(s) complete — ");
    runAveragedInference();
  }

  /* Reset all fingerprint flags; sensors continue accumulating their next cycles. */
  for (uint8_t s = 0; s < numActiveSensors; s++) {
    sensorStates[s].fingerprintReady = false;
  }
  collectStart = millis();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Inference  (forced mode)
 * ═══════════════════════════════════════════════════════════════════════ */

/* Triggers one forced measurement on every active sensor, z-scores and
 * averages the features, then runs inference. */
void takeForcedMeasurements() {
  TfLiteTensor *inp = interpreter->input(0);
  for (int i = 0; i < NUM_FEATURES; i++) inp->data.f[i] = 0.0f;
  float sumSq[NUM_FEATURES] = { 0 };

  uint8_t validCount = 0;

  for (uint8_t s = 0; s < numActiveSensors; s++) {
    SensorState &ss = sensorStates[s];
    ss.bme.setOpMode(BME68X_FORCED_MODE);
    delayMicroseconds(ss.bme.getMeasDur(BME68X_FORCED_MODE));

    bme68xData data;
    if (!ss.bme.fetchData()) continue;
    ss.bme.getData(data);

    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;
    if (!(data.status & BME68X_GASM_VALID_MSK)) continue;
    if (!(data.status & BME68X_HEAT_STAB_MSK)) continue;

    Serial.print("[S");
    Serial.print(activeSensors[s] + 1);
    Serial.print("] dT=");
    Serial.print(data.temperature - FEATURE_MEAN[0][0], 1);
    Serial.print(" dP=");
    Serial.print(data.pressure / 100.0f - FEATURE_MEAN[0][1], 1);
    Serial.print(" dH=");
    Serial.print(data.humidity - FEATURE_MEAN[0][2], 1);
    Serial.print(" dG=");
    Serial.println(data.gas_resistance - FEATURE_MEAN[0][3], 0);

    /* Diff features are 0 for forced mode (single-step, no sequence). */
    const float raw[NUM_FEATURES] = {
      // data.temperature,
      // data.pressure / 100.0f,
      data.humidity,
      data.gas_resistance,
      // 0.0f, 0.0f,
      0.0f,
      0.0f,
    };

    for (int i = 0; i < NUM_FEATURES; i++) {
      float v;
      if (PREPROCESSING_TYPE == "STANDARD") {
        v = (FEATURE_STD[0][i] != 0.0f) ? standardise(raw[i], 0, i) : raw[i];
      } else if (PREPROCESSING_TYPE == "BASELINE") {
        v = fractional_baseline_correction(raw[i], 0, i);
      } else if (PREPROCESSING_TYPE == "BASELINE_STD") {
        v = fractional_baseline_correction(raw[i], 0, i);
        v = (FEATURE_STD[0][i] != 0.0f) ? standardise(v, 0, i) : v;
      } else {
        v = raw[i];
      }
      inp->data.f[i] += v;
      sumSq[i] += v * v;
    }
    validCount++;
  }

  if (validCount == 0) return;

  /* Average across sensors. */
  for (int i = 0; i < NUM_FEATURES; i++) {
    inp->data.f[i] /= (float)validCount;
  }

  /* SD debug log — record the averaged, standardised input vector. */
  if (ilLogging) {
    const char *lbl = (currentLabel >= 0) ? CLASS_LABELS[currentLabel] : "none";
    ilLog(inp->data.f, NUM_FEATURES, lbl);
  }

  /* Sensor variance per feature (in standardised space). */
  Serial.print("Var:");
  for (int i = 0; i < NUM_FEATURES; i++) {
    float mean = inp->data.f[i];
    float var = sumSq[i] / (float)validCount - mean * mean;
    Serial.print(" ");
    // Serial.print(max(0.0f, var), 4);
    Serial.print(var, 4);
  }
  Serial.println();

  uint32_t t_inf0 = micros();
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("ERR: Invoke() failed.");
    return;
  }
  uint32_t t_inf1 = micros();

  TfLiteTensor *out = interpreter->output(0);
  int predicted = 0;
  float best = out->data.f[0];
  for (int i = 1; i < NUM_CLASSES; i++) {
    if (out->data.f[i] > best) {
      best = out->data.f[i];
      predicted = i;
    }
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
  recordResult(predicted);
  uint32_t collectMs = millis() - collectStart;
  uint32_t inferUs = t_inf1 - t_inf0;
  timingCollectSum += collectMs;
  timingInferSum += inferUs;
  timingCount++;
  Serial.print("Timing — collect: ");
  Serial.print(collectMs);
  Serial.print(" ms  |  inference: ");
  Serial.print(inferUs);
  Serial.println(" us");
  collectStart = millis();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Baseline capture  (forced mode)
 * ═══════════════════════════════════════════════════════════════════════ */

void captureBaselineForcedSample() {
  float sensorSum[NUM_FEATURES] = { 0 };
  uint8_t validCount = 0;

  for (uint8_t s = 0; s < numActiveSensors; s++) {
    SensorState &ss = sensorStates[s];
    ss.bme.setOpMode(BME68X_FORCED_MODE);
    delayMicroseconds(ss.bme.getMeasDur(BME68X_FORCED_MODE));

    bme68xData data;
    if (!ss.bme.fetchData()) continue;
    ss.bme.getData(data);

    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;
    if (!(data.status & BME68X_GASM_VALID_MSK)) continue;
    if (!(data.status & BME68X_HEAT_STAB_MSK)) continue;

    sensorSum[0] += data.temperature;
    sensorSum[1] += data.pressure / 100.0f;
    sensorSum[2] += data.humidity;
    sensorSum[3] += data.gas_resistance;
    sensorSum[4] += 0.0f;
    validCount++;
  }

  if (validCount == 0) return;

  for (int i = 0; i < NUM_FEATURES; i++)
    configAccum[i] += sensorSum[i] / (float)validCount;

  configCount++;
  Serial.println("[CONFIG] " + String(configCount) + "/" + String(configTarget) + " measurement(s) collected.");

  if (configCount < configTarget) return;

  for (int i = 0; i < NUM_FEATURES; i++)
    baselineFp[i] = configAccum[i] / (float)configTarget;
  baselineSeqLen = 1;
  baselineReady = true;
  configuring = false;

  Serial.print("[CONFIG] Baseline saved:");
  for (int i = 0; i < NUM_FEATURES; i++) {
    Serial.print(" ");
    Serial.print(baselineFp[i]);
  }
  Serial.println();
  Serial.println(">> Baseline ready. Send 'run' to start inference.");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Sensor initialisation
 * ═══════════════════════════════════════════════════════════════════════ */

bool initSensors() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  /* Initialise the first sensor to get the shared heater duration for
   * parallel mode (depends on TPH settings which are the same for all). */
  sharedHeatrDur = 0;

  for (uint8_t s = 0; s < numActiveSensors; s++) {
    uint8_t idx = activeSensors[s];
    SensorState &ss = sensorStates[s];

    ss.comm = commMuxSetConfig(Wire, SPI, idx, ss.comm);
    ss.bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &ss.comm);

    if (ss.bme.checkStatus()) {
      Serial.println("ERR: Sensor " + String(idx + 1) + " init failed — " + ss.bme.statusString());
      return false;
    }

    ss.bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);
    ss.bme.setFilter(BME68X_FILTER_SIZE_3);

    if (prof.scanMode == SCAN_PARALLEL) {
      uint16_t tArr[10], mArr[10];
      memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
      memcpy(mArr, prof.mulProf, prof.profileLen * sizeof(uint16_t));
      if (s == 0) {
        sharedHeatrDur = 140 - (ss.bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);
      }
      ss.bme.setHeaterProf(tArr, mArr, sharedHeatrDur, prof.profileLen);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      uint16_t tArr[10], dArr[10];
      memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
      memcpy(dArr, prof.mulProf, prof.profileLen * sizeof(uint16_t));
      ss.bme.setSeqSleep(prof.seqSleep);
      ss.bme.setHeaterProf(tArr, dArr, prof.profileLen);
    } else {
      ss.bme.setHeaterProf(prof.forcedTemp, prof.forcedDur);
    }

    /* Reset per-sensor state. */
    ss.lastTemp = 0;
    ss.lastHum = 0;
    ss.lastPres = 0;
    ss.diffTemp = 0;
    ss.diffHum = 0;
    ss.diffPres = 0;
    ss.lastGasIdx = -1;
    ss.cycleHasData = false;
    ss.fingerprintReady = false;
    resetSensorGasBuffer(s, prof.profileLen > 0 ? prof.profileLen : 1);
  }
  return true;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Cleaning cycle
 * ═══════════════════════════════════════════════════════════════════════ */

/* Called from loop() while cleaning == true.
 * Fires one 200 ms forced-mode pulse at CLEAN_TEMP_C every 4 s (5% duty cycle),
 * printing progress every pulse. Stops sensors and resets the flag when done. */
void runCleanCycle() {
  uint32_t now = millis();
  uint32_t elapsed = now - cleanStart;

  if (elapsed >= cleanDuration) {
    for (uint8_t s = 0; s < numActiveSensors; s++)
      sensorStates[s].bme.setOpMode(BME68X_SLEEP_MODE);
    cleaning = false;
    Serial.println(">> Cleaning complete — sensors idle. Send 'run' to start inference.");
    return;
  }

  /* Fire a pulse if the gap has elapsed (or at t=0 on the first call). */
  if (now - cleanLastPulse >= CLEAN_GAP_MS) {
    cleanLastPulse = now;
    uint32_t remaining = (cleanDuration - elapsed + 999) / 1000;
    Serial.println(">> Clean pulse — " + String(elapsed / 1000) + "/" + String(cleanDuration / 1000) + " s  (" + String(remaining) + " s left)");

    for (uint8_t s = 0; s < numActiveSensors; s++) {
      sensorStates[s].bme.setOpMode(BME68X_FORCED_MODE);
    }
    /* Wait for the heater to complete its on-time, then let sensors cool. */
    delay(CLEAN_DUR_MS);
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      bme68xData data;
      sensorStates[s].bme.fetchData();
      sensorStates[s].bme.getData(data); /* discard — we only care about heat */
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Command handling
 * ═══════════════════════════════════════════════════════════════════════ */

/* Parse a comma-separated list of sensor numbers (1-based) into activeSensors[].
 * E.g. "1,3,5" → {0, 2, 4}, numActiveSensors = 3. */
void parseSensorList(const String &list) {
  uint8_t parsed[MAX_ACTIVE];
  uint8_t count = 0;
  int start = 0;
  while (start <= (int)list.length() && count < MAX_ACTIVE) {
    int comma = list.indexOf(',', start);
    String token = (comma < 0) ? list.substring(start) : list.substring(start, comma);
    token.trim();
    int n = token.toInt();
    if (n >= 1 && n <= 8) {
      parsed[count++] = (uint8_t)(n - 1);
    } else if (token.length() > 0) {
      Serial.println("[WARN] Ignoring invalid sensor number: " + token);
    }
    if (comma < 0) break;
    start = comma + 1;
  }
  if (count == 0) {
    Serial.println("[WARN] No valid sensors specified. Keeping current selection.");
    return;
  }
  memcpy(activeSensors, parsed, count);
  numActiveSensors = count;
}

void handleCommand(const String &cmd) {

  /* ── run ───────────────────────────────────────────────────────────── */
  if (cmd == "run") {
    if (running) {
      Serial.println(">> Already running. Send 'stop' first.");
      return;
    }
    if (!initSensors()) {
      Serial.println(">> Sensor init failed — cannot start.");
      return;
    }
    const HeatingProfile &prof = PROFILES[activeProfile];

    if (prof.scanMode == SCAN_PARALLEL) {
      for (uint8_t s = 0; s < numActiveSensors; s++) {
        sensorStates[s].bme.setOpMode(BME68X_PARALLEL_MODE);
      }
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      for (uint8_t s = 0; s < numActiveSensors; s++) {
        sensorStates[s].bme.setOpMode(BME68X_SEQUENTIAL_MODE);
      }
    }
    /* Forced mode: op-mode is set per measurement in takeForcedMeasurements(). */

    running = true;
    collectStart = millis();

    Serial.print(">> Running  profile " + String(activeProfile + 1) + " [" + String(prof.name) + "]  sensors: ");
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      if (s > 0) Serial.print(",");
      Serial.print(activeSensors[s] + 1);
    }
    Serial.println();

    /* ── stop ──────────────────────────────────────────────────────────── */
  } else if (cmd == "stop") {
    if (!running && !cleaning && !configuring) {
      Serial.println(">> Not running.");
      return;
    }
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      sensorStates[s].bme.setOpMode(BME68X_SLEEP_MODE);
    }
    running = false;
    cleaning = false;
    configuring = false;
    Serial.println(">> Stopped.");

    /* ── clean [<seconds>] ─────────────────────────────────────────────── */
  } else if (cmd == "clean" || cmd.startsWith("clean ")) {
    if (running) {
      Serial.println(">> Stop inference before cleaning.");
      return;
    }
    if (cleaning) {
      Serial.println(">> Already cleaning. Send 'stop' to abort.");
      return;
    }
    uint32_t sec = CLEAN_DEF_SEC;
    if (cmd.startsWith("clean ")) {
      int n = cmd.substring(6).toInt();
      if (n > 0) sec = (uint32_t)n;
    }
    cleanDuration = sec * 1000UL;

    /* Initialise sensors in forced mode at the cleaning temperature. */
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      uint8_t idx = activeSensors[s];
      SensorState &ss = sensorStates[s];
      ss.comm = commMuxSetConfig(Wire, SPI, idx, ss.comm);
      ss.bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &ss.comm);
      if (ss.bme.checkStatus()) {
        Serial.println("ERR: Sensor " + String(idx + 1) + " init failed — " + ss.bme.statusString());
        return;
      }
      ss.bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);
      ss.bme.setHeaterProf(CLEAN_TEMP_C, CLEAN_DUR_MS);
    }

    cleaning = true;
    cleanStart = millis();
    cleanLastPulse = cleanStart - CLEAN_GAP_MS; /* fire first pulse immediately */
    Serial.println(">> Cleaning cycle started — " + String(sec) + " s at " + String(CLEAN_TEMP_C) + " C  (5% duty cycle, one pulse every 4 s).");
    Serial.println("   Send 'stop' to abort.");

    /* ── profile <N> ───────────────────────────────────────────────────── */
  } else if (cmd.startsWith("profile ")) {
    if (running) {
      Serial.println(">> Stop before changing profile.");
      return;
    }
    int n = cmd.substring(8).toInt();
    if (n < 1 || n > (int)NUM_PROFILES) {
      Serial.println(">> Profile must be 1–" + String(NUM_PROFILES) + ". Send 'profiles' to list.");
    } else {
      activeProfile = (uint8_t)(n - 1);
      Serial.println(">> Profile set to " + String(n) + " [" + String(PROFILES[activeProfile].name) + "]");
    }

    /* ── profiles ──────────────────────────────────────────────────────── */
  } else if (cmd == "profiles") {
    Serial.println("--- Available heating profiles ---");
    for (uint8_t i = 0; i < NUM_PROFILES; i++) {
      Serial.println("  " + String(i + 1) + ": " + String(PROFILES[i].name) + "  " + String(PROFILES[i].description));
    }
    Serial.println("----------------------------------");

    /* ── sensors [list] ────────────────────────────────────────────────── */
  } else if (cmd == "sensors") {
    Serial.print("Active sensors: ");
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      if (s > 0) Serial.print(", ");
      Serial.print(activeSensors[s] + 1);
    }
    Serial.println();
    Serial.println("  Use 'sensors <list>' to change, e.g.: sensors 1,2,3");

  } else if (cmd.startsWith("sensors ")) {
    if (running) {
      Serial.println(">> Stop before changing sensor selection.");
      return;
    }
    parseSensorList(cmd.substring(8));
    Serial.print(">> Active sensors: ");
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      if (s > 0) Serial.print(", ");
      Serial.print(activeSensors[s] + 1);
    }
    Serial.println();

    /* ── label [N] ─────────────────────────────────────────────────────── */
  } else if (cmd == "label") {
    if (currentLabel < 0) {
      Serial.println(">> No label set. Use 'label <N>' to set one.");
    } else {
      Serial.println(">> Current label: " + String(currentLabel + 1) + " [" + CLASS_LABELS[currentLabel] + "]");
    }
    Serial.println("   Classes:");
    for (int i = 0; i < NUM_CLASSES; i++) {
      Serial.println("   " + String(i + 1) + ": " + CLASS_LABELS[i]);
    }

  } else if (cmd.startsWith("label ")) {
    String arg = cmd.substring(6);
    arg.trim();
    if (arg == "off" || arg == "0") {
      currentLabel = -1;
      Serial.println(">> Label cleared (unlabelled mode).");
    } else {
      int n = arg.toInt();
      if (n >= 1 && n <= NUM_CLASSES) {
        currentLabel = (int8_t)(n - 1);
        Serial.println(">> Label set to " + String(n) + " [" + CLASS_LABELS[currentLabel] + "]");
      } else {
        Serial.println(">> Label must be 1–" + String(NUM_CLASSES) + " or 'off'. Classes:");
        for (int i = 0; i < NUM_CLASSES; i++) {
          Serial.println("   " + String(i + 1) + ": " + CLASS_LABELS[i]);
        }
      }
    }

    /* ── accuracy ──────────────────────────────────────────────────────── */
  } else if (cmd == "accuracy") {
    Serial.println("--- Per-class accuracy ---");
    uint32_t totalInf = 0, totalCorrect = 0;
    for (int i = 0; i < NUM_CLASSES; i++) {
      if (labelTotal[i] == 0) {
        Serial.println("  " + String(CLASS_LABELS[i]) + ": no data");
      } else {
        float acc = 100.0f * (float)labelCorrect[i] / (float)labelTotal[i];
        Serial.println("  " + String(CLASS_LABELS[i]) + ": " + String(labelCorrect[i]) + "/" + String(labelTotal[i]) + "  (" + String(acc, 1) + "%)");
        totalInf += labelTotal[i];
        totalCorrect += labelCorrect[i];
      }
    }
    if (totalInf > 0) {
      Serial.println("  Overall: " + String(totalCorrect) + "/" + String(totalInf) + "  (" + String(100.0f * (float)totalCorrect / (float)totalInf, 1) + "%)");
    } else {
      Serial.println("  No labelled inferences yet.");
    }
    Serial.println("--------------------------");

    /* ── timing [reset] ────────────────────────────────────────────────── */
  } else if (cmd == "timing") {
    if (timingCount == 0) {
      Serial.println("No timing data yet.");
    } else {
      Serial.println("--- Timing averages (" + String(timingCount) + " samples) ---");
      Serial.print("  Collect avg : ");
      Serial.print((float)timingCollectSum / timingCount, 1);
      Serial.println(" ms");
      Serial.print("  Infer  avg  : ");
      Serial.print((float)timingInferSum / timingCount, 1);
      Serial.println(" us");
      Serial.println("------------------------------");
    }

  } else if (cmd == "timing reset") {
    timingCount = timingCollectSum = timingInferSum = 0;
    Serial.println(">> Timing stats reset.");

    /* ── status ────────────────────────────────────────────────────────── */
  } else if (cmd == "status") {
    const HeatingProfile &prof = PROFILES[activeProfile];
    if (cleaning) {
      uint32_t elapsed = (millis() - cleanStart) / 1000;
      Serial.println("Running : cleaning  (" + String(elapsed) + "/" + String(cleanDuration / 1000) + " s)");
    } else if (configuring) {
      Serial.println("Running : config  (" + String(configCount) + "/" + String(configTarget) + " measurements)");
    } else {
      Serial.println("Running : " + String(running ? "yes" : "no"));
    }
    Serial.print("Sensors : ");
    for (uint8_t s = 0; s < numActiveSensors; s++) {
      if (s > 0) Serial.print(", ");
      Serial.print(activeSensors[s] + 1);
    }
    Serial.println();
    Serial.println("Profile : " + String(activeProfile + 1) + " [" + String(prof.name) + "] — " + String(prof.description));
    if (currentLabel >= 0) {
      Serial.println("Label   : " + String(currentLabel + 1) + " [" + CLASS_LABELS[currentLabel] + "]");
    } else {
      Serial.println("Label   : (none)");
    }
    Serial.println("Impute  : " + String(imputationEnabled ? "ON" : "OFF"));
    Serial.println("Baseline: " + String(baselineReady ? "ready (" + String(baselineSeqLen) + " step(s))" : "not set"));

    /* ── logfile [<name>] ──────────────────────────────────────────────── */
  } else if (cmd == "logfile") {
    Serial.println(">> Log file : " + ilFilename + ".csv");
    Serial.println("   Logging  : " + String(ilLogging ? "active" : "stopped"));
    Serial.println("   Use 'logfile <name>' to change filename (while not logging).");

  } else if (cmd.startsWith("logfile ")) {
    if (ilLogging) {
      Serial.println(">> Stop logging first ('logstop').");
      return;
    }
    ilFilename = cmd.substring(8);
    ilFilename.trim();
    Serial.println(">> Log filename set to: " + ilFilename + ".csv");

    /* ── logstart ───────────────────────────────────────────────────────
   * Opens the CSV file and enables per-inference logging.
   * The number of input columns is derived from the current profile so
   * the header matches the data.  Change the profile before 'logstart',
   * not after, to keep the file consistent. */
  } else if (cmd == "logstart") {
    if (ilLogging) {
      Serial.println(">> Already logging. Send 'logstop' first.");
      return;
    }
    const HeatingProfile &prof = PROFILES[activeProfile];
    int numInputs = (prof.scanMode == SCAN_FORCED)
                      ? NUM_FEATURES
                      : (int)prof.profileLen * NUM_FEATURES;
    if (ilOpen(ilFilename, numInputs)) {
      ilLogging = true;
      Serial.println(">> SD logging started — " + ilFilename + ".csv" + "  (" + String(numInputs) + " inputs/row).");
    }

    /* ── logstop ────────────────────────────────────────────────────────── */
  } else if (cmd == "logstop") {
    if (!ilLogging) {
      Serial.println(">> Not logging.");
      return;
    }
    ilClose();
    ilLogging = false;

    /* ── config [<N>] ──────────────────────────────────────────────────── */
  } else if (cmd == "config" || cmd.startsWith("config ")) {
    if (running || cleaning) {
      Serial.println(">> Stop inference/cleaning before configuring.");
      return;
    }
    uint8_t n = 5;
    if (cmd.startsWith("config ")) {
      int parsed = cmd.substring(7).toInt();
      if (parsed > 0 && parsed <= 255) n = (uint8_t)parsed;
    }
    configTarget = n;
    configCount = 0;
    memset(configAccum, 0, sizeof(configAccum));

    if (!initSensors()) {
      Serial.println(">> Sensor init failed — cannot start config.");
      return;
    }
    const HeatingProfile &prof = PROFILES[activeProfile];
    if (prof.scanMode == SCAN_PARALLEL) {
      for (uint8_t s = 0; s < numActiveSensors; s++)
        sensorStates[s].bme.setOpMode(BME68X_PARALLEL_MODE);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      for (uint8_t s = 0; s < numActiveSensors; s++)
        sensorStates[s].bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    }
    /* Forced mode: op-mode is set per measurement in captureBaselineForcedSample(). */

    configuring = true;
    collectStart = millis();
    Serial.println(">> Config mode: collecting " + String(n) + " measurement(s) on profile [" + String(prof.name) + "]...");
    Serial.println("   Send 'stop' to abort.");

    /* ── impute [on|off] ────────────────────────────────────────────────── */
  } else if (cmd == "impute") {
    Serial.println(">> Imputation: " + String(imputationEnabled ? "ON" : "OFF"));
    Serial.println("   Use 'impute on' or 'impute off' to toggle.");

  } else if (cmd == "impute on") {
    imputationEnabled = true;
    Serial.println(">> Imputation enabled — missing steps filled from peer sensors.");

  } else if (cmd == "impute off") {
    imputationEnabled = false;
    Serial.println(">> Imputation disabled — incomplete cycles dropped (original behaviour).");

    /* ── unknown ───────────────────────────────────────────────────────── */
  } else {
    Serial.println(">> Unknown: '" + cmd + "'");
    Serial.println("   Commands: run | stop | clean [<sec>] | profile <N> | profiles");
    Serial.println("             sensors [list] | label [N] | accuracy | timing [reset] | status");
    Serial.println("             logfile [<name>] | logstart | logstop | impute [on|off]");
    Serial.println("             config [<N>]");
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
  ilInit();

  if (!initSensors()) {
    Serial.println("Halting.");
    while (1)
      ;
  }
  Serial.print("Sensor(s) OK: ");
  for (uint8_t s = 0; s < numActiveSensors; s++) {
    if (s > 0) Serial.print(", ");
    Serial.print(activeSensors[s] + 1);
  }
  Serial.println();

  /* ── TFLite Micro ────────────────────────────────────────────────── */
  const tflite::Model *tfl_model = tflite::GetModel(g_model);

  if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERR: Model schema version mismatch ("
                   + String(tfl_model->version()) + " vs expected "
                   + String(TFLITE_SCHEMA_VERSION) + ")");
    while (1)
      ;
  }

  if (MODEL_TYPE == "LSTM") {
    resolver.AddUnpack();
    resolver.AddFullyConnected();
    resolver.AddAdd();
    resolver.AddSplit();
    resolver.AddMul();
    resolver.AddLogistic();
    resolver.AddTanh();
    resolver.AddSoftmax();
    resolver.AddStridedSlice();
    resolver.AddPack();
    resolver.AddTranspose();
    resolver.AddReshape();
  }

  if (MODEL_TYPE == "CNN") {
    resolver.AddReshape();
    resolver.AddConv2D();
    resolver.AddMean();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
  }

  static tflite::MicroInterpreter static_interpreter(
    tfl_model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERR: AllocateTensors() failed — arena may be too small.");
    while (1)
      ;
  }

  TfLiteTensor *inp = interpreter->input(0);
  TfLiteTensor *out = interpreter->output(0);
  String inpShape = "";
  for (int d = 0; d < inp->dims->size; d++) {
    if (d > 0) inpShape += "x";
    inpShape += String(inp->dims->data[d]);
  }
  Serial.println("Model OK  |  input " + inpShape
                 + "  output "
                 + String(out->dims->data[0]) + "x" + String(out->dims->data[1]));
  Serial.println("Arena used: " + String(interpreter->arena_used_bytes()) + " / " + String(kTensorArenaSize) + " bytes");
  Serial.println("Active profile: 1 [" + String(PROFILES[0].name) + "]");
  Serial.println("Commands: run | stop | clean [<sec>] | profile <N> | profiles");
  Serial.println("          sensors [list] | label [N] | accuracy | timing [reset] | status");
  Serial.println("          logfile [<name>] | logstart | logstop | impute [on|off]");
  Serial.println("          config [<N>]");
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

  if (cleaning) {
    runCleanCycle();
    return;
  }

  if (configuring) {
    const HeatingProfile &prof = PROFILES[activeProfile];
    if (prof.scanMode == SCAN_SEQUENTIAL) {
      pollAllSensors();
      delay(10);
    } else {
      captureBaselineForcedSample();
      delay(500);
    }
    return;
  }

  if (!running) return;

  const HeatingProfile &prof = PROFILES[activeProfile];

  if (prof.scanMode == SCAN_PARALLEL) {
    Serial.println("[WARN] Parallel inference not implemented.");
    delay(10);
  } else if (prof.scanMode == SCAN_SEQUENTIAL) {
    pollAllSensors();
    delay(10);
  } else {
    takeForcedMeasurements();
    delay(500);
  }
}
