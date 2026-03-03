/**
 * Data Collection — BME688 Dev Kit
 *
 * Plan-based data collection: runs a pre-defined sequence of timed recording
 * stages separated by optional cleaning phases.  The user controls progression
 * via the 'next' serial command.
 *
 * ── Collection plan ───────────────────────────────────────────────────────
 *   Edit the PLAN[] array (near the top of this file) to define each step:
 *     filename            — CSV output filename (no extension)
 *     label               — class label written to every row
 *     durationSec         — recording duration in seconds (auto-stops)
 *     sensorProfiles[8]   — 1-based heating profile ID per sensor; 0 = inactive
 *     cleaningDurationSec — sensors run unlogged for this many seconds after
 *                           the recording stage ends.  0 = skip cleaning.
 *
 * ── Stage sequence ────────────────────────────────────────────────────────
 *   start              → WAITING; send 'next' to begin step 1
 *   next               → RECORDING (timed, auto-stops)
 *   recording done     → CLEANING (timed) → WAITING; send 'next' → …
 *   all steps done     → IDLE, "Collection complete"
 *
 * ── Serial commands ───────────────────────────────────────────────────────
 *   start    — begin data collection
 *   next     — start recording the current step (when WAITING)
 *   stop     — abort and return to idle
 *   verbose  — toggle per-reading output to Serial
 *   status   — print current state and timing
 *   plan     — list all plan steps
 *   profiles — list all available heating profiles
 *
 * Required libraries: bme68xLibrary (Bosch Sensortec), SD (Arduino built-in)
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"
#include "sdLogger.h"

/* Both bits must be set for a gas reading to be valid at the intended temp. */
#define GAS_VALID_MSK (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define LED_PIN LED_BUILTIN

/* ═══════════════════════════════════════════════════════════════════════
 * Heating profile definitions
 * ═══════════════════════════════════════════════════════════════════════ */

enum ScanMode { SCAN_FORCED, SCAN_PARALLEL, SCAN_SEQUENTIAL };

struct HeatingProfile {
  const char *name;
  const char *description;
  ScanMode    scanMode;
  uint16_t    forcedTemp;
  uint16_t    forcedDur;
  uint8_t     profileLen;
  uint16_t    tempProf[10];
  uint16_t    mulProf[10];
  uint8_t     seqSleep;
};

const HeatingProfile PROFILES[] = {

  /* ── Forced mode ──────────────────────────────────────────────────── */
  { "Forced_Std",    "[F] 320 C / 150 ms  (default)", SCAN_FORCED, 320, 150, 0, {0}, {0}, 0 },
  { "Forced_Low",    "[F] 200 C / 150 ms",             SCAN_FORCED, 200, 150, 0, {0}, {0}, 0 },
  { "Forced_High",   "[F] 400 C / 200 ms",             SCAN_FORCED, 400, 200, 0, {0}, {0}, 0 },

  /* ── Parallel mode ────────────────────────────────────────────────── */
  {
    "Par_Bosch",
    "[P] Bosch standard 10-step: 320,100,100,100,200,200,200,320,320,320 C",
    SCAN_PARALLEL, 0, 0, 10,
    {320,100,100,100,200,200,200,320,320,320}, {5,2,10,30,5,5,5,5,5,5}, 0
  },
  {
    "Par_LinSweep",
    "[P] Linear sweep  200 -> 400 C  (10 equal steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {200,222,244,267,289,311,333,356,378,400}, {5,5,5,5,5,5,5,5,5,5}, 0
  },
  {
    "Par_WideSweep",
    "[P] Wide sweep    100 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100,150,200,250,300,325,350,380,415,450}, {10,8,7,6,5,5,5,5,5,5}, 0
  },
  {
    "Par_HighFocus",
    "[P] High-temp     300 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {300,317,333,350,367,383,400,417,433,450}, {5,5,5,5,5,5,5,5,5,5}, 0
  },
  {
    "Par_LowFocus",
    "[P] Low-temp      100 -> 250 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100,117,133,150,167,183,200,217,233,250}, {10,9,8,8,7,7,6,6,6,5}, 0
  },

  /* ── Sequential mode ──────────────────────────────────────────────── */
  {
    "Seq_Std",
    "[S] 100,200,320 C / 150 ms each, 250 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 3, {100,200,320}, {150,150,150}, BME68X_ODR_250_MS
  },
  {
    "Seq_LinSweep",
    "[S] Linear sweep 200->400 C (10 steps / 150 ms each, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {200,222,244,267,289,311,333,356,378,400},
    {150,150,150,150,150,150,150,150,150,150},
    BME68X_ODR_500_MS
  },
  {
    "Seq_Bosch",
    "[S] Bosch 10-step: 320,100x3,200x3,320x3 C / 150 ms each, 500 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {320,100,100,100,200,200,200,320,320,320},
    {150,150,150,150,150,150,150,150,150,150},
    BME68X_ODR_500_MS
  },
  {
    "Seq_WideSweep",
    "[S] Wide sweep 100->450 C (10 steps, longer dwell at low temps, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {100,150,200,250,300,325,350,380,415,450},
    {200,180,160,150,150,140,140,140,140,140},
    BME68X_ODR_500_MS
  },
};

const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

/* ═══════════════════════════════════════════════════════════════════════
 * Collection plan  ← edit this section
 * ═══════════════════════════════════════════════════════════════════════
 *
 * sensorProfiles[8]: 1-based profile ID per sensor slot (index 0 = sensor 1).
 *   Set a slot to 0 to exclude that sensor.
 *   Profile IDs: send 'profiles' over Serial to see the full list.
 *
 * cleaningDurationSec: wait time (seconds) after a recording stage ends before
 *   the next step becomes available.  0 = skip cleaning phase.
 */

struct CollectionStep {
  const char *filename;
  const char *label;
  uint32_t    durationSec;
  uint8_t     sensorProfiles[8];
  uint32_t    cleaningDurationSec;
};

const CollectionStep PLAN[] = {
  /* filename           label       dur(s)  sensors[8]                         clean(s) */
  //{ "setup",    "setup_air",        120,  { 9, 9, 11, 11, 4, 4, 8, 8 },      20 },
  { "lavender_baseline",    "lavender_baseline",        180,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
  { "lavender_sample", "lavender", 300,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
  { "grapefruit_baseline",    "grapefruit_baseline",      180,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
  { "grapefruit_sample",    "grapefruit",      300,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
  { "eucalyptus_baseline", "eucalyptus_baseline",   180,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
  { "eucalyptus_sample", "eucalyptus",   300,  { 9, 9, 11, 11, 9, 9, 11, 11 },      20 },
};

const uint8_t NUM_PLAN_STEPS = sizeof(PLAN) / sizeof(PLAN[0]);

/* ═══════════════════════════════════════════════════════════════════════
 * Global state
 * ═══════════════════════════════════════════════════════════════════════ */

Bme68x   bme;
commMux  commSetup;

uint8_t  sensorProfile[8]  = {0,0,0,0,0,0,0,0};
uint16_t sharedHeatrDur[8] = {0,0,0,0,0,0,0,0};

/* Sensors active in the current stage */
uint8_t  activeSensors[8];
uint8_t  activeSensorCount = 0;

enum Mode { IDLE, WAITING, CLEANING, RECORDING };
Mode     mode = IDLE;

uint8_t  planStep      = 0;   /* upcoming or current step index (0-based)   */
uint32_t stageStartMs  = 0;   /* millis() when RECORDING or CLEANING began  */
uint32_t cleaningDurMs = 0;   /* CLEANING phase duration (ms)               */

String   recordLabel = "";
bool     verboseMode = false;
String   cmdBuffer   = "";

/* Per-sensor parallel/sequential-mode cycle accumulation */
float    gasBuffer[8][10];
bool     gasReceived[8][10];
float    lastTemp[8]   = {0};
float    lastHum[8]    = {0};
float    lastPres[8]   = {0};
int8_t   lastGasIdx[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

uint32_t fingerprintIndex[8] = {0};
bool     cycleHasData[8]     = {false};

/* ── Row ring buffer ───────────────────────────────────────────────────
 * 8 sensors × 10 steps = 80 rows worst-case per iteration; 128 gives headroom. */
struct SdRow {
  uint8_t  sensorNum;
  uint32_t fpIdx;
  uint8_t  pos;
  uint16_t plateTemp;
  uint16_t heatDur;
  float    temperature;
  float    pressure;
  float    humidity;
  float    gas;
};
#define ROW_BUFFER_SIZE 128
static SdRow   rowBuffer[ROW_BUFFER_SIZE];
static uint8_t rowHead  = 0;
static uint8_t rowTail  = 0;
static uint8_t rowCount = 0;

/* LED blink (1 Hz during RECORDING) */
uint32_t lastLedToggle = 0;
bool     ledState      = false;

/* ═══════════════════════════════════════════════════════════════════════
 * LED helper
 * ═══════════════════════════════════════════════════════════════════════ */

void updateLed() {
  if (mode == RECORDING) {
    uint32_t now = millis();
    if (now - lastLedToggle >= 500) {
      lastLedToggle = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  } else {
    if (ledState) { ledState = false; digitalWrite(LED_PIN, LOW); }
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Sensor initialisation helpers
 * ═══════════════════════════════════════════════════════════════════════ */

void resetGasBuffer(uint8_t si, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    gasBuffer[si][i]   = 0.0f;
    gasReceived[si][i] = false;
  }
  lastGasIdx[si] = -1;
}

void initSensor(uint8_t idx) {
  const HeatingProfile &prof = PROFILES[sensorProfile[idx]];
  commSetup = commMuxSetConfig(Wire, SPI, idx, commSetup);
  bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup);
  if (bme.checkStatus()) {
    Serial.println("ERR sensor " + String(idx + 1) + ": " + bme.statusString());
    return;
  }
  bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);
  bme.setFilter(BME68X_FILTER_SIZE_3);
  if (prof.scanMode == SCAN_PARALLEL) {
    uint16_t tArr[10], mArr[10];
    memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
    memcpy(mArr, prof.mulProf,  prof.profileLen * sizeof(uint16_t));
    uint16_t sharedDur = 140 - (bme.getMeasDur(BME68X_PARALLEL_MODE) / 1000);
    bme.setHeaterProf(tArr, mArr, sharedDur, prof.profileLen);
    sharedHeatrDur[idx] = sharedDur;
  } else if (prof.scanMode == SCAN_SEQUENTIAL) {
    uint16_t tArr[10], dArr[10];
    memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
    memcpy(dArr, prof.mulProf,  prof.profileLen * sizeof(uint16_t));
    bme.setSeqSleep(prof.seqSleep);
    bme.setHeaterProf(tArr, dArr, prof.profileLen);
    sharedHeatrDur[idx] = 0;
  } else {
    bme.setHeaterProf(prof.forcedTemp, prof.forcedDur);
    sharedHeatrDur[idx] = 0;
  }
}

/* Populate sensorProfile[] and activeSensors[] from the given plan step. */
void applyPlanStep(uint8_t step) {
  activeSensorCount = 0;
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t pid = PLAN[step].sensorProfiles[i];
    if (pid == 0) continue;
    if (pid > NUM_PROFILES) {
      Serial.println("  Step " + String(step + 1) + " S" + String(i + 1) +
                     ": invalid profile " + String(pid) + " — skipped.");
      continue;
    }
    sensorProfile[i] = pid - 1;
    activeSensors[activeSensorCount++] = i;
  }
}

/* Initialise hardware for all sensors in the given plan step and arm them. */
void initActiveSensors(uint8_t step) {
  applyPlanStep(step);
  for (uint8_t i = 0; i < activeSensorCount; i++) {
    uint8_t si = activeSensors[i];
    initSensor(si);
    const HeatingProfile &prof = PROFILES[sensorProfile[si]];
    if (prof.scanMode == SCAN_PARALLEL) {
      resetGasBuffer(si, prof.profileLen);
      bme.setOpMode(BME68X_PARALLEL_MODE);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      resetGasBuffer(si, prof.profileLen);
      bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    }
    /* Forced mode: triggered per measurement in the loop. */
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Verbose row helper  (used during RECORDING)
 * ═══════════════════════════════════════════════════════════════════════ */

void verbosePrint(uint8_t sensorNum, uint32_t fpIdx, uint8_t pos,
                  uint16_t plateTemp, uint16_t heatDur,
                  float temp, float pres, float hum, float gas) {
  Serial.print("S"); Serial.print(sensorNum);
  Serial.print(" fp="); Serial.print(fpIdx);
  Serial.print(" pos="); Serial.print(pos);
  Serial.print(" | T="); Serial.print(temp, 1);
  Serial.print(" H="); Serial.print(hum, 1);
  Serial.print(" P="); Serial.print(pres, 1);
  Serial.print(" G="); Serial.print(gas, 0);
  Serial.print(" | plate="); Serial.print(plateTemp);
  Serial.print("C dur="); Serial.print(heatDur);
  Serial.println("ms");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Row ring buffer helpers
 * ═══════════════════════════════════════════════════════════════════════ */

void pushRow(uint8_t sensorNum, uint32_t fpIdx, uint8_t pos,
             uint16_t plateTemp, uint16_t heatDur,
             float temperature, float pressure, float humidity, float gas) {
  if (rowCount >= ROW_BUFFER_SIZE) return;
  SdRow &r      = rowBuffer[rowHead];
  r.sensorNum   = sensorNum;
  r.fpIdx       = fpIdx;
  r.pos         = pos;
  r.plateTemp   = plateTemp;
  r.heatDur     = heatDur;
  r.temperature = temperature;
  r.pressure    = pressure;
  r.humidity    = humidity;
  r.gas         = gas;
  rowHead = (rowHead + 1) % ROW_BUFFER_SIZE;
  rowCount++;
}

void flushRowBuffer(uint8_t maxRows = 4) {
  uint8_t written = 0;
  while (rowCount > 0 && written < maxRows) {
    const SdRow &r = rowBuffer[rowTail];
    sdLogRow(r.sensorNum, r.fpIdx, r.pos, r.plateTemp, r.heatDur,
             r.temperature, r.pressure, r.humidity, r.gas, recordLabel);
    rowTail = (rowTail + 1) % ROW_BUFFER_SIZE;
    rowCount--;
    written++;
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Measurement functions
 * Caller must have set commSetup to sensor si before calling.
 * ═══════════════════════════════════════════════════════════════════════ */

void takeForcedMeasurement(uint8_t si) {
  bme.setOpMode(BME68X_FORCED_MODE);
  delayMicroseconds(bme.getMeasDur(BME68X_FORCED_MODE));

  bme68xData data;
  if (!bme.fetchData()) return;
  bme.getData(data);
  if (!(data.status & BME68X_NEW_DATA_MSK))   return;
  if (!(data.status & BME68X_GASM_VALID_MSK)) return;
  if (!(data.status & BME68X_HEAT_STAB_MSK))  return;

  if (mode == RECORDING) {
    const HeatingProfile &prof = PROFILES[sensorProfile[si]];
    pushRow(si + 1, fingerprintIndex[si], 0,
            prof.forcedTemp, prof.forcedDur,
            data.temperature, data.pressure / 100.0f,
            data.humidity, data.gas_resistance);
    if (verboseMode)
      verbosePrint(si + 1, fingerprintIndex[si], 0,
                   prof.forcedTemp, prof.forcedDur,
                   data.temperature, data.pressure / 100.0f,
                   data.humidity, data.gas_resistance);
    fingerprintIndex[si]++;
  }
}

/* Poll for new parallel-mode samples from sensor si.
 * RECORDING: log each individual step; all steps of a cycle share the same
 *            fingerprintIndex. */
void pollParallelMeasurement(uint8_t si) {
  const HeatingProfile &prof = PROFILES[sensorProfile[si]];
  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Cycle boundary: gas_index wraps backwards → previous cycle is complete. */
    if (lastGasIdx[si] >= 0 && (int8_t)gi < lastGasIdx[si]) {
      if (cycleHasData[si]) fingerprintIndex[si]++;
      cycleHasData[si] = false;
      resetGasBuffer(si, prof.profileLen);
    }

    lastTemp[si] = data.temperature;
    lastHum[si]  = data.humidity;
    lastPres[si] = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      gasBuffer[si][gi]   = data.gas_resistance;
      gasReceived[si][gi] = true;

      if (mode == RECORDING) {
        uint16_t heatDur = (uint16_t)(
            (uint32_t)prof.mulProf[gi] * sharedHeatrDur[si] / 63);
        pushRow(si + 1, fingerprintIndex[si], gi,
                prof.tempProf[gi], heatDur,
                data.temperature, data.pressure / 100.0f,
                data.humidity, data.gas_resistance);
        if (verboseMode)
          verbosePrint(si + 1, fingerprintIndex[si], gi,
                       prof.tempProf[gi], heatDur,
                       data.temperature, data.pressure / 100.0f,
                       data.humidity, data.gas_resistance);
        cycleHasData[si] = true;
      }
    }
    lastGasIdx[si] = (int8_t)gi;
  }
}

/* Poll for new sequential-mode samples from sensor si.
 * Behaves identically to pollParallelMeasurement but uses sequential mode
 * and stores actual heater durations in mulProf rather than multipliers. */
void pollSequentialMeasurement(uint8_t si) {
  const HeatingProfile &prof = PROFILES[sensorProfile[si]];
  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    if (lastGasIdx[si] >= 0 && (int8_t)gi < lastGasIdx[si]) {
      if (cycleHasData[si]) fingerprintIndex[si]++;
      cycleHasData[si] = false;
      resetGasBuffer(si, prof.profileLen);
    }

    lastTemp[si] = data.temperature;
    lastHum[si]  = data.humidity;
    lastPres[si] = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      gasBuffer[si][gi]   = data.gas_resistance;
      gasReceived[si][gi] = true;

      if (mode == RECORDING) {
        uint16_t heatDur = prof.mulProf[gi];
        pushRow(si + 1, fingerprintIndex[si], gi,
                prof.tempProf[gi], heatDur,
                data.temperature, data.pressure / 100.0f,
                data.humidity, data.gas_resistance);
        if (verboseMode)
          verbosePrint(si + 1, fingerprintIndex[si], gi,
                       prof.tempProf[gi], heatDur,
                       data.temperature, data.pressure / 100.0f,
                       data.humidity, data.gas_resistance);
        cycleHasData[si] = true;
      }
    }
    lastGasIdx[si] = (int8_t)gi;
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Stage transition helpers
 * ═══════════════════════════════════════════════════════════════════════ */

/* Put all active sensors to sleep and move to WAITING. */
void enterWaitingStage() {
  for (uint8_t i = 0; i < activeSensorCount; i++) {
    commSetup = commMuxSetConfig(Wire, SPI, activeSensors[i], commSetup);
    bme.setOpMode(BME68X_SLEEP_MODE);
  }
  activeSensorCount = 0;
  mode = WAITING;
  if (planStep < NUM_PLAN_STEPS)
    Serial.println(">> Step " + String(planStep + 1) + "/" + String(NUM_PLAN_STEPS) +
                   ":  file=" + String(PLAN[planStep].filename) +
                   "  label=" + String(PLAN[planStep].label));
  Serial.println(">> Send 'next' to start recording.");
}

void enterCleaningStage(uint32_t durationSec) {
  cleaningDurMs = durationSec * 1000UL;
  stageStartMs  = millis();
  mode = CLEANING;
  Serial.println(">> CLEANING stage — " + String(durationSec) + "s...");
}

void startRecordingStage() {
  initActiveSensors(planStep);
  String filename = String(PLAN[planStep].filename);
  if (!sdOpenFile(filename)) {
    Serial.println(">> Recording aborted — SD card error.");
    return;
  }
  recordLabel = String(PLAN[planStep].label);
  for (uint8_t i = 0; i < activeSensorCount; i++) {
    uint8_t si = activeSensors[i];
    fingerprintIndex[si] = 0;
    cycleHasData[si]     = false;
  }
  stageStartMs = millis();
  mode = RECORDING;

  Serial.print(">> RECORDING step " + String(planStep + 1) + "/" +
               String(NUM_PLAN_STEPS) + ":  file=" + filename +
               ".csv  label=" + recordLabel +
               "  duration=" + String(PLAN[planStep].durationSec) + "s");
  Serial.print("  sensors:");
  for (uint8_t i = 0; i < activeSensorCount; i++) {
    uint8_t si = activeSensors[i];
    Serial.print(" S" + String(si + 1) +
                 "[" + String(PROFILES[sensorProfile[si]].name) + "]");
  }
  Serial.println();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Command handling
 * ═══════════════════════════════════════════════════════════════════════ */

void handleCommand(const String &cmd) {

  /* ── start ─────────────────────────────────────────────────────────── */
  if (cmd == "start") {
    if (mode != IDLE) {
      Serial.println(">> Already running. Send 'stop' first.");
      return;
    }
    if (NUM_PLAN_STEPS == 0) {
      Serial.println(">> PLAN[] is empty — edit and re-upload.");
      return;
    }
    planStep = 0;
    Serial.println(">> Starting data collection — " +
                   String(NUM_PLAN_STEPS) + " step(s) in plan.");
    enterWaitingStage();

  /* ── next ──────────────────────────────────────────────────────────── */
  } else if (cmd == "next") {
    if (mode == WAITING) {
      if (planStep < NUM_PLAN_STEPS) {
        startRecordingStage();
      } else {
        Serial.println(">> All steps done. Send 'stop'.");
      }
    } else {
      Serial.println(">> 'next' is only valid when WAITING between steps.");
    }

  /* ── stop ──────────────────────────────────────────────────────────── */
  } else if (cmd == "stop") {
    for (uint8_t i = 0; i < activeSensorCount; i++) {
      commSetup = commMuxSetConfig(Wire, SPI, activeSensors[i], commSetup);
      bme.setOpMode(BME68X_SLEEP_MODE);
    }
    activeSensorCount = 0;
    rowCount = 0; rowHead = 0; rowTail = 0;
    mode = IDLE;
    Serial.println(">> Stopped. Send 'start' to begin again.");

  /* ── verbose ───────────────────────────────────────────────────────── */
  } else if (cmd == "verbose") {
    verboseMode = !verboseMode;
    Serial.println(">> Verbose " + String(verboseMode ? "ON" : "OFF"));

  /* ── status ────────────────────────────────────────────────────────── */
  } else if (cmd == "status") {
    const char *mStr = (mode == RECORDING) ? "RECORDING" :
                       (mode == CLEANING)  ? "CLEANING"  :
                       (mode == WAITING)   ? "WAITING"   : "IDLE";
    Serial.println("Mode    : " + String(mStr));
    Serial.println("Verbose : " + String(verboseMode ? "ON" : "OFF"));
    Serial.println("Step    : " + String(planStep + 1) + " / " + String(NUM_PLAN_STEPS));
    if (mode == RECORDING) {
      uint32_t elapsed = (millis() - stageStartMs) / 1000;
      Serial.println("Elapsed : " + String(elapsed) + "s / " +
                     String(PLAN[planStep].durationSec) + "s");
      for (uint8_t i = 0; i < activeSensorCount; i++) {
        uint8_t si = activeSensors[i];
        Serial.println("  S" + String(si + 1) + " fp=" + String(fingerprintIndex[si]));
      }
    }
    if (mode == CLEANING) {
      uint32_t elapsed = millis() - stageStartMs;
      uint32_t remaining = (elapsed < cleaningDurMs)
                           ? (cleaningDurMs - elapsed) / 1000 : 0;
      Serial.println("Cleaning: " + String(remaining) + "s remaining");
    }

  /* ── plan ──────────────────────────────────────────────────────────── */
  } else if (cmd == "plan") {
    Serial.println("--- Collection plan (" + String(NUM_PLAN_STEPS) + " steps) ---");
    for (uint8_t s = 0; s < NUM_PLAN_STEPS; s++) {
      Serial.print("  " + String(s + 1) + ": " +
                   String(PLAN[s].filename) + " [" + String(PLAN[s].label) + "]" +
                   "  " + String(PLAN[s].durationSec) + "s" +
                   "  clean=" + String(PLAN[s].cleaningDurationSec) + "s" +
                   "  sensors:");
      for (uint8_t i = 0; i < 8; i++) {
        if (PLAN[s].sensorProfiles[i])
          Serial.print(" S" + String(i + 1) +
                       "=p" + String(PLAN[s].sensorProfiles[i]));
      }
      Serial.println();
    }
    Serial.println("---");

  /* ── profiles ──────────────────────────────────────────────────────── */
  } else if (cmd == "profiles") {
    Serial.println("--- Available heating profiles ---");
    for (uint8_t i = 0; i < NUM_PROFILES; i++)
      Serial.println("  " + String(i + 1) + ": " +
                     String(PROFILES[i].name) + "  " +
                     String(PROFILES[i].description));
    Serial.println("---");

  /* ── unknown ───────────────────────────────────────────────────────── */
  } else {
    Serial.println(">> Unknown: '" + cmd + "'");
    Serial.println("   Commands: start | next | stop | verbose | status | plan | profiles");
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Setup
 * ═══════════════════════════════════════════════════════════════════════ */

void setup() {
  Serial.begin(921600);
  Serial.println("Initializing...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  commMuxBegin(Wire, SPI);
  delay(100);
  sdInit();
  Serial.println("Ready.");
  Serial.println("Commands: start | next | stop | verbose | status | plan | profiles");
  Serial.println("Send 'plan' to review the collection sequence, then 'start'.");
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
      if (cmdBuffer.length() > 0) { handleCommand(cmdBuffer); cmdBuffer = ""; }
    } else {
      cmdBuffer += c;
    }
  }

  if (mode == IDLE || mode == WAITING) { updateLed(); return; }

  /* ── Poll all active sensors ──────────────────────────────────────── */
  bool anyMultiStep = false;
  for (uint8_t i = 0; i < activeSensorCount; i++) {
    uint8_t si = activeSensors[i];
    commSetup = commMuxSetConfig(Wire, SPI, si, commSetup);
    const HeatingProfile &prof = PROFILES[sensorProfile[si]];
    if (prof.scanMode == SCAN_PARALLEL) {
      pollParallelMeasurement(si);
      anyMultiStep = true;
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      pollSequentialMeasurement(si);
      anyMultiStep = true;
    } else {
      takeForcedMeasurement(si);
    }
  }
  if (anyMultiStep) delay(10);

  /* ── Mode-specific post-poll actions ─────────────────────────────── */
  if (mode == RECORDING) {
    /* Write buffered rows to SD card. */
    flushRowBuffer();

    /* Auto-stop when the recording duration elapses. */
    if (millis() - stageStartMs >= (uint32_t)PLAN[planStep].durationSec * 1000UL) {
      uint32_t cleanSec = PLAN[planStep].cleaningDurationSec;
      Serial.println(">> Recording complete for step " + String(planStep + 1) + ".");
      planStep++;

      if (planStep >= NUM_PLAN_STEPS) {
        /* All steps finished — stop sensors and return to idle. */
        for (uint8_t i = 0; i < activeSensorCount; i++) {
          commSetup = commMuxSetConfig(Wire, SPI, activeSensors[i], commSetup);
          bme.setOpMode(BME68X_SLEEP_MODE);
        }
        activeSensorCount = 0;
        mode = IDLE;
        Serial.println(">> All " + String(NUM_PLAN_STEPS) +
                       " steps complete! Collection finished.");
      } else if (cleanSec > 0) {
        enterCleaningStage(cleanSec);
      } else {
        enterWaitingStage();
      }
    }

  } else if (mode == CLEANING) {
    /* Check whether the cleaning phase has elapsed. */
    if (millis() - stageStartMs >= cleaningDurMs) {
      Serial.println(">> Cleaning complete.");
      enterWaitingStage();
    }
  }

  updateLed();
}
