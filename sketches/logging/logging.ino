/**
 * Real-Time Plotting & Recording — BME688 Dev Kit
 *
 * Streams sensor readings in Arduino Serial Plotter format (plot mode)
 * or saves them to an SD card CSV file (record mode).
 * Supports three operating modes selectable per sensor via heating profiles:
 *
 *   Forced mode     — single heater setpoint, one reading per trigger.
 *                     Outputs: Temperature, Humidity, Pressure, GasResistance
 *
 *   Parallel mode   — 10-step heater scan cycle (gas fingerprint).
 *                     Outputs per complete cycle: Temperature, Humidity,
 *                     Pressure, Gas0 … Gas9  (one channel per heater step)
 *
 *   Sequential mode — up to 10-step heater scan cycle with ODR sleep between
 *                     cycles. Same plotter output format as parallel mode.
 *
 * Autostart:
 *   Set AUTOSTART_ENABLED to 1 (near the top of the sketch) and fill in
 *   AUTOSTART_PROFILES[], AUTOSTART_FILENAME, and AUTOSTART_LABEL.
 *   The board will begin recording immediately on power-up — no commands needed.
 *
 * Serial commands:
 *   plot              — stream the active sensor to Serial Plotter
 *   record [1 2 …]    — record one or more sensors simultaneously to SD card.
 *                       Sensor list is 1-based, space-separated.
 *                       Omitting the list uses the currently active sensor.
 *                       Example: "record 1 2 3 4"
 *   stop              — pause plotting / recording
 *   verbose           — toggle verbose row output to Serial (while recording)
 *   1–8               — (stopped only) switch active sensor (for plotting)
 *   profile <N>       — (stopped only) set heating profile for active sensor
 *   profiles          — list all available heating profiles
 *   filename <name>   — (stopped only) set CSV filename (no extension needed)
 *   label <text>      — set the specimen / class label written to every row
 *   status            — show mode, active sensors, profile assignments
 *
 * SD card CSV columns:
 *   sensor_index, fingerprint_index, position, plate_temperature,
 *   heater_duration, temperature, pressure, humidity, gas_resistance, label
 *
 * Typical workflow for a new specimen:
 *   stop  →  filename specimen2  →  label lavender  →  record 1 2 3 4
 *
 * Power-recovery workflow (resume an interrupted file):
 *   record 1 2 3 4    (same filename — file is appended automatically)
 *
 * Required libraries: bme68xLibrary (Bosch Sensortec), SD (Arduino built-in)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <bme68xLibrary.h>
#include "commMux.h"
#include "sdLogger.h"
#include "bleStreamer.h"

/* Both bits must be set for a gas reading to be considered valid at the
 * intended heater temperature. */
#define GAS_VALID_MSK (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)

/* ── LED ────────────────────────────────────────────────────────────────
 * The built-in LED blinks at 1 Hz while recording is active.
 * Change LED_PIN if your board uses a different pin. */
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
  /* Forced mode */
  uint16_t    forcedTemp;      // target temperature, °C
  uint16_t    forcedDur;       // heater duration, ms
  /* Parallel / Sequential mode */
  uint8_t     profileLen;      // number of heater steps (max 10)
  uint16_t    tempProf[10];    // heater temperatures, °C
  uint16_t    mulProf[10];     // parallel: duration multipliers; sequential: durations (ms)
  /* Sequential mode only */
  uint8_t     seqSleep;        // ODR constant, e.g. BME68X_ODR_250_MS
};

/* Profile IDs are 1-based in user commands; 0-based as array index.
 *
 * Parallel-mode duration multipliers:
 *   The sensor's actual heater-on time per step ≈ mulProf[i] * sharedHeatrDur / 63 ms,
 *   where sharedHeatrDur = 140 − T/P/H measurement time (computed in initSensor).
 *   Larger multipliers → longer dwell at that temperature step.
 *
 * Sequential-mode mulProf values are actual heater durations in ms. */
const HeatingProfile PROFILES[] = {

  /* ── Forced mode ──────────────────────────────────────────────────── */
  {
    "Forced_Std",
    "[F] 320 C / 150 ms  (default)",
    SCAN_FORCED, 320, 150, 0, {0}, {0}, 0
  },
  {
    "Forced_Low",
    "[F] 200 C / 150 ms",
    SCAN_FORCED, 200, 150, 0, {0}, {0}, 0
  },
  {
    "Forced_High",
    "[F] 400 C / 200 ms",
    SCAN_FORCED, 400, 200, 0, {0}, {0}, 0
  },

  /* ── Parallel mode ────────────────────────────────────────────────── */
  {
    "Par_Bosch",
    "[P] Bosch standard 10-step: 320,100,100,100,200,200,200,320,320,320 C",
    SCAN_PARALLEL, 0, 0, 10,
    {320, 100, 100, 100, 200, 200, 200, 320, 320, 320},
    {  5,   2,  10,  30,   5,   5,   5,   5,   5,   5},
    0
  },
  {
    "Par_LinSweep",
    "[P] Linear sweep  200 -> 400 C  (10 equal steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {200, 222, 244, 267, 289, 311, 333, 356, 378, 400},
    {  5,   5,   5,   5,   5,   5,   5,   5,   5,   5},
    0
  },
  {
    "Par_WideSweep",
    "[P] Wide sweep    100 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100, 150, 200, 250, 300, 325, 350, 380, 415, 450},
    { 10,   8,   7,   6,   5,   5,   5,   5,   5,   5},
    0
  },
  {
    "Par_HighFocus",
    "[P] High-temp     300 -> 450 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {300, 317, 333, 350, 367, 383, 400, 417, 433, 450},
    {  5,   5,   5,   5,   5,   5,   5,   5,   5,   5},
    0
  },
  {
    "Par_LowFocus",
    "[P] Low-temp      100 -> 250 C  (10 steps)",
    SCAN_PARALLEL, 0, 0, 10,
    {100, 117, 133, 150, 167, 183, 200, 217, 233, 250},
    { 10,   9,   8,   8,   7,   7,   6,   6,   6,   5},
    0
  },

  /* ── Sequential mode ──────────────────────────────────────────────── */
  {
    "Seq_Std",
    "[S] 100,200,320 C / 150 ms each, 250 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 3,
    {100, 200, 320},
    {150, 150, 150},
    BME68X_ODR_250_MS
  },
  {
    "Seq_LinSweep",
    "[S] Linear sweep 200->400 C (10 steps / 150 ms each, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {200, 222, 244, 267, 289, 311, 333, 356, 378, 400},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150},
    BME68X_ODR_500_MS
  },
  {
    "Seq_Bosch",
    "[S] Bosch 10-step: 320,100x3,200x3,320x3 C / 150 ms each, 500 ms ODR sleep",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {320, 100, 100, 100, 200, 200, 200, 320, 320, 320},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150},
    BME68X_ODR_500_MS
  },
  {
    "Seq_WideSweep",
    "[S] Wide sweep 100->450 C (10 steps, longer dwell at low temps, 500 ms ODR sleep)",
    SCAN_SEQUENTIAL, 0, 0, 10,
    {100, 150, 200, 250, 300, 325, 350, 380, 415, 450},
    {200, 180, 160, 150, 150, 140, 140, 140, 140, 140},
    BME68X_ODR_500_MS
  },
  {
    "seq",
    "[S] Fast 3-step 220,320,400 C / 100 ms each, no sleep",
    SCAN_SEQUENTIAL, 0, 0, 3,
    {220, 320, 400},   // heater temps
    {100,  100,  100},    // heater durations in ms
    BME68X_ODR_NONE    // no sleep between cycles
  },
  {
    "seq2",
    "[S] Fast 5-step 100, 160, 250, 300, 380",
    SCAN_SEQUENTIAL, 0, 0, 5,
    {120, 200, 280, 350, 420},
    {200, 160, 150, 140, 140},
    BME68X_ODR_NONE
  }
};

const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

/* ═══════════════════════════════════════════════════════════════════════
 * Autostart preset  ← edit this section to configure power-on recording
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Set AUTOSTART_ENABLED to 1 and fill in the three settings below.
 * The board will open the SD file and start recording immediately after
 * power-up — no serial commands required.
 * Set AUTOSTART_ENABLED to 0 to use interactive commands as normal.
 *
 * AUTOSTART_PROFILES[8]
 *   One entry per sensor slot (index 0 = sensor 1, … index 7 = sensor 8).
 *   Value = 1-based profile ID  (send 'profiles' over Serial to list them).
 *   Set an entry to 0 to exclude that sensor from the recording.
 *
 */
#define AUTOSTART_ENABLED 0

static const uint8_t AUTOSTART_PROFILES[8] = {9, 10, 11, 12,  0,  0,  0,  0};
static const char    AUTOSTART_FILENAME[]   = "rose_open_container";
static const char    AUTOSTART_LABEL[]      = "rose";

/* ═══════════════════════════════════════════════════════════════════════
 * Global state
 * ═══════════════════════════════════════════════════════════════════════ */

Bme68x  bme;
commMux commSetup;

uint8_t sensorIdx        = 0;                   // 0-based active sensor (for plotting)
uint8_t sensorProfile[8] = {11,11,11,11,11,11,11,11};  // per-sensor profile index

/* Shared heater duration (ms) per sensor, computed in initSensor for
 * parallel profiles; used to convert multipliers to approximate ms. */
uint16_t sharedHeatrDur[8] = {0,0,0,0,0,0,0,0};

/* Operating mode */
enum Mode { IDLE, PLOTTING, RECORDING };
Mode mode = IDLE;

/* Recording state */
String   recordFilename    = "data";      // filename without extension
String   recordLabel       = "unlabeled"; // specimen / class label
uint8_t  recordSensors[8];               // 0-based sensor indices being recorded
uint8_t  recordSensorCount = 0;

/* Verbose mode: when true, each recorded row is echoed to Serial */
bool verboseMode = false;

String cmdBuffer = "";

/* Per-sensor parallel/sequential-mode cycle accumulation */
float   gasBuffer[8][10];        // gas resistance per sensor per heater step
bool    gasReceived[8][10];      // valid data flags per sensor per step
float   lastTemp[8]    = {0};
float   lastHum[8]     = {0};
float   lastPres[8]    = {0};
int8_t  lastGasIdx[8]  = {-1,-1,-1,-1,-1,-1,-1,-1};

/* Per-sensor fingerprint (scan-cycle) counter, reset on each record start */
uint32_t fingerprintIndex[8] = {0};

/* True when at least one valid row has been logged in the current cycle.
 * Prevents invalid cycle-end events from silently bumping the index. */
bool cycleHasData[8] = {false};

/* ── Row ring buffer ────────────────────────────────────────────────────
 * Sensor polls push rows here; the main loop flushes them to SD after all
 * sensors have been polled.  This keeps SD write latency from delaying the
 * next sensor's poll.
 *
 * Size: 8 sensors × 10 steps = 80 rows worst-case per iteration; 128 gives
 * comfortable headroom.  Each SdRow is ~28 bytes → ~3.5 KB total. */
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
static uint8_t rowHead  = 0;   // next write slot
static uint8_t rowTail  = 0;   // next read slot
static uint8_t rowCount = 0;

/* LED blink state (1 Hz during recording) */
uint32_t lastLedToggle = 0;
bool     ledState      = false;

/* ═══════════════════════════════════════════════════════════════════════
 * LED helper
 * ═══════════════════════════════════════════════════════════════════════ */

void updateLed() {
  if (mode == RECORDING) {
    uint32_t now = millis();
    if (now - lastLedToggle >= 500) {   // toggle every 500 ms → 1 Hz blink
      lastLedToggle = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
  } else {
    if (ledState) {
      ledState = false;
      digitalWrite(LED_PIN, LOW);
    }
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Sensor initialisation
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
    /* Copy to non-const locals — the library takes non-const pointers */
    uint16_t tArr[10], mArr[10];
    memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
    memcpy(mArr, prof.mulProf,  prof.profileLen * sizeof(uint16_t));
    /* sharedHeatrDur = ODR period minus T/P/H measurement time */
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
  /* Note: initSensor leaves commSetup pointing at idx. */
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

  initSensor(sensorIdx);
  sdInit();

  bleInit();

  Serial.println("Ready.");
  Serial.println("Commands: plot | record [1 2 …] | stop | verbose | 1-8");
  Serial.println("          profile <N> | profiles | filename <name> | label <text> | status");
  Serial.println("Active: sensor 1, profile 1 [Forced_Std]");
  Serial.println("File  : " + recordFilename + ".csv   Label: " + recordLabel);

#if AUTOSTART_ENABLED
  /* Apply the autostart preset and begin recording immediately. */
  Serial.println("Autostart preset active — starting recording...");
  recordFilename = AUTOSTART_FILENAME;
  recordLabel    = AUTOSTART_LABEL;
  recordSensorCount = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (AUTOSTART_PROFILES[i] == 0) continue;
    uint8_t profIdx = (uint8_t)(AUTOSTART_PROFILES[i] - 1);
    if (profIdx >= NUM_PROFILES) {
      Serial.println("  Sensor " + String(i + 1) + ": invalid profile " +
                     String(AUTOSTART_PROFILES[i]) + " — skipped.");
      continue;
    }
    sensorProfile[i] = profIdx;
    recordSensors[recordSensorCount++] = i;
  }
  if (recordSensorCount > 0) {
    startRecording();
  } else {
    Serial.println("  No valid sensors in preset — staying idle.");
  }
#endif
}

/* ═══════════════════════════════════════════════════════════════════════
 * Serial Plotter output helpers  (plotting mode — single sensor only)
 * ═══════════════════════════════════════════════════════════════════════ */

void printForcedLine(const bme68xData &d) {
  Serial.print("Temperature:"); Serial.print(d.temperature, 2);
  Serial.print(" Humidity:");   Serial.print(d.humidity,    2);
  Serial.print(" Pressure:");   Serial.print(d.pressure / 100.0f, 2);
  Serial.print(" GasResistance:"); Serial.println(d.gas_resistance, 0);
}

void printScanCycle(uint8_t si, uint8_t profileLen) {
  bool anyValid = false;
  for (uint8_t i = 0; i < profileLen; i++) {
    if (gasReceived[si][i]) { anyValid = true; break; }
  }
  if (!anyValid) return;

  Serial.print("Temperature:"); Serial.print(lastTemp[si], 2);
  Serial.print(" Humidity:");   Serial.print(lastHum[si],  2);
  Serial.print(" Pressure:");   Serial.print(lastPres[si], 2);
  for (uint8_t i = 0; i < profileLen; i++) {
    Serial.print(" Gas"); Serial.print(i); Serial.print(":");
    Serial.print(gasReceived[si][i] ? gasBuffer[si][i] : 0.0f, 0);
  }
  Serial.println();
}

/* ═══════════════════════════════════════════════════════════════════════
 * Verbose row helper
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
 * Ring buffer helpers
 * ═══════════════════════════════════════════════════════════════════════ */

void pushRow(uint8_t sensorNum, uint32_t fpIdx, uint8_t pos,
             uint16_t plateTemp, uint16_t heatDur,
             float temperature, float pressure, float humidity, float gas) {
  if (rowCount >= ROW_BUFFER_SIZE) return;   // buffer full — drop row
  SdRow &r    = rowBuffer[rowHead];
  r.sensorNum  = sensorNum;
  r.fpIdx      = fpIdx;
  r.pos        = pos;
  r.plateTemp  = plateTemp;
  r.heatDur    = heatDur;
  r.temperature = temperature;
  r.pressure   = pressure;
  r.humidity   = humidity;
  r.gas        = gas;
  rowHead = (rowHead + 1) % ROW_BUFFER_SIZE;
  rowCount++;
}

/* Write every buffered row to the SD card and reset the buffer.
 * Called once per main-loop iteration, after all sensors have been polled. */
void flushRowBuffer() {
  while (rowCount > 0) {
    const SdRow &r = rowBuffer[rowTail];
    sdLogRow(r.sensorNum, r.fpIdx, r.pos, r.plateTemp, r.heatDur,
             r.temperature, r.pressure, r.humidity, r.gas, recordLabel);

    bleStreamRow(r.sensorNum, r.fpIdx, r.pos, r.plateTemp, r.heatDur,
             r.temperature, r.pressure, r.humidity, r.gas, recordLabel);
    rowTail = (rowTail + 1) % ROW_BUFFER_SIZE;
    rowCount--;
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

  if (!(data.status & BME68X_NEW_DATA_MSK))        return;
  if (!(data.status & BME68X_GASM_VALID_MSK))      return;
  if (!(data.status & BME68X_HEAT_STAB_MSK))       return;

  if (mode == PLOTTING) {
    printForcedLine(data);
  }

  if (mode == RECORDING) {
    const HeatingProfile &prof = PROFILES[sensorProfile[si]];
    pushRow(si + 1,
            fingerprintIndex[si],
            0,                   // position — single step in forced mode
            prof.forcedTemp,
            prof.forcedDur,
            data.temperature,
            data.pressure / 100.0f,
            data.humidity,
            data.gas_resistance);
    if (verboseMode) {
      verbosePrint(si + 1, fingerprintIndex[si], 0,
                   prof.forcedTemp, prof.forcedDur,
                   data.temperature, data.pressure / 100.0f,
                   data.humidity, data.gas_resistance);
    }
    fingerprintIndex[si]++;
  }
}

/* Poll for new parallel-mode samples from sensor si.
 * PLOTTING mode: flush a complete-cycle line to Serial Plotter after each
 *   full scan cycle.
 * RECORDING mode: log each individual step to the SD card CSV file as soon
 *   as it arrives; all steps of one cycle share the same fingerprintIndex. */
void pollParallelMeasurement(uint8_t si) {
  const HeatingProfile &prof = PROFILES[sensorProfile[si]];

  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Detect cycle boundary by gas_index wrapping backwards.
     * This fires reliably on the first reading of a new cycle regardless of
     * whether the previous cycle's last step had valid gas. */
    if (lastGasIdx[si] >= 0 && (int8_t)gi < lastGasIdx[si]) {
      if (mode == PLOTTING) printScanCycle(si, prof.profileLen);
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
        /* Approximate heater-on time: mulProf[step] * sharedHeatrDur / 63 */
        uint16_t heatDur = (uint16_t)(
          (uint32_t)prof.mulProf[gi] * sharedHeatrDur[si] / 63);

        pushRow(si + 1,
                fingerprintIndex[si],
                gi,                    // position = heater step index
                prof.tempProf[gi],     // plate_temperature for this step
                heatDur,
                data.temperature,
                data.pressure / 100.0f,
                data.humidity,
                data.gas_resistance);
        if (verboseMode) {
          verbosePrint(si + 1, fingerprintIndex[si], gi,
                       prof.tempProf[gi], heatDur,
                       data.temperature, data.pressure / 100.0f,
                       data.humidity, data.gas_resistance);
        }
        cycleHasData[si] = true;
      }
    }

    lastGasIdx[si] = (int8_t)gi;
  }
}

/* Poll for new sequential-mode samples from sensor si.
 * Behaves like parallel mode but uses BME68X_SEQUENTIAL_MODE and stores
 * actual heater durations (ms) rather than multipliers in mulProf. */
void pollSequentialMeasurement(uint8_t si) {
  const HeatingProfile &prof = PROFILES[sensorProfile[si]];

  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Same wrap-around boundary detection as parallel mode. */
    if (lastGasIdx[si] >= 0 && (int8_t)gi < lastGasIdx[si]) {
      if (mode == PLOTTING) printScanCycle(si, prof.profileLen);
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
        uint16_t heatDur = prof.mulProf[gi];   // actual duration in sequential mode

        pushRow(si + 1,
                fingerprintIndex[si],
                gi,
                prof.tempProf[gi],
                heatDur,
                data.temperature,
                data.pressure / 100.0f,
                data.humidity,
                data.gas_resistance);
        if (verboseMode) {
          verbosePrint(si + 1, fingerprintIndex[si], gi,
                       prof.tempProf[gi], heatDur,
                       data.temperature, data.pressure / 100.0f,
                       data.humidity, data.gas_resistance);
        }
        cycleHasData[si] = true;
      }
    }

    lastGasIdx[si] = (int8_t)gi;
  }
}

/* ═══════════════════════════════════════════════════════════════════════
 * Recording start helper
 * Assumes recordSensors[], recordSensorCount, recordFilename, recordLabel
 * are already set.  Opens the SD file, arms all sensors, sets RECORDING.
 * ═══════════════════════════════════════════════════════════════════════ */

void startRecording() {
  if (!sdOpenFile(recordFilename)) {
    Serial.println(">> Recording aborted — SD card error.");
    return;
  }

  for (uint8_t i = 0; i < recordSensorCount; i++) {
    uint8_t si = recordSensors[i];
    initSensor(si);
    fingerprintIndex[si] = 0;
    cycleHasData[si]     = false;
    const HeatingProfile &prof = PROFILES[sensorProfile[si]];
    if (prof.scanMode == SCAN_PARALLEL) {
      resetGasBuffer(si, prof.profileLen);
      bme.setOpMode(BME68X_PARALLEL_MODE);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      resetGasBuffer(si, prof.profileLen);
      bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    }
    /* Forced-mode sensors are triggered individually in the loop. */
  }

  mode = RECORDING;

  Serial.print(">> Recording sensors:");
  for (uint8_t i = 0; i < recordSensorCount; i++) {
    Serial.print(" " + String(recordSensors[i] + 1) +
                 "[" + String(PROFILES[sensorProfile[recordSensors[i]]].name) + "]");
  }
  Serial.println("  file=" + recordFilename + ".csv  label=" + recordLabel);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Command handling
 * ═══════════════════════════════════════════════════════════════════════ */

void handleCommand(const String &cmd) {

  /* ── plot ──────────────────────────────────────────────────────────── */
  if (cmd == "plot") {
    if (mode != IDLE) {
      Serial.println(">> Stop first.");
      return;
    }
    const HeatingProfile &prof = PROFILES[sensorProfile[sensorIdx]];
    commSetup = commMuxSetConfig(Wire, SPI, sensorIdx, commSetup);
    mode = PLOTTING;
    if (prof.scanMode == SCAN_PARALLEL) {
      resetGasBuffer(sensorIdx, prof.profileLen);
      bme.setOpMode(BME68X_PARALLEL_MODE);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      resetGasBuffer(sensorIdx, prof.profileLen);
      bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    }
    Serial.println(">> Plotting sensor " + String(sensorIdx + 1) +
                   " [" + String(prof.name) + "]");

  /* ── record [1 2 …] ────────────────────────────────────────────────── */
  } else if (cmd == "record" || cmd.startsWith("record ")) {
    if (mode != IDLE) {
      Serial.println(">> Stop first.");
      return;
    }

    /* Parse space-separated 1-based sensor numbers after "record" */
    recordSensorCount = 0;
    if (cmd.length() > 7) {
      String args = cmd.substring(7);   // everything after "record "
      int pos = 0;
      while (pos < (int)args.length() && recordSensorCount < 8) {
        while (pos < (int)args.length() && args[pos] == ' ') pos++;
        if (pos >= (int)args.length()) break;
        int end = args.indexOf(' ', pos);
        String token = (end == -1) ? args.substring(pos) : args.substring(pos, end);
        int n = token.toInt();
        if (n >= 1 && n <= 8) recordSensors[recordSensorCount++] = (uint8_t)(n - 1);
        pos = (end == -1) ? args.length() : end + 1;
      }
    }
    if (recordSensorCount == 0) {
      /* Fallback: use the currently active sensor */
      recordSensors[0]  = sensorIdx;
      recordSensorCount = 1;
    }

    startRecording();

  /* ── stop ──────────────────────────────────────────────────────────── */
  } else if (cmd == "stop") {
    if (mode == PLOTTING) {
      commSetup = commMuxSetConfig(Wire, SPI, sensorIdx, commSetup);
      bme.setOpMode(BME68X_SLEEP_MODE);
    } else if (mode == RECORDING) {
      for (uint8_t i = 0; i < recordSensorCount; i++) {
        commSetup = commMuxSetConfig(Wire, SPI, recordSensors[i], commSetup);
        bme.setOpMode(BME68X_SLEEP_MODE);
      }
    }
    mode = IDLE;
    Serial.println(">> Stopped.");

  /* ── verbose ───────────────────────────────────────────────────────── */
  } else if (cmd == "verbose") {
    verboseMode = !verboseMode;
    Serial.println(">> Verbose " + String(verboseMode ? "ON" : "OFF") +
                   " — rows will " + String(verboseMode ? "" : "not ") +
                   "be echoed to Serial during recording.");

  /* ── sensor select (1–8) — affects plotting only ───────────────────── */
  } else if (cmd.length() == 1 && cmd[0] >= '1' && cmd[0] <= '8') {
    if (mode != IDLE) {
      Serial.println(">> Stop before switching sensors.");
    } else {
      sensorIdx = (uint8_t)(cmd[0] - '1');
      initSensor(sensorIdx);
      Serial.println(">> Active sensor " + String(sensorIdx + 1) +
                     " [" + String(PROFILES[sensorProfile[sensorIdx]].name) +
                     "].  Send 'plot' or 'record' to begin.");
    }

  /* ── profile <N> ───────────────────────────────────────────────────── */
  } else if (cmd.startsWith("profile ")) {
    if (mode != IDLE) {
      Serial.println(">> Stop before changing profile.");
    } else {
      int n = cmd.substring(8).toInt();
      if (n < 1 || n > (int)NUM_PROFILES) {
        Serial.println(">> Profile must be 1–" + String(NUM_PROFILES) +
                       ".  Send 'profiles' to list.");
      } else {
        sensorProfile[sensorIdx] = (uint8_t)(n - 1);
        initSensor(sensorIdx);
        Serial.println(">> Sensor " + String(sensorIdx + 1) +
                       " profile set to " + String(n) +
                       " [" + String(PROFILES[sensorProfile[sensorIdx]].name) + "]");
      }
    }

  /* ── filename <name> ───────────────────────────────────────────────── */
  } else if (cmd.startsWith("filename ")) {
    if (mode != IDLE) {
      Serial.println(">> Stop before changing filename.");
    } else {
      recordFilename = cmd.substring(9);
      recordFilename.trim();
      Serial.println(">> Filename set to: " + recordFilename + ".csv");
    }

  /* ── label <text> ──────────────────────────────────────────────────── */
  } else if (cmd.startsWith("label ")) {
    recordLabel = cmd.substring(6);
    recordLabel.trim();
    Serial.println(">> Label set to: " + recordLabel);

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
    const char *modeStr = (mode == PLOTTING)  ? "plotting"  :
                          (mode == RECORDING) ? "recording" : "idle";
    Serial.println("Mode          : " + String(modeStr));
    Serial.println("Verbose       : " + String(verboseMode ? "ON" : "OFF"));
    Serial.println("Active sensor : " + String(sensorIdx + 1) +
                   " [" + String(PROFILES[sensorProfile[sensorIdx]].name) + "]");
    Serial.println("Filename      : " + recordFilename + ".csv");
    Serial.println("Label         : " + recordLabel);
    if (mode == RECORDING) {
      Serial.print("Recording     :");
      for (uint8_t i = 0; i < recordSensorCount; i++) {
        uint8_t si = recordSensors[i];
        Serial.print("  S" + String(si + 1) +
                     "[fp=" + String(fingerprintIndex[si]) + "]");
      }
      Serial.println();
    }
    Serial.println("Per-sensor profile assignments:");
    for (uint8_t i = 0; i < 8; i++) {
      Serial.println("  S" + String(i + 1) + ": profile " +
                     String(sensorProfile[i] + 1) +
                     " [" + String(PROFILES[sensorProfile[i]].name) + "]");
    }

  /* ── unknown ───────────────────────────────────────────────────────── */
  } else {
    Serial.println(">> Unknown: '" + cmd + "'");
    Serial.println("   Commands: plot | record [1 2 …] | stop | verbose | 1-8");
    Serial.println("             profile <N> | profiles | filename <name> | label <text> | status");
  }
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

  if (mode == PLOTTING) {
    commSetup = commMuxSetConfig(Wire, SPI, sensorIdx, commSetup);
    const HeatingProfile &prof = PROFILES[sensorProfile[sensorIdx]];
    if (prof.scanMode == SCAN_PARALLEL) {
      pollParallelMeasurement(sensorIdx);
      delay(10);
    } else if (prof.scanMode == SCAN_SEQUENTIAL) {
      pollSequentialMeasurement(sensorIdx);
      delay(10);
    } else {
      takeForcedMeasurement(sensorIdx);
      delay(500);
    }

  } else if (mode == RECORDING) {
    bool anyMultiStep = false;
    for (uint8_t i = 0; i < recordSensorCount; i++) {
      uint8_t si = recordSensors[i];
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

    /* For multi-step sessions yield to the ESP32 scheduler;
     * forced-mode measurements already include their own wait. */
    if (anyMultiStep) delay(10);

    /* Write all rows collected this iteration to the SD card.
     * Doing this after all sensor polls means SD latency no longer delays
     * any sensor's poll — each sensor is read as soon as the previous one
     * finishes its SPI transaction, not after its SD writes complete. */
    flushRowBuffer();
  }

  updateLed();
}
