/**
 * Real-Time Inference — BME688 Dev Kit + LiteRT (TFLite Micro)
 *
 * Collects 10-step sequential heating scans from BME688 sensor and runs
 * CNN inference to classify odors (air, basil, cinnamon, oregano, rosemary).
 *
 * Model: CNN (10 timesteps × 8 features) → 5-class softmax
 * Features: temp, pressure, humidity, gas + deltas from previous step
 *
 * Serial commands: run | stop | profile <N> | profiles | status
 *
 * Hardware: Bosch BME688 Development Kit (ESP32 + 8-sensor SPI board)
 * Libraries: bme68xLibrary, espressif__esp-tflite-micro
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <bme68xLibrary.h>
#include "commMux.h"
#include "model.h"
#include "statistics.h"
#include "ble.h"

#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/schema/schema_generated.h>

/* Gas reading valid when both bits set */
#define GAS_VALID_MSK (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK)

/* Heating profile definitions */

struct HeatingProfile {
  const char *name;
  const char *description;
  uint8_t     profileLen;   // number of heater steps (max 10)
  uint16_t    tempProf[10]; // heater temperatures, °C
  uint16_t    durProf[10];  // heater durations, ms
  uint8_t     seqSleep;     // ODR constant, e.g. BME68X_ODR_250_MS
};

const HeatingProfile PROFILES[] = {
  { "Std",       "100,200,320 C / 150 ms each, 250 ms ODR",
    3,
    {100, 200, 320}, {150, 150, 150}, BME68X_ODR_250_MS },
  { "LinSweep",  "Linear sweep 200->400 C (10 steps / 150 ms, 500 ms ODR)",
    10,
    {200, 222, 244, 267, 289, 311, 333, 356, 378, 400},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150}, BME68X_ODR_500_MS },
  { "Bosch",     "Bosch 10-step: 320,100x3,200x3,320x3 C / 150 ms, 500 ms ODR",
    10,
    {320, 100, 100, 100, 200, 200, 200, 320, 320, 320},
    {150, 150, 150, 150, 150, 150, 150, 150, 150, 150}, BME68X_ODR_500_MS },
  { "WideSweep", "Wide sweep 100->450 C (10 steps, longer low-temp dwell, 500 ms ODR)",
    10,
    {100, 150, 200, 250, 300, 325, 350, 380, 415, 450},
    {200, 180, 160, 150, 150, 140, 140, 140, 140, 140}, BME68X_ODR_500_MS },
};

const uint8_t NUM_PROFILES = sizeof(PROFILES) / sizeof(PROFILES[0]);

/* Model configuration */
constexpr int kTensorArenaSize = 32 * 1024;
constexpr int NUM_CLASSES  = 5;
constexpr int NUM_FEATURES = 8;

const char* const CLASS_LABELS[NUM_CLASSES] = { "air", "basil", "cinnamon", "oregano", "rosemary"};

/* Globals */
Bme68x  bme;
commMux commSetup;

uint8_t  sensorIdx     = 0;
uint8_t  activeProfile = 1;  // default to LinSweep (10-step profile)
bool     running       = false;

/* Scan accumulation */
float   gasBuffer[10];
float   gasDiffBuffer[10];
bool    gasReceived[10];
float   lastTemp = 0, lastHum = 0, lastPres = 0;
float   diffTemp = 0, diffHum = 0, diffPres = 0;
int8_t  lastGasIdx   = -1;
bool    cycleHasData = false;

String cmdBuffer = "";

/* TFLite Micro objects */
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
static tflite::MicroMutableOpResolver<5> resolver;
static tflite::MicroInterpreter* interpreter = nullptr;

/* z-score normalization */
inline float standardise(float value, int step, int feat) {
  return (value - FEATURE_MEAN[step][feat]) / FEATURE_STD[step][feat];
}

/* Helpers */

void resetGasBuffer(uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    gasBuffer[i]   = 0.0f;
    gasReceived[i] = false;
  }
  lastGasIdx = -1;
}

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

/* Inference */
void runInference(const HeatingProfile &prof) {
  TfLiteTensor* inp = interpreter->input(0);

  if (inp->dims->size != 3) {
    Serial.println("[WARN] Model is not a sequence model (expected 3-D input).");
    return;
  }
  const int modelSeqLen = inp->dims->data[1];
  if ((int)prof.profileLen != modelSeqLen) {
    Serial.print("[WARN] Profile has ");
    Serial.print(prof.profileLen);
    Serial.print(" steps but model expects ");
    Serial.print(modelSeqLen);
    Serial.println(". Select matching profile.");
    return;
  }

  for (uint8_t t = 0; t < prof.profileLen; t++) {
    const float raw[NUM_FEATURES] = {
      lastTemp,
      lastPres,
      lastHum,
      gasBuffer[t],
      diffTemp,
      diffPres,
      diffHum,
      gasDiffBuffer[t],
    };

    Serial.print("Input array: ");
    int base = t * NUM_FEATURES;
    for (int i = 0; i < NUM_FEATURES; i++) {
      inp->data.f[base + i] = (FEATURE_STD[t][i] != 0.0f) ? standardise(raw[i], t, i) : raw[i];
      Serial.print(" ");
      Serial.print(inp->data.f[base + i]);
      Serial.print(" ");
    }
    Serial.println();
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

  // Construct JSON document
  JsonDocument doc;
  doc["op"] = 2;
  doc["d"]["prediction"] = CLASS_LABELS[predicted];

  JsonArray scores = doc["d"]["scores"].to<JsonArray>();

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

    JsonObject entry = scores.add<JsonObject>();
    entry["label"] = CLASS_LABELS[i];
    entry["score"] = out->data.f[i] * 100.0f;
  }
  Serial.println("]");

  // Transmit inference result to BLE client
  bleTransmit(doc);
}

/* Sensor initialization */
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

  uint16_t tArr[10], dArr[10];
  memcpy(tArr, prof.tempProf, prof.profileLen * sizeof(uint16_t));
  memcpy(dArr, prof.durProf,  prof.profileLen * sizeof(uint16_t));
  bme.setSeqSleep(prof.seqSleep);
  bme.setHeaterProf(tArr, dArr, prof.profileLen);

  return true;
}

/* Measurement polling */
void pollMeasurement() {
  const HeatingProfile &prof = PROFILES[activeProfile];

  uint8_t nFields = bme.fetchData();
  if (nFields == 0) return;

  bme68xData data;
  for (uint8_t f = 0; f < nFields; f++) {
    bme.getData(data);
    if (!(data.status & BME68X_NEW_DATA_MSK)) continue;

    uint8_t gi = data.gas_index;

    /* Cycle boundary detected by gas_index wrap-around */
    if (lastGasIdx >= 0 && (int8_t)gi < lastGasIdx) {
      if (cycleHasData && validateScanSize(prof.profileLen)) {
        Serial.print("Complete " + String(prof.profileLen) + "-step scan — ");
        runInference(prof);
      }
      cycleHasData = false;
      resetGasBuffer(prof.profileLen);
    }

    diffTemp = (lastTemp == 0.0f) ? 0.0f : (data.temperature - lastTemp);
    diffHum  = (lastHum == 0.0f) ? 0.0f : (data.humidity - lastHum);
    diffPres = (lastPres == 0.0f) ? 0.0f : (data.pressure / 100.0f - lastPres);

    lastTemp = data.temperature;
    lastHum  = data.humidity;
    lastPres = data.pressure / 100.0f;

    if ((data.status & GAS_VALID_MSK) == GAS_VALID_MSK) {
      gasDiffBuffer[gi] = (gi == 0) ? 0.0f : (gasReceived[gi - 1] ? (data.gas_resistance - gasBuffer[gi - 1]) : 0.0f);
      gasBuffer[gi]   = data.gas_resistance;
      gasReceived[gi] = true;
      cycleHasData    = true;
    }

    lastGasIdx = (int8_t)gi;
  }
}

/* Command handling */
void handleCommand(const String &cmd) {
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
    resetGasBuffer(prof.profileLen);
    bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    running = true;
    Serial.println(">> Running profile " + String(activeProfile + 1) +
                   " [" + String(prof.name) + "]");

  } else if (cmd == "stop") {
    if (!running) {
      Serial.println(">> Not running.");
      return;
    }
    bme.setOpMode(BME68X_SLEEP_MODE);
    running = false;
    Serial.println(">> Stopped.");

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
      Serial.println(">> Profile set to " + String(n) + " [" + String(PROFILES[activeProfile].name) + "]");
    }

  } else if (cmd == "profiles") {
    Serial.println("--- Available profiles ---");
    for (uint8_t i = 0; i < NUM_PROFILES; i++) {
      Serial.println("  " + String(i + 1) + ": " + String(PROFILES[i].name) + " — " + String(PROFILES[i].description));
    }
    Serial.println("--------------------------");

  } else if (cmd == "status") {
    const HeatingProfile &prof = PROFILES[activeProfile];
    Serial.println("Running : " + String(running ? "yes" : "no"));
    Serial.println("Sensor  : " + String(sensorIdx + 1));
    Serial.println("Profile : " + String(activeProfile + 1) + " [" + String(prof.name) + "] — " + String(prof.description));

  } else {
    Serial.println(">> Unknown: '" + cmd + "'");
    Serial.println("   Commands: run | stop | profile <N> | profiles | status");
  }
}

/* Setup */
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

  const tflite::Model* tfl_model = tflite::GetModel(g_model);

  if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERR: Model schema version mismatch (" + String(tfl_model->version()) +
                   " vs " + String(TFLITE_SCHEMA_VERSION) + ")");
    while (1);
  }

  resolver.AddReshape();
  resolver.AddConv2D();
  resolver.AddMean();
  resolver.AddFullyConnected();
  resolver.AddSoftmax();

  static tflite::MicroInterpreter static_interpreter(tfl_model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  if (interpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERR: AllocateTensors() failed.");
    while (1);
  }

  TfLiteTensor* inp = interpreter->input(0);
  TfLiteTensor* out = interpreter->output(0);
  String inpShape = "";
  for (int d = 0; d < inp->dims->size; d++) {
    if (d > 0) inpShape += "x";
    inpShape += String(inp->dims->data[d]);
  }
  Serial.println("Model OK | input " + inpShape + " output " + String(out->dims->data[0]) + "x" + String(out->dims->data[1]));
  Serial.println("Arena: " + String(interpreter->arena_used_bytes()) + " / " + String(kTensorArenaSize) + " bytes");
  Serial.println("Profile: " + String(activeProfile + 1) + " [" + String(PROFILES[activeProfile].name) + "]");
  Serial.println("Commands: run | stop | profile <N> | profiles | status");
}

/* Main loop */
void loop() {
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
  pollMeasurement();
  delay(10);
}
