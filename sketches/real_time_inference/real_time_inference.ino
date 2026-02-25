/**
 * Real-Time Inference — BME688 Dev Kit + LiteRT (TFLite Micro)
 *
 * Takes a forced-mode measurement from one BME688 sensor, z-score
 * normalises the four features using the training statistics in
 * statistics.h, and runs them through a neural network model to predict
 * one of four scent classes:
 *
 *   0 — lavender
 *   1 — grapefruit
 *   2 — eucalyptus
 *   3 — air
 *
 * Model architecture (inferred from model.tflite):
 *   Input [1, 4]  →  Dense(16, ReLU)  →  Dense(16, ReLU)
 *                 →  Dense( 4, Softmax)  →  Output [1, 4]
 *
 * Feature order fed to the model:
 *   [0] temperature    (°C)
 *   [1] pressure       (hPa)
 *   [2] humidity       (%)
 *   [3] gas_resistance (Ω)
 *
 * Hardware: Bosch BME688 Development Kit (Adafruit ESP32 Feather +
 *           8-sensor SPI board via I2C GPIO expander).
 *
 * Required libraries:
 *   • bme68xLibrary  (Bosch Sensortec)
 *   • TFLite Micro: uses espressif__esp-tflite-micro, which is bundled with the
 *     ESP32 Arduino core 3.3.7+. No external library needed.
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"
#include "model.h"
#include "statistics.h"

/* TFLite Micro --------------------------------------------------------- */
/* Uses espressif__esp-tflite-micro, bundled with ESP32 Arduino core 3.3.7+.
 * Headers are always in the compiler include path — no external library needed. */
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* ═══════════════════════════════════════════════════════════════════════
 * Configuration
 * ═══════════════════════════════════════════════════════════════════════ */

#define SENSOR_IDX 0        // 0-based sensor index on the dev kit (sensor 1)
#define INFERENCE_INTERVAL_MS 1000

/* Tensor arena — must be large enough for all intermediate tensors.
 * This model (4→16→16→4 float32) fits comfortably within 10 kB. */
constexpr int kTensorArenaSize = 10 * 1024;

/* ═══════════════════════════════════════════════════════════════════════
 * Class labels  (order must match the model's output indices)
 * ═══════════════════════════════════════════════════════════════════════ */

constexpr int NUM_CLASSES = 4;
constexpr int NUM_FEATURES = 4;
const char* const CLASS_LABELS[NUM_CLASSES] = {
    "lavender",
    "grapefruit",
    "eucalyptus",
    "air",
};

/* ═══════════════════════════════════════════════════════════════════════
 * Globals
 * ═══════════════════════════════════════════════════════════════════════ */

/* Hardware */
Bme68x  bme;
commMux commSetup;

/* TFLite Micro objects in static storage (required by the runtime) */
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
// Model needs: FullyConnected (3× dense layers with fused ReLU) + Softmax
static tflite::MicroMutableOpResolver<2> resolver;
static tflite::MicroInterpreter* interpreter = nullptr;

/* ═══════════════════════════════════════════════════════════════════════
 * Feature standardisation
 * ═══════════════════════════════════════════════════════════════════════ */

/* z-score:  z = (x − μ) / σ,  where σ = sqrt(variance) */
inline float standardise(float value, int feature_idx) {
    return (value - FEATURE_MEAN[feature_idx]) / sqrtf(FEATURE_VAR[feature_idx]);
}

/* ═══════════════════════════════════════════════════════════════════════
 * Setup
 * ═══════════════════════════════════════════════════════════════════════ */

void setup() {
    Serial.begin(921600);
    Serial.println("=== Real-Time Inference — BME688 + LiteRT ===");

    /* ── Sensor ──────────────────────────────────────────────────────── */
    commMuxBegin(Wire, SPI);
    delay(100);

    commSetup = commMuxSetConfig(Wire, SPI, SENSOR_IDX, commSetup);
    bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup);

    if (bme.checkStatus()) {
        Serial.println("ERR: Sensor init failed — " + bme.statusString());
        while (1);
    }

    bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);
    bme.setFilter(BME68X_FILTER_SIZE_3);
    bme.setHeaterProf(320, 150);

    Serial.println("Sensor OK (index " + String(SENSOR_IDX) + ")");

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

    /* Sanity-check tensor shapes */
    TfLiteTensor* inp = interpreter->input(0);
    TfLiteTensor* out = interpreter->output(0);

    Serial.println("Model OK  |  input "
                   + String(inp->dims->data[0]) + "x" + String(inp->dims->data[1])
                   + "  output "
                   + String(out->dims->data[0]) + "x" + String(out->dims->data[1]));
    Serial.println("Arena used: " + String(interpreter->arena_used_bytes()) + " / "
                   + String(kTensorArenaSize) + " bytes");
    Serial.println("---------------------------------------------");
    Serial.println("Classes: lavender | grapefruit | eucalyptus | air");
    Serial.println("---------------------------------------------");
}

/* ═══════════════════════════════════════════════════════════════════════
 * Main loop
 * ═══════════════════════════════════════════════════════════════════════ */

void loop() {
    /* ── Take a forced-mode measurement ──────────────────────────────── */
    bme.setOpMode(BME68X_FORCED_MODE);
    delayMicroseconds(bme.getMeasDur(BME68X_FORCED_MODE));

    bme68xData data;
    if (!bme.fetchData()) {
        delay(INFERENCE_INTERVAL_MS);
        return;
    }
    bme.getData(data);

    if (!(data.status & BME68X_NEW_DATA_MSK)) {
        delay(INFERENCE_INTERVAL_MS);
        return;
    }
    if (!(data.status & BME68X_GASM_VALID_MSK)) {
        Serial.println("[WARN] Gas measurement not stable yet, skipping.");
        delay(INFERENCE_INTERVAL_MS);
        return;
    }

    /* ── Build raw feature vector ────────────────────────────────────── */
    // keep in mind the order of features
    const float raw[NUM_FEATURES] = {
        //data.temperature,
        0.15 * FEATURE_VAR[0] + FEATURE_MEAN[0],
        data.pressure / 100.0f,
        //data.humidity,
        -1.0 * FEATURE_VAR[2] + FEATURE_MEAN[2],
        data.gas_resistance,
    };

    Serial.print("Raw scan: ");
    Serial.print("[");
    for (int i = 0; i < NUM_FEATURES; i++) {
        Serial.print(" ");
        Serial.print(raw[i]);
        Serial.print(" ");
    }
    Serial.print("]");
    Serial.println();

    Serial.print("Input array: ");
    Serial.print("[");
    for (int i = 0; i < NUM_FEATURES; i++) {
        Serial.print(" ");
        Serial.print(standardise(raw[i], i));
        Serial.print(" ");
    }
    Serial.print("]");
    Serial.println();

    /* ── Standardise and write to input tensor ───────────────────────── */
    TfLiteTensor* input_tensor = interpreter->input(0);
    for (int i = 0; i < 4; i++) {
        input_tensor->data.f[i] = standardise(raw[i], i);
    }

    /* ── Run inference ───────────────────────────────────────────────── */
    if (interpreter->Invoke() != kTfLiteOk) {
        Serial.println("ERR: Invoke() failed.");
        delay(INFERENCE_INTERVAL_MS);
        return;
    }

    /* ── Read output (class probabilities after Softmax) ─────────────── */
    TfLiteTensor* output_tensor = interpreter->output(0);

    int   predicted  = 0;
    float best_score = output_tensor->data.f[0];
    for (int i = 1; i < NUM_CLASSES; i++) {
        if (output_tensor->data.f[i] > best_score) {
            best_score = output_tensor->data.f[i];
            predicted  = i;
        }
    }

    /* ── Log prediction ──────────────────────────────────────────────── */
    Serial.print("Prediction: ");
    Serial.print(CLASS_LABELS[predicted]);
    Serial.print("  (");
    Serial.print(best_score * 100.0f, 1);
    Serial.print("%)    scores: [");
    for (int i = 0; i < NUM_CLASSES; i++) {
        Serial.print(CLASS_LABELS[i]);
        Serial.print("=");
        Serial.print(output_tensor->data.f[i] * 100.0f, 1);
        Serial.print("%");
        if (i < NUM_CLASSES - 1) Serial.print("  ");
    }
    Serial.println("]");

    delay(INFERENCE_INTERVAL_MS);
}
