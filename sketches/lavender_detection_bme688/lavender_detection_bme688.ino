/**
 * Lavender Detection — BME688 Dev Kit
 *
 * KNN classifier (k=1, samples=50) ported from the Adafruit-based
 * lavender_detection sketch to run on the Bosch BME688 development kit
 * (Adafruit ESP32 Feather + 8-sensor SPI board).
 *
 * Uses sensor index 0. The commMux layer selects it via the I2C GPIO
 * expander exactly as in the original data-logging sketch.
 *
 * Required libraries: bme68xLibrary (Bosch Sensortec)
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"

/* ── Hardware ─────────────────────────────────────────────────────────── */
#define SENSOR_IDX 0   // which of the 8 dev-kit sensors to use
const int LED_PIN = 2;

Bme68x    bme;
commMux   commSetup;

/* ── KNN parameters ───────────────────────────────────────────────────── */
const int K       = 20;
const int N_TRAIN = 50;

/* Scaler (mean / std) fit on the training set.
 * Feature order: [gas_resistance(Ω), temperature(°C), pressure(hPa), humidity(%)] */
const float SCALER_MEAN[4] = {
  2189607.11f, 36.58f,
  995.12f,     21.93f
};
const float SCALER_STD[4] = {
  5359799.95f, 2.26f,
  1.98f,       5.21f
};

/* Training samples — same values as in lavender_detection.ino */
const float X_train[N_TRAIN][4] = {
  {21079446.00f, 36.60f, 992.98f, 15.39f},
  {460846.09f,   35.09f, 992.90f, 19.24f},
  {153018.53f,   38.93f, 992.80f, 15.46f},
  {16873.19f,    39.66f, 997.25f, 22.79f},
  {13355.59f,    36.09f, 996.42f, 26.08f},
  {140543.52f,   36.44f, 993.03f, 18.01f},
  {18197.33f,    37.28f, 997.13f, 25.47f},
  {9975.06f,     40.31f, 997.46f, 21.90f},
  {91233.07f,    38.42f, 996.88f, 22.08f},
  {140891.58f,   37.10f, 993.11f, 16.92f},
  {434727.22f,   35.80f, 993.06f, 17.40f},
  {88858.04f,    38.16f, 996.76f, 22.31f},
  {677024.81f,   36.82f, 992.95f, 16.99f},
  {412653.62f,   35.34f, 993.06f, 18.10f},
  {875026.69f,   29.66f, 996.03f, 31.17f},
  {133926.23f,   38.52f, 997.29f, 22.41f},
  {127331.51f,   38.65f, 992.80f, 14.95f},
  {31880.45f,    37.04f, 997.03f, 25.09f},
  {15644784.00f, 35.95f, 993.15f, 17.91f},
  {26611.23f,    39.96f, 997.02f, 22.34f},
  {54794.52f,    33.08f, 996.50f, 28.79f},
  {25376.69f,    36.59f, 996.92f, 26.83f},
  {343278.59f,   37.26f, 996.82f, 24.58f},
  {11004.13f,    37.66f, 997.26f, 23.93f},
  {1473911.50f,  35.63f, 992.78f, 17.74f},
  {37780.40f,    28.63f, 996.37f, 39.33f},
  {412404.34f,   36.61f, 993.35f, 17.65f},
  {1362155.00f,  36.15f, 992.78f, 17.82f},
  {164365.97f,   38.89f, 992.82f, 15.00f},
  {2415806.50f,  33.07f, 993.02f, 21.15f},
  {443770.31f,   34.93f, 992.97f, 17.58f},
  {23911.82f,    39.76f, 997.29f, 21.80f},
  {203255.27f,   36.97f, 997.09f, 24.17f},
  {9572.24f,     38.48f, 997.38f, 23.96f},
  {76784.64f,    36.12f, 996.44f, 26.66f},
  {3191273.75f,  35.43f, 992.95f, 19.26f},
  {11653.31f,    37.50f, 996.84f, 25.92f},
  {23035500.00f, 36.73f, 993.14f, 16.51f},
  {11465.42f,    35.72f, 997.20f, 29.84f},
  {25152.29f,    37.67f, 997.12f, 24.02f},
  {26561.53f,    38.96f, 996.75f, 24.28f},
  {27432.49f,    37.10f, 996.98f, 26.32f},
  {174624.83f,   38.71f, 993.19f, 16.69f},
  {131958.77f,   37.13f, 993.09f, 16.68f},
  {8419322.00f,  34.24f, 993.02f, 17.92f},
  {10012.52f,    34.41f, 996.78f, 30.46f},
  {11702857.00f, 35.12f, 993.03f, 16.75f},
  {11038.29f,    38.32f, 996.99f, 23.98f},
  {15031193.00f, 35.48f, 993.10f, 17.19f},
  {34934.50f,    34.98f, 997.12f, 31.81f}
};

const int y_train[N_TRAIN] = {
  0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1,
  1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1,
  1, 1, 0, 0, 0, 1, 0, 1, 0, 1
};

/* ── Setup ────────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(921600);
  pinMode(LED_PIN, OUTPUT);
  Serial.println("Initializing...");

  /* Start SPI + I2C mux (mirrors commMuxBegin in the dev-kit sketch) */
  commMuxBegin(Wire, SPI);
  delay(100);

  /* Configure communication for sensor 0 */
  commSetup = commMuxSetConfig(Wire, SPI, SENSOR_IDX, commSetup);
  bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup);

  if (bme.checkStatus()) {
    Serial.println("Sensor init failed: " + bme.statusString());
    while (1);
  }

  /* Oversampling — matches the Adafruit config (temp 8x, pres 4x, hum 2x) */
  bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);

  /* IIR filter size 3 */
  bme.setFilter(BME68X_FILTER_SIZE_3);

  /* Single heater step for forced mode: 320 °C, 150 ms */
  bme.setHeaterProf(320, 150);

  Serial.println("Ready");
}

/* ── KNN prediction ───────────────────────────────────────────────────── */
int predict() {
  /* Trigger one forced-mode measurement */
  bme.setOpMode(BME68X_FORCED_MODE);

  /* Wait for the measurement to complete (getMeasDur returns µs) */
  delayMicroseconds(bme.getMeasDur(BME68X_FORCED_MODE));

  bme68xData data;
  if (!bme.fetchData()) return 0;
  bme.getData(data);

  /* Require valid new data and a stable heater */
  if (!(data.status & BME68X_NEW_DATA_MSK))  return 0;
  if (!(data.status & BME68X_GASM_VALID_MSK)) return 0;

  /* Feature order must match training: gas Ω, temp °C, pres hPa, hum % */
  float features[4] = {
    data.gas_resistance,
    data.temperature,
    data.pressure / 100.0f,  // Pa → hPa
    data.humidity
  };

  /* Normalise */
  for (int i = 0; i < 4; i++) {
    features[i] = (features[i] - SCALER_MEAN[i]) / SCALER_STD[i];
  }

  /* Squared Euclidean distances to every training sample */
  float distances[N_TRAIN];
  for (int i = 0; i < N_TRAIN; i++) {
    float sum = 0;
    for (int j = 0; j < 4; j++) {
      float x_norm = (X_train[i][j] - SCALER_MEAN[j]) / SCALER_STD[j];
      float diff   = features[j] - x_norm;
      sum += diff * diff;
    }
    distances[i] = sum;
  }

  /* Vote across K nearest neighbours */
  int vote_lavender = 0;
  for (int iter = 0; iter < K; iter++) {
    int   min_idx  = 0;
    float min_dist = distances[0];
    for (int i = 1; i < N_TRAIN; i++) {
      if (distances[i] < min_dist) {
        min_dist = distances[i];
        min_idx  = i;
      }
    }
    if (y_train[min_idx] == 1) vote_lavender++;
    distances[min_idx] = 999999.0f;  // exclude from next iteration
  }

  return (vote_lavender > K / 2) ? 1 : 0;
}

/* ── Main loop ────────────────────────────────────────────────────────── */
void loop() {
  int result = predict();
  digitalWrite(LED_PIN, result ? HIGH : LOW);
  Serial.println(result ? "Lavender detected" : "No lavender");
  delay(500);
}
