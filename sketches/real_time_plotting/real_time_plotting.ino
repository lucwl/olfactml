/**
 * Real-Time Plotting — BME688 Dev Kit
 *
 * Streams temperature, humidity, pressure, and gas resistance from one of the
 * 8 BME688 sensors on the Bosch dev kit in Arduino Serial Plotter format.
 *
 * Serial commands:
 *   start  — begin streaming the active sensor
 *   stop   — pause streaming
 *   1–8    — (only when stopped) switch to that sensor number
 *
 * Required libraries: bme68xLibrary (Bosch Sensortec)
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "commMux.h"

/* ── State ────────────────────────────────────────────────────────────── */
Bme68x  bme;
commMux commSetup;

uint8_t sensorIdx = 0;      // 0-based; default = sensor 1
bool    plotting  = false;

String  cmdBuffer = "";

/* ── Sensor initialisation ────────────────────────────────────────────── */
void initSensor(uint8_t idx) {
  commSetup = commMuxSetConfig(Wire, SPI, idx, commSetup);
  bme.begin(BME68X_SPI_INTF, commMuxRead, commMuxWrite, commMuxDelay, &commSetup);

  if (bme.checkStatus()) {
    Serial.println("ERR: Sensor " + String(idx + 1) + " init failed: " + bme.statusString());
    return;
  }

  /* Oversampling: temp 8x, pressure 4x, humidity 2x */
  bme.setTPH(BME68X_OS_8X, BME68X_OS_4X, BME68X_OS_2X);

  /* IIR filter size 3 */
  bme.setFilter(BME68X_FILTER_SIZE_3);

  /* Single heater step for forced mode: 320 °C, 150 ms */
  bme.setHeaterProf(320, 150);
}

/* ── Setup ────────────────────────────────────────────────────────────── */
void setup() {
  Serial.begin(921600);
  Serial.println("Initializing...");

  commMuxBegin(Wire, SPI);
  delay(100);

  initSensor(sensorIdx);

  Serial.println("Ready. Commands: start | stop | 1-8");
  Serial.println("Active sensor: 1");
}

/* ── Command handling ─────────────────────────────────────────────────── */
void handleCommand(const String &cmd) {
  if (cmd == "start") {
    plotting = true;
    Serial.println("Plotting sensor " + String(sensorIdx + 1));

  } else if (cmd == "stop") {
    plotting = false;
    Serial.println("Stopped. Active sensor: " + String(sensorIdx + 1));

  } else if (cmd.length() == 1 && cmd[0] >= '1' && cmd[0] <= '8') {
    if (plotting) {
      Serial.println("Stop plotting before switching sensors.");
    } else {
      sensorIdx = (uint8_t)(cmd[0] - '1');
      initSensor(sensorIdx);
      Serial.println("Switched to sensor " + String(sensorIdx + 1) + ". Send 'start' to begin.");
    }

  } else {
    Serial.println("Unknown command. Use: start | stop | 1-8");
  }
}

/* ── Single measurement + Serial Plotter output ───────────────────────── */
void takeMeasurement() {
  bme.setOpMode(BME68X_FORCED_MODE);
  delayMicroseconds(bme.getMeasDur(BME68X_FORCED_MODE));

  bme68xData data;
  if (!bme.fetchData()) return;
  bme.getData(data);

  if (!(data.status & BME68X_NEW_DATA_MSK))   return;
  if (!(data.status & BME68X_GASM_VALID_MSK)) return;

  /* Arduino Serial Plotter format: Label:value pairs separated by spaces.
   * All four channels are plotted simultaneously on the same graph. */
  Serial.print("Temperature:");
  Serial.print(data.temperature, 2);
  Serial.print(" Humidity:");
  Serial.print(data.humidity, 2);
  Serial.print(" Pressure:");
  Serial.print(data.pressure / 100.0f, 2);   // Pa → hPa
  Serial.print(" GasResistance:");
  Serial.println(data.gas_resistance, 0);     // Ω, no decimals needed
}

/* ── Main loop ────────────────────────────────────────────────────────── */
void loop() {
  /* Non-blocking serial command reader — accumulate until newline */
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

  if (plotting) {
    takeMeasurement();
    delay(500);
  }
}
