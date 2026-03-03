/**
 * SD Card Logger — BME688 Dev Kit
 *
 * Provides SD card initialisation, file creation / continuation, and
 * row-level CSV recording.
 *
 * CSV columns (one row per heater step / measurement):
 *   sensor_index, fingerprint_index, position, plate_temperature,
 *   heater_duration, temperature, pressure, humidity, gas_resistance, label
 *
 *   sensor_index      — 1-based sensor number
 *   fingerprint_index — running scan-cycle counter (reset each recording session)
 *   position          — heater-step index within the scan cycle (0-9 parallel; 0 forced)
 *   plate_temperature — target heater temperature for this step / reading, °C
 *   heater_duration   — heater-on time for this step / reading, ms
 *   temperature       — ambient temperature, °C
 *   pressure          — ambient pressure, hPa
 *   humidity          — relative humidity, %
 *   gas_resistance    — gas resistance, Ω
 *   label             — user-defined specimen / class label
 *
 * Forced mode:   one row per measurement; position = 0.
 * Parallel mode: one row per heater step; position = gas_index (0–9).
 *                All steps of one scan cycle share the same fingerprint_index.
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "Arduino.h"

/**
 * @brief Initialise the SD card.  Call once in setup().
 * @return true if a card is present and successfully initialised.
 */
bool sdInit();

/**
 * @brief Open (or create) a CSV file on the SD card.
 *
 *        If the file already exists it is opened for appending so that
 *        a recording session can be resumed after a power cycle.
 *        A header row is written only when the file is first created.
 *
 * @param filename  Base name without extension, e.g. "lavender_run1".
 *                  ".csv" is appended automatically if not present.
 * @return true on success.
 */
bool sdOpenFile(const String &filename);

/**
 * @brief Append one data row to the currently open CSV file.
 *
 * @param sensorIndex      1-based sensor number.
 * @param fingerprintIndex Running scan-cycle counter (resets each session).
 * @param position         Heater-step index within the cycle (0–9 / 0).
 * @param plateTemperature Target heater temperature for this step, °C.
 * @param heaterDuration   Heater-on time for this step, ms.
 * @param temperature      Ambient temperature, °C.
 * @param pressure         Ambient pressure, hPa.
 * @param humidity         Relative humidity, %.
 * @param gasResistance    Gas resistance, Ω.
 * @param label            User-defined specimen / class label.
 */
void sdLogRow(uint8_t       sensorIndex,
              uint32_t      fingerprintIndex,
              uint8_t       position,
              uint16_t      plateTemperature,
              uint16_t      heaterDuration,
              float         temperature,
              float         pressure,
              float         humidity,
              float         gasResistance,
              const String &label);

#endif /* SD_LOGGER_H */
