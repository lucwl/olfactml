/**
 * Inference Input Logger — SD-card CSV debug log.
 *
 * Records the exact floating-point input tensor that is fed to the model
 * at each inference step.  Completely self-contained: it makes no
 * assumptions about model architecture, scan mode, number of features,
 * or any other inference detail.
 *
 * CSV columns:
 *   inference_index, label, input_0, input_1, …, input_{N-1}
 *
 * Usage:
 *   1. Call ilInit() once in setup().
 *   2. Call ilOpen(filename, numInputs) to start a log session.
 *      Appends to an existing file; writes the header only for new files.
 *   3. Call ilLog(data, numInputs, label) after building each input tensor.
 *   4. Call ilClose() when done.
 */

#ifndef INFERENCE_LOGGER_H
#define INFERENCE_LOGGER_H

#include "Arduino.h"

/**
 * Initialise the SD card.  Call once in setup().
 * @return true if a card is found and initialised successfully.
 */
bool ilInit();

/**
 * Open (or continue) a CSV log file on the SD card.
 * Appends to an existing file; writes the CSV header only for new files.
 *
 * @param filename   Base name without extension (e.g. "debug_run1").
 *                   ".csv" is appended automatically if absent.
 * @param numInputs  Number of input elements per inference row.
 *                   Used to generate the column names in the header.
 * @return true on success.
 */
bool ilOpen(const String &filename, int numInputs);

/**
 * Append one row to the open log file.
 * Safe to call even if ilOpen() has not been called (no-op).
 *
 * @param inputs     Pointer to the flattened input tensor.
 * @param numInputs  Number of elements to write.
 * @param label      Human-readable label string (e.g. "basil" or "none").
 */
void ilLog(const float *inputs, int numInputs, const char *label);

/**
 * Stop logging and print a summary.  Safe to call when no file is open.
 */
void ilClose();

#endif /* INFERENCE_LOGGER_H */
