/**
 * Training data statistics for feature standardisation.
 *
 * Used to z-score normalise each sensor reading before passing it to the
 * model:
 *
 *   z = (x - mean) / sqrt(variance)
 *
 * Feature order — must match the order used when training the model:
 *   [0]  gas_resistance   (Ω)
 *   [1]  temperature      (°C)
 *   [2]  pressure         (hPa)
 *   [3]  humidity         (%)
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │  PLACEHOLDER VALUES — replace with the actual mean and variance     │
 * │  computed from your training dataset before deploying.              │
 * └─────────────────────────────────────────────────────────────────────┘
 */

#ifndef STATISTICS_H
#define STATISTICS_H

/* Per-feature mean of the training set */
const float FEATURE_MEAN[4] = {
    35.8f,        // temperature     (°C)
    997.9f,      // pressure        (hPa)
    20.7f,        // humidity        (%)
    1275333.1f,    // gas resistance  (Ω)
};

/* Per-feature variance of the training set  (std = sqrt(variance)) */
const float FEATURE_VAR[4] = {
    4.3f,        // temperature     (°C²) →  std ≈ 5 °C
    14.4f,       // pressure        (hPa²)→  std ≈ 10 hPa
    12.8f,       // humidity        (%²)  →  std ≈ 15 %
    15002761438664.4f,      // gas resistance  (Ω²)  →  std ≈ 500 000 Ω
};

#endif /* STATISTICS_H */
