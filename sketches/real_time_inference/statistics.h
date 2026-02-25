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
const float FEATURE_MEAN[6] = {
    320.0f,
    150.0f,
    31.8f,        // temperature     (°C)
    999.8f,      // pressure        (hPa)
    27.8f,        // humidity        (%)
    111772.5f,    // gas resistance  (Ω)
};

/* Per-feature standard deviation of the training set */
const float FEATURE_STD[6] = {
    0.0f,
    0.0f,
    1.3f,        // temperature     
    1.5f,       // pressure        
    3.0f,       // humidity        
    68951.5f,      // gas resistance
};

#endif /* STATISTICS_H */
