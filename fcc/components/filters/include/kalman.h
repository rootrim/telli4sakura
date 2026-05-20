#pragma once

/**
 * @brief Kalman filter state.
 *
 * One instance per filtered value.
 * q and r must be tuned per sensor:
 *   - q (process noise): higher = trusts measurement more, faster response
 *   - r (measurement noise): higher = trusts prediction more, smoother output
 */
typedef struct {
  float x; /*!< Current state estimate            */
  float p; /*!< Estimate error covariance          */
  float q; /*!< Process noise covariance           */
  float r; /*!< Measurement noise covariance       */
} kalman_t;

/**
 * @brief Initializes a Kalman filter instance.
 *
 * @param[out] k        Kalman filter instance.
 * @param[in]  q        Process noise covariance.
 * @param[in]  r        Measurement noise covariance.
 * @param[in]  initial  Initial state estimate.
 */
void kalman_init(kalman_t *k, float q, float r, float initial);

/**
 * @brief Updates the Kalman filter with a new measurement.
 *
 * @param[in,out] k           Kalman filter instance.
 * @param[in]     measurement New measurement value.
 * @return float              Filtered estimate.
 */
float kalman_update(kalman_t *k, float measurement);
