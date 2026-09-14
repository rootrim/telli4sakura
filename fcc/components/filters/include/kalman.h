#pragma once

typedef struct {
  float x;       /*!< Current state estimate             */
  float p;       /*!< Estimate error covariance          */
  float q;       /*!< Process noise covariance           */
  float r;       /*!< Measurement noise covariance       */
  float max_nis; /*!< Reject threshold for normalized innovation squared */
} kalman_t;

void kalman_init(kalman_t *k, float q, float r, float initial);

float kalman_update(kalman_t *k, float measurement);
