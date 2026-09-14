#include "kalman.h"

void kalman_init(kalman_t *k, float q, float r, float initial) {
  k->x = initial;
  k->p = 1.0f;
  k->q = q;
  k->r = r;
  k->max_nis = 1000000.0f;
}

float kalman_update(kalman_t *k, float measurement) {
  /* Predict */
  k->p += k->q;

  float innovation = measurement - k->x;

  float s = k->p + k->r;

  float nis = (innovation * innovation) / s;

  if (nis > k->max_nis) {
    return k->x;
  }

  float gain = k->p / s;
  k->x += gain * innovation;
  k->p = (1.0f - gain) * k->p;
  return k->x;
}
