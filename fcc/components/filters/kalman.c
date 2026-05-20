#include "kalman.h"

void kalman_init(kalman_t *k, float q, float r, float initial) {
  k->x = initial;
  k->p = 1.0f;
  k->q = q;
  k->r = r;
}

float kalman_update(kalman_t *k, float measurement) {
  // Predict
  k->p = k->p + k->q;

  // Update
  float gain = k->p / (k->p + k->r);
  k->x = k->x + gain * (measurement - k->x);
  k->p = (1.0f - gain) * k->p;

  return k->x;
}
