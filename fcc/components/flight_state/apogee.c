#include "include/apogee.h"
#include <string.h>

static float s_alt_window[APOGEE_WINDOW];
static int s_window_idx;
static int s_window_count;
static bool s_altitude_lock;
static bool s_apogee_fired;

void flight_state_init(void) {
  memset(s_alt_window, 0, sizeof(s_alt_window));
  s_window_idx = 0;
  s_window_count = 0;
  s_altitude_lock = false;
  s_apogee_fired = false;
}

bool flight_state_update(float altitude_m, float tilt_deg) {
  // Once apogee fires, never fire again
  if (s_apogee_fired)
    return false;

  // Altitude lock check
  if (!s_altitude_lock && altitude_m >= APOGEE_MIN_ALTITUDE_M) {
    s_altitude_lock = true;
  }
  if (!s_altitude_lock)
    return false;

  // Fill sliding window
  s_alt_window[s_window_idx] = altitude_m;
  s_window_idx = (s_window_idx + 1) % APOGEE_WINDOW;
  if (s_window_count < APOGEE_WINDOW)
    s_window_count++;

  // Need full window before deciding
  if (s_window_count < APOGEE_WINDOW)
    return false;

  // Check if all samples in window are descending
  bool descending = true;
  for (int i = 0; i < APOGEE_WINDOW - 1; i++) {
    int curr = (s_window_idx + i) % APOGEE_WINDOW;
    int next = (s_window_idx + i + 1) % APOGEE_WINDOW;
    if (s_alt_window[next] >= s_alt_window[curr]) {
      descending = false;
      break;
    }
  }

  // Tilt check
  bool tilted = (tilt_deg >= APOGEE_TILT_THRESHOLD);

  if (descending && tilted) {
    s_apogee_fired = true;
    return true;
  }

  return false;
}
