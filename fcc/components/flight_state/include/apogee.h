#pragma once

#include <stdbool.h>

// TODO: tune these values
#define APOGEE_MIN_ALTITUDE_M                                                  \
  2500.0f // Altitude lock: must exceed this before apogee can trigger
#define APOGEE_TILT_THRESHOLD                                                  \
  25.0f                  // Tilt angle (degrees) above which apogee can trigger
#define APOGEE_WINDOW 20 // Number of samples used to detect descending altitude

/**
 * @brief Initializes the flight state module.
 */
void flight_state_init(void);

/**
 * @brief Updates the flight state with new sensor data.
 *
 * Call this every loop iteration (5Hz).
 *
 * @param[in] altitude_m  Filtered altitude in meters.
 * @param[in] tilt_deg    Tilt angle in degrees (0 = vertical, 90 = horizontal).
 * @return true  Apogee detected.
 * @return false Not at apogee.
 */
bool flight_state_update(float altitude_m, float tilt_deg);
