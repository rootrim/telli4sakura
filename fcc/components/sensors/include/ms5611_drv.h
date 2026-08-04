#pragma once

#define SIT_FAKE_PRESSURE_BASE_PA 100332.0f

#include "esp_err.h"
#include "ms5611.h"

/**
 * Initializes MS5611
 * @param sda_gpio  SDA pin number
 * @param scl_gpio  SCL pin number
 */
esp_err_t ms5611_drv_init(int sda_gpio, int scl_gpio);

/**
 * Reads pressure and tempreture.
 * @param pressure_pa  Pascal
 * @param temp_c       Celsius
 */
esp_err_t ms5611_drv_read(int32_t *pressure_pa, float *temp_c);

float generate_pressure_noise(void);
