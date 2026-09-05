#pragma once
#include "esp_err.h"

esp_err_t bmp280_drv_init(int sda_gpio, int scl_gpio);
esp_err_t bmp280_drv_read(float *pressure_pa, float *temp_c);
