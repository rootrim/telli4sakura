#pragma once

#include <bmp390.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

/**
 * @brief Initializes the BMP390 sensor on the given I2C master bus.
 *
 * @param[in] bus_handle I2C master bus handle (created in main).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_drv_init(i2c_master_bus_handle_t bus_handle);

/**
 * @brief Reads pressure and temperature from BMP390.
 *
 * @param[out] pressure_pa  Pressure in Pascal.
 * @param[out] temp_c       Temperature in Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_drv_read(float *pressure_pa, float *temp_c);
