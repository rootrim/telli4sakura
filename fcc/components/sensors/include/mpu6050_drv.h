#pragma once

#include <esp_err.h>
#include <mpu6050.h>

/**
 * @brief Initializes the MPU6050 sensor.
 *
 * @param[in] sda_gpio SDA pin number.
 * @param[in] scl_gpio SCL pin number.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_drv_init(int sda_gpio, int scl_gpio);

/**
 * @brief Reads acceleration and rotation from MPU6050.
 *
 * @param[out] accel  Acceleration data (x, y, z) in g.
 * @param[out] gyro   Rotation data (x, y, z) in °/s.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_drv_read(mpu6050_acceleration_t *accel,
                           mpu6050_rotation_t *gyro);

/**
 * @brief Turns acceleration value to tilt value
 *
 * @param[in] accel mpu6050_acceleration_t value
 * @return float tilt value
 *
 */
float calc_tilt(const mpu6050_acceleration_t *accel);
