#pragma once

#define ACCEL_SENS 16384.0f
#define GYRO_SENS 131.0f
#define DEG2RAD 0.0174533f

typedef struct {
  float pitch;
  float roll;
} TiltState;

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
 * @param[in] magnitude magnitude value
 * @param[in] accel_z z-acceleration value
 * @return float tilt value
 *
 */
void tilt_update(TiltState *state, float ax_g, float ay_g, float az_g,
                 float gx_dps, float gy_dps, float gz_dps, float dt);
