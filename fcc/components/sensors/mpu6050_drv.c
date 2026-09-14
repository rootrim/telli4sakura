#include "mpu6050_drv.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "mpu6050_drv";
static mpu6050_dev_t dev;

esp_err_t mpu6050_drv_init(int sda_gpio, int scl_gpio) {
  esp_err_t ret;

  ret = mpu6050_init_desc(&dev, MPU6050_I2C_ADDRESS_LOW, I2C_NUM_0, sda_gpio,
                          scl_gpio);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Descriptor init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = mpu6050_init(&dev);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_2000);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set gyro range: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_16);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set accel range: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Initialized at 0x68");
  ESP_LOGI(TAG, "Gyro range: ±2000 deg/s");
  ESP_LOGI(TAG, "Accel range: ±16 g");

  return ESP_OK;
}

esp_err_t mpu6050_drv_read(mpu6050_acceleration_t *accel,
                           mpu6050_rotation_t *gyro) {
  esp_err_t ret = mpu6050_get_motion(&dev, accel, gyro);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

void tilt_update(TiltState *state, float ax_g, float ay_g, float az_g,
                 float gx_dps, float gy_dps, float gz_dps, float dt) {
  float ax = ax_g * 9.81f;
  float ay = ay_g * 9.81f;
  float az = az_g * 9.81f;

  float gx = gx_dps * DEG2RAD;
  float gy = gy_dps * DEG2RAD;

  state->pitch += gx * dt;
  state->roll += gy * dt;

  float acc_mag = sqrtf(ax * ax + ay * ay + az * az);

  if (fabsf(acc_mag - 9.81f) < 1.5f) {
    float pitch_acc = atan2f(ax, sqrtf(ay * ay + az * az));
    float roll_acc = atan2f(ay, sqrtf(ax * ax + az * az));

    const float alpha = 0.90f;
    state->pitch = alpha * state->pitch + (1.0f - alpha) * pitch_acc;
    state->roll = alpha * state->roll + (1.0f - alpha) * roll_acc;
  }
}

float calc_tilt_sut(float angle_x, float angle_y) {
  float tilt = sqrtf(angle_x * angle_x + angle_y * angle_y);

  if (tilt > 180.0f)
    tilt = 180.0f;

  return tilt;
}
