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

  ESP_LOGI(TAG, "Initialized at 0x68");
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

float calc_tilt(const mpu6050_acceleration_t *accel) {
  float magnitude =
      sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
  return acosf(accel->z / magnitude) * 180.0f / M_PI;
}
