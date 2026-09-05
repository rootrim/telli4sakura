#include "bmp280_drv.h"
#include "bmp280.h"
#include "esp_log.h"

static const char *TAG = "bmp280_drv";
static bmp280_t dev;

esp_err_t bmp280_drv_init(int sda_gpio, int scl_gpio) {
  esp_err_t ret;
  bmp280_params_t params;

  bmp280_init_default_params(&params);

  ret = bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_1, I2C_NUM_0, sda_gpio,
                         scl_gpio);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Descriptor init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = bmp280_init(&dev, &params);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Initialized at 0x77");
  return ESP_OK;
}

esp_err_t bmp280_drv_read(float *pressure_pa, float *temp_c) {
  float humidity;
  esp_err_t ret = bmp280_read_float(&dev, temp_c, pressure_pa, &humidity);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
  }
  return ret;
}
