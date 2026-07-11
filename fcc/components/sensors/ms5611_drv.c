#include "ms5611_drv.h"
#include "esp_log.h"

static const char *TAG = "ms5611_drv";
static ms5611_t dev;

esp_err_t ms5611_drv_init(int sda_gpio, int scl_gpio) {
  esp_err_t ret;

  ret = ms5611_init_desc(&dev, MS5611_ADDR_CSB_HIGH, I2C_NUM_0, sda_gpio,
                         scl_gpio);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Descriptor init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = ms5611_init(&dev, MS5611_OSR_4096);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Initialized at 0x76");
  return ESP_OK;
}

esp_err_t ms5611_drv_read(int32_t *pressure_pa, float *temp_c) {
  esp_err_t ret = ms5611_get_sensor_data(&dev, pressure_pa, temp_c);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
  }
  return ret;
}
