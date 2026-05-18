#include "bmp390_drv.h"
#include "esp_log.h"

static const char *TAG = "bmp390_drv";
static bmp390_handle_t handle = NULL;

esp_err_t bmp390_drv_init(i2c_master_bus_handle_t bus_handle) {
  bmp390_config_t config = I2C_BMP390_CONFIG_DEFAULT;
  config.i2c_address = I2C_BMP390_DEV_ADDR_HI; // 0x77

  esp_err_t ret = bmp390_init(bus_handle, &config, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ESP_LOGI(TAG, "Initialized at 0x77");
  return ESP_OK;
}

esp_err_t bmp390_drv_read(float *pressure_pa, float *temp_c) {
  esp_err_t ret = bmp390_get_measurements(handle, temp_c, pressure_pa);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
  }
  return ret;
}
