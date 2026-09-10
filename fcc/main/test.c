#include <apogee.h>
#include <bmp280_drv.h>
#include <gps_drv.h>
#include <kalman.h>
#include <lora.h>
#include <max3232_drv.h>
#include <mpu6050_drv.h>
#include <ms5611_drv.h>
#include <weighted_average.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include "soc/gpio_num.h"
#include <math.h>

static const char *TAG = "fcc";
static const char *I2C_SCANNER_TAG = "i2c_scanner";

static volatile fcc_mode_t current_mode = FCC_MODE_DUR;

// TODO: Set actual pin numbers
#define PIN_I2C_SDA 2
#define PIN_I2C_SCL 1

#define PIN_GPS_TX 10
#define PIN_GPS_RX 9

#define PIN_LORA_TX 13
#define PIN_LORA_RX 12

#define PIN_LED_APOGEE 3
#define PIN_HGG_APOGEE 8
#define PIN_BUZZER 6

// TODO: FIX OR ILL touch you
#define PIN_RS232_TX 43
#define PIN_RS232_RX 44

#define RS232_BAUD 115200
#define RS232_UART_NUM UART_NUM_0

#define I2C_PORT I2C_NUM_0
#define GPS_UART UART_NUM_1
#define LORA_UART UART_NUM_2
#define GPS_BAUD 38400

// 5Hz = 200ms
#define LOOP_PERIOD_MS 200

volatile uint32_t buzzer_interval_ms = 1000; // 0 = kapalı

// ============================================================
// I2C SCANNER
// ============================================================

static void i2c_scan(void) {
  ESP_LOGI(I2C_SCANNER_TAG, "========================================");
  ESP_LOGI(I2C_SCANNER_TAG, "Starting I2C scanner");
  ESP_LOGI(I2C_SCANNER_TAG, "SDA = GPIO%d", PIN_I2C_SDA);
  ESP_LOGI(I2C_SCANNER_TAG, "SCL = GPIO%d", PIN_I2C_SCL);
  ESP_LOGI(I2C_SCANNER_TAG, "========================================");

  i2c_master_bus_handle_t bus_handle = NULL;

  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_PORT,
      .sda_io_num = PIN_I2C_SDA,
      .scl_io_num = PIN_I2C_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);

  if (ret != ESP_OK) {
    ESP_LOGE(I2C_SCANNER_TAG, "Cannot create I2C bus: %s",
             esp_err_to_name(ret));
    return;
  }

  int found = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {

    ret = i2c_master_probe(bus_handle, addr, 100);

    if (ret == ESP_OK) {
      ESP_LOGI(I2C_SCANNER_TAG, "FOUND: 0x%02X", addr);

      found++;
    }
  }

  ESP_LOGI(I2C_SCANNER_TAG, "========================================");
  ESP_LOGI(I2C_SCANNER_TAG, "I2C scan completed. Found %d nigga(s).", found);
  ESP_LOGI(I2C_SCANNER_TAG, "========================================");

  /*
   * Scanner'ın açtığı native I2C bus'u kapatıyoruz.
   *
   * Daha sonra sensors_init() içerisinde i2cdev_init()
   * kendi I2C bus yönetimini yapacak.
   */
  ret = i2c_del_master_bus(bus_handle);

  if (ret != ESP_OK) {
    ESP_LOGW(I2C_SCANNER_TAG, "I2C bus silinemedi: %s", esp_err_to_name(ret));
  }
}

void buzzer_task(void *arg) {
  while (1) {
    if (buzzer_interval_ms == 0) {
      gpio_set_level(PIN_BUZZER, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    gpio_set_level(PIN_BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(PIN_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(buzzer_interval_ms));
  }
}

// ============================================================
// KALMAN
// ============================================================

// Kalman instances — one per filtered value
static kalman_t k_pressure_ms;
static kalman_t k_pressure_bmp;
static kalman_t k_accel_x;
static kalman_t k_accel_y;
static kalman_t k_accel_z;
static kalman_t k_gyro_x;
static kalman_t k_gyro_y;
static kalman_t k_gyro_z;

// ============================================================
// SENSOR INITIALIZATION
// ============================================================

static void sensors_init(void) {
  ESP_LOGI(TAG, "i2cdev_init basliyor");

  ESP_ERROR_CHECK(i2cdev_init());

  ESP_LOGI(TAG, "i2cdev_init tamamlandi");

  // --------------------------------------------------------
  // MS5611
  // --------------------------------------------------------

  ESP_LOGI(TAG, "ms5611_drv_init basliyor");

  ESP_ERROR_CHECK(ms5611_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));

  ESP_LOGI(TAG, "ms5611_drv_init tamamlandi");

  // --------------------------------------------------------
  // MPU6050
  // --------------------------------------------------------

  ESP_LOGI(TAG, "mpu6050_drv_init basliyor");

  ESP_ERROR_CHECK(mpu6050_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));

  ESP_LOGI(TAG, "mpu6050_drv_init tamamlandi");

  // --------------------------------------------------------
  // BMP280
  // --------------------------------------------------------

  ESP_LOGI(TAG, "bmp280_drv_init basliyor");

  ESP_ERROR_CHECK(bmp280_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));

  ESP_LOGI(TAG, "bmp280_drv_init tamamlandi");

  // --------------------------------------------------------
  // GPS
  // --------------------------------------------------------

  ESP_LOGI(TAG, "gps_drv_init basliyor");

  ESP_ERROR_CHECK(gps_drv_init(GPS_UART, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD));

  ESP_LOGI(TAG, "gps_drv_init tamamlandi");

  // --------------------------------------------------------
  // LoRa
  // --------------------------------------------------------

  ESP_LOGI(TAG, "lora_init basliyor");

  ESP_ERROR_CHECK(lora_init(LORA_UART, PIN_LORA_TX, PIN_LORA_RX, 9600));

  ESP_LOGI(TAG, "lora_init tamamlandi");
}

// ============================================================
// KALMAN INITIALIZATION
// ============================================================

static void kalman_init_all(void) {
  kalman_init(&k_pressure_ms, 0.05f, 1.44f, 101325.0f);

  kalman_init(&k_pressure_bmp, 0.05f, 0.0004f, 101325.0f);

  kalman_init(&k_accel_x, 0.05f, 0.0000769f, 0.0f);

  kalman_init(&k_accel_y, 0.05f, 0.0000769f, 0.0f);

  kalman_init(&k_accel_z, 0.05f, 0.0000769f, 9.81f);

  kalman_init(&k_gyro_x, 0.02f, 0.000125f, 0.0f);

  kalman_init(&k_gyro_y, 0.02f, 0.000125f, 0.0f);

  kalman_init(&k_gyro_z, 0.02f, 0.000125f, 0.0f);
}

// ============================================================
// MAIN FLIGHT LOOP
// ============================================================

void main_quest(void) {
  // --------------------------------------------------------
  // Apogee LED
  // --------------------------------------------------------

  gpio_reset_pin(PIN_LED_APOGEE);
  gpio_set_direction(PIN_LED_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED_APOGEE, 0);

  // --------------------------------------------------------
  // HGG Apogee
  // --------------------------------------------------------

  gpio_reset_pin(PIN_HGG_APOGEE);
  gpio_set_direction(PIN_HGG_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_HGG_APOGEE, 0);

  // --------------------------------------------------------
  // Sensors
  // --------------------------------------------------------

  sensors_init();
  kalman_init_all();

  ESP_LOGI(TAG, "FCC initialized, starting main loop at 5Hz");

  while (current_mode == FCC_MODE_DUR) {

    TickType_t loop_start = xTaskGetTickCount();

    // ----------------------------------------------------
    // Read sensors
    // ----------------------------------------------------

    int32_t ms5611_pressure;
    float ms5611_temp;

    float bmp280_pressure;
    float bmp280_temp;

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;

    gps_data_t gps;

    esp_err_t r_ms = ms5611_drv_read(&ms5611_pressure, &ms5611_temp);

    esp_err_t r_bmp = bmp280_drv_read(&bmp280_pressure, &bmp280_temp);

    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);

    esp_err_t r_gps = gps_drv_read(&gps);

    if (r_ms != ESP_OK || r_bmp != ESP_OK || r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error");
      goto next;
    }

    {
      // int32_t ms5611_pressure =
      //     (int32_t)(
      //         SIT_FAKE_PRESSURE_BASE_PA
      //         + generate_pressure_noise()
      //     );

      float ax = kalman_update(&k_accel_x, accel.x);

      float ay = kalman_update(&k_accel_y, accel.y);

      float az = kalman_update(&k_accel_z, accel.z);

      float gx = kalman_update(&k_gyro_x, gyro.x);

      float gy = kalman_update(&k_gyro_y, gyro.y);

      float gz = kalman_update(&k_gyro_z, gyro.z);

      float pressure_ms = kalman_update(&k_pressure_ms, ms5611_pressure);

      float pressure_bmp = kalman_update(&k_pressure_bmp, bmp280_pressure);

      float tilt = calc_tilt(ax, ay, az);

      // TODO: tune weights based on sensor accuracy tests

      float pressure = weighted_average(pressure_ms, 0.5f, pressure_bmp, 0.5f);

      float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

      float lat = (r_gps == ESP_OK) ? gps.latitude : 0.0f;

      float lon = (r_gps == ESP_OK) ? gps.longitude : 0.0f;

      ESP_LOGI(TAG,
               "alt=%.2f press=%.2f "
               "ax=%.3f ay=%.3f az=%.3f "
               "gx=%.3f gy=%.3f gz=%.3f "
               "lat=%.6f lon=%.6f tilt=%.2f",
               altitude, pressure, ax, ay, az, gx, gy, gz, lat, lon, tilt);

      if (flight_state_update(altitude, tilt)) {

        gpio_set_level(PIN_LED_APOGEE, 1);

        ESP_LOGI(TAG, "APOGEE DETECTED");

        // Apogee part
      }

      lora_packet_data_t pkt = {
          .altitude = altitude,
          .pressure = pressure,
          .accel_x = ax,
          .accel_y = ay,
          .accel_z = az,
          .angle_x = gx,
          .angle_y = gy,
          .angle_z = gz,
          .gps_lat = lat,
          .gps_lon = lon,
      };

      lora_send(&pkt);
      lora_dump_raw();
    }

  next:
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void app_main(void) {
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    i2c_scan();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
  }
}
