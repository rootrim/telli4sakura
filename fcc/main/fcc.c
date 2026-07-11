#include <apogee.h>
#include <bmp390_drv.h>
#include <gps_drv.h>
#include <kalman.h>
#include <lora.h>
#include <mpu6050_drv.h>
#include <ms5611_drv.h>
#include <weighted_average.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <math.h>

static const char *TAG = "fcc";

// TODO: Set actual pin numbers
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 6
#define PIN_GPS_TX 17
#define PIN_GPS_RX 18
#define PIN_LORA_TX 7
#define PIN_LORA_RX 8
#define PIN_LED_APOGEE 10

#define I2C_PORT I2C_NUM_0
#define GPS_UART UART_NUM_1
#define LORA_UART UART_NUM_2
#define GPS_BAUD 9600

// 5Hz = 200ms
#define LOOP_PERIOD_MS 200

// Kalman instances — one per filtered value
static kalman_t k_pressure;
static kalman_t k_accel_x;
static kalman_t k_accel_y;
static kalman_t k_accel_z;
static kalman_t k_gyro_x;
static kalman_t k_gyro_y;
static kalman_t k_gyro_z;

static void sensors_init(void) {
  ESP_ERROR_CHECK(i2cdev_init());
  ESP_ERROR_CHECK(ms5611_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  ESP_ERROR_CHECK(mpu6050_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  i2c_master_bus_handle_t i2c_bus;
  ESP_ERROR_CHECK(i2cdev_get_shared_handle(I2C_PORT, (void **)&i2c_bus));
  ESP_ERROR_CHECK(bmp390_drv_init(i2c_bus));
  ESP_ERROR_CHECK(gps_drv_init(GPS_UART, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD));
  ESP_ERROR_CHECK(lora_init(LORA_UART, PIN_LORA_TX, PIN_LORA_RX, 9600));
}

static void kalman_init_all(void) {
  // q: process noise, r: measurement noise
  // TODO: tune these values after testing
  kalman_init(&k_pressure, 0.1f, 1.0f, 0.0f);
  kalman_init(&k_accel_x, 0.1f, 0.5f, 0.0f);
  kalman_init(&k_accel_y, 0.1f, 0.5f, 0.0f);
  kalman_init(&k_accel_z, 0.1f, 0.5f, 9.81f);
  kalman_init(&k_gyro_x, 0.1f, 0.5f, 0.0f);
  kalman_init(&k_gyro_y, 0.1f, 0.5f, 0.0f);
  kalman_init(&k_gyro_z, 0.1f, 0.5f, 0.0f);
}

void app_main(void) {

  ////////////////////////
  gpio_reset_pin(PIN_LED_APOGEE);
  gpio_set_direction(PIN_LED_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED_APOGEE, 0);
  ////////////////////////

  sensors_init();
  kalman_init_all();

  ESP_LOGI(TAG, "FCC initialized, starting main loop at 5Hz");

  while (1) {
    TickType_t loop_start = xTaskGetTickCount();

    // --- Read sensors ---
    int32_t ms5611_pressure;
    float ms5611_temp;
    float bmp390_pressure, bmp390_temp;
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;
    gps_data_t gps;

    esp_err_t r_ms = ms5611_drv_read(&ms5611_pressure, &ms5611_temp);
    esp_err_t r_bmp = bmp390_drv_read(&bmp390_pressure, &bmp390_temp);
    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);
    esp_err_t r_gps = gps_drv_read(&gps);

    if (r_ms != ESP_OK || r_bmp != ESP_OK || r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error");
      goto next;
    }

    {
      // --- Weighted average: pressure (50/50 for now) ---
      // TODO: tune weights based on sensor accuracy tests
      float raw_pressure =
          weighted_average((float)ms5611_pressure, 0.5f, bmp390_pressure, 0.5f);

      // --- Kalman filter ---
      float pressure = kalman_update(&k_pressure, raw_pressure);
      float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

      float ax = kalman_update(&k_accel_x, accel.x);
      float ay = kalman_update(&k_accel_y, accel.y);
      float az = kalman_update(&k_accel_z, accel.z);
      float tilt = calc_tilt(&accel);
      float gx = kalman_update(&k_gyro_x, gyro.x);
      float gy = kalman_update(&k_gyro_y, gyro.y);
      float gz = kalman_update(&k_gyro_z, gyro.z);

      float lat = (r_gps == ESP_OK) ? gps.latitude : 0.0f;
      float lon = (r_gps == ESP_OK) ? gps.longitude : 0.0f;

      ESP_LOGI(TAG,
               "alt=%.2f press=%.2f ax=%.3f ay=%.3f az=%.3f "
               "gx=%.3f gy=%.3f gz=%.3f lat=%.6f lon=%.6f tilt=%.2f",
               altitude, pressure, ax, ay, az, gx, gy, gz, lat, lon, tilt);

      if (flight_state_update(altitude, tilt)) {
        gpio_set_level(PIN_LED_APOGEE, 1);
        ESP_LOGI(TAG, "APOGEE DETECTED");
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
    }

  next:
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}
