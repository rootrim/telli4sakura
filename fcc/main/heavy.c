#include <apogee.h>
#include <bmp390_drv.h>
#include <gps_drv.h>
#include <kalman.h>
#include <lora.h>
#include <max3232_drv.h>
#include <mpu6050_drv.h>
#include <ms5611_drv.h>
#include <weighted_average.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <math.h>

static const char *TAG = "KGD";

// TODO: Set actual pin numbers
#define PIN_GPS_TX 11
#define PIN_GPS_RX 10
#define PIN_LORA_TX 8
#define PIN_LORA_RX 9
#define PIN_BUZZER 1

#define GPS_UART UART_NUM_1
#define LORA_UART UART_NUM_2
#define GPS_BAUD 38400

// 5Hz = 200ms
#define LOOP_PERIOD_MS 200

volatile uint32_t buzzer_interval_ms = 1000; // 0 = kapalı

void buzzer_task(void *arg) {
  while (1) {
    if (buzzer_interval_ms == 0) {
      gpio_set_level(PIN_BUZZER, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    gpio_set_level(PIN_BUZZER, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Bip süresi

    gpio_set_level(PIN_BUZZER, 0);
    vTaskDelay(pdMS_TO_TICKS(buzzer_interval_ms));
  }
}

static void sensors_init(void) {
  ESP_ERROR_CHECK(lora_init(LORA_UART, PIN_LORA_TX, PIN_LORA_RX, 9600));
}

void main_quest(void) {
  gpio_set_level(PIN_BUZZER, 1);
  sensors_init();

  ESP_LOGI(TAG, "Initialized, starting main loop at 5Hz");

  while (1) {
    TickType_t loop_start = xTaskGetTickCount();

    gps_data_t gps;

    esp_err_t r_gps = gps_drv_read(&gps);

    float lat = (r_gps == ESP_OK) ? gps.latitude : 0.0f;
    float lon = (r_gps == ESP_OK) ? gps.longitude : 0.0f;

    ESP_LOGI(TAG, "lon=%.6f lat=%.2f", lon, lat);

    lora_packet_data_t pkt = {
        .altitude = 0,
        .pressure = 0,
        .accel_x = 0,
        .accel_y = 0,
        .accel_z = 0,
        .angle_x = 0,
        .angle_y = 0,
        .angle_z = 0,
        .gps_lat = lat,
        .gps_lon = lon,
    };
    lora_send(&pkt);
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void app_main(void) { main_quest(); }
