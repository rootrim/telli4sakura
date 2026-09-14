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
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include <math.h>

static const char *TAG = "KGD";

// TODO: Set actual pin numbers
#define PIN_GPS_TX 4
#define PIN_GPS_RX 5
#define PIN_LORA_TX 7
#define PIN_LORA_RX 8
#define PIN_LORA_M1 10
#define PIN_BUZZER 1
#define PIN_LED 11

#define GPS_UART UART_NUM_1
#define LORA_UART UART_NUM_2
#define GPS_BAUD 38400

// 5Hz = 200ms
#define LOOP_PERIOD_MS 200

volatile uint32_t buzzer_interval_ms = 1000; // 0 = kapalı

void buzzer_init(void) {
  ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .freq_hz = 2000,
      .clk_cfg = LEDC_AUTO_CLK,
  };

  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {
      .gpio_num = PIN_BUZZER,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .timer_sel = LEDC_TIMER_0,
      .duty = 512,
      .hpoint = 0,
  };

  ledc_channel_config(&channel);
}

void buzzer_on(uint32_t freq) {
  ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_off(void) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void buzzer_task(void *pvParameters) {
  while (1) { // NOTE: change to heartbeat
    uint32_t interval = buzzer_interval_ms;

    if (interval == 0) {
      buzzer_off();
      gpio_set_level(PIN_LED, 0);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    buzzer_on(2000); // 2 kHz
    gpio_set_level(PIN_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(interval));

    buzzer_off();
    gpio_set_level(PIN_LED, 0);
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

static void sensors_init(void) {
  ESP_ERROR_CHECK(
      lora_init(LORA_UART, PIN_LORA_TX, PIN_LORA_RX, PIN_LORA_M1, 9600));
  ESP_ERROR_CHECK(gps_drv_init(GPS_UART, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD));
}

void main_quest(void) {
  gpio_reset_pin(PIN_LORA_M1);
  gpio_set_direction(PIN_LORA_M1, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LORA_M1, 0);

  gpio_reset_pin(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED, 0);

  buzzer_init();
  xTaskCreate(buzzer_task, "buzzer_task", 4096, NULL, 5, NULL);
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
        .gyro_x = 0,
        .gyro_y = 0,
        .gyro_z = 0,
        .gps_lat = lat,
        .gps_lon = lon,
    };
    lora_send(&pkt);
    ESP_LOGI(TAG, "Lora package send");
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void app_main(void) { main_quest(); }
