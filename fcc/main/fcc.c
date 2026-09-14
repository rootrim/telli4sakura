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
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include "soc/gpio_num.h"
#include <math.h>

static const char *TAG = "fcc";
static const char *I2C_SCANNER_TAG = "i2c_scanner";

static volatile fcc_mode_t current_mode = FCC_MODE_DUR;

#define PIN_I2C_SDA 2
#define PIN_I2C_SCL 1

#define PIN_GPS_TX 9
#define PIN_GPS_RX 10

#define PIN_LORA_TX 12
#define PIN_LORA_RX 13
#define PIN_LORA_M1 4

#define PIN_LED_APOGEE 3
#define PIN_HGG_APOGEE 8
#define PIN_BUZZER 6

#define PIN_RS232_TX 43
#define PIN_RS232_RX 44

#define RS232_BAUD 115200
#define RS232_UART_NUM UART_NUM_0

#define I2C_PORT I2C_NUM_0
#define GPS_UART UART_NUM_1
#define LORA_UART UART_NUM_2
#define GPS_BAUD 38400

#define LOOP_PERIOD_MS 10

#define PRIO_MAIN_LOOP 10
#define PRIO_CMD_CHECK 6
#define PRIO_LORA 5
#define PRIO_GPS 5
#define PRIO_BUZZER 3

volatile uint32_t buzzer_interval_ms = 1000; // 0 = kapali

float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float altitude;
float pressure;
TiltState tilt = {0};
float lat;
float lon;

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
  ESP_LOGI(I2C_SCANNER_TAG, "I2C scan completed. Found %d device.", found);
  ESP_LOGI(I2C_SCANNER_TAG, "========================================");

  ret = i2c_del_master_bus(bus_handle);

  if (ret != ESP_OK) {
    ESP_LOGW(I2C_SCANNER_TAG, "I2C bus silinemedi: %s", esp_err_to_name(ret));
  }
}

void buzzer_task(void *pvParameters) {
  while (1) { // NOTE: change to heartbeat
    uint32_t interval = buzzer_interval_ms;

    if (interval == 0) {
      buzzer_off();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    buzzer_on(2000); // 2 kHz
    vTaskDelay(pdMS_TO_TICKS(interval));

    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void gps_task(void *pvParameters) {
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    gps_data_t gps;

    esp_err_t r_gps = gps_drv_read(&gps);

    if (r_gps == ESP_OK) {
      lat = gps.latitude;
      lon = gps.longitude;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
  }
}

static kalman_t k_pressure_ms;
static kalman_t k_pressure_bmp;

static kalman_t k_accel_x;
static kalman_t k_accel_y;
static kalman_t k_accel_z;

static kalman_t k_gyro_x;
static kalman_t k_gyro_y;
static kalman_t k_gyro_z;

static bool pressure_kalman_initialized = false;

static float ground_pressure = 0.0f;
static bool ground_pressure_set = false;

static void sensors_init(void) {
  ESP_LOGI(TAG, "i2cdev_init basliyor");
  ESP_ERROR_CHECK(i2cdev_init());
  ESP_LOGI(TAG, "i2cdev_init tamamlandi");

  ESP_LOGI(TAG, "mpu6050_drv_init basliyor");
  ESP_ERROR_CHECK(mpu6050_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  ESP_LOGI(TAG, "mpu6050_drv_init tamamlandi");

  ESP_LOGI(TAG, "bmp280_drv_init basliyor");
  ESP_ERROR_CHECK(bmp280_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  ESP_LOGI(TAG, "bmp280_drv_init tamamlandi");

  ESP_LOGI(TAG, "ms5611_drv_init basliyor");
  ESP_ERROR_CHECK(ms5611_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  ESP_LOGI(TAG, "ms5611_drv_init tamamlandi");

  ESP_LOGI(TAG, "gps_drv_init basliyor");
  ESP_ERROR_CHECK(gps_drv_init(GPS_UART, PIN_GPS_TX, PIN_GPS_RX, GPS_BAUD));
  ESP_LOGI(TAG, "gps_drv_init tamamlandi");

  ESP_LOGI(TAG, "lora_init basliyor");
  ESP_ERROR_CHECK(
      lora_init(LORA_UART, PIN_LORA_TX, PIN_LORA_RX, PIN_LORA_M1, 9600));
  ESP_LOGI(TAG, "lora_init tamamlandi");
}

static void kalman_init_all(void) {
  kalman_init(&k_accel_x, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_y, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_z, 0.05f, 0.0000769f, 9.81f);
  kalman_init(&k_gyro_x, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_y, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_z, 0.02f, 0.000125f, 0.0f);
}

void main_quest(void) {
  gpio_reset_pin(PIN_LED_APOGEE);
  gpio_set_direction(PIN_LED_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED_APOGEE, 0);

  gpio_reset_pin(PIN_HGG_APOGEE);
  gpio_set_direction(PIN_HGG_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_HGG_APOGEE, 0);

  gpio_reset_pin(PIN_LORA_M1);
  gpio_set_direction(PIN_LORA_M1, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LORA_M1, 0);

  sensors_init();
  kalman_init_all();

  pressure_kalman_initialized = false;
  ground_pressure_set = false;

  ESP_LOGI(TAG, "FCC initialized, starting main loop at 20Hz");

  uint64_t last_tilt_time = esp_timer_get_time();

  while (current_mode == FCC_MODE_DUR) {
    uint8_t test_message[] = {0xDE, 0xAD, 0xBE, 0xEF};

    uart_write_bytes(RS232_UART_NUM, test_message, sizeof(test_message));

    TickType_t loop_start = xTaskGetTickCount();

    int32_t ms5611_pressure;
    float ms5611_temp;

    float bmp280_pressure;
    float bmp280_temp;

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;

    esp_err_t r_ms = ms5611_drv_read(&ms5611_pressure, &ms5611_temp);
    esp_err_t r_bmp = bmp280_drv_read(&bmp280_pressure, &bmp280_temp);
    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);

    if (r_ms != ESP_OK || r_bmp != ESP_OK || r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error");
      goto next;
    }

    if (!pressure_kalman_initialized) {
      kalman_init(&k_pressure_ms, 0.05f, 100.0f, (float)ms5611_pressure);
      kalman_init(&k_pressure_bmp, 0.05f, 100.0f, bmp280_pressure);
      pressure_kalman_initialized = true;
      ESP_LOGI(TAG,
               "Pressure Kalman initialized: "
               "MS=%.2f | BMP=%.2f",
               (float)ms5611_pressure, bmp280_pressure);
    }

    ax = kalman_update(&k_accel_x, accel.x);
    ay = kalman_update(&k_accel_y, accel.y);
    az = kalman_update(&k_accel_z, accel.z);
    gx = kalman_update(&k_gyro_x, gyro.x);
    gy = kalman_update(&k_gyro_y, gyro.y);
    gz = kalman_update(&k_gyro_z, gyro.z);

    float pressure_ms = kalman_update(&k_pressure_ms, (float)ms5611_pressure);
    float pressure_bmp = kalman_update(&k_pressure_bmp, bmp280_pressure);

    pressure = weighted_average(pressure_ms, 0.4f, pressure_bmp, 0.6f);

    if (!ground_pressure_set) {
      ground_pressure = pressure;
      ground_pressure_set = true;
      ESP_LOGI(TAG, "Ground pressure reference set: %.2f", ground_pressure);
    }

    altitude = 44330.0f * (1.0f - powf(pressure / ground_pressure, 0.1903f));

    uint64_t now = esp_timer_get_time();
    float dt = (now - last_tilt_time) / 1e6f;
    last_tilt_time = now;
    tilt_update(&tilt, ax, ay, az, gx, gy, gz, dt);
    float tilt_mag = sqrtf(tilt.pitch * tilt.pitch + tilt.roll * tilt.roll);
    float tilt_deg = tilt_mag * 180.0f / (float)M_PI;
    ESP_LOGI(TAG,
             "alt=%.2f press=%.2f "
             "ax=%.3f ay=%.3f az=%.3f "
             "gx=%.3f gy=%.3f gz=%.3f "
             "lat=%.6f lon=%.6f tilt=%.2f",
             altitude, pressure, ax, ay, az, gx, gy, gz, lat, lon, tilt_deg);

    if (flight_state_update(altitude, tilt_deg)) {
      gpio_set_level(PIN_LED_APOGEE, 1);
      gpio_set_level(PIN_HGG_APOGEE, 1);
      ESP_LOGI(TAG, "APOGEE DETECTED");
      // Apogee part
    }

  next:
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }
}

void lora_send_task(void *pvParameters) {
  TickType_t last_wake = xTaskGetTickCount();

  while (1) {
    lora_packet_data_t pkt = {
        .altitude = altitude,
        .pressure = pressure,
        .accel_x = ax,
        .accel_y = ay,
        .accel_z = az,
        .gyro_x = gx,
        .gyro_y = gy,
        .gyro_z = gz,
        .gps_lat = lat,
        .gps_lon = lon,
    };

    ESP_LOGI(TAG, "Lora packet send");

    lora_send(&pkt);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
  }
}

void command_check_task(void *pvParameters) {
  while (1) {
    check_mode_command(&current_mode);
    mode_apply_pending(&current_mode);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void app_main(void) {
  vTaskPrioritySet(NULL, PRIO_MAIN_LOOP);

  i2c_scan();

  ESP_ERROR_CHECK(
      max3232_drv_init(RS232_UART_NUM, PIN_RS232_TX, PIN_RS232_RX, RS232_BAUD));

  xTaskCreate(command_check_task, "cmd_task", 4096, NULL, PRIO_CMD_CHECK, NULL);
  xTaskCreate(lora_send_task, "lora_task", 4096, NULL, PRIO_LORA, NULL);
  xTaskCreate(gps_task, "gps_task", 4096, NULL, PRIO_GPS, NULL);

  buzzer_init();
  buzzer_on(2000);
  vTaskDelay(pdMS_TO_TICKS(1000));
  buzzer_off();

  for (;;) {
    switch (current_mode) {
    case FCC_MODE_SIT:
      run_sit(&current_mode);
      break;
    case FCC_MODE_SUT:
      ESP_LOGI(TAG, "Switching to SUT");
      run_sut(&current_mode);
      ESP_LOGI(TAG, "Returned from SUT");
      break;
    case FCC_MODE_DUR:
      main_quest();
      break;
    default:
      // safe default: flight mode (DUR)
      main_quest();
      break;
    }
  }
}
