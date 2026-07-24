#include "driver/uart.h"
#include "esp_log.h"
#include "math.h"
#include <apogee.h>
#include <bits/pthreadtypes.h>
#include <bmp390_drv.h>
#include <gps_drv.h>
#include <kalman.h>
#include <limits.h>
#include <math.h>
#include <max3232_drv.h>
#include <mpu6050_drv.h>
#include <ms5611_drv.h>
#include <stdint.h>
#include <string.h>
#include <weighted_average.h>

static const char *TAG = "MAX3232";
static fcc_mode_t s_current_mode = FCC_MODE_DUR;

static uint8_t calc_checksum(const uint8_t *buf, int len) {
  uint8_t cs = 0;
  for (int i = 0; i < len; i++)
    cs += buf[i];
  return cs;
}

/*
 * RS232 hattinda 5 byte'lik bir mod komutu bekleyip bekleyip
 * bekleyip beklenmedigini kontrol eder. Non-blocking: veri yoksa
 * hemen doner, mevcut modu degistirmez.
 *
 * Donus: yeni mod algilandiysa true, yoksa false.
 */
static bool check_mode_command(fcc_mode_t *out_mode) {
  uint8_t buf[CMD_PACKET_SIZE];

  // timeout=0: veri yoksa hemen don, ana donguyu bloklamasin
  int len = uart_read_bytes(RS232_UART, buf, CMD_PACKET_SIZE, 0);
  if (len != CMD_PACKET_SIZE)
    return false;

  if (buf[0] != CMD_HEADER) {
    ESP_LOGW(TAG, "Invalid header: 0x%02X", buf[0]);
    return false;
  }

  // TODO: Check if checksums are true or not
  uint8_t expected_cs = calc_checksum(buf, 2);
  if (buf[2] != expected_cs) {
    ESP_LOGW(TAG, "Checksum mismatch: got 0x%02X expected 0x%02X", buf[4],
             expected_cs);
    return false;
  }

  switch (buf[1]) {
  case CMD_DUR_COMMAND:
    *out_mode = FCC_MODE_DUR;
    return true;
  case CMD_SIT_COMMAND:
    *out_mode = FCC_MODE_SIT;
    return true;
  case CMD_SUT_COMMAND:
    *out_mode = FCC_MODE_SUT;
    return true;
  default:
    ESP_LOGW(TAG, "Unknown mode byte: 0x%02X", buf[1]);
    return false;
  }
}

/*
 * SIT: 36 byte'lik test paketini gonder.
 * Icerigi kendi test verine gore doldur (su an sifirlarla dolduruyor).
 */

// TODO: Set actual pin numbers
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 6

#define I2C_PORT I2C_NUM_0

// 10Hz = 100ms
#define LOOP_PERIOD_MS 100

// Kalman instances — one per filtered value
static kalman_t k_pressure_ms;
static kalman_t k_pressure_bmp;
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
}

static void kalman_init_all(void) {
  // TODO: tune parameters for 10Hz
  kalman_init(&k_pressure_ms, 0.05f, 1.44f, 0.0f);
  kalman_init(&k_pressure_bmp, 0.05f, 0.0004f, 0.0f);
  kalman_init(&k_accel_x, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_y, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_z, 0.05f, 0.0000769f, 9.81f);
  kalman_init(&k_gyro_x, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_y, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_z, 0.02f, 0.000125f, 0.0f);
}

static void pack_float(uint8_t *buf, float val) {
  memcpy(buf, &val, sizeof(float));
}

static void run_sit(void) {
  uint8_t packet[SIT_PACKET_SIZE] = {0};

  sensors_init();
  kalman_init_all();

  ESP_LOGI(TAG, "SIT Sensors initialized");

  while (s_current_mode == FCC_MODE_SIT) {
    TickType_t loop_start = xTaskGetTickCount();

    int32_t ms5611_pressure;
    float ms5611_temp;
    float bmp390_pressure, bmp390_temp;
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;

    esp_err_t r_ms = ms5611_drv_read(&ms5611_pressure, &ms5611_temp);
    esp_err_t r_bmp = bmp390_drv_read(&bmp390_pressure, &bmp390_temp);
    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);

    if (r_ms != ESP_OK || r_bmp != ESP_OK || r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error");
      goto next;
    }

    {
      // --- Kalman filter ---
      float ax = kalman_update(&k_accel_x, accel.x);
      float ay = kalman_update(&k_accel_y, accel.y);
      float az = kalman_update(&k_accel_z, accel.z);
      float gx = kalman_update(&k_gyro_x, gyro.x);
      float gy = kalman_update(&k_gyro_y, gyro.y);
      float gz = kalman_update(&k_gyro_z, gyro.z);
      float pressure_ms = kalman_update(&k_pressure_ms, ms5611_pressure);
      float pressure_bmp = kalman_update(&k_pressure_bmp, bmp390_pressure);

      float pressure = weighted_average(pressure_ms, 0.5f, pressure_bmp, 0.5f);
      float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

      packet[0] = SIT_HEADER;
      pack_float(&packet[1], altitude);
      pack_float(&packet[5], pressure);
      pack_float(&packet[9], ax);
      pack_float(&packet[13], ay);
      pack_float(&packet[17], az);
      pack_float(&packet[21], gx);
      pack_float(&packet[25], gy);
      pack_float(&packet[29], gz);
      packet[33] = calc_checksum(packet, 33);
      packet[34] = SIT_FOOTER1;
      packet[35] = SIT_FOOTER2;
    }
    int sent =
        uart_write_bytes(RS232_UART, (const char *)packet, SIT_PACKET_SIZE);
    if (sent != SIT_PACKET_SIZE) {
      ESP_LOGE(TAG, "SIT write incomplete: %d/%d", sent, SIT_PACKET_SIZE);
    }
  next:
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    check_mode_command(&s_current_mode);
  }
}

/*
 * SUT: 36 byte oku, isle, 6 byte cevap yaz.
 * timeout ile bloklayici okuma yapiyor; veri gelmezse belli bir sure sonra
 * vazgecer.
 */

#define SUT_PACKET_SIZE 36

static kalman_t k_pressure_sut;
static kalman_t k_altitude_sut;
static kalman_t k_accel_x_sut;
static kalman_t k_accel_y_sut;
static kalman_t k_accel_z_sut;
static kalman_t k_gyro_x_sut;
static kalman_t k_gyro_y_sut;
static kalman_t k_gyro_z_sut;

static void kalman_init_all_sut(void) {
  // TODO: Tune these parameters
  kalman_init(&k_pressure_sut, 0.05f, 1.44f, 0.0f);
  kalman_init(&k_altitude_sut, 0.05f, 0.0004f, 0.0f);
  kalman_init(&k_accel_x_sut, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_y_sut, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_z_sut, 0.05f, 0.0000769f, 9.81f);
  kalman_init(&k_gyro_x_sut, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_y_sut, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_z_sut, 0.02f, 0.000125f, 0.0f);
}

#define GRAVITY 9.81f
#define LIFTOFF_ACCEL_THRESHOLD (2.5f * GRAVITY)
#define LIFTOFF_CONFIRM_SAMPLES 5

static void check_liftoff(uint16_t *state, double magnitude) {
  static int liftoff_counter = 0;

  if (*state & 0b1)
    return;

  if (magnitude >= LIFTOFF_ACCEL_THRESHOLD)
    liftoff_counter++;
  else
    liftoff_counter = 0;

  if (liftoff_counter >= LIFTOFF_CONFIRM_SAMPLES)
    *state |= 0b1;
}

#define BURNOUT_WINDOW_SIZE 10
#define BURNOUT_ACCEL_THRESHOLD 0.0

static void check_burnout(uint16_t *state, double magnitude) {
  static double magnitude_window[BURNOUT_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;

  if (!(*state & 0b1))
    return;
  if (*state & 0b10)
    return;

  magnitude_window[window_index++] = magnitude;
  if (!first_window_iteration && (window_index > BURNOUT_WINDOW_SIZE - 1)) {
    first_window_iteration = true;
  }
  window_index %= BURNOUT_WINDOW_SIZE;

  if (!first_window_iteration)
    return;

  double sum = 0;
  for (int i = 0; i < BURNOUT_WINDOW_SIZE; i++) {
    sum += magnitude_window[i];
  }
  double average = sum / BURNOUT_WINDOW_SIZE;

  if (average <= BURNOUT_ACCEL_THRESHOLD) {
    *state |= 0b10;
  }
}

static uint16_t state = 0;

static void check_state(double magnitude) {
  check_liftoff(&state, magnitude);
  check_burnout(&state, magnitude);
}

static void run_sut(void) {
  sut_data sut_data = {0};
  int len =
      uart_read_bytes(RS232_UART, &sut_data, SUT_READ_SIZE, pdMS_TO_TICKS(200));

  if (len != SUT_READ_SIZE) {
    if (len > 0) {
      ESP_LOGW(TAG, "SUT partial read: %d/%d", len, SUT_READ_SIZE);
    }
    return;
  }

  while (s_current_mode == FCC_MODE_SUT) {
    TickType_t loop_start = xTaskGetTickCount();

    // TODO: rx icerigini isleyip gercek cevabi hazirla
    uint8_t tx[SUT_WRITE_SIZE] = {0};
    uint16_t command_bytes;

    // TODO: Read virtual data

    {
      float ax = kalman_update(&k_accel_x_sut, sut_data.accel_x);
      float ay = kalman_update(&k_accel_y_sut, sut_data.accel_y);
      float az = kalman_update(&k_accel_z_sut, sut_data.accel_z);
      float gx = kalman_update(&k_gyro_x_sut, sut_data.angle_x);
      float gy = kalman_update(&k_gyro_y_sut, sut_data.angle_y);
      float gz = kalman_update(&k_gyro_z_sut, sut_data.angle_z);
      float pr = kalman_update(&k_pressure_sut, sut_data.pressure);
      float al = kalman_update(&k_altitude_sut, sut_data.altitude);

      double magnitude = sqrt(ax * ax + ay * ay + az * az);
      check_state(magnitude);
    }

    int sent = uart_write_bytes(RS232_UART, (const char *)tx, SUT_WRITE_SIZE);
    if (sent != SUT_WRITE_SIZE) {
      ESP_LOGE(TAG, "SUT write incomplete: %d/%d", sent, SUT_WRITE_SIZE);
    }

    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    check_mode_command(&s_current_mode);
  }
}

void fcc_mode_step(void) {
  // fcc_mode_t new_mode;
  // if (check_mode_command(&new_mode)) {
  //   if (new_mode != s_current_mode) {
  //     ESP_LOGI(TAG, "Mode changed: %d -> %d", s_current_mode, new_mode);
  //     s_current_mode = new_mode;
  //   }
  // }

  switch (s_current_mode) {
  case FCC_MODE_SIT:
    run_sit();
    break;
  case FCC_MODE_SUT:
    run_sut();
    break;
  case FCC_MODE_DUR:
  default:
    // Normal FCC dongusu (sensor okuma + kalman + LoRa) burada
    // zaten var olan app_main dongunde calisiyor olmali;
    // bu fonksiyon sadece diger iki moda mudahale ediyor.
    break;
  }
}
