#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "math.h"
#include <apogee.h>
#include <bmp280_drv.h>
#include <gps_drv.h>
#include <kalman.h>
#include <limits.h>
#include <max3232_drv.h>
#include <mpu6050_drv.h>
#include <ms5611_drv.h>
#include <stdint.h>
#include <string.h>
#include <weighted_average.h>

static const char *TAG = "MAX3232";

static volatile fcc_mode_t pending_mode;
static bool mode_pending = false;
static TickType_t mode_pending_since;

/* ============================================================
 * CHECKSUM
 * ============================================================ */

static uint8_t calc_checksum(const uint8_t *buf, int len) {
  uint8_t cs = 0;

  for (int i = 0; i < len; i++)
    cs += buf[i];

  return cs;
}

/* ============================================================
 * BIG ENDIAN HELPERS
 * ============================================================ */

static inline void write_be_float(uint8_t *p, float f) {
  union {
    float f;
    uint32_t u;
  } v;

  v.f = f;

  p[0] = (uint8_t)((v.u >> 24) & 0xFF);
  p[1] = (uint8_t)((v.u >> 16) & 0xFF);
  p[2] = (uint8_t)((v.u >> 8) & 0xFF);
  p[3] = (uint8_t)(v.u & 0xFF);
}

static inline void write_be_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

static inline float read_be_float(const uint8_t *p) {
  union {
    uint32_t u;
    float f;
  } v;

  v.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
        ((uint32_t)p[2] << 8) | ((uint32_t)p[3]);

  return v.f;
}

/* ============================================================
 * COMMAND PACKET
 * ============================================================ */

static bool apply_command_packet(const uint8_t *buf) {
  if (buf[0] != CMD_HEADER) {
    ESP_LOGW(TAG, "Invalid header: 0x%02X", buf[0]);
    return false;
  }

  uint8_t expected_cs = calc_checksum(buf, 2);

  if (buf[2] != expected_cs) {
    ESP_LOGW(TAG, "Checksum mismatch: got 0x%02X expected 0x%02X", buf[2],
             expected_cs);
    return false;
  }

  if (buf[3] != 0x0D || buf[4] != 0x0A) {
    ESP_LOGW(TAG, "Invalid footer: 0x%02X 0x%02X", buf[3], buf[4]);
    return false;
  }

  fcc_mode_t new_mode;

  switch (buf[1]) {
  case CMD_DUR_COMMAND:
    new_mode = FCC_MODE_DUR;
    break;

  case CMD_SIT_COMMAND:
    new_mode = FCC_MODE_SIT;
    break;

  case CMD_SUT_COMMAND:
    new_mode = FCC_MODE_SUT;
    break;

  default:
    ESP_LOGW(TAG, "Unknown mode byte: 0x%02X", buf[1]);
    return false;
  }

  pending_mode = new_mode;
  mode_pending = true;
  mode_pending_since = xTaskGetTickCount();

  ESP_LOGI(TAG, "Command accepted, switching in 1 second");

  return true;
}

bool check_mode_command(fcc_mode_t volatile *out_mode) {
  if (out_mode && *out_mode == FCC_MODE_SUT)
    return false;

  uint8_t buf[CMD_PACKET_SIZE];

  int len = uart_read_bytes(RS232_UART_NUM, buf, CMD_PACKET_SIZE, 0);

  if (len != CMD_PACKET_SIZE)
    return false;

  return apply_command_packet(buf);
}

/* ============================================================
 * HARDWARE
 * ============================================================ */

#define PIN_I2C_SDA 2
#define PIN_I2C_SCL 1

#define PIN_LED_APOGEE 3
#define PIN_HGG_APOGEE 8
#define PIN_BUZZER 6

#define I2C_PORT I2C_NUM_0

#define LOOP_PERIOD_MS 100

/* ============================================================
 * SIT KALMAN
 * ============================================================ */

static kalman_t k_pressure_ms;
static kalman_t k_pressure_bmp;

static kalman_t k_accel_x;
static kalman_t k_accel_y;
static kalman_t k_accel_z;

static kalman_t k_gyro_x;
static kalman_t k_gyro_y;
static kalman_t k_gyro_z;

/* ============================================================
 * SENSOR INIT
 * ============================================================ */

static void sensors_init(void) {
  ESP_ERROR_CHECK(i2cdev_init());

  ESP_ERROR_CHECK(ms5611_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));

  ESP_ERROR_CHECK(mpu6050_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));

  ESP_ERROR_CHECK(bmp280_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
}

/* ============================================================
 * SIT KALMAN INIT
 * ============================================================ */

static void kalman_init_all_except_bmp_ms(void) {
  kalman_init(&k_accel_x, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_y, 0.05f, 0.0000769f, 0.0f);
  kalman_init(&k_accel_z, 0.05f, 0.0000769f, 9.81f);
  kalman_init(&k_gyro_x, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_y, 0.02f, 0.000125f, 0.0f);
  kalman_init(&k_gyro_z, 0.02f, 0.000125f, 0.0f);
}

void run_sit(fcc_mode_t volatile *current_mode) {
  uart_flush_input(RS232_UART_NUM);

  uint8_t packet[SIT_PACKET_SIZE] = {0};

  bool pressure_kalman_initialized = false;

  sensors_init();
  kalman_init_all_except_bmp_ms();

  float ground_altitude = 0.0f;
  bool ground_calibrated = false;

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  TickType_t last_time = xTaskGetTickCount();

  while (*current_mode == FCC_MODE_SIT) {
    TickType_t loop_start = xTaskGetTickCount();

    float dt =
        (float)(loop_start - last_time) * ((float)portTICK_PERIOD_MS / 1000.0f);
    last_time = loop_start;
    if (dt <= 0.0f)
      dt = 0.1f;

    int32_t ms5611_pressure;
    float ms5611_temp;
    float bmp280_pressure;
    float bmp280_temp;
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;
    TiltState tilt = {0};

    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);
    esp_err_t r_ms = ms5611_drv_read(&ms5611_pressure, &ms5611_temp);
    esp_err_t r_bmp = bmp280_drv_read(&bmp280_pressure, &bmp280_temp);

    if (!pressure_kalman_initialized) {
      kalman_init(&k_pressure_ms, 0.05f, 100.0f, (float)ms5611_pressure);
      kalman_init(&k_pressure_bmp, 0.05f, 100.0f, bmp280_pressure);
      pressure_kalman_initialized = true;
      ESP_LOGI(TAG,
               "Pressure Kalman initialized: "
               "MS=%.2f | BMP=%.2f",
               (float)ms5611_pressure, bmp280_pressure);
      ESP_LOGI(TAG, "SIT Sensors initialized");
    }

    if (r_ms != ESP_OK || r_bmp != ESP_OK || r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error");
      goto next;
    }

    // Kalman Filtresi Güncellemeleri
    float ax = kalman_update(&k_accel_x, accel.x);
    float ay = kalman_update(&k_accel_y, accel.y);
    float az = kalman_update(&k_accel_z, accel.z);

    float gx = kalman_update(&k_gyro_x, gyro.x);
    float gy = kalman_update(&k_gyro_y, gyro.y);
    float gz = kalman_update(&k_gyro_z, gyro.z);

    float pressure_ms = kalman_update(&k_pressure_ms, (float)ms5611_pressure);
    float pressure_bmp = kalman_update(&k_pressure_bmp, bmp280_pressure);

    float pressure = weighted_average(pressure_ms, 0.4f, pressure_bmp, 0.6f);

    float abs_altitude =
        44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

    if (!ground_calibrated) {
      ground_altitude = abs_altitude;
      ground_calibrated = true;
      ESP_LOGI(TAG, "Ground altitude calibrated: %.2f m", ground_altitude);
    }

    float relative_altitude = abs_altitude - ground_altitude;

    tilt_update(&tilt, ax, ay, az, gx, gy, gz, dt);
    float tilt_mag = sqrtf(tilt.pitch * tilt.pitch + tilt.roll * tilt.roll);
    float tilt_deg = tilt_mag * 180.0f / (float)M_PI;
    float tilt_ten = tilt_deg * 10.0;

    float accel_roll = atan2f(ay, az) * (180.0f / M_PI);
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);

    roll = 0.90f * (roll + gx * dt) + 0.04f * accel_roll;
    pitch = 0.90f * (pitch + gy * dt) + 0.04f * accel_pitch;
    yaw += gz * dt;
    ESP_LOGI(TAG,
             "RelAlt=%.2f m | Roll=%.1f deg | Pitch=%.1f deg | Yaw=%.1f deg",
             relative_altitude, roll, pitch, yaw);

    packet[0] = SIT_HEADER;

    write_be_float(&packet[1], relative_altitude);
    write_be_float(&packet[5], pressure / 100.0f);

    write_be_float(&packet[9], gx);
    write_be_float(&packet[13], gy);
    write_be_float(&packet[17], gz);

    write_be_float(&packet[21], tilt_ten);
    write_be_float(&packet[25], tilt_ten);
    write_be_float(&packet[29], tilt_ten);

    packet[33] = calc_checksum(packet, 33);
    packet[34] = SIT_FOOTER1;
    packet[35] = SIT_FOOTER2;

    int sent = uart_write_bytes(RS232_UART_NUM, packet, SIT_PACKET_SIZE);
    if (sent != SIT_PACKET_SIZE) {
      ESP_LOGE(TAG, "SIT write incomplete: %d/%d", sent, SIT_PACKET_SIZE);
    }

  next:
    vTaskDelayUntil(&loop_start, pdMS_TO_TICKS(LOOP_PERIOD_MS));
  }

  ESP_LOGI(TAG, "SIT loop exited");
}

#define SUT_PACKET_SIZE 36

static kalman_t k_pressure_sut;
static kalman_t k_altitude_sut;

static kalman_t k_accel_x_sut;
static kalman_t k_accel_y_sut;
static kalman_t k_accel_z_sut;

static kalman_t k_angle_x_sut;
static kalman_t k_angle_y_sut;
static kalman_t k_angle_z_sut;

static bool sut_kalman_initialized = false;

static void kalman_init_all_sut(float pressure, float altitude, float accel_x,
                                float accel_y, float accel_z, float angle_x,
                                float angle_y, float angle_z) {

  kalman_init(&k_pressure_sut, 0.05f, 1.44f, pressure);
  kalman_init(&k_altitude_sut, 0.05f, 1.0f, altitude);
  kalman_init(&k_accel_x_sut, 1.0f, 1.0f, accel_x);
  kalman_init(&k_accel_y_sut, 1.0f, 1.0f, accel_y);
  kalman_init(&k_accel_z_sut, 1.0f, 1.0f, accel_z);
  kalman_init(&k_angle_x_sut, 1.0f, 1.0f, angle_x);
  kalman_init(&k_angle_y_sut, 1.0f, 1.0f, angle_y);
  kalman_init(&k_angle_z_sut, 1.0f, 1.0f, angle_z);
  sut_kalman_initialized = true;
}

#define GRAVITY 9.81f
#define LIFTOFF_ACCEL_THRESHOLD (2.5f * GRAVITY)
#define LIFTOFF_CONFIRM_SAMPLES 5

static void check_liftoff(uint16_t *state, double magnitude) {
  static int liftoff_counter = 0;
  if (*state & 0x0001)
    return;
  if (magnitude >= LIFTOFF_ACCEL_THRESHOLD) {
    liftoff_counter++;
  } else {
    liftoff_counter = 0;
  }
  if (liftoff_counter >= LIFTOFF_CONFIRM_SAMPLES) {
    *state |= 0x0001;
    ESP_LOGI(TAG, "LIFTOFF DETECTED");
  }

  if (*state & 0x8000) {
    liftoff_counter = 0;
  }
}

#define BURNOUT_WINDOW_SIZE 3
#define BURNOUT_ACCEL_THRESHOLD 12.0f

static void check_burnout(uint16_t *state, double magnitude) {
  static double magnitude_window[BURNOUT_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;
  if (!(*state & 0x0001))
    return;
  if (*state & 0x0002)
    return;
  magnitude_window[window_index++] = magnitude;
  if (!first_window_iteration && window_index > BURNOUT_WINDOW_SIZE - 1) {
    first_window_iteration = true;
  }

  window_index %= BURNOUT_WINDOW_SIZE;

  if (!first_window_iteration)
    return;

  double sum = 0.0;
  for (int i = 0; i < BURNOUT_WINDOW_SIZE; i++) {
    sum += magnitude_window[i];
  }

  double average = sum / BURNOUT_WINDOW_SIZE;
  if (average <= BURNOUT_ACCEL_THRESHOLD) {
    *state |= 0x0002;
    ESP_LOGI(TAG, "BURNOUT DETECTED");
  }

  if (*state & 0x8000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

#define ALTITUDE_LOCK 1000.0f
#define ALTITUDE_WINDOW_SIZE 5

static void check_altitude_lock(uint16_t *state, float altitude) {
  static double altitude_window[ALTITUDE_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;
  if (*state & 0x0004)
    return;
  altitude_window[window_index++] = altitude;
  if (!first_window_iteration && window_index > ALTITUDE_WINDOW_SIZE - 1) {
    first_window_iteration = true;
  }

  window_index %= ALTITUDE_WINDOW_SIZE;
  if (!first_window_iteration)
    return;
  float sum = 0.0f;
  for (int i = 0; i < ALTITUDE_WINDOW_SIZE; i++) {
    sum += (float)altitude_window[i];
  }
  float average = sum / ALTITUDE_WINDOW_SIZE;
  if (average >= ALTITUDE_LOCK) {
    *state |= 0x0004;
    ESP_LOGI(TAG, "ALTITUDE LOCK DETECTED: %.2f m", average);
  }

  if (*state & 0x8000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

#define TILT_THRESHOLD 75.0f
#define TILT_WINDOW_SIZE 10

static void check_tilt(uint16_t *state, float tilt) {
  static double tilt_window[TILT_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;
  if (!(*state & 0x0004))
    return;
  if (*state & 0x0008)
    return;
  tilt_window[window_index++] = tilt;
  if (!first_window_iteration && window_index > TILT_WINDOW_SIZE - 1) {
    first_window_iteration = true;
  }

  window_index %= TILT_WINDOW_SIZE;
  if (!first_window_iteration)
    return;
  float sum = 0.0f;
  for (int i = 0; i < TILT_WINDOW_SIZE; i++) {
    sum += (float)tilt_window[i];
  }

  float average = sum / TILT_WINDOW_SIZE;
  if (average >= TILT_THRESHOLD) {
    *state |= 0x0008;
    ESP_LOGI(TAG, "TILT LIMIT DETECTED: %.2f deg", average);
  }

  if (*state & 0x8000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

static void check_if_altitude_descending(uint16_t *state, float altitude) {
  static double altitude_window[ALTITUDE_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;
  if (!(*state & 0x0008))
    return;
  if (*state & 0x0010)
    return;
  altitude_window[window_index++] = altitude;
  if (!first_window_iteration && window_index > ALTITUDE_WINDOW_SIZE - 1)
    first_window_iteration = true;
  window_index %= ALTITUDE_WINDOW_SIZE;
  if (!first_window_iteration)
    return;
  bool descending = true;
  for (int i = 0, c = window_index; i < ALTITUDE_WINDOW_SIZE - 1;
       i++, c = (c + 1) % ALTITUDE_WINDOW_SIZE) {
    int next = (c + 1) % ALTITUDE_WINDOW_SIZE;
    if (altitude_window[next] >= altitude_window[c]) {
      descending = false;
      break;
    }
  }

  if (descending) {
    *state |= 0x0010;
    ESP_LOGI(TAG, "ALTITUDE DESCENDING");
  }

  if (*state & 0x8000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

static void check_parachute(uint16_t *state) {

  if (!(*state & 0x0010))
    return;

  if (*state & 0x0080)
    return;

  gpio_reset_pin(PIN_LED_APOGEE);
  gpio_set_direction(PIN_LED_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED_APOGEE, 0);

  gpio_reset_pin(PIN_HGG_APOGEE);
  gpio_set_direction(PIN_HGG_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_HGG_APOGEE, 0);

  *state |= 0x0080;
  gpio_set_level(PIN_LED_APOGEE, 1);
  gpio_set_level(PIN_HGG_APOGEE, 1);
  ESP_LOGI(TAG, "MAIN PARACHUTE COMMAND");
}

static uint16_t state = 0;
static void check_state(double magnitude, float altitude, float tilt) {
  check_liftoff(&state, magnitude);
  check_burnout(&state, magnitude);
  check_altitude_lock(&state, altitude);
  check_tilt(&state, tilt);
  check_if_altitude_descending(&state, altitude);
  check_parachute(&state);
  if (state & 0x8000)
    state = 0;
}

#define SUT_HEADER 0xAA
#define SUT_DATA_HEADER 0xAB

#define SUT_FOOTER1 0x0D
#define SUT_FOOTER2 0x0A

static float calc_tilt_sut(float angle_x, float angle_y) {
  float tilt = sqrtf(angle_x * angle_x + angle_y * angle_y);
  if (tilt > 180.0f)
    tilt = 180.0f;
  return tilt;
}

void run_sut(fcc_mode_t volatile *current_mode) {
  ESP_LOGI(TAG, "ENTERED run_sut");
  uart_flush_input(RS232_UART_NUM);
  sut_kalman_initialized = false;
  const int PKT_SIZE = SUT_PACKET_SIZE;
  uint8_t raw[PKT_SIZE];
  while (*current_mode == FCC_MODE_SUT) {
    uint8_t b;

    while (1) {
      if (uart_read_bytes(RS232_UART_NUM, &b, 1, pdMS_TO_TICKS(1000)) != 1) {
        if (*current_mode != FCC_MODE_SUT)
          return;
        continue;
      }

      if (b == SUT_DATA_HEADER) // 0xAB
        break;

      if (b == CMD_HEADER) { // 0xAA
        uint8_t cmd_buf[CMD_PACKET_SIZE];
        cmd_buf[0] = b;
        int got = uart_read_bytes(RS232_UART_NUM, &cmd_buf[1],
                                  CMD_PACKET_SIZE - 1, pdMS_TO_TICKS(100));
        if (got == CMD_PACKET_SIZE - 1)
          apply_command_packet(cmd_buf);
      }
    }

    ESP_LOGW(TAG, "RX %02X", b);

    raw[0] = b;

    int got = uart_read_bytes(RS232_UART_NUM, &raw[1], PKT_SIZE - 1,
                              pdMS_TO_TICKS(100));
    if (got != PKT_SIZE - 1) {
      ESP_LOGW(TAG, "Short packet %d/%d", got + 1, PKT_SIZE);
      continue;
    }

    ESP_LOGI(TAG, "========== PACKET ==========");
    ESP_LOG_BUFFER_HEXDUMP(TAG, raw, PKT_SIZE, ESP_LOG_INFO);
    ESP_LOGI(TAG, "============================");

    if (raw[34] != SUT_FOOTER1 || raw[35] != SUT_FOOTER2) {
      ESP_LOGW(TAG, "Footer error %02X %02X", raw[34], raw[35]);
      continue;
    }

    float altitude = read_be_float(&raw[1]);
    float pressure = read_be_float(&raw[5]);
    float accel_x = read_be_float(&raw[9]);
    float accel_y = read_be_float(&raw[13]);
    float accel_z = read_be_float(&raw[17]);
    float angle_x = read_be_float(&raw[21]);
    float angle_y = read_be_float(&raw[25]);
    float angle_z = read_be_float(&raw[29]);

    ESP_LOGI(TAG,
             "ALT=%f PRES=%f "
             "AX=%f AY=%f AZ=%f "
             "ANGLE_X=%f ANGLE_Y=%f ANGLE_Z=%f",
             altitude, pressure, accel_x, accel_y, accel_z, angle_x, angle_y,
             angle_z);

    if (!sut_kalman_initialized) {
      kalman_init_all_sut(pressure, altitude, accel_x, accel_y, accel_z,
                          angle_x, angle_y, angle_z);
      ESP_LOGI(TAG, "SUT Kalman initialized "
                    "from first packet");
    }

    float ax = kalman_update(&k_accel_x_sut, accel_x);
    float ay = kalman_update(&k_accel_y_sut, accel_y);
    float az = kalman_update(&k_accel_z_sut, accel_z);
    float al = kalman_update(&k_altitude_sut, altitude);
    float filtered_angle_x = kalman_update(&k_angle_x_sut, angle_x);
    float filtered_angle_y = kalman_update(&k_angle_y_sut, angle_y);
    float filtered_angle_z = kalman_update(&k_angle_z_sut, angle_z);

    float filtered_pressure = kalman_update(&k_pressure_sut, pressure);
    (void)filtered_pressure;

    double magnitude =
        sqrt((double)ax * ax + (double)ay * ay + (double)az * az);

    float tilt = calc_tilt_sut(filtered_angle_x, filtered_angle_y);
    ESP_LOGI(TAG,
             "MAG=%.2f ALT=%.2f "
             "ANGLE_X=%.2f "
             "ANGLE_Y=%.2f "
             "ANGLE_Z=%.2f "
             "TILT=%.2f",
             magnitude, al, filtered_angle_x, filtered_angle_y,
             filtered_angle_z, tilt);

    check_state(magnitude, al, tilt);

    uint8_t tx[SUT_WRITE_SIZE];

    tx[0] = SUT_HEADER;
    memcpy(&tx[1], &state, 2);
    tx[3] = calc_checksum(tx, 3);
    tx[4] = SUT_FOOTER1;
    tx[5] = SUT_FOOTER2;

    int sent =
        uart_write_bytes(RS232_UART_NUM, (const char *)tx, SUT_WRITE_SIZE);
    if (sent != SUT_WRITE_SIZE) {
      ESP_LOGE(TAG,
               "SUT status write incomplete: "
               "%d/%d",
               sent, SUT_WRITE_SIZE);
    }

    ESP_LOGI(TAG, "========== TX STATUS PACKET ==========");
    ESP_LOG_BUFFER_HEXDUMP(TAG, tx, SUT_WRITE_SIZE, ESP_LOG_INFO);
    ESP_LOGI(TAG,
             "state=0x%04X "
             "data1=0x%02X "
             "data2=0x%02X "
             "checksum=0x%02X",
             state, tx[1], tx[2], tx[3]);
    ESP_LOGI(TAG, "=======================================");
  }
}

void mode_apply_pending(fcc_mode_t volatile *current_mode) {
  if (!mode_pending)
    return;
  if ((xTaskGetTickCount() - mode_pending_since) >= pdMS_TO_TICKS(1000)) {
    *current_mode = pending_mode;
    mode_pending = false;
    uart_flush_input(RS232_UART_NUM);
    ESP_LOGI(TAG, "Mode applied after 1 second");
  }
}

esp_err_t max3232_drv_init(int uart_num, int tx_gpio, int rx_gpio,
                           int baud_rate) {
  uart_config_t uart_cfg = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  esp_err_t ret = uart_param_config(uart_num, &uart_cfg);

  if (ret != ESP_OK)
    return ret;

  ret = uart_set_pin(uart_num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);

  if (ret != ESP_OK)
    return ret;

  ret = uart_driver_install(uart_num, 256, 0, 0, NULL, 0);

  if (ret != ESP_OK)
    return ret;

  ESP_LOGI(TAG, "Initialized RS232 UART%d at %d baud", uart_num, baud_rate);
  return ESP_OK;
}
