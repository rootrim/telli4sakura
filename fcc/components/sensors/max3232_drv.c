#include "driver/uart.h"
#include "esp_log.h"
#include "math.h"
#include <apogee.h>
// #include <bmp390_drv.h>
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
static volatile fcc_mode_t pending_mode;
static volatile bool mode_pending = false;
static volatile TickType_t mode_pending_since;

static uint8_t calc_checksum(const uint8_t *buf, int len) {
  uint8_t cs = 0;
  for (int i = 0; i < len; i++)
    cs += buf[i];
  return cs;
}

static inline void write_be_float(uint8_t *p, float f) {
  union {
    float f;
    uint32_t u;
  } v;

  v.f = f;

  p[0] = (v.u >> 24) & 0xFF;
  p[1] = (v.u >> 16) & 0xFF;
  p[2] = (v.u >> 8) & 0xFF;
  p[3] = v.u & 0xFF;
}

static inline void write_be_u16(uint8_t *p, uint16_t v) {
  p[0] = (v >> 8) & 0xFF;
  p[1] = v & 0xFF;
}

bool check_mode_command(fcc_mode_t volatile *out_mode) {
  uint8_t buf[CMD_PACKET_SIZE];

  int len = uart_read_bytes(RS232_UART_NUM, buf, CMD_PACKET_SIZE, 0);
  if (len != CMD_PACKET_SIZE)
    return false;

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

  // Footer kontrolu (spesifikasyonun istedigi, eskiden hic yoktu)
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

  // Komut dogrulandi ama HENUZ uygulanmiyor -- 1 saniye sonra uygulanacak
  // pending_mode = new_mode;
  // mode_pending = true;
  // mode_pending_since = xTaskGetTickCount();
  // ESP_LOGI(TAG, "Command validated, will apply in 1000ms");
  // return true;
  pending_mode = new_mode;
  mode_pending = true;
  mode_pending_since = xTaskGetTickCount();

  ESP_LOGI(TAG, "Command accepted, switching in 1 second");
  return true;
}

// TODO: Set actual pin numbers
#define PIN_I2C_SDA 5
#define PIN_I2C_SCL 1

#define I2C_PORT I2C_NUM_0

// 10Hz = 100ms
#define LOOP_PERIOD_MS 100

// Kalman instances — one per filtered value
static kalman_t k_pressure_ms;
// static kalman_t k_pressure_bmp;
static kalman_t k_accel_x;
static kalman_t k_accel_y;
static kalman_t k_accel_z;
static kalman_t k_gyro_x;
static kalman_t k_gyro_y;
static kalman_t k_gyro_z;

static void sensors_init(void) {
  ESP_ERROR_CHECK(i2cdev_init());
  // ESP_ERROR_CHECK(ms5611_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  ESP_ERROR_CHECK(mpu6050_drv_init(PIN_I2C_SDA, PIN_I2C_SCL));
  i2c_master_bus_handle_t i2c_bus;
  ESP_ERROR_CHECK(i2cdev_get_shared_handle(I2C_PORT, (void **)&i2c_bus));
  // ESP_ERROR_CHECK(bmp390_drv_init(i2c_bus));
}

static void kalman_init_all(void) {
  // TODO: tune parameters for 10Hz
  kalman_init(&k_pressure_ms, 0.05f, 1.44f, 0.0f);
  // kalman_init(&k_pressure_bmp, 0.05f, 0.0004f, 0.0f);
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

  sensors_init();
  kalman_init_all();

  ESP_LOGI(TAG, "SIT Sensors initialized");

  TickType_t sit_start = xTaskGetTickCount();

  while (*current_mode == FCC_MODE_SIT) {
    TickType_t loop_start = xTaskGetTickCount();

    if ((loop_start - sit_start) >= pdMS_TO_TICKS(5000)) {
      ESP_LOGI(TAG, "SIT duration elapsed, switching to SUT");
      *current_mode = FCC_MODE_SUT;
      break;
    }

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t gyro;

    esp_err_t r_mpu = mpu6050_drv_read(&accel, &gyro);

    if (r_mpu != ESP_OK) {
      ESP_LOGE(TAG, "Sensor read error: %s", esp_err_to_name(r_mpu));
      goto next;
    }

    {
      int32_t ms5611_pressure =
          (int32_t)(SIT_FAKE_PRESSURE_BASE_PA + generate_pressure_noise());

      float ax = kalman_update(&k_accel_x, accel.x);
      float ay = kalman_update(&k_accel_y, accel.y);
      float az = kalman_update(&k_accel_z, accel.z);
      float gx = kalman_update(&k_gyro_x, gyro.x);
      float gy = kalman_update(&k_gyro_y, gyro.y);
      float gz = kalman_update(&k_gyro_z, gyro.z);
      float pressure_ms = kalman_update(&k_pressure_ms, ms5611_pressure);

      float pressure = weighted_average(pressure_ms, 0.5f, pressure_ms, 0.5f);
      float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));

      ESP_LOGI(TAG,
               "alt=%.2f press=%.2f ax=%.3f ay=%.3f az=%.3f "
               "gx=%.3f gy=%.3f gz=%.3f",
               altitude, pressure, ax, ay, az, gx, gy, gz);

      packet[0] = SIT_HEADER;
      write_be_float(&packet[1], altitude);
      write_be_float(&packet[5], pressure / 100.0f);
      write_be_float(&packet[9], ax);
      write_be_float(&packet[13], ay);
      write_be_float(&packet[17], az);
      write_be_float(&packet[21], gx);
      write_be_float(&packet[25], gy);
      write_be_float(&packet[29], gz);
      packet[33] = calc_checksum(packet, 33);
      packet[34] = SIT_FOOTER1;
      packet[35] = SIT_FOOTER2;

      ESP_LOGI(TAG, "========== TX SIT PACKET ==========");
      ESP_LOG_BUFFER_HEXDUMP(TAG, packet, SIT_PACKET_SIZE, ESP_LOG_INFO);
      ESP_LOGI(TAG, "header=0x%02X checksum=0x%02X footer=0x%02X 0x%02X",
               packet[0], packet[33], packet[34], packet[35]);
      ESP_LOGI(TAG, "====================================");
    }

    int sent = uart_write_bytes(RS232_UART_NUM, packet, SIT_PACKET_SIZE);
    if (sent != SIT_PACKET_SIZE) {
      ESP_LOGE(TAG, "SIT write incomplete: %d/%d", sent, SIT_PACKET_SIZE);
    } else {
      ESP_LOGI(TAG, "SIT packet sent (%d bytes)", sent);
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
// TODO: Tune G value
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

  if (*state & 0b1000000000000000) {
    liftoff_counter = 0;
  }
}

#define BURNOUT_WINDOW_SIZE 3
#define BURNOUT_ACCEL_THRESHOLD 12.0f

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

  if (*state & 0b1000000000000000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

#define ALTITUDE_LOCK 1000
#define ALTITUDE_WINDOW_SIZE 10

static void check_altitude_lock(uint16_t *state, float altitude) {
  static double altitude_window[ALTITUDE_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;

  // if (!(*state & 0b10))
  //   return;
  if (*state & 0b100)
    return;
  altitude_window[window_index++] = altitude;

  if (!first_window_iteration && (window_index > ALTITUDE_WINDOW_SIZE - 1)) {
    first_window_iteration = true;
  }

  window_index %= ALTITUDE_WINDOW_SIZE;
  if (!first_window_iteration)
    return;

  float sum = 0;
  for (int y = 0; y < ALTITUDE_WINDOW_SIZE; y++) {
    sum += altitude_window[y];
  }
  float average = sum / ALTITUDE_WINDOW_SIZE;

  if (average >= ALTITUDE_LOCK)
    *state |= 0b100;

  if (*state & 0b1000000000000000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

#define TILT_THRESHOLD 80.0f
#define TILT_WINDOW_SIZE 10

static void check_tilt(uint16_t *state, float tilt) {
  static double tilt_window[TILT_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;

  if (!(*state & 0b100))
    return;
  if (*state & 0b1000)
    return;
  tilt_window[window_index++] = tilt;

  if (!first_window_iteration && (window_index > TILT_WINDOW_SIZE - 1)) {
    first_window_iteration = true;
  }

  window_index %= TILT_WINDOW_SIZE;

  if (!first_window_iteration)
    return;

  float sum = 0;
  for (int i = 0; i < TILT_WINDOW_SIZE; i++) {
    sum += tilt_window[i];
  }

  float average = sum / TILT_WINDOW_SIZE;
  if (average >= TILT_THRESHOLD)
    *state |= 0b1000;

  if (*state & 0b1000000000000000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

static void check_if_altitude_descending(uint16_t *state, float altitude) {
  static double altitude_window[ALTITUDE_WINDOW_SIZE];
  static int window_index = 0;
  static bool first_window_iteration = false;

  if (!(*state & 0b1000))
    return;
  if (*state & 0b10000)
    return;

  altitude_window[window_index++] = altitude;

  if (!first_window_iteration && (window_index > ALTITUDE_WINDOW_SIZE - 1)) {
    first_window_iteration = true;
  }

  window_index %= ALTITUDE_WINDOW_SIZE;

  if (!first_window_iteration)
    return;

  bool descending = true;

  // window_index en eski elemanı gösteriyor; N-1 ardışık çifti kontrol et
  for (int i = 0, c = window_index; i < ALTITUDE_WINDOW_SIZE - 1;
       i++, c = (c + 1) % ALTITUDE_WINDOW_SIZE) {
    int next = (c + 1) % ALTITUDE_WINDOW_SIZE;
    if (altitude_window[next] >= altitude_window[c]) {
      descending = false;
      break;
    }
  }

  if (descending)
    *state |= 0b10000;

  if (*state & 0b1000000000000000) {
    window_index = 0;
    first_window_iteration = false;
  }
}

#define PIN_LED_APOGEE 3

static void check_parachute(uint16_t *state) {
  if (!(*state & 0b10000))
    return;
  if (*state & 0b10000000)
    return;

  gpio_reset_pin(PIN_LED_APOGEE);
  gpio_set_direction(PIN_LED_APOGEE, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_LED_APOGEE, 0);

  *state |= 0b10000000;

  gpio_set_level(PIN_LED_APOGEE, 1);
}

static uint16_t state = 0;

static void check_state(double magnitude, float altitude, float tilt) {
  check_liftoff(&state, magnitude);
  check_burnout(&state, magnitude);
  check_altitude_lock(&state, altitude);
  check_tilt(&state, tilt);
  check_if_altitude_descending(&state, altitude);
  check_parachute(&state);
  if (state & 0b1000000000000000) {
    state = 0;
  }
}

#define SUT_HEADER 0xAA
#define SUT_DATA_HEADER 0xAB
#define SUT_FOOTER1 0x0D
#define SUT_FOOTER2 0x0A

static inline float read_be_float(const uint8_t *p) {
  union {
    uint32_t u;
    float f;
  } v;

  v.u = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
        ((uint32_t)p[2] << 8) | ((uint32_t)p[3]);

  return v.f;
}

float calc_tilt_sut(float ax, float ay, float az) {
  float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  if (magnitude < 1e-6f)
    return 0.0f;

  float c = az / magnitude;

  // Sayısal hatalara karşı
  if (c > 1.0f)
    c = 1.0f;
  if (c < -1.0f)
    c = -1.0f;

  return acosf(-az / magnitude) * 180.0f / (float)M_PI;
}

void run_sut(fcc_mode_t volatile *current_mode) {
  ESP_LOGI(TAG, "ENTERED run_sut");

  uart_flush_input(RS232_UART_NUM);
  kalman_init_all_sut();

  const int PKT_SIZE = 36;
  uint8_t raw[PKT_SIZE];

  while (*current_mode == FCC_MODE_SUT) {
    uint8_t b;

    /* Header ara */
    while (1) {
      if (uart_read_bytes(RS232_UART_NUM, &b, 1, pdMS_TO_TICKS(1000)) != 1) {
        if (*current_mode != FCC_MODE_SUT)
          return;

        continue;
      }

      if (b == SUT_DATA_HEADER) // 0xAB
        break;
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

    ESP_LOGI(TAG, "ALT=%f PRES=%f AX=%f AY=%f AZ=%f GX=%f GY=%f GZ=%f",
             altitude, pressure, accel_x, accel_y, accel_z, angle_x, angle_y,
             angle_z);

    float ax = kalman_update(&k_accel_x_sut, accel_x);
    float ay = kalman_update(&k_accel_y_sut, accel_y);
    float az = kalman_update(&k_accel_z_sut, accel_z);
    float al = kalman_update(&k_altitude_sut, altitude);

    double magnitude = sqrt(ax * ax + ay * ay + az * az);
    float tilt = calc_tilt_sut(ax, ay, az);

    ESP_LOGI(TAG, "MAG=%.2f ALT=%.2f TILT=%.2f", magnitude, al, tilt);

    check_state(magnitude, al, tilt);

    uint8_t tx[SUT_WRITE_SIZE];

    tx[0] = SUT_HEADER;

    memcpy(&tx[1], &state, 2);

    tx[3] = calc_checksum(tx, 3);

    tx[4] = SUT_FOOTER1;
    tx[5] = SUT_FOOTER2;

    uart_write_bytes(RS232_UART_NUM, (const char *)tx, SUT_WRITE_SIZE);

    ESP_LOGI(TAG, "========== TX STATUS PACKET ==========");
    ESP_LOG_BUFFER_HEXDUMP(TAG, tx, SUT_WRITE_SIZE, ESP_LOG_INFO);
    ESP_LOGI(TAG, "state=0x%04X data1=0x%02X data2=0x%02X checksum=0x%02X",
             state, tx[1], tx[2], tx[3]);
  }
}

void mode_apply_pending(fcc_mode_t volatile *current_mode) {
  // if (mode_pending &&
  //     (xTaskGetTickCount() - mode_pending_since) >= pdMS_TO_TICKS(1000)) {
  //   *current_mode = pending_mode;
  //   mode_pending = false;
  //   ESP_LOGI(TAG, "Mode applied after 1s delay");
  // }
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
