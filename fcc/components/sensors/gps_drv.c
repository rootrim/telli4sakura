#include "gps_drv.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"

#define GPS_BUF_SIZE (512)
#define GPS_DATA_HZ (10)

static const char *TAG = "gps_drv";
static int s_uart_num;

static void ubx_checksum(const uint8_t *data, int len, uint8_t *ck_a,
                         uint8_t *ck_b) {
  *ck_a = 0;
  *ck_b = 0;
  for (int i = 0; i < len; i++) {
    *ck_a = (*ck_a + data[i]) & 0xFF;
    *ck_b = (*ck_b + *ck_a) & 0xFF;
  }
}

static esp_err_t gps_drv_set_rate(int hz) {
  if (hz <= 0 || hz > 20) {
    ESP_LOGE(TAG, "Invalid rate: %d", hz);
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t meas_rate_ms = 1000 / hz;
  uint16_t nav_rate = 1;
  uint16_t time_ref = 1;

  uint8_t body[10];
  body[0] = 0x06;
  body[1] = 0x08;
  body[2] = 0x06;
  body[3] = 0x00;
  memcpy(&body[4], &meas_rate_ms, 2);
  memcpy(&body[6], &nav_rate, 2);
  memcpy(&body[8], &time_ref, 2);

  uint8_t ck_a, ck_b;
  ubx_checksum(body, sizeof(body), &ck_a, &ck_b);

  uint8_t packet[2 + sizeof(body) + 2];
  packet[0] = 0xB5;
  packet[1] = 0x62;
  memcpy(&packet[2], body, sizeof(body));
  packet[sizeof(packet) - 2] = ck_a;
  packet[sizeof(packet) - 1] = ck_b;

  // Once gelen NMEA/UBX ne varsa temizle, ACK'i temiz gorelim
  uart_flush_input(s_uart_num);

  int sent = uart_write_bytes(s_uart_num, (const char *)packet, sizeof(packet));
  if (sent != sizeof(packet)) {
    ESP_LOGE(TAG, "UBX-CFG-RATE write incomplete: %d/%d", sent,
             (int)sizeof(packet));
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "UBX-CFG-RATE sent: %dHz (measRate=%dms)", hz, meas_rate_ms);

  // ACK/NAK bekle (B5 62 05 01 = ACK, B5 62 05 00 = NAK)
  uint8_t resp[64];
  int total = 0;
  TickType_t start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(500) &&
         total < (int)sizeof(resp)) {
    int n = uart_read_bytes(s_uart_num, &resp[total], 1, pdMS_TO_TICKS(50));
    if (n == 1) {
      total++;
      if (total >= 4) {
        for (int i = 0; i <= total - 4; i++) {
          if (resp[i] == 0xB5 && resp[i + 1] == 0x62 && resp[i + 2] == 0x05) {
            if (resp[i + 3] == 0x01) {
              ESP_LOGI(TAG, "UBX ACK received");
              return ESP_OK;
            } else if (resp[i + 3] == 0x00) {
              ESP_LOGW(TAG, "UBX NAK received - command rejected");
              return ESP_FAIL;
            }
          }
        }
      }
    }
  }

  ESP_LOGW(TAG, "No UBX ACK/NAK received within timeout");
  return ESP_ERR_TIMEOUT;
}

esp_err_t gps_drv_init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate) {
  s_uart_num = uart_num;

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

  ret = uart_driver_install(uart_num, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK)
    return ret;

  esp_err_t rate_ret = gps_drv_set_rate(GPS_DATA_HZ);
  if (rate_ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to set GPS rate, continuing with default");
  }

  ESP_LOGI(TAG, "Initialized UART%d at %d baud", uart_num, baud_rate);
  return ESP_OK;
}

static char line[MINMEA_MAX_SENTENCE_LENGTH];
static int idx = 0;

esp_err_t gps_drv_read(gps_data_t *data) {
  uint8_t ch;

  while (uart_read_bytes(s_uart_num, &ch, 1, 0) == 1) {
    if (ch == '\n') {
      line[idx] = '\0';
      idx = 0;
      if (minmea_sentence_id(line, true) == MINMEA_SENTENCE_GGA) {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, line) && frame.fix_quality > 0) {
          ESP_LOGI(TAG, "GGA emitted, fix_quality=%d, sats=%d",
                   frame.fix_quality, frame.satellites_tracked);
          data->latitude = minmea_tocoord(&frame.latitude);
          data->longitude = minmea_tocoord(&frame.longitude);
          return ESP_OK;
        }
      }
    } else if (ch != '\r') {
      if (idx < (int)sizeof(line) - 1) {
        line[idx++] = ch;
      } else {
        idx = 0;
      }
    }
  }

  ESP_LOGD(TAG, "GGA not yet complete, will continue next call");
  return ESP_ERR_NOT_FOUND;
}
