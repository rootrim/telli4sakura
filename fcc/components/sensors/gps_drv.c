#include "gps_drv.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"

#define GPS_BUF_SIZE (512)
#define GPS_TIMEOUT_MS (2000)

static const char *TAG = "gps_drv";
static int s_uart_num;

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

  ESP_LOGI(TAG, "Initialized UART%d at %d baud", uart_num, baud_rate);
  return ESP_OK;
}

esp_err_t gps_drv_read(gps_data_t *data) {
  char line[MINMEA_MAX_SENTENCE_LENGTH];
  uint8_t ch;
  int idx = 0;

  TickType_t timeout = pdMS_TO_TICKS(GPS_TIMEOUT_MS);
  TickType_t start = xTaskGetTickCount();

  while ((xTaskGetTickCount() - start) < timeout) {
    if (uart_read_bytes(s_uart_num, &ch, 1, pdMS_TO_TICKS(10)) != 1) {
      continue;
    }

    if (ch == '\n') {
      line[idx] = '\0';
      idx = 0;

      // GGA cümlesi: lat/lon/fix bilgisi
      if (minmea_sentence_id(line, false) == MINMEA_SENTENCE_GGA) {
        struct minmea_sentence_gga frame;
        if (minmea_parse_gga(&frame, line) && frame.fix_quality > 0) {
          data->latitude = minmea_tocoord(&frame.latitude);
          data->longitude = minmea_tocoord(&frame.longitude);
          return ESP_OK;
        }
      }
    } else if (ch != '\r') {
      if (idx < (int)sizeof(line) - 1) {
        line[idx++] = ch;
      } else {
        idx = 0; // overflow, satırı at
      }
    }
  }

  ESP_LOGW(TAG, "Timeout waiting for GPS fix");
  return ESP_ERR_TIMEOUT;
}
