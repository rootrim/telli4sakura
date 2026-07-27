#include "gps_drv.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "minmea.h"

#define GPS_BUF_SIZE (512)

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
