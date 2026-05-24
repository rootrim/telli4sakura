#include "lora.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "lora";
static int s_uart_num;

esp_err_t lora_init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate) {
  s_uart_num = uart_num;

  uart_config_t cfg = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  esp_err_t ret = uart_param_config(uart_num, &cfg);
  if (ret != ESP_OK)
    return ret;

  ret = uart_set_pin(uart_num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (ret != ESP_OK)
    return ret;

  ret = uart_driver_install(uart_num, 256, 0, 0, NULL, 0);
  if (ret != ESP_OK)
    return ret;

  ESP_LOGI(TAG, "Initialized UART%d at %d baud", uart_num, baud_rate);
  return ESP_OK;
}

static uint8_t calc_checksum(const uint8_t *buf, int len) {
  uint8_t cs = 0;
  for (int i = 0; i < len; i++) {
    cs += buf[i];
  }
  return cs;
}

static void pack_float(uint8_t *buf, float val) {
  memcpy(buf, &val, sizeof(float));
}

esp_err_t lora_send(const lora_packet_data_t *data) {
  uint8_t packet[LORA_PACKET_SIZE] = {0};

  packet[0] = LORA_HEADER;

  pack_float(&packet[1], data->altitude);
  pack_float(&packet[5], data->pressure);
  pack_float(&packet[9], data->accel_x);
  pack_float(&packet[13], data->accel_y);
  pack_float(&packet[17], data->accel_z);
  pack_float(&packet[21], data->angle_x);
  pack_float(&packet[25], data->angle_y);
  pack_float(&packet[29], data->angle_z);
  pack_float(&packet[33], data->gps_lat);
  pack_float(&packet[37], data->gps_lon);

  // Checksum: sum of bytes [0..40]
  packet[41] = calc_checksum(packet, 41);

  packet[42] = LORA_FOOTER_1;
  packet[43] = LORA_FOOTER_2;

  int sent =
      uart_write_bytes(s_uart_num, (const char *)packet, LORA_PACKET_SIZE);
  if (sent != LORA_PACKET_SIZE) {
    ESP_LOGE(TAG, "Send failed: wrote %d of %d bytes", sent, LORA_PACKET_SIZE);
    return ESP_FAIL;
  }

  return ESP_OK;
}
