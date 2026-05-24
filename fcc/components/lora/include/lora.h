#pragma once

#include "esp_err.h"
#include <stdint.h>

#define LORA_PACKET_SIZE 44
#define LORA_HEADER 0xAB
#define LORA_FOOTER_1 0x0D
#define LORA_FOOTER_2 0x0A

/**
 * @brief Sensor data to be packed and sent over LoRa.
 */
typedef struct {
  float altitude;
  float pressure;
  float accel_x;
  float accel_y;
  float accel_z;
  float angle_x;
  float angle_y;
  float angle_z;
  float gps_lat;
  float gps_lon;
} lora_packet_data_t;

/**
 * @brief Initializes the LoRa UART.
 *
 * @param[in] uart_num  UART port number.
 * @param[in] tx_gpio   TX pin.
 * @param[in] rx_gpio   RX pin.
 * @param[in] baud_rate Baud rate.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t lora_init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate);

/**
 * @brief Packs sensor data into 44-byte packet and sends over LoRa UART.
 *
 * @param[in] data Sensor data.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t lora_send(const lora_packet_data_t *data);
