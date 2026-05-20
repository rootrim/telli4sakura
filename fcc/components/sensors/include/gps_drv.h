#pragma once

#include <esp_err.h>

/**
 * @brief GPS position data.
 */
typedef struct {
  float latitude;  /*!< Latitude in decimal degrees  */
  float longitude; /*!< Longitude in decimal degrees */
} gps_data_t;

/**
 * @brief Initializes the GPS UART.
 *
 * @param[in] uart_num  UART port number.
 * @param[in] tx_gpio   TX pin number.
 * @param[in] rx_gpio   RX pin number.
 * @param[in] baud_rate Baud rate (SE100 default: 9600).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t gps_drv_init(int uart_num, int tx_gpio, int rx_gpio, int baud_rate);

/**
 * @brief Reads lat/lon from GPS.
 *
 * Blocks until a valid GGA sentence is received or timeout occurs.
 *
 * @param[out] data GPS position data.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if no valid sentence.
 */
esp_err_t gps_drv_read(gps_data_t *data);
