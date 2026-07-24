#pragma once

#include <stdint.h>
#define RS232_UART UART_NUM_0

#define CMD_HEADER 0xAA
#define CMD_FOOTER1 0x0D
#define CMD_FOOTER2 0x0A
#define CMD_SIT_COMMAND 0x20
#define CMD_SUT_COMMAND 0x22
#define CMD_DUR_COMMAND 0x24
#define CMD_SIT_CHECKSUM 0x8C
#define CMD_SUT_CHECKSUM 0x8E
#define CMD_DUR_CHECKSUM 0x90
#define CMD_PACKET_SIZE 5

#define SUT_READ_SIZE 36
#define SUT_WRITE_SIZE 6

#define SIT_PACKET_SIZE 36
#define SIT_HEADER 0xAB
#define SIT_FOOTER1 0x0D
#define SIT_FOOTER2 0x0A

typedef enum {
  FCC_MODE_SIT = 0x20,
  FCC_MODE_SUT = 0x22,
  FCC_MODE_DUR = 0x24,
} fcc_mode_t;

typedef struct {
  uint8_t header;
  float altitude;
  float pressure;
  float accel_x;
  float accel_y;
  float accel_z;
  float angle_x;
  float angle_y;
  float angle_z;
  uint8_t checksum;
  uint8_t footer1;
  uint8_t footer2;
} sut_data;
