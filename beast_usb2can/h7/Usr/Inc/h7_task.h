#ifndef USB2CAN_H7_
#define USB2CAN_H7_

#include "usr_spi.h"
#include "usr_usb.h"

#define H7_RUN 1
#define H7_STOP 0

typedef struct _H7_BOARD {
  SPI_PROTOCAL_TypeDef* g474_chips[CHIP_NUMBERS];
  USB_PROTOCAL_TypeDef* h7_usb;
  uint8_t (*receive_usb_cmd)(uint8_t* usb_out_cmd_buf);
  void (*send_spi_cmd_dma)(void);
  uint8_t (*check_chips_data)(SPI_PROTOCAL_TypeDef* _spi_protocal);
  uint8_t (*send_usb_data)(void);
  void (*pack_usb_data)(void);

  uint8_t cmd_update;
  uint8_t run;
  uint32_t hit_miss;
} H7_Board_TypeDef;

void init_h7_board(void);
H7_Board_TypeDef* get_h7_board(void);
void send_data(void);

static inline int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline uint32_t data_checksum(uint32_t* data_to_check, uint32_t check_length) {
  uint32_t t = 0;
  for (int i = 0; i < check_length; i++) {
    t = t ^ data_to_check[i];
  }
  return t;
}
#endif