#ifndef USR_DELAY_H
#define USR_DELAY_H
#include "stdint.h"

void delay_init(void);
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
#endif