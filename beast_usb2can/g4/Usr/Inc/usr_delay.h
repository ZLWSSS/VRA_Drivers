#ifndef USR_DELAY_H_
#define USR_DELAY_H_

#include "stdint.h"

void delay_us(uint16_t nus);
void delay_init(void);
void delay_ms(uint16_t nms);

#endif