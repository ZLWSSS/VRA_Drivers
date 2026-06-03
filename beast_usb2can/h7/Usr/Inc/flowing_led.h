#ifndef FLOWING_LED_H_
#define FLOWING_LED_H_

#include "stdint.h"

typedef enum flowing_mode_{
  Led_Null = 0,
  Led_Standby,
  Led_Error,
  Led_Working,
  Led_Pause,
} FLOWING_MODE_t;

typedef void (*led_run)(FLOWING_MODE_t mode);
void flow_led_handle_init(void);


typedef struct flow_led_handle {
  float delta_alpha, delta_red, delta_green, delta_blue;
  float alpha, red, green, blue;
  uint32_t aRGB;
  FLOWING_MODE_t mode_;
  led_run func_run;
} flowing_led_handle_t;

flowing_led_handle_t* get_flowing_led_handle(void);

#endif