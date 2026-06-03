#ifndef USR_BUZZER_H_
#define USR_BUZZER_H_

#include "stdint.h"

typedef enum buzzer_mode_{
  b_NULL = 0,
  b_Standby,
  b_Error,
  b_Working,
  b_Pause,
} BUZZER_MODE_t;

typedef void (*buzzer_run)(void);

typedef struct buzzer_handle {
  BUZZER_MODE_t mode_;
  buzzer_run func_run;
  uint32_t set_beep_time_;
  uint32_t set_delay_time_; // ms
} buzzer_handle_t;

buzzer_handle_t* get_buzzer_handle(void);
void buzzer_run_isr(void);
void buzzer_start(void);

#endif