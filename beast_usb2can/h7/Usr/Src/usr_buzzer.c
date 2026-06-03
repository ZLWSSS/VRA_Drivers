#include "usr_buzzer.h"
#include "stm32h7xx_ll_tim.h"
#include "tim.h"
#include "usr_delay.h"

buzzer_handle_t buzzer;
#define OC_Init 1000
#define OC_Beep 2000
#define OC_Null 0
uint32_t count_down_time, beep_delay;

buzzer_handle_t* get_buzzer_handle(void) {
  return &buzzer;
}

void Beep(void)    // ms
{
  if (beep_delay == 1) {
    LL_TIM_OC_SetCompareCH1(TIM1, OC_Beep);
    count_down_time--;
    if (count_down_time == 0) {
      count_down_time = buzzer.set_delay_time_;
      beep_delay      = 0;
    }
  } else if (beep_delay == 0) {
    LL_TIM_OC_SetCompareCH1(TIM1, OC_Null);
    count_down_time--;
    if (count_down_time == 0) {
      count_down_time = buzzer.set_beep_time_;
      beep_delay      = 1;
    }
  } else {
  }
}

static void Run(void) {
  switch (buzzer.mode_) {
  case b_Standby:
    buzzer.set_beep_time_  = 100;
    buzzer.set_delay_time_ = 60000;
    beep_delay             = 1;
    count_down_time        = buzzer.set_beep_time_;
    break;
  case b_Working:
    buzzer.set_beep_time_  = 0;
    buzzer.set_delay_time_ = 0;
    beep_delay             = 2; // no beep
    count_down_time        = buzzer.set_beep_time_;
    break;
  default:
    break;
  }
  buzzer.mode_ = b_NULL;
  Beep();
}

void buzzer_run_isr(void) {
  buzzer.func_run();
}

void buzzer_start(void) {
  buzzer.func_run = Run;
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  LL_TIM_OC_SetCompareCH1(TIM1, OC_Init);
  delay_ms(50);
  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  delay_ms(50);
  LL_TIM_OC_SetCompareCH1(TIM1, OC_Init);
  delay_ms(50);
  LL_TIM_OC_SetCompareCH1(TIM1, 0);
  delay_ms(50);
}
