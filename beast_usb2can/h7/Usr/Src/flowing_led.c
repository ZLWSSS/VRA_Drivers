#include "flowing_led.h"
#include "stm32h7xx_ll_tim.h"
#include "tim.h"
#include "usr_delay.h"

flowing_led_handle_t local_fl_handle;

#define RGB_FLOW_COLOR_CHANGE_TIME 80
#define RGB_FLOW_COLOR_LENGHT 3
#define Mode_Flowing_Delay 2    // ms
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFF0000FF};
uint8_t red_g, blue_g, green_g, alpha_g, flowing_direct;

void aRGB_Status_Normal(uint32_t aRGB) {
  static uint8_t alpha;
  static uint16_t red, green, blue;
  alpha = (aRGB & 0xFF000000) >> 24;
  red   = ((aRGB & 0x00FF0000) >> 16) * alpha;
  green = ((aRGB & 0x0000FF00) >> 8) * alpha;
  blue  = ((aRGB & 0x000000FF) >> 0) * alpha;
  LL_TIM_OC_SetCompareCH3(TIM4, blue);
  LL_TIM_OC_SetCompareCH2(TIM4, green);
  LL_TIM_OC_SetCompareCH1(TIM4, red);    // ch1 blue
}

static void run_led(FLOWING_MODE_t mode) {
  switch (mode) {
  case Led_Working:
    for (uint16_t i = 0; i < RGB_FLOW_COLOR_LENGHT; i++) {
      local_fl_handle.alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
      local_fl_handle.red   = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
      local_fl_handle.green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
      local_fl_handle.blue  = ((RGB_flow_color[i] & 0x000000FF) >> 0);

      local_fl_handle.delta_alpha =
          (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
      local_fl_handle.delta_red =
          (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
      local_fl_handle.delta_green =
          (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
      local_fl_handle.delta_blue =
          (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

      local_fl_handle.delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
      local_fl_handle.delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
      local_fl_handle.delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
      local_fl_handle.delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
      for (uint16_t j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++) {
        local_fl_handle.alpha += local_fl_handle.delta_alpha;
        local_fl_handle.red += local_fl_handle.delta_red;
        local_fl_handle.green += local_fl_handle.delta_green;
        local_fl_handle.blue += local_fl_handle.delta_blue;
        local_fl_handle.aRGB = ((uint32_t)(local_fl_handle.alpha)) << 24 | ((uint32_t)(local_fl_handle.red)) << 16 |
            ((uint32_t)(local_fl_handle.green)) << 8 | ((uint32_t)(local_fl_handle.blue)) << 0;
        aRGB_Status_Normal(local_fl_handle.aRGB);
        delay_ms(1);
      }
    }
    break;
  case Led_Standby:
    if (!flowing_direct) {
      alpha_g++;
      green_g++;
    } else {
      alpha_g--;
      green_g--;
    }
    if (green_g == 0XFF) {
      flowing_direct = 1;
    } else if (green_g == 0) {
      flowing_direct = 0;
    }
    local_fl_handle.aRGB = (alpha_g) << 24 | green_g << 8;
    aRGB_Status_Normal(local_fl_handle.aRGB);
    delay_ms(Mode_Flowing_Delay);
    break;
  case Led_Error:
    if (!flowing_direct) {
      alpha_g++;
      red_g++;
    } else {
      alpha_g--;
      red_g--;
    }
    if (red_g == 0XFF) {
      flowing_direct = 1;
    } else if (red_g == 0) {
      flowing_direct = 0;
    }
    local_fl_handle.aRGB = (alpha_g) << 24 | red_g << 16;
    aRGB_Status_Normal(local_fl_handle.aRGB);
    delay_ms(Mode_Flowing_Delay);
    break;
  case Led_Pause:
    if (!flowing_direct) {
      alpha_g++;
      blue_g++;
    } else {
      alpha_g--;
      blue_g--;
    }
    if (blue_g == 0XFF) {
      flowing_direct = 1;
    } else if (blue_g == 0) {
      flowing_direct = 0;
    }
    local_fl_handle.aRGB = (alpha_g) << 24 | blue_g;
    aRGB_Status_Normal(local_fl_handle.aRGB);
    delay_ms(Mode_Flowing_Delay);
    break;
  default:
    break;
  }
}

void flow_led_handle_init(void) {
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  LL_TIM_OC_SetCompareCH1(TIM4, 0);
  LL_TIM_OC_SetCompareCH2(TIM4, 0);
  LL_TIM_OC_SetCompareCH3(TIM4, 0);
  local_fl_handle.func_run = run_led;
}

flowing_led_handle_t* get_flowing_led_handle(void) {
  return &local_fl_handle;
}