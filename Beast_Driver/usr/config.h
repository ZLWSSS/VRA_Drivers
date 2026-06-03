#ifndef CONFIG_H_
#define CONFIG_H_

#include "stdint.h"
// WARNING ***************************************************************************************
//             DO NOT CHANGE THIS FILE
//  *********************************************************************************************

extern const int MT6835_Resolution;
extern const int Rotor_Encoder_Half_CPR;
extern const float R_sense;
extern const float DRV_Gain;
extern const float T_vel;
extern const float LPF_ALpha_Vel;
extern const float LPF_Beta_Vel;
extern const float LPF_U_Alpha;
extern const float LPF_U_Beta;
extern const float MI_PI2;
extern const int encoder_cali_shift;

#define N_Offset_Section 128
#define N_Offset_Section_2 64
// this mode: max rotor speed 85rad/s
// ATTENTION Different from Motors

// Parameters related to pi controller
// WARNING parts following Only modify by LW
// adc 12bits: 3+12 = 15 adcclocks, APB2:90Mhz, pclk/4 = 22,500,000Hz, t_c = 44.44ns
// sampling time: 15*44.44 = 666.6666666666667ns;
// counterfrequency: 180Mhz, T_counter = 5.555555555555556ns;
// adc counts: 666.67/5.56= 120counts, set sampling point:Fpwm - 120/2 = 2189;
// 实际上有漂移，需要手动矫正
#define Sampling_Point 2100 // 取中点采样 2125
#define Fpwm 0x109A // 4250
#define Timer_ARR 0x84D // Fpwm/2 2125
#define T_PWM 0.000025f
#define Inverse_T_Pwm 40000


void init_config(void);

#endif