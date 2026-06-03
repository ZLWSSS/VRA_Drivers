#include "config.h"

const int MT6835_Resolution = 2097152;
const int Rotor_Encoder_Half_CPR = MT6835_Resolution / 2;
const float R_sense = 0.002f;
const float DRV_Gain = 20;
const float T_vel = 0.00025;
const float LPF_ALpha_Vel = 0.13575524816;
const float LPF_Beta_Vel = 0.864244751836;
const float LPF_U_Alpha = 0.611;
const float LPF_U_Beta =  0.389;
const float MI_PI2 = 6.283185307179586;
const int encoder_cali_shift = 21 - 7; //21: the resolution of encoders.
