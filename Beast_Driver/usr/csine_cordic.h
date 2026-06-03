#ifndef CSINE_CORDIC_H_
#define CSINE_CORDIC_H_

#include "stm32g4xx_ll_cordic.h"
#include "types.h"

typedef void (*get_cordic_result)(float *etheta);

typedef struct Cordic_Handle
{
    int cordic_etheta_;
    int cordic_cos_;
    int cordic_sin_;
    theta_s_c_t cordic_output_;
    get_cordic_result pfct_get_csine;
} cordic_t;

void Cordic_Handle_Init(void);
cordic_t *get_cordic_handle(void);

#endif