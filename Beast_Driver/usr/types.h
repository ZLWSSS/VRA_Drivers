#ifndef TYPES_H_
#define TYPES_H_

#include "stdint.h"

typedef enum Control_Mode
{
    Idle_Mode = 0,
    Hybrid_MODE,
    Vel_Mode,
    Position_Mode,
    Anti_Cogging_Mode,
} Control_Mode_e;

typedef struct
{
    int32_t q;
    int32_t d;
} qd_t;

typedef struct
{
    float q;
    float d;
} qd_f_t;

// abc frame
typedef struct
{
    int32_t a;
    int32_t b;
    int32_t c;
} abc_t;

typedef struct
{
    float a;
    float b;
    float c;
} abc_f_t;

typedef struct
{
    int32_t alpha;
    int32_t beta;
} alphabeta_t;

typedef struct
{
    float alpha;
    float beta;
} alphabeta_f_t;

typedef struct
{
    float sin_;
    float cos_;
} theta_s_c_t;

#endif