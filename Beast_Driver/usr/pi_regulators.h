#ifndef PID_Handler_H_
#define PID_Handler_H_

#include "types.h"

typedef struct Current_PI_Handler Cur_PI_Handler_t;
typedef void (*Cur_PI_Run)(qd_f_t *iqd_error, qd_f_t *BMF, float vlimit);
typedef void (*Cur_Set_Ka)(float value);
typedef void (*Cur_Set_Kb)(float value);
typedef void (*Cur_PI_Reset)(void);

typedef struct Vel_PI_Handler Vel_PI_Handler_t;
typedef void (*Vel_PI_Run)(float fb_vel_value);
typedef void (*Vel_Set_Ka)(float value);
typedef void (*Vel_Set_Kb)(float value);
typedef void (*Vel_PI_Reset)(void);

struct Current_PI_Handler
{
    qd_f_t Vqd_out;
    qd_f_t Ka_out;
    float Ka;
    float Kb;
    float Kb_const;
    qd_f_t Kb_out;
    Cur_PI_Run pfct_cur_run;
    Cur_Set_Ka pfct_cur_setka;
    Cur_Set_Kb pfct_cur_setkb;
    Cur_PI_Reset pfct_reset_cur;
};

struct Vel_PI_Handler
{
    float vel_ref;
    float vel_last_ref;
    float vel_ramp_ref;
    float vel_ramp_point;
    float Ka;
    float Kb;
    float Kb_const;
    float iq_ref;
    float Ka_out;
    float Kb_out;
    uint8_t use_anti_cog;
    float error;
    int ramp_counter;
    Control_Mode_e control_mode;
    Vel_PI_Run pfct_vel_run;
    Vel_Set_Ka pfct_vel_setka;
    Vel_Set_Kb pfct_vel_setkb;
    Vel_PI_Reset pfct_vel_reset;
};

typedef struct P_Pos_Handler
{
    float p_ref_target;
    float p_ref_out;
} P_Pos_Handler_t;

void Current_PI_Handler_Init(void);
void Vel_PI_Handler_Init(void);
Cur_PI_Handler_t *get_cur_pi_handler(void);
Vel_PI_Handler_t *get_vel_pi_handler(void);

#endif