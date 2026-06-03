#ifndef AS5047_HANDLER_H_
#define AS5047_HANDLER_H_

#include "spi.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_spi.h"
#define MT6835_angle 0xA003 // with dynamic error offset
#define MT6835_nop 0x0000

typedef struct Encoders_Handle Encoders_Handle_t;
typedef void (*Set_Offset_Ele)(float value);
typedef void (*FOC_Encoder_Sample)(int *lut_offset);
typedef float (*Get_Rotor_Pos)(int *lut_offset);
typedef uint32_t (*Get_Rotor_Encoder_Raw_Data)(void);
typedef void (*Set_Zero)(int *lut_offset);

struct Encoders_Handle
{
    int32_t fl_sample_data[3];
    int32_t rotor_CPR;
    int32_t half_rotor_CPR;
    int32_t rotor_pos_cpr_last;
    int32_t rotor_pos_cpr;
    int32_t loop_count;
    float pos_ele_;
    float pos_flange_;
    float rotor_pos_fl_;
    float rotor_pos_old_fl_;
    float rotor_vel_;
    float vel_ele_;
    float flange_vel_;
    float flange_vel_last_;
    float flange_acc_;
    float offset_ele; // calibration offset
    float offset_me;
    float offset_fl2rotor;
    int32_t offset_flange;
    int32_t flange_pos;
    int32_t flange_ini;
    uint8_t flg_zero;

    Set_Offset_Ele pfct_set_offset_ele;
    FOC_Encoder_Sample pfct_foc_sample;
    Get_Rotor_Pos pfct_get_rotor_pos;
    Get_Rotor_Encoder_Raw_Data pfct_get_rotor_encoder_raw_Data;
    Set_Zero pfct_set_zero;
};

void Encoders_Handler_Init(void);
Encoders_Handle_t *get_encoder_handler(void);
#endif