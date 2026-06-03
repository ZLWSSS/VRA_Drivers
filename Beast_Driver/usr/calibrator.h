#ifndef CALIBRATOR_H_
#define CALIBRATOR_H_

#include "encoders.h"
#include "svpwm.h"
#include "types.h"
#include "config.h"

#define N_Vec_Size 2688             //(N_Offset_Section * Npp)
#define Delta_Angle 0.001227184609f // 2*PI*NPP/(N_Vec_Size * N_Increment_Sample)
#define N_Increment_Sample 40
#define V_cal 0.15f
#define V_bus_cal 2.f
#define ANTI_COGGING_SIZE 512

typedef struct Calibrator_Handler Calibrator_Handler_t;
typedef void (*Phase_Check)(SVPWM_Handler_t *svpwm_handle, Encoders_Handle_t *encoders_handle);

struct Calibrator_Handler
{
    int offset_lut[N_Offset_Section];
    uint16_t flg_calibrated;
    uint16_t flg_anti_cogging_calibrated;
    uint16_t flg_anti_sampler;
    int32_t flg_anti_calicounter;
    Phase_Check pfct_phase_check;
};

void Calibrator_Init(void);
void calibrate(SVPWM_Handler_t *svpwm_handle, Encoders_Handle_t *encoders_handle);
Calibrator_Handler_t *get_cali_handler(void);
#endif
