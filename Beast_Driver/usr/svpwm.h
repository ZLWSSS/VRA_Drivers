#ifndef PWM_HANDLER_H_
#define PWM_HANDLER_H_

#include "types.h"

typedef struct SVPWM_Handler SVPWM_Handler_t;
typedef void (*Get_Phase_Current)(void);
typedef void (*SVPWM)(qd_f_t *v_dq, theta_s_c_t *csin, float vbus);
typedef void (*Reset_pwm)(void);
typedef void (*Set_Occupation)(void);
typedef void (*Start_ADC_Sampling)(void);
typedef void (*Stop_ADC_Sampling)(void);
typedef void (*Zero_Current)(void);
typedef void (*Get_Error)(theta_s_c_t *csin);

struct SVPWM_Handler
{
    uint32_t CntPHA;
    uint32_t CntPHB;
    uint32_t CntPHC;
    uint32_t CntSampling;
    abc_f_t I_abc;
    qd_f_t I_dq;
    qd_f_t I_dq_ref;
    qd_f_t I_dq_error;
    float I_dq_ref_norm;

    int32_t adc_raw_PHA;
    int32_t adc_raw_PHB;
    int32_t adc_raw_PHC;
    float adc_offset_PHA;
    float adc_offset_PHB;
    float adc_offset_PHC;
    uint32_t PhaseAOffset;
    uint32_t PhaseBOffset;
    uint32_t PhaseCOffset;
    float I_Scale;
    uint8_t Sector;
    uint32_t phase_order;
    Get_Phase_Current pfct_get_phase_current;
    SVPWM pfct_svm;
    Reset_pwm pfct_reset_pwm;
    Set_Occupation pfct_set_occupation;
    Start_ADC_Sampling pfct_start_adc_sampling;
    Stop_ADC_Sampling pfct_stop_adc_sampling;
    Zero_Current pfct_zero_current;
    Get_Error pfct_trans_get_error;
};

void SVPWM_Handler_Init(void);
SVPWM_Handler_t *get_svpwm_handler(void);
#endif