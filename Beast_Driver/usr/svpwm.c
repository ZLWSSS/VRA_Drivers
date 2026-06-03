#include "svpwm.h"
#include "adc.h"
#include "config.h"
#include "flash_writer.h"
#include "motor_config.h"
#include "status_monitor.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_tim.h"
#include "usr_delay.h"

SVPWM_Handler_t svpwm_handle_;

// help calculating ************************************************************
float para1, para2, para3;
float c_helper1 = 0, c_helper2 = 0, c_helper3 = 0;                 // help for removing redundant calculation
float x = 0, y = 0, z = 0, T1 = 0, T2 = 0, ta = 0, tb = 0, tc = 0; // occupation time
float T1_T2_sum = 0;
int CntPha = 0, CntPhb = 0, CntPhc = 0;
// ***************************************************************************

static void zero_current(void)
{
    // return adc1 rank back to make sure the right channel for zero current
    // the ADC2 is regular rank by default
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    int adc1_offset = 0;
    int adc2_offset = 0;
    for (int i = 0; i < 1024; i++)
    {
        // open all lowside mosfet(close all high side mosfet. all value > ccr)
        svpwm_handle_.CntPHA = (Fpwm >> 1) * 0.f;
        svpwm_handle_.CntPHB = (Fpwm >> 1) * 0.f;
        svpwm_handle_.CntPHC = (Fpwm >> 1) * 0.f;
        svpwm_handle_.pfct_set_occupation();

        LL_ADC_REG_StartConversion(ADC1);
        // LL_ADC_REG_StartConversion(ADC2);
        while (!LL_ADC_IsActiveFlag_EOC(ADC1))
        {
        }
        // while (!LL_ADC_IsActiveFlag_EOC(ADC2))
        //{
        // }
        LL_ADC_ClearFlag_EOC(ADC1);
        LL_ADC_ClearFlag_EOC(ADC2);
        LL_ADC_ClearFlag_EOS(ADC1);
        LL_ADC_ClearFlag_EOS(ADC2);
        adc1_offset += LL_ADC_REG_ReadConversionData12(ADC1);
        adc2_offset += LL_ADC_REG_ReadConversionData12(ADC2);
        delay_ms(1);
    }
    svpwm_handle_.adc_offset_PHC = (float)adc1_offset / (float)1024;
    svpwm_handle_.adc_offset_PHB = (float)adc2_offset / (float)1024;
}

// TODO add sampling point;
static void set_occupation(void)
{
    // LL_TIM_OC_SetCompareCH1(TIM1, svpwm_handle_.CntPHC);
    // LL_TIM_OC_SetCompareCH2(TIM1, svpwm_handle_.CntPHB);
    // LL_TIM_OC_SetCompareCH3(TIM1, svpwm_handle_.CntPHA);
    LL_TIM_OC_SetCompareCH1(TIM1, svpwm_handle_.CntPHA);
    LL_TIM_OC_SetCompareCH2(TIM1, svpwm_handle_.CntPHB);
    LL_TIM_OC_SetCompareCH3(TIM1, svpwm_handle_.CntPHC);
}

static void pwm_trans_get_error(theta_s_c_t *csin)
{
    // TODO alternate with cordic
    // clark + park transform
    // check these two equation:: Correct;
    svpwm_handle_.I_dq.d = 0.6666667f * (csin->cos_ * svpwm_handle_.I_abc.a +
                                         (0.86602540378f * csin->sin_ - 0.5f * csin->cos_) * svpwm_handle_.I_abc.b +
                                         (-0.86602540378f * csin->sin_ - 0.5f * csin->cos_) * svpwm_handle_.I_abc.c);
    svpwm_handle_.I_dq.q = 0.6666667f * (-csin->sin_ * svpwm_handle_.I_abc.a -
                                         (-0.86602540378f * csin->cos_ - 0.5f * csin->sin_) * svpwm_handle_.I_abc.b -
                                         (0.86602540378f * csin->cos_ - 0.5f * csin->sin_) * svpwm_handle_.I_abc.c);

    svpwm_handle_.I_dq_error.d = svpwm_handle_.I_dq_ref.d - svpwm_handle_.I_dq.d;
    svpwm_handle_.I_dq_error.q = svpwm_handle_.I_dq_ref.q - svpwm_handle_.I_dq.q;
    // NOTE delete I_MAX protection, no meaning because of the voltage saturation
    if (svpwm_handle_.I_dq_error.q >= di_MAX)
    {
        svpwm_handle_.I_dq_error.q = di_MAX;
    }
    else if (svpwm_handle_.I_dq_error.q < -di_MAX)
    {
        svpwm_handle_.I_dq_error.q = -di_MAX;
    }
}
// get position and vel before this func.
// return I_dq_error
static void pwm_get_phase_current(void)
{
    svpwm_handle_.adc_raw_PHB = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
    svpwm_handle_.adc_raw_PHC = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    // convert adc value to voltage
    svpwm_handle_.I_abc.b = (svpwm_handle_.I_Scale) * (float)(svpwm_handle_.adc_raw_PHB - svpwm_handle_.adc_offset_PHB);
    svpwm_handle_.I_abc.c = (svpwm_handle_.I_Scale) * (float)(svpwm_handle_.adc_raw_PHC - svpwm_handle_.adc_offset_PHC);
    svpwm_handle_.I_abc.a = -svpwm_handle_.I_abc.c - svpwm_handle_.I_abc.b;
}

static void svm(qd_f_t *v_dq, theta_s_c_t *csin, float vbus)
{
    // reverse clark
    float v_alpha = v_dq->d * csin->cos_ - v_dq->q * csin->sin_;
    float v_beta = v_dq->d * csin->sin_ + v_dq->q * csin->cos_;
    c_helper1 = 1.7320508f * v_alpha;
    c_helper2 = v_beta * 0.8660254f;
    c_helper3 = Fpwm / (vbus);

    svpwm_handle_.Sector = 0;
    para1 = v_beta;
    para2 = (c_helper1 - v_beta) * 0.5f;
    para3 = (-c_helper1 - v_beta) * 0.5f;

    if (para1 > 0)
    {
        svpwm_handle_.Sector = 1;
    }
    if (para2 > 0)
    {
        svpwm_handle_.Sector += 2;
    }
    if (para3 > 0)
    {
        svpwm_handle_.Sector += 4;
    }

    x = 1.7320508f * v_beta * c_helper3;
    y = c_helper3 * (1.5f * v_alpha + c_helper2);
    z = c_helper3 * (-1.5f * v_alpha + c_helper2);

    switch (svpwm_handle_.Sector)
    {
    case 1: {
        T1 = z;
        T2 = y;
        break;
    }
    case 2: {
        T1 = y;
        T2 = -x;
        break;
    }
    case 3: {
        T1 = -z;
        T2 = x;
        break;
    }
    case 4: {
        T1 = -x;
        T2 = z;
        break;
    }
    case 5: {
        T1 = x;
        T2 = -y;
        break;
    }
    case 6: {
        T1 = -y;
        T2 = -z;
        break;
    }
    }

    T1_T2_sum = T1 + T2;
    // if (T1_T2_sum > Fpwm)
    //{
    //     T1 = Fpwm * T1 / (T1_T2_sum);
    //     T2 = Fpwm * T2 / (T1_T2_sum);
    // }

    ta = (Fpwm - (T1_T2_sum)) * 0.25f;
    tb = ta + T1 * 0.5f;
    tc = tb + T2 * 0.5f;

    // revert time
    CntPha = Timer_ARR - (int)ta;
    CntPhb = Timer_ARR - (int)tb;
    CntPhc = Timer_ARR - (int)tc;

    switch (svpwm_handle_.Sector)
    {
    case 1: {
        svpwm_handle_.CntPHA = CntPhb;
        svpwm_handle_.CntPHB = CntPha;
        svpwm_handle_.CntPHC = CntPhc;
        break;
    }
    case 2: {
        svpwm_handle_.CntPHA = CntPha;
        svpwm_handle_.CntPHB = CntPhc;
        svpwm_handle_.CntPHC = CntPhb;
        break;
    }
    case 3: {
        svpwm_handle_.CntPHA = CntPha;
        svpwm_handle_.CntPHB = CntPhb;
        svpwm_handle_.CntPHC = CntPhc;
        break;
    }
    case 4: {
        svpwm_handle_.CntPHA = CntPhc;
        svpwm_handle_.CntPHB = CntPhb;
        svpwm_handle_.CntPHC = CntPha;
        break;
    }
    case 5: {
        svpwm_handle_.CntPHA = CntPhc;
        svpwm_handle_.CntPHB = CntPha;
        svpwm_handle_.CntPHC = CntPhb;
        break;
    }
    case 6: {
        svpwm_handle_.CntPHA = CntPhb;
        svpwm_handle_.CntPHB = CntPhc;
        svpwm_handle_.CntPHC = CntPha;
        break;
    }
    }
}

// func work
// tickle period: 2125, middle level: 1062
static void reset_pwm(void)
{
    // 最大值乘以1/4
    svpwm_handle_.CntPHA = (Fpwm >> 1) * 0.5f;
    svpwm_handle_.CntPHB = (Fpwm >> 1) * 0.5f;
    svpwm_handle_.CntPHC = (Fpwm >> 1) * 0.5f;
    svpwm_handle_.I_dq_ref.d = svpwm_handle_.I_dq_ref.q = 0;
}

static void start_adc_trisample(void)
{
    LL_TIM_OC_SetCompareCH4(TIM1, Sampling_Point);
    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    // reset adc perphiral
    LL_ADC_ClearFlag_JEOC(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOC(ADC2);
    LL_ADC_ClearFlag_JEOS(ADC2);

    // simultaneous ADC1 + ADC2, ADC3 independent
    LL_ADC_EnableIT_JEOC(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);
}

static void stop_adc_trisample(void)
{
    LL_TIM_OC_SetCompareCH4(TIM1, Sampling_Point);
    // reset adc perphiral
    LL_ADC_ClearFlag_JEOC(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_ClearFlag_JEOC(ADC2);
    LL_ADC_ClearFlag_JEOS(ADC2);
    LL_ADC_DisableIT_JEOC(ADC1);
}

void SVPWM_Handler_Init(void)
{
    svpwm_handle_.pfct_svm = svm;
    svpwm_handle_.pfct_get_phase_current = pwm_get_phase_current;
    svpwm_handle_.pfct_reset_pwm = reset_pwm;
    svpwm_handle_.pfct_set_occupation = set_occupation;
    svpwm_handle_.pfct_start_adc_sampling = start_adc_trisample;
    svpwm_handle_.pfct_stop_adc_sampling = stop_adc_trisample;
    svpwm_handle_.pfct_zero_current = zero_current;
    svpwm_handle_.pfct_trans_get_error = pwm_trans_get_error;
    svpwm_handle_.pfct_reset_pwm();

    // N_Offset_Section_2 = 64
    flash_read((ADDR_FLASH_SECTOR_1 + (64 + 1) * 64), (uint32_t *)&(svpwm_handle_.phase_order), 1);
    if ((svpwm_handle_.phase_order != 0) && (svpwm_handle_.phase_order != 1))
    {
        svpwm_handle_.phase_order = 0;
    }

    float voltage_vrefint_proportion = get_vreint_portion();
    svpwm_handle_.I_Scale = voltage_vrefint_proportion / (R_sense * DRV_Gain);

    // disable the current loop
    LL_ADC_DisableIT_JEOC(ADC1);
    LL_ADC_DisableIT_JEOC(ADC2);
}

SVPWM_Handler_t *get_svpwm_handler(void)
{
    return &svpwm_handle_;
}
