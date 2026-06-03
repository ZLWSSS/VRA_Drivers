#include "status_monitor.h"
#include "adc.h"
#include "gpio.h"
#include "motor_config.h"
#include "usr_delay.h"
#include "math.h"

Board_Status_t board_status_;

static void ADC_Set_Channel(void)
{
    // direct the channel to 12
    ADC_ChannelConfTypeDef sConfig = {0};
    // adc1
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    // adc2
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

float TS_CAL1 = 0; // always 1045
float TS_CAL2 = 0; // always 1378
float TS_CAL1_Temp = 30;
float TS_CAL2_Temp = 130;
// cost about 6us, 23.3us worst because of interrupt
static void Update_Board_Status(void)
{
    LL_ADC_REG_StartConversion(ADC1);
    while (!LL_ADC_IsActiveFlag_EOC(ADC1))
    {
    }
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_EOC(ADC2);
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_ClearFlag_EOS(ADC2);
    board_status_.temp_value = LL_ADC_REG_ReadConversionData12(ADC1);
    board_status_.v_value = LL_ADC_REG_ReadConversionData12(ADC2);

    board_status_.V_bus_ = (float)board_status_.v_value * board_status_.voltage_vrefint_portion * 25.f; // 25 /10
    board_status_.V_ins_circle_ =
        board_status_.V_bus_ * 1.15f; // inner circle has already counted in svpwm transformation
    board_status_.V_limit_ = board_status_.V_bus_ * 0.61747611289305f; // take 93% voltage. 2/3 * (sqrt(3)/2) * 1.15

    // TODO add temperature
    float voltage = (float) board_status_.temp_value * board_status_.voltage_vrefint_portion;
    board_status_.R_value = (10 * voltage)/(3.3-voltage);
    board_status_.Temp_coil_ = 1.f/(0.0033540164f - log(10.f/board_status_.R_value)/3950.f) -273.15f;
    // STM32 board temperature: (T_CAL1 - T_CAL2)/(CAL1-CAL2) * (DATA * 3.3 / 3 - CAL1) + T_CAL1
    board_status_.board_temp_value = 0;

    HAL_ADC_Start(&hadc5);
    if (HAL_OK == HAL_ADC_PollForConversion(&hadc5, 100))
    {
        board_status_.board_temp_value += (uint16_t)HAL_ADC_GetValue(&hadc5);
    }

    board_status_.board_temp_ = 0.3003f * ((float)board_status_.board_temp_value * 1.1f - TS_CAL1) + TS_CAL1_Temp;
}

static void show_can_id(void)
{
    int id_indi = CAN_ID;

    while (id_indi > 0)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        delay_ms(500);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        delay_ms(500);
        id_indi--;
    }
}

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;

    HAL_ADC_Start(ADCx);
    if (HAL_OK == HAL_ADC_PollForConversion(ADCx, 100))
        return (uint16_t)HAL_ADC_GetValue(ADCx);
    else
        return 0;
}

// Get V_portion and V_cc

void Init_Board_Status(void)
{
    int i = 0;
    int vrefint_adc = 0;
    for (i = 0; i < 100; i++)
    {
        vrefint_adc += adcx_get_chx_value(&hadc3, ADC_CHANNEL_VREFINT);
        delay_ms(1);
    }
    vrefint_adc = (float)(vrefint_adc) / 100.f;
    //__HAL_ADC_CALC_VREFANALOG_VOLTAGE
    int32_t v_ref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE((int)vrefint_adc, ADC_RESOLUTION_12B);
    board_status_.voltage_vrefint_portion = (float)v_ref / 4096000.f;
    int total_vcc = 0;
    for (i = 0; i < 100; i++)
    {
        total_vcc += adcx_get_chx_value(&hadc3, ADC_CHANNEL_VBAT);
    }
    board_status_.V_CC = 3.f * (float)total_vcc / 100.f * board_status_.voltage_vrefint_portion * 0.90901f;
    board_status_.pfct_set_channel = ADC_Set_Channel;
    board_status_.pfct_show_id = show_can_id;
    board_status_.pfct_update_board = Update_Board_Status;

    TS_CAL1 = *(__IO uint16_t *)(0x1FFF75A8);
    TS_CAL2 = *(__IO uint16_t *)(0x1FFF75CA);
}

float get_vreint_portion(void)
{
    return board_status_.voltage_vrefint_portion;
}

Board_Status_t *get_board_status_handle(void)
{
    return &board_status_;
}