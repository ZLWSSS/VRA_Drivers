#include "fsm.h"
#include "adc.h"
#include "calibrator.h"
#include "can_handler.h"
#include "cordic.h"
#include "csine_cordic.h"
#include "dac.h"
#include "dma.h"
#include "drv.h"
#include "encoders.h"
#include "error_status.h"
#include "gpio.h"
#include "math.h"
#include "pi_regulators.h"
#include "status_monitor.h"
#include "string.h"
#include "svpwm.h"
#include "tim.h"
#include "usr_delay.h"

CAN_Handler_t *fsm_can_h;
SVPWM_Handler_t *fsm_svpwm_h;
Drv_Handler_t *fsm_drv_h;
Calibrator_Handler_t *fsm_cali_h;
Encoders_Handle_t *fsm_encoders_h;
Cur_PI_Handler_t *fsm_cpi_h;
Vel_PI_Handler_t *fsm_vpi_h;
cordic_t *fsm_cordic_h;
Board_Status_t *fsm_board_h;

FSM_State_t idle_state, cali_state, zero_state, hybrid_ctrl, error_state;
FSM_State_t *current_state;
volatile uint8_t foc_flag = 0;
volatile uint8_t sample_encoder = 0;
uint8_t initialized = 0;

// **********************************************************************************************************************
//                                     States Function
// **********************************************************************************************************************
void ram_fsm(void)
{
    if (current_state->next_state != current_state)
    {
        current_state->pfct_state_exit();
        current_state = current_state->next_state;
        current_state->next_state = current_state;
        current_state->pfct_state_enter();
    }
    else
    {
        current_state->pfct_state_run();
    }
}

// id_state functions
static void idle_state_enter(void)
{
    fsm_svpwm_h->pfct_reset_pwm();
    fsm_cpi_h->pfct_reset_cur();
    fsm_can_h->pfct_can_reset_cmd();
    fsm_drv_h->pfct_drv_disable();
    sample_encoder = 1;
    foc_flag = 0;
}

static void idle_state_exit(void)
{
    LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
}

static void idle_state_run(void)
{
    LL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    delay_ms(500);
}

// cali_state functions
static void cali_state_enter(void)
{
    LL_ADC_DisableIT_JEOC(ADC1);
    HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    fsm_svpwm_h->pfct_reset_pwm();
    fsm_svpwm_h->pfct_stop_adc_sampling();
    fsm_cpi_h->pfct_reset_cur();
    fsm_can_h->pfct_can_reset_cmd();
    fsm_drv_h->pfct_drv_disable();
    delay_ms(100);
    sample_encoder = 0;
    foc_flag = 0;
    memset(fsm_cali_h->offset_lut, 0, sizeof(int) * N_Offset_Section);
    fsm_encoders_h->offset_ele = 0;
    fsm_encoders_h->loop_count = 0;
    fsm_encoders_h->offset_me = 0;
}

static void cali_state_exit(void)
{
    fsm_drv_h->pfct_drv_disable();
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

static void cali_state_run(void)
{
    fsm_drv_h->pfct_drv_enable();
    fsm_cali_h->pfct_phase_check(fsm_svpwm_h, fsm_encoders_h);
    calibrate(fsm_svpwm_h, fsm_encoders_h);
    current_state->next_state = &idle_state;
}

// zero_state functions
static void zero_state_enter(void)
{
    sample_encoder = 0;
    foc_flag = 0;
    LL_ADC_DisableIT_JEOC(ADC1);
    fsm_svpwm_h->pfct_reset_pwm();
    fsm_cpi_h->pfct_reset_cur();
    fsm_can_h->pfct_can_reset_cmd();
    fsm_drv_h->pfct_drv_disable();
    HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

static void zero_state_exit(void)
{
    sample_encoder = 1;
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

static void zero_state_run(void)
{
    fsm_encoders_h->pfct_set_zero(fsm_cali_h->offset_lut);
    for (int i = 0; i < 10; i++)
    {
        LL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        delay_ms(100);
    }
    LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);

    current_state->next_state = &idle_state;
}
// hybrid control
static void hybrid_state_enter(void)
{
    LL_ADC_DisableIT_JEOC(ADC1);
    fsm_svpwm_h->pfct_reset_pwm();
    fsm_cpi_h->pfct_reset_cur();
    fsm_can_h->pfct_can_reset_cmd();
    if (fsm_drv_h->DRV_Status)
    {
        fsm_drv_h->pfct_drv_enable();

        if (fsm_cali_h->flg_calibrated) // zero is not necessary but cali compulsory
        {
            foc_flag = 1;
            sample_encoder = 1;
            fsm_vpi_h->control_mode = Hybrid_MODE;
            fsm_svpwm_h->pfct_start_adc_sampling();
        }
        else
        {
            Add_Error(Cali_Error);
            Add_Error(Zero_Positon_Error);
            current_state->next_state = &error_state;
        }
    }
    else
    {
        current_state->next_state = &error_state;
    }
}

static void hybrid_state_exit(void)
{

    LL_ADC_DisableIT_JEOC(ADC1);
    fsm_svpwm_h->pfct_reset_pwm();
    fsm_cpi_h->pfct_reset_cur();
    fsm_can_h->pfct_can_reset_cmd();
    fsm_drv_h->pfct_drv_disable();
    foc_flag = 0;
    sample_encoder = 0;
}

static void hybrid_state_run(void)
{
}

// error status

static void error_state_enter(void)
{
    delay_us(100);
    LL_GPIO_SetOutputPin(LED_R_GPIO_Port, LED_R_Pin);
    LL_GPIO_SetOutputPin(LED_G_GPIO_Port, LED_G_Pin);
    LL_GPIO_SetOutputPin(LED_B_GPIO_Port, LED_B_Pin);
    fsm_drv_h->pfct_drv_check();
    if (HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) == GPIO_PIN_RESET)
    {
        Add_Error(DRV_PROTECT_ERROR);
        foc_flag = 0;
        sample_encoder = 0;
        LL_ADC_DisableIT_JEOC(ADC1);
        fsm_svpwm_h->pfct_reset_pwm();
        fsm_cpi_h->pfct_reset_cur();
        fsm_can_h->pfct_can_reset_cmd();
        fsm_drv_h->pfct_drv_disable();
        fsm_drv_h->pfct_drv_check();
        HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        // sample_enc = 0;
    }
    else
    {
        // current_state->next_state = &idle_state;
    }
}

static void error_state_exit(void)
{
}

static void error_state_run(void)
{
    LL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    delay_ms(500);
}

// **********************************************************************************************************************
//                                     States Function End
// **********************************************************************************************************************

// **********************************************************************************************************************
//                                     Interrupt Functions
// **********************************************************************************************************************
qd_f_t bemf;
uint16_t __attribute((section(".CCM_RAM_Data"))) dac_data; // 4096 for 50.f iq
void current_loop(void)
{

    // before foc_flag: 1.7us
    fsm_svpwm_h->pfct_get_phase_current();
    fsm_cordic_h->pfct_get_csine(&(fsm_encoders_h->pos_ele_));
    fsm_svpwm_h->pfct_trans_get_error(&fsm_cordic_h->cordic_output_);

#ifdef DAC_OUT
    //  TODO Mind here!
    dac_data = (uint16_t)((fsm_svpwm_h->I_dq_error.q) * 0.02f * 2048.f + 2048);
#endif
    //  bracket block: 3.6us
    //  LL_GPIO_SetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
    if (foc_flag)
    {
        bemf.q = fsm_encoders_h->vel_ele_ * (phi_m + L_s * fsm_svpwm_h->I_dq.d);
        bemf.d = -fsm_encoders_h->vel_ele_ * fsm_svpwm_h->I_dq.q * L_s;
        fsm_cpi_h->pfct_cur_run(&fsm_svpwm_h->I_dq_error, &bemf, fsm_board_h->V_limit_);
        fsm_svpwm_h->pfct_svm(&fsm_cpi_h->Vqd_out, &fsm_cordic_h->cordic_output_, fsm_board_h->V_ins_circle_);
        fsm_svpwm_h->pfct_set_occupation();
    }
    // LL_GPIO_ResetOutputPin(Test_Pin_1_GPIO_Port, Test_Pin_1_Pin);
}

// encoder: 8.8us
// uint32_t test_raw_data = 0;
// float test_data = 0;
void Sample_Encoder(void)
{
    if (sample_encoder)
    {
        fsm_encoders_h->pfct_foc_sample(fsm_cali_h->offset_lut);
        // test_raw_data = fsm_encoders_h->pfct_get_rotor_encoder_raw_Data();
        // test_data = fsm_encoders_h->pfct_get_rotor_pos(fsm_cali_h->offset_lut);
    }
}

void iq_cmd_update_loop(void)
{
    if (foc_flag)
    {
        switch (fsm_vpi_h->control_mode)
        {
        case Hybrid_MODE: {
            fsm_svpwm_h->I_dq_ref.q = fsm_can_h->cmd_kp * (fsm_can_h->cmd_p_target - fsm_encoders_h->pos_flange_) +
                                      fsm_can_h->cmd_kd * (fsm_can_h->cmd_v_target - fsm_encoders_h->flange_vel_) +
                                      fsm_can_h->cmd_t_target;
            fsm_svpwm_h->I_dq_ref.q *= Inverse_KT_Out; //;
            fsm_svpwm_h->I_dq_ref.d = 0;
            break;
        }
        case Idle_Mode: {
            fsm_svpwm_h->I_dq_ref.q = 0;
            fsm_svpwm_h->I_dq_ref.d = 0;
            break;
        }

        default:
            break;
        }
    }
}

volatile uint8_t can_fresh_flag;
FDCAN_RxHeaderTypeDef rx_header;
void can_message_pending(void)
{
    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, fsm_can_h->cmd_buf);
    can_fresh_flag = 1;
    // 遥控帧与数据帧
    switch (rx_header.Identifier)
    {
    case CAN_ID:
        fsm_can_h->pfct_get_control_cmd(fsm_can_h->cmd_buf);
        // not add gear ratio yet
        fsm_can_h->pfct_reply_data(fsm_encoders_h->pos_flange_, fsm_encoders_h->flange_vel_,
                                   fsm_svpwm_h->I_dq.q * KT_Out, fsm_cpi_h->Vqd_out.q, fsm_cpi_h->Vqd_out.d);
        break;
    case Require_Status_ID:
        fsm_can_h->pfct_reply_data(fsm_encoders_h->pos_flange_, fsm_encoders_h->flange_vel_,
                                   fsm_svpwm_h->I_dq.q * KT_Out, fsm_cpi_h->Vqd_out.q, fsm_cpi_h->Vqd_out.d);
        break;
    case DISABLE_ID:
        current_state->next_state = &idle_state;
        break;
    case CALIBRATION_ID:
        current_state->next_state = &cali_state;
        break;
    case En_MIT_MODE_ID:
        current_state->next_state = &hybrid_ctrl;
        break;
    case ZERO_POSITION_ID:
        current_state->next_state = &zero_state;
        break;
    default:
        break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == DRV_NFAULT_Pin)
    {
        current_state->next_state = &error_state;
    }
}

volatile uint32_t fresh_time;
float timer_damping = 0;
void Cmd_Fresh_Check(uint32_t timer)
{
    if (can_fresh_flag && initialized)
    {
        fresh_time = timer;
        can_fresh_flag = 0;
        timer_damping = 0;
    }
    uint32_t delta_time = timer - fresh_time;

    if ((!can_fresh_flag) && (delta_time > 2000) && (current_state != &idle_state) && (current_state != &error_state))
    {
        current_state->next_state = &idle_state;
    }
    else if ((!can_fresh_flag) && (delta_time > 100))
    {
        fsm_can_h->pfct_can_reset_cmd();
        timer_damping++;
        timer_damping = (timer_damping > 1000) ? 1000 : timer_damping;
        fsm_can_h->cmd_kd = 0.003f * timer_damping;
    }
}

void update_voltage_temperature(void)
{
    // LL_GPIO_SetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
    fsm_board_h->pfct_update_board();
    // LL_GPIO_ResetOutputPin(Test_Pin_2_GPIO_Port, Test_Pin_2_Pin);
}
// **********************************************************************************************************************
//                                     Interrupt Functions End
// **********************************************************************************************************************

void init_fsm(void)
{
    // calibrate adcs before init board status
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED))
    {
        Add_Error(ADC1_CALIB_ERROR);
        Error_Handler();
    }
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))
    {
        Add_Error(ADC2_CALIB_ERROR);
        Error_Handler();
    }
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED))
    {
        Add_Error(ADC3_CALIB_ERROR);
        Error_Handler();
    }
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED))
    {
        Add_Error(ADC5_CALIB_ERROR);
        Error_Handler();
    }

    delay_init();
    Calibrator_Init();
    Can_Handler_Init();
    Cordic_Handle_Init();
    Drv_Init();
    Encoders_Handler_Init();
    Current_PI_Handler_Init();
    Vel_PI_Handler_Init();
    Init_Board_Status();
    SVPWM_Handler_Init();

    fsm_can_h = get_can_handler_t();
    fsm_cali_h = get_cali_handler();
    fsm_svpwm_h = get_svpwm_handler();
    fsm_drv_h = get_drv_handler();
    fsm_encoders_h = get_encoder_handler();
    fsm_cpi_h = get_cur_pi_handler();
    fsm_vpi_h = get_vel_pi_handler();
    fsm_cordic_h = get_cordic_handle();
    fsm_board_h = get_board_status_handle();

    // initialize states
    idle_state.next_state = NULL;
    idle_state.pfct_state_enter = idle_state_enter;
    idle_state.pfct_state_exit = idle_state_exit;
    idle_state.pfct_state_run = idle_state_run;

    cali_state.next_state = NULL;
    cali_state.pfct_state_enter = cali_state_enter;
    cali_state.pfct_state_exit = cali_state_exit;
    cali_state.pfct_state_run = cali_state_run;

    zero_state.pfct_state_enter = zero_state_enter;
    zero_state.pfct_state_run = zero_state_run;
    zero_state.pfct_state_exit = zero_state_exit;
    zero_state.next_state = NULL;

    hybrid_ctrl.pfct_state_enter = hybrid_state_enter;
    hybrid_ctrl.pfct_state_run = hybrid_state_run;
    hybrid_ctrl.pfct_state_exit = hybrid_state_exit;
    error_state.next_state = NULL;

    error_state.pfct_state_enter = error_state_enter;
    error_state.pfct_state_run = error_state_run;
    error_state.pfct_state_exit = error_state_exit;
    error_state.next_state = NULL;
}

void do_preparation(void)
{

    __HAL_DBGMCU_FREEZE_TIM1();
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    LL_ADC_Enable(ADC1);
    LL_ADC_Enable(ADC2);
    LL_ADC_Enable(ADC3);

    // measure offset current
    if (fsm_drv_h->DRV_Status)
    {
        fsm_drv_h->pfct_drv_enable();
        delay_ms(10);
        // NOTE the function will set adc1 regular channel to channel 1
        fsm_svpwm_h->pfct_zero_current();
        fsm_drv_h->pfct_drv_disable();
        fsm_svpwm_h->pfct_reset_pwm();

        // reconfigure the adc channel to vbus sensing channel
        fsm_board_h->pfct_set_channel();
        fsm_board_h->pfct_update_board();
        fsm_board_h->pfct_show_id();

        // TODO check here
        // current_state = &hybrid_ctrl;
        // current_state->next_state = &hybrid_ctrl;
        // foc_flag = 1;
        // sample_encoder = 1;
        // fsm_vpi_h->control_mode = Hybrid_MODE;

        current_state = &idle_state;
        current_state->next_state = &idle_state;
        foc_flag = 0;
        sample_encoder = 1;
        fsm_vpi_h->control_mode = Idle_Mode;
        fsm_svpwm_h->pfct_start_adc_sampling();
    }
    else
    {
        Error_Handler();
    }
}

void SystemClock_Config(void);

void ram_main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_ADC5_Init();
    MX_FDCAN1_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_CORDIC_Init();
    MX_SPI3_Init();
    MX_DMA_Init();
    MX_DAC1_Init();
    MX_TIM2_Init();

    init_fsm(); // initialize all devices first
#ifdef DAC_OUT
    dac_out_start();
#endif
    // calibrate ADCs
    do_preparation();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    initialized = 1;
    // #ifdef DAC_OUT

    // #endif
    while (1)
    {
        ram_fsm();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

// **********************************************************************************************************************
//                                     Debug Functions
// **********************************************************************************************************************
// 12bit adc
void dac_out_start(void)
{
    // start timer2

    HAL_TIM_Base_Start(&htim2);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)&dac_data, 1, DAC_ALIGN_12B_R);
}

void MX_DMA_Init(void)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    // HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
    // HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}