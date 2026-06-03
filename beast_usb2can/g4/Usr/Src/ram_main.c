#include "ram_main.h"
#include "G474_Motor.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "stm32g4xx_it.h"
#include "stm32g4xx_ll_gpio.h"
#include "tim.h"
#include "usr_delay.h"

#ifdef FDCAN
uint16_t can_delay_us = 130;  // 108 * 2 = 216
#endif

#ifdef CLASSICCAN
uint16_t can_delay_us = 250;
#endif
 
void SystemClock_Config(void);
int ram_main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  delay_init();

  // usr code
  init_motors_manager();
  delay_us(50);
  init_isr_manager();
  MOTORs_MANAGER_t* local_manager = get_motors_manager();
  HAL_SPI_TransmitReceive_DMA(
      &hspi1, (uint8_t*)local_manager->motor_datas_.spi_data_buff,
      (uint8_t*)local_manager->motor_cmds_.spi_cmd_buff, SPI_CMD_SIZE);
  local_manager->control_mode_ = null_ctrl;

  LL_GPIO_SetOutputPin(GPIOA, LED_1_Pin | LED_2_Pin);
  HAL_TIM_Base_Start_IT(&htim1);
  // id 2-1 3-2 4-3

  HAL_TIM_Base_Start_IT(&htim3);
  delay_us(can_delay_us);
  HAL_TIM_Base_Start_IT(&htim4);
  delay_us(can_delay_us);
  HAL_TIM_Base_Start_IT(&htim2);
  
  // HAL_GPIO_WritePin(GPIOA, LED_1_Pin | LED_2_Pin, GPIO_PIN_SET);
  while (1) {
    manager_task();
  }
}

void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}