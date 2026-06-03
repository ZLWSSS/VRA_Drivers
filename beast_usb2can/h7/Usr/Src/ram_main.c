#include "dma.h"
#include "flowing_led.h"
#include "gpio.h"
#include "h7_task.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "usr_buzzer.h"
#include "usr_delay.h"
#include "usr_spi.h"

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);

uint8_t timer_started = 0;

int ram_main(void) {
  MPU_Config();
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();

  // usr initialize
  delay_init();
  init_spi_protocals();
  init_h7_board();
  flow_led_handle_init();
  flowing_led_handle_t* fsm_fl_handle = get_flowing_led_handle();
  H7_Board_TypeDef* h7_board_handle   = get_h7_board();

  buzzer_handle_t* buzzer_handle = get_buzzer_handle();

  // usr initialize done

  // the timer for spi sending is started in this func.
  buzzer_start();
  // start the sending and buzzer timer.
  HAL_TIM_Base_Start_IT(&htim2);

  buzzer_handle->mode_ = b_Standby;
  fsm_fl_handle->mode_ = Led_Standby;
  //h7_board_handle->run = H7_RUN;

  while (1) {
    if ((h7_board_handle->hit_miss < 3) & !timer_started) {
      h7_board_handle->run = H7_RUN;
      buzzer_handle->mode_ = b_Working;
      timer_started        = 1;
      fsm_fl_handle->mode_ = Led_Working;
      // 2000hz, count 20: 10ms gap.
    } else if ((h7_board_handle->hit_miss > 5) & timer_started) {
      h7_board_handle->run = H7_STOP;
      buzzer_handle->mode_ = b_Standby;
      timer_started        = 0;
      fsm_fl_handle->mode_ = Led_Pause;
    } else {
      // TODO add error checker.
      //  fsm_fl_handle->mode_ = Error;
    }
    fsm_fl_handle->func_run(fsm_fl_handle->mode_);
  }
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 5;
  RCC_OscInitStruct.PLL.PLLN       = 192;
  RCC_OscInitStruct.PLL.PLLP       = 2;
  RCC_OscInitStruct.PLL.PLLQ       = 2;
  RCC_OscInitStruct.PLL.PLLR       = 2;
  RCC_OscInitStruct.PLL.PLLRGE     = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN   = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
      RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3 | RCC_PERIPHCLK_SPI1 | RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M           = 5;
  PeriphClkInitStruct.PLL2.PLL2N           = 180;
  PeriphClkInitStruct.PLL2.PLL2P           = 10;
  PeriphClkInitStruct.PLL2.PLL2Q           = 10;
  PeriphClkInitStruct.PLL2.PLL2R           = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL      = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN       = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection  = RCC_SPI45CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
  MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress      = 0x0;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0;
  MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress      = 0x20000000;
  MPU_InitStruct.Size             = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Number       = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress  = 0x24000000;
  MPU_InitStruct.IsCacheable  = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.Size         = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  MPU_InitStruct.Number       = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress  = 0x38000000;
  MPU_InitStruct.IsCacheable  = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.Size         = MPU_REGION_SIZE_64KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
