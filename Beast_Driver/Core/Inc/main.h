/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOC
#define SOA_Pin GPIO_PIN_0
#define SOA_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_1
#define SOB_GPIO_Port GPIOA
#define DRV_NFAULT_Pin GPIO_PIN_3
#define DRV_NFAULT_GPIO_Port GPIOA
#define DRV_NFAULT_EXTI_IRQn EXTI3_IRQn
#define DAC_OUT_Pin GPIO_PIN_4
#define DAC_OUT_GPIO_Port GPIOA
#define DRV_SCK_Pin GPIO_PIN_5
#define DRV_SCK_GPIO_Port GPIOA
#define DRV_MISO_Pin GPIO_PIN_6
#define DRV_MISO_GPIO_Port GPIOA
#define DRV_MOSI_Pin GPIO_PIN_7
#define DRV_MOSI_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_0
#define DRV_CS_GPIO_Port GPIOB
#define Temp_Sen_Pin GPIO_PIN_1
#define Temp_Sen_GPIO_Port GPIOB
#define V_Bus_Sen_Pin GPIO_PIN_2
#define V_Bus_Sen_GPIO_Port GPIOB
#define Test_Pin_1_Pin GPIO_PIN_10
#define Test_Pin_1_GPIO_Port GPIOB
#define Test_Pin_2_Pin GPIO_PIN_11
#define Test_Pin_2_GPIO_Port GPIOB
#define ENCODER_CS_Pin GPIO_PIN_12
#define ENCODER_CS_GPIO_Port GPIOB
#define ENCODER_SCK_Pin GPIO_PIN_13
#define ENCODER_SCK_GPIO_Port GPIOB
#define ENCODER_MISO_Pin GPIO_PIN_14
#define ENCODER_MISO_GPIO_Port GPIOB
#define ENCODER_MOSI_Pin GPIO_PIN_15
#define ENCODER_MOSI_GPIO_Port GPIOB
#define F_ENCODER_CS_Pin GPIO_PIN_15
#define F_ENCODER_CS_GPIO_Port GPIOA
#define F_ENCODER_SCK_Pin GPIO_PIN_3
#define F_ENCODER_SCK_GPIO_Port GPIOB
#define F_ENCODER_MISO_Pin GPIO_PIN_4
#define F_ENCODER_MISO_GPIO_Port GPIOB
#define F_ENCODER_MOSI_Pin GPIO_PIN_5
#define F_ENCODER_MOSI_GPIO_Port GPIOB
#define DRV_ENABLE_Pin GPIO_PIN_6
#define DRV_ENABLE_GPIO_Port GPIOB
#define DRV_INL_Pin GPIO_PIN_7
#define DRV_INL_GPIO_Port GPIOB
#define FDCAN_STB_Pin GPIO_PIN_9
#define FDCAN_STB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
