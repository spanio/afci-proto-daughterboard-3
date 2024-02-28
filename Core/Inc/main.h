/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define L1V_P_Pin GPIO_PIN_0
#define L1V_P_GPIO_Port GPIOC
#define L1V_N_Pin GPIO_PIN_1
#define L1V_N_GPIO_Port GPIOC
#define L2V_N_Pin GPIO_PIN_2
#define L2V_N_GPIO_Port GPIOC
#define L2V_P_Pin GPIO_PIN_3
#define L2V_P_GPIO_Port GPIOC
#define LOCAL_CKT1_ADC_Pin GPIO_PIN_6
#define LOCAL_CKT1_ADC_GPIO_Port GPIOA
#define LOCAL_CKT2_ADC_Pin GPIO_PIN_7
#define LOCAL_CKT2_ADC_GPIO_Port GPIOA
#define LOCAL_CKT1_ADC_DIRECT_Pin GPIO_PIN_5
#define LOCAL_CKT1_ADC_DIRECT_GPIO_Port GPIOC
#define GPIO1_Pin GPIO_PIN_9
#define GPIO1_GPIO_Port GPIOD
#define GPIO2_Pin GPIO_PIN_10
#define GPIO2_GPIO_Port GPIOD
#define GPIO3_Pin GPIO_PIN_11
#define GPIO3_GPIO_Port GPIOD
#define L1_ZERO_CROSS_Pin GPIO_PIN_5
#define L1_ZERO_CROSS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
