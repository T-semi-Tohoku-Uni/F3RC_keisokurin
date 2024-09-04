/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define eno_rst_Pin GPIO_PIN_13
#define eno_rst_GPIO_Port GPIOC
#define lmt_sw7_Pin GPIO_PIN_2
#define lmt_sw7_GPIO_Port GPIOC
#define lmt_sw6_Pin GPIO_PIN_3
#define lmt_sw6_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOA
#define ENC1_X_Pin GPIO_PIN_5
#define ENC1_X_GPIO_Port GPIOA
#define lmt_sw5_Pin GPIO_PIN_4
#define lmt_sw5_GPIO_Port GPIOC
#define lmt_sw1_Pin GPIO_PIN_5
#define lmt_sw1_GPIO_Port GPIOC
#define ENC3_A_Pin GPIO_PIN_2
#define ENC3_A_GPIO_Port GPIOB
#define lmt_sw8_Pin GPIO_PIN_11
#define lmt_sw8_GPIO_Port GPIOB
#define ENC3_X_Pin GPIO_PIN_12
#define ENC3_X_GPIO_Port GPIOB
#define ENC3_X_EXTI_IRQn EXTI15_10_IRQn
#define lmt_sw3_Pin GPIO_PIN_13
#define lmt_sw3_GPIO_Port GPIOB
#define lmt_sw2_Pin GPIO_PIN_14
#define lmt_sw2_GPIO_Port GPIOB
#define lmt_sw4_Pin GPIO_PIN_15
#define lmt_sw4_GPIO_Port GPIOB
#define ENC3_B_Pin GPIO_PIN_12
#define ENC3_B_GPIO_Port GPIOC
#define ENC2_A_Pin GPIO_PIN_4
#define ENC2_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_5
#define ENC2_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
