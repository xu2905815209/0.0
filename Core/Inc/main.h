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
#include "stm32f4xx_hal.h"

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
#define motor3_Pin GPIO_PIN_12
#define motor3_GPIO_Port GPIOB
#define motor3n_Pin GPIO_PIN_13
#define motor3n_GPIO_Port GPIOB
#define motor4_Pin GPIO_PIN_14
#define motor4_GPIO_Port GPIOB
#define motor4n_Pin GPIO_PIN_15
#define motor4n_GPIO_Port GPIOB
#define motor1_Pin GPIO_PIN_8
#define motor1_GPIO_Port GPIOD
#define motor1n_Pin GPIO_PIN_9
#define motor1n_GPIO_Port GPIOD
#define motor2_Pin GPIO_PIN_10
#define motor2_GPIO_Port GPIOD
#define motor2n_Pin GPIO_PIN_11
#define motor2n_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
