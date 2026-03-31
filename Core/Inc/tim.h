/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void MX_TIM6_Init(void);//ִ��
void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void set_pwm_duty(TIM_HandleTypeDef *htim, uint32_t channel, float duty_percent);
void motor_init(void);
void motor_A(int PWM, uint8_t di);
void motor_B(int PWM, uint8_t di);
void motor_C(int PWM, uint8_t di);
void motor_D(int PWM, uint8_t di);
void Move(long int pwma,long int pwmb,long int pwmc,long int pwmd,int8_t flag_a,int8_t flag_b,int8_t flag_c,int8_t flag_d);
void Back_Up(long int pwma,long int pwmb,long int pwmc,long int pwmd);
void Advance(long int pwma,long int pwmb,long int pwmc,long int pwmd);
void Right(long int pwma,long int pwmb,long int pwmc,long int pwmd);
void Left(long int pwma,long int pwmb,long int pwmc,long int pwmd);
void motor_Stop(void);
/* 编码器读取函数声�? */
int32_t Encoder_Read_TIM1(void);
int32_t Encoder_Read_TIM3(void);
int32_t Encoder_Read_TIM4(void);
int32_t Encoder_Read_TIM5(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

