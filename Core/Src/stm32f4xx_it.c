/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "control.h"
#include "supvc.h"
#include "usart.h"
#include "filter.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 鍏ㄥ眬鍙橀噺鐢ㄤ簬瀛樺偍缂栫爜鍣ㄧ疮璁¤锟? */
int32_t Encoder_TIM1_Count=0;
int32_t Encoder_TIM3_Count = 0;  // TIM3缂栫爜鍣ㄧ疮璁¤锟?
int32_t Encoder_TIM4_Count = 0;  // TIM4缂栫爜鍣ㄧ疮璁¤锟?
int32_t Encoder_TIM5_Count = 0;  // TIM5缂栫爜鍣ㄧ疮璁¤锟? 

typedef struct {
  uint16_t last_count_1;
  uint16_t last_count_3;
  uint16_t last_count_4;
  uint16_t last_count_5;
  uint8_t initialized;
  FilterLowPass2_t lpf_enc_1;
  FilterLowPass2_t lpf_enc_3;
  FilterLowPass2_t lpf_enc_4;
  FilterLowPass2_t lpf_enc_5;
  float enc_residual_1;
  float enc_residual_3;
  float enc_residual_4;
  float enc_residual_5;
} EncoderFilterState_t;

static EncoderFilterState_t g_encoder_state = {0};

static const int8_t ENCODER_DIR_1 = +1;
static const int8_t ENCODER_DIR_3 = -1;
static const int8_t ENCODER_DIR_4 = +1;
static const int8_t ENCODER_DIR_5 = -1;
static const float ENCODER_LPF_ALPHA = 0.40f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 使用当前计数值初始化编码器滤波状态。 */
static void Encoder_Filter_InitState(uint16_t count_1,
                                     uint16_t count_3,
                                     uint16_t count_4,
                                     uint16_t count_5)
{
  g_encoder_state.last_count_1 = count_1;
  g_encoder_state.last_count_3 = count_3;
  g_encoder_state.last_count_4 = count_4;
  g_encoder_state.last_count_5 = count_5;

  Filter_LowPass2_Init(&g_encoder_state.lpf_enc_1, ENCODER_LPF_ALPHA);
  Filter_LowPass2_Init(&g_encoder_state.lpf_enc_3, ENCODER_LPF_ALPHA);
  Filter_LowPass2_Init(&g_encoder_state.lpf_enc_4, ENCODER_LPF_ALPHA);
  Filter_LowPass2_Init(&g_encoder_state.lpf_enc_5, ENCODER_LPF_ALPHA);

  g_encoder_state.enc_residual_1 = 0.0f;
  g_encoder_state.enc_residual_3 = 0.0f;
  g_encoder_state.enc_residual_4 = 0.0f;
  g_encoder_state.enc_residual_5 = 0.0f;
  g_encoder_state.initialized = 1U;
}

/* 用二阶低通后的增量更新编码器累计值。 */
static void Encoder_Filter_UpdateFromCounter(uint16_t count_1,
                                             uint16_t count_3,
                                             uint16_t count_4,
                                             uint16_t count_5)
{
  int16_t diff_1;
  int16_t diff_3;
  int16_t diff_4;
  int16_t diff_5;
  float filt_1;
  float filt_3;
  float filt_4;
  float filt_5;
  float delta_1;
  float delta_3;
  float delta_4;
  float delta_5;
  int32_t step_1;
  int32_t step_3;
  int32_t step_4;
  int32_t step_5;

  if (!g_encoder_state.initialized) {
    Encoder_Filter_InitState(count_1, count_3, count_4, count_5);
    return;
  }

  diff_1 = (int16_t)(count_1 - g_encoder_state.last_count_1);
  diff_3 = (int16_t)(count_3 - g_encoder_state.last_count_3);
  diff_4 = (int16_t)(count_4 - g_encoder_state.last_count_4);
  diff_5 = (int16_t)(count_5 - g_encoder_state.last_count_5);

  filt_1 = Filter_LowPass2_Update(&g_encoder_state.lpf_enc_1, (float)diff_1);
  filt_3 = Filter_LowPass2_Update(&g_encoder_state.lpf_enc_3, (float)diff_3);
  filt_4 = Filter_LowPass2_Update(&g_encoder_state.lpf_enc_4, (float)diff_4);
  filt_5 = Filter_LowPass2_Update(&g_encoder_state.lpf_enc_5, (float)diff_5);

  delta_1 = (float)ENCODER_DIR_1 * filt_1 + g_encoder_state.enc_residual_1;
  delta_3 = (float)ENCODER_DIR_3 * filt_3 + g_encoder_state.enc_residual_3;
  delta_4 = (float)ENCODER_DIR_4 * filt_4 + g_encoder_state.enc_residual_4;
  delta_5 = (float)ENCODER_DIR_5 * filt_5 + g_encoder_state.enc_residual_5;

  step_1 = (int32_t)delta_1;
  step_3 = (int32_t)delta_3;
  step_4 = (int32_t)delta_4;
  step_5 = (int32_t)delta_5;

  g_encoder_state.enc_residual_1 = delta_1 - (float)step_1;
  g_encoder_state.enc_residual_3 = delta_3 - (float)step_3;
  g_encoder_state.enc_residual_4 = delta_4 - (float)step_4;
  g_encoder_state.enc_residual_5 = delta_5 - (float)step_5;

  Encoder_TIM1_Count += step_1;
  Encoder_TIM3_Count += step_3;
  Encoder_TIM4_Count += step_4;
  Encoder_TIM5_Count += step_5;

  g_encoder_state.last_count_1 = count_1;
  g_encoder_state.last_count_3 = count_3;
  g_encoder_state.last_count_4 = count_4;
  g_encoder_state.last_count_5 = count_5;
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim3);

}


/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
 
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{

  HAL_I2C_ER_IRQHandler(&hi2c1);

}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  extern UART_HandleTypeDef huart2;
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  * @note  TIM1 浣滀负缂栫爜鍣ㄦ帴鍙ｆ椂锛屾洿鏂颁腑鏂彲鐢ㄤ簬澶勭悊璁℃暟鍥炵粫/绱銆?
  */
void TIM1_UP_TIM10_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
  * @brief 定时器周期回调（10ms 控制调度入口）。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint16_t current_count_1;
  uint16_t current_count_3;
  uint16_t current_count_4;
  uint16_t current_count_5;

  if (htim->Instance != TIM6) {
      return;
  }

  current_count_1 = __HAL_TIM_GET_COUNTER(&htim1);
  current_count_3 = __HAL_TIM_GET_COUNTER(&htim3);
  current_count_4 = __HAL_TIM_GET_COUNTER(&htim4);
  current_count_5 = __HAL_TIM_GET_COUNTER(&htim5);

  Encoder_Filter_UpdateFromCounter(current_count_1, current_count_3, current_count_4, current_count_5);

  /* 统一控制入口（固定 10ms）:
   * 内部完成传感处理、模式外环、速度内环与遥测快照。 */
  ld();

}
/**
  * @brief TIM6 全局中断服务函数。
  */
void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

/* USER CODE END 1 */




