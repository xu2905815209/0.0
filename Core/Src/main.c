/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "control.h"
#include "supvc.h"

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  motor_init();
  SUPVC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  Chassis_PID_Init();
  uart_printf("BOOT: USART1 OK\r\n");

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  Chassis_Control_ResetState();
  HAL_TIM_Base_Start_IT(&htim6);

  while (1)
  {
    float d1;
    float d2;
    float d3;
    uint32_t w1;
    uint32_t w2;
    uint32_t raw3;
    uint8_t v1;
    uint8_t v2;
    uint8_t v3;
    static uint32_t print_count = 0;

    d1 = SUPVC_GetDistanceCm(1);
    d2 = SUPVC_GetDistanceCm(2);
    d3 = SUPVC_GetDistanceCm(3);
    w1 = SUPVC_GetEchoWidthUs(1);
    w2 = SUPVC_GetEchoWidthUs(2);
    raw3 = SUPVC_GetEchoWidthUs(3);
    v1 = SUPVC_IsValid(1);
    v2 = SUPVC_IsValid(2);
    v3 = SUPVC_IsValid(3);

    print_count++;
    if ((print_count % 2U) == 0U) {
      uart_printf("ULTRA_FILT: L=%.2fcm R=%.2fcm F=%.2fcm\r\n", d1, d2, d3);
      uart_printf("SUPVC: D1=%.2fcm W1=%lu V1=%u | D2=%.2fcm W2=%lu V2=%u | D3=%.2fcm A3=%lu V3=%u\r\n",
                  d1, (unsigned long)w1, (unsigned int)v1,
                  d2, (unsigned long)w2, (unsigned int)v2,
                  d3, (unsigned long)raw3, (unsigned int)v3);
    }

    HAL_Delay(100);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
