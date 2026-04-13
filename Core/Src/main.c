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
#include "jy61p_uart.h"

void SystemClock_Config(void);

/* 关键控制调度入口（由定时器中断周期调用）:
 * 1) 信号采集: 传感器/编码器/IMU 原始量更新
 * 2) 信号预处理: 滤波、归一化
 * 3) 信号处理: 差值计算、方向闭环、速度闭环、PWM 计算
 * 4) 硬件控制: 四电机 PWM 下发
 *
 * 具体细节由 Control_10ms_Task() 负责实现，ld() 负责统一入口管理。 */
void ld(void)
{
  Control_10ms_Task();
}

int main(void)
{
  /* 基础硬件初始化（HAL + 时钟）。 */
  HAL_Init();
  SystemClock_Config();

  /* 外设初始化顺序:
   * GPIO/电机/传感器/串口/IMU/控制定时器。 */
  MX_GPIO_Init();
  motor_init();
  /* MX_I2C1_Init(); // 已禁用：PB6/PB7 现在由软件 I2C 驱动 VL53L0X CH6 */
  MX_USART1_UART_Init();
  JY61P_UART_Init();
  SUPVC_Init();  /* 必须在串口之后初始化，以便输出诊断日志 */
  MX_TIM6_Init();

  /* 控制模块初始化（参数、PID、状态机）。 */
  Chassis_PID_Init();

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  Chassis_Control_ResetState();

  /* 启动 10ms 控制中断。 */
  HAL_TIM_Base_Start_IT(&htim6);

  while (1)
  {
    /* 主循环只做轻量后台任务:
     * 1) 蓝牙命令出队并解析
     * 2) 校准与遥测发送 */
    Bluetooth_UART_ProcessPending();
    Control_MainLoop_Task();
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
