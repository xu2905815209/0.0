/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "control.h"

/* USER CODE BEGIN 0 */
/* 蓝牙命令策略:
 * - 兼容旧的单字符命令（'0'~'9'）
 * - 支持按行文本命令（如 CMD,KS103,GET），以 CR/LF 结尾 */
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
static uint8_t bluetooth_rx_byte = 0;
static char bluetooth_line_buf[96];
static uint8_t bluetooth_line_len = 0U;

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  Bluetooth_UART_StartReceive();
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

/* USER CODE BEGIN 1 */
void Bluetooth_UART_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &bluetooth_rx_byte, 1);
}

void Bluetooth_UART_RxCallback(UART_HandleTypeDef *huart)
{
    uint8_t byte;

    if (huart->Instance != USART1) {
        return;
    }

    byte = bluetooth_rx_byte;

  if ((byte == '\r') || (byte == '\n')) {
    if (bluetooth_line_len > 0U) {
      bluetooth_line_buf[bluetooth_line_len] = '\0';
      UART_Command_ProcessLine(bluetooth_line_buf);
      bluetooth_line_len = 0U;
    }

    Bluetooth_UART_StartReceive();
    return;
  }

  if ((byte >= 0x20U) && (byte <= 0x7EU)) {
    if ((bluetooth_line_len == 0U) && (byte >= '0') && (byte <= '9')) {
      UART_Command_ProcessByte(byte);
    } else {
      if (bluetooth_line_len < (sizeof(bluetooth_line_buf) - 1U)) {
        bluetooth_line_buf[bluetooth_line_len++] = (char)byte;
      } else {
        /* 溢出保护：丢弃本行，等待下一次换行重新同步。 */
        bluetooth_line_len = 0U;
      }
    }
    }

    Bluetooth_UART_StartReceive();
}

void Bluetooth_UART_ProcessPending(void)
{
  /* 当前命令在接收中断内闭环处理，主循环阶段无需额外动作。 */
}

int uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len > 0) {
        if (len >= (int)sizeof(buf)) {
            len = (int)sizeof(buf) - 1;
        }
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 100);
    }
    return len;
}
/* USER CODE END 1 */
