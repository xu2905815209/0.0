#include "jy61p_uart.h"
#include "stm32f4xx_hal.h"
#include "usart.h"

UART_HandleTypeDef huart2;
JY61P_Data_t JY61P_Data = {0};

static uint8_t jy61_buf[11];
static uint8_t jy61_idx = 0;

static void JY61P_ParsePacket(const uint8_t *buf)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    uint8_t sum = 0;
    int i;

    if (buf[0] != 0x55U) return;

    for (i = 0; i < 10; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    if (sum != buf[10]) return;

    raw_x = (int16_t)((buf[3] << 8) | buf[2]);
    raw_y = (int16_t)((buf[5] << 8) | buf[4]);
    raw_z = (int16_t)((buf[7] << 8) | buf[6]);

    switch (buf[1]) {
        case 0x51:
            JY61P_Data.acc_x = -raw_y / 32768.0f * 16.0f;
            JY61P_Data.acc_y = -raw_x / 32768.0f * 16.0f;
            JY61P_Data.acc_z =  raw_z / 32768.0f * 16.0f;
            break;

        case 0x52:
            JY61P_Data.gyro_x = -raw_y / 32768.0f * 2000.0f;
            JY61P_Data.gyro_y = -raw_x / 32768.0f * 2000.0f;
            JY61P_Data.gyro_z = -raw_z / 32768.0f * 2000.0f;
            break;

        case 0x53:
            JY61P_Data.angle_z = raw_z / 32768.0f * 180.0f;
            JY61P_Data.angle_y = -raw_x / 32768.0f * 180.0f;
            JY61P_Data.angle_x = -raw_y / 32768.0f * 180.0f;
            break;

        default:
            break;
    }
}

void JY61P_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    HAL_UART_Receive_IT(&huart2, &jy61_buf[0], 1);
}

void JY61P_Update(void)
{
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Bluetooth_UART_RxCallback(huart);
        return;
    }

    if (huart->Instance != USART2) {
        return;
    }

    if (jy61_idx == 0U) {
        if (jy61_buf[0] == 0x55U) {
            jy61_idx = 1U;
            HAL_UART_Receive_IT(&huart2, &jy61_buf[1], 10);
        } else {
            HAL_UART_Receive_IT(&huart2, &jy61_buf[0], 1);
        }
        return;
    }

    JY61P_ParsePacket(jy61_buf);
    jy61_idx = 0U;
    HAL_UART_Receive_IT(&huart2, &jy61_buf[0], 1);
}
