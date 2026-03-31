/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// 延迟函数 (10us)
static void MyI2C_Delay(void)
{
    // 使用HAL库的微秒延迟函数
    HAL_Delay(1); // 注意：HAL_Delay�?小是1ms，可以替换为更精确的微秒延迟
}

// 写SCL引脚
void MyI2C_W_SCL(uint8_t BitValue)
{
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
    MyI2C_Delay();
}

// 写SDA引脚
void MyI2C_W_SDA(uint8_t BitValue)
{
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, (BitValue ? GPIO_PIN_SET : GPIO_PIN_RESET));
    MyI2C_Delay();
}

// 读SDA引脚
uint8_t MyI2C_R_SDA(void)
{
    uint8_t BitValue;
    BitValue = HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN);
    MyI2C_Delay();
    return BitValue;
}
//// 软件I2C引脚初始�?
//void MyI2C_Init(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    
//    // 使能GPIOB时钟
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    
//    // 配置GPIO
//    GPIO_InitStructure.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
//    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD; // �?漏输出模�?
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
//    
//    // 初始状�?�：SCL和SDA都为高电�?
//    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
//}

// I2C起始条件
void MyI2C_Start(void)
{
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(0);
}

// I2C终止条件
void MyI2C_Stop(void)
{
    MyI2C_W_SDA(0);
    MyI2C_W_SCL(1);
    MyI2C_W_SDA(1);
}

// 发�?�一个字�?
void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t count;
    for(count = 0; count < 8; count++)
    {
        MyI2C_W_SDA(Byte & (0x80 >> count));
        MyI2C_W_SCL(1);  // 从机读数�?
        MyI2C_W_SCL(0);  // 主机写数�?
    }
}

// 接收�?个字�?
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t count, Byte = 0x00;
    MyI2C_W_SDA(1);  // 主机释放SDA�?
    
    for(count = 0; count < 8; count++)
    {
        MyI2C_W_SCL(1);  // 主机读数�?
        if(MyI2C_R_SDA() == 1)
        {
            Byte |= (0x80 >> count);
        }
        MyI2C_W_SCL(0);  // 从机写数�?
    }
    return Byte;
}

// 发�?�一个应�?
void MyI2C_SendAck(uint8_t AckBit)
{
    MyI2C_W_SDA(AckBit);  // 发�?�应答位
    MyI2C_W_SCL(1);        // 从机读应�?
    MyI2C_W_SCL(0);        // 进入下一个时序单�?
}

// 接收�?个应�?
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
    MyI2C_W_SDA(1);
    MyI2C_W_SCL(1);
    AckBit = MyI2C_R_SDA();
    MyI2C_W_SCL(0);  // 进入下一个时序单�?
    return AckBit;
}

// 多字节写�?
uint8_t MyI2C_WriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
    uint8_t count = 0;
    
    MyI2C_Start();
    MyI2C_SendByte(dev);        // 发�?�写命令
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg);        // 发�?�寄存器地址
    MyI2C_ReceiveAck();
    
    for(count = 0; count < length; count++)
    {
        MyI2C_SendByte(data[count]);
        MyI2C_ReceiveAck();
    }
    
    MyI2C_Stop();  // 产生停止条件
    
    return 1;
}
/* USER CODE END 1 */
