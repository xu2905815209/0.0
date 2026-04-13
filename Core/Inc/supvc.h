#ifndef __SUPVC_H__
#define __SUPVC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define SUPVC_CHANNEL_COUNT 6

#define SUPVC_TRIG1_GPIO_PORT GPIOB
#define SUPVC_TRIG1_PIN GPIO_PIN_0
#define SUPVC_TRIG2_GPIO_PORT GPIOB
#define SUPVC_TRIG2_PIN GPIO_PIN_1

#define SUPVC_US016_CH3_RANGE_GPIO_PORT GPIOB
#define SUPVC_US016_CH3_RANGE_PIN GPIO_PIN_5
#define SUPVC_US016_CH3_ADC_CHANNEL 6U

#define SUPVC_US016_CH4_RANGE_GPIO_PORT GPIOC
#define SUPVC_US016_CH4_RANGE_PIN GPIO_PIN_1
#define SUPVC_US016_CH4_ADC_CHANNEL 4U

#define SUPVC_US016_CH5_RANGE_GPIO_PORT GPIOC
#define SUPVC_US016_CH5_RANGE_PIN GPIO_PIN_2
#define SUPVC_US016_CH5_ADC_CHANNEL 10U

#define SUPVC_KS103_I2C_ADDRESS_7BIT 0x74U

void MX_TIM2_Init(void);
void SUPVC_Init(void);
void SUPVC_Trigger(uint8_t ch);
void SUPVC_Service_10ms(void);
void SUPVC_Service_MainLoop(void);
void SUPVC_SetKS103Address7bit(uint8_t addr7);
void SUPVC_SetKS103Mode(uint8_t direct_read, uint8_t alt_trigger);
void SUPVC_SetKS103AutoDetect(void);
uint8_t SUPVC_GetKS103Address7bit(void);
void SUPVC_GetKS103Flags(uint8_t *manual_addr, uint8_t *direct_read, uint8_t *alt_trigger);
float SUPVC_GetDistanceCm(uint8_t ch);
uint32_t SUPVC_GetEchoWidthUs(uint8_t ch);
uint8_t SUPVC_IsValid(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
