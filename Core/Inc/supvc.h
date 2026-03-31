#ifndef __SUPVC_H__
#define __SUPVC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim2;

#define SUPVC_CHANNEL_COUNT 3

#define SUPVC_TRIG1_GPIO_PORT GPIOB
#define SUPVC_TRIG1_PIN GPIO_PIN_0
#define SUPVC_TRIG2_GPIO_PORT GPIOB
#define SUPVC_TRIG2_PIN GPIO_PIN_1
#define SUPVC_TRIG3_GPIO_PORT GPIOB
#define SUPVC_TRIG3_PIN GPIO_PIN_5

void MX_TIM2_Init(void);
void SUPVC_Init(void);
void SUPVC_Trigger(uint8_t ch);
void SUPVC_Service_10ms(void);
float SUPVC_GetDistanceCm(uint8_t ch);
uint32_t SUPVC_GetEchoWidthUs(uint8_t ch);
uint8_t SUPVC_IsValid(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
