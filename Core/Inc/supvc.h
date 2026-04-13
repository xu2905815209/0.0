#ifndef __SUPVC_H__
#define __SUPVC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define SUPVC_CHANNEL_COUNT 6

void SUPVC_Init(void);
void SUPVC_Service_10ms(void); // empty dummy if needed
void SUPVC_Service_MainLoop(void);
float SUPVC_GetDistanceCm(uint8_t ch);
uint8_t SUPVC_IsValid(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
