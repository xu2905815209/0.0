#ifndef __JY61P_UART_H__
#define __JY61P_UART_H__

#include "main.h"

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float angle_x;
    float angle_y;
    float angle_z;
} JY61P_Data_t;

extern JY61P_Data_t JY61P_Data;

void JY61P_UART_Init(void);
void JY61P_Update(void);

#endif // __JY61P_UART_H__
