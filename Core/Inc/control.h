#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "PID.h"
#include "tim.h"

#define WHEEL_DIAMETER_MM    74.0f
#define ENCODER_PPR          11.0f
#define GEAR_RATIO           30.0f
#define PULSES_PER_MM        ((ENCODER_PPR * 4.0f * GEAR_RATIO) / (3.1415926f * WHEEL_DIAMETER_MM))

/* 超声中线跟随参数配置。 */
#define ULTRA_NORM_MIN_CM                 2.0f
#define ULTRA_NORM_MAX_CM                 50.0f
#define ULTRA_NORM_CENTER_VALUE           50.0f
#define ULTRA_CENTERLINE_KP               0.35f
#define ULTRA_CENTERLINE_LATERAL_LIMIT    20.0f
#define ULTRA_CENTERLINE_DEFAULT_VX       18.0f
#define ULTRA_CENTERLINE_MAX_ABS_VX       60.0f

extern PID_TypeDef PID_Pos_A, PID_Pos_B, PID_Pos_C, PID_Pos_D;
extern float Target_Pulses_A;
extern float Target_Pulses_B;
extern float Target_Pulses_C;
extern float Target_Pulses_D;

void Chassis_PID_Init(void);
void Chassis_Control_ResetState(void);
int32_t Read_Encoder_Total(TIM_HandleTypeDef *htim, int32_t *overflow_count);

void Chassis_Drive_Speed(float Vx, float Vy, float Vw);
void Move_Forward(float speed);
void Move_Backward(float speed);
void Move_Left(float speed);
void Move_Right(float speed);
void Move_TopLeft(float speed);
void Move_TopRight(float speed);
void Move_BottomLeft(float speed);
void Move_BottomRight(float speed);
void Rotate_In_Place(float speed);

void Chassis_Set_Target_Distance(float X_mm, float Y_mm, float W_deg);
void Chassis_Position_Control_Loop(void);
uint8_t Chassis_Is_Position_Reached(void);

void Move_Distance_Forward(float distance_mm);
void Move_Distance_Backward(float distance_mm);
void Move_Distance_Left(float distance_mm);
void Move_Distance_Right(float distance_mm);

void Chassis_UltrasonicCenterline_Start(float forward_speed);
void Chassis_UltrasonicCenterline_Stop(void);
void Chassis_UltrasonicCenterline_Update(void);

void UART_Command_Process(uint8_t command);

#endif

