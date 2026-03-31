#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "PID.h"
#include "tim.h"

#define WHEEL_DIAMETER_MM    74.0f
#define ENCODER_PPR          11.0f
#define GEAR_RATIO           30.0f
#define PULSES_PER_MM        ((ENCODER_PPR * 4.0f * GEAR_RATIO) / (3.1415926f * WHEEL_DIAMETER_MM))

/* 控制模式状态机。
 * IDLE: 电机停转，仅采样与回传。
 * MANUAL_VEL: 直接跟踪上位机给定的 vx/vy/wz 速度指令。
 * LINE_FOLLOW: 走廊直线模式，利用左右超声差值做外环纠偏。
 * CALIBRATION: 校准模式，仅采集并回传超声原始值统计。 */
typedef enum {
    CONTROL_MODE_IDLE = 0,
    CONTROL_MODE_MANUAL_VEL = 1,
    CONTROL_MODE_LINE_FOLLOW = 2,
    CONTROL_MODE_CALIBRATION = 3
} ControlMode_t;

/* 初始化控制参数、PID 和运行时状态。 */
void Chassis_PID_Init(void);
/* 清空运行状态并急停输出（不改硬件初始化）。 */
void Chassis_Control_ResetState(void);
int32_t Read_Encoder_Total(TIM_HandleTypeDef *htim, int32_t *overflow_count);

/* 10ms 控制任务（在 TIM6 中断中调用）。
 * 执行顺序: 采样 -> 模式外环 -> 速度环 -> 快照。 */
void Control_10ms_Task(void);
/* 主循环后台任务。
 * 执行串口命令出队、校准报告发送、FireWater 遥测输出。 */
void Control_MainLoop_Task(void);
/* 单字符命令入口（推荐用于蓝牙实时控制）。 */
void UART_Command_ProcessByte(uint8_t cmd);
/* 处理单行 ASCII 命令（格式: CMD,xxx,...）。 */
void UART_Command_ProcessLine(const char *line);

/* 模式切换接口。 */
void Chassis_SetMode(ControlMode_t mode);
ControlMode_t Chassis_GetMode(void);
/* 手动速度模式入口，单位: mm/s, mm/s, deg/s。 */
void Chassis_SetManualVelocity(float vx_mmps, float vy_mmps, float wz_dps);
/* 启动走廊模式，forward_mmps 为前进速度，line_target 为左右归一化差值目标。 */
void Chassis_StartLineFollow(float forward_mmps, float line_target);
/* 在线更新方向环目标值。 */
void Chassis_SetLineTarget(float line_target);
/* 停止走廊模式并进入 IDLE。 */
void Chassis_StopLineFollow(void);
/* 启动超声校准采样（按通道位掩码，采样数量）。 */
void Chassis_StartCalibration(uint8_t channel_mask, uint16_t sample_count);
/* 手工设置超声归一化上下限（原始值域）。 */
void Chassis_SetUltrasonicNormRawRange(uint8_t channel, float min_raw, float max_raw);

/* Legacy helper wrappers kept for compatibility with existing test scripts. */
void Move_Forward(float speed);
void Move_Backward(float speed);
void Move_Left(float speed);
void Move_Right(float speed);
void Move_TopLeft(float speed);
void Move_TopRight(float speed);
void Move_BottomLeft(float speed);
void Move_BottomRight(float speed);
void Rotate_In_Place(float speed);

#endif
