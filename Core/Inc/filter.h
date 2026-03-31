#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
	/* 过程噪声协方差 */
	float q;
	/* 测量噪声协方差 */
	float r;
	/* 估计误差协方差 */
	float p;
	/* 当前估计值 */
	float x;
	/* 0: 未初始化, 1: 已初始化 */
	uint8_t initialized;
} FilterKalman1D_t;

typedef struct {
	/* 一阶低通系数, (0,1], 越小越平滑 */
	float alpha;
	/* 第一级低通状态 */
	float stage1;
	/* 第二级低通状态 */
	float stage2;
	/* 0: 未初始化, 1: 已初始化 */
	uint8_t initialized;
} FilterLowPass2_t;

/* 初始化一阶卡尔曼滤波器。 */
void Filter_Kalman1D_Init(FilterKalman1D_t *kf, float q, float r, float init_value);
/* 复位一阶卡尔曼滤波器。 */
void Filter_Kalman1D_Reset(FilterKalman1D_t *kf, float init_value);
/* 输入新测量值，返回卡尔曼估计值。 */
float Filter_Kalman1D_Update(FilterKalman1D_t *kf, float measurement);

/* 初始化二阶低通滤波器（两级一阶串联）。 */
void Filter_LowPass2_Init(FilterLowPass2_t *filter, float alpha);
/* 复位二阶低通滤波器内部状态。 */
void Filter_LowPass2_Reset(FilterLowPass2_t *filter, float value);
/* 输入新采样值，返回二阶低通输出。 */
float Filter_LowPass2_Update(FilterLowPass2_t *filter, float sample);

#ifdef __cplusplus
}
#endif

#endif
