#include "filter.h"

/* 限幅工具函数，避免参数越界。 */
static float Filter_Clamp(float value, float min_value, float max_value)
{
	if (value > max_value) {
		return max_value;
	}
	if (value < min_value) {
		return min_value;
	}
	return value;
}

/* 初始化卡尔曼滤波器参数和状态。 */
void Filter_Kalman1D_Init(FilterKalman1D_t *kf, float q, float r, float init_value)
{
	kf->q = q;
	kf->r = r;
	kf->p = 1.0f;
	kf->x = init_value;
	kf->initialized = 1U;
}

/* 复位卡尔曼状态，下一次更新时以测量值重启。 */
void Filter_Kalman1D_Reset(FilterKalman1D_t *kf, float init_value)
{
	kf->p = 1.0f;
	kf->x = init_value;
	kf->initialized = 0U;
}

/* 一阶卡尔曼更新：预测误差 -> 计算增益 -> 修正估计。 */
float Filter_Kalman1D_Update(FilterKalman1D_t *kf, float measurement)
{
	float gain;

	if (!kf->initialized) {
		kf->x = measurement;
		kf->p = 1.0f;
		kf->initialized = 1U;
		return measurement;
	}

	kf->p += kf->q;
	gain = kf->p / (kf->p + kf->r);
	kf->x = kf->x + gain * (measurement - kf->x);
	kf->p = (1.0f - gain) * kf->p;
	return kf->x;
}

/* 初始化二阶低通（两级一阶串联）。 */
void Filter_LowPass2_Init(FilterLowPass2_t *filter, float alpha)
{
	filter->alpha = Filter_Clamp(alpha, 0.01f, 1.0f);
	filter->stage1 = 0.0f;
	filter->stage2 = 0.0f;
	filter->initialized = 0U;
}

/* 将二阶低通内部状态重置为给定值。 */
void Filter_LowPass2_Reset(FilterLowPass2_t *filter, float value)
{
	filter->stage1 = value;
	filter->stage2 = value;
	filter->initialized = 1U;
}

/* 二阶低通更新：先过一级，再过二级。 */
float Filter_LowPass2_Update(FilterLowPass2_t *filter, float sample)
{
	if (!filter->initialized) {
		Filter_LowPass2_Reset(filter, sample);
		return sample;
	}

	filter->stage1 += filter->alpha * (sample - filter->stage1);
	filter->stage2 += filter->alpha * (filter->stage1 - filter->stage2);
	return filter->stage2;
}
