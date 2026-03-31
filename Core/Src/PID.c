#include "PID.h"
#include <math.h>

static float PID_Clamp(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
}

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_i)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->max_output = max_out;
    pid->max_integral = max_i;
    pid->integral_separation = max_out;
    pid->derivative_alpha = 0.25f;
    pid->output_ramp = max_out;
    PID_Reset(pid);
}

float PID_Calc(PID_TypeDef *pid, float target, float actual, float dt)
{
    float error_delta;
    float raw_derivative;
    float output_delta;

    if (dt <= 0.0f) {
        dt = 0.01f;
    }

    pid->target = target;
    pid->actual = actual;
    pid->error = pid->target - pid->actual;

    if ((pid->integral_separation <= 0.0f) || (fabsf(pid->error) < pid->integral_separation)) {
        pid->integral += pid->error * dt;
        pid->integral = PID_Clamp(pid->integral, -pid->max_integral, pid->max_integral);
    }

    error_delta = pid->error - pid->last_error;
    raw_derivative = error_delta / dt;
    pid->derivative += pid->derivative_alpha * (raw_derivative - pid->derivative);

    pid->output = pid->Kp * pid->error +
                  pid->Ki * pid->integral +
                  pid->Kd * pid->derivative;

    pid->output = PID_Clamp(pid->output, -pid->max_output, pid->max_output);

    output_delta = pid->output - pid->last_output;
    if (pid->output_ramp > 0.0f) {
        output_delta = PID_Clamp(output_delta, -pid->output_ramp, pid->output_ramp);
        pid->output = pid->last_output + output_delta;
    }

    pid->last_error = pid->error;
    pid->last_output = pid->output;
    return pid->output;
}
