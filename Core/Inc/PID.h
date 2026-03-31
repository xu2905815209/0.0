#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target;
    float actual;

    float error;
    float last_error;
    float integral;
    float derivative;

    float max_output;
    float max_integral;
    float output;
    float last_output;

    float integral_separation;
    float derivative_alpha;
    float output_ramp;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_i);
void PID_Reset(PID_TypeDef *pid);
float PID_Calc(PID_TypeDef *pid, float target, float actual, float dt);

#endif
