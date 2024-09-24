#pragma once

#include <stdint.h>

typedef struct {
    float* Kp;
    float* Ki;
    float* Kd;
    float Ts;
    float previousError;
    float sumError;
    float maxOutput;
    float minOutput;
    float maxIntegral;
} PID_t;

void PID_init(PID_t *pid, float* Kp, float* Ki, float* Kd, float Ts, float minOutput, float maxOutput);
float PID_calculate(PID_t *pid, int setpoint, int feedback);
