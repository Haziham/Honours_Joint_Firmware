#pragma once

#include <stdint.h>

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double Ts;
    double previousError;
    double sumError;
    double maxOutput;
    double minOutput;
} PID_t;

void PID_init(PID_t *pid, double Kp, double Ki, double Kd, double Ts, double minOutput, double maxOutput);
double PID_calculate(PID_t *pid, int setpoint, int feedback);
