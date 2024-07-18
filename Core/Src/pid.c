#include "pid.h"

void PID_init(PID_t *pid, double Kp, double Ki, double Kd, double Ts, double minOutput, double maxOutput)
{
	pid->Kd = Kd;
	pid->Ki = Ki;
	pid->Kp = Kp;
	pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
	pid->Ts = Ts;
	pid->previousError = 0;
	pid->sumError = 0;
}

double PID_calculate(PID_t *pid, int setpoint, int feedback)
{
	float output;
	float error = feedback - setpoint;

	output = pid->Kp * error;
    output += pid->Ki * pid->sumError * pid->Ts;
    output += pid->Kd * (error - pid->previousError) / pid->Ts;

    pid->previousError = error;
    output = output > pid->maxOutput ? pid->maxOutput : output;
    output = output < pid->minOutput ? pid->minOutput : output;

    return output;
}