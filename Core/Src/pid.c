#include "pid.h"
#include "joint.h" // only for debugging

void PID_init(PID_t *pid, float* Kp, float* Ki, float* Kd, float Ts, float minOutput, float maxOutput)
{
	pid->Kd = Kd;
	pid->Ki = Ki;
	pid->Kp = Kp;
	pid->maxOutput = maxOutput;
	pid->minOutput = minOutput;
	pid->Ts = Ts;
	pid->previousError = 0;
	pid->sumError = 0;
	pid->maxIntegral = 1000;
}

float PID_calculate(PID_t *pid, int setPoint, int feedback)
{
	float output;
	float error =  setPoint - feedback;
	pid->sumError += error;

	// Anti-windup
	if (pid->sumError > pid->maxIntegral)
		pid->sumError = pid->maxIntegral;
	else if (pid->sumError < -pid->maxIntegral)
		pid->sumError = -pid->maxIntegral;

	output = *(pid->Kp) * error;
	output += *(pid->Ki) * pid->sumError * pid->Ts;
	output += *(pid->Kd) * (error - pid->previousError) / pid->Ts;

	joint.statusC.debugValue1 = *(pid->Kp) * error;
	joint.statusC.debugValue2 = *(pid->Ki) * pid->sumError * pid->Ts;
	joint.statusC.debugValue3 = *(pid->Kd) * (error - pid->previousError) / pid->Ts;

	pid->previousError = error;
	output = output > pid->maxOutput ? pid->maxOutput : output;
	output = output < pid->minOutput ? pid->minOutput : output;


	
	return output;
}