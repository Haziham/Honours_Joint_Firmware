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
	pid->maxIntegral = 1000000;
	pid->timeCounter = 0;
	pid->previousDerivative = 0;
	pid->derivativeTsMs = 10;
}

float PID_calculate(PID_t *pid, int setPoint, int feedback)
{
	float output;
	float error =  setPoint - feedback;
	pid->sumError += error;
	pid->timeCounter++;

	// Anti-windup
	if (pid->sumError > pid->maxIntegral)
		pid->sumError = pid->maxIntegral;
	else if (pid->sumError < -pid->maxIntegral)
		pid->sumError = -pid->maxIntegral;

	output = *(pid->Kp) * error;
	output += *(pid->Ki) * pid->sumError * pid->Ts;
	output += *(pid->Kd)/10000 * (error - pid->previousError) / pid->Ts;
	// output += pid->previousDerivative;

	joint.statusC.debugValue1 = *(pid->Kp) * error;
	joint.statusC.debugValue3 = *(pid->Ki) * pid->sumError * pid->Ts;
	joint.statusC.debugValue2 = *(pid->Kd)/1000 * (error - pid->previousError) / pid->Ts;
	// joint.statusC.debugValue2 = pid->previousDerivative;

	// if (pid->timeCounter % pid->derivativeTsMs == 0)
	// {
	// 	pid->previousDerivative = *(pid->Kd) * (error - pid->previousError) / pid->derivativeTsMs/1000;
	// 	pid->previousError = error;
	// }




	output = output > pid->maxOutput ? pid->maxOutput : output;
	output = output < pid->minOutput ? pid->minOutput : output;


	
	return output;
}