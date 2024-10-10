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
	pid->previousFeedback = 0;
	pid->sumError = 0;
	pid->maxIntegral = 1000000;
	pid->previousError = 0;
	
}

float PID_calculate(PID_t *pid, int setPoint, int feedback)
{
	float output;
	float error =  setPoint - feedback;
	// pid->sumError += error;

	if (error == 0)
	{
		pid->sumError = 0;
	}
	else 
	{
		pid->sumError += 100/error;
	}


	// Anti-windup
	if (pid->sumError > pid->maxIntegral)
		pid->sumError = pid->maxIntegral;
	else if (pid->sumError < -pid->maxIntegral)
		pid->sumError = -pid->maxIntegral;

	// if change in error sign then dont integreate
	// if (error * pid->previousError <= 0)
	// 	pid->sumError = 0;
	// else {
	//  t

	float proportional = *(pid->Kp)/10 * error;
	// float derivative = *(pid->Kd)/1000 * (error - pid->previousError) / pid->Ts;
	float derivative = *(pid->Kd)/1000 * (feedback - pid->previousFeedback) / pid->Ts;
	float integral = *(pid->Ki)/10 * pid->sumError * pid->Ts;

	output = proportional + integral + derivative; 
	// output = *(pid->Kp)/1000 * error;
	// output += *(pid->Ki)/1000 * pid->sumError * pid->Ts;
	// output += *(pid->Kd)/1000 * (error - pid->previousError) / pid->Ts;
	// output += *(pid->Kd)/1000 * (feedback - pid->previousFeedback) / pid->Ts;
	// output += pid->previousDerivative;

	joint.statusC.debugValue1 = proportional;
	joint.statusC.debugValue3 = integral;
	joint.statusC.debugValue2 = derivative;
	// joint.statusC.debugValue2 = pid->previousDerivative;

	// if (pid->timeCounter % pid->derivativeTsMs == 0)
	// {
	// 	pid->previousDerivative = *(pid->Kd) * (error - pid->previousFeedback) / pid->derivativeTsMs/1000;
 // 	pid->previousFeedback = error;	// }




	output = output > pid->maxOutput ? pid->maxOutput : output;
	output = output < pid->minOutput ? pid->minOutput : output;


	
	pid->previousFeedback = feedback;
	pid->previousError = error;
	return output;
}