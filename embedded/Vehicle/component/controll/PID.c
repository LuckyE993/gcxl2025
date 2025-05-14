#include "PID.h"
#include "stdlib.h"
#include "stdio.h"

uint16_t POS_MAX = 8000;

void PID_init(PID *pid, float init_pid[3])
{

	pid->kp = init_pid[0];
	pid->ki = init_pid[1];
	pid->kd = init_pid[2];
	pid->output = 0.0f;
	pid->err[0] = pid->err[1] = pid->err[2] = 0.0f;
	pid->integral = 0.0f;
	//	pid_speed->kperr[0]=pid_speed->kperr[1]=pid_speed->kperr[2]=0.0f;

	//	pid_pos->kperr[0]=pid_pos->kperr[1]=pid_pos->kperr[2]=0.0f;
}

float PID_Cal(PID *pid, float fdbck, float set)
{

	pid->err[2] = pid->err[1];
	pid->err[1] = pid->err[0];
	pid->target = set;
	pid->feedback = fdbck;
	pid->err[0] = set - fdbck;
	pid->integral += pid->err[0];
	pid->output = pid->kp * pid->err[0] + pid->ki * pid->integral + pid->kd * (pid->err[0] - pid->err[1]);

	// 电流限幅

	return pid->output;
}

float PID_Cal_w(PID *pid, float fdbck, float set)
{
	if (set == 0)
	{
		if (fdbck > 270 && fdbck <= 360)
			fdbck = fdbck - 360;
		pid->err[2] = pid->err[1];
		pid->err[1] = pid->err[0];
		pid->target = set;
		pid->feedback = fdbck;
		pid->err[0] = set - fdbck;
		pid->integral += pid->err[0];
		pid->output = pid->kp * pid->err[0] + pid->ki * pid->integral + pid->kd * (pid->err[0] - pid->err[1]);
	}
	else
	{

		pid->err[2] = pid->err[1];
		pid->err[1] = pid->err[0];
		pid->target = set;
		pid->feedback = fdbck;

		if (__fabs(set - fdbck) > __fabs(set + 360.0f - fdbck))
			pid->err[0] = set + 360.0f - fdbck;
		else
			pid->err[0] = set - fdbck;
		pid->integral += pid->err[0];
		pid->output = pid->kp * pid->err[0] + pid->ki * pid->integral + pid->kd * (pid->err[0] - pid->err[1]);
	}
	// 电流限幅

	if (pid->output > 4000)
		pid->output = 4000;
	if (pid->output < -4000)
		pid->output = -4000;
	return pid->output;
}

float PID_Cal_Pos(PID *pid_0, PID *pid_1, float fdbck, float set, uint8_t pid_pid_pos_1_flag)
{

	if (__fabs(set - fdbck) > 15 && pid_pid_pos_1_flag == 1)
	{
		pid_0->err[2] = pid_0->err[1];
		pid_0->err[1] = pid_0->err[0];
		pid_0->target = set;
		pid_0->feedback = fdbck;
		pid_0->err[0] = set - fdbck;
		pid_0->integral += pid_0->err[0];
		pid_0->output = pid_0->kp * pid_0->err[0] + pid_0->ki * pid_0->integral + pid_0->kd * (pid_0->err[0] - pid_0->err[1]);
		return pid_0->output;
	}
	else
	{
		pid_1->err[2] = pid_1->err[1];
		pid_1->err[1] = pid_1->err[0];
		pid_1->target = set;
		pid_1->feedback = fdbck;
		pid_1->err[0] = set - fdbck;
		pid_1->integral += pid_1->err[0];
		pid_1->output = pid_1->kp * pid_1->err[0] + pid_1->ki * pid_1->integral + pid_1->kd * (pid_1->err[0] - pid_1->err[1]);
		return pid_1->output;
	}

	// 电流限幅
}

float PID_Cal_Pos1(PID *pid_1, float fdbck, float set)
{

	pid_1->err[2] = pid_1->err[1];
	pid_1->err[1] = pid_1->err[0];
	pid_1->target = set;
	pid_1->feedback = fdbck;
	pid_1->err[0] = set - fdbck;
	pid_1->integral += pid_1->err[0];
	pid_1->output = pid_1->kp * pid_1->err[0] + pid_1->ki * pid_1->integral + pid_1->kd * (pid_1->err[0] - pid_1->err[1]);
	return pid_1->output;

	// 电流限幅
}

void Current_Limit(float motor_current_output[4])
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (motor_current_output[i] > CURRENT_MAX)
			motor_current_output[i] = CURRENT_MAX;
		if (motor_current_output[i] < -CURRENT_MAX)
			motor_current_output[i] = -CURRENT_MAX;
	}
}

void Current_Limit_left(float *motor_current_output)
{

	if (*motor_current_output > CURRENT_MAX)
		*motor_current_output = CURRENT_MAX;
	if (*motor_current_output < -CURRENT_MAX)
		*motor_current_output = -CURRENT_MAX;
}
void Speed_Limit_left(float *motor_Speed_output)
{

	if (*motor_Speed_output > SPEED_LEFT)
		*motor_Speed_output = SPEED_LEFT;
	if (*motor_Speed_output < -SPEED_LEFT)
		*motor_Speed_output = -SPEED_LEFT;
}

void Speed_Limit(float motor_pos_output[4])
{
	for (uint8_t i = 0; i < 4; i++)
	{
		if (motor_pos_output[i] > POS_MAX)
			motor_pos_output[i] = POS_MAX;
		if (motor_pos_output[i] < -POS_MAX)
			motor_pos_output[i] = -POS_MAX;
		//        if (motor_pos_output[i] > 0&&motor_pos_output[i] < POS_MIN)
		//            motor_pos_output[i] = POS_MIN;
		//        if (motor_pos_output[i] < 0&&motor_pos_output[i] > -POS_MIN)
		//            motor_pos_output[i] = -POS_MIN;
	}
}

void Speed_Limit1(float motor_pos_output1[4], float motor_pos_output2, float motor_pos_output3, uint8_t id, float yaw)
{

	if (__fabs(motor_pos_output2 - motor_pos_output3) > 360)
		POS_MAX = 0.8 * POS_MAX;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 720 && __fabs(motor_pos_output2 - motor_pos_output3) > 600)
	//			 POS_MAX = 3000;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 600 && __fabs(motor_pos_output2 - motor_pos_output3) > 480)
	//			 POS_MAX = 2400;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 480 && __fabs(motor_pos_output2 - motor_pos_output3) > 360)
	//			 POS_MAX = 1800;
	if (__fabs(motor_pos_output2 - motor_pos_output3) < 360 && __fabs(motor_pos_output2 - motor_pos_output3) > 240)
		POS_MAX = 0.5 * POS_MAX;
	if (__fabs(motor_pos_output2 - motor_pos_output3) < 240 && __fabs(motor_pos_output2 - motor_pos_output3) > 120)
		POS_MAX = 0.375 * POS_MAX;

	if (motor_pos_output1[id] > POS_MAX)
		motor_pos_output1[id] = POS_MAX - yaw;
	if (motor_pos_output1[id] < -POS_MAX)
		motor_pos_output1[id] = -POS_MAX - yaw;
}

float Speed_Limit2(float motor_pos_output1, float motor_pos_output2, float motor_pos_output3)
{

	if (__fabs(motor_pos_output2 - motor_pos_output3) > 360)
		POS_MAX = MAX;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 720 && __fabs(motor_pos_output2 - motor_pos_output3) > 600)
	//			 POS_MAX = 3000;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 600 && __fabs(motor_pos_output2 - motor_pos_output3) > 480)
	//			 POS_MAX = 2400;
	//		if(__fabs(motor_pos_output2 - motor_pos_output3) < 480 && __fabs(motor_pos_output2 - motor_pos_output3) > 360)
	//			 POS_MAX = 1800;
	if (__fabs(motor_pos_output2 - motor_pos_output3) < 360 && __fabs(motor_pos_output2 - motor_pos_output3) > 240)
		POS_MAX = 2200;
	if (__fabs(motor_pos_output2 - motor_pos_output3) < 240 && __fabs(motor_pos_output2 - motor_pos_output3) > 120)
		POS_MAX = 1500;

	if (motor_pos_output1 > POS_MAX)
		return POS_MAX;
	if (motor_pos_output1 < -POS_MAX)
		return -POS_MAX;
	return motor_pos_output1;
}
