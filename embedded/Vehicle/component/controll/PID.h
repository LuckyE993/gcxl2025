#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

#define CURRENT_MAX 10000.0f
#define CURRENT_MIN 50.0f
#define SPEED_LEFT 6000.0f
#define MAX 8000;

#define POS_MIN 200.0f

typedef struct
{

	float target;
	float feedback;

	float kp;
	float ki;
	float kd;

	float integral;
	float err[3];

	float output;

} PID;

void PID_init(PID *pid, float init_pid[3]);
float PID_Cal(PID *pid, float fdbck, float set);
float PID_Cal_Pos(PID *pid_0, PID *pid_1, float fdbck, float set, uint8_t pid_pid_pos_1_flag);
float PID_Cal_Pos1(PID *pid_1, float fdbck, float set); // 用来区别模糊pid的位置环
float PID_Cal_w(PID *pid, float fdbck, float set);

void Current_Limit(float motor_current_output[4]);
void Speed_Limit(float motor_pos_output[4]);
void Current_Limit_left(float *motor_current_output);
void Speed_Limit_left(float *motor_Speed_output);

void Speed_Limit1(float motor_pos_output1[4], float motor_pos_output2, float motor_pos_output3, uint8_t id, float yaw);
float Speed_Limit2(float motor_pos_output1, float motor_pos_output2, float motor_pos_output3);

#endif
