#ifndef MYTASK_H
#define MYTASK_H
#include "struct_typedef.h"
#include "PID.h"

#define D 76
#define Pi 3.141592653589
#define mm2angle (360/(Pi*D))
#define red 35
#define green 155
#define blue 275

typedef struct{
	float x_set;
	float y_set; 
	float yaw_set;
	float tar_pos[4];
	float yaw_real;
}chassis_move_t;

struct move_i
{
	float x;
	float y;
	float yaw;
};

struct pos_correct
{
	float x;
	float y;
	float yaw;
};

// Define the speed target structure type
typedef struct
{
    float vx;   // X direction target speed
    float vy;   // Y direction target speed
    float vyaw; // Yaw angle target speed
} speed_target_t;

// Declare the speed target as extern so it can be accessed from other files
extern speed_target_t speed_target;
#define target_vx speed_target.vx
#define target_vy speed_target.vy
#define target_vyaw speed_target.vyaw
// Function to safely set the chassis speed
void set_chassis_speed(float vx, float vy, float vyaw);

void mode_switch(uint8_t mode);

void move_set_interrupt(float x, float y, float yaw_t, float acc);

void update_movement(void);

extern uint8_t color_shunxu[6];
extern float yaw;
extern float motor_current_output[4];
extern float CAM_X,CAM_Y;
void motor_task (void const *pvParameters);
void gimbal_task (void const *pvParameters);

void main_task (void const *pvParameters);
void move_set(float x,float y,float yaw_t,float acc);
void move_set_without_yaw(float x,float y,float acc);
void arm_move(float angle_1,float angle_2,uint8_t state_1,uint8_t state_2);//state_1：驱动机械臂下舵机（0为朝前，1为朝后）  state_2：驱动爪子下舵机（0为收拢，1为张开）  angle_1：机械臂上下圈数  angle_2：转盘处转动角度
void shijue(uint8_t mode);
void goal_task(void);
void pos_jiaozhun(void);
void mode_choose(uint8_t mode);    //模式选择
void corret(uint8_t state,float high,uint8_t times) ;             //视觉定位
void my_destory(void);

#endif

