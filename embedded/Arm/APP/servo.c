#include "servo.h"
#include "ring_buffer.h"
#include "fashion_star_uart_servo.h"
#include "PWM.h"
#include "math.h"
#include "board.h"
#include "motor.h"

uint8_t MQ[9] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5A};

#define pai 3.141593
#define abs(x) ((x > 0) ? (x) : (-x))
#define take_x 50
#define take_y 35
#define back_x 48
#define back_y 15
#define put3_y2_1 5
#define put3_y2_2 5
#define back_1 61
#define back_2 -36.5
#define back_3 -120.5
#define grab_y1_1 60
#define put3_delay1 300
#define put3_delay2 300

#define back_delay 250
#define take_delay 100

int angle_front = 0; // 机械臂朝向角度值
int angle_back = 167;

int angle_open =145; // 爪子开合角度值
int angle_close = 100;
int angle_open_max = 80;

float l_1 = 130; // 大臂
float l_2 = 140; // 小臂
float angle_bef = 0;

uint8_t QRCode[6] = {1, 2, 3, 1, 2, 3}; // 顺序数组
uint8_t MV[6] = {1, 2, 3, 1, 3, 2};		// 物料颜色顺序(从左到右)
int P2[3] = {1, 2, 3};

int t[9] = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5A}; // 32通讯数据组

float target_x = 50; //              (90,100,225,1)
float target_y = 100;
float target_A = 8;
float target_D = 60;
float real_x = 225;
float real_y = 100;
float real_A = 8;
float real_D = 60;
float B = 60;
float C = 0;

float change_x;
float change_y;
float change_A;

float Bias_x = 0, PWM_x = 0, Integral_bias_x = 0, Last_Bias_x = 0;
float Bias_y = 0, PWM_y = 0, Integral_bias_y = 0, Last_Bias_y = 0;

// 函数功能 ：舵机PWM初始化
int sign = 1;

// Usart_DataTypeDef* servo_usart = &usart1;

// 舵机控制相关的参数
// 舵机的ID号
uint8_t servo_id = 0;
// 舵机的目标角度
// 舵机角度在-135度到135度之间, 精确到小数点后一位
float angle = 0;
// 时间间隔ms
// 可以尝试修改设置更小的时间间隔，例如500ms
uint16_t interval;
// 目标转速
float velocity = 500;
// 加速时间
uint16_t t_acc = 100;
// 减速时间
uint16_t t_dec = 100;
// 舵机执行功率 mV 默认为0
uint16_t power = 0;
// 设置舵机角度的时候, 是否为阻塞式
// 0:不等待 1:等待舵机旋转到特定的位置;
uint8_t wait = 1;
// 读取的角度
float angle_read;

char i = 1;

void servo_1(uint8_t state) // 机械臂处的舵机（使用此函数前需将舵机先调零）
{
	if (state == 0)
	{

		float compare = angle_front * (2000 / 270) + 500;
		PWM_SetCompare1(compare);
		delay_ms(10);
	}
	else if (state == 1)
	{
		float compare = (angle_back) * (2000 / 270) + 500;
		PWM_SetCompare1(compare);
		delay_ms(10);
	}
}

void servo_2(uint8_t state) // 爪子处的舵机（使用此函数前需将舵机先调至135）
{

	if (state == 2)
	{

		float compare = (angle_open_max) * (2000 / 270) + 500;
		PWM_SetCompare2(compare);
		delay_ms(10);
	}
	if (state == 1)
	{

		float compare = (angle_close) * (2000 / 270) + 500;
		PWM_SetCompare2(compare);
		delay_ms(10);
	}
	else if (state == 0)
	{

		float compare = (angle_open) * (2000 / 270) + 500;
		PWM_SetCompare2(compare);
		delay_ms(10);
	}
}

void motor_1(float angle) // 电机驱动（ID为 1 or 2    1号电机为升降，2号电机为转盘处）
{

	Emm_V5_Pos_Control(1, 1, 50, 5, 3200 * (angle / 360), 1, 0);
}

void motor_2(float angle)
{
	// 声明整型变量用于储存转换后的角度值
	uint32_t angle_int;

	// 边界检查 - 避免负值
	if (angle < 1)
	{
		angle = 1; // 设置最小安全值为1度
	}

	// 将输入角度转换为电机步数
	// 电机步进为3200步/圈，减去1度是为了校正偏差
	angle = 3200 * ((angle - 1) / 360);

	// 确保不会出现负值
	if (angle < 0)
	{
		angle = 0;
	}

	// 使用lroundf将浮点数转换为最接近的整数
	angle_int = (uint32_t)lroundf(angle);

	// 控制2号电机位置
	// 参数: 电机ID, 方向, 速度, 加速时间, 位置, 等待标志, 上电标志
	Emm_V5_Pos_Control(2, 0, 100, 60, angle_int, 1, 0);

	// 短暂延时确保命令被处理
	delay_ms(50);
}

void servo_init(void) // 机械臂初始化函数
{
	servo_1(real_A);
	servo_2(B);
}

void normal(void) // 机械臂初始状态设置
{
}

// void zero(void)
//{
//     FSUS_SetServoAngleByVelocity(servo_usart, 0,0 , velocity,  t_acc,   t_dec,   power,  wait = 1);
//     FSUS_SetServoAngleByVelocity(servo_usart, 1,0 , velocity,  t_acc,   t_dec,   power,  wait = 1);
//     FSUS_SetServoAngleByVelocity(servo_usart, 2,0 , velocity,  t_acc,   t_dec,   power,  wait = 1);
// }

void normal_1(int angle, u8 x, int y) // 机械臂初始状态设置
{
}

u8 Round1Part1target1(void) // 物料拿取识别位置
{

	return 1;
}

void Round1Part1target2(void) // 到达视觉矫正位置
{
}

void Round1Part1target3(void) // 到达巡线矫正位置
{
}

void armreback(void) // 机械臂准备动作放置打到物料
{
}

void Round1Part1Grab1(void) //
{
	//  收(90,100,096,1)
}

void Round1Part1Back1(void) //
{
}

void Round1Part1Back2(void) //
{
}

void Round1Part1Back3(void) //
{
}

void Round2Part3Put1(void) //
{
}

void Round2Part3Put2(void) //
{
}

void Round2Part3Put3(void) //
{
}

void Round1Part2Take1(void) //
{
}

void Round1Part2Take2(void) //
{
}

void Round1Part2Take3(void) //
{
}

void Round1Part2Put1(void) //
{
}

void Round1Part2Put2(void) //
{
}

void Round1Part2Put3(void) //
{
}

void Round1Part2Grab1(void) //
{
}

void Round1Part2Grab2(void) //
{
}

void Round1Part2Grab3(void) //
{
}

void Round1Part2Back1(void) //
{
}

void Round1Part2Back2(void) //
{
}

void Round1Part2Back3(void) //
{
}

void Grab1(void) // 原盘抓取
{
}

void Grab2_you(void) // 原盘抓取
{
}

void Grab3_zuo(void) // 原盘抓取
{
}
