/********************************************************
 * 写入用户自定义数据 - 写入舵机角度上限与下限，并开启舵机角度限制开关
 * 	PB6.7.8 作为PWM输出通道
 ********************************************************/
#include "stm32f10x.h"
#include "usart.h"
#include "sys_tick.h"
#include "fashion_star_uart_servo.h"
#include "PWM.h"
#include "servo.h"
#include "math.h"
#include "pic.h"
#include "motor.h"
#include "board.h"

// 使用串口1空闲
// <接线说明>
// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
// STM32F103 GND 	  <----> 串口舵机转接板 GND
// STM32F103 V5 	  <----> 串口舵机转接板 5V
// <注意事项>
// 使用前确保已设置usart.h里面的USART1_ENABLE为1
// 设置完成之后, 将下行取消注释
// Usart_DataTypeDef* servoUsart = &usart1;

// 使用串口2                       //作为视觉通信端口
// <接线说明>
// STM32F103 PA2(Tx,黄线) <----> USB转TTL Rx
// STM32F103 PA3(Rx，蓝线) <----> USB转TTL Tx
// STM32F103 GND 	 <----> USB转TTL GND

// STM32F103 V5 	 <----> USB转TTL 5V (可选)
// <注意事项>
// 使用前确保已设置usart.h里面的USART2_ENABLE为1
// Usart_DataTypeDef* loggingUsart = &usart2;
//
//   串口3作为与C板

// STM32F103 PB10(Tx)  <---->  底盘 (Rx)
// STM32F103 PB11(Rx) <---->  底盘 (Tx)

//   串口4作为舵机控制
// STM32F103 PB6  <----> 舵机1，机械臂处的舵机
// STM32F103 PB7  <----> 舵机2，爪子处的舵机
// STM32F103 PB8  <----> 舵机2，转盘处的舵机
// STM32F103 GND 	  <----> 扫码模块 GND
// STM32F103 V5 	  <----> 扫码模块 7.4V

//   串口5作为步进电机通信
// STM32F103 PC12(Tx)  <----> 扫码模块 Rx
// STM32F103 PD2(Rx) <----> 扫码模块 Tx
// STM32F103 GND 	  <----> 扫码模块 GND
// STM32F103 V5 	  <----> 扫码模块 5V

// 重定向c库函数printf到串口，重定向后可使用printf函数

// 在全局变量区域添加以下变量，用于记录上一次的值
float last_MOTMOR_1 = -1; // 初始化为不可能的值确保首次执行
float last_MOTMOR_2 = -1;
uint8_t last_state_1 = 255; // 初始化为不可能的值确保首次执行
uint8_t last_state_2 = 255;

uint32_t motor_1_H, motor_2_H, motor_1_L, motor_2_L;
uint8_t state_1, state_2;
uint8_t usart3RecvBuf1[10] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE};

float MOTMOR_1, MOTMOR_2;

void shijue_2(void);

int main(void)
{

	// 嘀嗒定时器初始化
	SysTick_Init();
	PWM_Init();
	// 串口初始化
	Usart_Init();
	clock_init();
	delay_ms(2000);
	//	PWM_SetCompare1(90 * (2000 / 270)+500);
	//	PWM_SetCompare2( 110 * (2000 / 270)+500);
	// Emm_V5_Pos_Control(2, 0, 300, 120, 12800, true, false);
	while (1)
	{

		if (Finish_flag_U2 == 1)
		{
			Finish_flag_U2 = 0;
			if (usart2RecvBuf[1] == 0x02)
			{
				// 在main函数的if (usart2RecvBuf[2] == 0x00)代码块内作如下修改:
				if (usart2RecvBuf[2] == 0x00)
				{
					motor_1_H = usart2RecvBuf[3];
					motor_1_L = usart2RecvBuf[4];
					MOTMOR_1 = (motor_1_H << 8 | motor_1_L);

					// 只有当值发生变化时才执行电机1控制
					if (MOTMOR_1 != last_MOTMOR_1)
					{
						motor_1(MOTMOR_1);
						last_MOTMOR_1 = MOTMOR_1;
						delay_ms(10);
					}

					motor_2_H = usart2RecvBuf[5];
					motor_2_L = usart2RecvBuf[6];
					MOTMOR_2 = (motor_2_H << 8 | motor_2_L);

					// 只有当值发生变化时才执行电机2控制
					if (MOTMOR_2 != last_MOTMOR_2)
					{
						// 确保MOTMOR_2不为0(或小于1的值)以避免之前提到的问题
						if (MOTMOR_2 < 1)
						{
							MOTMOR_2 = 0;
						}
						motor_2(MOTMOR_2);
						last_MOTMOR_2 = MOTMOR_2;
						delay_ms(10);
					}

					state_1 = usart2RecvBuf[7];
					// 只有当值发生变化时才执行舵机1控制
					if (state_1 != last_state_1)
					{
						servo_1(state_1);
						last_state_1 = state_1;
						delay_ms(10);
					}

					state_2 = usart2RecvBuf[8];
					// 只有当值发生变化时才执行舵机2控制
					if (state_2 != last_state_2)
					{
						servo_2(state_2);
						last_state_2 = state_2;
						delay_ms(10);
					}

					// Clear usart2RecvBuf to prevent repeated execution
					for (int i = 0; i < 10; i++)
					{
						usart2RecvBuf[i] = 0;
					}
				}
				else if (usart2RecvBuf[2] == 0x01)
				{
					Serial_SendArray(USART3, usart2RecvBuf, 10);
					// Clear usart2RecvBuf to prevent repeated execution
					for (int i = 0; i < 10; i++)
					{
						usart2RecvBuf[i] = 0;
					}
				}
			}
			else
			{
				Serial_SendArray(USART3, usart2RecvBuf, 10);
				// Clear usart2RecvBuf to prevent repeated execution
				for (int i = 0; i < 10; i++)
				{
					usart2RecvBuf[i] = 0;
				}
			}
		}

		if (Finish_flag_U3 == 1)
		{
			Finish_flag_U3 = 0;

			Serial_SendArray(USART2, usart3RecvBuf, 10);
		}
	}
}