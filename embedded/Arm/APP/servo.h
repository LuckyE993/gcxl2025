#ifndef __SERVO_H
#define __SERVO_H
#include "stm32f10x.h"
#include "usart.h"
#include "sys_tick.h"



//extern Usart_DataTypeDef* servo_usart;


void ServoInit(void);

void servo_1(uint8_t state);  //��е�۳�����
void servo_2(uint8_t state);   //צ�ӿ��϶��
void motor_1(float angle); //�����õ��
void motor_2(float angle);   //ת�̴��ĵ��

//void turntable(u8 color);

void servo_init(void);

void implement_A(void);
void implement_x(void);
void implement_y(void);
void implement_D(void);
void solution(void);
void normal(void);


u8 Round1Part1target1(void);   ///
void Round1Part1target2(void);
void Round1Part1target3(void);

void Round1Part1Grab1(void);   ///

void Round1Part1Back1(void);   //
void Round1Part1Back2(void);   //
void Round1Part1Back3(void);   //
void zero(void);

void Round2Part3Put1(void);//
void Round2Part3Put2(void);//
void Round2Part3Put3(void);//

void Round1Part3Put1(void);//
void Round1Part3Put2(void);//
void Round1Part3Put3(void);//

void Round1Part2Take1(void);  ///
void Round1Part2Take2(void);  ///
void Round1Part2Take3(void);  ///

void Round1Part2Put1(void);   ///
void Round1Part2Put2(void);   ///
void Round1Part2Put3(void);   ///

void Round1Part2Grab1(void);  ///
void Round1Part2Grab2(void);  ///
void Round1Part2Grab3(void);  ///

void Round1Part2Back1(void);  ///
void Round1Part2Back2(void);  ///
void Round1Part2Back3(void);  ///



								
void armreback(void);		


void normal_1(int angle,u8 x ,int y)  ;											
#endif

