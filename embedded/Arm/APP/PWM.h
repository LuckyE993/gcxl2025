#ifndef __PWM_H
#define __PWM_H
#include "stm32f10x.h"

void PWM_Init(void);

void PWM_SetCompare1(uint16_t Compare_1);
void PWM_SetCompare2(uint16_t Compare_2);
void PWM_SetCompare3(uint16_t Compare_3);

#endif
