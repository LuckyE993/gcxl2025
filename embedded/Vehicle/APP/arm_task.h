#ifndef  ARM_TASK_H
#define  ARM_TASK_H
#include "stm32f4xx.h"                  // Device header

#define red_area 1
#define green_area 2
#define blue_area 3


void Round1Part1task1(uint8_t i);           //到达转盘处
void Round1Part2task1(uint8_t i);           //到达粗加工区放
void Round1Part2task2(uint8_t i);           //粗加工区拿
void Round1Part3task(void);           //第一次精加工区放
void Round2Part3task(void);           //第二次精加工区码垛
void zhuaqu_1(uint8_t color);        //圆盘处抓取
void fangzhi_1(uint8_t color);           //正常放置
void zhuaqu_2(uint8_t color);       //暂存区抓取
void fangzhi_2(uint8_t color);           //码垛放置


void get_half(uint8_t i);

void put_half(uint8_t i);
void put_final(uint8_t i);

#endif
