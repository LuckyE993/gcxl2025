#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f10x.h"
#include "fifo.h"

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

void clock_init(void);
void usart_SendByte(uint16_t data);

void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
void UART5_IRQHandler(void);


#endif
