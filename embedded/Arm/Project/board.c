#include "board.h"
#include "fifo.h"
#include "string.h"

__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;


void clock_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);

	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;				/* 复用推挽输出 */
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;					/* 浮空输入 */
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART5, &USART_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	UART5->SR; UART5->DR;
	USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(UART5, ENABLE);
}

void UART5_IRQHandler(void)
{
	__IO uint16_t i = 0;

/**********************************************************
***	串口接收中断
**********************************************************/
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		// 未完成一帧数据接收，数据进入缓冲队列
		fifo_enQueue((uint8_t)UART5->DR);

		// 清除串口接收中断
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}

/**********************************************************
***	串口空闲中断
**********************************************************/
	else if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
	{
		// 先读SR再读DR，清除IDLE中断
		UART5->SR; UART5->DR;

		// 提取一帧数据命令
		rxCount = fifo_queueLength(); for(i=0; i < rxCount; i++) { rxCmd[i] = fifo_deQueue(); }

		// 一帧数据接收完成，置位帧标志位
		rxFrameFlag = true;
	}
}

void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0;
	
	for(i=0; i < len; i++) { usart_SendByte(cmd[i]); }
}

void usart_SendByte(uint16_t data)
{
//	__IO uint16_t t0 = 0;
//	
//	UART5->DR = (data & (uint16_t)0x01FF);

//	while(!(UART5->SR & USART_FLAG_TXE))
//	{
//		++t0; if(t0 > 8000)	{	return; }
//	}
	
	USART_SendData(UART5, data);
	while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
	{}

}

