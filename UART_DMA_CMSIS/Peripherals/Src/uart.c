/*
 * uart.c
 *
 *  Created on: Aug 16, 2024
 *      Author: hlqth
 */


#include "uart.h"

void uart_UART1_GPIO_config(void)
{
    //PA9->Tx, PA10->Rx
    //Enable PortA clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    //Mode to AF (UART1)
    GPIOA->CRH &= ~(GPIO_CRH_CNF9);
    GPIOA->CRH |= (GPIO_CRH_CNF9_1);

    GPIOA->CRH &= ~(GPIO_CRH_CNF10);
	GPIOA->CRH |= (GPIO_CRH_CNF10_0);

	//Output max 10MHz
	GPIOA->CRH &= ~(GPIO_CRH_MODE9_1);
	GPIOA->CRH |= (GPIO_CRH_MODE9_0);
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);

	//Map PA9, PA10 mapped to UART1
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->MAPR 	 &=	~(AFIO_MAPR_USART1_REMAP);
}

void uart_UART1_config(void)
{
	//Enable UART1 Clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	//Enable UART Transmit
	USART1->CR1 |= USART_CR1_TE;
	//Enable UART Receive
	USART1->CR1 |= USART_CR1_RE;
	//Parity to Even
	USART1->CR1 &= ~(USART_CR1_PS);
	//Parity Control Enable
	USART1->CR1 &= ~(USART_CR1_PCE);
	//Word length 8 bits
	USART1->CR1 &= ~(USART_CR1_M);
	//Stop bits 1
	USART1->CR2 &= ~(USART_CR2_STOP);
	//Disable HW Flow Control
	USART1->CR3 &= ~(USART_CR3_CTSE);
	USART1->CR3 &= ~(USART_CR3_RTSE);
	//Set baud rate 115200
	USART1->BRR = 0;
	USART1->BRR |= (39UL << 4);
	USART1->BRR |= (1UL << 0);
	//Clear some flag and enable
	USART1->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART1->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
	//Enable UART
	USART1->CR1 |= USART_CR1_UE;
}

bool uart_UART1_transmit(uint8_t *data, uint8_t len, uint32_t timeout)
{
	//Wait on TXE to start transmit
	//Write to DR as TXE flag is High (Tx buffer empty)
	uint8_t dataIdx = 0;
	uint32_t startTick = HAL_GetTick();
	while(dataIdx < len)
	{
		if(USART1->SR & USART_SR_TXE)
		{
			USART1->DR = data[dataIdx];
			dataIdx++;
		}
		else
		{
			if((HAL_GetTick()-startTick) >= timeout)
			return false;
		}
	}
	//wait for busy flag
	while(USART1->SR & USART_SR_TC)
	{
		if((HAL_GetTick()-startTick) >= timeout)
		return false;
	}
	return true;
}

bool uart_UART1_receive(uint8_t *data, uint8_t len, uint32_t timeout)
{
	uint8_t dataRemain = len;
	uint32_t startTick = HAL_GetTick();
	while(dataRemain > 0)
	{
		if(USART1->SR & USART_SR_RXNE)
		{
			*data++ = (uint8_t)(USART1->DR & 0xFF);
			dataRemain--;
		}
		else
		{
			if((HAL_GetTick()-startTick) >= timeout)
			return false;
		}
	}
	return true;
}



