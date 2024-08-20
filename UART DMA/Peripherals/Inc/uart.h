/*
 * uart.h
 *
 *  Created on: Aug 15, 2024
 *      Author: hlqth
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "string.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_gpio.h"

// Kích thước rxBuf và txBuf
#define RXBUF_SIZE 8
#define TXBUF_SIZE 5

extern char rxBuf[RXBUF_SIZE];
//Config thông số UART (Baudrate, parity,...)
void configure_uart(void);

//Config UART DMA
void configure_dma(void);

//Transmit Function
void start_transmission(void);

void transmit_data(void);

//Receive Function
void start_receive(void);

void receive_data(void);

#endif /* INC_UART_H_ */
