/*
 * uart.h
 *
 *  Created on: Aug 16, 2024
 *      Author: hlqth
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include "stdbool.h"
#include "rcc.h"

void uart_UART1_GPIO_config(void);

void uart_UART1_config(void);

bool uart_UART1_transmit(uint8_t *data, uint8_t len, uint32_t timeout);

bool uart_UART1_receive(uint8_t *data, uint8_t len, uint32_t timeout);

#endif /* INC_UART_H_ */
