/*
 * uart.c
 *
 *  Created on: Aug 15, 2024
 *      Author: hlqth
 */

#include "uart.h"


char rxBuf[RXBUF_SIZE];
char txBuf[TXBUF_SIZE] = {'1', '2', '3', '4', '\0'};
char *p = txBuf;

uint32_t volatile timeout_uart = 0;

void configure_uart(void) {
    // Cấu hình UART (USART1)
    LL_USART_SetBaudRate(USART1, 64000000, 115200);
    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
    LL_USART_Enable(USART1);
}

void configure_dma(void)
{
    // Cấu hình DMA cho UART1 TX
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)&USART1->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, (uint32_t)txBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, TXBUF_SIZE);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

    // Cấu hình DMA cho UART1 RX
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&USART1->DR);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)rxBuf);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, RXBUF_SIZE);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);
}

void start_transmission(void)
{
    // Bật UART DMA TX
    LL_USART_EnableDMAReq_TX(USART1);
    // Kích hoạt DMA
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

void start_receive(void)
{
    // Bật UART DMA TX
    LL_USART_EnableDMAReq_RX(USART1);
    // Kích hoạt DMA
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

void transmit_data(void)
{
	while (*p)
	{
		timeout_uart = 2000;
        while (!LL_USART_IsActiveFlag_TXE(USART1))
        {
        	if (timeout_uart == 0) return;
        }

		LL_USART_TransmitData8( USART1, *p++);

		while (!LL_USART_IsActiveFlag_TC(USART1))
        {
			if (timeout_uart == 0) return;
        }
	}
}

void receive_data(void)
{
    // Chỉ thị số lượng byte đã nhận
    uint8_t received_count = 0;

    // �?�?c dữ liệu cho đến khi nhận đủ số byte
    while (received_count < RXBUF_SIZE)
    {
        // Kiểm tra nếu có dữ liệu đến
        if (LL_USART_IsActiveFlag_RXNE(USART1)) {
            // �?�?c dữ liệu từ USART và lưu vào rxbuf
        	rxBuf[received_count] = LL_USART_ReceiveData8(USART1);
            received_count++;
        }
    }
}
