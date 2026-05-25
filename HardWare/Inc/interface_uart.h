/*
 * 文件: interface_uart.h
 * 说明: CRC、USART1 日志输出、USART2 DMA 屏幕协议发送接口。
 */
#ifndef __INTERFACE_UART_H__
#define __INTERFACE_UART_H__

#include <stdarg.h>
#include "main.h"

int __io_putchar(int ch) __attribute__((weak));
int __io_getchar(void) __attribute__((weak));

uint16_t Calculate_CRC(uint8_t *data, uint16_t length);

void LOG(const char *format, ...);
void LOGI(const char *format, ...);
void LOGW(const char *format, ...);
void LOGE(const char *format, ...);
void LOG_ISR(const char *format, ...);
void LOG_CLEAR(void);
void RTT_VAR(const char *format, ...);

#define USART2_TX_BUFFER_SIZE 256

typedef struct {
    uint8_t txBuffer[2][USART2_TX_BUFFER_SIZE];
    uint8_t activeBuffer;
} USART2_DMA_HandleTypeDef;

extern USART2_DMA_HandleTypeDef huart2_dma;

void USART2_DMA_Init(void);
void USART2_DMA_Send(uint8_t *data, uint16_t length);

#endif
