/*
 * 文件: interface_uart.c
 * 说明: CRC、RTT 日志输出、USART2 DMA 屏幕协议发送。
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "SEGGER_RTT.h"
#include "interface_uart.h"
#include "UserApp.h"

#define LOG_BUFFER_INDEX 0
#define LOG_FORMAT_BUFFER_SIZE 192

uint16_t Calculate_CRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

#ifdef __GNUC__
int __io_putchar(int ch)
{
    SEGGER_RTT_PutChar(LOG_BUFFER_INDEX, (char)ch);
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    SEGGER_RTT_PutChar(LOG_BUFFER_INDEX, (char)ch);
    return ch;
}
#endif

static void RTT_LogV(const char *color, const char *level, const char *format, va_list args)
{
    char buffer[LOG_FORMAT_BUFFER_SIZE];

    vsnprintf(buffer, sizeof(buffer), format, args);
    SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, color);
    SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, level);
    SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, buffer);
    SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, RTT_CTRL_RESET);
}

void LOG(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        RTT_LogV("", "", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        RTT_LogV("", "", format, args);
        va_end(args);
        xSemaphoreGive(logSemaphore);
    }
}

void LOGI(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_GREEN, "[INFO] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_GREEN, "[INFO] ", format, args);
        va_end(args);
        xSemaphoreGive(logSemaphore);
    }
}

void LOGW(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_YELLOW, "[WARN] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_YELLOW, "[WARN] ", format, args);
        va_end(args);
        xSemaphoreGive(logSemaphore);
    }
}

void LOGE(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_RED, "[ERROR] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        RTT_LogV(RTT_CTRL_TEXT_BRIGHT_RED, "[ERROR] ", format, args);
        va_end(args);
        xSemaphoreGive(logSemaphore);
    }
}

void LOG_ISR(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    RTT_LogV(RTT_CTRL_TEXT_BRIGHT_CYAN, "[ISR] ", format, args);
    va_end(args);
}

void LOG_CLEAR(void)
{
    SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, RTT_CTRL_CLEAR);
}

void RTT_VAR(const char *format, ...)
{
    char buffer[LOG_FORMAT_BUFFER_SIZE * 2];
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, buffer);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        SEGGER_RTT_WriteString(LOG_BUFFER_INDEX, buffer);
        xSemaphoreGive(logSemaphore);
    }
}

USART2_DMA_HandleTypeDef huart2_dma;
extern UART_HandleTypeDef huart2;

void USART2_DMA_Init(void)
{
    SEGGER_RTT_Init();

    usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    if (usart2_dmatxSemaphore != NULL) {
        xSemaphoreGive(usart2_dmatxSemaphore);
    }
    huart2_dma.activeBuffer = 0;
}

void USART2_DMA_Send(uint8_t *data, uint16_t length)
{
    if (xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdTRUE) {
        memcpy(huart2_dma.txBuffer[huart2_dma.activeBuffer], data, length);
        HAL_UART_Transmit_DMA(&huart2, huart2_dma.txBuffer[huart2_dma.activeBuffer], length);
        huart2_dma.activeBuffer ^= 1;
    }
}
