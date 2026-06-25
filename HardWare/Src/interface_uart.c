/*
 * 文件: interface_uart.c
 * 说明: CRC、USART1 日志输出、USART2 DMA 屏幕协议发送。
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "interface_uart.h"
#include "usart.h"
#include "UserApp.h"

#define LOG_FORMAT_BUFFER_SIZE 192
#define LOG_UART_TIMEOUT_MS 50

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

static void UART_LogWrite(const char *data, size_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)data, (uint16_t)length, LOG_UART_TIMEOUT_MS);
}

#ifdef __GNUC__
int __io_putchar(int ch)
{
    uint8_t data = (uint8_t)ch;

    HAL_UART_Transmit(&huart1, &data, 1U, LOG_UART_TIMEOUT_MS);
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t data = (uint8_t)ch;

    HAL_UART_Transmit(&huart1, &data, 1U, LOG_UART_TIMEOUT_MS);
    return ch;
}
#endif

static void UART_LogV(const char *level, const char *format, va_list args)
{
    char buffer[LOG_FORMAT_BUFFER_SIZE];

    vsnprintf(buffer, sizeof(buffer), format, args);
    UART_LogWrite(level, strlen(level));
    UART_LogWrite(buffer, strlen(buffer));
}

void LOG(const char *format, ...)
{
    va_list args;

    if (format == NULL) {
        return;
    }

    if (logSemaphore == NULL) {
        va_start(args, format);
        UART_LogV("", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        UART_LogV("", format, args);
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
        UART_LogV("[INFO] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        UART_LogV("[INFO] ", format, args);
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
        UART_LogV("[WARN] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        UART_LogV("[WARN] ", format, args);
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
        UART_LogV("[ERROR] ", format, args);
        va_end(args);
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        UART_LogV("[ERROR] ", format, args);
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
    UART_LogV("[ISR] ", format, args);
    va_end(args);
}

void LOG_CLEAR(void)
{
    static const char clearSequence[] = "\r\n";

    UART_LogWrite(clearSequence, sizeof(clearSequence) - 1U);
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
        UART_LogWrite(buffer, strlen(buffer));
        return;
    }

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        UART_LogWrite(buffer, strlen(buffer));
        xSemaphoreGive(logSemaphore);
    }
}

USART2_DMA_HandleTypeDef huart2_dma;
extern UART_HandleTypeDef huart2;

typedef struct {
    uint16_t length;
    uint8_t data[USART2_TX_BUFFER_SIZE];
} ScreenTxFrame_t;

static QueueHandle_t screenTxQueueHandle;

void USART2_DMA_Init(void)
{
    if (usart2_dmatxSemaphore == NULL) {
        usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    }
    if (usart2_dmatxSemaphore != NULL) {
        xSemaphoreGive(usart2_dmatxSemaphore);
    }
    huart2_dma.activeBuffer = 0;
}

static void ScreenTx_SendDma(uint8_t *data, uint16_t length)
{
    if (xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdTRUE) {
        memcpy(huart2_dma.txBuffer[huart2_dma.activeBuffer], data, length);
        HAL_UART_Transmit_DMA(&huart2, huart2_dma.txBuffer[huart2_dma.activeBuffer], length);
        huart2_dma.activeBuffer ^= 1;
    }
}

void ScreenTx_Init(void)
{
    if (screenTxQueueHandle == NULL) {
        screenTxQueueHandle = xQueueCreate(SCREEN_TX_QUEUE_LENGTH, sizeof(ScreenTxFrame_t));
    }
    configASSERT(screenTxQueueHandle != NULL);
}

HAL_StatusTypeDef ScreenTx_Post(const uint8_t *data, uint16_t length, TickType_t timeout)
{
    ScreenTxFrame_t frame;

    if ((data == NULL) || (length == 0U) || (length > USART2_TX_BUFFER_SIZE)) {
        return HAL_ERROR;
    }

    if (screenTxQueueHandle == NULL) {
        return HAL_ERROR;
    }

    frame.length = length;
    memcpy(frame.data, data, length);
    if (xQueueSend(screenTxQueueHandle, &frame, timeout) != pdTRUE) {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

void ScreenTxTask(void *argument)
{
    ScreenTxFrame_t frame;

    (void)argument;

    for (;;) {
        if (xQueueReceive(screenTxQueueHandle, &frame, portMAX_DELAY) == pdTRUE) {
            ScreenTx_SendDma(frame.data, frame.length);
        }
    }
}
