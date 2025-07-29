
#include <string.h>
#include "interface_uart.h"
#include "UserApp.h"
#include <stdio.h>
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
extern UART_HandleTypeDef huart1;

/* Variables */


//extern UART_HandleTypeDef huart1;
#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,100);

    return ch;
}
#else
int fputc(int ch,FILE *f)//重定向printf
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart2,temp,1,2);
  return ch;
}
#endif /* __GNUC__ */
// 封装的日志函数 (支持可变参数)
// void LOG(const char *format, ...) {
//     if (logSemaphore != NULL) {
//         if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
//             va_list args;
//             va_start(args, format);
//             vprintf(format, args);  // 调用 printf (会重定向到 UART)
//             va_end(args);
//             xSemaphoreGive(logSemaphore);  // 释放信号量
//         }
//     }
// }
void LOG(const char *format, ...)
{
    if (logSemaphore == NULL) return; // 信号量未初始化直接返回

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        char buf[128];   // 可根据实际需要调整长度
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);

        xSemaphoreGive(logSemaphore);
    }
}

// 专门给中断/异常用
void LOG_ISR(const char *format, ...)
{
    char buf[128]; // 根据需要调整缓冲区大小
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}

 USART2_DMA_HandleTypeDef huart2_dma;
extern UART_HandleTypeDef huart2;

void USART2_DMA_Init(void) {
    // 创建二值信号量 (初始为可用状态)
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    if (usart2_dmatxSemaphore != NULL) {
        xSemaphoreGive(usart2_dmatxSemaphore);  // 初始释放一次
    }
    huart2_dma.activeBuffer = 0;

}

void USART2_DMA_Send(uint8_t *data, uint16_t length) {
    // 等待前一次发送完成

    if (xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdTRUE) {

//        while(xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdFALSE) {
//        }
        // 拷贝数据到当前缓冲区
        memcpy(huart2_dma.txBuffer[huart2_dma.activeBuffer], data, length);
        // 启动DMA传输
        HAL_UART_Transmit_DMA(&huart2, huart2_dma.txBuffer[huart2_dma.activeBuffer], length);
        // 切换缓冲区 (0 -> 1, 1 -> 0)
        huart2_dma.activeBuffer ^= 1;
    }
}