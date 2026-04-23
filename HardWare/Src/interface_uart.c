/*
 * 鏂囦欢: interface_uart.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
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





#ifdef __GNUC__
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,100);

    return ch;
}
#else
int fputc(int ch,FILE *f)
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart2,temp,1,2);
  return ch;
}
#endif












void LOG(const char *format, ...)
{
    if (logSemaphore == NULL) return;

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        char buf[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);

        xSemaphoreGive(logSemaphore);
    }
}


void LOG_ISR(const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}

 USART2_DMA_HandleTypeDef huart2_dma;
extern UART_HandleTypeDef huart2;

/**
 * @brief USART2_DMA_Init 鍑芥暟瀹炵幇銆? */
void USART2_DMA_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    if (usart2_dmatxSemaphore != NULL) {
        xSemaphoreGive(usart2_dmatxSemaphore);
    }
    huart2_dma.activeBuffer = 0;

}

/**
 * @brief USART2_DMA_Send 鍑芥暟瀹炵幇銆? * @param data 鍙傛暟銆? * @param length 鍙傛暟銆? */
void USART2_DMA_Send(uint8_t *data, uint16_t length) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */


    if (xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdTRUE) {




        memcpy(huart2_dma.txBuffer[huart2_dma.activeBuffer], data, length);

        HAL_UART_Transmit_DMA(&huart2, huart2_dma.txBuffer[huart2_dma.activeBuffer], length);

        huart2_dma.activeBuffer ^= 1;
    }
}


