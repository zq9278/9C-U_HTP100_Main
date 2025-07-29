
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
int fputc(int ch,FILE *f)//�ض���printf
{
  uint8_t temp[1]={ch};
  HAL_UART_Transmit(&huart2,temp,1,2);
  return ch;
}
#endif /* __GNUC__ */
// ��װ����־���� (֧�ֿɱ����)
// void LOG(const char *format, ...) {
//     if (logSemaphore != NULL) {
//         if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
//             va_list args;
//             va_start(args, format);
//             vprintf(format, args);  // ���� printf (���ض��� UART)
//             va_end(args);
//             xSemaphoreGive(logSemaphore);  // �ͷ��ź���
//         }
//     }
// }
void LOG(const char *format, ...)
{
    if (logSemaphore == NULL) return; // �ź���δ��ʼ��ֱ�ӷ���

    if (xSemaphoreTake(logSemaphore, portMAX_DELAY) == pdTRUE) {
        char buf[128];   // �ɸ���ʵ����Ҫ��������
        va_list args;
        va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args);
        va_end(args);

        HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);

        xSemaphoreGive(logSemaphore);
    }
}

// ר�Ÿ��ж�/�쳣��
void LOG_ISR(const char *format, ...)
{
    char buf[128]; // ������Ҫ������������С
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}

 USART2_DMA_HandleTypeDef huart2_dma;
extern UART_HandleTypeDef huart2;

void USART2_DMA_Init(void) {
    // ������ֵ�ź��� (��ʼΪ����״̬)
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    if (usart2_dmatxSemaphore != NULL) {
        xSemaphoreGive(usart2_dmatxSemaphore);  // ��ʼ�ͷ�һ��
    }
    huart2_dma.activeBuffer = 0;

}

void USART2_DMA_Send(uint8_t *data, uint16_t length) {
    // �ȴ�ǰһ�η������

    if (xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdTRUE) {

//        while(xSemaphoreTake(usart2_dmatxSemaphore, portMAX_DELAY) == pdFALSE) {
//        }
        // �������ݵ���ǰ������
        memcpy(huart2_dma.txBuffer[huart2_dma.activeBuffer], data, length);
        // ����DMA����
        HAL_UART_Transmit_DMA(&huart2, huart2_dma.txBuffer[huart2_dma.activeBuffer], length);
        // �л������� (0 -> 1, 1 -> 0)
        huart2_dma.activeBuffer ^= 1;
    }
}