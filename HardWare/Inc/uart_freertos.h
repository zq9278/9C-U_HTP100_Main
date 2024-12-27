/*
 * @Author: zhangqi 
 * @Date: 2024-12-26 16:24:57 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-26 19:41:57
 */

#ifndef USRT_FREERTOS_H
#define USRT_FREERTOS_H
#include "main.h"
#define UART_REC_LEN 20 
typedef struct
{
    uint8_t buffer[UART_REC_LEN]; // the lengh of received array
    uint16_t length;     // 数据实际长度
} uart_data;




#endif // !USRT_FREERTOS_H