/*
 * @Author: zhangqi 
 * @Date: 2024-12-26 16:24:57 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-26 19:41:57
 */

#ifndef USRT_FREERTOS_H
#define USRT_FREERTOS_H
#include "main.h"
#define UART_REC_LEN 30 
typedef struct
{
    uint8_t buffer[UART_REC_LEN]; // the lengh of received array
    uint16_t length;     // 数据实际长度
} uart_data;


typedef struct {
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  // 
    uint8_t cmd_type_high; // 
    uint8_t cmd_type_low;  // 
    float data;
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data, *recept_data_p;
//recept_data结构体类型，表示一个完整的结构体对象。用 . 访问成员
//recept_data_p结构体指针类型，表示一个指向recept_data结构体对象的指针。用 -> 访问成员

void command_parsing(uart_data *received_data);
void UART1_CMDHandler(recept_data_p msg);












#endif // !USRT_FREERTOS_H