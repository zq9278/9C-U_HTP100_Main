/*
 * @Author: zhangqi 
 * @Date: 2024-12-26 16:24:57 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 10:15:32
 */

#ifndef COMMUNICATION
#define COMMUNICATION
#include "stm32g0xx_hal.h"

#define UART_RX_BUFFER_SIZE  256 // 串口接收缓冲区大小
#define UART_REC_LEN 256//从缓冲区复制到数组里
#define UART_BUFFER_QUANTITY 5   // 使用双缓冲
#define FRAME_HEADER_BYTE1   0x5A // 帧头第一个字节
#define FRAME_HEADER_BYTE2   0xA5 // 帧头第二个字节
#define FRAME_TAIL_BYTE1     0xFF // 帧尾第一个字节
#define FRAME_TAIL_BYTE2     0xFF // 帧尾第二个字节
typedef struct __attribute__((packed))
{
    uint8_t buffer[UART_REC_LEN]; // the lengh of received array
    uint16_t length;     // 数据实际长度
} uart_data;
typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  //
    uint8_t frame_length; // 帧长度0x0d
    uint8_t cmd_type_high; // 
    uint8_t cmd_type_low;  // 
    float data;
    uint16_t crc; // CRC 校验
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data, *recept_data_p;
//recept_data结构体类型，表示一个完整的结构体对象。用 . 访问成员
//recept_data_p结构体指针类型，表示一个指向recept_data结构体对象的指针。用 -> 访问成员
//调试结构体
typedef struct __attribute__((packed)){
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  //
    uint8_t frame_length; // 帧长度0x17
    float p;
    float i;
    float d;
    float setpoint;
    uint16_t crc; // CRC 校验
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data_debug, *recept_data_debug_p;
typedef struct __attribute__((packed)){
  uint8_t cmd_head_high; //
  uint8_t cmd_head_low;  //
    uint8_t frame_length; // 帧长度0x0b
  uint8_t cmd_type_high; //
  uint8_t cmd_type_low;  //
  uint16_t value;
    uint16_t crc; // CRC 校验
  uint8_t end_high; //
  uint8_t end_low;  //
} prepare_data, *prepare_data_p;
void command_parsing(uart_data *received_data);
void UART1_CMDHandler(recept_data_p msg);
void ScreenUpdateSOC(float value);
void ScreenWorkModeQuit(void);
void ScreenTimerStart(void);
void Eye_twitching_invalid(void) ;
void Eye_twitching_invalid_master(prepare_data_p myprepare_data);
void ScreenWorkMode_count(float count);

void ScreenUpdateForce(float value);
void ScreenUpdateTemperature(float value);
void Serial_data_stream_parsing(uart_data *frameData);



#endif // !COMMUNICATION