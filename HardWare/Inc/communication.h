/*
 * @Author: zhangqi 
 * @Date: 2024-12-26 16:24:57 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 10:15:32
 */

#ifndef COMMUNICATION
#define COMMUNICATION
#include "stm32g0xx_hal.h"

#define UART_RX_BUFFER_SIZE  256 // ���ڽ��ջ�������С
#define UART_REC_LEN 256//�ӻ��������Ƶ�������
#define UART_BUFFER_QUANTITY 5   // ʹ��˫����
#define FRAME_HEADER_BYTE1   0x5A // ֡ͷ��һ���ֽ�
#define FRAME_HEADER_BYTE2   0xA5 // ֡ͷ�ڶ����ֽ�
#define FRAME_TAIL_BYTE1     0xFF // ֡β��һ���ֽ�
#define FRAME_TAIL_BYTE2     0xFF // ֡β�ڶ����ֽ�
typedef struct __attribute__((packed))
{
    uint8_t buffer[UART_REC_LEN]; // the lengh of received array
    uint16_t length;     // ����ʵ�ʳ���
} uart_data;
typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  //
    uint8_t frame_length; // ֡����0x0d
    uint8_t cmd_type_high; // 
    uint8_t cmd_type_low;  // 
    float data;
    uint16_t crc; // CRC У��
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data, *recept_data_p;
//recept_data�ṹ�����ͣ���ʾһ�������Ľṹ������� . ���ʳ�Ա
//recept_data_p�ṹ��ָ�����ͣ���ʾһ��ָ��recept_data�ṹ������ָ�롣�� -> ���ʳ�Ա
//���Խṹ��
typedef struct __attribute__((packed)){
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  //
    uint8_t frame_length; // ֡����0x17
    float p;
    float i;
    float d;
    float setpoint;
    uint16_t crc; // CRC У��
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data_debug, *recept_data_debug_p;
typedef struct __attribute__((packed)){
  uint8_t cmd_head_high; //
  uint8_t cmd_head_low;  //
    uint8_t frame_length; // ֡����0x0b
  uint8_t cmd_type_high; //
  uint8_t cmd_type_low;  //
  uint16_t value;
    uint16_t crc; // CRC У��
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