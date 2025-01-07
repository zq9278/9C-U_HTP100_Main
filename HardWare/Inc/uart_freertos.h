/*
 * @Author: zhangqi 
 * @Date: 2024-12-26 16:24:57 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 10:15:32
 */

#ifndef USRT_FREERTOS_H
#define USRT_FREERTOS_H
#include "stm32g0xx_hal.h"
#define UART_REC_LEN 40 
typedef struct
{
    uint8_t buffer[UART_REC_LEN]; // the lengh of received array
    uint16_t length;     // ����ʵ�ʳ���
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
//recept_data�ṹ�����ͣ���ʾһ�������Ľṹ������� . ���ʳ�Ա
//recept_data_p�ṹ��ָ�����ͣ���ʾһ��ָ��recept_data�ṹ������ָ�롣�� -> ���ʳ�Ա

//���Խṹ��
typedef struct __attribute__((packed)){
    uint8_t cmd_head_high; // 
    uint8_t cmd_head_low;  // 
    float p;
    float i;
    float d;
    float setpoint;
    uint8_t end_high; // 
    uint8_t end_low;  // 
} recept_data_debug, *recept_data_debug_p;
void command_parsing(uart_data *received_data);
void UART1_CMDHandler(recept_data_p msg);
void ScreenUpdateSOC(float value);
void ScreenWorkModeQuit(void);
void ScreenTimerStart(void);
void Eye_twitching_invalid(void) ;
void Eye_twitching_invalid_master(float count);
void ScreenWorkMode_count(float count);

void ScreenUpdateForce(float value);
void ScreenUpdateTemperature(float value);






#endif // !USRT_FREERTOS_H