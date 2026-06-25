/*
 * 文件: communication.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef COMMUNICATION
#define COMMUNICATION
#include "stm32g0xx_hal.h"

#define UART_RX_BUFFER_SIZE  256
#define UART_REC_LEN 256
#define UART_BUFFER_QUANTITY 5
#define FRAME_HEADER_BYTE1   0x5A
#define FRAME_HEADER_BYTE2   0xA5
#define FRAME_TAIL_BYTE1     0xFF
#define FRAME_TAIL_BYTE2     0xFF

#define COMM_CMD_SET_LANGUAGE       0x1060
#define COMM_RESP_LANGUAGE          0x00AB
#define COMM_RESP_SCREEN_BOOT_ACK   0x00AD
#define COMM_RESP_SOFTWARE_VERSION  0x2060
#define COMM_RESP_FAULT_CODE        0x2070
extern uint8_t factory_mode;
typedef struct __attribute__((packed))
{
    uint8_t buffer[UART_REC_LEN];
    uint16_t length;
} uart_data;
/**
 * @brief __attribute__ 函数实现。
 * @return 返回值见函数实现。
 */
typedef struct __attribute__((packed)) {
    uint8_t cmd_head_high;
    uint8_t cmd_head_low;
    uint8_t frame_length;
    uint8_t cmd_type_high;
    uint8_t cmd_type_low;
    float data;
    uint16_t crc;
    uint8_t end_high;
    uint8_t end_low;
} recept_data, *recept_data_p;



/**
 * @brief __attribute__ 函数实现。
 * @return 返回值见函数实现。
 */
typedef struct __attribute__((packed)){
    uint8_t cmd_head_high;
    uint8_t cmd_head_low;
    uint8_t frame_length;
    float p;
    float i;
    float d;
    float setpoint;
    uint16_t crc;
    uint8_t end_high;
    uint8_t end_low;
} recept_data_debug, *recept_data_debug_p;
/**
 * @brief __attribute__ 函数实现。
 * @return 返回值见函数实现。
 */
typedef struct __attribute__((packed)){
  uint8_t cmd_head_high;
  uint8_t cmd_head_low;
    uint8_t frame_length;
  uint8_t cmd_type_high;
  uint8_t cmd_type_low;
  uint16_t value;
    uint16_t crc;
  uint8_t end_high;
  uint8_t end_low;
} prepare_data, *prepare_data_p;

typedef struct __attribute__((packed)){
  uint8_t cmd_head_high;
  uint8_t cmd_head_low;
  uint8_t frame_length;
  uint8_t cmd_type_high;
  uint8_t cmd_type_low;
  uint32_t value;
  uint16_t crc;
  uint8_t end_high;
  uint8_t end_low;
} uint32_data, *uint32_data_p;

void command_parsing(uart_data *received_data);
void UART1_CMDHandler(recept_data_p msg);
void ScreenUpdateSOC(float value);
void ScreenSendBootAck(void);
void ScreenSendLanguageSetting(void);
void ScreenSendSoftwareVersion(void);
void ScreenWorkModeQuit(void);
void ScreenTimerStart(void);
void ScreenTimerStop(void) ;
void NEW_EYE(void);
void Eye_twitching_invalid(void) ;
void Eye_twitching_invalid_master(prepare_data_p myprepare_data);
void ScreenWorkMode_count(float count);
void EYE_checkout(float data);
        void ScreenUpdateForce(float value);
void ScreenUpdateTemperature(float value);
void ScreenUpdateHeatPower(float value);
void ScreenUpdateHeatLoadStatus(float value);
void Serial_data_stream_parsing(uart_data *frameData);



#endif
