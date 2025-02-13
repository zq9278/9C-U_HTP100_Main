
#include "main.h"
#include "pid.h"
#include "ws2812b.h"
#include <stdint.h>
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern volatile SystemState_t currentState;
extern PID_TypeDef HeatPID;
extern PID_TypeDef MotorPID;
extern uint8_t emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
extern prepare_data my_prepare_data;
extern osMessageQueueId_t prepare_dataHandle;
extern osSemaphoreId_t BUTTON_SEMAPHOREHandle; // 按键信号量句柄
extern uint8_t soft_button;
uint16_t save_prepare,set_prepare;
void UART1_CMDHandler(recept_data_p msg) {
    if (msg == NULL) {
        printf("Error: msg is NULL!\n");
        return;
    }

  uint16_t hot_count,crimp_count,auto_count;
  // 提取命令类型
  uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
  float data = (float)msg->data;
  // 根据命令类型切换状态
  switch (cmd_type) {
  case 0x8900:
    if (currentState == STATE_HEAT) {
      heat_finish = 1;
      hot_count = AT24CXX_ReadOrWriteZero(0x00);
      AT24CXX_WriteUInt16(0x00,hot_count+1);
      my_prepare_data.cmd_type_low = 0xA0;
      my_prepare_data.value = hot_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    }
    if (currentState == STATE_PRESS) {
      press_finish = 1;
      crimp_count = AT24CXX_ReadOrWriteZero(0x02);
      AT24CXX_WriteUInt16(0x02,crimp_count+1);
      my_prepare_data.cmd_type_low = 0xA1;
      my_prepare_data.value = crimp_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    }
    if (currentState == STATE_AUTO) {
      auto_finish = 1;
      auto_count = AT24CXX_ReadOrWriteZero(0x04);
      AT24CXX_WriteUInt16(0x04,auto_count+1);
      my_prepare_data.cmd_type_low = 0xA2;
      my_prepare_data.value = auto_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    }
    break;
  /* 屏幕加热开始 */
  case 0x1041:
    currentState = STATE_PRE_HEAT; // 切换到预加热状态
    emergency_stop=0;
    HeatPWM(1);                    // 启动加热PWM
    //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // 将数据发送到队列
    osEventFlagsSet(HEAT_ONHandle, (1 << 0)); // 设置第0位 // 启动加热任务
    break;
  /* 屏幕加热停止 */
  case 0x1030:
    if (currentState == STATE_HEAT || STATE_PRE_HEAT) {

      HeatPWM(0); // 启动加热PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止加热任务
      if ((heat_finish == 0) &&
          (currentState ==
           STATE_HEAT)) { // 如果加热任务未完成，且工作在正式模式下（非预热阶段）则设置紧急停止标志
        emergency_stop =1; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
     
    }
     currentState = STATE_OFF; // 从加热停止回到关闭状态

    break;
  /* 屏幕挤压开始 */
  case 0x1005:
    currentState = STATE_PRE_PRESS; // 切换到预挤压状态
    emergency_stop=0;
    // data=((data * 1.0 / 88.4) / 9.8)*1000;
    MotorPID.setpoint = data;

    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // 将数据发送到队列
    break;
  /* 屏幕挤压停止 */
  case 0x1034:
    if (currentState == STATE_PRESS) {

      if (press_finish ==
          0) { // 如果挤压任务未完成，且工作在正式模式下（非预热阶段）则设置紧急停止标志
        emergency_stop =1; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止挤压任务
    }
    currentState = STATE_OFF; // 从挤压停止回到关闭状态

    break;
  /* 屏幕自动模式开始 */
  case 0x1037:
    //printf("1\n");
    currentState = STATE_PRE_AUTO; // 切换到预自动模式
    emergency_stop=0;
    HeatPWM(1);                    // 启动加热PWM
     //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // 将数据发送到队列
    MotorPID.setpoint = data;

    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // 将数据发送到队列
    osEventFlagsSet(HEAT_ONHandle, (1 << 0));   // 设置第0位 // 启动加热任务
    break;
  /* 屏幕自动模式停止 */
  case 0x1038:
    // if (currentState == STATE_AUTO || STATE_PRE_AUTO) {

    //   if ((auto_finish == 0) &&
    //       (currentState ==
    //        STATE_AUTO)) { //
    //        如果自动任务未完成，且工作在正式模式下（非自动阶段）则设置紧急停止标志
    //     emergency_stop =
    //         true; // 设置紧急停止标志(1 << 0)); // 清除第0位//
    //         通知停止加热任务
    //   }

    //   HeatPWM(0); // 启动加热PWM
    //   osEventFlagsClear(HEAT_ONHandle,
    //                     (1 << 0)); // 清除第0位// 通知停止加热任务
    //   osEventFlagsClear(PRESS_ONHandle,
    //                     (1 << 0)); // 清除第0位// 通知停止加热任务
    //   currentState = STATE_OFF;    // 从自动模式回到关闭状态
    // }

    if (currentState == STATE_PRE_AUTO) {
      HeatPWM(0); // 启动加热PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止加热任务
    }
    if (currentState == STATE_AUTO) {
      HeatPWM(0); // 启动加热PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止加热任务
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止加热任务
      if (auto_finish ==0) { // 如果自动任务未完成，且工作在正式模式下（非自动阶段）则设置紧急停止标志
        emergency_stop =1; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
    }

    currentState = STATE_OFF; // 从自动模式回到关闭状态
    break;
  case 0x1040:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // 通知按键任务;

    break;
  case 0x1006:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // 通知按键任务;
    break;
  case 0x1036:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // 通知按键任务;
    break;

  /* 其他未知命令 */
  default:
    break;
  }
}
uint32_t a;
void UART1_CMDHandler_prepare(prepare_data_p msg) {
    /*test
     *
     * */

  uint16_t hot_count,crimp_count,auto_count;
  uint16_t prepare_press_pre,prepare_temperature_pre,prepare_time_pre;
  // 提取命令类型
  uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
  uint16_t data = (uint16_t)msg->value;
    // 初始化实例的成员变量
    my_prepare_data.cmd_head_high = 0x6A;
    my_prepare_data.cmd_head_low = 0xA6;
    my_prepare_data.cmd_type_high = 0x00;
    my_prepare_data.end_high = 0xFF;
    my_prepare_data.end_low = 0xFF;
  // 根据命令类型切换状态
  switch (cmd_type) {
  case 0x1042://设置预设值具体数值时需要先读取,小箭头
    save_prepare=data;
    switch (save_prepare) {
    case 0:
      prepare_press_pre = 150;
      prepare_temperature_pre = 42;
      prepare_time_pre = 1;
      break;
    case 1:
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x08);
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x0A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x0C);
      break;
    case 2:
       prepare_temperature_pre= AT24CXX_ReadOrWriteZero(0x10);
            prepare_press_pre  = AT24CXX_ReadOrWriteZero(0x12);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x14);
      break;
    case 3:
        prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x18);
            prepare_press_pre  = AT24CXX_ReadOrWriteZero(0x1A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x1C);
      break;
    }
    my_prepare_data.cmd_type_low = 0xA9;
    my_prepare_data.value = prepare_press_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    my_prepare_data.cmd_type_low = 0xA8;
    my_prepare_data.value = prepare_temperature_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    my_prepare_data.cmd_type_low = 0xAA;
    my_prepare_data.value = prepare_time_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    break;
  case 0x1044://选择预设值，大框选择
    set_prepare=data;
    AT24CXX_WriteUInt16(0xFC,set_prepare);
    switch (set_prepare) {
    case 0:
      prepare_press_pre = 150;
      prepare_temperature_pre = 42;
      prepare_time_pre = 1;
      break;
    case 1:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x08);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x0A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x0C);
      break;
    case 2:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x10);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x12);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x14);
      break;
    case 3:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x18);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x1A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x1C);
      break;
    }
    my_prepare_data.cmd_type_low = 0xA4;
    my_prepare_data.value = prepare_press_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    my_prepare_data.cmd_type_low = 0xA5;
    my_prepare_data.value = prepare_temperature_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
    my_prepare_data.cmd_type_low = 0xA6;
    my_prepare_data.value = prepare_time_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列

    break;
  case 0x1039://预设温度 prepare temprature
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x08,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x10,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x18,data);
      break;
    }
      break;
  case 0x1040://预设压力 prepare press
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x0A,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x12,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x1A,data);
      break;
    }
    break;
  case 0x1041://预设时间 prepare time
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x0c,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x14,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x1c,data);
      break;
    }
    break;
  case 0x1043:
    AT24CXX_WriteUInt16(0xf8,data);
    break;
  case 0x1046:
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    AT24C02_WriteAllBytes(0xff);//清理ee存储
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    break;
    /* 其他未知命令 */
  default:
    break;
  }
}
void command_parsing(uart_data *received_data) { // 区分调试命令和屏幕工作命令
//    a++;
//    printf("a=%u\n",a);
  // 确认帧尾是否合法
  // if (received_data->buffer[received_data->length - 2] != 0xFF ||
  //     received_data->buffer[received_data->length - 1] != 0xFF) {
  //   return; // 非法帧，直接返回
  // }
  // 提取命令类型（高字节和低字节）
  uint16_t cmd_type =
      (received_data->buffer[0] << 8) |
      received_data->buffer[1]; // 第一位和第二位判断是不是调试参数
  // 解析命令
  switch (cmd_type) {
  case 0x5aa5: { // 命令类型 0x9000 - 处理 UART1 数据
    UART1_CMDHandler(received_data->buffer);

    break;
  }
  case 0x6aa6: { // 命令类型 0x6aa6 - 处理 UART1 数据
    UART1_CMDHandler_prepare(received_data->buffer);

    break;
  }
  case 0x7aa7: { // 命令类型 0x0200 - 更新 PID 参数（速度控制）
    recept_data_debug_p press_pid_data =
        (recept_data_debug *)received_data->buffer; // 解析数据
    xQueueSend(PRESS_DATAHandle, &MotorPID, 0);     // 将数据发送到队列
    PID_Init(&MotorPID, press_pid_data->p, press_pid_data->i, press_pid_data->d,
             5000, -5000, 50000, -50000, press_pid_data->setpoint);

    break;
  }

  case 0x9aa9: { // 命令类型 0x0400 - 更新 PID 参数（加热控制）
    recept_data_debug_p heat_pid_data =
        (recept_data_debug *)(received_data->buffer); // 解析数据
    PID_Init(&HeatPID, heat_pid_data->p, heat_pid_data->i, heat_pid_data->d,
             500, 0, 255, 0, heat_pid_data->setpoint);
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // 将数据发送到队列
  } break;

  default:
    // 未知命令，忽略
    break;
  }
}
extern UART_HandleTypeDef huart2;
uint8_t usart1_tx = 1;
void ScreenUpdateForce(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.data = value;
  if (currentState == STATE_PRESS) {
    pData.cmd_type_low = 0x05;
  } else if (currentState == STATE_AUTO) {
    pData.cmd_type_low = 0x47;
  }

  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData));
  }
}
void ScreenUpdateTemperature(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  if (currentState == STATE_HEAT || currentState == STATE_PRE_HEAT) {
    pData.cmd_type_low = 0x41;
  }
  if (currentState == STATE_AUTO || currentState == STATE_PRE_AUTO) {
    pData.cmd_type_low = 0x37;
  }
  pData.data = value;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData));
  }
}
void ScreenUpdateSOC(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x50;
  pData.data = value;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData)); // ??????
  }
}
void ScreenWorkModeQuit(void) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x51;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
      taskENTER_CRITICAL();
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????????
      taskEXIT_CRITICAL();
  }
}
void ScreenTimerStart(void) {

  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x52;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
      taskENTER_CRITICAL();
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
                         taskEXIT_CRITICAL();
  }
}
void Eye_twitching_invalid(void) {

  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x91;
  pData.cmd_type_low = 0x00;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}
void Eye_twitching_invalid_master(prepare_data_p myprepare_data) {
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)myprepare_data,
                         sizeof(prepare_data)); // ???????????????
  }
}
void ScreenWorkMode_count(float count) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x90;
  pData.cmd_type_low = 0x00;
  pData.data = count;
  pData.end_high = 0xff; // 帧尾
  pData.end_low = 0xff;  // 帧尾
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}
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