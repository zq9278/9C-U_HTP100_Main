
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
extern bool emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
float weight0 = 0; // test_variable
//  void UART1_CMDHandler(recept_data_p msg) { // 屏幕发出命令以后的动作
//   uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
//   float data = (float)msg->data;
//   switch (cmd_type) {
//   /*收到屏幕加热开始*/
//   case 0x1041:
//     break;
//   /*收到屏幕加热停止*/
//   case 0x1030:
//     break;
//   /*收到屏幕脉动开始*/
//   case 0x1005:
//     //HX711_Init();
//     // ForceRawSet = data * 1.0;
//     // switch ((int)data) {
//     // case 150:
//     //   MotorPID_speed.Kp = 100;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 250:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 350:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 450:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.4;
//     //   break;
//     // case 550:
//     //   MotorPID_speed.Kp = 70;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // default:
//     //   break;
//     // }
//     break;
//   /*收到屏幕脉动停止*/
//   case 0x1034:
//     break;
//   /*收到屏幕自动开始*/
//   case 0x1037:
//   /*收到屏幕自动停止*/
//   case 0x1038:
//     break;
//   case 0x8900:
//     // switch (currentState) {
//     // case 0x01:
//     //   AT24CXX_Read(0x20, &heat_counttem, 2);
//     //   heat_count = (heat_counttem[0] << 8) | heat_counttem[1];
//     //   if (heat_count == 65535) {
//     //     heat_count = 0;
//     //   }
//     //   heat_count += 1;
//     //   heat_counttem[0] = (heat_count >> 8) & 0xFF; // 高字节
//     //   heat_counttem[1] = heat_count & 0xFF;        // 低字节
//     //   AT24CXX_Write(0x20, &heat_counttem, 2);
//     //   ScreencurrentState_count((float)heat_count);
//     //   break;
//     // case 0x02:
//     //   AT24CXX_Read(0x22, &pulsation_counttem, 2);
//     //   pulsation_count = (pulsation_counttem[0] << 8) |
//     pulsation_counttem[1];
//     //   if (heat_count == 65535) {
//     //     pulsation_count = 0;
//     //   }
//     //   pulsation_count += 1;
//     //   pulsation_counttem[0] = (pulsation_count >> 8) & 0xFF; // 高字节
//     //   pulsation_counttem[1] = pulsation_count & 0xFF;        // 低字节
//     //   AT24CXX_Write(0x22, &pulsation_counttem, 2);
//     //   ScreencurrentState_count((float)pulsation_count);
//     //   break;
//     // case 0x03:
//     //   AT24CXX_Read(0x24, &auto_counttem, 2);
//     //   auto_count = (auto_counttem[0] << 8) | auto_counttem[1];
//     //   if (auto_count == 65535) {
//     //     auto_count = 0;
//     //   }
//     //   auto_count += 1;
//     //   auto_counttem[0] = (auto_count >> 8) & 0xFF; // 高字节
//     //   auto_counttem[1] = auto_count & 0xFF;        // 低字节
//     //   AT24CXX_Write(0x24, &auto_counttem, 2);
//     //   ScreencurrentState_count((float)auto_count);
//     //}
//   default:
//     break;
//   }
//  }
void UART1_CMDHandler(recept_data_p msg) {
  // 提取命令类型
  uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
  float data = (float)msg->data;
  // 根据命令类型切换状态
  switch (cmd_type) {
  case 0x8900:
    if (currentState == STATE_HEAT) {
      heat_finish = 1;
    }
    if (currentState == STATE_PRESS) {
      press_finish = 1;
    }
    if (currentState == STATE_AUTO) {
      auto_finish = 1;
    }
    break;
  /* 屏幕加热开始 */
  case 0x1041:
    currentState = STATE_PRE_HEAT; // 切换到预加热状态
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
        emergency_stop =
            true; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
     
    }
     currentState = STATE_OFF; // 从加热停止回到关闭状态

    break;
  /* 屏幕挤压开始 */
  case 0x1005:
    currentState = STATE_PRE_PRESS; // 切换到预挤压状态
    // data=((data * 1.0 / 88.4) / 9.8)*1000;
    MotorPID.setpoint = data;

    weight0 = ADS1220_ReadPressure();           // 读取初始压力值
    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // 将数据发送到队列
    break;
  /* 屏幕挤压停止 */
  case 0x1034:
    if (currentState == STATE_PRESS) {

      if (press_finish ==
          0) { // 如果挤压任务未完成，且工作在正式模式下（非预热阶段）则设置紧急停止标志
        emergency_stop =
            true; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // 清除第0位// 通知停止挤压任务
    }
    currentState = STATE_OFF; // 从挤压停止回到关闭状态

    break;
  /* 屏幕自动模式开始 */
  case 0x1037:
    currentState = STATE_PRE_AUTO; // 切换到预自动模式
    HeatPWM(1);                    // 启动加热PWM
     //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // 将数据发送到队列
    MotorPID.setpoint = data;
    weight0 = ADS1220_ReadPressure();           // 读取初始压力值
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
        emergency_stop =
            true; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
      }
    }

    currentState = STATE_OFF; // 从自动模式回到关闭状态
    break;

  /* 其他未知命令 */
  default:
    break;
  }
}

void command_parsing(uart_data *received_data) { // 区分调试命令和屏幕工作命令
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
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????????
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
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
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

void Eye_twitching_invalid_master(float count) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x92;
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
