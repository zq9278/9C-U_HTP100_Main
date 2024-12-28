#include "uart_freertos.h"
#include <stdint.h>
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern volatile SystemState_t currentState;
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
//     // switch (WorkMode) {
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
//     //   ScreenWorkMode_count((float)heat_count);
//     //   break;
//     // case 0x02:
//     //   AT24CXX_Read(0x22, &pulsation_counttem, 2);
//     //   pulsation_count = (pulsation_counttem[0] << 8) | pulsation_counttem[1];
//     //   if (heat_count == 65535) {
//     //     pulsation_count = 0;
//     //   }
//     //   pulsation_count += 1;
//     //   pulsation_counttem[0] = (pulsation_count >> 8) & 0xFF; // 高字节
//     //   pulsation_counttem[1] = pulsation_count & 0xFF;        // 低字节
//     //   AT24CXX_Write(0x22, &pulsation_counttem, 2);
//     //   ScreenWorkMode_count((float)pulsation_count);
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
//     //   ScreenWorkMode_count((float)auto_count);
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
    /* 屏幕加热开始 */
    case 0x1041:
        currentState = STATE_PRE_HEAT; // 切换到预加热状态
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // 启动加热任务
        break;
    /* 屏幕加热停止 */
    case 0x1030:
        if (currentState == STATE_HEAT) {
            currentState = STATE_OFF; // 从加热停止回到关闭状态
            xTaskNotify(HEAT_ONHandle, 0, eNoAction); // 通知停止加热任务
        }
        break;
    /* 屏幕挤压开始 */
    case 0x1005:
        currentState = STATE_PRE_PRESS; // 切换到预挤压状态
        break;
    /* 屏幕挤压停止 */
    case 0x1034:
        if (currentState == STATE_PRESS) {
            currentState = STATE_OFF; // 从挤压停止回到关闭状态
            xTaskNotify(PRESS_ONHandle, 0, eNoAction); // 通知停止挤压任务
        }
        break;
    /* 屏幕自动模式开始 */
    case 0x1037:
        currentState = STATE_PRE_AUTO; // 切换到预自动模式
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // 启动加热任务
        xTaskNotify(PRESS_ONHandle, 0, eSetBits); // 启动挤压任务
        break;
    /* 屏幕自动模式停止 */
    case 0x1038:
        if (currentState == STATE_AUTO) {
            currentState = STATE_OFF; // 从自动模式回到关闭状态
            xTaskNotify(HEAT_ONHandle, 0, eNoAction);  // 停止加热任务
            xTaskNotify(PRESS_ONHandle, 0, eNoAction); // 停止挤压任务
        }
        break;
    /* 其他未知命令 */
    default:
        break;
    }

}



void command_parsing(uart_data *received_data) {//区分调试命令和屏幕工作命令
  // 确认帧尾是否合法
  if (received_data->buffer[received_data->length - 2] != 0xFF ||
      received_data->buffer[received_data->length - 1] != 0xFF) {
    return; // 非法帧，直接返回
  }
  // 提取命令类型（高字节和低字节）
  uint16_t cmd_type =
      (received_data->buffer[0] << 8) | received_data->buffer[1];//第一位和第二位判断是不是调试参数
  // 解析命令
  switch (cmd_type) {
  case 0x5aa5: { // 命令类型 0x9000 - 处理 UART1 数据
    UART1_CMDHandler(received_data->buffer);
    break;
  }
  case 0x7aa7: { // 命令类型 0x0200 - 更新 PID 参数（速度控制）
    recept_data_p pid_data = (recept_data_p)received_data->buffer; // 解析数据
    //   PID_Init(&MotorPID_speed, pid_data->p, pid_data->i, pid_data->d, 1000,
    //            -1000, 50000.0f, -50000.0f, ForceRawSet);
    break;
  }

  case 0x9aa9: { // 命令类型 0x0400 - 更新 PID 参数（加热控制）
    recept_data_p heat_pid_data =
        (recept_data_p)received_data->buffer; // 解析数据
    //   PID_Init(&MotorPID_speed, pid_data->p, pid_data->i, pid_data->d, 1000,
    //            -1000, 50000.0f, -50000.0f, ForceRawSet);
  } break;

  default:
    // 未知命令，忽略
    break;
  }
}
