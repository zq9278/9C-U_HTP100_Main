/*
 * @Author: zhangqi
 * @Date: 2024-12-28 11:50:08
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 13:08:02
 */

#include "main.h"
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
volatile SystemState_t currentState = STATE_OFF;
extern uint8_t heat_finish,press_finish,auto_finish;
extern PID_TypeDef HeatPID;
extern bool emergency_stop;
void Button_detection(void) {
  // 根据当前状态切换到下一个状态
  switch (currentState) {
  case STATE_OFF:

    break;

  case STATE_PRE_HEAT:
    currentState = STATE_HEAT; // 从预热进入加热
    heat_finish=0;
    HeatPID.setpoint = 42.5;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // 将加热数据发送到队列
    osEventFlagsSet(HEAT_ONHandle, (1 << 0)); // 设置第0位 // 启动加热任务
    ScreenTimerStart();
    break;

  case STATE_HEAT:
    currentState = STATE_OFF;                   // 从加热进入关闭
    emergency_stop = true;                      // 设置紧急停止标志
    if (heat_finish==0) {//完成之后按钮不起作用
    ScreenWorkModeQuit();
    }
    
    osEventFlagsClear(HEAT_ONHandle, (1 << 0)); // 清除第0位// 通知停止加热任务
    break;

  case STATE_PRE_PRESS:
    currentState = STATE_PRESS;                // 从预挤压进入挤压
   press_finish=0;
    TMC_ENN(0);                                // 启动电机
    osEventFlagsSet(PRESS_ONHandle, (1 << 0)); // 设置第0位
    ScreenTimerStart();
    break;

  case STATE_PRESS:
    currentState = STATE_OFF;                    // 从挤压进入关闭
    emergency_stop = true;                       // 设置紧急停止标志
    if (press_finish==0) {
    ScreenWorkModeQuit();
    }
    osEventFlagsClear(PRESS_ONHandle, (1 << 0)); // 清除第0位// 通知停止挤压任务
    break;

  case STATE_PRE_AUTO:
    currentState = STATE_AUTO; // 从预自动进入自动模式
     auto_finish= 0;
    HeatPID.setpoint = 42.5;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0);  // 将加热数据发送到队列
    TMC_ENN(0);                                // 启动电机
    osEventFlagsSet(PRESS_ONHandle, (1 << 0)); // 设置第0位
    ScreenTimerStart();
    break;

  case STATE_AUTO:
    currentState = STATE_OFF; // 从自动返回关闭
    emergency_stop =true; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
     if (auto_finish==0) {
    ScreenWorkModeQuit();
    }
    osEventFlagsClear(HEAT_ONHandle, (1 << 0));  // 清除第0位// 通知停止加热任务
    osEventFlagsClear(PRESS_ONHandle, (1 << 0)); // 清除第0位// 通知停止挤压任务
    break;

  default:
    break;
  }

  // 执行当前状态对应的逻辑（如果有）
}