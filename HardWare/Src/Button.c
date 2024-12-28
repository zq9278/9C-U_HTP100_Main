/*
 * @Author: zhangqi 
 * @Date: 2024-12-28 11:50:08 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-28 14:42:26
 */
#include "Button.h"
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
volatile SystemState_t currentState = STATE_OFF;
void Button_detection(void){
    // 根据当前状态切换到下一个状态
      switch (currentState) {
      case STATE_OFF:
      
        break;

      case STATE_PRE_HEAT:
        currentState = STATE_HEAT; // 从预热进入加热
        break;

      case STATE_HEAT:
        currentState = STATE_OFF;                 // 从加热进入关闭
        xTaskNotify(HEAT_ONHandle, 0, eNoAction); // 停止加热任务
        break;

      case STATE_PRE_PRESS:
        currentState = STATE_PRESS; // 从预挤压进入挤压
        break;

      case STATE_PRESS:
        currentState = STATE_OFF;                  // 从挤压进入关闭
        xTaskNotify(PRESS_ONHandle, 0, eNoAction); // 停止挤压任务
        break;

      case STATE_PRE_AUTO:
        currentState = STATE_AUTO;                // 从预自动进入自动模式
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // 启动加热任务
        xTaskNotify(PRESS_ONHandle, 0, eSetBits); // 启动挤压任务
        break;

      case STATE_AUTO:
        currentState = STATE_OFF;                  // 从自动返回关闭
        xTaskNotify(HEAT_ONHandle, 0, eNoAction);  // 停止加热任务
        xTaskNotify(PRESS_ONHandle, 0, eNoAction); // 停止挤压任务
        break;

      default:
        break;
      }

      // 执行当前状态对应的逻辑（如果有）
}