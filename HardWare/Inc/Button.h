/*
 * @Author: zhangqi 
 * @Date: 2024-12-28 11:50:08 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-29 13:19:46
 */
#ifndef __BUTTON_H
#define __BUTTON_H
#include "stm32g0xx_hal.h"
// 定义系统状态
typedef enum {
    STATE_OFF,
    STATE_PRE_HEAT,
    STATE_HEAT,
    STATE_PRE_PRESS,
    STATE_PRESS,
    STATE_PRE_AUTO,
    STATE_AUTO
} SystemState_t;
extern volatile SystemState_t currentState;
void Button_detection(void);
#endif