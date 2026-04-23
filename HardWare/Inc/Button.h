/*
 * 文件: Button.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __BUTTON_H
#define __BUTTON_H
#include "stm32g0xx_hal.h"
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


