/*
 * 文件: time_callback.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef TIME_CALLBACK_H
#define TIME_CALLBACK_H

#include "main.h"
#include "UserApp.h"


#define PRESS_PID_TIMER_PERIOD_MS      20u

#define PRESS_DISPLAY_UPDATE_PERIOD_MS 400u

#define FACTORY_MODE_RUN_WINDOW_MS     180000u
#define FACTORY_MODE_PAUSE_WINDOW_MS   5000u

extern TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle,
        motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle,
        press_updateHandle, serialTimeoutTimerHandle, IIC_EYETimeoutTimerHandle,
        factory_cycleHandle,
        eye_is_existHandle, breathTimer;

extern volatile uint8_t press_pid_tick_flag;
extern uint8_t serialTimeoutFlag;

void ws2812_white_delay_callback(TimerHandle_t xTimer);
void ws2812_yellow_callback(TimerHandle_t xTimer);
void breath_delay_Callback(TimerHandle_t xTimer);
void motor_grab3s_Callback(TimerHandle_t xTimer);
void motor_back_1sCallback(TimerHandle_t xTimer);
void buttton_Callback(TimerHandle_t xTimer);
void tempareture_pid_timer(TimerHandle_t xTimer);
void press_update_timer(TimerHandle_t xTimer);
void SerialTimeout_Callback(TimerHandle_t xTimer);
void IIC_EYETimeout_Callback(TimerHandle_t xTimer);
void eye_is_exist_callback(TimerHandle_t xTimer);
void BreathingLightCallback(TimerHandle_t xTimer);
void FactoryModeCycleCallback(TimerHandle_t xTimer);
void FactoryModeCycleStartRunWindow(void);
void FactoryModeCycleStartPauseWindow(void);
void FactoryModeCycleStop(void);
uint8_t FactoryModeCycleIsAutoStopPending(void);
void FactoryModeCycleClearAutoStopPending(void);

#endif

