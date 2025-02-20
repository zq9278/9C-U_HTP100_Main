// Created by zq on 2025/2/19.
//

#ifndef TIME_CALLBACK_H
#define TIME_CALLBACK_H
#include "main.h"
extern TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle, motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle,serialTimeoutTimerHandle,IIC_EYETimeoutTimerHandle;
void ws2812_white_delay_callback(TimerHandle_t xTimer);
void ws2812_yellow_callback(TimerHandle_t xTimer);
void breath_delay_Callback(TimerHandle_t xTimer);
void motor_grab3s_Callback(TimerHandle_t xTimer);
void motor_back_1sCallback(TimerHandle_t xTimer);
void buttton_Callback(TimerHandle_t xTimer);
void tempareture_pid_timer(TimerHandle_t xTimer);
// 串口超时处理函数
void SerialTimeout_Callback(TimerHandle_t xTimer);
void IIC_EYETimeout_Callback(TimerHandle_t xTimer);
extern uint8_t serialTimeoutFlag;
#endif //SLK_01_V1_0_TIME_CALLBACK_H
