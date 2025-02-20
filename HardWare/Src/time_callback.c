//
// Created by zq on 2025/2/19.
//

#include "time_callback.h"
TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle, motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle,serialTimeoutTimerHandle,IIC_EYETimeoutTimerHandle;
void ws2812_white_delay_callback(TimerHandle_t xTimer) {
    Flag_400ms = 1;
    white_delay = !white_delay;
    xTimerStart(ws2812_white_delayHandle, 0);
}
void ws2812_yellow_callback(TimerHandle_t xTimer) {
    emergency_stop = false;
    yellow_delay = 0;
}
void breath_delay_Callback(TimerHandle_t xTimer) {
    breathing_flag = 1;
}
void motor_grab3s_Callback(TimerHandle_t xTimer) {
    Flag_3s = 1;
}
void motor_back_1sCallback(TimerHandle_t xTimer) {
    Flag_1s = 1;
}
void buttton_Callback(TimerHandle_t xTimer) {
    if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
        xSemaphoreGive(BUTTON_SEMAPHOREHandle);
    }
}
void tempareture_pid_timer(TimerHandle_t xTimer) {
    flag_200ms = 1;
    tempature_flag_400ms = 1;
    press_flag_400ms = 1;
    battery_flag_400ms = 1;
}
// 串口超时处理函数
uint8_t serialTimeoutFlag = 0;//有屏幕
void SerialTimeout_Callback(TimerHandle_t xTimer)
{
    serialTimeoutFlag = 1;
}
//uint8_t IIC_EYETimeoutFlag = 0;//有眼盾
//void IIC_EYETimeout_Callback(TimerHandle_t xTimer)
//{
//    IIC_EYETimeoutFlag= 1;
//}