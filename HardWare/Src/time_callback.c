/*
 * 鏂囦欢: time_callback.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "time_callback.h"
#include <stdbool.h>
#include "tmp112.h"
#include "ws2812b.h"

TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle,
        motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle,
        press_updateHandle, serialTimeoutTimerHandle, IIC_EYETimeoutTimerHandle,
        eye_is_existHandle, breathTimer;


volatile uint8_t press_pid_tick_flag = 0;


/**
 * @brief ws2812_white_delay_callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void ws2812_white_delay_callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    Flag_400ms = 1;
    white_delay = !white_delay;
    xTimerStart(ws2812_white_delayHandle, 0);
}


/**
 * @brief ws2812_yellow_callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void ws2812_yellow_callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    emergency_stop = false;
    yellow_delay = 0;
}


/**
 * @brief breath_delay_Callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void breath_delay_Callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    breathing_flag = 1;
}


/**
 * @brief motor_grab3s_Callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void motor_grab3s_Callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    Flag_3s = 1;
}


/**
 * @brief motor_back_1sCallback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void motor_back_1sCallback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    Flag_1s = 1;
}


/**
 * @brief buttton_Callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void buttton_Callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
        xSemaphoreGive(BUTTON_SEMAPHOREHandle);
    }
}


/**
 * @brief tempareture_pid_timer 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void tempareture_pid_timer(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    flag_200ms = 1;
    tempature_flag_400ms = 1;
    battery_flag_400ms = 1;
    is_charging_flag = 1;
}


/**
 * @brief press_update_timer 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void press_update_timer(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    static uint32_t display_tick_accum_ms = 0;

    press_pid_tick_flag = 1;
    display_tick_accum_ms += PRESS_PID_TIMER_PERIOD_MS;
    if (display_tick_accum_ms >= PRESS_DISPLAY_UPDATE_PERIOD_MS) {
        display_tick_accum_ms = 0;
        press_flag_400ms = 1;
    }
}


uint8_t serialTimeoutFlag = 0;


/**
 * @brief SerialTimeout_Callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void SerialTimeout_Callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    serialTimeoutFlag = 1;
}


/**
 * @brief eye_is_exist_callback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void eye_is_exist_callback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    eye_workingtime_1s = 1;
    eye_existtime_1s = 1;


    if (EYE_exist_Flag) {
    } else if (EYE_working_Flag) {
    }
}


static int16_t breath_val = 0;
static int8_t breath_dir = 1;


/**
 * @brief BreathingLightCallback 鍑芥暟瀹炵幇銆? * @param xTimer 鍙傛暟銆? */
void BreathingLightCallback(TimerHandle_t xTimer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)xTimer;
    breath_val += breath_dir;

    if (breath_val >= 100) {
        breath_val = 100;
        breath_dir = -1;
    } else if (breath_val <= 30) {
        breath_val = 30;
        breath_dir = 1;
    }


    uint8_t brightness = (uint8_t)((breath_val * breath_val) / 100);
    uint32_t GRB = (brightness << 16) | (brightness << 8) | brightness;

    PWM_WS2812B_Write_24Bits(LED_NUM, GRB);
    PWM_WS2812B_Show(LED_NUM);
}


