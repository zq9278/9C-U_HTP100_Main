/*
 * 鏂囦欢: ws2812b.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include "ws2812b.h"
#include "UserApp.h"
#include "time_callback.h"

uint16_t Single_LED_Buffer[DATA_SIZE * LED_NUM + 200];

extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_tim16_ch1;
extern TIM_HandleTypeDef htim17;
extern uint8_t charging, working, fully_charged, emergency_stop,low_battery;

#define WS2812_POWER_ON_DELAY_MS 300U


/**
 * @brief PWM_WS2812B_Init 鍑芥暟瀹炵幇銆? */
void PWM_WS2812B_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  charging=0;
  working=0;
  fully_charged=0;
  low_battery=0;
  emergency_stop=0;

  HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_SET);
  WS2812B_LOW;
  osDelay(WS2812_POWER_ON_DELAY_MS);
  HAL_GPIO_WritePin(WS2812_PW_GPIO_Port, WS2812_PW_Pin, GPIO_PIN_RESET);
  osDelay(5);

  __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
  HAL_TIM_Base_Start_IT(&htim17);
  PWM_WS2812B_Write_24Bits(LED_NUM, 0x000000);
  PWM_WS2812B_Show(LED_NUM);
  osTimerStart(ws2812_white_delayHandle, 400);
    }



/**
 * @brief WS2812B_Reset 鍑芥暟瀹炵幇銆? */
void WS2812B_Reset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  __HAL_TIM_DISABLE(&htim16);
  WS2812B_LOW;
  HAL_Delay(1);
}

/**
 * @brief PWM_WS2812B_Write_24Bits 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? * @param GRB_Data 鍙傛暟銆? */
void PWM_WS2812B_Write_24Bits(uint16_t num, uint32_t GRB_Data) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t i, j;

  for (j = 0; j < num; j++) {
    for (i = 0; i < DATA_SIZE; i++) {
      Single_LED_Buffer[j * DATA_SIZE + i] =
          ((GRB_Data << i) & 0x800000) ? T1H : T0H;
    }
  }
}
/**
 * @brief PWM_WS2812B_Show 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? */
void PWM_WS2812B_Show(uint16_t num) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

  while (hdma_tim16_ch1.State != HAL_DMA_STATE_READY)
    ;
  HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)Single_LED_Buffer,
                        num * DATA_SIZE + 150);
}

/**
 * @brief PWM_WS2812B_Red 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? */
void PWM_WS2812B_Red(uint16_t num) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  PWM_WS2812B_Write_24Bits(num, 0x00ff00);
  PWM_WS2812B_Show(num);
}

/**
 * @brief PWM_WS2812B_Green 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? */
void PWM_WS2812B_Green(uint16_t num) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  PWM_WS2812B_Write_24Bits(num, 0xff0000);
  PWM_WS2812B_Show(num);
}

/**
 * @brief PWM_WS2812B_Blue 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? */
void PWM_WS2812B_Blue(uint16_t num) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  PWM_WS2812B_Write_24Bits(num, 0x0000ff);
  PWM_WS2812B_Show(num);
}

/**
 * @brief PWM_WS2812B 鍑芥暟瀹炵幇銆? * @param num 鍙傛暟銆? * @param RGB_data 鍙傛暟銆? */
void PWM_WS2812B(uint16_t num, uint32_t RGB_data) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  PWM_WS2812B_Write_24Bits(num, RGB_data);
  PWM_WS2812B_Show(num);
}

extern ChargeState_t ChargeState;
extern uint8_t STATE_POWER_5V;
/**
 * @brief LEDUpdate 鍑芥暟瀹炵幇銆? */
void LEDUpdate(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
}
void UpdateState(uint8_t emergency_stop, uint8_t charging, uint8_t low_battery,
                 uint8_t fully_charged, uint8_t working) {
  if (emergency_stop) {
    ChargeState = STATE_EMERGENCY_STOP;
  } else if (fully_charged) {
    ChargeState = STATE_CHARGED;
  } else if (charging) {
    ChargeState = STATE_CHARGING;
  } else if (low_battery) {
    ChargeState = STATE_LOW_BATTERY;
  } else if (working) {
    ChargeState = STATE_WORKING;
  } else {
    ChargeState = STATE_POWER_ON;
  }
}


uint8_t white_delay = 0;
uint8_t yellow_delay = 0;
uint8_t breathing_flag = 0;
uint32_t GRB_DATA;
 static uint8_t RGB_DATA = 0;
    static uint8_t LEDDir = 0;
















/**
 * @brief UpdateBreathingLight 鍑芥暟瀹炵幇銆? */
void UpdateBreathingLight(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (LEDDir == 0) {
        RGB_DATA += 1;
        if (RGB_DATA >= 100) LEDDir = 1;
    } else {
        RGB_DATA -= 1;
        if (RGB_DATA <= 10)  LEDDir = 0;
    }


    uint8_t brightness = (RGB_DATA * RGB_DATA) / 100;


    uint32_t GRB_DATA = (brightness << 16) | (brightness << 8) | brightness;
    PWM_WS2812B_Write_24Bits(LED_NUM, GRB_DATA);
}


uint32_t ScaleColor(uint32_t color, float brightness)
{
    uint8_t g = (color >> 16) & 0xFF;
    uint8_t r = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    g = (uint8_t)(g * brightness);
    r = (uint8_t)(r * brightness);
    b = (uint8_t)(b * brightness);

    return (g << 16) | (r << 8) | b;
}


void UpdateLightState(ChargeState_t state)
{
     float brightness=0.5;
    if (state != STATE_CHARGING && xTimerIsTimerActive(breathTimer)) {
        xTimerStop(breathTimer, 10);
    }
    switch (state) {
        case STATE_POWER_ON:
        case STATE_WORKING:
        case STATE_CHARGED:

            PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0x444444, brightness));
            PWM_WS2812B_Show(LED_NUM);
            break;

        case STATE_LOW_BATTERY:
            if (white_delay) {
                PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0x444444, brightness));
            } else {
                PWM_WS2812B_Write_24Bits(LED_NUM, 0x000000);
            }
            PWM_WS2812B_Show(LED_NUM);
            break;

        case STATE_CHARGING:


            xTimerStart(breathTimer, 10);
            break;

        case STATE_EMERGENCY_STOP:
            if (yellow_delay == 0) {
                osTimerStart(ws2812_yellow_delayHandle, 3000);
                yellow_delay = 1;
            }
            PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0xFFFF00, brightness));
            PWM_WS2812B_Show(LED_NUM);
            break;

        default:
            PWM_WS2812B_Write_24Bits(LED_NUM, 0x000000);
            PWM_WS2812B_Show(LED_NUM);
            break;
    }
}

