#include "main.h"

uint16_t Single_LED_Buffer[DATA_SIZE * LED_NUM + 200];

extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_tim16_ch1;
extern TIM_HandleTypeDef htim17;
extern uint8_t charging, working, fully_charged, emergency_stop,low_battery;
//extern osTimerId_t ws2812_white_delayHandle;
//extern osTimerId_t ws2812_yellow_delayHandle;
void PWM_WS2812B_Init(void) {
  charging=0;
  working=0;
  fully_charged=0;
  low_battery=0;
  emergency_stop=0;
  //	__HAL_TIM_DISABLE(&htim16);
  __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
  HAL_TIM_Base_Start_IT(&htim17);
  osTimerStart(ws2812_white_delayHandle, 400); // 重新启动定时器
    }
  //	__HAL_DMA_DISABLE(&hdma_tim16_ch1);


void WS2812B_Reset(void) {
  __HAL_TIM_DISABLE(&htim16);
  WS2812B_LOW;
  HAL_Delay(1);
}

void PWM_WS2812B_Write_24Bits(uint16_t num, uint32_t GRB_Data) {
  uint8_t i, j;
  //	while(hdma_tim16_ch1.State != HAL_DMA_STATE_READY){};
  for (j = 0; j < num; j++) {
    for (i = 0; i < DATA_SIZE; i++) {
      Single_LED_Buffer[j * DATA_SIZE + i] =
          ((GRB_Data << i) & 0x800000) ? T1H : T0H;
    }
  }
}
void PWM_WS2812B_Show(uint16_t num) {
  //  通过DMA输出数据
  while (hdma_tim16_ch1.State != HAL_DMA_STATE_READY)
    ; // 等待DMA完成
  HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t *)Single_LED_Buffer,
                        num * DATA_SIZE + 150);
}

void PWM_WS2812B_Red(uint16_t num) {
  PWM_WS2812B_Write_24Bits(num, 0x00ff00);
  PWM_WS2812B_Show(num);
}

void PWM_WS2812B_Green(uint16_t num) {
  PWM_WS2812B_Write_24Bits(num, 0xff0000);
  PWM_WS2812B_Show(num);
}

void PWM_WS2812B_Blue(uint16_t num) {
  PWM_WS2812B_Write_24Bits(num, 0x0000ff);
  PWM_WS2812B_Show(num);
}

void PWM_WS2812B(uint16_t num, uint32_t RGB_data) {
  PWM_WS2812B_Write_24Bits(num, RGB_data);
  PWM_WS2812B_Show(num);
}

extern ChargeState_t ChargeState; // 当前状态
extern uint8_t STATE_POWER_5V;
void LEDUpdate(void) {
}
void UpdateState(uint8_t emergency_stop, uint8_t charging, uint8_t low_battery,
                 uint8_t fully_charged, uint8_t working) {
  if (emergency_stop) {
    ChargeState = STATE_EMERGENCY_STOP; // 急停优先级最高
  } else if (fully_charged) {
    ChargeState = STATE_CHARGED; // 充满状态优先级第二
  } else if (charging) {
    ChargeState = STATE_CHARGING; // 充电优先级第三
  } else if (low_battery) {
    ChargeState = STATE_LOW_BATTERY; // 低电量优先级第四
  } else if (working) {
    ChargeState = STATE_WORKING; // 工作优先级第五
  } else {
    ChargeState = STATE_POWER_ON; // 默认开机状态
  }
}

//extern osTimerId_t breath_delayHandle;
uint8_t white_delay = 0;    // 状态变量（同时作为标志位和灯珠状态）
uint8_t yellow_delay = 0;    // 状态变量（同时作为标志位和灯珠状态）
uint8_t breathing_flag = 0; // 标志位，用于控制呼吸灯的开关
uint32_t GRB_DATA;
 static uint8_t RGB_DATA = 0;
    static uint8_t LEDDir = 0;
//void UpdateBreathingLight(void) {
//    if (LEDDir == 0) {
//      RGB_DATA += 1;
//      if (RGB_DATA >= 100) {
//        LEDDir = 1;
//      }
//    } else {
//      RGB_DATA -= 2;
//      if (RGB_DATA <= 10) {
//        LEDDir = 0;
//      }
//    }
// GRB_DATA= RGB_DATA * 0x010101; // 等价于 (RGB_DATA << 16) | (RGB_DATA << 8) | RGB_DATA
//PWM_WS2812B_Write_24Bits(LED_NUM, GRB_DATA);
// //
//}
void UpdateBreathingLight(void) {
    if (LEDDir == 0) {
        RGB_DATA += 1;
        if (RGB_DATA >= 100) LEDDir = 1;
    } else {
        RGB_DATA -= 1;
        if (RGB_DATA <= 10)  LEDDir = 0;
    }

    // 可选：加个非线性映射增强呼吸感
    uint8_t brightness = (RGB_DATA * RGB_DATA) / 100;

    // 构造 GRB 数据
    uint32_t GRB_DATA = (brightness << 16) | (brightness << 8) | brightness; // G,R,B
    PWM_WS2812B_Write_24Bits(LED_NUM, GRB_DATA);
}

// 新增一个亮度缩放函数
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

// 修改后的 UpdateLightState
void UpdateLightState(ChargeState_t state)
{
     float brightness=0.5;
    // ?? 非 CHARGING 状态，关闭呼吸灯定时器
    if (state != STATE_CHARGING && xTimerIsTimerActive(breathTimer)) {
        xTimerStop(breathTimer, 10);
    }
    switch (state) {
        case STATE_POWER_ON:
        case STATE_WORKING:
        case STATE_CHARGED:
            // 白色常亮
            PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0x444444, brightness));
            PWM_WS2812B_Show(LED_NUM);
            break;

        case STATE_LOW_BATTERY:
            if (white_delay) {
                PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0x444444, brightness)); // 白色
            } else {
                PWM_WS2812B_Write_24Bits(LED_NUM, 0x000000); // 黑色（熄灭）
            }
            PWM_WS2812B_Show(LED_NUM);
            break;

        case STATE_CHARGING:
//            UpdateBreathingLight(); // 这里是呼吸灯动画
//            PWM_WS2812B_Show(LED_NUM);
            xTimerStart(breathTimer, 10); // ? 启动呼吸动画,允许等待10ms
            break;

        case STATE_EMERGENCY_STOP:
            if (yellow_delay == 0) {
                osTimerStart(ws2812_yellow_delayHandle, 3000); // 黄灯持续 3 秒
                yellow_delay = 1;
            }
            PWM_WS2812B_Write_24Bits(LED_NUM, ScaleColor(0xFFFF00, brightness)); // 黄色
            PWM_WS2812B_Show(LED_NUM);
            break;

        default:
            PWM_WS2812B_Write_24Bits(LED_NUM, 0x000000); // 默认熄灭
            PWM_WS2812B_Show(LED_NUM);
            break;
    }
}
