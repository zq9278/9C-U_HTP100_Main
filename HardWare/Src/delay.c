

#include "main.h"
extern TIM_HandleTypeDef htim7;
// 延迟函数，延迟指定微秒
void delay_us(uint16_t us) {
  __HAL_TIM_SET_COUNTER(&htim7, 0); // 设置定时器计数器初始值为 0
  HAL_TIM_Base_Start(&htim7);       // 启动定时器
  while (__HAL_TIM_GET_COUNTER(&htim7) <us) { // 等待定时器计数器达到指定的延迟值
    // 轮询直到计数器值达到设定的微秒数
  }
  HAL_TIM_Base_Stop(&htim7); // 停止定时器
}