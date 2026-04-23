/*
 * 鏂囦欢: delay.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
extern TIM_HandleTypeDef htim7;

/**
 * @brief delay_us 鍑芥暟瀹炵幇銆? * @param us 鍙傛暟銆? */
void delay_us(uint16_t us) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  HAL_TIM_Base_Start(&htim7);
  while (__HAL_TIM_GET_COUNTER(&htim7) <us) {

  }
  HAL_TIM_Base_Stop(&htim7);
}


