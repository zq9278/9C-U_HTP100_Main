#include "bq25895.h"
#include "main.h"
#include <stdint.h>
uint8_t STATE_POWER_5V;
uint8_t reset=0;
void STATE_POWER_5V_Update(void) {
  //if (HAL_GPIO_ReadPin(PWR_SENSE_GPIO_Port, PWR_SENSE_Pin) == 1) {
    if (reset==1) {

  //NVIC_SystemReset();
  STATE_POWER_5V = 1; // 开关开
} else {
  STATE_POWER_5V = 0; // 开关关
};
}
void close_mianAPP(void){
    currentState = STATE_OFF;
    HeatPWM(0); // 关闭加热PWM
   taskENTER_CRITICAL();  // ✅进入临界区，禁止上下文切换
    if (PressHandle != NULL) {
        xTaskNotifyGive(PressHandle); // 通知任务自己退出
    }
    if (HeatHandle != NULL) {
        xTaskNotifyGive(HeatHandle); // 通知任务自己退出
    }
    if (motor_homeHandle == NULL) {
        //osDelay(1000);
        if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle) == pdPASS) {
        } else {
            LOG("Failed to create motor_home task.\r\n");
        }
    } else {
        LOG("motor_home task already exists.\r\n");
    }
  taskEXIT_CRITICAL();   // ✅退出临界区
}