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
    HeatPWM(0); // 关闭加热PWM
    currentState = STATE_OFF; // 从自动模式回到关闭状态
    ScreenTimerStop();
    if (PressHandle != NULL) {
        vTaskDelete(PressHandle);
        PressHandle = NULL;  // 避免再次访问无效句柄
    }
    if (HeatHandle != NULL) {
        vTaskDelete(HeatHandle);
        HeatHandle = NULL;  // 避免再次访问无效句柄
    }
    if (motor_homeHandle == NULL) {
        if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 4, &motor_homeHandle) == pdPASS) {
        } else {
            LOG("Failed to create motor_home task.\r\n");
        }
    } else {
        LOG("motor_home task already exists.\r\n");
    }

}