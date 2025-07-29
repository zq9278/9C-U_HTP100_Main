#include "bq25895.h"
#include "main.h"
#include <stdint.h>
#include "Button.h"
#include "communication.h"
#include "UserApp.h"
#include "interface_uart.h"
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
    if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
        ScreenWorkModeQuit();
        ScreenTimerStop();
    }

   taskENTER_CRITICAL();  // ✅进入临界区，禁止上下文切换
    currentState = STATE_OFF;
//    HeatPWM(0); // 关闭加热PWM
    if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
        xTaskNotifyGive(PressHandle);
    }
    if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
        xTaskNotifyGive(HeatHandle);
    }
    if (motor_homeHandle == NULL) {
        if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle)== pdPASS) {
        } else {
            LOG("Failed to create motor_homeHandle task.\r\n");
        }
    } else {
        LOG("motor_homeHandle task already exists.\r\n");
    }

  taskEXIT_CRITICAL();   // ✅退出临界区
}