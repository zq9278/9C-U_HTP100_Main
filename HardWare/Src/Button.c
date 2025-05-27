/*
 * @Author: zhangqi
 * @Date: 2024-12-28 11:50:08
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 13:08:02
 */

#include "main.h"

//extern osEventFlagsId_t PRESS_ONHandle;
//extern osEventFlagsId_t HEAT_ONHandle;
//extern osMessageQueueId_t HEAT_DATAHandle;
//extern osMessageQueueId_t PRESS_DATAHandle;
volatile SystemState_t currentState = STATE_OFF;
extern uint8_t heat_finish, press_finish, auto_finish;
extern PID_TypeDef HeatPID;
extern uint8_t emergency_stop;
extern int motor_go_home;


void Button_detection(void) {
    // 根据当前状态切换到下一个状态
    switch (currentState) {
        case STATE_OFF:
            ScreenWorkModeQuit();

            break;

        case STATE_PRE_HEAT://开启加热
            Device_StartUsage();
            currentState = STATE_HEAT; // 从预热进入加热
            heat_finish = 0;
            // HeatPID.setpoint = 42.5;
//          HeatPID.previous_error = pid_heat.previous_error;
//          HeatPID.integral = pid_heat.integral;
            HeatPID.setpoint = 41.5 + temperature_compensation;
            //HeatPID.Ki=0.04;
            ScreenTimerStart();

            break;

        case STATE_HEAT://关闭加热
            EYE_working_Flag = 0;//眼盾不在工作
            currentState = STATE_OFF;                   // 从加热进入关闭
            emergency_stop = 1;                      // 设置紧急停止标志
            if (heat_finish == 0) {//完成之后按钮不起作用

                ScreenWorkModeQuit();

            }
            HeatPWM(0); // 关闭加热PWM
            if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
                xTaskNotifyGive(HeatHandle);
            }
            break;

        case STATE_PRE_PRESS://开启挤压

            if (motor_homeHandle != NULL) {
                vTaskDelete(motor_homeHandle);
                motor_homeHandle = NULL;
            }
            Device_StartUsage();
            currentState = STATE_PRESS;                // 从预挤压进入挤压
            press_finish = 0;
            TMC_ENN(0);                                // 启动电机
            if (PressHandle != NULL) {
                vTaskResume(PressHandle);
            }
            ScreenTimerStart();

            break;

        case STATE_PRESS://关闭挤压
            EYE_working_Flag = 0;//眼盾不在工作
            currentState = STATE_OFF;                    // 从挤压进入关闭
            emergency_stop = 1;                       // 设置紧急停止标志
            if (press_finish == 0) {

                ScreenWorkModeQuit();

            }
            if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
                xTaskNotifyGive(PressHandle);
            }
//            if (motor_homeHandle != NULL) {
//                vTaskResume(motor_homeHandle);
//            }
            if (motor_homeHandle == NULL) {
                if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle)== pdPASS) {
                } else {
                    LOG("Failed to create motor_homeHandle task.\r\n");
                }
            } else {
                LOG("motor_homeHandle task already exists.\r\n");
            }
            break;

        case STATE_PRE_AUTO://开启自动
//            while (motor_go_home != 1) {
//                vTaskDelay(10);
//            }
            if (motor_homeHandle != NULL) {
                vTaskDelete(motor_homeHandle);
                motor_homeHandle = NULL;
            }
            if (!Device_StartUsage()) {
                break;
                // 错误处理
            }
            currentState = STATE_AUTO; // 从预自动进入自动模式
            auto_finish = 0;
            // HeatPID.setpoint = 42.5;
//
            HeatPID.setpoint = 41.5 + temperature_compensation;
            //HeatPID.Ki=0.04;
            //xQueueSend(HEAT_DATAHandle, &HeatPID, 0);  // 将加热数据发送到队列
            TMC_ENN(0);                                // 启动电机
            if (PressHandle != NULL) {
                vTaskResume(PressHandle);
            }
            ScreenTimerStart();

            break;

        case STATE_AUTO://关闭自动
            EYE_working_Flag = 0;//眼盾不在工作
            currentState = STATE_OFF; // 从自动返回关闭
            emergency_stop = 1; // 设置紧急停止标志(1 << 0)); // 清除第0位// 通知停止加热任务
            if (auto_finish == 0) {

                ScreenWorkModeQuit();

            }
            HeatPWM(0); // 关闭加热PWM
            if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
                xTaskNotifyGive(PressHandle);
            }
            if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
                xTaskNotifyGive(HeatHandle);
            }
            if (motor_homeHandle == NULL) {
                if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle)== pdPASS) {
                } else {
                    LOG("Failed to create motor_homeHandle task.\r\n");
                }
            } else {
                LOG("motor_homeHandle task already exists.\r\n");
            }
            break;

        default:
            break;
    }

    // 执行当前状态对应的逻辑（如果有）
}