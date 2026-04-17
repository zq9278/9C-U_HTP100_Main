/*
 * @Author: zhangqi
 * @Date: 2024-12-28 11:50:08
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 13:08:02
 */
#include "main.h"
#include "interface_uart.h"
#include "Button.h"
#include "pid.h"
#include "communication.h"
#include "device_lifetime.h"
#include "heat.h"
#include "tmp112.h"
#include "UserApp.h"
#include "tmc5130.h"
//extern osEventFlagsId_t PRESS_ONHandle;
//extern osEventFlagsId_t HEAT_ONHandle;
//extern osMessageQueueId_t HEAT_DATAHandle;
//extern osMessageQueueId_t PRESS_DATAHandle;
volatile SystemState_t currentState = STATE_OFF;
extern uint8_t heat_finish, press_finish, auto_finish;
extern PID_TypeDef HeatPID;
extern uint8_t emergency_stop;
extern int motor_go_home;
volatile uint8_t button_pressed = 0;

void Button_detection(void) {
    switch (currentState) {
    case STATE_OFF:
        ScreenWorkModeQuit();
        LOG("[STATE] -> OFF\r\n");
        break;

    case STATE_PRE_HEAT: // 进入加热
        Device_TryMarkNormalEyeShield();
        currentState = STATE_HEAT;
        heat_finish = 0;
        HeatPID.setpoint = 42.5 + temperature_compensation;
        ScreenTimerStart();
        LOG("[STATE] PRE_HEAT -> HEAT, setpoint=%.2f\r\n", HeatPID.setpoint);
        break;

    case STATE_HEAT: // 从加热进入关闭
        EYE_working_Flag = 0;
        currentState = STATE_OFF;
        emergency_stop = 1;
        if (heat_finish == 0) {
            ScreenWorkModeQuit();
        }
        HeatPWM(0);
        if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
            xTaskNotifyGive(HeatHandle);
        }
        LOG("[STATE] HEAT -> OFF (emergency_stop=%d)\r\n", emergency_stop);
        break;

    case STATE_PRE_PRESS: // 进入挤压
        Device_TryMarkNormalEyeShield();
        if (motor_homeHandle != NULL) {
            vTaskDelete(motor_homeHandle);
            motor_homeHandle = NULL;
        }
        currentState = STATE_PRESS;
        press_finish = 0;
        TMC_ENN(0);
        if (PressHandle != NULL) {
            vTaskResume(PressHandle);
        }
        ScreenTimerStart();
        LOG("[STATE] PRE_PRESS -> PRESS\r\n");
        break;

    case STATE_PRESS: // 挤压关闭进入 OFF
        EYE_working_Flag = 0;
        currentState = STATE_OFF;
        emergency_stop = 1;
        if (press_finish == 0) {
            ScreenWorkModeQuit();
        }
        if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
            xTaskNotifyGive(PressHandle);
        }

        if (motor_homeHandle == NULL) {
            if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle) == pdPASS) {
                LOG("[TASK] Motor_go_home created\r\n");
            } else {
                LOG("[ERROR] Failed to create Motor_go_home task.\r\n");
            }
        } else {
            LOG("[INFO] Motor_go_home task already exists\r\n");
        }

        LOG("[STATE] PRESS -> OFF\r\n");
        break;

    case STATE_PRE_AUTO: // 自动模式准备
        Device_TryMarkNormalEyeShield();
        if (motor_homeHandle != NULL) {
            vTaskDelete(motor_homeHandle);
            motor_homeHandle = NULL;
        }
        currentState = STATE_AUTO;
        auto_finish = 0;
        HeatPID.setpoint = 42.5 + temperature_compensation;
        TMC_ENN(0);
        if (PressHandle != NULL) {
            vTaskResume(PressHandle);
        }
        ScreenTimerStart();
        LOG("[STATE] PRE_AUTO -> AUTO, setpoint=%.2f\r\n", HeatPID.setpoint);
        break;

    case STATE_AUTO:
        EYE_working_Flag = 0;
        currentState = STATE_OFF;
        emergency_stop = 1;
        if (auto_finish == 0) {
            ScreenWorkModeQuit();
        }
        HeatPWM(0);
        if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
            xTaskNotifyGive(PressHandle);
        }
        if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
            xTaskNotifyGive(HeatHandle);
        }
        if (motor_homeHandle == NULL) {
            if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle) == pdPASS) {
                LOG("[TASK] Motor_go_home created\r\n");
            } else {
                LOG("[ERROR] Failed to create Motor_go_home task.\r\n");
            }
        } else {
            LOG("[INFO] Motor_go_home task already exists\r\n");
        }
        LOG("[STATE] AUTO -> OFF\r\n");
        break;

    default:
        LOG("[STATE] Unknown state=%d\r\n", currentState);
        break;
    }
}