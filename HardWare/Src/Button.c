/*
 * 鏂囦欢: Button.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
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




volatile SystemState_t currentState = STATE_OFF;
extern uint8_t heat_finish, press_finish, auto_finish;
extern PID_TypeDef HeatPID;
extern uint8_t emergency_stop;
extern int motor_go_home;
volatile uint8_t button_pressed = 0;

/**
 * @brief Button_detection 鍑芥暟瀹炵幇銆? */
void Button_detection(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    switch (currentState) {
    case STATE_OFF:
        ScreenWorkModeQuit();
        LOGI("[Button] Event\r\n");
        break;

    case STATE_PRE_HEAT:
        currentState = STATE_HEAT;
        Device_RequestMarkNormalEyeShield();
        heat_finish = 0;
        HeatPID.setpoint = 42.5f + TEMPERATURE_CONTROL_COMPENSATION;
        ScreenTimerStart();
        LOGI("[Button] PRE_HEAT -> HEAT, setpoint=%.2f\r\n", HeatPID.setpoint);
        break;

    case STATE_HEAT:
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
        LOGW("[Button] HEAT -> OFF, emergency_stop=%d\r\n", emergency_stop);
        break;

    case STATE_PRE_PRESS:
        currentState = STATE_PRESS;
        Device_RequestMarkNormalEyeShield();
        if (motor_homeHandle != NULL) {
            vTaskDelete(motor_homeHandle);
            motor_homeHandle = NULL;
        }
        press_finish = 0;
        TMC_ENN(0);
        PressTask_RequestStart();
        ScreenTimerStart();
        LOGI("[Button] Event\r\n");
        break;

    case STATE_PRESS:
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
                LOGI("[Button] Event\r\n");
            } else {
                LOGE("[Button] Event\r\n");
            }
        } else {
            LOGW("[Button] Event\r\n");
        }

        LOGI("[Button] Event\r\n");
        break;

    case STATE_PRE_AUTO:
        currentState = STATE_AUTO;
        Device_RequestMarkNormalEyeShield();
        if (motor_homeHandle != NULL) {
            vTaskDelete(motor_homeHandle);
            motor_homeHandle = NULL;
        }
        currentState = STATE_AUTO;
        auto_finish = 0;
        HeatPID.setpoint = 42.5f + TEMPERATURE_CONTROL_COMPENSATION;
        TMC_ENN(0);
        PressTask_RequestStart();
        ScreenTimerStart();
        LOGI("[Button] PRE_AUTO -> AUTO, setpoint=%.2f\r\n", HeatPID.setpoint);
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
                LOGI("[Button] Event\r\n");
            } else {
                LOGE("[Button] Event\r\n");
            }
        } else {
            LOGW("[Button] Event\r\n");
        }
        LOGI("[Button] Event\r\n");
        break;

    default:
        LOGW("[Button] Unknown state=%d\r\n", currentState);
        break;
    }
}
