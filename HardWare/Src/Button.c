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

void Button_detection(void) {
    // ���ݵ�ǰ״̬�л�����һ��״̬
    switch (currentState) {
        case STATE_OFF:

            break;

        case STATE_PRE_HEAT:
            Device_StartUsage();
            currentState = STATE_HEAT; // ��Ԥ�Ƚ������
            heat_finish = 0;
            // HeatPID.setpoint = 42.5;
//          HeatPID.previous_error = pid_heat.previous_error;
//          HeatPID.integral = pid_heat.integral;
            HeatPID.setpoint = 41.5 + temperature_compensation;
            //HeatPID.Ki=0.04;
            ScreenTimerStart();

            break;

        case STATE_HEAT:
            EYE_working_Flag = 0;//�۶ܲ��ڹ���
            currentState = STATE_OFF;                   // �Ӽ��Ƚ���ر�
            emergency_stop = 1;                      // ���ý���ֹͣ��־
            if (heat_finish == 0) {//���֮��ť��������

                ScreenWorkModeQuit();

            }
            HeatPWM(0); // �رռ���PWM
            if (HeatHandle != NULL) {
                LOG("heat_stop");
                vTaskDelete(HeatHandle);
                HeatHandle = NULL;  // �����ٴη�����Ч���
            }
            break;

        case STATE_PRE_PRESS:
            Device_StartUsage();
            currentState = STATE_PRESS;                // ��Ԥ��ѹ���뼷ѹ
            if (motor_homeHandle != NULL) {
                vTaskDelete(motor_homeHandle);
                motor_homeHandle = NULL;  // �����ٴη�����Ч���
            }
            press_finish = 0;
            TMC_ENN(0);                                // �������
            if (PressHandle == NULL) {
                if (xTaskCreate(Press_Task, "Press", 256, NULL, 3, &PressHandle) == pdPASS) {
                } else {
                    LOG("Failed to create press task.\r\n");
                }
            } else {
                LOG("press task already exists.\r\n");
            }
            ScreenTimerStart();

            break;

        case STATE_PRESS:
            EYE_working_Flag = 0;//�۶ܲ��ڹ���
            currentState = STATE_OFF;                    // �Ӽ�ѹ����ر�
            emergency_stop = 1;                       // ���ý���ֹͣ��־
            if (press_finish == 0) {

                ScreenWorkModeQuit();

            }
            if (PressHandle != NULL) {
                vTaskDelete(PressHandle);
                PressHandle = NULL;  // �����ٴη�����Ч���
            }

            if (motor_homeHandle == NULL) {
                if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle) == pdPASS) {
                    printf("Motor go home task created successfully.\r\n");
                } else {
                    printf("Failed to create motor go home task.\r\n");
                }
            } else {
                printf("Motor go home task already exists.\r\n");
            }
            break;

        case STATE_PRE_AUTO:
            Device_StartUsage();
            currentState = STATE_AUTO; // ��Ԥ�Զ������Զ�ģʽ
            if (motor_homeHandle != NULL) {
                vTaskDelete(motor_homeHandle);
                motor_homeHandle = NULL;  // �����ٴη�����Ч���
            }
            auto_finish = 0;
            // HeatPID.setpoint = 42.5;
//
            HeatPID.setpoint = 41.5 + temperature_compensation;
            //HeatPID.Ki=0.04;
            //xQueueSend(HEAT_DATAHandle, &HeatPID, 0);  // ���������ݷ��͵�����
            TMC_ENN(0);                                // �������

            if (PressHandle == NULL) {
                if (xTaskCreate(Press_Task, "Press", 256, NULL, 3, &PressHandle) == pdPASS) {
                } else {
                    LOG("Failed to create press task.\r\n");
                }
            } else {
                LOG("press task already exists.\r\n");
            }
            ScreenTimerStart();

            break;

        case STATE_AUTO:
            EYE_working_Flag = 0;//�۶ܲ��ڹ���
            currentState = STATE_OFF; // ���Զ����عر�
            emergency_stop = 1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
            if (auto_finish == 0) {

                ScreenWorkModeQuit();

            }
            HeatPWM(0); // �رռ���PWM
            if (PressHandle != NULL) {
                vTaskDelete(PressHandle);
                PressHandle = NULL;  // �����ٴη�����Ч���
            }
            if (HeatHandle != NULL) {
                LOG("heat_stop");
                vTaskDelete(HeatHandle);
                HeatHandle = NULL;  // �����ٴη�����Ч���
            }
            if (motor_homeHandle == NULL) {
                if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle) == pdPASS) {
                } else {
                    LOG("Failed to create motor go home task.\r\n");
                }
            } else {
                LOG("Motor go home task already exists.\r\n");
            }
            break;

        default:
            break;
    }

    // ִ�е�ǰ״̬��Ӧ���߼�������У�
}