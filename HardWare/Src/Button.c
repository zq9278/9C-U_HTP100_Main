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
extern uint8_t heat_finish,press_finish,auto_finish;
extern PID_TypeDef HeatPID;
extern uint8_t emergency_stop;
void Button_detection(void) {
  // ���ݵ�ǰ״̬�л�����һ��״̬
  switch (currentState) {
  case STATE_OFF:

    break;

  case STATE_PRE_HEAT:
    currentState = STATE_HEAT; // ��Ԥ�Ƚ������
    heat_finish=0;
    // HeatPID.setpoint = 42.5;
//          HeatPID.previous_error = pid_heat.previous_error;
//          HeatPID.integral = pid_heat.integral;
    HeatPID.setpoint = 42+temperature_compensation;
          //HeatPID.Ki=0.04;
    ScreenTimerStart();
          
    break;

  case STATE_HEAT:
    currentState = STATE_OFF;                   // �Ӽ��Ƚ���ر�
    emergency_stop = 1;                      // ���ý���ֹͣ��־
    if (heat_finish==0) {//���֮��ť��������
        
    ScreenWorkModeQuit();
        
    }
    HeatPWM(0); // �رռ���PWM
          vTaskDelete(HeatHandle);// ֹ֪ͨͣ��������
    break;

  case STATE_PRE_PRESS:
    currentState = STATE_PRESS;                // ��Ԥ��ѹ���뼷ѹ
   press_finish=0;
    TMC_ENN(0);                                // �������
          xTaskCreate(Press_Task, "Press", 256, NULL, 3, &PressHandle);
          
    ScreenTimerStart();
          
    break;

  case STATE_PRESS:
    currentState = STATE_OFF;                    // �Ӽ�ѹ����ر�
    emergency_stop = 1;                       // ���ý���ֹͣ��־
    if (press_finish==0) {
        
    ScreenWorkModeQuit();
        
    }
          vTaskDelete(PressHandle);// ֹ֪ͨͣ��������
          xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle);
    break;

  case STATE_PRE_AUTO:
    currentState = STATE_AUTO; // ��Ԥ�Զ������Զ�ģʽ
     auto_finish= 0;
    // HeatPID.setpoint = 42.5;
//
    HeatPID.setpoint = 42+temperature_compensation;
          //HeatPID.Ki=0.04;
    //xQueueSend(HEAT_DATAHandle, &HeatPID, 0);  // ���������ݷ��͵�����
    TMC_ENN(0);                                // �������
          xTaskCreate(Press_Task, "Press", 256, NULL, 3, &PressHandle);
          
    ScreenTimerStart();
          
    break;

  case STATE_AUTO:
    currentState = STATE_OFF; // ���Զ����عر�
    emergency_stop =1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
     if (auto_finish==0) {
         
    ScreenWorkModeQuit();
         
    }
    HeatPWM(0); // �رռ���PWM
          vTaskDelete(PressHandle);// ֹ֪ͨͣ��������
          vTaskDelete(HeatHandle);// ֹ֪ͨͣ��������
          xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle);
    break;

  default:
    break;
  }

  // ִ�е�ǰ״̬��Ӧ���߼�������У�
}