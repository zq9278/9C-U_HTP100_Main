/*
 * @Author: zhangqi
 * @Date: 2024-12-28 11:50:08
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-30 13:08:02
 */

#include "main.h"
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
volatile SystemState_t currentState = STATE_OFF;
extern uint8_t heat_finish,press_finish,auto_finish;
extern PID_TypeDef HeatPID;
extern bool emergency_stop;
void Button_detection(void) {
  // ���ݵ�ǰ״̬�л�����һ��״̬
  switch (currentState) {
  case STATE_OFF:

    break;

  case STATE_PRE_HEAT:
    currentState = STATE_HEAT; // ��Ԥ�Ƚ������
    heat_finish=0;
    HeatPID.setpoint = 42.5;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // ���������ݷ��͵�����
    osEventFlagsSet(HEAT_ONHandle, (1 << 0)); // ���õ�0λ // ������������
    ScreenTimerStart();
    break;

  case STATE_HEAT:
    currentState = STATE_OFF;                   // �Ӽ��Ƚ���ر�
    emergency_stop = true;                      // ���ý���ֹͣ��־
    if (heat_finish==0) {//���֮��ť��������
    ScreenWorkModeQuit();
    }
    
    osEventFlagsClear(HEAT_ONHandle, (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
    break;

  case STATE_PRE_PRESS:
    currentState = STATE_PRESS;                // ��Ԥ��ѹ���뼷ѹ
   press_finish=0;
    TMC_ENN(0);                                // �������
    osEventFlagsSet(PRESS_ONHandle, (1 << 0)); // ���õ�0λ
    ScreenTimerStart();
    break;

  case STATE_PRESS:
    currentState = STATE_OFF;                    // �Ӽ�ѹ����ر�
    emergency_stop = true;                       // ���ý���ֹͣ��־
    if (press_finish==0) {
    ScreenWorkModeQuit();
    }
    osEventFlagsClear(PRESS_ONHandle, (1 << 0)); // �����0λ// ֹ֪ͨͣ��ѹ����
    break;

  case STATE_PRE_AUTO:
    currentState = STATE_AUTO; // ��Ԥ�Զ������Զ�ģʽ
     auto_finish= 0;
    HeatPID.setpoint = 42.5;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0);  // ���������ݷ��͵�����
    TMC_ENN(0);                                // �������
    osEventFlagsSet(PRESS_ONHandle, (1 << 0)); // ���õ�0λ
    ScreenTimerStart();
    break;

  case STATE_AUTO:
    currentState = STATE_OFF; // ���Զ����عر�
    emergency_stop =true; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
     if (auto_finish==0) {
    ScreenWorkModeQuit();
    }
    osEventFlagsClear(HEAT_ONHandle, (1 << 0));  // �����0λ// ֹ֪ͨͣ��������
    osEventFlagsClear(PRESS_ONHandle, (1 << 0)); // �����0λ// ֹ֪ͨͣ��ѹ����
    break;

  default:
    break;
  }

  // ִ�е�ǰ״̬��Ӧ���߼�������У�
}