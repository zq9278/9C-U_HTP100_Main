/*
 * @Author: zhangqi 
 * @Date: 2024-12-28 11:50:08 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-28 14:42:26
 */
#include "Button.h"
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
volatile SystemState_t currentState = STATE_OFF;
void Button_detection(void){
    // ���ݵ�ǰ״̬�л�����һ��״̬
      switch (currentState) {
      case STATE_OFF:
      
        break;

      case STATE_PRE_HEAT:
        currentState = STATE_HEAT; // ��Ԥ�Ƚ������
        break;

      case STATE_HEAT:
        currentState = STATE_OFF;                 // �Ӽ��Ƚ���ر�
        xTaskNotify(HEAT_ONHandle, 0, eNoAction); // ֹͣ��������
        break;

      case STATE_PRE_PRESS:
        currentState = STATE_PRESS; // ��Ԥ��ѹ���뼷ѹ
        break;

      case STATE_PRESS:
        currentState = STATE_OFF;                  // �Ӽ�ѹ����ر�
        xTaskNotify(PRESS_ONHandle, 0, eNoAction); // ֹͣ��ѹ����
        break;

      case STATE_PRE_AUTO:
        currentState = STATE_AUTO;                // ��Ԥ�Զ������Զ�ģʽ
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // ������������
        xTaskNotify(PRESS_ONHandle, 0, eSetBits); // ������ѹ����
        break;

      case STATE_AUTO:
        currentState = STATE_OFF;                  // ���Զ����عر�
        xTaskNotify(HEAT_ONHandle, 0, eNoAction);  // ֹͣ��������
        xTaskNotify(PRESS_ONHandle, 0, eNoAction); // ֹͣ��ѹ����
        break;

      default:
        break;
      }

      // ִ�е�ǰ״̬��Ӧ���߼�������У�
}