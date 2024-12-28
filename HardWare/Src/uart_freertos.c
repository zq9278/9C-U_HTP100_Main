#include "uart_freertos.h"
#include <stdint.h>
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern volatile SystemState_t currentState;
//  void UART1_CMDHandler(recept_data_p msg) { // ��Ļ���������Ժ�Ķ��� 
//   uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
//   float data = (float)msg->data;
//   switch (cmd_type) {
//   /*�յ���Ļ���ȿ�ʼ*/
//   case 0x1041: 
//     break;
//   /*�յ���Ļ����ֹͣ*/
//   case 0x1030:
//     break;
//   /*�յ���Ļ������ʼ*/
//   case 0x1005:
//     //HX711_Init();
//     // ForceRawSet = data * 1.0;
//     // switch ((int)data) {
//     // case 150:
//     //   MotorPID_speed.Kp = 100;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 250:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 350:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // case 450:
//     //   MotorPID_speed.Kp = 120;
//     //   MotorPID_speed.Ki = 0.4;
//     //   break;
//     // case 550:
//     //   MotorPID_speed.Kp = 70;
//     //   MotorPID_speed.Ki = 0.5;
//     //   break;
//     // default:
//     //   break;
//     // }
//     break;
//   /*�յ���Ļ����ֹͣ*/
//   case 0x1034:                 
//     break;
//   /*�յ���Ļ�Զ���ʼ*/
//   case 0x1037:
//   /*�յ���Ļ�Զ�ֹͣ*/
//   case 0x1038:
//     break;
//   case 0x8900:
//     // switch (WorkMode) {
//     // case 0x01:
//     //   AT24CXX_Read(0x20, &heat_counttem, 2);
//     //   heat_count = (heat_counttem[0] << 8) | heat_counttem[1];
//     //   if (heat_count == 65535) {
//     //     heat_count = 0;
//     //   }
//     //   heat_count += 1;
//     //   heat_counttem[0] = (heat_count >> 8) & 0xFF; // ���ֽ�
//     //   heat_counttem[1] = heat_count & 0xFF;        // ���ֽ�
//     //   AT24CXX_Write(0x20, &heat_counttem, 2);
//     //   ScreenWorkMode_count((float)heat_count);
//     //   break;
//     // case 0x02:
//     //   AT24CXX_Read(0x22, &pulsation_counttem, 2);
//     //   pulsation_count = (pulsation_counttem[0] << 8) | pulsation_counttem[1];
//     //   if (heat_count == 65535) {
//     //     pulsation_count = 0;
//     //   }
//     //   pulsation_count += 1;
//     //   pulsation_counttem[0] = (pulsation_count >> 8) & 0xFF; // ���ֽ�
//     //   pulsation_counttem[1] = pulsation_count & 0xFF;        // ���ֽ�
//     //   AT24CXX_Write(0x22, &pulsation_counttem, 2);
//     //   ScreenWorkMode_count((float)pulsation_count);
//     //   break;
//     // case 0x03:
//     //   AT24CXX_Read(0x24, &auto_counttem, 2);
//     //   auto_count = (auto_counttem[0] << 8) | auto_counttem[1];
//     //   if (auto_count == 65535) {
//     //     auto_count = 0;
//     //   }
//     //   auto_count += 1;
//     //   auto_counttem[0] = (auto_count >> 8) & 0xFF; // ���ֽ�
//     //   auto_counttem[1] = auto_count & 0xFF;        // ���ֽ�
//     //   AT24CXX_Write(0x24, &auto_counttem, 2);
//     //   ScreenWorkMode_count((float)auto_count);
//     //}
//   default:
//     break;
//   }
//  }
void UART1_CMDHandler(recept_data_p msg) {
    // ��ȡ��������
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    float data = (float)msg->data;
    // �������������л�״̬
    switch (cmd_type) {
    /* ��Ļ���ȿ�ʼ */
    case 0x1041:
        currentState = STATE_PRE_HEAT; // �л���Ԥ����״̬
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // ������������
        break;
    /* ��Ļ����ֹͣ */
    case 0x1030:
        if (currentState == STATE_HEAT) {
            currentState = STATE_OFF; // �Ӽ���ֹͣ�ص��ر�״̬
            xTaskNotify(HEAT_ONHandle, 0, eNoAction); // ֹ֪ͨͣ��������
        }
        break;
    /* ��Ļ��ѹ��ʼ */
    case 0x1005:
        currentState = STATE_PRE_PRESS; // �л���Ԥ��ѹ״̬
        break;
    /* ��Ļ��ѹֹͣ */
    case 0x1034:
        if (currentState == STATE_PRESS) {
            currentState = STATE_OFF; // �Ӽ�ѹֹͣ�ص��ر�״̬
            xTaskNotify(PRESS_ONHandle, 0, eNoAction); // ֹ֪ͨͣ��ѹ����
        }
        break;
    /* ��Ļ�Զ�ģʽ��ʼ */
    case 0x1037:
        currentState = STATE_PRE_AUTO; // �л���Ԥ�Զ�ģʽ
        xTaskNotify(HEAT_ONHandle, 0, eSetBits);  // ������������
        xTaskNotify(PRESS_ONHandle, 0, eSetBits); // ������ѹ����
        break;
    /* ��Ļ�Զ�ģʽֹͣ */
    case 0x1038:
        if (currentState == STATE_AUTO) {
            currentState = STATE_OFF; // ���Զ�ģʽ�ص��ر�״̬
            xTaskNotify(HEAT_ONHandle, 0, eNoAction);  // ֹͣ��������
            xTaskNotify(PRESS_ONHandle, 0, eNoAction); // ֹͣ��ѹ����
        }
        break;
    /* ����δ֪���� */
    default:
        break;
    }

}



void command_parsing(uart_data *received_data) {//���ֵ����������Ļ��������
  // ȷ��֡β�Ƿ�Ϸ�
  if (received_data->buffer[received_data->length - 2] != 0xFF ||
      received_data->buffer[received_data->length - 1] != 0xFF) {
    return; // �Ƿ�֡��ֱ�ӷ���
  }
  // ��ȡ�������ͣ����ֽں͵��ֽڣ�
  uint16_t cmd_type =
      (received_data->buffer[0] << 8) | received_data->buffer[1];//��һλ�͵ڶ�λ�ж��ǲ��ǵ��Բ���
  // ��������
  switch (cmd_type) {
  case 0x5aa5: { // �������� 0x9000 - ���� UART1 ����
    UART1_CMDHandler(received_data->buffer);
    break;
  }
  case 0x7aa7: { // �������� 0x0200 - ���� PID �������ٶȿ��ƣ�
    recept_data_p pid_data = (recept_data_p)received_data->buffer; // ��������
    //   PID_Init(&MotorPID_speed, pid_data->p, pid_data->i, pid_data->d, 1000,
    //            -1000, 50000.0f, -50000.0f, ForceRawSet);
    break;
  }

  case 0x9aa9: { // �������� 0x0400 - ���� PID ���������ȿ��ƣ�
    recept_data_p heat_pid_data =
        (recept_data_p)received_data->buffer; // ��������
    //   PID_Init(&MotorPID_speed, pid_data->p, pid_data->i, pid_data->d, 1000,
    //            -1000, 50000.0f, -50000.0f, ForceRawSet);
  } break;

  default:
    // δ֪�������
    break;
  }
}
