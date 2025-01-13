
#include "main.h"
#include "pid.h"
#include "ws2812b.h"
#include <stdint.h>
extern osMessageQueueId_t HEAT_DATAHandle;
extern osMessageQueueId_t PRESS_DATAHandle;
extern osEventFlagsId_t PRESS_ONHandle;
extern osEventFlagsId_t HEAT_ONHandle;
extern volatile SystemState_t currentState;
extern PID_TypeDef HeatPID;
extern PID_TypeDef MotorPID;
extern bool emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
float weight0 = 0; // test_variable
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
//     // switch (currentState) {
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
//     //   ScreencurrentState_count((float)heat_count);
//     //   break;
//     // case 0x02:
//     //   AT24CXX_Read(0x22, &pulsation_counttem, 2);
//     //   pulsation_count = (pulsation_counttem[0] << 8) |
//     pulsation_counttem[1];
//     //   if (heat_count == 65535) {
//     //     pulsation_count = 0;
//     //   }
//     //   pulsation_count += 1;
//     //   pulsation_counttem[0] = (pulsation_count >> 8) & 0xFF; // ���ֽ�
//     //   pulsation_counttem[1] = pulsation_count & 0xFF;        // ���ֽ�
//     //   AT24CXX_Write(0x22, &pulsation_counttem, 2);
//     //   ScreencurrentState_count((float)pulsation_count);
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
//     //   ScreencurrentState_count((float)auto_count);
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
  case 0x8900:
    if (currentState == STATE_HEAT) {
      heat_finish = 1;
    }
    if (currentState == STATE_PRESS) {
      press_finish = 1;
    }
    if (currentState == STATE_AUTO) {
      auto_finish = 1;
    }
    break;
  /* ��Ļ���ȿ�ʼ */
  case 0x1041:
    currentState = STATE_PRE_HEAT; // �л���Ԥ����״̬
    HeatPWM(1);                    // ��������PWM
    //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // �����ݷ��͵�����
    osEventFlagsSet(HEAT_ONHandle, (1 << 0)); // ���õ�0λ // ������������
    break;
  /* ��Ļ����ֹͣ */
  case 0x1030:
    if (currentState == STATE_HEAT || STATE_PRE_HEAT) {

      HeatPWM(0); // ��������PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      if ((heat_finish == 0) &&
          (currentState ==
           STATE_HEAT)) { // �����������δ��ɣ��ҹ�������ʽģʽ�£���Ԥ�Ƚ׶Σ������ý���ֹͣ��־
        emergency_stop =
            true; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
     
    }
     currentState = STATE_OFF; // �Ӽ���ֹͣ�ص��ر�״̬

    break;
  /* ��Ļ��ѹ��ʼ */
  case 0x1005:
    currentState = STATE_PRE_PRESS; // �л���Ԥ��ѹ״̬
    // data=((data * 1.0 / 88.4) / 9.8)*1000;
    MotorPID.setpoint = data;

    weight0 = ADS1220_ReadPressure();           // ��ȡ��ʼѹ��ֵ
    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // �����ݷ��͵�����
    break;
  /* ��Ļ��ѹֹͣ */
  case 0x1034:
    if (currentState == STATE_PRESS) {

      if (press_finish ==
          0) { // �����ѹ����δ��ɣ��ҹ�������ʽģʽ�£���Ԥ�Ƚ׶Σ������ý���ֹͣ��־
        emergency_stop =
            true; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��ѹ����
    }
    currentState = STATE_OFF; // �Ӽ�ѹֹͣ�ص��ر�״̬

    break;
  /* ��Ļ�Զ�ģʽ��ʼ */
  case 0x1037:
    currentState = STATE_PRE_AUTO; // �л���Ԥ�Զ�ģʽ
    HeatPWM(1);                    // ��������PWM
     //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // �����ݷ��͵�����
    MotorPID.setpoint = data;
    weight0 = ADS1220_ReadPressure();           // ��ȡ��ʼѹ��ֵ
    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // �����ݷ��͵�����
    osEventFlagsSet(HEAT_ONHandle, (1 << 0));   // ���õ�0λ // ������������
    break;
  /* ��Ļ�Զ�ģʽֹͣ */
  case 0x1038:
    // if (currentState == STATE_AUTO || STATE_PRE_AUTO) {

    //   if ((auto_finish == 0) &&
    //       (currentState ==
    //        STATE_AUTO)) { //
    //        ����Զ�����δ��ɣ��ҹ�������ʽģʽ�£����Զ��׶Σ������ý���ֹͣ��־
    //     emergency_stop =
    //         true; // ���ý���ֹͣ��־(1 << 0)); // �����0λ//
    //         ֹ֪ͨͣ��������
    //   }

    //   HeatPWM(0); // ��������PWM
    //   osEventFlagsClear(HEAT_ONHandle,
    //                     (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
    //   osEventFlagsClear(PRESS_ONHandle,
    //                     (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
    //   currentState = STATE_OFF;    // ���Զ�ģʽ�ص��ر�״̬
    // }

    if (currentState == STATE_PRE_AUTO) {
      HeatPWM(0); // ��������PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
    }
    if (currentState == STATE_AUTO) {
      HeatPWM(0); // ��������PWM
      osEventFlagsClear(HEAT_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      if (auto_finish ==0) { // ����Զ�����δ��ɣ��ҹ�������ʽģʽ�£����Զ��׶Σ������ý���ֹͣ��־
        emergency_stop =
            true; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
    }

    currentState = STATE_OFF; // ���Զ�ģʽ�ص��ر�״̬
    break;

  /* ����δ֪���� */
  default:
    break;
  }
}

void command_parsing(uart_data *received_data) { // ���ֵ����������Ļ��������
  // ȷ��֡β�Ƿ�Ϸ�
  // if (received_data->buffer[received_data->length - 2] != 0xFF ||
  //     received_data->buffer[received_data->length - 1] != 0xFF) {
  //   return; // �Ƿ�֡��ֱ�ӷ���
  // }
  // ��ȡ�������ͣ����ֽں͵��ֽڣ�
  uint16_t cmd_type =
      (received_data->buffer[0] << 8) |
      received_data->buffer[1]; // ��һλ�͵ڶ�λ�ж��ǲ��ǵ��Բ���
  // ��������
  switch (cmd_type) {
  case 0x5aa5: { // �������� 0x9000 - ���� UART1 ����
    UART1_CMDHandler(received_data->buffer);
    break;
  }
  case 0x7aa7: { // �������� 0x0200 - ���� PID �������ٶȿ��ƣ�
    recept_data_debug_p press_pid_data =
        (recept_data_debug *)received_data->buffer; // ��������
    xQueueSend(PRESS_DATAHandle, &MotorPID, 0);     // �����ݷ��͵�����
    PID_Init(&MotorPID, press_pid_data->p, press_pid_data->i, press_pid_data->d,
             5000, -5000, 50000, -50000, press_pid_data->setpoint);

    break;
  }

  case 0x9aa9: { // �������� 0x0400 - ���� PID ���������ȿ��ƣ�
    recept_data_debug_p heat_pid_data =
        (recept_data_debug *)(received_data->buffer); // ��������
    PID_Init(&HeatPID, heat_pid_data->p, heat_pid_data->i, heat_pid_data->d,
             500, 0, 255, 0, heat_pid_data->setpoint);
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // �����ݷ��͵�����
  } break;

  default:
    // δ֪�������
    break;
  }
}
extern UART_HandleTypeDef huart2;
uint8_t usart1_tx = 1;

void ScreenUpdateForce(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.data = value;
  if (currentState == STATE_PRESS) {
    pData.cmd_type_low = 0x05;
  } else if (currentState == STATE_AUTO) {
    pData.cmd_type_low = 0x47;
  }

  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData));
  }
}

void ScreenUpdateTemperature(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  if (currentState == STATE_HEAT || currentState == STATE_PRE_HEAT) {
    pData.cmd_type_low = 0x41;
  }
  if (currentState == STATE_AUTO || currentState == STATE_PRE_AUTO) {
    pData.cmd_type_low = 0x37;
  }
  pData.data = value;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData));
  }
}
void ScreenUpdateSOC(float value) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x50;
  pData.data = value;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData, sizeof(pData)); // ??????
  }
}
void ScreenWorkModeQuit(void) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x51;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????????
  }
}

void ScreenTimerStart(void) {

  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x20;
  pData.cmd_type_low = 0x52;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}
void Eye_twitching_invalid(void) {

  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x91;
  pData.cmd_type_low = 0x00;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}

void Eye_twitching_invalid_master(float count) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x92;
  pData.cmd_type_low = 0x00;
  pData.data = count;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}

void ScreenWorkMode_count(float count) {
  static recept_data pData;
  pData.cmd_head_high = 0x5A;
  pData.cmd_head_low = 0xA5;
  pData.cmd_type_high = 0x90;
  pData.cmd_type_low = 0x00;
  pData.data = count;
  pData.end_high = 0xff; // ֡β
  pData.end_low = 0xff;  // ֡β
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
  }
}
