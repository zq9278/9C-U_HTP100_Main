
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
extern uint8_t emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
extern prepare_data my_prepare_data;
extern osMessageQueueId_t prepare_dataHandle;
extern osSemaphoreId_t BUTTON_SEMAPHOREHandle; // �����ź������
extern uint8_t soft_button;
uint16_t save_prepare,set_prepare;
void UART1_CMDHandler(recept_data_p msg) {
    if (msg == NULL) {
        printf("Error: msg is NULL!\n");
        return;
    }

  uint16_t hot_count,crimp_count,auto_count;
  // ��ȡ��������
  uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
  float data = (float)msg->data;
  // �������������л�״̬
  switch (cmd_type) {
  case 0x8900:
    if (currentState == STATE_HEAT) {
      heat_finish = 1;
      hot_count = AT24CXX_ReadOrWriteZero(0x00);
      AT24CXX_WriteUInt16(0x00,hot_count+1);
      my_prepare_data.cmd_type_low = 0xA0;
      my_prepare_data.value = hot_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    }
    if (currentState == STATE_PRESS) {
      press_finish = 1;
      crimp_count = AT24CXX_ReadOrWriteZero(0x02);
      AT24CXX_WriteUInt16(0x02,crimp_count+1);
      my_prepare_data.cmd_type_low = 0xA1;
      my_prepare_data.value = crimp_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    }
    if (currentState == STATE_AUTO) {
      auto_finish = 1;
      auto_count = AT24CXX_ReadOrWriteZero(0x04);
      AT24CXX_WriteUInt16(0x04,auto_count+1);
      my_prepare_data.cmd_type_low = 0xA2;
      my_prepare_data.value = auto_count+1;
      xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    }
    break;
  /* ��Ļ���ȿ�ʼ */
  case 0x1041:
    currentState = STATE_PRE_HEAT; // �л���Ԥ����״̬
    emergency_stop=0;
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
        emergency_stop =1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
     
    }
     currentState = STATE_OFF; // �Ӽ���ֹͣ�ص��ر�״̬

    break;
  /* ��Ļ��ѹ��ʼ */
  case 0x1005:
    currentState = STATE_PRE_PRESS; // �л���Ԥ��ѹ״̬
    emergency_stop=0;
    // data=((data * 1.0 / 88.4) / 9.8)*1000;
    MotorPID.setpoint = data;

    xQueueSend(PRESS_DATAHandle, &MotorPID, 0); // �����ݷ��͵�����
    break;
  /* ��Ļ��ѹֹͣ */
  case 0x1034:
    if (currentState == STATE_PRESS) {

      if (press_finish ==
          0) { // �����ѹ����δ��ɣ��ҹ�������ʽģʽ�£���Ԥ�Ƚ׶Σ������ý���ֹͣ��־
        emergency_stop =1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
      osEventFlagsClear(PRESS_ONHandle,
                        (1 << 0)); // �����0λ// ֹ֪ͨͣ��ѹ����
    }
    currentState = STATE_OFF; // �Ӽ�ѹֹͣ�ص��ر�״̬

    break;
  /* ��Ļ�Զ�ģʽ��ʼ */
  case 0x1037:
    //printf("1\n");
    currentState = STATE_PRE_AUTO; // �л���Ԥ�Զ�ģʽ
    emergency_stop=0;
    HeatPWM(1);                    // ��������PWM
     //HeatPID.setpoint = 37.5;
    HeatPID.setpoint = 37.5+temperature_compensation;
    xQueueSend(HEAT_DATAHandle, &HeatPID, 0); // �����ݷ��͵�����
    MotorPID.setpoint = data;

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
        emergency_stop =1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
      }
    }

    currentState = STATE_OFF; // ���Զ�ģʽ�ص��ر�״̬
    break;
  case 0x1040:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;

    break;
  case 0x1006:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;
    break;
  case 0x1036:
    soft_button=1;
    xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;
    break;

  /* ����δ֪���� */
  default:
    break;
  }
}
uint32_t a;
void UART1_CMDHandler_prepare(prepare_data_p msg) {
    /*test
     *
     * */

  uint16_t hot_count,crimp_count,auto_count;
  uint16_t prepare_press_pre,prepare_temperature_pre,prepare_time_pre;
  // ��ȡ��������
  uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
  uint16_t data = (uint16_t)msg->value;
    // ��ʼ��ʵ���ĳ�Ա����
    my_prepare_data.cmd_head_high = 0x6A;
    my_prepare_data.cmd_head_low = 0xA6;
    my_prepare_data.cmd_type_high = 0x00;
    my_prepare_data.end_high = 0xFF;
    my_prepare_data.end_low = 0xFF;
  // �������������л�״̬
  switch (cmd_type) {
  case 0x1042://����Ԥ��ֵ������ֵʱ��Ҫ�ȶ�ȡ,С��ͷ
    save_prepare=data;
    switch (save_prepare) {
    case 0:
      prepare_press_pre = 150;
      prepare_temperature_pre = 42;
      prepare_time_pre = 1;
      break;
    case 1:
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x08);
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x0A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x0C);
      break;
    case 2:
       prepare_temperature_pre= AT24CXX_ReadOrWriteZero(0x10);
            prepare_press_pre  = AT24CXX_ReadOrWriteZero(0x12);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x14);
      break;
    case 3:
        prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x18);
            prepare_press_pre  = AT24CXX_ReadOrWriteZero(0x1A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x1C);
      break;
    }
    my_prepare_data.cmd_type_low = 0xA9;
    my_prepare_data.value = prepare_press_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    my_prepare_data.cmd_type_low = 0xA8;
    my_prepare_data.value = prepare_temperature_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    my_prepare_data.cmd_type_low = 0xAA;
    my_prepare_data.value = prepare_time_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    break;
  case 0x1044://ѡ��Ԥ��ֵ�����ѡ��
    set_prepare=data;
    AT24CXX_WriteUInt16(0xFC,set_prepare);
    switch (set_prepare) {
    case 0:
      prepare_press_pre = 150;
      prepare_temperature_pre = 42;
      prepare_time_pre = 1;
      break;
    case 1:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x08);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x0A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x0C);
      break;
    case 2:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x10);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x12);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x14);
      break;
    case 3:
      prepare_press_pre = AT24CXX_ReadOrWriteZero(0x18);
      prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x1A);
      prepare_time_pre = AT24CXX_ReadOrWriteZero(0x1C);
      break;
    }
    my_prepare_data.cmd_type_low = 0xA4;
    my_prepare_data.value = prepare_press_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    my_prepare_data.cmd_type_low = 0xA5;
    my_prepare_data.value = prepare_temperature_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����
    my_prepare_data.cmd_type_low = 0xA6;
    my_prepare_data.value = prepare_time_pre;
    xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // �����ݷ��͵�����

    break;
  case 0x1039://Ԥ���¶� prepare temprature
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x08,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x10,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x18,data);
      break;
    }
      break;
  case 0x1040://Ԥ��ѹ�� prepare press
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x0A,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x12,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x1A,data);
      break;
    }
    break;
  case 0x1041://Ԥ��ʱ�� prepare time
    switch (save_prepare) {
    case 1:
      AT24CXX_WriteUInt16(0x0c,data);
      break;
    case 2:
      AT24CXX_WriteUInt16(0x14,data);
      break;
    case 3:
      AT24CXX_WriteUInt16(0x1c,data);
      break;
    }
    break;
  case 0x1043:
    AT24CXX_WriteUInt16(0xf8,data);
    break;
  case 0x1046:
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    AT24C02_WriteAllBytes(0xff);//����ee�洢
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    break;
    /* ����δ֪���� */
  default:
    break;
  }
}
void command_parsing(uart_data *received_data) { // ���ֵ����������Ļ��������
//    a++;
//    printf("a=%u\n",a);
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
  case 0x6aa6: { // �������� 0x6aa6 - ���� UART1 ����
    UART1_CMDHandler_prepare(received_data->buffer);

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
      taskENTER_CRITICAL();
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????????
      taskEXIT_CRITICAL();
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
      taskENTER_CRITICAL();
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)&pData,
                         sizeof(pData)); // ???????????????
                         taskEXIT_CRITICAL();
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
void Eye_twitching_invalid_master(prepare_data_p myprepare_data) {
  if (usart1_tx) {
    usart1_tx = 0;
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)myprepare_data,
                         sizeof(prepare_data)); // ???????????????
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
uint16_t Calculate_CRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}