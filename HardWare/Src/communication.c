
#include "main.h"
#include <stdint.h>

//extern TaskHandle_t HeatHandle;
//extern volatile SystemState_t currentState;
extern PID_TypeDef HeatPID;
extern PID_TypeDef MotorPID;
extern uint8_t emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
extern prepare_data my_prepare_data;
extern uint8_t soft_button;
uint16_t save_prepare, set_prepare;

void UART1_CMDHandler(recept_data_p msg) {
   // LOG("into CMD handle\n");
    if (msg == NULL) {
        printf("Error: msg is NULL!\n");
        return;
    }

    uint16_t hot_count, crimp_count, auto_count;
    // ��ȡ��������
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    float data = (float) msg->data;
    // �������������л�״̬
    switch (cmd_type) {
        case 0x8900:
            if (currentState == STATE_HEAT) {
                heat_finish = 1;
                hot_count = AT24CXX_ReadOrWriteZero(0x00);
                AT24CXX_WriteUInt16(0x00, hot_count + 1);
                my_prepare_data.cmd_type_low = 0xA0;
                my_prepare_data.value = hot_count + 1;
                Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            }
            if (currentState == STATE_PRESS) {
                press_finish = 1;
                crimp_count = AT24CXX_ReadOrWriteZero(0x02);
                AT24CXX_WriteUInt16(0x02, crimp_count + 1);
                my_prepare_data.cmd_type_low = 0xA1;
                my_prepare_data.value = crimp_count + 1;
                Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            }
            if (currentState == STATE_AUTO) {
                auto_finish = 1;
                auto_count = AT24CXX_ReadOrWriteZero(0x04);
                AT24CXX_WriteUInt16(0x04, auto_count + 1);
                my_prepare_data.cmd_type_low = 0xA2;
                my_prepare_data.value = auto_count + 1;
                Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            }
            break;
            /* ��Ļ���ȿ�ʼ */
        case 0x1041:
            currentState = STATE_PRE_HEAT; // �л���Ԥ����״̬
            emergency_stop = 0;
            HeatPWM(1);                    // ��������PWM
            HeatPID.integral = 0;
            HeatPID.setpoint = 37 + temperature_compensation;
            if (HeatHandle == NULL) {
                if (xTaskCreate(Heat_Task, "Heat", 256, NULL, 4, &HeatHandle) == pdPASS) {
                } else {
                    LOG("Failed to create heat task.\r\n");
                }
            } else {
                LOG("heat task already exists.\r\n");
            }
            break;
            /* ��Ļ����ֹͣ */
        case 0x1030:
            if ((currentState == STATE_HEAT) || (currentState == STATE_PRE_HEAT)) {
                HeatPWM(0); // ��������PWM
                if (HeatHandle != NULL) {
                    vTaskDelete(HeatHandle);
                    HeatHandle = NULL;  // �����ٴη�����Ч���
                }
                if ((heat_finish == 0) &&
                    (currentState ==
                     STATE_HEAT)) { // �����������δ��ɣ��ҹ�������ʽģʽ�£���Ԥ�Ƚ׶Σ������ý���ֹͣ��־
                    emergency_stop = 1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
                }

            }
            currentState = STATE_OFF; // �Ӽ���ֹͣ�ص��ر�״̬

            break;
            /* ��Ļ��ѹ��ʼ */
        case 0x1005:
            currentState = STATE_PRE_PRESS; // �л���Ԥ��ѹ״̬
            emergency_stop = 0;
            // data=((data * 1.0 / 88.4) / 9.8)*1000;
            MotorPID.setpoint = data;


            break;
            /* ��Ļ��ѹֹͣ */
        case 0x1034:
            if (currentState == STATE_PRESS) {

                if (press_finish ==
                    0) { // �����ѹ����δ��ɣ��ҹ�������ʽģʽ�£���Ԥ�Ƚ׶Σ������ý���ֹͣ��־
                    emergency_stop = 1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
                }
                if (PressHandle != NULL) {
                    vTaskDelete(PressHandle);
                    PressHandle = NULL;  // �����ٴη�����Ч���
                }
                if (motor_homeHandle == NULL) {
                    if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle) == pdPASS) {
                    } else {
                        LOG("Failed to create motor_home task.\r\n");
                    }
                } else {
                    LOG("motor_home task already exists.\r\n");
                }
            }
            currentState = STATE_OFF; // �Ӽ�ѹֹͣ�ص��ر�״̬

            break;
            /* ��Ļ�Զ�ģʽ��ʼ */
        case 0x1037:
            //printf("1\n");
            currentState = STATE_PRE_AUTO; // �л���Ԥ�Զ�ģʽ
            emergency_stop = 0;
            HeatPWM(1);                    // ��������PWM
            HeatPID.integral = 0;
            HeatPID.setpoint = 37 + temperature_compensation;
            MotorPID.setpoint = data;
            if (HeatHandle == NULL) {
                if (xTaskCreate(Heat_Task, "Heat", 256, NULL, 4, &HeatHandle) == pdPASS) {
                } else {
                    LOG("Failed to create HeatHandle task.\r\n");
                }
            } else {
                LOG("HeatHandle task already exists.\r\n");
            }
            break;
            /* ��Ļ�Զ�ģʽֹͣ */
        case 0x1038:
            if (currentState == STATE_PRE_AUTO) {
                HeatPWM(0); // ����PWM
                if (HeatHandle != NULL) {
                    vTaskDelete(HeatHandle);
                    HeatHandle = NULL;  // �����ٴη�����Ч���
                }
            }
            if (currentState == STATE_AUTO) {
                HeatPWM(0); // ��������PWM
                close_mianAPP();
                if (auto_finish == 0) { // ����Զ�����δ��ɣ��ҹ�������ʽģʽ�£����Զ��׶Σ������ý���ֹͣ��־
                    emergency_stop = 1; // ���ý���ֹͣ��־(1 << 0)); // �����0λ// ֹ֪ͨͣ��������
                }
            }

            currentState = STATE_OFF; // ���Զ�ģʽ�ص��ر�״̬

            break;
        case 0x1040:
            soft_button = 1;
            xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;
            break;
        case 0x1006:
            soft_button = 1;
            xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;
            break;
        case 0x1036:
            soft_button = 1;
            xSemaphoreGive(BUTTON_SEMAPHOREHandle); // ֪ͨ��������;
            break;
        case 0x1051://��Ļ�����ź�
            serialTimeoutFlag = 0;
            xTimerReset(serialTimeoutTimerHandle, 0); // ��������ʱ��
            break;
        case 0x1050://��Ļ�����ź�
       // LOG("into 1050\n");
            serialTimeoutFlag = 0;
            prepare_data_set();
            break;
        case 0x1052://��ĻӦ���ź�
            //xSemaphoreGive(usart2_dmatxSemaphore);
            LOG("screen is open\n");
            break;
        case 0x1053:
            HeatPID.setpoint = data + temperature_compensation;
            break;
        case 0x1054:
            MotorPID.setpoint = data;
            break;
        case 0x1055:
            currentState = STATE_PRE_AUTO; // �л���Ԥ�Զ�ģʽ
            emergency_stop = 0;
            HeatPWM(1);                    // ��������PWM
            xTaskCreate(Heat_Task, "Heat", 256, NULL, 4, &HeatHandle);
            if (motor_homeHandle != NULL) {
                vTaskDelete(motor_homeHandle);
                motor_homeHandle = NULL;  // �����ٴη�����Ч���
            }//��ֹ����̫����û�й�λ
            break;
              prepare_data my_prepare_data_times;
        case 0x1056:

                // ��ʼ��ʵ���ĳ�Ա����
                my_prepare_data_times.cmd_head_high = 0x6A;
                my_prepare_data_times.cmd_head_low = 0xA6;
                my_prepare_data_times.frame_length=0x0b;
                my_prepare_data_times.cmd_type_high = 0x00;
                my_prepare_data_times.end_high = 0xFF;
                my_prepare_data_times.end_low = 0xFF;
            uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);//��ȡ�����˼�¼�Ĵ���
            my_prepare_data_times.cmd_type_low = 0xb0;
            my_prepare_data_times.value = eye_times;
            Eye_twitching_invalid_master(&my_prepare_data_times); // �����ݷ��͵�����
            break;
            /* ����δ֪���� */
        default:
            break;
    }
}

void UART1_CMDHandler_prepare(prepare_data_p msg) {


    uint16_t hot_count, crimp_count, auto_count;
    uint16_t prepare_press_pre, prepare_temperature_pre, prepare_time_pre;
    // ��ȡ��������
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    uint16_t data = (uint16_t) msg->value;
    // ��ʼ��ʵ���ĳ�Ա����
    my_prepare_data.cmd_head_high = 0x6A;
    my_prepare_data.cmd_head_low = 0xA6;
    my_prepare_data.cmd_type_high = 0x00;
    my_prepare_data.end_high = 0xFF;
    my_prepare_data.end_low = 0xFF;
    // �������������л�״̬
    switch (cmd_type) {
        case 0x1042://����Ԥ��ֵ������ֵʱ��Ҫ�ȶ�ȡ,С��ͷ
            save_prepare = data;
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
                    prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x10);
                    prepare_press_pre = AT24CXX_ReadOrWriteZero(0x12);
                    prepare_time_pre = AT24CXX_ReadOrWriteZero(0x14);
                    break;
                case 3:
                    prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x18);
                    prepare_press_pre = AT24CXX_ReadOrWriteZero(0x1A);
                    prepare_time_pre = AT24CXX_ReadOrWriteZero(0x1C);
                    break;
            }
            my_prepare_data.cmd_type_low = 0xA9;
            my_prepare_data.value = prepare_press_pre;
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����

            my_prepare_data.cmd_type_low = 0xA8;
            my_prepare_data.value = prepare_temperature_pre;
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            my_prepare_data.cmd_type_low = 0xAA;
            my_prepare_data.value = prepare_time_pre;
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            break;
        case 0x1044://ѡ��Ԥ��ֵ�����ѡ��
            set_prepare = data;
            AT24CXX_WriteUInt16(0xFC, set_prepare);
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
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            my_prepare_data.cmd_type_low = 0xA5;
            my_prepare_data.value = prepare_temperature_pre;
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
            my_prepare_data.cmd_type_low = 0xA6;
            my_prepare_data.value = prepare_time_pre;
            Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����

            break;
        case 0x1039://Ԥ���¶� prepare temprature
            switch (save_prepare) {
                case 1:
                    AT24CXX_WriteUInt16(0x08, data);
                    break;
                case 2:
                    AT24CXX_WriteUInt16(0x10, data);
                    break;
                case 3:
                    AT24CXX_WriteUInt16(0x18, data);
                    break;
            }
            break;
        case 0x1040://Ԥ��ѹ�� prepare press
            switch (save_prepare) {
                case 1:
                    AT24CXX_WriteUInt16(0x0A, data);
                    break;
                case 2:
                    AT24CXX_WriteUInt16(0x12, data);
                    break;
                case 3:
                    AT24CXX_WriteUInt16(0x1A, data);
                    break;
            }
            break;
        case 0x1041://Ԥ��ʱ�� prepare time
            switch (save_prepare) {
                case 1:
                    AT24CXX_WriteUInt16(0x0c, data);
                    break;
                case 2:
                    AT24CXX_WriteUInt16(0x14, data);
                    break;
                case 3:
                    AT24CXX_WriteUInt16(0x1c, data);
                    break;
            }
            break;
        case 0x1043:
            AT24CXX_WriteUInt16(0xf8, data);
            break;
        case 0x1046:
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            AT24C02_WriteAllBytes(0xff);//����ee�洢
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            break;
        case 0x1047:
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
            AT24C02_WriteAllBytes_eye(0xff);//����ee�洢
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
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
            LOG("into 5A A5\n");
            UART1_CMDHandler(received_data->buffer);

            break;
        }
        case 0x6aa6: { // �������� 0x6aa6 - ���� UART1 ����
            UART1_CMDHandler_prepare(received_data->buffer);

            break;
        }
        case 0x7aa7: { // �������� 0x0200 - ���� PID �������ٶȿ��ƣ�
            recept_data_debug_p press_pid_data =
                    (recept_data_debug *) received_data->buffer; // ��������
            //xQueueSend(PRESS_DATAHandle, &MotorPID, 0);     // �����ݷ��͵�����
            PID_Init(&MotorPID, press_pid_data->p, press_pid_data->i, press_pid_data->d,
                     5000, -5000, 50000, -50000, press_pid_data->setpoint);

            break;
        }
        case 0x9aa9: { // �������� 0x0400 - ���� PID ���������ȿ��ƣ�
            recept_data_debug_p heat_pid_data =
                    (recept_data_debug *) (received_data->buffer); // ��������
            PID_Init(&HeatPID, heat_pid_data->p, heat_pid_data->i, heat_pid_data->d,
                     1500, 0, 255, 0, heat_pid_data->setpoint);
//      HeatPID.previous_error=0;
//      HeatPID.integral=0;
        }
            break;

        default:
            // δ֪�������
            break;
    }
}

extern UART_HandleTypeDef huart2;

void ScreenUpdateForce(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    if (currentState == STATE_PRESS) {
        pData.cmd_type_low = 0x05;
    } else if (currentState == STATE_AUTO) {
        pData.cmd_type_low = 0x47;
    }
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));

}

void ScreenUpdateTemperature(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    if (currentState == STATE_HEAT || currentState == STATE_PRE_HEAT) {
        pData.cmd_type_low = 0x41;
    }
    if (currentState == STATE_AUTO || currentState == STATE_PRE_AUTO) {
        pData.cmd_type_low = 0x37;
    }
    pData.data = value;

    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));

}

void ScreenUpdateSOC(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x50;
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));

}

void ScreenWorkModeQuit(void) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x51;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));

}

void EYE_checkout(float data) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x55;
    pData.data = data;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));

}

void ScreenTimerStart(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x52;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send(&pData, sizeof(pData));
}
void ScreenTimerStop(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x56;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send(&pData, sizeof(pData));
}
void NEW_EYE(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x57;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send(&pData, sizeof(pData));
}
void Eye_twitching_invalid(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x91;
    pData.cmd_type_low = 0x00;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send(&pData, sizeof(pData));
}

void Eye_twitching_invalid_master(prepare_data_p myprepare_data) {
    myprepare_data->frame_length = 0x0b;
    myprepare_data->crc = Calculate_CRC((uint8_t *) myprepare_data, sizeof(*myprepare_data) - 4);
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)myprepare_data, sizeof(*myprepare_data),100);
//    taskEXIT_CRITICAL();
    vTaskDelay(10);
    USART2_DMA_Send((uint8_t *) myprepare_data, sizeof(*myprepare_data));

}

void ScreenWorkMode_count(float count) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x90;
    pData.cmd_type_low = 0x00;
    pData.data = count;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // ֡β
    pData.end_low = 0xff;  // ֡β
    USART2_DMA_Send(&pData, sizeof(pData));
}

void Serial_data_stream_parsing(uart_data *frameData) {
    if (frameData == NULL) {
        printf("Error: Invalid frameData pointer or size\n");
    }
    for (uint16_t i = 0; i < frameData->length - 1; i++) {
        // if (frameData->buffer[i] == FRAME_HEADER_BYTE1 && i + 1 < frameData->length && frameData->buffer[i + 1] == FRAME_HEADER_BYTE2) {
        if ((frameData->buffer[i] == 0x7a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xA7) ||
            (frameData->buffer[i] == FRAME_HEADER_BYTE1 && i + 1 < frameData->length &&
             frameData->buffer[i + 1] == FRAME_HEADER_BYTE2) ||
            (frameData->buffer[i] == 0x6a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xa6) ||
            (frameData->buffer[i] == 0x9a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xa9)) {
            for (uint16_t j = i + 2; j < frameData->length - 1; j++) {
                if (frameData->buffer[j] == FRAME_TAIL_BYTE1 && j + 1 < frameData->length &&
                    frameData->buffer[j + 1] == FRAME_TAIL_BYTE2) {
                    uint16_t frame_size = j - i + 2;
                    if (frame_size > UART_RX_BUFFER_SIZE) {
                        printf("Error: Frame size exceeds buffer limit\n");
                        break;
                    }
                    // ����CRC��У��
                    uint16_t received_crc = (frameData->buffer[j - 2] | (frameData->buffer[j - 1] << 8));
                    uint16_t calculated_crc = Calculate_CRC(&frameData->buffer[i], frame_size - 4); // ������CRC��֡β
                    if (calculated_crc == received_crc) {
                        command_parsing(&frameData->buffer[i]);
for(uint16_t i = 0; i < frameData->length; i++) {
                             //LOG("%02X ", frameData->buffer[i]);
                         }
                         printf("\n");
                    } else {
                        printf("Error: CRC mismatch\n");
                    }
                    // �����ѽ�����֡���ݣ������ظ�����
                    i = j + 1;
                    break;
                }
            }
        }
    }

}