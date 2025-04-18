/*
 * device_lifetime.c
 * �豸��������ģ�� - ״̬��ʵ��
 */

#include "device_lifetime.h"
#include <string.h>

// ��̬������ʵ��
static DeviceContext_t device_ctx;
extern I2C_HandleTypeDef hi2c2;
extern SemaphoreHandle_t i2c2_mutex;
extern TimerHandle_t eye_is_existHandle;
char *i2c2_mutex_owner = NULL; // ��ǰ�������ĺ���/������
prepare_data my_prepare_data_times;
extern volatile int eye_workingtime_1s;
extern volatile int eye_existtime_1s;
extern uint8_t EYE_exist_Flag;
extern uint8_t EYE_working_Flag;

// ��ʼ���豸״̬����ϵͳ����ʱ���ã�
void Device_Init(void) {
    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("��ʼ���豸״̬����ɣ��ȴ��豸����\n");
}

// �����豸Ϊ����״̬
void Device_MarkAsExpired(const char* reason) {
    bool is_new_device = (device_ctx.started_usage == false);
    LOG("? �豸���ϣ�%s\n", reason);
    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;

    if (is_new_device) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_Write(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("? ���豸ʹ�ô����Ѽ�¼: %d\n", eye_times);
    }

    EYE_AT24CXX_Write(EYE_MARK_MAP, 0xFFFF);
    close_mianAPP();
    ScreenTimerStop();
    xTimerStop(eye_is_existHandle, 0);
}

// �ⲿ���ã��豸������ʽʹ�ý׶Σ���ʼ B������
void Device_StartUsage(void) {
    my_prepare_data_times.cmd_head_high = 0x6A;
    my_prepare_data_times.cmd_head_low = 0xA6;
    my_prepare_data_times.frame_length = 0x0b;
    my_prepare_data_times.cmd_type_high = 0x00;
    my_prepare_data_times.end_high = 0xFF;
    my_prepare_data_times.end_low = 0xFF;

    if (device_ctx.state == DEVICE_STATE_CONNECTED_IDLE) {
        device_ctx.started_usage = true;
        device_ctx.time_a_left = 0;
        device_ctx.state = DEVICE_STATE_ACTIVE;
        xTimerStart(eye_is_existHandle, 0);
        LOG("��ʱ�� eye_is_existHandle ����\n");
        LOG("? �豸������ʽʹ�ý׶Σ���ʼ B������\n");
    } else {
        LOG("?? �豸δ���ڿ�����״̬��StartUsage ����ʧ��\n");
    }
}

// I2C ����豸�Ƿ����ߣ�����������
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    if (i2c2_mutex == NULL) {
        LOG("����I2C������δ��ʼ����\n");
        return HAL_ERROR;
    }
    if (xSemaphoreTake(i2c2_mutex, 100) == pdTRUE) {
        for (uint8_t i = 0; i < retries; i++) {
            result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 100);
            if (result == HAL_OK) {
                xSemaphoreGive(i2c2_mutex);
                i2c2_mutex_owner = NULL;
                return HAL_OK;
            }
            osDelay(100);
        }
        xSemaphoreGive(i2c2_mutex);
        return HAL_ERROR;
    } else {
        LOG("I2C_CheckDevice����ȡ������ʧ�ܣ�\n");
        return HAL_ERROR;
    }
}



// ��״̬����ѯ���º�������Ƶ���ã�֧���Ȳ�Σ�
void DeviceStateMachine_Update(void) {
    bool online = (I2C_CheckDevice(0x91, 2) == HAL_OK);

    switch (device_ctx.state) {
        case DEVICE_STATE_DISCONNECTED:
            if (online) {
                LOG("? ��⵽�豸����\n");
                device_ctx.connected = true;
                device_ctx.time_a_left = EYE_AT24CXX_Read(0xA0);
                device_ctx.time_b_left = EYE_AT24CXX_Read(0xB0);
                uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
                device_ctx.started_usage = (mark != 0xFFFF);
                device_ctx.state = device_ctx.started_usage ? DEVICE_STATE_ACTIVE : DEVICE_STATE_CONNECTED_IDLE;// �����ʹ�������B���������������A��Ԥ��״̬

                uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
                my_prepare_data_times.cmd_type_low = 0xb0;
                my_prepare_data_times.value = eye_times;
                Eye_twitching_invalid_master(&my_prepare_data_times);

                LOG("�����˼�¼�Ĵ���: %d\n", eye_times);
                LOG("�豸״̬��ʼ����A=%d, B=%d, ���=%d\n", device_ctx.time_a_left, device_ctx.time_b_left, device_ctx.started_usage);

                xTimerStart(eye_is_existHandle, 0);
            }
            break;

        case DEVICE_STATE_CONNECTED_IDLE://����A��Ԥ��״̬
            if (!online) {
                LOG("?? �豸�Ͽ�����\n");
                device_ctx.connected = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                break;
            }
            if (eye_existtime_1s && device_ctx.time_a_left > 0) {
                device_ctx.time_a_left--;
                eye_existtime_1s = 0;
                LOG("? A������ʣ�ࣺ%d\n", device_ctx.time_a_left);
            }
            if (device_ctx.time_a_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("A�������ľ�");
            }
            break;

        case DEVICE_STATE_ACTIVE:
            if (!online) {
                LOG("? ʹ�����豸�Ͽ���ǿ�Ʊ���\n");
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("ʹ���жϿ�����");
                break;
            }
            if (eye_workingtime_1s && device_ctx.time_b_left > 0) {
                device_ctx.time_b_left--;
                eye_workingtime_1s = 0;
                LOG("? B������ʣ�ࣺ%d\n", device_ctx.time_b_left);
            }
            if (device_ctx.time_b_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("B�������ľ�");
            }
            break;

        case DEVICE_STATE_EXPIRED:
            break;

        default:
            LOG("? ״̬��δ֪״̬��%d\n", device_ctx.state);
            break;
    }
}
