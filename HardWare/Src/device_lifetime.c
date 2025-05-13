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
char *i2c2_mutex_owner = NULL;
prepare_data my_prepare_data_times;

extern volatile int eye_workingtime_1s;
extern volatile int eye_existtime_1s;
extern uint8_t EYE_exist_Flag;
extern uint8_t EYE_working_Flag;
extern uint8_t EYE_status;

// ��ʼ���豸״̬����ϵͳ����ʱ���ã�
void Device_Init(void) {
    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("��ʼ���豸״̬����ɣ��ȴ��豸����\n");
}
void Test_EEPROM_FullReadWrite_256B(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFF;  // ����ַ 256 �ֽ� EEPROM
    const uint8_t  test_pattern_base = 0x5A;

    LOG("? ��ʼ EEPROM ȫ��д����� (0x00 ~ 0xFF)...\n");

    // д���������
    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t data = test_pattern_base + (addr & 0xFF);
        AT24CXX_WriteOneByte(addr, data);
        osDelay(5);  // �ȴ�д���
    }

    LOG("? д����ɣ���ʼУ��...\n");

    // У��ÿ����ַ
    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t expected = test_pattern_base + (addr & 0xFF);
        uint8_t actual = AT24CXX_ReadOneByte(addr);

        if (actual != expected) {
            LOG("? У��ʧ�ܣ���ַ 0x%02X������ 0x%02X������ 0x%02X\n", addr, expected, actual);
            return;
        }

        if ((addr % 16) == 0) {
            LOG("? У����ȣ�0x%02X\n", addr);
        }
    }

    LOG("? EEPROM ȫ�̶�д������ɣ�����һ�£�\n");
}
// �����豸Ϊ����״̬
void Device_MarkAsExpired(const char* reason) {
    //bool is_new_device = (device_ctx.started_usage == false);
    uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
    bool is_new_device = (mark == 0xFFFF);  // �� �ж����ݸĳ� EEPROM ���λ
    LOG("? �豸���ϣ�%s\n", reason);
    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;

    if (EYE_AT24CXX_Read(EYE_EEPROM_USE_COUNT_FLAG) == 0xffff) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_Write(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("Normal: ���豸ʹ�ô����Ѽ�¼: %d\n", eye_times);
        EYE_AT24CXX_Write( EYE_EEPROM_USE_COUNT_FLAG, 1);
    }
    close_mianAPP();
    ScreenTimerStop();
    xTimerStop(eye_is_existHandle, 0);
    EYE_status = 0;
}


// �ⲿ���ã�������ʽʹ�ã��л���B������
void Device_StartUsage(void) {
    EYE_AT24CXX_Write(EYE_MARK_MAP, eye_workingtime_1s);
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
        LOG("Normal: �豸������ʽʹ�ý׶Σ���ʼ B������\n");
    } else {
        LOG("Woring: �豸δ���ڿ�����״̬��StartUsage ����ʧ��\n");
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


// ��״̬����ѯ���º�������Ƶ���ã�
void DeviceStateMachine_Update(void) {
    bool online = (I2C_CheckDevice(0x91, 2) == HAL_OK);

    switch (device_ctx.state) {
        case DEVICE_STATE_DISCONNECTED:
            if (online) {
                LOG("Normal: ��⵽�豸����\n");
                device_ctx.connected = true;

                uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
                LOG("Debug: EYE_MARK_MAP ��ȡֵ = 0x%04X\n", mark);
                osDelay(100);
                device_ctx.started_usage = (mark != 0xFFFF);
                LOG("Debug: started_usage = %d��0��ʾ���豸��1��ʾ��ʹ���豸��\n", device_ctx.started_usage);

                // ���Ƕ�ȡ��ǰ EEPROM ʣ������
                device_ctx.time_a_left = EYE_AT24CXX_Read(0xA0);
                device_ctx.time_b_left = EYE_AT24CXX_Read(0xB0);
                LOG("Debug: EEPROM A������ = %d��B������ = %d\n", device_ctx.time_a_left, device_ctx.time_b_left);

                // ? �ѱ�� + A/B ��������Ϊ 0 �� ֱ�ӽ��뱨��
                if (device_ctx.started_usage ||device_ctx.time_a_left == 0 || device_ctx.time_b_left == 0) {
                    LOG("? �ѱ���豸�����ľ���ֱ�ӽ��뱨��״̬��\n");
                    Device_MarkAsExpired("������⵽����");
                    device_ctx.state = DEVICE_STATE_EXPIRED;
                    xTimerStop(eye_is_existHandle, 0);
                    EYE_status = 0;
                    return;
                }

                // �������豸������ A/B ��У��д��
                if (!device_ctx.started_usage) {
                    if (device_ctx.time_a_left == 0xFFFF || device_ctx.time_a_left > DEVICE_LIFETIME_A_DEFAULT) {
                        LOG("Warning: ��ȡ���Ƿ� A������ֵ��%d����ʼ����\n", device_ctx.time_a_left);
                        device_ctx.time_a_left = DEVICE_LIFETIME_A_DEFAULT;
                        if (EYE_AT24CXX_Write(0xA0, device_ctx.time_a_left) == HAL_OK) {
                            LOG("Correct: A����������дΪĬ��ֵ %d\n", DEVICE_LIFETIME_A_DEFAULT);
                        } else {
                            LOG("Error: A��������дʧ��\n");
                        }
                    } else {
                        LOG("Normal: δ����豸��A�ε�ǰ����=%d�����Ϊ%d��\n", device_ctx.time_a_left, DEVICE_LIFETIME_A_DEFAULT);
                    }

                    if (device_ctx.time_b_left == 0xFFFF || device_ctx.time_b_left > DEVICE_LIFETIME_B_DEFAULT) {
                        LOG("Warning: ��ȡ���Ƿ� B������ֵ��%d����ʼ����\n", device_ctx.time_b_left);
                        device_ctx.time_b_left = DEVICE_LIFETIME_B_DEFAULT;
                        if (EYE_AT24CXX_Write(0xB0, device_ctx.time_b_left) == HAL_OK) {
                            LOG("Correct: B����������дΪĬ��ֵ %d\n", DEVICE_LIFETIME_B_DEFAULT);
                        } else {
                            LOG("Error: B��������дʧ��\n");
                        }
                    } else {
                        LOG("Normal: δ����豸��B�ε�ǰ����=%d�����Ϊ%d��\n", device_ctx.time_b_left, DEVICE_LIFETIME_B_DEFAULT);
                    }
                }

                device_ctx.state = device_ctx.started_usage ? DEVICE_STATE_ACTIVE : DEVICE_STATE_CONNECTED_IDLE;
                EYE_status = device_ctx.started_usage ? 0 : 1;
                LOG("Debug: ״̬Ǩ��Ϊ��%s\n", device_ctx.started_usage ? "ACTIVE��B�Σ�" : "CONNECTED_IDLE��A�Σ�");

                uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
                LOG("Debug: ��ȡ����ʹ�ô�����%d\n", eye_times);
                my_prepare_data_times.cmd_type_low = 0xb0;
                my_prepare_data_times.value = eye_times;
                Eye_twitching_invalid_master(&my_prepare_data_times);
                LOG("�����˼�¼�Ĵ���: %d\n", eye_times);
                LOG("�豸״̬��ʼ����A=%d, B=%d, ���=%d\n", device_ctx.time_a_left, device_ctx.time_b_left, device_ctx.started_usage);

                xTimerStart(eye_is_existHandle, 0);
                LOG("��ʱ�� eye_is_existHandle ����\n");
            }

            break;




        case DEVICE_STATE_CONNECTED_IDLE://a��
            if (!online) {
                currentState = STATE_OFF;
                LOG("Woring: �豸�Ͽ�����\n");
                device_ctx.connected = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                EYE_status = 0;
                break;
            }

            if (eye_existtime_1s && device_ctx.time_a_left > 0) {
                device_ctx.time_a_left--;
                EYE_AT24CXX_Write(0xA0, device_ctx.time_a_left);  // ? д�� EEPROM��ȷ���ϵ���Ա���
                eye_existtime_1s = 0;
                LOG("Countdown: A������ʣ�ࣺ%d\n", device_ctx.time_a_left);
            }

            if (device_ctx.time_a_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("A�������ľ�");
            }
            break;


        case DEVICE_STATE_ACTIVE://b��
            if (!online) {
                currentState = STATE_OFF;
                LOG("Woring: ʹ�����豸�Ͽ���ǿ�Ʊ���\n");
                //device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("ʹ���жϿ�����");
                break;
            }
            if (eye_workingtime_1s && device_ctx.time_b_left > 0) {
                device_ctx.time_b_left--;
                eye_workingtime_1s = 0;
                LOG("Countdown: B������ʣ�ࣺ%d\n", device_ctx.time_b_left);
            }
            if (device_ctx.time_b_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("B�������ľ�");
            }
            break;

        case DEVICE_STATE_EXPIRED:
            if (!online) {
                LOG("Normal: �����豸�Ѱγ����ȴ����豸����\n");
                device_ctx.connected = false;
                device_ctx.started_usage = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                EYE_status = 0;
            }
            break;


        default:
            LOG("Woring: ״̬��δ֪״̬��%d\n", device_ctx.state);
            break;
    }

    // ʵʱ�����豸״̬��0 �� 1��
    EYE_checkout((float)EYE_status);
}
