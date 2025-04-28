//
// Created by zq on 2025/4/18.
//
#ifndef DEVICE_LIFETIME_H
#define DEVICE_LIFETIME_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#define EYE_MARK_MAP 0x10
//#define DEVICE_LIFETIME_A_DEFAULT  600
//#define DEVICE_LIFETIME_B_DEFAULT  360
#define DEVICE_LIFETIME_A_DEFAULT  180
#define DEVICE_LIFETIME_B_DEFAULT  120


// ״̬ö�ٶ���
typedef enum {
    DEVICE_STATE_DISCONNECTED = 0,   // δ����
    DEVICE_STATE_CONNECTED_IDLE,     // ������δ��ʼʹ��
    DEVICE_STATE_ACTIVE,             // ��ʽʹ����
    DEVICE_STATE_EXPIRED             // �ѱ���
} DeviceState_t;

// ״̬�������Ľṹ��
typedef struct {
    DeviceState_t state;       // ��ǰ״̬
    uint16_t time_a_left;      // ʣ�� A������
    uint16_t time_b_left;      // ʣ�� B������
    bool started_usage;        // �Ƿ��ѿ�ʼʹ��
    bool connected;            // �Ƿ�����
} DeviceContext_t;

#ifdef __cplusplus
extern "C" {
#endif
// ��ʼ���豸״̬��
void Device_Init(void);

// ״̬�����У�Ӧ��ʱ���ã�
void DeviceStateMachine_Update(void);

// �л�����ʽʹ��״̬������ A -> B��
void Device_StartUsage(void);

// �豸���ϴ���
void Device_MarkAsExpired(const char* reason);
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries);
#ifdef __cplusplus
}
#endif

#endif // DEVICE_LIFETIME_H
