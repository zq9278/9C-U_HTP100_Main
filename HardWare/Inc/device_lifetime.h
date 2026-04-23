/*
 * 文件: device_lifetime.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef DEVICE_LIFETIME_H
#define DEVICE_LIFETIME_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#define EYE_MARK_MAP 0x01
#define super_eyes 0x03
#define DEVICE_LIFETIME_A_DEFAULT  180
#define DEVICE_LIFETIME_B_DEFAULT  360


#define EYE_EEPROM_USE_COUNT_FLAG     0x08



typedef enum {
    DEVICE_STATE_DISCONNECTED = 0,
    DEVICE_STATE_CONNECTED_IDLE,
    DEVICE_STATE_ACTIVE,
    DEVICE_STATE_EXPIRED
} DeviceState_t;


typedef struct {
    DeviceState_t state;
    uint16_t time_a_left;
    uint16_t time_b_left;
    bool started_usage;
    bool connected;
} DeviceContext_t;
extern DeviceContext_t device_ctx;
#ifdef __cplusplus
extern "C" {
#endif

void Device_Init(void);


void DeviceStateMachine_Update(void);


bool  Device_StartUsage(void);

void Device_TryMarkNormalEyeShield(void);
void Device_RequestMarkNormalEyeShield(void);
void Device_HandlePendingMarkRequest(void);


void Device_MarkAsExpired(const char* reason);
void Test_EEPROM_FullReadWrite_256B(void);
void Test_EYE_AT24CXX_ReadWrite_FullCycle(void);
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries);
#ifdef __cplusplus
}
#endif

#endif


