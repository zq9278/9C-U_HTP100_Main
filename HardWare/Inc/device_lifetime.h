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


// 状态枚举定义
typedef enum {
    DEVICE_STATE_DISCONNECTED = 0,   // 未连接
    DEVICE_STATE_CONNECTED_IDLE,     // 已连接未开始使用
    DEVICE_STATE_ACTIVE,             // 正式使用中
    DEVICE_STATE_EXPIRED             // 已报废
} DeviceState_t;

// 状态机上下文结构体
typedef struct {
    DeviceState_t state;       // 当前状态
    uint16_t time_a_left;      // 剩余 A段寿命
    uint16_t time_b_left;      // 剩余 B段寿命
    bool started_usage;        // 是否已开始使用
    bool connected;            // 是否在线
} DeviceContext_t;

#ifdef __cplusplus
extern "C" {
#endif
// 初始化设备状态机
void Device_Init(void);

// 状态机运行（应定时调用）
void DeviceStateMachine_Update(void);

// 切换至正式使用状态（即从 A -> B）
void Device_StartUsage(void);

// 设备报废处理
void Device_MarkAsExpired(const char* reason);
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries);
#ifdef __cplusplus
}
#endif

#endif // DEVICE_LIFETIME_H
