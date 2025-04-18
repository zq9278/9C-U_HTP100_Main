/*
 * device_lifetime.c
 * 设备寿命管理模块 - 状态机实现
 */

#include "device_lifetime.h"
#include <string.h>

// 静态上下文实例
static DeviceContext_t device_ctx;
extern I2C_HandleTypeDef hi2c2;
extern SemaphoreHandle_t i2c2_mutex;
extern TimerHandle_t eye_is_existHandle;
char *i2c2_mutex_owner = NULL; // 当前持有锁的函数/任务名
prepare_data my_prepare_data_times;
extern volatile int eye_workingtime_1s;
extern volatile int eye_existtime_1s;
extern uint8_t EYE_exist_Flag;
extern uint8_t EYE_working_Flag;

// 初始化设备状态机（系统启动时调用）
void Device_Init(void) {
    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("初始化设备状态机完成，等待设备接入\n");
}

// 设置设备为报废状态
void Device_MarkAsExpired(const char* reason) {
    bool is_new_device = (device_ctx.started_usage == false);
    LOG("? 设备报废：%s\n", reason);
    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;

    if (is_new_device) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_Write(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("? 新设备使用次数已记录: %d\n", eye_times);
    }

    EYE_AT24CXX_Write(EYE_MARK_MAP, 0xFFFF);
    close_mianAPP();
    ScreenTimerStop();
    xTimerStop(eye_is_existHandle, 0);
}

// 外部调用：设备进入正式使用阶段（开始 B寿命）
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
        LOG("定时器 eye_is_existHandle 启动\n");
        LOG("? 设备进入正式使用阶段，开始 B段寿命\n");
    } else {
        LOG("?? 设备未处于可启动状态，StartUsage 调用失败\n");
    }
}

// I2C 检测设备是否在线（带互斥锁）
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    if (i2c2_mutex == NULL) {
        LOG("错误：I2C互斥锁未初始化！\n");
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
        LOG("I2C_CheckDevice：获取互斥锁失败！\n");
        return HAL_ERROR;
    }
}



// 主状态机轮询更新函数（高频调用，支持热插拔）
void DeviceStateMachine_Update(void) {
    bool online = (I2C_CheckDevice(0x91, 2) == HAL_OK);

    switch (device_ctx.state) {
        case DEVICE_STATE_DISCONNECTED:
            if (online) {
                LOG("? 检测到设备接入\n");
                device_ctx.connected = true;
                device_ctx.time_a_left = EYE_AT24CXX_Read(0xA0);
                device_ctx.time_b_left = EYE_AT24CXX_Read(0xB0);
                uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
                device_ctx.started_usage = (mark != 0xFFFF);
                device_ctx.state = device_ctx.started_usage ? DEVICE_STATE_ACTIVE : DEVICE_STATE_CONNECTED_IDLE;// 如果已使用则进入B段寿命，否则进入A段预扣状态

                uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
                my_prepare_data_times.cmd_type_low = 0xb0;
                my_prepare_data_times.value = eye_times;
                Eye_twitching_invalid_master(&my_prepare_data_times);

                LOG("主机端记录的次数: %d\n", eye_times);
                LOG("设备状态初始化：A=%d, B=%d, 标记=%d\n", device_ctx.time_a_left, device_ctx.time_b_left, device_ctx.started_usage);

                xTimerStart(eye_is_existHandle, 0);
            }
            break;

        case DEVICE_STATE_CONNECTED_IDLE://进入A段预扣状态
            if (!online) {
                LOG("?? 设备断开连接\n");
                device_ctx.connected = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                break;
            }
            if (eye_existtime_1s && device_ctx.time_a_left > 0) {
                device_ctx.time_a_left--;
                eye_existtime_1s = 0;
                LOG("? A段寿命剩余：%d\n", device_ctx.time_a_left);
            }
            if (device_ctx.time_a_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("A段寿命耗尽");
            }
            break;

        case DEVICE_STATE_ACTIVE:
            if (!online) {
                LOG("? 使用中设备断开，强制报废\n");
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("使用中断开连接");
                break;
            }
            if (eye_workingtime_1s && device_ctx.time_b_left > 0) {
                device_ctx.time_b_left--;
                eye_workingtime_1s = 0;
                LOG("? B段寿命剩余：%d\n", device_ctx.time_b_left);
            }
            if (device_ctx.time_b_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("B段寿命耗尽");
            }
            break;

        case DEVICE_STATE_EXPIRED:
            break;

        default:
            LOG("? 状态机未知状态：%d\n", device_ctx.state);
            break;
    }
}
