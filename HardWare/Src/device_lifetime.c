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
char *i2c2_mutex_owner = NULL;
prepare_data my_prepare_data_times;

extern volatile int eye_workingtime_1s;
extern volatile int eye_existtime_1s;
extern uint8_t EYE_exist_Flag;
extern uint8_t EYE_working_Flag;
extern uint8_t EYE_status;

// 初始化设备状态机（系统启动时调用）
void Device_Init(void) {
    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("初始化设备状态机完成，等待设备接入\n");
}
void Test_EEPROM_FullReadWrite_256B(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFF;  // 最大地址 256 字节 EEPROM
    const uint8_t  test_pattern_base = 0x5A;

    LOG("? 开始 EEPROM 全盘写入测试 (0x00 ~ 0xFF)...\n");

    // 写入测试数据
    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t data = test_pattern_base + (addr & 0xFF);
        AT24CXX_WriteOneByte(addr, data);
        osDelay(5);  // 等待写完成
    }

    LOG("? 写入完成，开始校验...\n");

    // 校验每个地址
    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t expected = test_pattern_base + (addr & 0xFF);
        uint8_t actual = AT24CXX_ReadOneByte(addr);

        if (actual != expected) {
            LOG("? 校验失败：地址 0x%02X，期望 0x%02X，读出 0x%02X\n", addr, expected, actual);
            return;
        }

        if ((addr % 16) == 0) {
            LOG("? 校验进度：0x%02X\n", addr);
        }
    }

    LOG("? EEPROM 全盘读写测试完成，数据一致！\n");
}
// 设置设备为报废状态
void Device_MarkAsExpired(const char* reason) {
    //bool is_new_device = (device_ctx.started_usage == false);
    uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
    bool is_new_device = (mark == 0xFFFF);  // ← 判断依据改成 EEPROM 标记位
    LOG("? 设备报废：%s\n", reason);
    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;

    if (EYE_AT24CXX_Read(EYE_EEPROM_USE_COUNT_FLAG) == 0xffff) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_Write(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("Normal: 新设备使用次数已记录: %d\n", eye_times);
        EYE_AT24CXX_Write( EYE_EEPROM_USE_COUNT_FLAG, 1);
    }
    close_mianAPP();
    ScreenTimerStop();
    xTimerStop(eye_is_existHandle, 0);
    EYE_status = 0;
}


// 外部调用：进入正式使用（切换到B寿命）
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
        LOG("定时器 eye_is_existHandle 启动\n");
        LOG("Normal: 设备进入正式使用阶段，开始 B段寿命\n");
    } else {
        LOG("Woring: 设备未处于可启动状态，StartUsage 调用失败\n");
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


// 主状态机轮询更新函数（高频调用）
void DeviceStateMachine_Update(void) {
    bool online = (I2C_CheckDevice(0x91, 2) == HAL_OK);

    switch (device_ctx.state) {
        case DEVICE_STATE_DISCONNECTED:
            if (online) {
                LOG("Normal: 检测到设备接入\n");
                device_ctx.connected = true;

                uint16_t mark = EYE_AT24CXX_Read(EYE_MARK_MAP);
                LOG("Debug: EYE_MARK_MAP 读取值 = 0x%04X\n", mark);
                osDelay(100);
                device_ctx.started_usage = (mark != 0xFFFF);
                LOG("Debug: started_usage = %d（0表示新设备，1表示已使用设备）\n", device_ctx.started_usage);

                // 总是读取当前 EEPROM 剩余寿命
                device_ctx.time_a_left = EYE_AT24CXX_Read(0xA0);
                device_ctx.time_b_left = EYE_AT24CXX_Read(0xB0);
                LOG("Debug: EEPROM A段寿命 = %d，B段寿命 = %d\n", device_ctx.time_a_left, device_ctx.time_b_left);

                // ? 已标记 + A/B 寿命任意为 0 → 直接进入报废
                if (device_ctx.started_usage ||device_ctx.time_a_left == 0 || device_ctx.time_b_left == 0) {
                    LOG("? 已标记设备寿命耗尽，直接进入报废状态！\n");
                    Device_MarkAsExpired("开机检测到报废");
                    device_ctx.state = DEVICE_STATE_EXPIRED;
                    xTimerStop(eye_is_existHandle, 0);
                    EYE_status = 0;
                    return;
                }

                // 若是新设备，进行 A/B 段校正写回
                if (!device_ctx.started_usage) {
                    if (device_ctx.time_a_left == 0xFFFF || device_ctx.time_a_left > DEVICE_LIFETIME_A_DEFAULT) {
                        LOG("Warning: 读取到非法 A段寿命值：%d，开始修正\n", device_ctx.time_a_left);
                        device_ctx.time_a_left = DEVICE_LIFETIME_A_DEFAULT;
                        if (EYE_AT24CXX_Write(0xA0, device_ctx.time_a_left) == HAL_OK) {
                            LOG("Correct: A段寿命已重写为默认值 %d\n", DEVICE_LIFETIME_A_DEFAULT);
                        } else {
                            LOG("Error: A段寿命重写失败\n");
                        }
                    } else {
                        LOG("Normal: 未标记设备，A段当前寿命=%d（最大为%d）\n", device_ctx.time_a_left, DEVICE_LIFETIME_A_DEFAULT);
                    }

                    if (device_ctx.time_b_left == 0xFFFF || device_ctx.time_b_left > DEVICE_LIFETIME_B_DEFAULT) {
                        LOG("Warning: 读取到非法 B段寿命值：%d，开始修正\n", device_ctx.time_b_left);
                        device_ctx.time_b_left = DEVICE_LIFETIME_B_DEFAULT;
                        if (EYE_AT24CXX_Write(0xB0, device_ctx.time_b_left) == HAL_OK) {
                            LOG("Correct: B段寿命已重写为默认值 %d\n", DEVICE_LIFETIME_B_DEFAULT);
                        } else {
                            LOG("Error: B段寿命重写失败\n");
                        }
                    } else {
                        LOG("Normal: 未标记设备，B段当前寿命=%d（最大为%d）\n", device_ctx.time_b_left, DEVICE_LIFETIME_B_DEFAULT);
                    }
                }

                device_ctx.state = device_ctx.started_usage ? DEVICE_STATE_ACTIVE : DEVICE_STATE_CONNECTED_IDLE;
                EYE_status = device_ctx.started_usage ? 0 : 1;
                LOG("Debug: 状态迁移为：%s\n", device_ctx.started_usage ? "ACTIVE（B段）" : "CONNECTED_IDLE（A段）");

                uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
                LOG("Debug: 读取主机使用次数：%d\n", eye_times);
                my_prepare_data_times.cmd_type_low = 0xb0;
                my_prepare_data_times.value = eye_times;
                Eye_twitching_invalid_master(&my_prepare_data_times);
                LOG("主机端记录的次数: %d\n", eye_times);
                LOG("设备状态初始化：A=%d, B=%d, 标记=%d\n", device_ctx.time_a_left, device_ctx.time_b_left, device_ctx.started_usage);

                xTimerStart(eye_is_existHandle, 0);
                LOG("定时器 eye_is_existHandle 启动\n");
            }

            break;




        case DEVICE_STATE_CONNECTED_IDLE://a段
            if (!online) {
                currentState = STATE_OFF;
                LOG("Woring: 设备断开连接\n");
                device_ctx.connected = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                EYE_status = 0;
                break;
            }

            if (eye_existtime_1s && device_ctx.time_a_left > 0) {
                device_ctx.time_a_left--;
                EYE_AT24CXX_Write(0xA0, device_ctx.time_a_left);  // ? 写回 EEPROM，确保断电后仍保留
                eye_existtime_1s = 0;
                LOG("Countdown: A段寿命剩余：%d\n", device_ctx.time_a_left);
            }

            if (device_ctx.time_a_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("A段寿命耗尽");
            }
            break;


        case DEVICE_STATE_ACTIVE://b段
            if (!online) {
                currentState = STATE_OFF;
                LOG("Woring: 使用中设备断开，强制报废\n");
                //device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("使用中断开连接");
                break;
            }
            if (eye_workingtime_1s && device_ctx.time_b_left > 0) {
                device_ctx.time_b_left--;
                eye_workingtime_1s = 0;
                LOG("Countdown: B段寿命剩余：%d\n", device_ctx.time_b_left);
            }
            if (device_ctx.time_b_left == 0) {
                device_ctx.state = DEVICE_STATE_EXPIRED;
                Device_MarkAsExpired("B段寿命耗尽");
            }
            break;

        case DEVICE_STATE_EXPIRED:
            if (!online) {
                LOG("Normal: 报废设备已拔出，等待新设备接入\n");
                device_ctx.connected = false;
                device_ctx.started_usage = false;
                device_ctx.state = DEVICE_STATE_DISCONNECTED;
                xTimerStop(eye_is_existHandle, 0);
                EYE_status = 0;
            }
            break;


        default:
            LOG("Woring: 状态机未知状态：%d\n", device_ctx.state);
            break;
    }

    // 实时发送设备状态（0 或 1）
    EYE_checkout((float)EYE_status);
}
