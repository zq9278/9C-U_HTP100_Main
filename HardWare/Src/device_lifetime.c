/*
 * device_lifetime.c
 * 设备寿命管理模块 - 状态机实现
 */

#include "device_lifetime.h"
#include <string.h>
#include "UserApp.h"
#include "interface_uart.h"
#include "24cxx.h"
#include "app_sys.h"
#include "Button.h"
// 静态上下文实例
DeviceContext_t device_ctx;
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
HAL_StatusTypeDef status;
// 初始化设备状态机（系统启动时调用）
void Device_Init(void) {
    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("初始化设备状态机完成，等待设备接入\n");
}
void Test_EYE_AT24CXX_ReadWrite_FullCycle(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFE;  // 注意：每次写2字节，最大地址必须为偶数
    const uint16_t test_base  = 0x5A00;

    LOG("? 开始 EYE_AT24CXX 读写测试 (0x00 ~ 0xFE)...\n");

    // 写入数据
    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        uint16_t value = test_base + addr;

        if (EYE_AT24CXX_WriteUInt16(addr, value) != HAL_OK) {
            LOG("? 写入失败：地址 0x%02X，值 0x%04X\n", addr, value);
            return;
        }
    }

    LOG("? 写入完成，开始校验...\n");

    // 读取校验
    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        HAL_StatusTypeDef status;
        uint16_t expected = test_base + addr;
        uint16_t actual   = EYE_AT24CXX_ReadUInt16(addr);



        if (actual != expected) {
            LOG("? 校验失败：地址 0x%02X，期望 0x%04X，读出 0x%04X\n", addr, expected, actual);
        }

        if ((addr % 16) == 0) {
            LOG("? 校验进度：0x%02X\n", addr);
        }
    }

    LOG("? EEPROM 测试完成，数据一致无误！\n");
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
    uint16_t mark = EYE_AT24CXX_ReadUInt16(EYE_MARK_MAP);
    bool is_new_device = (mark == 0xFFFF);

    // 状态变化：设备报废
    LOG("[STATE] Device expired, reason=\"%s\"\n", reason);

    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;

    close_mianAPP();
    ScreenTimerStop();

    if (xTimerStop(eye_is_existHandle, 0) != pdPASS) {
        LOG("[ERROR] Failed to stop eye_is_exist timer\n");
    }

    EYE_status = 0;

    // 第一次使用才更新 EEPROM
    if (EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG) == 0xFFFF) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xF2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xF2, eye_times);
        EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
        EYE_AT24CXX_WriteUInt16(EYE_EEPROM_USE_COUNT_FLAG, 1234);

        LOG("[STATE] New device usage recorded, count=%u\n", eye_times);
    }
}


// 外部调用：进入正式使用（切换到B寿命）
bool Device_StartUsage(void) {
    // 标记寿命起点
    EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
    LOG("[EEPROM] Write MARK_MAP=0x%04X @0x%04X\n", eye_workingtime_1s, EYE_MARK_MAP);

    my_prepare_data_times.cmd_head_high = 0x6A;
    my_prepare_data_times.cmd_head_low  = 0xA6;
    my_prepare_data_times.frame_length  = 0x0b;
    my_prepare_data_times.cmd_type_high = 0x00;
    my_prepare_data_times.end_high      = 0xFF;
    my_prepare_data_times.end_low       = 0xFF;

    if (device_ctx.state == DEVICE_STATE_CONNECTED_IDLE) {
        device_ctx.started_usage = true;
        device_ctx.time_a_left   = 0;
        device_ctx.state         = DEVICE_STATE_ACTIVE;

        if (xTimerStart(eye_is_existHandle, 0) != pdPASS) {
            LOG("[ERROR] Failed to start eye_is_existHandle timer\n");
        }

        LOG("[STATE] Device entered ACTIVE state (B-life started)\n");
    } else {
        LOG("[ERROR] Device not in CONNECTED_IDLE, StartUsage failed (state=%d)\n",
            device_ctx.state);
        return false;
    }

    uint16_t use_flag = EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG);
    LOG("[EEPROM] Read USE_COUNT_FLAG=0x%04X @0x%04X\n", use_flag, EYE_EEPROM_USE_COUNT_FLAG);

    if (use_flag == 0xFFFF) {
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xF2);
        LOG("[EEPROM] Read usage count=%u @0xF2\n", eye_times);

        eye_times += 1;
        AT24CXX_WriteUInt16(0xF2, eye_times);
        LOG("[EEPROM] Write usage count=%u @0xF2\n", eye_times);

        EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("[EEPROM] Write workingtime=%u @MARK_MAP\n", eye_workingtime_1s);

        EYE_AT24CXX_WriteUInt16(EYE_EEPROM_USE_COUNT_FLAG, 1234);
        LOG("[EEPROM] Write USE_COUNT_FLAG=1234 @0x%04X\n", EYE_EEPROM_USE_COUNT_FLAG);

        LOG("[STATE] New device usage recorded, count=%u\n", eye_times);
    }

    return true;
}

// 仅在进入 PRE_* 状态时调用，延后标记未写入过的普通眼盾，
// 避免设备上电检测到新眼盾后立即写入标记。
void Device_TryMarkNormalEyeShield(void) {
    uint16_t mark = EYE_AT24CXX_ReadUInt16(EYE_MARK_MAP);

    if (mark != 0xFFFF) {
        return;
    }

    if (EYE_AT24CXX_ReadUInt16(super_eyes) == 0x0202) {
        return;
    }

    EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, 1);
    LOG("[STATE] Normal eye shield marked on PRE state entry\n");
}

HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    uint8_t consecutive_success = 0;
    uint8_t consecutive_fail = 0;

    if (i2c2_mutex == NULL) {
        LOG("[ERROR] I2C mutex not initialized!\n");
        return HAL_ERROR;
    }

    if (xSemaphoreTake(i2c2_mutex, 100) == pdTRUE) {
        while (1) {
            result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 100);
            if (result == HAL_OK) {
                consecutive_success++;
                consecutive_fail = 0;
            } else {
                consecutive_fail++;
                consecutive_success = 0;
            }

            if (consecutive_success >= retries) {
                xSemaphoreGive(i2c2_mutex);
                i2c2_mutex_owner = NULL;
                // LOG("[STATE] I2C device 0x%02X detected OK (%u consecutive)\n",
                //     i2c_addr, consecutive_success);
                return HAL_OK;   // 连续 N 次成功
            }

            if (consecutive_fail >= retries) {
                xSemaphoreGive(i2c2_mutex);
                i2c2_mutex_owner = NULL;
                // LOG("[ERROR] I2C device 0x%02X not responding (%u consecutive fails)\n",
                //     i2c_addr, consecutive_fail);
                return HAL_ERROR; // 连续 N 次失败
            }

            osDelay(100);
        }
    } else {
        LOG("[ERROR] I2C_CheckDevice: failed to get mutex\n");
        return HAL_ERROR;
    }
}




typedef enum {
    ST_CHECK_ONLINE = 0,
    ST_OFFLINE,
    ST_CHECK_MARK,
    ST_ONLINE_NEW,
    ST_ONLINE_OLD
} EyeState;
void DeviceStateMachine_Update(void) {
    bool online = (I2C_CheckDevice(0x91, 4) == HAL_OK);
    uint16_t mark = 0;
    uint16_t eye_times = 0;

    static EyeState eye_state = ST_CHECK_ONLINE;
    EyeState prev_state = eye_state; // 保存上一次状态，方便判断是否切换

    switch (eye_state) {
    case ST_OFFLINE:
        if (!online) {
            if (EYE_status != 0) {
                EYE_status = 0;
                close_mianAPP();
                LOG("[STATE] EYE set OFFLINE\n");
            }
            break; // 保持离线
        }
        // -> 上线
        eye_state = ST_CHECK_MARK;
        LOG("[STATE] Transition: OFFLINE -> CHECK_MARK\n");
        break;

    case ST_CHECK_MARK:
        vTaskDelay(10);
        mark = EYE_AT24CXX_ReadUInt16(EYE_MARK_MAP);
        LOG("[EEPROM] Read EYE_MARK_MAP=0x%04X\n", mark);
        osDelay(5);

        if (mark == 0xFFFF) {
            // 新设备
            eye_times = AT24CXX_ReadOrWriteZero(0xF2);
            eye_times += 1;
            AT24CXX_WriteUInt16(0xF2, eye_times);

            my_prepare_data_times.cmd_type_low = 0xB0;
            my_prepare_data_times.value = eye_times;
            Eye_twitching_invalid_master(&my_prepare_data_times);

            EYE_status = 1;
            eye_state = ST_ONLINE_NEW;
            LOG("[STATE] New device ONLINE, count=%u\n", eye_times);
        } else {
            // 旧设备
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_ONLINE_OLD;
            LOG("[STATE] Old device ONLINE, usage denied\n");
        }
        break;

    case ST_ONLINE_NEW:
        if (!online) {
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_OFFLINE;
            LOG("[STATE] Transition: ONLINE_NEW -> OFFLINE (removed)\n");
        } else {
            EYE_status = 1; // 保持在线新设备
        }
        break;

    case ST_ONLINE_OLD:
        if (!online) {
            eye_state = ST_OFFLINE;
            LOG("[STATE] Transition: ONLINE_OLD -> OFFLINE (removed)\n");
        } else {
            if (EYE_status != 0) {
                EYE_status = 0;
                close_mianAPP();
                LOG("[STATE] Old device still ONLINE, access denied\n");
            }
        }
        break;

    default:
        eye_state = ST_OFFLINE;
        LOG("[WARN] Invalid state detected, reset -> OFFLINE\n");
        break;
    }

    // 调试辅助：状态切换检测
    if (eye_state != prev_state) {
        LOG("[DEBUG] State changed: %d -> %d\n", prev_state, eye_state);
    }
}
