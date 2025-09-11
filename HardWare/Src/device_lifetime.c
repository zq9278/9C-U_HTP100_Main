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
    //bool is_new_device = (device_ctx.started_usage == false);
    uint16_t mark = EYE_AT24CXX_ReadUInt16(EYE_MARK_MAP);
    bool is_new_device = (mark == 0xFFFF);  // ← 判断依据改成 EEPROM 标记位
    LOG("? 设备报废：%s\n", reason);
    device_ctx.state = DEVICE_STATE_EXPIRED;
    device_ctx.connected = false;
    device_ctx.started_usage = true;
    close_mianAPP();
    ScreenTimerStop();
    xTimerStop(eye_is_existHandle, 0);
    EYE_status = 0;
    if (EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG) == 0xffff) {//
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("Normal: 新设备使用次数已记录: %d\n", eye_times);
        EYE_AT24CXX_WriteUInt16( EYE_EEPROM_USE_COUNT_FLAG, 1234);
//        osDelay(10);
//        uint16_t eye_timess=EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG);
    }
}


// 外部调用：进入正式使用（切换到B寿命）
bool  Device_StartUsage(void) {
    EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
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
        //return false;
    }
    if (EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG) == 0xffff) {//
        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        eye_times += 1;
        AT24CXX_WriteUInt16(0xf2, eye_times);
        EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, eye_workingtime_1s);
        LOG("Normal: 新设备使用次数已记录: %d\n", eye_times);
        EYE_AT24CXX_WriteUInt16( EYE_EEPROM_USE_COUNT_FLAG, 1234);
//        osDelay(10);
//        uint16_t eye_timess=EYE_AT24CXX_ReadUInt16(EYE_EEPROM_USE_COUNT_FLAG);
    }
    return true;
}

// // I2C 检测设备是否在线（带互斥锁）
// HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
//     HAL_StatusTypeDef result;
//
//     // 判断互斥锁是否已初始化，防止空指针操作
//     if (i2c2_mutex == NULL) {
//         LOG("错误：I2C互斥锁未初始化！\n");
//         return HAL_ERROR;
//     }
//
//     // 尝试获取 I2C2 互斥锁，超时时间100个tick
//     if (xSemaphoreTake(i2c2_mutex, 100) == pdTRUE) {
//         // 获取到互斥锁，进行设备检查
//         for (uint8_t i = 0; i < retries; i++) {
//             // 检查I2C设备是否准备好（即地址有响应）
//             result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 100);
//             if (result == HAL_OK) {
//                 // 检查通过，释放互斥锁并返回OK
//                 xSemaphoreGive(i2c2_mutex);
//                 i2c2_mutex_owner = NULL;  // 可选：清除所有者标记
//                 return HAL_OK;
//             }
//             // 未找到设备，延时3ms后重试
//             osDelay(50);
//         }
//         // 多次尝试后依旧失败，释放互斥锁并返回错误
//         xSemaphoreGive(i2c2_mutex);
//         return HAL_ERROR;
//     } else {
//         // 获取互斥锁失败，直接返回错误
//         LOG("I2C_CheckDevice：获取互斥锁失败！\n");
//         return HAL_ERROR;
//     }
// }

// I2C 检测设备是否在线（带互斥锁）
// // 必须连续 retries 次都成功才返回 HAL_OK
// HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
//     HAL_StatusTypeDef result;
//     uint8_t success_count = 0;
//
//     if (i2c2_mutex == NULL) {
//         LOG("错误：I2C互斥锁未初始化！\n");
//         return HAL_ERROR;
//     }
//
//     if (xSemaphoreTake(i2c2_mutex, 100) == pdTRUE) {
//         for (uint8_t i = 0; i < retries; i++) {
//             result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 100);
//             if (result == HAL_OK) {
//                 success_count++;
//             } else {
//                 // 只要有一次失败就提前退出
//                 break;
//             }
//             osDelay(10); // 每次检查之间的延时
//         }
//
//         xSemaphoreGive(i2c2_mutex);
//         i2c2_mutex_owner = NULL;
//
//         if (success_count == retries) {
//             return HAL_OK;   // 全部成功
//         } else {
//             return HAL_ERROR; // 有一次失败就算失败
//         }
//     } else {
//         LOG("I2C_CheckDevice：获取互斥锁失败！\n");
//         return HAL_ERROR;
//     }
// }


HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    uint8_t consecutive_success = 0;
    uint8_t consecutive_fail = 0;

    if (i2c2_mutex == NULL) {
        LOG("错误：I2C互斥锁未初始化！\n");
        return HAL_ERROR;
    }

    if (xSemaphoreTake(i2c2_mutex, 100) == pdTRUE) {
        while (1) {
            result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 100);
            if (result == HAL_OK) {
                consecutive_success++;
                consecutive_fail = 0; // 成功则失败计数清零
            } else {
                consecutive_fail++;
                consecutive_success = 0; // 失败则成功计数清零
            }

            if (consecutive_success >= retries) {
                xSemaphoreGive(i2c2_mutex);
                i2c2_mutex_owner = NULL;
                return HAL_OK;   // 连续 N 次成功
            }

            if (consecutive_fail >= retries) {
                xSemaphoreGive(i2c2_mutex);
                i2c2_mutex_owner = NULL;
                return HAL_ERROR; // 连续 N 次失败
            }

            osDelay(100);
        }
    } else {
        LOG("I2C_CheckDevice：获取互斥锁失败！\n");
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

    switch (eye_state) {
    case ST_OFFLINE:
        if (!online) {
            EYE_status = 0;
            close_mianAPP();
            break; // 继续保持离线，直到检测到上线
        }
        // 上线沿：仅在上线瞬间检查标志
        eye_state = ST_CHECK_MARK;
        break;

    case ST_CHECK_MARK:
        vTaskDelay(10);
        mark = EYE_AT24CXX_ReadUInt16(EYE_MARK_MAP);
        LOG("Debug: EYE_MARK_MAP 读取值 = 0x%04X\n", mark);
        osDelay(5);

        if (mark == 0xFFFF) {
            // 新设备：首次使用，写入标志并计数
            eye_times = AT24CXX_ReadOrWriteZero(0xF2);
            LOG("Debug: 读取主机使用次数：%d\n", eye_times);

            eye_times += 1;
            //EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, 1);
            AT24CXX_WriteUInt16(0xF2, eye_times);

            my_prepare_data_times.cmd_type_low = 0xB0;
            my_prepare_data_times.value = eye_times;
            Eye_twitching_invalid_master(&my_prepare_data_times);

            LOG("主机端记录的次数: %d\n", eye_times);
            EYE_status = 1;              // 本次插入会话允许使用
            eye_state = ST_ONLINE_NEW;   // 保持允许，直到拔出/断电
        } else {
            // 旧设备：不允许使用（实时禁止）
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_ONLINE_OLD;   // 保持禁止，直到拔出/断电
        }
        break;

    case ST_ONLINE_NEW:
        if (!online) {
            // 拔出/断电后，回到离线，下一次再插即按“旧设备”处理
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_OFFLINE;
        } else {
            // 仍在线：继续允许，不再重复读/写标志
            EYE_status = 1;
        }
        break;

    case ST_ONLINE_OLD:
        if (!online) {
            // 拔出/断电后回到离线
            eye_state = ST_OFFLINE;
        } else {
            // 旧设备在线：持续禁止
            EYE_status = 0;
            close_mianAPP();
        }
        break;

    default:
        eye_state = ST_OFFLINE;
        break;
    }

}