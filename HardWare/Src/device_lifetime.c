/*
 * 鏂囦欢: device_lifetime.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "device_lifetime.h"
#include <string.h>
#include "UserApp.h"
#include "interface_uart.h"
#include "24cxx.h"
#include "app_sys.h"
#include "Button.h"

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

static volatile uint8_t g_pending_mark_normal_eye_shield = 0;

/**
 * @brief Device_Init 鍑芥暟瀹炵幇銆? */
void Device_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    memset(&device_ctx, 0, sizeof(DeviceContext_t));
    device_ctx.state = DEVICE_STATE_DISCONNECTED;
    LOG("鍒濆鍖栬澶囩姸鎬佹満瀹屾垚锛岀瓑寰呰澶囨帴鍏n");
}
void Test_EYE_AT24CXX_ReadWrite_FullCycle(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFE;
    const uint16_t test_base  = 0x5A00;

    LOG("? 寮€濮?EYE_AT24CXX 璇诲啓娴嬭瘯 (0x00 ~ 0xFE)...\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        uint16_t value = test_base + addr;

        if (EYE_AT24CXX_WriteUInt16(addr, value) != HAL_OK) {
            LOG("? 鍐欏叆澶辫触锛氬湴鍧€ 0x%02X锛屽€?0x%04X\n", addr, value);
            return;
        }
    }

    LOG("? 鍐欏叆瀹屾垚锛屽紑濮嬫牎楠?..\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        HAL_StatusTypeDef status;
        uint16_t expected = test_base + addr;
        uint16_t actual   = EYE_AT24CXX_ReadUInt16(addr);



        if (actual != expected) {
            LOG("? 鏍￠獙澶辫触锛氬湴鍧€ 0x%02X锛屾湡鏈?0x%04X锛岃鍑?0x%04X\n", addr, expected, actual);
        }

        if ((addr % 16) == 0) {
            LOG("? 鏍￠獙杩涘害锛?x%02X\n", addr);
        }
    }

    LOG("? EEPROM 娴嬭瘯瀹屾垚锛屾暟鎹竴鑷存棤璇紒\n");
}


void Test_EEPROM_FullReadWrite_256B(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFF;
    const uint8_t  test_pattern_base = 0x5A;

    LOG("? 寮€濮?EEPROM 鍏ㄧ洏鍐欏叆娴嬭瘯 (0x00 ~ 0xFF)...\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t data = test_pattern_base + (addr & 0xFF);
        AT24CXX_WriteOneByte(addr, data);
        osDelay(5);
    }

    LOG("? 鍐欏叆瀹屾垚锛屽紑濮嬫牎楠?..\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t expected = test_pattern_base + (addr & 0xFF);
        uint8_t actual = AT24CXX_ReadOneByte(addr);

        if (actual != expected) {
            LOG("? 鏍￠獙澶辫触锛氬湴鍧€ 0x%02X锛屾湡鏈?0x%02X锛岃鍑?0x%02X\n", addr, expected, actual);
            return;
        }

        if ((addr % 16) == 0) {
            LOG("? 鏍￠獙杩涘害锛?x%02X\n", addr);
        }
    }

    LOG("? EEPROM 鍏ㄧ洏璇诲啓娴嬭瘯瀹屾垚锛屾暟鎹竴鑷达紒\n");
}



/**
 * @brief Device_TryMarkNormalEyeShield 鍑芥暟瀹炵幇銆? */
void Device_TryMarkNormalEyeShield(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    uint16_t mark = 0;
    uint16_t super_mark = 0;
    LOG("[MARK] formal-entry mark attempt begin\n");

    if (EYE_AT24CXX_ReadUInt16Ex(EYE_MARK_MAP, &mark) != HAL_OK) {
        LOG("[ERROR] Read EYE_MARK_MAP failed, skip mark\n");
        return;
    }
    LOG("[MARK] EYE_MARK_MAP=0x%04X\n", mark);

    if (mark != 0xFFFF) {
        LOG("[MARK] skip: already marked\n");
        return;
    }

    if (EYE_AT24CXX_ReadUInt16Ex(super_eyes, &super_mark) != HAL_OK) {
        LOG("[ERROR] Read super_eyes failed, skip mark\n");
        return;
    }
    LOG("[MARK] super_eyes=0x%04X\n", super_mark);

    if (super_mark == 0x0202) {
        LOG("[MARK] skip: super eye shield (0x0202)\n");
        return;
    }



    uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xF2);
    eye_times += 1;
    AT24CXX_WriteUInt16(0xF2, eye_times);

    for (uint8_t retry = 0; retry < 3; retry++) {
        uint16_t verify_mark = 0;

        if (EYE_AT24CXX_WriteUInt16(EYE_MARK_MAP, 1) == HAL_OK &&
            EYE_AT24CXX_ReadUInt16Ex(EYE_MARK_MAP, &verify_mark) == HAL_OK &&
            verify_mark == 1) {
            LOG("[STATE] Normal eye shield marked on formal state entry, retry=%u\n", retry);
            return;
        }
        LOG("[MARK] retry=%u verify failed\n", retry);
        osDelay(8);
    }
    LOG("[ERROR] Normal eye shield mark failed after 3 retries\n");
}

/**
 * @brief Device_RequestMarkNormalEyeShield 鍑芥暟瀹炵幇銆? */
void Device_RequestMarkNormalEyeShield(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    g_pending_mark_normal_eye_shield = 1;
}

/**
 * @brief Device_HandlePendingMarkRequest 鍑芥暟瀹炵幇銆? */
void Device_HandlePendingMarkRequest(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    if (g_pending_mark_normal_eye_shield == 0) {
        return;
    }
    if (EYE_status != 1) {
        return;
    }
    g_pending_mark_normal_eye_shield = 0;
    Device_TryMarkNormalEyeShield();
}

/**
 * @brief I2C_CheckDevice 鍑芥暟瀹炵幇銆? * @param i2c_addr 鍙傛暟銆? * @param retries 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    HAL_StatusTypeDef result;
    uint8_t consecutive_success = 0;
    uint8_t consecutive_fail = 0;

    if (i2c2_mutex == NULL) {
        LOG("[ERROR] I2C mutex not initialized!\n");
        return HAL_ERROR;
    }

    while (1) {
        if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(40)) == pdTRUE) {
            result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, 50);
            xSemaphoreGive(i2c2_mutex);
            i2c2_mutex_owner = NULL;
        } else {
            result = HAL_BUSY;
        }

        if (result == HAL_OK) {
            consecutive_success++;
            consecutive_fail = 0;
        } else {
            consecutive_fail++;
            consecutive_success = 0;
        }

        if (consecutive_success >= retries) {
            return HAL_OK;
        }

        if (consecutive_fail >= retries) {
            return HAL_ERROR;
        }

        osDelay(20);
    }
}

typedef enum {
    ST_CHECK_ONLINE = 0,
    ST_OFFLINE,
    ST_CHECK_MARK,
    ST_ONLINE_NEW,
    ST_ONLINE_OLD
} EyeState;
/**
 * @brief DeviceStateMachine_Update 鍑芥暟瀹炵幇銆? */
void DeviceStateMachine_Update(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    bool online = (I2C_CheckDevice(0x91, 4) == HAL_OK);
    uint16_t mark = 0;
    uint16_t eye_times = 0;

    static EyeState eye_state = ST_CHECK_ONLINE;
    EyeState prev_state = eye_state;

    switch (eye_state) {
    case ST_OFFLINE:
        if (!online) {
            g_pending_mark_normal_eye_shield = 0;
            if (EYE_status != 0) {
                EYE_status = 0;
                close_mianAPP();
                LOG("[STATE] EYE set OFFLINE\n");
            }
            break;
        }

        eye_state = ST_CHECK_MARK;
        LOG("[STATE] Transition: OFFLINE -> CHECK_MARK\n");
        break;

    case ST_CHECK_MARK:

        vTaskDelay(10);
        if (EYE_AT24CXX_ReadUInt16Ex(EYE_MARK_MAP, &mark) != HAL_OK) {
            LOG("[EEPROM] Read EYE_MARK_MAP failed\n");
            g_pending_mark_normal_eye_shield = 0;
            eye_state = ST_OFFLINE;
            EYE_status = 0;
            close_mianAPP();
            break;
        }
        LOG("[EEPROM] Read EYE_MARK_MAP=0x%04X\n", mark);
        osDelay(5);

        if (mark == 0xFFFF) {

            g_pending_mark_normal_eye_shield = 0;









            EYE_status = 1;
            eye_state = ST_ONLINE_NEW;
            LOG("[STATE] New device ONLINE, count=%u\n", eye_times);
        } else {

            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_ONLINE_OLD;
            LOG("[STATE] Old device ONLINE, usage denied\n");
        }
        break;

    case ST_ONLINE_NEW:
        if (!online) {
            g_pending_mark_normal_eye_shield = 0;
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_OFFLINE;
            LOG("[STATE] Transition: ONLINE_NEW -> OFFLINE (removed)\n");
        } else {
            EYE_status = 1;
        }
        break;

    case ST_ONLINE_OLD:
        if (!online) {
            g_pending_mark_normal_eye_shield = 0;
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


    if (eye_state != prev_state) {
        LOG("[DEBUG] State changed: %d -> %d\n", prev_state, eye_state);
    }
}


