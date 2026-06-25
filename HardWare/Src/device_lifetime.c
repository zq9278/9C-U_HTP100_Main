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
#include "tmp112.h"
#include "fault_code.h"

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
    LOGI("[Device] Event");
}
void Test_EYE_AT24CXX_ReadWrite_FullCycle(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFE;
    const uint16_t test_base  = 0x5A00;

    LOGI("[Device] Event\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        uint16_t value = test_base + addr;

        if (EYE_AT24CXX_WriteUInt16(addr, value) != HAL_OK) {
            LOGE("[Device] Write failed: addr=0x%02X, value=0x%04X\n", addr, value);
            return;
        }
    }

    LOGI("[Device] Event\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr += 2) {
        HAL_StatusTypeDef status;
        uint16_t expected = test_base + addr;
        uint16_t actual   = EYE_AT24CXX_ReadUInt16(addr);



        if (actual != expected) {
            LOGE("[Device] Verify failed: addr=0x%02X, expected=0x%04X, actual=0x%04X\n", addr, expected, actual);
        }

        if ((addr % 16) == 0) {
            LOGI("[Device] Verify progress: addr=0x%02X\n", addr);
        }
    }

    LOGI("[Device] Event\n");
}


void Test_EEPROM_FullReadWrite_256B(void)
{
    const uint16_t start_addr = 0x00;
    const uint16_t end_addr   = 0xFF;
    const uint8_t  test_pattern_base = 0x5A;

    LOGI("[Device] Event\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t data = test_pattern_base + (addr & 0xFF);
        AT24CXX_WriteOneByte(addr, data);
        osDelay(5);
    }

    LOGI("[Device] Event\n");


    for (uint16_t addr = start_addr; addr <= end_addr; addr++) {
        uint8_t expected = test_pattern_base + (addr & 0xFF);
        uint8_t actual = AT24CXX_ReadOneByte(addr);

        if (actual != expected) {
            LOGE("[Device] Verify failed: addr=0x%02X, expected=0x%02X, actual=0x%02X\n", addr, expected, actual);
            return;
        }

        if ((addr % 16) == 0) {
            LOGI("[Device] Verify progress: addr=0x%02X\n", addr);
        }
    }

    LOGI("[Device] Event\n");
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
    LOGI("[Device] Event\n");

    if (EYE_AT24CXX_ReadUInt16Ex(EYE_MARK_MAP, &mark) != HAL_OK) {
        LOGE("[Device] Event\n");
        return;
    }
    LOGI("[Device] Read EYE_MARK_MAP=0x%04X\n", mark);

    if (mark != 0xFFFF) {
        LOGW("[Device] Event\n");
        return;
    }

    if (EYE_AT24CXX_ReadUInt16Ex(super_eyes, &super_mark) != HAL_OK) {
        LOGE("[Device] Event\n");
        return;
    }
    LOGI("[Device] Read super_eyes=0x%04X\n", super_mark);

    if (super_mark == 0x0202) {
        LOGW("[Device] Event\n");
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
            LOGI("[Device] Normal eye shield marked, retry=%u\n", retry);
            return;
        }
        LOGI("[Device] Mark verify retry=%u\n", retry);
        osDelay(8);
    }
    LOGE("[Device] Event\n");
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
        LOGE("[Device] Event\n");
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
            if (i2c_addr == TMP112_ADDR) {
                FaultCode_ClearCode(FAULT_CODE_TMP112_COMM_ERROR);
            }
        } else {
            consecutive_fail++;
            consecutive_success = 0;
        }

        if (consecutive_success >= retries) {
            return HAL_OK;
        }

        if (consecutive_fail >= retries) {
            I2C2_RequestRecovery();
            if (i2c_addr == TMP112_ADDR) {
                FaultCode_Report(FAULT_CODE_TMP112_COMM_ERROR);
            }
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

    bool online = (I2C_CheckDevice(TMP112_ADDR, 4) == HAL_OK);
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
                LOGW("[Device] Event\n");
            }
            break;
        }

        eye_state = ST_CHECK_MARK;
        LOGW("[Device] Event\n");
        break;

    case ST_CHECK_MARK:

        vTaskDelay(10);
        if (EYE_AT24CXX_ReadUInt16Ex(EYE_MARK_MAP, &mark) != HAL_OK) {
            LOGE("[Device] Event\n");
            g_pending_mark_normal_eye_shield = 0;
            eye_state = ST_OFFLINE;
            EYE_status = 0;
            close_mianAPP();
            break;
        }
        LOGI("[Device] Read EYE_MARK_MAP=0x%04X\n", mark);
        osDelay(5);

        if (mark == 0xFFFF) {

            g_pending_mark_normal_eye_shield = 0;









            EYE_status = 1;
            eye_state = ST_ONLINE_NEW;
            LOGI("[Device] New device online, count=%u\n", eye_times);
        } else {

            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_ONLINE_OLD;
            LOGW("[Device] Event\n");
        }
        break;

    case ST_ONLINE_NEW:
        if (!online) {
            g_pending_mark_normal_eye_shield = 0;
            EYE_status = 0;
            close_mianAPP();
            eye_state = ST_OFFLINE;
            LOGW("[Device] Event\n");
        } else {
            EYE_status = 1;
        }
        break;

    case ST_ONLINE_OLD:
        if (!online) {
            g_pending_mark_normal_eye_shield = 0;
            eye_state = ST_OFFLINE;
            LOGW("[Device] Event\n");
        } else {
            if (EYE_status != 0) {
                EYE_status = 0;
                close_mianAPP();
                LOGW("[Device] Event\n");
            }
        }
        break;

    default:
        eye_state = ST_OFFLINE;
        LOGW("[Device] Event\n");
        break;
    }


    if (eye_state != prev_state) {
        LOGI("[Device] State changed: %d -> %d\n", prev_state, eye_state);
    }
}
