/*
 * 文件: fault_code.c
 * 说明: 故障码通过屏幕串口协议上报。
 */
#include "fault_code.h"
#include "communication.h"
#include "interface_uart.h"

#define FAULT_CODE_REPEAT_INTERVAL_MS 1000U

static FaultCode_t g_last_fault_code = FAULT_CODE_NONE;
static uint32_t g_last_fault_send_tick = 0U;

static void FaultCode_SendToScreen(FaultCode_t code)
{
    static uint32_data pData;

    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = (uint8_t)(COMM_RESP_FAULT_CODE >> 8);
    pData.cmd_type_low = (uint8_t)(COMM_RESP_FAULT_CODE & 0xFF);
    pData.value = (uint32_t)code;
    pData.crc = Calculate_CRC((uint8_t *)&pData, sizeof(pData) - 4);
    pData.end_high = 0xff;
    pData.end_low = 0xff;
    ScreenTx_Post((uint8_t *)&pData, sizeof(pData), pdMS_TO_TICKS(20));
}

void FaultCode_Report(FaultCode_t code)
{
    if (code == FAULT_CODE_NONE) {
        return;
    }

    if (code == g_last_fault_code) {
        return;
    }

    g_last_fault_code = code;
    FaultCode_SendToScreen(code);
    g_last_fault_send_tick = HAL_GetTick();
    LOGE("[Fault] code=0x%08lX\r\n", (uint32_t)code);
}

void FaultCode_Clear(void)
{
    g_last_fault_code = FAULT_CODE_NONE;
    g_last_fault_send_tick = 0U;
}

void FaultCode_ClearCode(FaultCode_t code)
{
    if (g_last_fault_code == code) {
        FaultCode_Clear();
    }
}

void FaultCode_Poll(void)
{
    uint32_t now_tick;

    if (g_last_fault_code == FAULT_CODE_NONE) {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - g_last_fault_send_tick) >= FAULT_CODE_REPEAT_INTERVAL_MS) {
        FaultCode_SendToScreen(g_last_fault_code);
        g_last_fault_send_tick = now_tick;
    }
}

FaultCode_t FaultCode_GetLast(void)
{
    return g_last_fault_code;
}
