/*
 * 文件: fault_code.h
 * 说明: 设备故障码定义与屏幕上报接口。
 */
#ifndef FAULT_CODE_H
#define FAULT_CODE_H

#include <stdint.h>

typedef enum {
    FAULT_CODE_NONE = 0x00000000U,
    FAULT_CODE_TMP112_COMM_ERROR = 0x00000101U,
    FAULT_CODE_MOTOR_NOT_FOUND = 0x00000201U,
    FAULT_CODE_MOTOR_DRIVER_COMM_ERROR = 0x00000202U,
    FAULT_CODE_MOTOR_HOME_FAILED = 0x00000203U,
    FAULT_CODE_PRESSURE_SENSOR_NOT_FOUND = 0x00000301U,
    FAULT_CODE_PRESSURE_DRIVER_COMM_ERROR = 0x00000302U,
} FaultCode_t;

void FaultCode_Report(FaultCode_t code);
void FaultCode_Clear(void);
void FaultCode_ClearCode(FaultCode_t code);
void FaultCode_Poll(void);
FaultCode_t FaultCode_GetLast(void);

#endif
