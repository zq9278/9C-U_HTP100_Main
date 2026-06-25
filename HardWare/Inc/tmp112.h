/*
 * 文件: tmp112.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __TMP112_H
#define __TMP112_H
#include "stm32g0xx_hal.h"


#define TMP112_ADDR (0x48U << 1)









typedef struct {
    float Q;
    float R;
    float P;
    float K;
    float x;
} KalmanFilter;
extern KalmanFilter kf;
void Kalman_Init(KalmanFilter *kf, float Q, float R);
float Kalman_Update(KalmanFilter *kf, float measurement);
void TempDisplayTargetFilterReset(void);
float TempDisplayTargetFilterUpdate(float measured_value, float target_value);
void TMP112_Init(void);
HAL_StatusTypeDef TMP112_WriteWord(uint8_t WriteAddr,uint8_t*  WriteData);
HAL_StatusTypeDef TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer);



float TmpRaw2Ture(void);
uint8_t TMP112_IsDevicePresent(void);
extern uint8_t IIC_EYETimeoutFlag;
extern uint8_t EYE_exist_Flag, EYE_working_Flag, EYE_exist_new_Flag,EYE_status;
#endif


