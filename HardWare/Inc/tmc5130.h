/*
 * 文件: tmc5130.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __TMC5130_H
#define __TMC5130_H
#include "stm32g0xx_hal.h"
#define TMC_ENN(n)		(n?HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_RESET))
#define TMC_CSN(n)		(n?HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET))
#define Positive 1
#define Negative 2
#ifndef ENABLE_PRESSURE_LEVEL_PID_TUNING
#define ENABLE_PRESSURE_LEVEL_PID_TUNING 1
#endif

void TMC5130_Init(void);
void TMC5130_Read(uint8_t ReadAddr, uint8_t *pBuffer);
void TMC5130_Write(uint8_t WriteAddr, uint32_t WriteData);
void MotorCtrl(int32_t Step);
void StepMinMax(int32_t *Step, int32_t MinValue, int32_t MaxValue);
void MotorSetHome(void);
uint8_t MotorChecking(void);
void VelocityModeMove(uint8_t direction);
void MotorMove(uint8_t DriverID, int16_t ADCValue, uint16_t absADCValue, uint16_t ADCThreshold, uint16_t MotorSpeed);
uint8_t MotorCompare(int32_t SetData, int32_t CompareData);

void SetMotorposition(int speed);
void SetMotorSpeed(int speed);

void PressureDisplayTargetFilterReset(void);
float PressureDisplayTargetFilterUpdate(float measured_value, float target_value);
#if ENABLE_PRESSURE_LEVEL_PID_TUNING
uint8_t PressurePIDSetByLevel(float pressure_level_mmhg, float kp, float ki, float kd);
#endif
void PressureControlReset(void);
void PressureControl(float pid_dt_s);
#endif



