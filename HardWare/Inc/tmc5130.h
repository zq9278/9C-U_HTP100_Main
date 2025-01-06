/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-20 11:04:03
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\tmc5130\tmc5130.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __TMC5130_H
#define __TMC5130_H
#include "stm32g0xx_hal.h"
#define TMC_ENN(n)		(n?HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_ENN_GPIO_Port,TMC_ENN_Pin,GPIO_PIN_RESET))
#define TMC_CSN(n)		(n?HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(TMC_CSN_GPIO_Port,TMC_CSN_Pin,GPIO_PIN_RESET))
#define Positive 1
#define Negative 2

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

void PressureControl(void);
#endif
