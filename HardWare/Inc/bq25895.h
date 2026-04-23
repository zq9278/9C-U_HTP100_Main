/*
 * 文件: bq25895.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __BQ25895_H
#define __BQ25895_H

#include "stm32g0xx_hal.h"
#define CHG_CE(n)			(n?HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_RESET))
#define BQ25895Address	0xd4
#define I2C_TIMEOUT_MS 200
typedef enum {
    STATE_POWER_ON,
    STATE_WORKING,
    STATE_LOW_BATTERY,
    STATE_CHARGING,
    STATE_CHARGED,
    STATE_EMERGENCY_STOP
} ChargeState_t;
extern uint8_t charging_flag;
void BQ25895_Init(void);
void BQ25895_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ25895_MultiRead(uint8_t* pBuffer);
void BQ25895_Write(uint8_t WriteAddr,uint8_t WriteData);
void UpdateChargeState_bq25895(void);
HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) ;
void bq25895_reinitialize_if_vbus_inserted(void);
HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) ;
void BQ25895_AutoRecover(void);


#endif



