#ifndef __BQ25895_H
#define __BQ25895_H

#include "stm32g0xx_hal.h"
#define CHG_CE(n)			(n?HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_SET):HAL_GPIO_WritePin(CHG_CE_GPIO_Port,CHG_CE_Pin,GPIO_PIN_RESET))
#define BQ25895Address	0xd4

typedef enum {
    STATE_POWER_ON,       // 开机：白色常亮
    STATE_WORKING,        // 工作：白色常亮
    STATE_LOW_BATTERY,    // 电量不足：白色闪烁
    STATE_CHARGING,       // 充电：白色呼吸
    STATE_CHARGED,        // 充满电：白色常亮
    STATE_EMERGENCY_STOP  // 急停：黄色常亮
} ChargeState_t;

void BQ25895_Init(void);
void BQ25895_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ25895_MultiRead(uint8_t* pBuffer);
void BQ25895_Write(uint8_t WriteAddr,uint8_t WriteData);
void UpdateChargeState_bq25895(void);
HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) ;

HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) ;

		 
#endif

