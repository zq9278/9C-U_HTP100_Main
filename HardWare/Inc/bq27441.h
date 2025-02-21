#ifndef __BQ27441_H
#define __BQ27441_H
#include "stm32g0xx_hal.h"

#define BQ27441Address	0xaa


typedef struct
{
  uint16_t Voltage;
	uint16_t Temperature;
	int16_t AvgCurrent;
	uint16_t SOC;
	uint16_t FullChargeCapacity;
}BQ27441_typedef;

void I2C_Semaphore_Init(void);
uint8_t BQ27441_Init(void);
void BQ27441_Read(uint8_t ReadAddr,uint8_t* pBuffer);
void BQ27441_MultiRead(BQ27441_typedef *BQ_State);
void BQ27441_WriteByte(uint8_t WriteAddr,uint8_t WriteData);
void BQ27441_WriteWord(uint8_t WriteAddr,uint16_t WriteData);
void battery_status_update_bq27441(void);
void BATCheckDIS(void);
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ_State);
HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size);
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size);

#endif

