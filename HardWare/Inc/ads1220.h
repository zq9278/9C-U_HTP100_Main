/*
 * 文件: ads1220.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __ADS1220_H
#define __ADS1220_H
#include "stm32g0xx_hal.h"

#define ADS1220_CMD_RESET 0x06
#define ADS1220_CMD_SELFCAL 0xF0
#define ADS1220_CMD_START_SYNC 0x08
#define ADS1220_CMD_RDATA 0x10
#define ADS1220_CMD_POWERDOWN 0x02
#define ADS1220_CMD_RREG 0x23


#define ADS1220_REG_CONFIG0 0x00
#define ADS1220_REG_CONFIG1 0x01
#define ADS1220_REG_CONFIG2 0x02
#define ADS1220_REG_CONFIG3 0x03


#define SENSITIVITY 0.365
#define SENSITIVITY_150 0.380
#define SENSITIVITY_250 0.370
#define SENSITIVITY_350 0.370
#define SENSITIVITY_450 0.370
#define SENSITIVITY_550 0.370
#define MAX_WEIGHT 2.0
#define EXCITATION_VOLTAGE 3.3
#define GAIN 128
#define VREF 3.3
extern SPI_HandleTypeDef hspi2;


#define ADS1220_CS_LOW()                                                       \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define ADS1220_CS_HIGH()                                                      \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

void ADS1220_WriteRegister(uint8_t reg, uint8_t value);
void ADS1220_Init(void);
void ADS1220_ReinitRegisters(void);
void ADS1220_StartConversion(void);
void ADS1220_StopConversion(void);
int32_t ADS1220_ReadData(void);
float ADS1220_ReadPressure(void);
void Discard_dirty_data(void);





#include "main.h"
#include <stdint.h>





void SPI2_DMA_Semaphores_Init(void);


HAL_StatusTypeDef SPI2_TransmitReceive_DMA(uint8_t *txData, uint8_t *rxData, uint16_t size, uint32_t timeout);

HAL_StatusTypeDef SPI2_Transmit_DMA(uint8_t *txData, uint16_t size, uint32_t timeout);

HAL_StatusTypeDef SPI2_Receive_DMA(uint8_t *rxData, uint16_t size, uint32_t timeout);

#endif




