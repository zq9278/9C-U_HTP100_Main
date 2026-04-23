/*
 * 文件: 24cxx.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef _24CXX_H
#define _24CXX_H
#include "stm32g0xx_hal.h"


#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767

#define EE_TYPE AT24C02




#define EEPROM_BQ27441Config_Add 0xFA

uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr);
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);
void AT24C02_WriteAllBytes(uint8_t value);
void AT24C02_WriteAllBytes_eye(uint8_t value);
uint8_t AT24CXX_Check(void);
void AT24CXX_Init(void);
void Heating_film_Check(void);
void prepare_data_set(void);

uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr);
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value);
uint16_t EYE_AT24CXX_Read(uint16_t startAddr);
HAL_StatusTypeDef EYE_AT24CXX_Write(uint16_t WriteAddr, uint16_t value) ;

HAL_StatusTypeDef EYE_AT24CXX_WriteByte(uint16_t addr, uint8_t data);
uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out);
HAL_StatusTypeDef EYE_AT24CXX_WriteUInt16(uint16_t addr, uint16_t value);
uint16_t EYE_AT24CXX_ReadUInt16(uint16_t addr);
HAL_StatusTypeDef EYE_AT24CXX_ReadUInt16Ex(uint16_t addr, uint16_t *value_out);

void AD24C01_Factory_formatted(void);
#endif


