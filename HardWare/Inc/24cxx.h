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
//#define EYE_MARK_MAP 0x02



#define EEPROM_BQ27441Config_Add 0xFA
					  
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr);							//Ö¸ï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½È¡Ò»ï¿½ï¿½ï¿½Ö½ï¿½
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//Ö¸ï¿½ï¿½ï¿½ï¿½Ö·Ð´ï¿½ï¿½Ò»ï¿½ï¿½ï¿½Ö½ï¿½
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//Ö¸ï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½Ê¼Ð´ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ï¿½ï¿½Èµï¿½ï¿½ï¿½ï¿½ï¿½
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);					//Ö¸ï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½Ê¼ï¿½ï¿½È¡Ö¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½Ê¼Ð´ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ï¿½ï¿½Èµï¿½ï¿½ï¿½ï¿½ï¿½
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ï¿½Ö·ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½Ö¸ï¿½ï¿½ï¿½ï¿½ï¿½Èµï¿½ï¿½ï¿½ï¿½ï¿½
void AT24C02_WriteAllBytes(uint8_t value);// µ¥×Ö½ÚÌî³äËùÓÐµØÖ·
void AT24C02_WriteAllBytes_eye(uint8_t value);
uint8_t AT24CXX_Check(void);  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
void AT24CXX_Init(void); //ï¿½ï¿½Ê¼ï¿½ï¿½IIC
void Heating_film_Check(void);
void prepare_data_set(void);
// ´Ó EEPROM ¶ÁÈ¡ uint32_t Êý¾Ý£¬²¢ÔÚÎ´³õÊ¼»¯Ê±Ð´Èë 0
uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr);
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value);
uint16_t EYE_AT24CXX_Read(uint16_t startAddr);
HAL_StatusTypeDef EYE_AT24CXX_Write(uint16_t WriteAddr, uint16_t value) ;

HAL_StatusTypeDef EYE_AT24CXX_WriteByte(uint16_t addr, uint8_t data);
uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out);
HAL_StatusTypeDef EYE_AT24CXX_WriteUInt16(uint16_t addr, uint16_t value);
uint16_t EYE_AT24CXX_ReadUInt16(uint16_t addr);

void AD24C01_Factory_formatted(void);
#endif