#ifndef __TMP112_H
#define __TMP112_H
#include "stm32g0xx_hal.h"

//#define TMP112_ADDR 0x93//��ַ��vcc
#define TMP112_ADDR 0x49//��ַ��GND

//#define TMP112_ADDR 0x49//��ַ��vcc
//#define TMP112_ADDR 0x48//��ַ��GND

//#define TMP112_ADDR 0x49<<1//��ַ��vcc
//#define TMP112_ADDR 0x48<<1//��ַ��GND
void TMP112_Init(void);
void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer);
//void TMP112_MultiRead(uint8_t* pBuffer);
// void TMP112_WriteByte(uint8_t WriteAddr,uint8_t WriteData);
// void TMP112_WriteWord(uint8_t WriteAddr,u16 WriteData);
float TmpRaw2Ture(void);


		 
#endif

