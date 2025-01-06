
#include "main.h"


uint8_t EyeTmpRaw[2];


extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;



void TMP112_Init(void)
{ 
	
}  

void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
 { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
    ; // 绛夊緟DMA瀹屾垚
	
   // 使用 HAL_I2C_Mem_Read_DMA 进行读操作
   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
   
}  

// void TMP112_MultiRead(uint8_t* pBuffer)   
// { 	
// 	HAL_I2C_Mem_Read_DMA(&hi2c2, TMP112_ADDR, 0x00,I2C_MEMADD_SIZE_8BIT, pBuffer,0x14);
// }  
 
// void TMP112_WriteByte(uint8_t WriteAddr,uint8_t WriteData)
// {
// 	uint8_t Data[1];
// 	Data[0]=WriteData;
// 	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP112_ADDR, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 1);
// } 

// void TMP112_WriteWord(uint8_t WriteAddr,u16 WriteData)
// {
// 	uint8_t Data[2];
// 	Data[0]=WriteData;
// 	Data[1]=WriteData>>8;
// 	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP112_ADDR, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 2);
// 	delay_ms(2);
// } 
int16_t TmpData;
float TmpRaw2Ture(void)
{   TMP112_Read(0x00,EyeTmpRaw);
	TmpData=(EyeTmpRaw[0]<<8) | EyeTmpRaw[1];
		TmpData = TmpData >> 4;
	return TmpData*0.0625;
}
