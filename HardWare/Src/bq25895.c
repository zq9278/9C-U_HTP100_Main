
#include "main.h"
uint8_t BQ25895Reg[21];
uint8_t BQ25895TempData[1];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void)
{ 
    CHG_CE(0); // �رճ��ʹ��
    BQ25895_Write(0x02, 0x7C); // ���üĴ�??0x02����AUTO_DPDM_EN����
	//BQ25895_Write(0x02,0x7d);//ADC???????
    BQ25895_Write(0x03, 0x1E); // OTG�رգ���Сϵͳ��ѹ???��??3.5V
	BQ25895_Write(0x04, 0x20); // ���ó�����??4096mA
    //BQ25895_Write(0x05, 0x10); // ���ó����???����Ϊ64mA
    BQ25895_Write(0x05, 0x14); // ���ó����ֹ����Ϊ250mA
    BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ??
    // ����REG00��???5λΪ1������Ϊ0
    //BQ25895_Write(0x00, 0x20);//1.6A
    BQ25895_Write(0x00, 0x3F);//3.25A

}
void BQ25895_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
{ 	
	while(hi2c1.State != HAL_I2C_STATE_READY){;}
	HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, ReadAddr,I2C_MEMADD_SIZE_8BIT, pBuffer,1);
}  

void BQ25895_MultiRead(uint8_t* pBuffer)   
{ 
	while(hi2c1.State != HAL_I2C_STATE_READY){;}	
	HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, 0x00,I2C_MEMADD_SIZE_8BIT, pBuffer,0x14);
}  
 
void BQ25895_Write(uint8_t WriteAddr,uint8_t WriteData)
{
	while(hi2c1.State != HAL_I2C_STATE_READY){;}	
	BQ25895TempData[0]=WriteData;
	HAL_I2C_Mem_Write_DMA(&hi2c1, BQ25895Address, WriteAddr,I2C_MEMADD_SIZE_8BIT, BQ25895TempData, 1);
} 

ChargeState_t ChargeState = STATE_POWER_ON;
bool charging, working, fully_charged,low_battery, emergency_stop;
void UpdateChargeState_bq25895(void) {
    BQ25895_MultiRead(BQ25895Reg); 
    uint8_t CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;
    // ��ʼ��״̬Ϊ false
    charging = false;
    fully_charged = false;
    working = false;
    // ���� CHRG_STAT ����״̬
    switch (CHRG_STAT) {
    case 1: // Pre-charge
    case 2: // Fast Charging
        charging = true;
        break;
    case 3: // Charge Termination Done
        fully_charged = true;
        break;
    case 0: // Not Charging
        working = true;
        break;
    }
}
