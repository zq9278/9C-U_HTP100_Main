
#include "main.h"
uint8_t BQ25895Reg[21];
uint8_t BQ25895TempData[1];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void)
{ 
    CHG_CE(0); // 关闭充电使能
    BQ25895_Write(0x02, 0x7C); // 设置寄存??0x02，将AUTO_DPDM_EN清零
	//BQ25895_Write(0x02,0x7d);//ADC???????
    BQ25895_Write(0x03, 0x1E); // OTG关闭，最小系统电压???置??3.5V
	BQ25895_Write(0x04, 0x20); // 设置充电电流??4096mA
    //BQ25895_Write(0x05, 0x10); // 设置充电终???电流为64mA
    BQ25895_Write(0x05, 0x14); // 设置充电终止电流为250mA
    BQ25895_Write(0x07, 0x8D); // 关闭充电定时??
    // 设置REG00，???5位为1，其他为0
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
    // 初始化状态为 false
    charging = false;
    fully_charged = false;
    working = false;
    // 根据 CHRG_STAT 更新状态
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
