
#include "main.h"
uint8_t BQ25895Reg[21];
uint8_t BQ25895TempData[1];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {

  CHG_CE(0);                 // 关闭充电使能
  //BQ25895_Write(0x02, 0x7C); // 设置寄存??0x02，将AUTO_DPDM_EN清零
  BQ25895_Write(0x02, 0xFC); // 开启ADC
  BQ25895_Write(0x03, 0x1E); // OTG关闭，最小系统电压???置??3.5V
  BQ25895_Write(0x04, 0x20); // 设置充电电流??4096mA
  // BQ25895_Write(0x05, 0x10); // 设置充电终???电流为64mA
  BQ25895_Write(0x05, 0x13); // 设置充电终止电流为150mA
  BQ25895_Write(0x07, 0x8D); // 关闭充电定时??
  BQ25895_Write(0x00, 0x3F); // 3.25A
}

void BQ25895_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}
// 全局变量，用于标记 DMA 传输状态
extern volatile bool dma_transfer_complete;
void BQ25895_MultiRead(uint8_t *pBuffer) {

  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, 0x00, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 20);
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ; // 等待传输完成
  }
}

void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ25895TempData[0] = WriteData;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ25895Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ25895TempData, 1);
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ; // 等待传输完成
  }
}




#define VBAT_REG_LSB 0x0E
#define VBAT_REG_MSB 0x0F

float read_battery_voltage(uint8_t * BQ25895Reg) {

  // 合成 16 位数据
  uint16_t vbat_raw=(BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

  // 转换为电压 (mV)
  return vbat_raw * 20.0/1000;
}
uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
extern uint8_t Flag_400ms;
void UpdateChargeState_bq25895(void) {
  BQ25895_MultiRead(BQ25895Reg);
//  float battery=read_battery_voltage(BQ25895Reg);
//  vTaskDelay(100);
//  printf("电压%f;",battery);
//  float bat=(4.199-(float)battery)/(0.00078);//电量百分比去掉%
//  printf("电量%f;",bat);
//  printf("百分比%f\n",bat/42);
   CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;
  // 初始化状态为 false

  // 根据 CHRG_STAT 更新状态
  switch (CHRG_STAT) {
  case 1: // Pre-charge
  case 2: // Fast Charging
    if(fully_charged==0){// 即将充满时刻，会在2和3状态来回切换
      charging = 1;
      fully_charged = 0;
      working = 0;
    }
    break;
  case 3: // Charge Termination Done
    fully_charged = 1;
    charging = 0;
    working = 0;
    break;
  case 0: // Not Charging
    working = 1;
    charging = 0;
    fully_charged = 0;
    break;
  }
}