
#include "main.h"
uint8_t BQ25895Reg[21];
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
    BQ25895_Read_IT(ReadAddr, pBuffer, 1);
}
void BQ25895_MultiRead(uint8_t *pBuffer) {
    BQ25895_Read_IT(0x00, pBuffer, 20);  // 读取 20 字节
}
void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
    BQ25895_Write_IT(WriteAddr, WriteData);
}
HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t temp = WriteData;
    // 获取 I2C 互斥锁，确保总线不会冲突
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空完成信号量，防止上次操作影响
        xSemaphoreTake(xI2CCompleteSem, 0);

        // 启动 I2C 写操作 (中断模式)
        status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, &temp, 1);
        if (status == HAL_OK) {
            // 等待传输完成 (带超时)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 传输成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // 释放 I2C 互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // 总线繁忙
    }
    return status;
}
HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // 获取 I2C 互斥锁，防止总线冲突
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空完成信号量，防止上次操作影响
        xSemaphoreTake(xI2CCompleteSem, 0);

        // 启动 I2C 读操作 (中断模式)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        if (status == HAL_OK) {
            // 等待传输完成 (带超时)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 传输成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // 释放 I2C 互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // 总线繁忙
    }
    return status;
}

float read_battery_voltage(uint8_t * BQ25895Reg) {
  // 合成 16 位数据
  uint16_t vbat_raw=(BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

  // 转换为电压 (mV)
  return vbat_raw * 20.0/1000;
}
uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
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