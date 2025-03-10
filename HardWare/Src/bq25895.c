
#include "main.h"
uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {
    // 1. 解除芯片的出厂默认模式
    BQ25895_Write(0x00, 0x30);  // 复位所有寄存器，确保非主机模式
    // 2. 设置输入电流限制 (适用于5V/9V/12V输入, 选5A)
    BQ25895_Write(0x00, 0x3A);  // 3.25A 输入电流限制 (IINLIM=3250mA)//重发
    // 3. 设定充电电流为 3.5A
    BQ25895_Write(0x04, 0x7C);  // ICHG = 3500mA (0x7C 对应 3.5A)
    // 4. 设置充电电压为 4.2V (标准锂电池)
    BQ25895_Write(0x06, 0x96);  // VREG = 4.2V
    // 5. 设置端电流 (终止充电电流, 设为 250mA)
    BQ25895_Write(0x05, 0x07);  // ITERM = 250mA
    // 6. 使能自动充电 (DPM, BATFET 控制)
    BQ25895_Write(0x09, 0x40);  // 自动充电，省去主机控制
    // 7. 确保不会受到主机干扰
    BQ25895_Write(0x0B, 0x80);  // 禁用 OTG, 仅作为充电器
    // 8. 开启充电
    BQ25895_Write(0x03, 0x30);  // 启用自动充电
    CHG_CE(0);
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
//  case 3: // Charge Termination Done
//    fully_charged = 1;
//    charging = 0;
//    working = 0;
//    break;
  case 0: // Not Charging
    working = 1;
    charging = 0;
    fully_charged = 0;
    break;
  }
}
void bq25895_reinitialize_if_vbus_inserted(void) {
    if (is_charging_flag){
        is_charging_flag=0;
        static uint8_t last_vbus_status = 0x00;
        // 读取 VBUS 状态寄存器 0x0B
        uint8_t vbus_status ;
        BQ25895_Read(0x0B,&vbus_status);
        // 检测 VBUS 是否插入 (Bit7 = 1 表示插入)
        if ((vbus_status & 0x80) && !(last_vbus_status & 0x80)) {
            BQ25895_Init(); // 重新初始化
            LOG("充电器已插入，重新初始化 bq25895...\n");

        }
        last_vbus_status = vbus_status;
    }
}