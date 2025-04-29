
#include "main.h"
uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {
//    BQ25895_Write(0x00, 0x30);  // 复位所有寄存器
//    HAL_Delay(1000);  // 确保芯片完成复位
//
//    // 读取寄存器，确保复位完成
//    uint8_t check_reset;
//    BQ25895_Read(0x00, &check_reset);
//    if (check_reset != 0x30) {
//        LOG("复位失败，可能未写入成功\n");
//        return;
//    }
//
//    // 重新写入寄存器
//    //BQ25895_Write(0x00, 0x3A);  // 3.25A 输入电流限制
//    BQ25895_Write(0x00, 0x28);  // 降低输入电流限制到 2A
//    BQ25895_Write(0x04, 0x28);  // 设置充电电流为 2A
//    BQ25895_Write(0x06, 0x96);  // 充电电压 4.2V
//    BQ25895_Write(0x05, 0x07);  // 终止电流 250mA
//    BQ25895_Write(0x09, 0x40);  // 自动充电
//    BQ25895_Write(0x0B, 0x80);  // 禁用 OTG
//    BQ25895_Write(0x07, 0x8D); // 关闭充电定时******重要********
//    BQ25895_Write(0x03, 0x30);  // 重新启用自动充电
//    CHG_CE(0);
    CHG_CE(0);                 // 关闭充电使能
    //BQ25895_Write(0x02, 0x7C); // 设置寄存??0x02，将AUTO_DPDM_EN清零
    BQ25895_Write(0x02, 0xFC); // 开启ADC
    BQ25895_Write(0x03, 0x1E); // OTG关闭，最小系统电压???置??3.5V
    BQ25895_Write(0x04, 0x20); // 设置充电电流2048mA
    // BQ25895_Write(0x05, 0x10); // 设置充电终???电流为64mA
    BQ25895_Write(0x05, 0x11); // 设置充电终止电流为64mA
    BQ25895_Write(0x07, 0x8D); // 关闭充电定时??
    BQ25895_Write(0x06, 0x94);  // 充电电压 4.2V
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
//void bq25895_reinitialize_if_vbus_inserted(void) {
//    if (is_charging_flag){
//        is_charging_flag=0;
//        static uint8_t last_vbus_status = 0x00;
//        // 读取 VBUS 状态寄存器 0x0B
//        uint8_t vbus_status ;
//        BQ25895_Read(0x0B,&vbus_status);
//        // 检测 VBUS 是否插入 (Bit7 = 1 表示插入)
//        if (((vbus_status & 0x80) || (vbus_status == 0x16)) && !(last_vbus_status & 0x80)) {
//            BQ25895_Init(); // 重新初始化
//            LOG("充电器已插入，重新初始化 bq25895...\n");
//
//        }
//        last_vbus_status = vbus_status;
//    }
//}
void bq25895_reinitialize_if_vbus_inserted(void) {
    static uint8_t last_vbus_status = 0x00;  // 存储上一次的 VBUS 状态
    uint8_t vbus_status;

    BQ25895_Read(0x0B, &vbus_status);

    // 检测 VBUS 插入 (仅当状态从未插入 -> 插入时执行)
    if (((vbus_status & 0x80) || (vbus_status == 0x16)) &&
        !((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOG("充电器已插入，重新初始化 bq25895...\n");
        BQ25895_Init();
    }

    // 检测 VBUS 拔出 (仅当状态从插入 -> 拔出时执行)
    if (!(vbus_status & 0x80) && !(vbus_status == 0x16) &&
        ((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOG("充电器已拔出，清除初始化标记...\n");
    }

    last_vbus_status = vbus_status;  // 更新状态
}


//    uint8_t reg00, reg0D;
//    BQ25895_Read(0x00, &reg00);
//    BQ25895_Read(0x0D, &reg0D);
//    LOG("IINLIM (输入电流): 0x%02X, VINDPM (输入电压): 0x%02X\n", reg00, reg0D);
//    uint8_t reg0B, reg0C;
//    BQ25895_Read(0x0B, &reg0B);
//    BQ25895_Read(0x0C, &reg0C);
//    LOG("VBUS状态: 0x%02X, 充电状态: 0x%02X\n", reg0B, reg0C);
//    uint8_t reg02, reg0E;
//    BQ25895_Read(0x02, &reg02);
//    BQ25895_Read(0x0E, &reg0E);
//    LOG("温度调节: 0x%02X, 热保护状态: 0x%02X\n", reg02, reg0E);
//    uint8_t reg03;
//    BQ25895_Read(0x03, &reg03);
//    LOG("充电控制: 0x%02X\n", reg03);
//    uint8_t reg11;
//    BQ25895_Read(0x11, &reg11);
//    LOG("VBUS 电压: 0x%02X\n", reg11);
//    uint8_t reg09, reg0Ba;
//    BQ25895_Read(0x03, &reg09);
//    BQ25895_Read(0x0B, &reg0Ba);
//
//    if (!(reg09 & 0x30)) {  // 如果充电被禁用
//        LOG("充电被意外关闭，重新开启充电...\n");
//        BQ25895_Write(0x03, 0x30);  // 重新启动充电
//    }
//
//    if (!(reg0Ba & 0x80)) {  // 如果 VBUS 掉线
//        LOG("VBUS 断开，等待重新插入...\n");
//    }
//}
