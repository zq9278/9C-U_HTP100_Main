
#include "main.h"

uint8_t BQ27441_TempData[2];

// 3300mAh 的电池参数
uint8_t NEW_DC_MSB = 0x0C; // 设计容量高字节
uint8_t NEW_DC_LSB = 0xE4; // 设计容量低字节

uint8_t NEW_DE_MSB = 0x2F; // 设计能量高字节
uint8_t NEW_DE_LSB = 0xA2; // 设计能量低字节

uint8_t NEW_TV_MSB = 0x0B; // 终止电压高字节（3000mV）
uint8_t NEW_TV_LSB = 0xB8; // 终止电压低字节（3000mV）

uint8_t NEW_TR_MSB = 0x00; // Taper Rate 高字节（150mA）
uint8_t NEW_TR_LSB = 0x16; // Taper Rate 低字节（150mA）

uint8_t OLD_DC_LSB, OLD_DC_MSB, OLD_DE_LSB, OLD_DE_MSB, OLD_TV_LSB, OLD_TV_MSB,
    OLD_TR_LSB, OLD_TR_MSB;
uint8_t CHKSUM;
uint8_t BQ27441_FLAG_LSB, BQ27441_CONTROL_STATUS_MSB;

BQ27441_typedef BQ27441;

extern I2C_HandleTypeDef hi2c1;

//uint8_t BQ27441_Init(void) {
//  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM;
//  uint8_t tempee;
//  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
// if (tempee != 0x55) {
//   // 进入命令模式，通过向 0x00 寄存器写入 0x8000 激活 BQ27441
//   BQ27441_WriteWord(0x00, 0x8000);
//   BQ27441_WriteWord(0x00, 0x8000);
//
//   // 将 0x0013 写入 0x00，表示发送 UNSEAL 命令，解锁访问寄存器
//   BQ27441_WriteWord(0x00, 0x0013);
//
//   // 等待 2 秒，确保解锁完成
//   HAL_Delay(200);
//
//   // 读取 0x06 寄存器，检查 FLAG 标志位
//   BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//   HAL_Delay(1);
//
//   // 等待 I?C 总线空闲
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // 如果 FLAG 标志未正确设置，返回失败
//   if ((BQ27441_FLAG_LSB & 0x10) != 0x10) {
//     return 0; // 解锁失败
//   }
//
//   // 配置数据类和地址指针
//   BQ27441_WriteByte(0x61, 0x00); // 写入 0x61 寄存器，设置控制命令
//   BQ27441_WriteByte(0x3E, 0x52); // 写入 0x3E 数据类地址为 0x52
//   BQ27441_WriteByte(0x3F, 0x00); // 设置子地址为 0x00
//   HAL_Delay(1);
//
//   // 读取旧的校验和
//   BQ27441_Read(0x60, &OLD_CHKSUM);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // 计算临时校验和
//   TMP_CHKSUM = 0xFF - OLD_CHKSUM;
//
//   // 读取旧的电池参数
//   BQ27441_Read(0x4A, &OLD_DC_MSB); // 设计容量高字节
//   BQ27441_Read(0x4B, &OLD_DC_LSB); // 设计容量低字节
//   BQ27441_Read(0x4C, &OLD_DE_MSB); // 设计能量高字节
//   BQ27441_Read(0x4D, &OLD_DE_LSB); // 设计能量低字节
//   BQ27441_Read(0x50, &OLD_TV_MSB); // 终止电压高字节
//   BQ27441_Read(0x51, &OLD_TV_LSB); // 终止电压低字节
//   BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate 高字节
//   BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate 低字节
//
//   // 再次等待 I?C 总线空闲
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // 检查旧参数是否需要更新
//   if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
//     // 更新校验和以减去旧值
//     TMP_CHKSUM = TMP_CHKSUM - OLD_DC_LSB - OLD_DC_MSB - OLD_DE_LSB -
//                  OLD_DE_MSB - OLD_TV_LSB - OLD_TV_MSB - OLD_TR_LSB -
//                  OLD_TR_MSB;
//
//     HAL_Delay(1);
//
//     // 写入新的电池参数
//     BQ27441_WriteByte(0x4A, NEW_DC_MSB); // 设计容量高字节
//     BQ27441_WriteByte(0x4B, NEW_DC_LSB); // 设计容量低字节
//     BQ27441_WriteByte(0x4C, NEW_DE_MSB); // 设计能量高字节
//     BQ27441_WriteByte(0x4D, NEW_DE_LSB); // 设计能量低字节
//     BQ27441_WriteByte(0x50, NEW_TV_MSB); // 终止电压高字节
//     BQ27441_WriteByte(0x51, NEW_TV_LSB); // 终止电压低字节
//     BQ27441_WriteByte(0x5B, NEW_TR_MSB); // Taper Rate 高字节
//     BQ27441_WriteByte(0x5C, NEW_TR_LSB); // Taper Rate 低字节
//
//     // 更新校验和以加上新值
//     TMP_CHKSUM = TMP_CHKSUM + NEW_DC_LSB + NEW_DC_MSB + NEW_DE_LSB +
//                  NEW_DE_MSB + NEW_TV_LSB + NEW_TV_MSB + NEW_TR_LSB +
//                  NEW_TR_MSB;
//
//     // 计算新的校验和并写入
//     NEW_CHKSUM = 0xFF - TMP_CHKSUM;
//     BQ27441_WriteByte(0x60, NEW_CHKSUM);
//     HAL_Delay(1000);
//
//     // 再次设置数据类和子地址
//     BQ27441_WriteByte(0x3E, 0x52);
//     BQ27441_WriteByte(0x3F, 0x00);
//     HAL_Delay(1);
//
//     // 读取并验证新的校验和
//     BQ27441_Read(0x60, &CHKSUM);
//     while (hi2c1.State != HAL_I2C_STATE_READY) {
//       ;
//     }
//
//     // 校验和验证成功
//     if (CHKSUM == NEW_CHKSUM) {
//       // 等待 DF 执行完成
//       do {
//         BQ27441_WriteWord(0x00, 0x0042);
//         HAL_Delay(1);
//         BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//         HAL_Delay(1);
//       } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//       // 发送 CONTROL 0x0020 命令
//       BQ27441_WriteWord(0x00, 0x0020);
//
//       // 更新 EEPROM 配置
//       AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//
//       return 1; // 操作成功
//     } else {
//       return 0; // 校验和错误
//     }
//   } else {
//     // 如果参数未更新，直接执行 DF 完成流程
//     do {
//       BQ27441_WriteWord(0x00, 0x0042);
//       HAL_Delay(1);
//       BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//       HAL_Delay(1);
//     } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//     // 发送 CONTROL 0x0020 命令
//     BQ27441_WriteWord(0x00, 0x0020);
//
//     // 更新 EEPROM 配置
//     AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//
//     return 1; // 操作成功
//   }
// }
//}


uint8_t BQ27441_Init(void) {
  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM; // 校验和变量
  uint8_t tempee;

  // 从EEPROM读取配置标志，用于判断是否需要重新初始化
  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
  printf("[LOG] Read EEPROM flag: 0x%02X\n", tempee);

  //if (tempee != 0x55) { // 如果标志未设置，开始初始化
  if (1) { // 如果标志未设置，开始初始化
    printf("[LOG] Starting initialization...\n");

    // 激活BQ27441，进入命令模式
    BQ27441_WriteWord(0x00, 0x8000);
    BQ27441_WriteWord(0x00, 0x8000);
    HAL_Delay(1000); // 确保命令模式激活(必须要）
    // 发送UNSEAL解锁命令以访问寄存器
    BQ27441_WriteWord(0x00, 0x0013);
    // 读取FLAG寄存器，确认解锁状态
    for (int i = 0; i < 20; ++i) {
      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
      HAL_Delay(1); // 等待解锁完成
    }

    printf("[LOG] FLAG register: 0x%02X\n", BQ27441_FLAG_LSB);
    if ((BQ27441_FLAG_LSB & 0x10) == 0) { // 检查FLAG标志
      printf("[ERROR] Unlock failed.\n");
      return 0; // 如果解锁失败，返回失败状态
    }

    // 设置数据类为0x52（Design）以读取和写入电池参数
    //BQ27441_WriteByte(0x61, 0x00); // 数据类地址
    BQ27441_WriteByte(0x3E, 0x52); // 数据类地址
    BQ27441_WriteByte(0x3F, 0x00); // 子地址
    HAL_Delay(5); // 等待配置完成
    printf("[LOG] Set data class to 0x52.\n");

    // 读取旧的校验和，用于更新参数后重新计算
    BQ27441_Read(0x60, &OLD_CHKSUM);
    printf("[LOG] Old checksum: 0x%02X\n", OLD_CHKSUM);

    // 读取旧的电池参数
    BQ27441_Read(0x4A, &OLD_DC_MSB); // 设计容量高字节
    BQ27441_Read(0x4B, &OLD_DC_LSB); // 设计容量低字节
    BQ27441_Read(0x4C, &OLD_DE_MSB); // 设计能量高字节
    BQ27441_Read(0x4D, &OLD_DE_LSB); // 设计能量低字节
    BQ27441_Read(0x50, &OLD_TV_MSB); // 终止电压高字节
    BQ27441_Read(0x51, &OLD_TV_LSB); // 终止电压低字节
    BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate高字节
    BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate低字节
    printf("[LOG] Old parameters: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
           OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB, OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);

    // 如果旧参数与新参数不同，则需要更新
    if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
      printf("[LOG] Parameters mismatch, updating...\n");
      TMP_CHKSUM = 0xFF - OLD_CHKSUM; // 计算临时校验和，移除旧值
      TMP_CHKSUM -= OLD_DC_MSB + OLD_DC_LSB + OLD_DE_MSB + OLD_DE_LSB;
      TMP_CHKSUM -= OLD_TV_MSB + OLD_TV_LSB + OLD_TR_MSB + OLD_TR_LSB;

      // 写入新的电池参数
      BQ27441_WriteByte(0x4A, NEW_DC_MSB); // 写入设计容量高字节
      BQ27441_WriteByte(0x4B, NEW_DC_LSB); // 写入设计容量低字节
      BQ27441_WriteByte(0x4C, NEW_DE_MSB); // 写入设计能量高字节
      BQ27441_WriteByte(0x4D, NEW_DE_LSB); // 写入设计能量低字节
      BQ27441_WriteByte(0x50, NEW_TV_MSB); // 写入终止电压高字节
      BQ27441_WriteByte(0x51, NEW_TV_LSB); // 写入终止电压低字节
      BQ27441_WriteByte(0x5B, NEW_TR_MSB); // 写入Taper Rate高字节
      BQ27441_WriteByte(0x5C, NEW_TR_LSB); // 写入Taper Rate低字节

      TMP_CHKSUM += NEW_DC_MSB + NEW_DC_LSB + NEW_DE_MSB + NEW_DE_LSB;
      TMP_CHKSUM += NEW_TV_MSB + NEW_TV_LSB + NEW_TR_MSB + NEW_TR_LSB;
      NEW_CHKSUM = 0xFF - TMP_CHKSUM; // 计算新的校验和

      // 写入新的校验和
      BQ27441_WriteByte(0x60, NEW_CHKSUM);
      HAL_Delay(100);
      printf("[LOG] New checksum written: 0x%02X\n", NEW_CHKSUM);

      // 验证校验和是否正确
      BQ27441_Read(0x60, &CHKSUM);
      if (CHKSUM != NEW_CHKSUM) {
        printf("[ERROR] Checksum verification failed.\n");
        return 0; // 如果校验和错误，返回失败
      }
    }

    // 等待DF执行完成
    do {
      BQ27441_WriteWord(0x00, 0x0042);
      HAL_Delay(5);
      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
    } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);

    // 发送Control命令0x0020，保存更改
    BQ27441_WriteWord(0x00, 0x0020);
    printf("[LOG] DF execution completed.\n");

    // 更新EEPROM标志，表示初始化完成
    AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
    printf("[LOG] Initialization completed successfully.\n");

    return 1; // 初始化成功
  }

  printf("[LOG] Initialization skipped, already configured.\n");
  return 1; // 如果已初始化，则直接返回成功
}

void BQ27441_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}

// void BQ27441_MultiRead(BQ27441_typedef *BQ_State) {
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x04, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->Voltage), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x02, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->Temperature), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x10, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->AvgCurrent), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x1C, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->SOC), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x0E, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->FullChargeCapacity), 2);
// }

// 全局变量，用于标记 DMA 传输状态
extern volatile bool dma_transfer_complete;
// DMA 读取函数
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ_State) {
  HAL_StatusTypeDef status;

  // Voltage
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ; // 等待 I2C 状态为空闲
  }
  status =
      HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x04, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&(BQ_State->Voltage), 2);
  if (status != HAL_OK) {
    return status;
  }
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ; // 等待传输完成
  }

  // Temperature
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  status =
      HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x02, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&(BQ_State->Temperature), 2);
  if (status != HAL_OK) {
    return status;
  }
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }

  // AvgCurrent
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  status =
      HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x10, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&(BQ_State->AvgCurrent), 2);
  if (status != HAL_OK) {
    return status;
  }
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }

  // SOC
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  status =
      HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x1C, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&(BQ_State->SOC), 2);
  if (status != HAL_OK) {
    return status;
  }
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }

  // FullChargeCapacity
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  status =
      HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x0E, I2C_MEMADD_SIZE_8BIT,
                           (uint8_t *)&(BQ_State->FullChargeCapacity), 2);
  if (status != HAL_OK) {
    return status;
  }
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }

  return HAL_OK; // 所有读取操作成功
}
void BQ27441_WriteByte(uint8_t WriteAddr, uint8_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ27441Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ27441_TempData, 1);
}

void BQ27441_WriteWord(uint8_t WriteAddr, uint16_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  BQ27441_TempData[1] = WriteData >> 8;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ27441Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ27441_TempData, 2);
}

extern uint8_t low_battery;
extern uint8_t flag_200ms;
extern osMessageQueueId_t Battery_DATAHandle;
float batp;
void battery_status_update_bq27441(void) {
  BQ27441_MultiRead_DMA(&BQ27441);
  low_battery = (BQ27441.SOC < 20) && (BQ27441.SOC != 0);
    float battery = (float)BQ27441.SOC;
    if(flag_200ms){
        //flag_200ms=0;
        xQueueSend(Battery_DATAHandle, &battery, 0); // 将数据发送到队列
    }
//   float bat=(4199-(float)BQ27441.Voltage)/(0.78);//电量百分比去掉%
//    batp=(2000-bat)/20;
//   xQueueSend(Battery_DATAHandle, &batp, 0); // 将数据发送到队列
    if (BQ27441.Voltage <= 3000) {
      BQ25895_Write(0x09, 0x64); // 电压低，关断保护
    }
  }