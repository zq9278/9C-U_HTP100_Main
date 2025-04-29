
#include "main.h"
#define LOW_BATTERY_SOC 3200
#define WORKING_BATTERY_SOC    3300  // 提醒用户电压
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
volatile bool i2c_transfer_success = false; // 传输结果状态
extern I2C_HandleTypeDef hi2c1;
uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM;
uint8_t tempee;
uint8_t BQ27441_Init(void) {

// 读取 EEPROM 配置标志
    tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
    LOG("读取 EEPROM 配置标志: 0x%02X\n", tempee);

// if (tempee != 0x55) {
    if (1) {
        LOG("开始 BQ27441 初始化...\n");

        // 进入命令模式，通过向 0x00 寄存器写入 0x8000 激活 BQ27441
        BQ27441_WriteWord(0x00, 0x8000);
        BQ27441_WriteWord(0x00, 0x8000);
        LOG("激活 BQ27441，进入命令模式\n");

        // 发送 UNSEAL 解锁命令
        BQ27441_WriteWord(0x00, 0x0013);
        LOG("发送 UNSEAL 命令\n");

        HAL_Delay(2000); // 等待解锁完成

        // 读取 0x06 FLAG 寄存器，检查标志位
        BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
        HAL_Delay(1);
        LOG("FLAG 寄存器: 0x%02X\n", BQ27441_FLAG_LSB);

        // 等待 I2C 总线空闲
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }

        // 如果 FLAG 标志未正确设置，返回失败
        if ((BQ27441_FLAG_LSB & 0x10) != 0x10) {
            LOG("解锁失败！\n");
            //return 0;
        }

        // 选择 Data Memory 类
        BQ27441_WriteByte(0x3E, 0x52); // 0x52 = Design Data Class
        BQ27441_WriteByte(0x3F, 0x00); // 偏移量 = 0x00
        HAL_Delay(1);
        LOG("设置数据类: 0x52, 偏移量: 0x00\n");

        // 读取旧的校验和
        BQ27441_Read(0x60, &OLD_CHKSUM);
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }
        LOG("读取到的旧校验和: 0x%02X\n", OLD_CHKSUM);

        // 计算临时校验和
        TMP_CHKSUM = 0xFF - OLD_CHKSUM;

        // 读取旧的电池参数
        BQ27441_Read(0x4A, &OLD_DC_MSB); // 设计容量高字节
        BQ27441_Read(0x4B, &OLD_DC_LSB); // 设计容量低字节
        BQ27441_Read(0x4C, &OLD_DE_MSB); // 设计能量高字节
        BQ27441_Read(0x4D, &OLD_DE_LSB); // 设计能量低字节
        BQ27441_Read(0x50, &OLD_TV_MSB); // 终止电压高字节
        BQ27441_Read(0x51, &OLD_TV_LSB); // 终止电压低字节
        BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate 高字节
        BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate 低字节

        // 再次等待 I2C 总线空闲
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }

        LOG("读取到的旧参数: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
            OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB,
            OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);

        // 检查旧参数是否需要更新
        if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
            LOG("参数不同，开始更新...\n");

            // 更新校验和以减去旧值
            TMP_CHKSUM -= OLD_DC_LSB + OLD_DC_MSB + OLD_DE_LSB + OLD_DE_MSB;
            TMP_CHKSUM -= OLD_TV_LSB + OLD_TV_MSB + OLD_TR_LSB + OLD_TR_MSB;

            HAL_Delay(1);

            // 写入新的电池参数
            BQ27441_WriteByte(0x4A, NEW_DC_MSB);
            BQ27441_WriteByte(0x4B, NEW_DC_LSB);
            BQ27441_WriteByte(0x4C, NEW_DE_MSB);
            BQ27441_WriteByte(0x4D, NEW_DE_LSB);
            BQ27441_WriteByte(0x50, NEW_TV_MSB);
            BQ27441_WriteByte(0x51, NEW_TV_LSB);
            BQ27441_WriteByte(0x5B, NEW_TR_MSB);
            BQ27441_WriteByte(0x5C, NEW_TR_LSB);

            // 更新校验和以加上新值
            TMP_CHKSUM += NEW_DC_LSB + NEW_DC_MSB + NEW_DE_LSB + NEW_DE_MSB;
            TMP_CHKSUM += NEW_TV_LSB + NEW_TV_MSB + NEW_TR_LSB + NEW_TR_MSB;

            // 计算新的校验和并写入
            NEW_CHKSUM = 0xFF - TMP_CHKSUM;
            BQ27441_WriteByte(0x60, NEW_CHKSUM);
            HAL_Delay(1000);

            LOG("新校验和计算完成: 0x%02X\n", NEW_CHKSUM);

            // 读取并验证新的校验和
            BQ27441_Read(0x60, &CHKSUM);
            if (CHKSUM != NEW_CHKSUM) {
                LOG("校验和验证失败！实际: 0x%02X, 期望: 0x%02X\n", CHKSUM, NEW_CHKSUM);
                return 0;
            }

            LOG("校验和验证成功！\n");

            // 等待 DF 执行完成
            do {
                BQ27441_WriteWord(0x00, 0x0042);
                HAL_Delay(1);
                BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
                HAL_Delay(1);
            } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);

            // 发送 CONTROL 0x0020 命令
            BQ27441_WriteWord(0x00, 0x0020);

            // 更新 EEPROM 配置
            AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);

            LOG("设计容量更新完成！\n");

            return 1; // 操作成功
        } else {
            LOG("参数未变更，无需更新。\n");
            return 1;
        }
    }
}

    void I2C_Semaphore_Init(void) {
    xI2CMutex = xSemaphoreCreateMutex();
    xI2CCompleteSem = xSemaphoreCreateBinary();

    if (xI2CMutex == NULL || xI2CCompleteSem == NULL) {
        printf("I2C信号量创建失败!\r\n");
        Error_Handler();
    }
}

//uint8_t BQ27441_Init(void) {
//  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM; // 校验和变量
//  uint8_t tempee;
//
//  // 从EEPROM读取配置标志，用于判断是否需要重新初始化
//  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
//  printf("[LOG] Read EEPROM flag: 0x%02X\n", tempee);
//
//  //if (tempee != 0x55) { // 如果标志未设置，开始初始化
//  if (1) { // 如果标志未设置，开始初始化
//    printf("[LOG] Starting initialization...\n");
//
//    // 激活BQ27441，进入命令模式
////    BQ27441_WriteWord(0x00, 0x8000);
////    BQ27441_WriteWord(0x00, 0x8000);
//
//    HAL_Delay(1000); // 确保命令模式激活(必须要）
//    // 发送UNSEAL解锁命令以访问寄存器
//    BQ27441_WriteWord(0x00, 0x0013);
//    // 读取FLAG寄存器，确认解锁状态
//    for (int i = 0; i < 20; ++i) {
//      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//      HAL_Delay(1); // 等待解锁完成
//    }
//
//    printf("[LOG] FLAG register: 0x%02X\n", BQ27441_FLAG_LSB);
//    if ((BQ27441_FLAG_LSB & 0x10) == 0) { // 检查FLAG标志
//      printf("[ERROR] Unlock failed.\n");
//      return 0; // 如果解锁失败，返回失败状态
//    }
//
//    // 设置数据类为0x52（Design）以读取和写入电池参数
//    //BQ27441_WriteByte(0x61, 0x00); // 数据类地址
//    BQ27441_WriteByte(0x3E, 0x52); // 数据类地址
//    BQ27441_WriteByte(0x3F, 0x00); // 子地址
//    HAL_Delay(5); // 等待配置完成
//    printf("[LOG] Set data class to 0x52.\n");
//
//    // 读取旧的校验和，用于更新参数后重新计算
//    BQ27441_Read(0x60, &OLD_CHKSUM);
//    printf("[LOG] Old checksum: 0x%02X\n", OLD_CHKSUM);
//
//    // 读取旧的电池参数
//    BQ27441_Read(0x4A, &OLD_DC_MSB); // 设计容量高字节
//    BQ27441_Read(0x4B, &OLD_DC_LSB); // 设计容量低字节
//    BQ27441_Read(0x4C, &OLD_DE_MSB); // 设计能量高字节
//    BQ27441_Read(0x4D, &OLD_DE_LSB); // 设计能量低字节
//    BQ27441_Read(0x50, &OLD_TV_MSB); // 终止电压高字节
//    BQ27441_Read(0x51, &OLD_TV_LSB); // 终止电压低字节
//    BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate高字节
//    BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate低字节
//    printf("[LOG] Old parameters: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
//           OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB, OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);
//
//    // 如果旧参数与新参数不同，则需要更新
//    if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
//      printf("[LOG] Parameters mismatch, updating...\n");
//      TMP_CHKSUM = 0xFF - OLD_CHKSUM; // 计算临时校验和，移除旧值
//      TMP_CHKSUM -= OLD_DC_MSB + OLD_DC_LSB + OLD_DE_MSB + OLD_DE_LSB;
//      TMP_CHKSUM -= OLD_TV_MSB + OLD_TV_LSB + OLD_TR_MSB + OLD_TR_LSB;
//
//      // 写入新的电池参数
//      BQ27441_WriteByte(0x4A, NEW_DC_MSB); // 写入设计容量高字节
//      BQ27441_WriteByte(0x4B, NEW_DC_LSB); // 写入设计容量低字节
//      BQ27441_WriteByte(0x4C, NEW_DE_MSB); // 写入设计能量高字节
//      BQ27441_WriteByte(0x4D, NEW_DE_LSB); // 写入设计能量低字节
//      BQ27441_WriteByte(0x50, NEW_TV_MSB); // 写入终止电压高字节
//      BQ27441_WriteByte(0x51, NEW_TV_LSB); // 写入终止电压低字节
//      BQ27441_WriteByte(0x5B, NEW_TR_MSB); // 写入Taper Rate高字节
//      BQ27441_WriteByte(0x5C, NEW_TR_LSB); // 写入Taper Rate低字节
//
//      TMP_CHKSUM += NEW_DC_MSB + NEW_DC_LSB + NEW_DE_MSB + NEW_DE_LSB;
//      TMP_CHKSUM += NEW_TV_MSB + NEW_TV_LSB + NEW_TR_MSB + NEW_TR_LSB;
//      NEW_CHKSUM = 0xFF - TMP_CHKSUM; // 计算新的校验和
//
//      // 写入新的校验和
//      BQ27441_WriteByte(0x60, NEW_CHKSUM);
//      HAL_Delay(100);
//      printf("[LOG] New checksum written: 0x%02X\n", NEW_CHKSUM);
//
//      // 验证校验和是否正确
//      BQ27441_Read(0x60, &CHKSUM);
//      if (CHKSUM != NEW_CHKSUM) {
//        printf("[ERROR] Checksum verification failed.\n");
//        return 0; // 如果校验和错误，返回失败
//      }
//    }
//
//    // 等待DF执行完成
//    do {
//      BQ27441_WriteWord(0x00, 0x0042);
//      HAL_Delay(5);
//      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//    } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//    // 发送Control命令0x0020，保存更改
//    BQ27441_WriteWord(0x00, 0x0020);
//    printf("[LOG] DF execution completed.\n");
//
//    // 更新EEPROM标志，表示初始化完成
//    AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//    printf("[LOG] Initialization completed successfully.\n");
//
//    return 1; // 初始化成功
//  }
//
//  printf("[LOG] Initialization skipped, already configured.\n");
//  return 1; // 如果已初始化，则直接返回成功
//}

void BQ27441_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}


//HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
//    HAL_StatusTypeDef status = HAL_ERROR;
//
//    // 获取 I2C 互斥锁，防止多个任务同时访问 I2C
//    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
//        // 先清空信号量，防止上次操作影响
//        xSemaphoreTake(xI2CCompleteSem, 0);
//
//        // 启动 I2C 读操作 (使用中断模式)
//        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
//
//        if (status == HAL_OK) {
//
//            // 等待 I2C 操作完成信号量 (带超时)
//            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
//                status = HAL_OK;  // 操作成功
//                LOG("I2C 读取成功: 寄存器=0x%02X, 数据=[0x%02X]\n", regAddr, *pBuffer);
//            } else {
//                LOG("I2C 读取超时！尝试重新初始化 I2C\n");
//                HAL_I2C_DeInit(&hi2c1);
//                HAL_I2C_Init(&hi2c1);
//                status = HAL_TIMEOUT;
//            }
//        } else {
//            LOG("I2C 读取失败: HAL 状态=%d\n", status);
//        }
//
//        // 释放 I2C 互斥锁
//        xSemaphoreGive(xI2CMutex);
//    } else {
//        status = HAL_BUSY;  // 获取锁失败，I2C 总线繁忙
//        LOG("I2C 互斥锁获取失败，I2C 总线繁忙\n");
//    }
//
//    return status;
//}

HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // 获取互斥锁，防止多个任务同时访问I2C
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空信号量，防止上次操作影响
        xSemaphoreTake(xI2CCompleteSem, 0);
        // 启动 I2C 读操作 (使用中断模式)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        //status = HAL_I2C_Mem_Read(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size,0xffff);
        if (status == HAL_OK) {
            // 等待 I2C 操作完成信号量 (带超时)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 操作成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }
        // 释放I2C互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // 获取锁失败，I2C总线繁忙
    }
    return status;
}
//HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
//    HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
//}
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空信号量
        xSemaphoreTake(xI2CCompleteSem, 0);

        // 启动 I2C 写操作 (使用中断模式)
        //status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size);
        status = HAL_I2C_Mem_Write(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size,0xffff);
        if (status == HAL_OK) {
            // 等待 I2C 操作完成信号量
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 操作成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // 释放I2C互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }

    return status;
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
    BQ27441_Read_IT(0x04,(uint8_t *)&(BQ_State->Voltage), 2);
    BQ27441_Read_IT(0x02,(uint8_t *)&(BQ_State->Temperature), 2);
    BQ27441_Read_IT(0x10,(uint8_t *)&(BQ_State->AvgCurrent), 2);
    BQ27441_Read_IT(0x1C,(uint8_t *)&(BQ_State->SOC), 2);
    BQ27441_Read_IT(0x0E,(uint8_t *)&(BQ_State->FullChargeCapacity), 2);


//    bq_read((uint8_t *)&(BQ_State->Voltage), 0x04,2);
//    bq_read((uint8_t *)&(BQ_State->Temperature),0x02, 2);
//    bq_read((uint8_t *)&(BQ_State->AvgCurrent),0x10, 2);
//    bq_read((uint8_t *)&(BQ_State->SOC), 0x1C,2);
//    bq_read((uint8_t *)&(BQ_State->FullChargeCapacity),0x0E, 2);
  return HAL_OK; // 所有读取操作成功
}
void BQ27441_WriteByte(uint8_t WriteAddr, uint8_t WriteData) {
  BQ27441_TempData[0] = WriteData;
    BQ27441_Write_IT( WriteAddr, BQ27441_TempData, 1);
}
void BQ27441_WriteWord(uint8_t WriteAddr, uint16_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  BQ27441_TempData[1] = WriteData >> 8;
  BQ27441_Write_IT(WriteAddr,BQ27441_TempData, 2);
}

extern uint8_t low_battery;
extern uint8_t battery_flag_400ms;
//extern osMessageQueueId_t Battery_DATAHandle;
float batp;
static float lastBatteryValue = 0;// 声明一个变量用于存储上一次的值



typedef enum {
    BATTERY_NORMAL = 0,
    BATTERY_CHECK,
    BATTERY_CONFIRM_SHUTDOWN,
    BATTERY_SHUTDOWN,
    BATTERY_WORKING_LOW
} BatteryState_t;

typedef struct {
    BatteryState_t state;
    uint8_t lowVoltageCounter;
    TickType_t lastCheckTick;
} BatteryMonitor_t;

static BatteryMonitor_t batteryMonitor = {
        .state = BATTERY_NORMAL,
        .lowVoltageCounter = 0,
        .lastCheckTick = 0,
};

// 你已有的函数


void BatteryMonitor_Run(void)
{
    // 每次进入都拿一次时间戳
    TickType_t nowTick = xTaskGetTickCount();
    const TickType_t checkInterval = pdMS_TO_TICKS(500); // 每500ms检测一次

    switch (batteryMonitor.state)
    {
        case BATTERY_NORMAL:
            if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
            {
                LOG("[Battery] [NORMAL] 检测到严重低电压 %d mV，进入检测...\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_CHECK;
                batteryMonitor.lowVoltageCounter = 0;
                batteryMonitor.lastCheckTick = nowTick;
            }
            else if (BQ27441.Voltage <= WORKING_BATTERY_SOC && BQ27441.Voltage != 0)
            {
                // 电压低于工作电压线，提前UART发送提示
                LOG("[Battery] [NORMAL] 电池电量偏低（%d mV），提醒用户。\n", BQ27441.Voltage);
                // 这里可以通过UART发出去
                // uart_printf("Warning: Battery voltage low (%d mV)\r\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_WORKING_LOW;
            }
            else
            {
                // 电压正常，不动
            }
            break;

        case BATTERY_WORKING_LOW:
            // 在工作低电状态下继续监测
            if (nowTick - batteryMonitor.lastCheckTick >= checkInterval)
            {
                batteryMonitor.lastCheckTick = nowTick;

                if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
                {
                    LOG("[Battery] [WORKING_LOW] 电压进一步下降到严重低电（%d mV），进入检测关机流程。\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_CHECK;
                    batteryMonitor.lowVoltageCounter = 0;
                }
                else if (BQ27441.Voltage > WORKING_BATTERY_SOC)
                {
                    LOG("[Battery] [WORKING_LOW] 电压恢复正常（%d mV），回到正常状态。\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
                else
                {
                    // 保持在 WORKING_LOW
                }
            }
            break;

        case BATTERY_CHECK:
            if (nowTick - batteryMonitor.lastCheckTick >= checkInterval)
            {
                batteryMonitor.lastCheckTick = nowTick;

                if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
                {
                    batteryMonitor.lowVoltageCounter++;
                    LOG("[Battery] [CHECK] 低电压确认 %d 次（%d mV）\n", batteryMonitor.lowVoltageCounter, BQ27441.Voltage);

                    if (batteryMonitor.lowVoltageCounter >= 5)
                    {
                        LOG("[Battery] [CHECK] 满足关断条件，进入确认关断状态。\n");
                        batteryMonitor.state = BATTERY_CONFIRM_SHUTDOWN;
                    }
                }
                else
                {
                    LOG("[Battery] [CHECK] 电压恢复（%d mV），回到正常状态。\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
            }
            break;

        case BATTERY_CONFIRM_SHUTDOWN:
            LOG("[Battery] [CONFIRM_SHUTDOWN] 连续低电确认，执行关机逻辑...\n");
            // 执行关机动作，比如
             BQ25895_Write(0x09, 0x64); // 示例：写寄存器关机
            batteryMonitor.state = BATTERY_SHUTDOWN;
            break;

        case BATTERY_SHUTDOWN:
            LOG("[Battery] [SHUTDOWN] 当前处于关断状态，保护模式中。\n");
            break;

        default:
            LOG("[Battery] [ERROR] 未知电池状态！强制回归正常。\n");
            batteryMonitor.state = BATTERY_NORMAL;
            break;
    }
}



void battery_status_update_bq27441(void) {
  BQ27441_MultiRead_DMA(&BQ27441);
    //LOG("BQ27441: Voltage=%d mV, Temperature=%d C, AvgCurrent=%d mA, SOC=%d%%, FullChargeCapacity=%d mAh\n", BQ27441.Voltage, BQ27441.Temperature, BQ27441.AvgCurrent, BQ27441.SOC, BQ27441.FullChargeCapacity);
  //low_battery = (BQ27441.SOC < 30) && (BQ27441.SOC != 0);
  low_battery = (BQ27441.SOC < 30) ;
    fully_charged=(BQ27441.SOC>=99);
    float battery = (float)BQ27441.SOC;
    //ScreenUpdateSOC(battery);
    if(battery_flag_400ms){
        battery_flag_400ms=0;
        if (lastBatteryValue == 0 || fabs(battery - lastBatteryValue) <= 10.0f) {
            lastBatteryValue = battery;
            ScreenUpdateSOC(battery);
        }
    }
//    if (BQ27441.Voltage <= 3300) {
//        LOG("电池电压%d\n", BQ27441.Voltage); // 电压过低，关断保护\n");
//      BQ25895_Write(0x09, 0x64); // 电压低，关断保护
//    }
    BatteryMonitor_Run();
  }



#include <stdint.h>
#include <stdio.h>
#include "i2c.h"  // 需要你的 I2C 读写库，例如 STM32 或 ESP32 的 I2C 驱动

#define BQ27441_I2C_ADDR  0x55  // BQ27441 默认 I2C 地址
#define CONTROL_REG       0x00  // 控制寄存器地址
#define FLAGS_REG         0x06  // 设备标志寄存器
#define BLOCK_DATA_CTRL   0x61  // 使能 Block Data Memory Control
#define DATA_BLOCK_CLASS  0x3E  // 数据块分类寄存器
#define DATA_BLOCK        0x3F  // 数据块偏移寄存器
#define BLOCK_DATA_CHECKSUM 0x60  // 校验和寄存器
#define DESIGN_CAP_OFFSET 0x0A  // 设计容量偏移地址
#define SOFT_RESET        0x0042  // 软复位命令
#define SEAL_CMD          0x0020  // 重新封装命令
#define UNSEAL_KEY1       0x8000  // 解封密钥第一部分
#define UNSEAL_KEY2       0x8000  // 解封密钥第二部分
#define TIMEOUT 2000  // 超时时间



// 使能 Block Data Memory Control
void BQ27441_EnableBlockDataMemoryControl() {
    uint8_t data[2] = {0x00, 0x00};
    BQ27441_Write_IT(BLOCK_DATA_CTRL, &data[0], 1);  // 使能数据块存储控制
}

// 选择数据块（82 = 0x52）
void BQ27441_SelectDataBlock(uint8_t block) {
    uint8_t data = block;
    BQ27441_Write_IT(DATA_BLOCK_CLASS, &data, 1);  // 选择数据块
}

// 设置数据块偏移
void BQ27441_SetDataBlockOffset(uint8_t offset) {
    uint8_t data = offset;
    BQ27441_Write_IT(DATA_BLOCK, &data, 1);  // 选择数据块偏移量
}

// 读取数据块校验和
uint8_t BQ27441_ReadChecksum() {
    uint8_t checksum;
   BQ27441_Read_IT( BLOCK_DATA_CHECKSUM, &checksum, 1);  // 读取校验和
    //HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, BLOCK_DATA_CHECKSUM, I2C_MEMADD_SIZE_8BIT, checksum, 2);
    return checksum;
}

// 读取设备标志寄存器
uint16_t BQ27441_ReadFlags() {
    uint8_t data[2] = {0};
    BQ27441_Read_IT(FLAGS_REG, data, 2);
    return (data[1] << 8) | data[0];
}


// 解封 BQ27441
void BQ27441_Unseal() {
    uint8_t data[2];

// 发送第一部分密钥
    data[0] = 0x00; data[1] = 0x80;  // 0x8000
    BQ27441_Write_IT(0x00, data, 2);

// 发送第二部分密钥
    BQ27441_Write_IT(0x00, data, 2);
    LOG("发送第二部分解封密钥: 0x8000\n");

}

// 进入配置更新模式
void BQ27441_SetConfigUpdate() {
    LOG("进入配置更新模式...\n");
    uint8_t data[2] = {0x13, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);

    int timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("进入配置更新模式超时！\n");
            return;
        }
    }
    LOG("成功进入配置更新模式\n");
}



// 读取设计容量
void BQ27441_ReadDesignCapacity(uint8_t *data) {
    BQ27441_Read_IT(DESIGN_CAP_OFFSET, data, 2);
    LOG("读取到的设计容量: %d mAh\n", (data[1] << 8) | data[0]);
}

// 写入新的设计容量
void BQ27441_WriteDesignCapacity(uint16_t new_capacity) {
    uint8_t data[2] = {new_capacity & 0xFF, (new_capacity >> 8) & 0xFF};
    BQ27441_Write_IT(DESIGN_CAP_OFFSET, data, 2);
    LOG("新设计容量写入成功: %d mAh\n", new_capacity);

    // 读取回来的值，确认是否真正写入成功
    uint8_t check_data[2] = {0};
    BQ27441_ReadDesignCapacity(check_data);
    uint16_t written_capacity = (check_data[1] << 8) | check_data[0];

    if (written_capacity == new_capacity) {
        LOG("写入验证成功，设计容量已正确写入: %d mAh\n", written_capacity);
    } else {
        LOG("写入验证失败！读取到的设计容量 = %d mAh，期望值 = %d mAh\n", written_capacity, new_capacity);

        // **尝试重新写入**
        LOG("重新尝试写入设计容量...\n");
        BQ27441_Write_IT(DESIGN_CAP_OFFSET, data, 2);
        HAL_Delay(10); // **给芯片足够时间处理**

        // 再次检查
        BQ27441_ReadDesignCapacity(check_data);
        written_capacity = (check_data[1] << 8) | check_data[0];

        if (written_capacity == new_capacity) {
            LOG("二次写入成功！\n");
        } else {
            LOG("二次写入仍然失败，可能是校验和问题\n");
        }
    }
}



// 更新校验和
void BQ27441_UpdateChecksum() {
    uint8_t old_checksum, new_checksum;
    uint8_t block_data[32];  // 0x40 - 0x5F 数据块
    uint8_t sumX = 0;  // 存储 BlockData 的 8 位求和

    // **读取当前校验和**
    old_checksum = BQ27441_ReadChecksum();
    LOG("读取到的旧校验和: 0x%02X\n", old_checksum);

    if (old_checksum == 0x00) {
        LOG("警告：校验和读取失败，可能 I2C 读取出错\n");
    }

    // **读取整个 BlockData(0x40 - 0x5F)**
    BQ27441_Read_IT(0x40, block_data, 32);

    // **计算 BlockData 的 8 位和**
    for (int i = 0; i < 32; i++) {
        sumX += block_data[i];
    }

    // **计算新校验和**
    new_checksum = 255 - (sumX % 256);
    LOG("计算的新校验和 = 0x%02X\n", new_checksum);

    // **写入新的校验和**
    BQ27441_Write_IT(BLOCK_DATA_CHECKSUM, &new_checksum, 1);
    LOG("新校验和写入成功: 0x%02X\n", new_checksum);

    // **确认是否成功写入**
    uint8_t verify_checksum = BQ27441_ReadChecksum();
    if (verify_checksum != new_checksum) {
        LOG("校验和写入失败！实际 = 0x%02X, 期望 = 0x%02X\n", verify_checksum, new_checksum);
    } else {
        LOG("校验和更新成功！\n");
    }
}




// 发送软复位命令
void BQ27441_SoftReset() {
    uint8_t data[2] = {0x42, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);
    LOG("发送软复位命令");
}
void BQ27441_DoubleSoftReset() {
    LOG("执行第一次软复位...\n");
    BQ27441_SoftReset();
    HAL_Delay(100);  // **给芯片足够时间**

    LOG("执行第二次软复位...\n");
    BQ27441_SoftReset();
    HAL_Delay(100);
}

// 重新封装 BQ27441
void BQ27441_Seal() {
    uint8_t data[2] = {0x20, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);
    LOG("重新封装设备");
}
void BQ27441_SelectAndVerifyBlock() {
    BQ27441_SelectDataBlock(0x52);  // 选择 0x52
    HAL_Delay(10);

    // 读取 BlockData 的第一位，确保已经选中
    uint8_t verify_block = 0;
    BQ27441_Read_IT(DATA_BLOCK_CLASS, &verify_block, 1);
    if (verify_block != 0x52) {
        LOG("警告：BlockData 选中失败，当前值: 0x%02X\n", verify_block);
    } else {
        LOG("BlockData 选中成功: 0x%02X\n", verify_block);
    }
}

// 更新 BQ27441 设计容量
void BQ27441_UpdateDesignCapacity(uint16_t new_capacity) {
    BQ27441_Unseal();

    BQ27441_SetConfigUpdate();

    int timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("等待 CFGUPDATE 置位超时！\n");
            return;
        }
    }
    LOG("CFGUPDATE 置位成功\n");

    BQ27441_EnableBlockDataMemoryControl();

    // **手动选中 BlockData(0x52)**
    BQ27441_SelectAndVerifyBlock();

    BQ27441_SetDataBlockOffset(0x00);

    uint8_t old_checksum = BQ27441_ReadChecksum();
    LOG("读取到的旧校验和: 0x%02X\n", old_checksum);

    uint8_t old_data[2];
    BQ27441_ReadDesignCapacity(old_data);

    BQ27441_WriteDesignCapacity(new_capacity);

    // **读取数据，确保写入成功**
    uint8_t verify_data[2] = {0};
    BQ27441_ReadDesignCapacity(verify_data);
    uint16_t final_capacity = (verify_data[1] << 8) | verify_data[0];

    if (final_capacity != new_capacity) {
        LOG("设计容量写入失败！尝试重新写入...\n");
        BQ27441_WriteDesignCapacity(new_capacity);
        HAL_Delay(10);
    }

    // **更新校验和**
    BQ27441_UpdateChecksum();

    // **强制提交更改**
    BQ27441_SoftReset();

    timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("等待 CFGUPDATE 复位超时！\n");
            return;
        }
    }
    LOG("CFGUPDATE 复位成功\n");

    // **最终确认数据是否生效**
    BQ27441_ReadDesignCapacity(verify_data);
    final_capacity = (verify_data[1] << 8) | verify_data[0];
    LOG("最终读取到的设计容量: %d mAh\n", final_capacity);

    if (final_capacity != new_capacity) {
        LOG("设计容量更新失败！\n");
    } else {
        LOG("设计容量更新成功！\n");
    }

    BQ27441_Seal();
}


// 主应用函数
void main_app1(void) {
    BQ27441_Unseal();
    BQ27441_UpdateDesignCapacity(3000);
    BQ27441_SoftReset();
}

//
//
//const uint16_t terminatevoltage ={0x0C80};       //terminate voltage=3000mV,system shutdown voltage  ,      系统的正常工作电压
////
//const uint16_t loadselect ={0x81};               //load select/load loadselectmode; 0x81 power-mode 0x01 current mode, normally use 0x81 ,if design capacity > 8500mAh change to 0x01
//#define BATTERY_3200MAH
//uint32_t Frack_Num;
//
///**************************************************************************************
//**************************************************************************************/
//
//
//#ifdef BATTERY_3200MAH
////3200mAH
//unsigned short const Designcapacity={0x0C80};     //Design capacity=2500mAh   这个值随着电池的使用会变化
//unsigned short const DesignEnergy={0x2E40};       //Design Energy=2500*3.8mWh; Design Energy=Design capacity*3.7 for 4.2V battery,Design Energy=Design capacity*3.8 for 4.35V battery,
//#define   FCC_SIZE   950
//
////220   充电
//const uint16_t Taperrate ={0x0140};              //Taper rate=250,taper rate=Design Capacity*10/taper current mA;(2500*10/220 = 125) a little higher than charger taper current(~>20mA)
////Taperrate  是一个充电截止判定的量
//
////subclass 81
////放电电流 大于62mA  2500*10 / 62.5ma = 400
//const uint16_t Dsgcurrentthreshold ={0x190};    // 50000/167=299mA,  Dsg current threshold(num)=500, Dsg current threshold(mAh)=Design capacity*10/Dsg current threshold(num)=80mA
//
//
////充电电流一定大于这个电流 100mA 2500*10/100 = 250
//const uint16_t Chgcurrentthreshold ={0xfa};    //Chg current threshold(num)=500, Chg current threshold(mAh)=Design capacity*10/Chg current threshold(num)=80mA,must smaller than charger taper current
//
//
////进入低功耗电流 2500 * 10 / 50ma = 500
//const uint16_t Quitcurrent ={0x01f4};  //{0x03E8};     //Quit current threshold(num)=1000,Quit current threshold(mAh)=Design capacity*10/Quit current threshold(num)=40mA
//
//#endif
//
//
//
//
//
//
//
//
//
///**************************************************************************************
//**************************************************************************************/
//
//
//#ifdef BATTERY_3200MAH
//const uint16_t  Qmax ={0x4000};
//
//const uint16_t  Ra[] ={0x66, 0x66, 0x63, 0x6b, 0x48, 0x3b, 0x3e, 0x3f, 0x35, 0x2f, 0x3c, 0x46, 0x8c, 0x171, 0x24c};
////const uint16_t  Ra[] ={0x34, 0x34, 0x32, 0x36, 0x2d, 0x32, 0x47, 0x52, 0x50, 0x4b, 0x57, 0x52, 0x72, 0x108, 0x1a4};
//#endif


#define DESIGN_CAPACITY_MAH  3300   // 直接输入毫安时 (mAh)
#define NOMINAL_VOLTAGE_MV   3600   // 3.7V 电池
#define TAPER_CURRENT_MA     64    // 充电截止电流 220mA
#define DISCHARGE_CURRENT_MA 62     // 放电电流门限 62mA
#define CHARGE_THRESHOLD_MA  64    // 充电门限 100mA
#define QUIT_CURRENT_MA      50     // 退出低功耗模式电流 50mA

// **电池基本参数**
const uint16_t Designcapacity = (uint16_t)(DESIGN_CAPACITY_MAH);  // 3000mAh
const uint16_t DesignEnergy   = (uint16_t)(DESIGN_CAPACITY_MAH * (NOMINAL_VOLTAGE_MV / 1000.0));  // 计算mWh
const uint16_t terminatevoltage = 3000;  // 系统最低工作电压 3000mV
const uint16_t loadselect = 0x81;  // 负载模式 (0x81: 功率模式, 0x01: 电流模式)

// **充电控制参数**
const uint16_t Taperrate = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / TAPER_CURRENT_MA);
const uint16_t Dsgcurrentthreshold = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / DISCHARGE_CURRENT_MA);
const uint16_t Chgcurrentthreshold = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / CHARGE_THRESHOLD_MA);
const uint16_t Quitcurrent = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / QUIT_CURRENT_MA);
uint32_t Frack_Num;
// **内阻 (Ra) 和 Qmax**
const uint16_t Qmax = 0x4000;  // 固定值
const uint16_t Ra[] = {0x66, 0x66, 0x63, 0x6b, 0x48, 0x3b, 0x3e, 0x3f, 0x35, 0x2f, 0x3c, 0x46, 0x8c, 0x171, 0x24c};

/**************************************************************************************
**************************************************************************************/


/**************************************************************************************
**************************************************************************************/

/**************************************************************************************
**************************************************************************************/


/********************************************************
向subaddr 这个地址写入cmd这个指令（写单个指令)
 ********************************************************/
uint32_t bq_cmdWrite(uint8_t subaddr, uint8_t cmd)
{
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&cmd,1,0xff);
}
/*************************************
  *************************************/
uint32_t bq_read(uint8_t *pBuf,  uint8_t addr, uint8_t len)
{
   return HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, addr, len, pBuf, len, 0xfff);

}
/********************************************************
*********************************************************/
uint32_t bq_MemoryWrite(uint8_t subaddr, uint16_t data )
{
    uint8_t Mdata=(data>>8);
    uint8_t Ldata=(data&0xFF);
            HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&Mdata,1,0xff);
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr+1,I2C_MEMADD_SIZE_8BIT,&Ldata,1,0xff);
}
/********************************************************
*********************************************************/
uint8_t GetCheckSum(uint8_t  addr)
{
    uint8_t checksum = 0, i;
    uint8_t rtBuf[2];

    for (i = 0; i < 32; i++)
    {
        rtBuf[0] = 0;
        bq_read(rtBuf, addr, 1);
        checksum = checksum + rtBuf[0];
        addr++;
    }

    checksum = 0xFF - checksum;
    return checksum;
}
/********************************************************
*********************************************************/
void  bq_Read_Ta_and_Qmax(void)
{

	unsigned short  qmax=0x00;
	uint8_t  Ra_table[32]= {0x0A};
	unsigned short  cap=0x00;

	uint8_t buf[1];
 	uint8_t tbuf[2];

	uint8_t i;
        tbuf[0]=0;
        tbuf[1]=0;
        Ra_table[31]='\r';
        Ra_table[30]='\n';
	bq_cmdWrite(0x00,0x00);   //Places the device in UNSEALED access mode.
	bq_cmdWrite(0x01,0x80);

	bq_cmdWrite(0x00,0x00);
	bq_cmdWrite(0x01,0x80);


	bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

	bq_cmdWrite(0x3e,0x52);     //选择0x52区域      //access the state subclass (decimal 82, 0x52 hex)

	bq_cmdWrite(0x3f,0x00);         //use offset 0x00 for offsets 0 to 31


	bq_read(buf, 0x40, 2);
	qmax = (buf[0] << 8) | buf[1];  //高低位交换

	bq_read(buf, 0x4A, 2);
	cap = (buf[0] << 8) | buf[1]; //高低位交换


	bq_cmdWrite(0x3e,0x59);//选择0x59区域
	bq_cmdWrite(0x3f,0x00);

	for(i=0; i<30; i++)
	{
		bq_read(buf, 0x40+i, 1);
		Ra_table[i] = buf[0]; //高低位交换
	}
      

        Frack_Num++;
        tbuf[0]=Frack_Num>>24;
        tbuf[0]=((Frack_Num>>16)&0xFF);
        tbuf[0]=((Frack_Num>>8)&0xFF);
        tbuf[0]=(Frack_Num&0xFF);
//	bq_cmdWrite(0x3e,0x40);//选择0x59区域
//	bq_cmdWrite(0x3f,0x00);
//
//
//        bq_read(buf, 0x40, 1);
//        HAL_UART_Transmit(&huart2,buf,1,0xff);
//        bq_read(tbuf, 0x41, 1);
//        HAL_UART_Transmit(&huart2,tbuf,1,0xff);

}

/********************************************************
*********************************************************/
void bq_fullReset(void)
{
	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))   //Places the device in UNSEALED access mode.
	{
	    return ;
	}

	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))
	{
	    return ;
	}

	if (bq_cmdWrite(0x00,0x41) || bq_cmdWrite(0x01,0x00)) //Performs a full device reset.
	{
	    return;
	}
}

/********************************************************
*********************************************************/

void bq_CONFIG_subclass82(void)
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x52); //选择0x52区域      //access the state subclass (decimal 82, 0x52 hex)
    bq_cmdWrite(0x3f, 0x00); //use offset 0x00 for offsets 0 to 31


    bq_MemoryWrite(0x40, Qmax);

    bq_MemoryWrite(0x50, terminatevoltage); //terminatevoltage 截止电压(系统能够正常运行的最低电压)

    bq_cmdWrite(0x45, loadselect);

    bq_MemoryWrite(0x4A, Designcapacity); //电池容量  mAh

    bq_MemoryWrite(0x4C, DesignEnergy); //mWh

    bq_MemoryWrite(0x5B, Taperrate);

    checksum = GetCheckSum(0x40);

    bq_cmdWrite(0x60, checksum); //0xba  checksum

}
void bq_CONFIG_subclass81(void) //充放电阈值设置
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x51); //选择0x51区域
    bq_cmdWrite(0x3f, 0x00);
    bq_MemoryWrite(0x40, Dsgcurrentthreshold); //放电电流
    bq_MemoryWrite(0x42, Chgcurrentthreshold); //充电电流，充电电流的判断标准
    bq_MemoryWrite(0x44, Quitcurrent); //静态电源
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum); //0x5e
}

void bq_CONFIG_subclass89(void)//内阻表设置
{
    uint8_t checksum = 0, i = 0;
    bq_cmdWrite(0x3e, 0x59); //选择0x59区域
    bq_cmdWrite(0x3f, 0x00);
    for (i = 0; i < 15; i++) {
        bq_MemoryWrite(0x40 + i * 2, Ra[i]);
    }
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum);
}

/********************************************************
返回0,  进入配置模式并退出成功, 返回1, 进入配置失败, 返回0xFE, 超时
*********************************************************/
uint32_t bq_Config(void) {
    uint8_t rtBuf[2];
    uint16_t value;
    uint32_t result;

    static uint32_t m_tryCnt = 0;

    // **防止配置无限循环**，如果尝试 10 次失败，则返回超时错误
    if (m_tryCnt++ > 10) {
        m_tryCnt = 0;
        LOG("BQ27441 配置超时！\n");
        return 0xFE;
    }

    // **解封 (UNSEAL) BQ27441**
    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) {
        LOG("解封 BQ27441 失败 (步骤 1)\n");
        return 1;
    }
    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) {
        LOG("解封 BQ27441 失败 (步骤 2)\n");
        return 2;
    }

    // **进入配置模式 (CFGUPDATE)**
    if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00)) {
        LOG("进入配置模式失败！\n");
        return 3;
    }

    // 等待 1ms，确保配置模式生效
    HAL_Delay(1);
    result = 1;

    // **等待 BQ27441 进入 CFGUPDATE 模式**
    while (result) {
        result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
        if (result == 0) {
            value = (rtBuf[1] << 8) | rtBuf[0];
            result = (value & 0x10);  // 检查 CFGUPDATE 标志位
        }
    }
    LOG("进入 CFGUPDATE 模式成功！\n");

    // **使能 BlockData RAM 访问**
    bq_cmdWrite(0x61, 0x00);

    // **执行 BQ27441 配置 (Subclass 82)**
    bq_CONFIG_subclass82();

    // **退出配置模式，应用设置**
    LOG("执行软复位，退出配置模式...\n");
    bq_cmdWrite(0x00, 0x42);  // 软复位 (Soft Reset)
    bq_cmdWrite(0x01, 0x00);
    HAL_Delay(10);

    // 发送 `0x00 0x00` 指令，使芯片回到正常工作模式
    bq_cmdWrite(0x00, 0x00);
    bq_cmdWrite(0x01, 0x00);

    LOG("BQ27441 配置完成！\n");
    return 0;
}

/********************************************************
返回0, 配置成功, 返回1, 配置失败  返回其他, 配置超时
*********************************************************/
uint32_t bq_ConfigRead(void) {
    uint8_t rtBuf[2] = {0};
    uint16_t value;
    uint32_t result;

    // 读取 BQ27441 `Flags(0x06)`，检查 CFGUPDATE 状态
    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result != 0) {
        LOG("读取 Flags 失败！\n");
        return 0xFF;
    }

    value = (rtBuf[1] << 8) | rtBuf[0];

    // **如果 CFGUPDATE 仍然置 1，说明配置仍在进行**
    if (value & 0x10) {
        LOG("CFGUPDATE 仍然置位，配置可能未完成。\n");
        return 1;
    }

    // **如果配置完成，发送 `SEAL` 指令，重新封装设备**
    LOG("CFGUPDATE 复位，执行 SEAL 操作...\n");
    bq_cmdWrite(0x00, 0x20);
    bq_cmdWrite(0x01, 0x00);

    return 0;
}


/********************************************************
 读参数
********************************************************/
uint32_t bq_Rdarg(uint16_t *volt, int16_t *avgCur, uint16_t *soc, uint16_t *fcc)
{
  uint8_t  lrtBuf[1];
  uint8_t  mrtBuf[1];

  uint16_t  value;
  uint16_t ret1;
  uint16_t ret2;


  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_LSB, 1, lrtBuf, 1, 0xfff);   //volt  为电压
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *volt = value;

  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_LSB, 1, lrtBuf, 1, 0xfff);     //avgCur 为平均电流
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *avgCur = value;
  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_LSB, 1, lrtBuf, 1, 0xfff);    //soc 为电量百份比
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *soc = value;
  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_LSB, 1, lrtBuf, 1, 0xfff);   //FCC为充电电量
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *fcc = value;
  }
  else
  {
    return 0xF1;
  }
    return ret1;
}
/********************************************************
返回1 需要配置 返回0 已配置过 返回其他值出错
********************************************************/
uint16_t bq_ITPOR(void) {
    uint8_t rtBuf[2] = {0};
    uint32_t ret;

    // **读取 Flags(0x06)，检查 IT (Initial Power On Reset) 标志**
    ret = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 1);
    if (ret != 0) {
        LOG("读取 IT (Initial Power On) 失败！\n");
        return 0xFF;
    }

    // **检查 `0x20` 位是否置 1，判断是否需要重新配置**
    if (rtBuf[0] & 0x20) {
        LOG("ITPOR 置位，需要重新配置 BQ27441\n");
        return 1;
    }

    LOG("ITPOR 未置位，BQ27441 配置正常\n");
    return 0xAA;  // 0xAA 代表配置已经完成，无需修改
}
uint8_t getBoxBattery(void)
{
  uint16_t volt,soc,fcc ;
  int16_t avgCur;

  if(bq_ITPOR() !=0xAA )
  {
    HAL_Delay(500);
    bq_Config();
    if(bq_ConfigRead() != 0)
      return 0xFE;

  }

  bq_Rdarg(&volt, &avgCur, &soc, &fcc);

  return soc&0xFF;
}
uint8_t Batter_Gauge_Test(){
  uint8_t data;
  uint16_t volt,soc,fcc ;
  int16_t avgCur;
  volt=bq_ITPOR();


        if(volt == 1 )
        {
          bq_Config();
          if(bq_ConfigRead() != 0)
          return 2;
        }




  bq_Read_Ta_and_Qmax();
  bq_Rdarg(&volt, &avgCur, &soc, &fcc);






  return 1;

}

/********************************************************
返回0,  进入配置模式并退出成功, 返回1, 进入配置失败, 返回0xFE, 超时
*********************************************************/
uint32_t bq_Config_nww(void)
{
  uint8_t  rtBuf[2];
  uint16_t value;
  uint32_t result;

  static uint8_t m_tryCnt = 0;

  if (m_tryCnt++ > 10)
  {
    m_tryCnt = 0;
    return 0xFE;
  }

  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) //Places the device in UNSEALED access mode.
  {
    return 1;
  }

  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80))
  {
    return 2;
  }

  if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00))
  {
    return 3;
  }

  HAL_Delay(1);
  result=1;
  while(result){
    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result == 0)
    {
      value = (rtBuf[1] << 8) | rtBuf[0];
      result=(value & 0x10);

    }

  }
  bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

  //bq_CONFIG_subclass52();

  return result;
}
uint16_t read_DesignCapacity() {
    uint8_t buf[2] = {0};
    uint16_t capacity = 0;

    // 读取 0x4A 设计容量寄存器（2 字节）
    if (bq_read(buf, 0x4A, 2) == 0) {
        capacity = (buf[1] << 8) | buf[0];  // **高字节在前**
        LOG("读取设计容量成功: %d mAh (0x%04X)\n", capacity, capacity);
    } else {
        LOG("读取设计容量失败!\n");
    }

    return capacity;
}

void main_app(void) {
    // 读取 ITPOR 标志，判断是否需要重新配置电池管理芯片
    if (bq_ITPOR() == 1) {
        LOG("电池管理芯片需要配置...\n");

        // 延迟 1.1 秒，确保 BQ27441 进入稳定状态
        HAL_Delay(1100);

        // 进入配置模式，解封 BQ27441 并设置关键参数
        LOG("开始 BQ27441 配置...\n");
        uint32_t config_status = bq_Config();
read_DesignCapacity();
        // 检查配置是否成功
        if (config_status != 0) {
            LOG("配置 BQ27441 失败，返回值: %d\n", config_status);
            return;
        }
        LOG("BQ27441 配置成功！\n");

        // 读取配置状态，确认是否配置完成
        HAL_Delay(10);
        uint32_t config_read_status = bq_ConfigRead();

        // 确保配置成功
        if (config_read_status != 0) {
            LOG("读取配置失败，返回值: %d\n", config_read_status);
        } else {


            LOG("BQ27441 配置读取成功！\n");
        }

    } else {
        LOG("电池管理芯片已配置，无需重新配置。\n");

    }
}

