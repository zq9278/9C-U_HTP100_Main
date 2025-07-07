
#include "main.h"
#define LOW_BATTERY_SOC 3150
#define WORKING_BATTERY_SOC    3150  // 提醒用户电压
uint8_t BQ27441_TempData[2];
BQ27441_typedef BQ27441;
extern I2C_HandleTypeDef hi2c1;
#define DESIGN_CAPACITY     2450   // mAh
//#define DESIGN_ENERGY      12210   // mWh
#define DESIGN_ENERGY      9065   // mWh
#define TERMINATE_VOLTAGE  3400    // mV
#define TAPER_RATE         330     // 见BQ27441官方推荐，一般=设计容量/10。充满电的电流33mA
// 推荐在.h文件或本文件顶部定义
#define RT_TABLE_LEN 30

//const uint8_t RT_TABLE[30] = {
//        0x00,0x66, 0x00,0x66, 0x00,0x63, 0x00,0x6B, 0x00,0x48,
//        0x00,0x3B, 0x00,0x3E, 0x00,0x3F, 0x00,0x35, 0x00,0x2F,
//        0x00,0x3C, 0x00,0x46, 0x00,0x8C, 0x01,0x71, 0x02,0x4C
//};
const uint8_t RT_TABLE[30] = {
        0x00,0xD0, 0x00,0xD0, 0x00,0xCE, 0x00,0xE2, 0x00,0x9D,
        0x00,0x83, 0x00,0x8D, 0x00,0x90, 0x00,0x7A, 0x00,0x6D,
        0x00,0x89, 0x00,0xA3, 0x01,0x43, 0x03,0x52, 0x05,0x4C
};
//0xCC00,0xCC00,0xC600,0xD600,0x9000,0x7600,0x7C00,0x7E00,0x6A00,0x5E00,0x7800,0x8C00,0x1801,0xE202,0x9804 12223
static I2C_HandleTypeDef *_bq_i2c = &hi2c1;

static HAL_StatusTypeDef i2c_write(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Write(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *data, uint8_t len) {
    return HAL_I2C_Mem_Read(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static uint16_t read_word(uint8_t reg) {
    uint8_t buf[2];
    i2c_read(reg, buf, 2);
    return (buf[1] << 8) | buf[0];
}


bool BQ27441_Unseal(void) {
    uint8_t key[2] = {0x00, 0x80};
    i2c_write(0x00, key, 2);
    i2c_write(0x00, key, 2);
    osDelay(10);
    return true;
}

bool BQ27441_EnterConfigMode(void) {
    uint8_t cmd[2] = {0x13, 0x00};
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}

bool BQ27441_ExitConfigMode(void) {
    uint8_t cmd[2] = {0x42, 0x00};
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}
// 一次性写入完整Ra Table，推荐在CFGUPDATE模式下调用
bool BQ27441_WriteRaTable(const uint8_t* rt_table, uint8_t len)
{
    if(len != 30) return false;  // 只支持30字节标准RT表
    uint8_t block[32] = {0};
    uint8_t checksum = 0;

    // 1. 选择Ra Table窗口
    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);    // BlockDataControl
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x59}, 1);  // DataClass=0x59(RaTable)
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);  // DataBlock=0
    osDelay(10);//重要

    // 2. 先读出原始block（32字节）
    i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);

    // 3. 写入30字节RT表，覆盖0~29
    for(uint8_t i=0; i<30; ++i) {
        block[i] = rt_table[i];
        i2c_write(BQ27441_EXTENDED_BLOCKDATA + i, &block[i], 1);
    }

    // 4. 重新读回block，刷新校验和
    //i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);
    for(uint8_t i=0; i<32; ++i) checksum += block[i];
    checksum = 0xFF - (checksum&0xFF);
    i2c_write(BQ27441_EXTENDED_CHECKSUM, &checksum, 1);

    uint8_t verify_checksum = 0;
    i2c_read(BQ27441_EXTENDED_CHECKSUM, &verify_checksum, 1);
    if (verify_checksum != checksum) {
        return false; // 校验和不一致，写入失败
    }

    return true;
}

bool BQ27441_WriteStateBlock_All(void) {
    uint8_t block[32] = {0};
    uint8_t checksum = 0;
    uint8_t OLD_checksum = 0;
    uint8_t sum = 0;
    // ---- 1. 设置BlockData窗口 ----
    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);   // BlockDataControl
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x52}, 1); // DataClass (State)
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1); // DataBlock = 0
    osDelay(10);
    //i2c_read(BQ27441_CHECKSUM_DATA, &OLD_checksum, 1);
    // ---- 2. 先读出原始block[32] ----
    i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);
    // ---- 3. 写入关键参数 ----
//    block[0] = (0x00);
//    block[1] = (0x40);
//    // DesignCapacity (0x0A/0x0B)
    block[0] = (0x2f);
    block[1] = (0xbf);

    block[11] = (DESIGN_CAPACITY & 0xFF);
    block[10] = (DESIGN_CAPACITY&0xff00) >> 8;

    // DesignEnergy (0x0C/0x0D)
    block[13] = (DESIGN_ENERGY & 0xFF);
    block[12] = (DESIGN_ENERGY&0xff00) >> 8;

    // TerminateVoltage (0x10/0x11)
    block[17] = (TERMINATE_VOLTAGE & 0xFF);
    block[16] = (TERMINATE_VOLTAGE&0xff00) >> 8;

    // TaperRate (0x1B/0x1C)
    block[28] = (TAPER_RATE & 0xFF);
    block[27] = (TAPER_RATE&0xff00) >> 8;

    // ---- 4. 回写全部参数 ----
    for (uint8_t i = 0; i < 32; i++) {
        i2c_write(BQ27441_EXTENDED_BLOCKDATA + i, &block[i], 1);
    }

// ---- 计算校验和 ----
    // i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);
    for (uint8_t i = 0; i < 32; i++) sum += block[i];
    checksum = 0xFF - (sum&0xFF);
    i2c_write(BQ27441_EXTENDED_CHECKSUM, &checksum, 1);
    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);   // BlockDataControl
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x52}, 1); // DataClass (State)
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1); // DataBlock = 0
    osDelay(10);
    uint8_t verify_checksum = 0;
    i2c_read(BQ27441_EXTENDED_CHECKSUM, &verify_checksum, 1);
    if (verify_checksum != checksum) {
        LOG("写入 checksum = 0x%02X，读回 checksum = 0x%02X\n", checksum, verify_checksum);
        return false; // 校验和不一致，写入失败
    }


    return true;
}

uint8_t BQ27441_ReadExtended(uint8_t classID, uint8_t offset) {
    uint8_t value = 0;

    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);     // 0x61 <- 0x00
    i2c_write(BQ27441_EXTENDED_DATACLASS, &classID, 1);            // 0x3E <- classID
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);   // 0x3F <- 0x00

    osDelay(10);  // 等待 block 数据刷新

    i2c_read(BQ27441_EXTENDED_BLOCKDATA + offset, &value, 1);      // 从 block 区偏移读取
    return value;
}
uint16_t BQ27441_ReadQmax(void) {
    uint8_t lsb = BQ27441_ReadExtended(BQ27441_ID_STATE, 0);
    uint8_t msb = BQ27441_ReadExtended(BQ27441_ID_STATE, 1);
    //return (msb << 8) | lsb;
    return (lsb << 8) | msb;
}
bool BQ27441_HardwareReset(void) {
    uint8_t cmd[2] = { 0x41, 0x00 };
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}
void BQ27441_DEMO(void) {
    uint16_t flags = read_word(BQ27441_COMMAND_FLAGS);
    if ((flags & 0x20) == 0) {
        LOG("? 非首次上电，跳过配置（Flags=0x%04X）\n", flags);
        return;
    }
    if (!BQ27441_Unseal()) {
        LOG(" 解封失败\n");
        return;
    }
    LOG(" 解封成功\n");
    if (!BQ27441_EnterConfigMode()) {
        LOG(" 进入配置模式失败\n");
        return;
    }
    // 等待CFGUPDATE
    do {
        flags = read_word(0x06);
    } while ((flags & 0x10) == 0);

    LOG(" 已进入配置模式\n");
    // 重点：一次性写入所有核心参数
    if (!BQ27441_WriteStateBlock_All()) {
        LOG(" 写入全部参数失败！\n");
        return;
    }
//    // 在CFGUPDATE模式下调用
    if (!BQ27441_WriteRaTable(RT_TABLE, RT_TABLE_LEN)) {
        LOG("写入RT表失败！\n");
    } else {
        LOG("写入RT表成功！\n");
    }
    LOG(" 写入所有设计参数完成\n");
    // 退出配置模式
    if (!BQ27441_ExitConfigMode()) {
        LOG(" 退出配置模式失败\n");
        return;
    }

    // 等待CFGUPDATE清零
    uint16_t flags1;
    do { flags1 = read_word(0x06); } while (flags1 & 0x10);
    // BQ27441_EnableIT();
    LOG(" BQ27441 电池配置完成！\n");
    osDelay(10);
}

void BQ27441_VerifyConfig(void) {
    uint16_t designCap = BQ27441_ReadExtended(BQ27441_ID_STATE, 11) |
                         (BQ27441_ReadExtended(BQ27441_ID_STATE, 10) << 8);
    uint16_t designEnergy = BQ27441_ReadExtended(BQ27441_ID_STATE, 13) |
                            (BQ27441_ReadExtended(BQ27441_ID_STATE, 12) << 8);
    uint16_t termVolt = BQ27441_ReadExtended(BQ27441_ID_STATE, 17) |
                        (BQ27441_ReadExtended(BQ27441_ID_STATE, 16) << 8);
    uint16_t taperRate = BQ27441_ReadExtended(BQ27441_ID_STATE, 28) |
                         (BQ27441_ReadExtended(BQ27441_ID_STATE, 27) << 8);

    LOG(" 验证配置：\n");
    LOG("  DesignCapacity   = %d mAh\n", designCap);
    LOG("  DesignEnergy     = %d mWh\n", designEnergy);
    LOG("  TerminateVoltage = %d mV\n", termVolt);
    LOG("  TaperRate        = %d\n", taperRate);
}

void BQ27441_PrintRaTable(void) {
    //LOG("当前芯片内置 Ra 表：\n");
    for (uint8_t i = 0; i < 15; i++) {
        uint8_t lsb = BQ27441_ReadExtended(BQ27441_ID_RACOMP, i * 2);
        uint8_t msb = BQ27441_ReadExtended(BQ27441_ID_RACOMP, i * 2 + 1);
        uint16_t ra = (msb << 8) | lsb;
        LOG("0x%02X,",ra);
    }
    LOG("当前 Qmax=%d mAh\n", BQ27441_ReadQmax());
}
bool BQ27441_EnableIT(void) {
    uint8_t enableIT[2] = { 0x20, 0x00 };  // Enable IT subcmd
    //uint8_t enableIT[2] = { 0x00, 0x00 };  // Enable IT subcmd
    return i2c_write(0x00, enableIT, 2) == HAL_OK;
}
    void I2C_Semaphore_Init(void) {
    xI2CMutex = xSemaphoreCreateMutex();
    xI2CCompleteSem = xSemaphoreCreateBinary();

    if (xI2CMutex == NULL || xI2CCompleteSem == NULL) {
        printf("I2C信号量创建失败!\r\n");
        Error_Handler();
    }
}


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
// DMA 读取函数
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ) {
    BQ27441_Read_IT(0x02, (uint8_t *)&BQ->Temperature, 2);             // 温度
    BQ27441_Read_IT(0x04, (uint8_t *)&BQ->Voltage, 2);                 // 电压
    BQ27441_Read_IT(0x06, (uint8_t *)&BQ->Flags, 2);                   // 标志位
    BQ27441_Read_IT(0x08, (uint8_t *)&BQ->NomAvailableCap, 2);         // 标称可用容量
    BQ27441_Read_IT(0x0A, (uint8_t *)&BQ->FullAvailableCap, 2);        // 满可用容量
    BQ27441_Read_IT(0x0C, (uint8_t *)&BQ->RemainingCap, 2);            // 剩余容量
    BQ27441_Read_IT(0x0E, (uint8_t *)&BQ->FullChargeCap, 2);           // 满充容量
    BQ27441_Read_IT(0x10, (uint8_t *)&BQ->AvgCurrent, 2);              // 平均电流
    BQ27441_Read_IT(0x12, (uint8_t *)&BQ->StandbyCurrent, 2);          // 待机电流
    BQ27441_Read_IT(0x14, (uint8_t *)&BQ->MaxLoadCurrent, 2);          // 最大负载电流
    BQ27441_Read_IT(0x18, (uint8_t *)&BQ->AvgPower, 2);                // 平均功率
    BQ27441_Read_IT(0x1C, (uint8_t *)&BQ->SOC, 2);                     // SOC
    BQ27441_Read_IT(0x1E, (uint8_t *)&BQ->InternalTemp, 2);            // 芯片内部温度
    BQ27441_Read_IT(0x20, (uint8_t *)&BQ->percent, 1);                     // 电池健康度
    BQ27441_Read_IT(0x21, (uint8_t *)&BQ->status, 1);                     // 电池健康度
  return HAL_OK; // 所有读取操作成功
}
extern uint8_t low_battery;
extern uint8_t battery_flag_400ms;
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
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//关闭屏幕
//            close_mianAPP();
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
    LOG(
            "BQ27441 状态：\n"
            "  电池电压 = %d mV\n"
            "  电池温度 = %.1f ℃\n"
            "  状态标志 = 0x%04X\n"
            "  标称可用容量(无负载下) = %d mAh\n"
            "  满可用容量（充满总容量，无负载） = %d mAh\n"
            "  实际剩余容量（当前负载下） = %d mAh\n"
            "  实际满充容量（当前负载下） = %d mAh\n"
            "  平均电流 = %d mA\n"
            "  待机电流 = %d mA\n"
            "  最大负载电流 = %d mA\n"
            "  平均功率 = %d mW\n"
            "  当前电量 = %d %%\n"
            "  芯片内部温度 = %.1f ℃\n"
            "  电池健康度 = %d %%（状态码：0x%02X）\n",
            BQ27441.Voltage,
            (BQ27441.Temperature * 0.1f) - 273.15f,
            BQ27441.Flags,
            BQ27441.NomAvailableCap,
            BQ27441.FullAvailableCap,
            BQ27441.RemainingCap,
            BQ27441.FullChargeCap,
            BQ27441.AvgCurrent,
            BQ27441.StandbyCurrent,
            BQ27441.MaxLoadCurrent,
            BQ27441.AvgPower,
            BQ27441.SOC,
            (BQ27441.InternalTemp * 0.1f) - 273.15f,
            BQ27441.percent,
            BQ27441.status
    );
    //LOG( "%d mV\n",BQ27441.Voltage);
//if(charging_flag==1) {
//    if ((BQ27441.Voltage < 4200) || (BQ27441.AvgCurrent < 70)) {
//        LOG("low voltage or current\n");
//        BQ25895_Init();
//    }
//}
    //LOG("BQ27441: Voltage=%d mV, Temperature=%d C, AvgCurrent=%d mA, SOC=%d%%, FullChargeCapacity=%d mAh\n", BQ27441.Voltage, BQ27441.Temperature, BQ27441.AvgCurrent, BQ27441.SOC, BQ27441.FullChargeCapacity);
  //low_battery = (BQ27441.SOC < 30) && (BQ27441.SOC != 0);
  low_battery = (BQ27441.SOC < 30) ;
    //fully_charged=(BQ27441.SOC==100);
    float battery = (float)BQ27441.SOC;
    //ScreenUpdateSOC(battery);
    //LOG("BQ27441: SOC=%d%%\n", BQ27441.SOC);
    if(battery_flag_400ms){
        battery_flag_400ms=0;
//        if (lastBatteryValue == 0 || fabs(battery - lastBatteryValue) <= 10.0f) {
//            lastBatteryValue = battery;
//            ScreenUpdateSOC(battery);
//        }
        ScreenUpdateSOC(battery);
    }
    BatteryMonitor_Run();
  }

