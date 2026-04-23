/*
 * 鏂囦欢: bq27441.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include "bq27441.h"
#include "interface_uart.h"
#include "UserApp.h"

#define LOW_BATTERY_SOC 3150
#define WORKING_BATTERY_SOC    3150
uint8_t BQ27441_TempData[2];
BQ27441_typedef BQ27441;
extern I2C_HandleTypeDef hi2c1;
#define DESIGN_CAPACITY     2450

#define DESIGN_ENERGY      9065
#define TERMINATE_VOLTAGE  2800
#define TAPER_RATE         330

#define RT_TABLE_LEN 30








const uint8_t RT_TABLE[30] = {
        0x00, 0x8A, 0x00, 0x8A, 0x00, 0x89, 0x00, 0x9A, 0x00, 0x78,
        0x00, 0x74, 0x00, 0x91, 0x00, 0xA5, 0x00, 0x9B, 0x00, 0x9C,
        0x00, 0xDE, 0x01, 0x10, 0x02, 0x12, 0x05, 0x64, 0x08, 0x9B
};

static I2C_HandleTypeDef *_bq_i2c = &hi2c1;

/**
 * @brief i2c_write 鍑芥暟瀹炵幇銆? * @param reg 鍙傛暟銆? * @param data 鍙傛暟銆? * @param len 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static HAL_StatusTypeDef i2c_write(uint8_t reg, uint8_t *data, uint8_t len) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return HAL_I2C_Mem_Write(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief i2c_read 鍑芥暟瀹炵幇銆? * @param reg 鍙傛暟銆? * @param data 鍙傛暟銆? * @param len 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *data, uint8_t len) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return HAL_I2C_Mem_Read(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief read_word 鍑芥暟瀹炵幇銆? * @param reg 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static uint16_t read_word(uint8_t reg) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t buf[2];
    i2c_read(reg, buf, 2);
    return (buf[1] << 8) | buf[0];
}


/**
 * @brief BQ27441_Unseal 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_Unseal(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t key[2] = {0x00, 0x80};
    i2c_write(0x00, key, 2);
    i2c_write(0x00, key, 2);
    osDelay(10);
    return true;
}

/**
 * @brief BQ27441_EnterConfigMode 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_EnterConfigMode(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t cmd[2] = {0x13, 0x00};
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}

/**
 * @brief BQ27441_ExitConfigMode 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_ExitConfigMode(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t cmd[2] = {0x42, 0x00};
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}

bool BQ27441_WriteRaTable(const uint8_t* rt_table, uint8_t len)
{
    if(len != 30) return false;
    uint8_t block[32] = {0};
    uint8_t checksum = 0;


    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x59}, 1);
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);
    osDelay(10);


    i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);


    for(uint8_t i=0; i<30; ++i) {
        block[i] = rt_table[i];
        i2c_write(BQ27441_EXTENDED_BLOCKDATA + i, &block[i], 1);
    }



    for(uint8_t i=0; i<32; ++i) checksum += block[i];
    checksum = 0xFF - (checksum&0xFF);
    i2c_write(BQ27441_EXTENDED_CHECKSUM, &checksum, 1);

    uint8_t verify_checksum = 0;
    i2c_read(BQ27441_EXTENDED_CHECKSUM, &verify_checksum, 1);
    if (verify_checksum != checksum) {
        return false;
    }

    return true;
}

/**
 * @brief BQ27441_WriteStateBlock_All 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_WriteStateBlock_All(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t block[32] = {0};
    uint8_t checksum = 0;

    uint8_t sum = 0;

    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x52}, 1);
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);
    osDelay(10);


    i2c_read(BQ27441_EXTENDED_BLOCKDATA, block, 32);

    block[0] = 0x32;
    block[1] = 0x04;




    block[11] = (DESIGN_CAPACITY & 0xFF);
    block[10] = (DESIGN_CAPACITY&0xff00) >> 8;


    block[13] = (DESIGN_ENERGY & 0xFF);
    block[12] = (DESIGN_ENERGY&0xff00) >> 8;


    block[17] = (TERMINATE_VOLTAGE & 0xFF);
    block[16] = (TERMINATE_VOLTAGE&0xff00) >> 8;


    block[28] = (TAPER_RATE & 0xFF);
    block[27] = (TAPER_RATE&0xff00) >> 8;


    for (uint8_t i = 0; i < 32; i++) {
        i2c_write(BQ27441_EXTENDED_BLOCKDATA + i, &block[i], 1);
    }



    for (uint8_t i = 0; i < 32; i++) sum += block[i];
    checksum = 0xFF - (sum&0xFF);
    i2c_write(BQ27441_EXTENDED_CHECKSUM, &checksum, 1);
    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);
    i2c_write(BQ27441_EXTENDED_DATACLASS, (uint8_t[]){0x52}, 1);
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);
    osDelay(10);
    uint8_t verify_checksum = 0;
    i2c_read(BQ27441_EXTENDED_CHECKSUM, &verify_checksum, 1);
    if (verify_checksum != checksum) {
        LOG("鍐欏叆 checksum = 0x%02X锛岃鍥?checksum = 0x%02X\n", checksum, verify_checksum);
        return false;
    }


    return true;
}

/**
 * @brief BQ27441_ReadExtended 鍑芥暟瀹炵幇銆? * @param classID 鍙傛暟銆? * @param offset 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t BQ27441_ReadExtended(uint8_t classID, uint8_t offset) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t value = 0;

    i2c_write(BQ27441_EXTENDED_CONTROL, (uint8_t[]){0x00}, 1);
    i2c_write(BQ27441_EXTENDED_DATACLASS, &classID, 1);
    i2c_write(BQ27441_EXTENDED_DATABLOCK, (uint8_t[]){0x00}, 1);

    osDelay(10);

    i2c_read(BQ27441_EXTENDED_BLOCKDATA + offset, &value, 1);
    return value;
}
/**
 * @brief BQ27441_ReadQmax 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint16_t BQ27441_ReadQmax(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t lsb = BQ27441_ReadExtended(BQ27441_ID_STATE, 0);
    uint8_t msb = BQ27441_ReadExtended(BQ27441_ID_STATE, 1);

    return (lsb << 8) | msb;
}
/**
 * @brief BQ27441_HardwareReset 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_HardwareReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t cmd[2] = { 0x41, 0x00 };
    return i2c_write(0x00, cmd, 2) == HAL_OK;
}
/**
 * @brief BQ27441_DEMO 鍑芥暟瀹炵幇銆? */
void BQ27441_DEMO(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint16_t flags = read_word(BQ27441_COMMAND_FLAGS);
    if ((flags & 0x20) == 0) {
        LOG("? 闈為娆′笂鐢碉紝璺宠繃閰嶇疆锛團lags=0x%04X锛塡n", flags);
        return;
    }
    if (!BQ27441_Unseal()) {
        LOG(" 瑙ｅ皝澶辫触\n");
        return;
    }
    LOG(" 瑙ｅ皝鎴愬姛\n");
    if (!BQ27441_EnterConfigMode()) {
        LOG(" 杩涘叆閰嶇疆妯″紡澶辫触\n");
        return;
    }

    do {
        flags = read_word(0x06);
    } while ((flags & 0x10) == 0);

    LOG(" 宸茶繘鍏ラ厤缃ā寮廫n");

    if (!BQ27441_WriteStateBlock_All()) {
        LOG(" 鍐欏叆鍏ㄩ儴鍙傛暟澶辫触锛乗n");
        return;
    }

    if (!BQ27441_WriteRaTable(RT_TABLE, RT_TABLE_LEN)) {
        LOG("鍐欏叆RT琛ㄥけ璐ワ紒\n");
    } else {
        LOG("鍐欏叆RT琛ㄦ垚鍔燂紒\n");
    }
    LOG(" 鍐欏叆鎵€鏈夎璁″弬鏁板畬鎴怽n");

    if (!BQ27441_ExitConfigMode()) {
        LOG(" 閫€鍑洪厤缃ā寮忓け璐n");
        return;
    }


    uint16_t flags1;
    do { flags1 = read_word(0x06); } while (flags1 & 0x10);

    LOG(" BQ27441 鐢垫睜閰嶇疆瀹屾垚锛乗n");
    osDelay(10);
}

/**
 * @brief BQ27441_VerifyConfig 鍑芥暟瀹炵幇銆? */
void BQ27441_VerifyConfig(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint16_t designCap = BQ27441_ReadExtended(BQ27441_ID_STATE, 11) |
                         (BQ27441_ReadExtended(BQ27441_ID_STATE, 10) << 8);
    uint16_t designEnergy = BQ27441_ReadExtended(BQ27441_ID_STATE, 13) |
                            (BQ27441_ReadExtended(BQ27441_ID_STATE, 12) << 8);
    uint16_t termVolt = BQ27441_ReadExtended(BQ27441_ID_STATE, 17) |
                        (BQ27441_ReadExtended(BQ27441_ID_STATE, 16) << 8);
    uint16_t taperRate = BQ27441_ReadExtended(BQ27441_ID_STATE, 28) |
                         (BQ27441_ReadExtended(BQ27441_ID_STATE, 27) << 8);

    LOG(" 楠岃瘉閰嶇疆锛歕n");
    LOG("  DesignCapacity   = %d mAh\n", designCap);
    LOG("  DesignEnergy     = %d mWh\n", designEnergy);
    LOG("  TerminateVoltage = %d mV\n", termVolt);
    LOG("  TaperRate        = %d\n", taperRate);
}

/**
 * @brief BQ27441_PrintRaTable 鍑芥暟瀹炵幇銆? */
void BQ27441_PrintRaTable(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
#ifdef  LOG_SWITCH_OF_BQ27441

    for (uint8_t i = 0; i < 15; i++) {
        uint8_t lsb = BQ27441_ReadExtended(BQ27441_ID_RACOMP, i * 2);
        uint8_t msb = BQ27441_ReadExtended(BQ27441_ID_RACOMP, i * 2 + 1);
        uint16_t ra = (msb << 8) | lsb;
        LOG("0x%02X,",ra);
    }
    LOG("褰撳墠 Qmax=%d mAh\n", BQ27441_ReadQmax());
#endif
}
/**
 * @brief BQ27441_EnableIT 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_EnableIT(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t enableIT[2] = { 0x20, 0x00 };

    return i2c_write(0x00, enableIT, 2) == HAL_OK;
}
/**
 * @brief I2C_Semaphore_Init 鍑芥暟瀹炵幇銆? */
    void I2C_Semaphore_Init(void) {
        /* 步骤说明：
         * 1) 处理输入参数与前置条件。
         * 2) 执行本函数核心业务逻辑。
         * 3) 输出结果/更新状态并返回。
         */
    xI2CMutex = xSemaphoreCreateMutex();
    xI2CCompleteSem = xSemaphoreCreateBinary();

    if (xI2CMutex == NULL || xI2CCompleteSem == NULL) {
        LOG("I2C淇″彿閲忓垱寤哄け璐?\r\n");
        Error_Handler();
    }
}


/**
 * @brief BQ27441_Read_IT 鍑芥暟瀹炵幇銆? * @param regAddr 鍙傛暟銆? * @param pBuffer 鍙傛暟銆? * @param size 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {

        xSemaphoreTake(xI2CCompleteSem, 0);

        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);

        if (status == HAL_OK) {

            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;
            } else {

                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }
    return status;
}
/**
 * @brief BQ27441_Write_IT 鍑芥暟瀹炵幇銆? * @param regAddr 鍙傛暟銆? * @param pData 鍙傛暟銆? * @param size 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {

        xSemaphoreTake(xI2CCompleteSem, 0);


        status = HAL_I2C_Mem_Write(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size,0xffff);
        if (status == HAL_OK) {

            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;
            } else {

                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }


        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }

    return status;
}

/**
 * @brief BQ27441_MultiRead_DMA 鍑芥暟瀹炵幇銆? * @param BQ 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    BQ27441_Read_IT(0x02, (uint8_t *)&BQ->Temperature, 2);
    BQ27441_Read_IT(0x04, (uint8_t *)&BQ->Voltage, 2);
    BQ27441_Read_IT(0x06, (uint8_t *)&BQ->Flags, 2);
    BQ27441_Read_IT(0x08, (uint8_t *)&BQ->NomAvailableCap, 2);
    BQ27441_Read_IT(0x0A, (uint8_t *)&BQ->FullAvailableCap, 2);
    BQ27441_Read_IT(0x0C, (uint8_t *)&BQ->RemainingCap, 2);
    BQ27441_Read_IT(0x0E, (uint8_t *)&BQ->FullChargeCap, 2);
    BQ27441_Read_IT(0x10, (uint8_t *)&BQ->AvgCurrent, 2);
    BQ27441_Read_IT(0x12, (uint8_t *)&BQ->StandbyCurrent, 2);
    BQ27441_Read_IT(0x14, (uint8_t *)&BQ->MaxLoadCurrent, 2);
    BQ27441_Read_IT(0x18, (uint8_t *)&BQ->AvgPower, 2);
    BQ27441_Read_IT(0x1C, (uint8_t *)&BQ->SOC, 2);
    BQ27441_Read_IT(0x1E, (uint8_t *)&BQ->InternalTemp, 2);
    BQ27441_Read_IT(0x20, (uint8_t *)&BQ->percent, 1);
    BQ27441_Read_IT(0x21, (uint8_t *)&BQ->status, 1);
  return HAL_OK;
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

    TickType_t nowTick = xTaskGetTickCount();
    const TickType_t checkInterval = pdMS_TO_TICKS(500);

    switch (batteryMonitor.state)
    {
        case BATTERY_NORMAL:
            if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
            {
                LOG("[Battery] [NORMAL] 妫€娴嬪埌涓ラ噸浣庣數鍘?%d mV锛岃繘鍏ユ娴?..\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_CHECK;
                batteryMonitor.lowVoltageCounter = 0;
                batteryMonitor.lastCheckTick = nowTick;
            }
            else if (BQ27441.Voltage <= WORKING_BATTERY_SOC && BQ27441.Voltage != 0)
            {

                LOG("[Battery] [NORMAL] 鐢垫睜鐢甸噺鍋忎綆锛?d mV锛夛紝鎻愰啋鐢ㄦ埛銆俓n", BQ27441.Voltage);


                batteryMonitor.state = BATTERY_WORKING_LOW;
            }
            else
            {

            }
            break;

        case BATTERY_WORKING_LOW:

            if (nowTick - batteryMonitor.lastCheckTick >= checkInterval)
            {
                batteryMonitor.lastCheckTick = nowTick;

                if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
                {
                    LOG("[Battery] [WORKING_LOW] 鐢靛帇杩涗竴姝ヤ笅闄嶅埌涓ラ噸浣庣數锛?d mV锛夛紝杩涘叆妫€娴嬪叧鏈烘祦绋嬨€俓n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_CHECK;
                    batteryMonitor.lowVoltageCounter = 0;
                }
                else if (BQ27441.Voltage > WORKING_BATTERY_SOC)
                {
                    LOG("[Battery] [WORKING_LOW] 鐢靛帇鎭㈠姝ｅ父锛?d mV锛夛紝鍥炲埌姝ｅ父鐘舵€併€俓n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
                else
                {

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
                    LOG("[Battery] [CHECK] 浣庣數鍘嬬‘璁?%d 娆★紙%d mV锛塡n", batteryMonitor.lowVoltageCounter, BQ27441.Voltage);

                    if (batteryMonitor.lowVoltageCounter >= 5)
                    {
                        LOG("[Battery] [CHECK] 婊¤冻鍏虫柇鏉′欢锛岃繘鍏ョ‘璁ゅ叧鏂姸鎬併€俓n");
                        batteryMonitor.state = BATTERY_CONFIRM_SHUTDOWN;
                    }
                }
                else
                {
                    LOG("[Battery] [CHECK] 鐢靛帇鎭㈠锛?d mV锛夛紝鍥炲埌姝ｅ父鐘舵€併€俓n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
            }
            break;

        case BATTERY_CONFIRM_SHUTDOWN:
            LOG("[Battery] [CONFIRM_SHUTDOWN] 杩炵画浣庣數纭锛屾墽琛屽叧鏈洪€昏緫...\n");
        if (charging_flag==0)
        {
            BQ25895_Write(0x09, 0x64);
        }


            batteryMonitor.state = BATTERY_SHUTDOWN;
            break;

        case BATTERY_SHUTDOWN:
            LOG("[Battery] [SHUTDOWN] 褰撳墠澶勪簬鍏虫柇鐘舵€侊紝淇濇姢妯″紡涓€俓n");
            batteryMonitor.state = BATTERY_NORMAL;
            break;

        default:
            LOG("[Battery] [ERROR] 鏈煡鐢垫睜鐘舵€侊紒寮哄埗鍥炲綊姝ｅ父銆俓n");
            batteryMonitor.state = BATTERY_NORMAL;
            break;
    }
}



/**
 * @brief battery_status_update_bq27441 鍑芥暟瀹炵幇銆? */
void battery_status_update_bq27441(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  BQ27441_MultiRead_DMA(&BQ27441);
#ifdef  LOG_SWITCH_OF_BQ27441
    LOG("BQ27441 鐘舵€侊細\n");
    LOG("  鐢垫睜鐢靛帇 = %d mV\n", BQ27441.Voltage);
    LOG("  鐢垫睜娓╁害 = %.1f 鈩僜n", (BQ27441.Temperature * 0.1f) - 273.15f);
    LOG("  鐘舵€佹爣蹇?= 0x%04X\n", BQ27441.Flags);
    LOG("  鏍囩О鍙敤瀹归噺(鏃犺礋杞戒笅) = %d mAh\n", BQ27441.NomAvailableCap);
    LOG("  婊″彲鐢ㄥ閲忥紙鍏呮弧鎬诲閲忥紝鏃犺礋杞斤級 = %d mAh\n", BQ27441.FullAvailableCap);
    LOG("  瀹為檯鍓╀綑瀹归噺锛堝綋鍓嶈礋杞戒笅锛?= %d mAh\n", BQ27441.RemainingCap);
    LOG("  瀹為檯婊″厖瀹归噺锛堝綋鍓嶈礋杞戒笅锛?= %d mAh\n", BQ27441.FullChargeCap);
    LOG("  骞冲潎鐢垫祦 = %d mA\n", BQ27441.AvgCurrent);
    LOG("  寰呮満鐢垫祦 = %d mA\n", BQ27441.StandbyCurrent);
    LOG("  鏈€澶ц礋杞界數娴?= %d mA\n", BQ27441.MaxLoadCurrent);
    LOG("  骞冲潎鍔熺巼 = %d mW\n", BQ27441.AvgPower);
    LOG("  褰撳墠鐢甸噺 = %d %%\n", BQ27441.SOC);
    LOG("  鑺墖鍐呴儴娓╁害 = %.1f 鈩僜n", (BQ27441.InternalTemp * 0.1f) - 273.15f);
    LOG("  鐢垫睜鍋ュ悍搴?= %d %%锛堢姸鎬佺爜锛?x%02X锛塡n", BQ27441.percent, BQ27441.status);
#endif









  low_battery = (BQ27441.SOC < 10) ;
    fully_charged=(BQ27441.SOC > 98);
    float battery = (float)BQ27441.SOC;


    if(battery_flag_400ms){
        battery_flag_400ms=0;




        ScreenUpdateSOC(battery);
    }
    BatteryMonitor_Run();
  }


