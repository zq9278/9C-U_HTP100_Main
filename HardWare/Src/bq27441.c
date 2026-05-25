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
#define DESIGN_CAPACITY     3300
#define DESIGN_ENERGY       12210
#define TERMINATE_VOLTAGE   3000
#define TAPER_RATE          515

#define BQ27441_FLAG_ITPOR       0x0020
#define BQ27441_FLAG_CFGUPMODE   0x0010
#define BQ27441_CMD_CONTROL      0x00
#define BQ27441_SUBCMD_CONTROL_STATUS 0x0000
#define BQ27441_SUBCMD_SET_CFGUPDATE  0x0013
#define BQ27441_SUBCMD_SOFT_RESET     0x0042
#define BQ27441_SUBCMD_SEAL      0x0020
#define BQ27441_CONTROL_STATUS_SS     0x2000

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
    HAL_StatusTypeDef status;

    if (xI2CMutex != NULL) {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
            return HAL_BUSY;
        }
    }

    status = HAL_I2C_Mem_Write(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);

    if (xI2CMutex != NULL) {
        xSemaphoreGive(xI2CMutex);
    }

    return status;
}

/**
 * @brief i2c_read 鍑芥暟瀹炵幇銆? * @param reg 鍙傛暟銆? * @param data 鍙傛暟銆? * @param len 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static HAL_StatusTypeDef i2c_read(uint8_t reg, uint8_t *data, uint8_t len) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    HAL_StatusTypeDef status;

    if (xI2CMutex != NULL) {
        if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE) {
            return HAL_BUSY;
        }
    }

    status = HAL_I2C_Mem_Read(_bq_i2c, BQ27441_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);

    if (xI2CMutex != NULL) {
        xSemaphoreGive(xI2CMutex);
    }

    return status;
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

static bool BQ27441_WriteControl(uint16_t subcmd)
{
    uint8_t cmd[2] = {
            (uint8_t)(subcmd & 0xFF),
            (uint8_t)((subcmd >> 8) & 0xFF)
    };

    return i2c_write(BQ27441_CMD_CONTROL, cmd, 2) == HAL_OK;
}

static uint16_t BQ27441_ReadControlStatus(void)
{
    if (!BQ27441_WriteControl(BQ27441_SUBCMD_CONTROL_STATUS)) {
        return 0xFFFF;
    }

    osDelay(2);
    return read_word(BQ27441_CMD_CONTROL);
}

static bool BQ27441_WaitFlagsSet(uint16_t mask, uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    do {
        if ((read_word(BQ27441_COMMAND_FLAGS) & mask) == mask) {
            return true;
        }
        osDelay(10);
    } while ((HAL_GetTick() - start) < timeoutMs);

    return false;
}

static bool BQ27441_WaitFlagsClear(uint16_t mask, uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();

    do {
        if ((read_word(BQ27441_COMMAND_FLAGS) & mask) == 0) {
            return true;
        }
        osDelay(10);
    } while ((HAL_GetTick() - start) < timeoutMs);

    return false;
}

static bool BQ27441_ReadExtendedData(uint8_t classID, uint8_t offset, uint8_t *value)
{
    uint8_t block = offset / 32U;
    uint8_t blockOffset = offset % 32U;
    uint8_t enable = 0x00;

    if (value == NULL) {
        return false;
    }

    if (i2c_write(BQ27441_EXTENDED_CONTROL, &enable, 1) != HAL_OK) {
        return false;
    }
    if (i2c_write(BQ27441_EXTENDED_DATACLASS, &classID, 1) != HAL_OK) {
        return false;
    }
    if (i2c_write(BQ27441_EXTENDED_DATABLOCK, &block, 1) != HAL_OK) {
        return false;
    }

    osDelay(10);

    return i2c_read(BQ27441_EXTENDED_BLOCKDATA + blockOffset, value, 1) == HAL_OK;
}

static uint16_t BQ27441_ReadExtendedWord(uint8_t classID, uint8_t msbOffset, uint8_t lsbOffset)
{
    uint8_t msb = 0xFF;
    uint8_t lsb = 0xFF;

    (void)BQ27441_ReadExtendedData(classID, msbOffset, &msb);
    (void)BQ27441_ReadExtendedData(classID, lsbOffset, &lsb);

    return (uint16_t)((msb << 8) | lsb);
}

static bool BQ27441_ConfigMatchesDesign(void)
{
    uint16_t designCap = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 10, 11);
    uint16_t designEnergy = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 12, 13);
    uint16_t termVolt = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 16, 17);
    uint16_t taperRate = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 27, 28);

    return designCap == DESIGN_CAPACITY &&
           designEnergy == DESIGN_ENERGY &&
           termVolt == TERMINATE_VOLTAGE &&
           taperRate == TAPER_RATE;
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
    i2c_write(BQ27441_CMD_CONTROL, key, 2);
    osDelay(2);
    i2c_write(BQ27441_CMD_CONTROL, key, 2);
    osDelay(50);

    return (BQ27441_ReadControlStatus() & BQ27441_CONTROL_STATUS_SS) == 0U;
}

bool BQ27441_Seal(void)
{
    bool ok = BQ27441_WriteControl(BQ27441_SUBCMD_SEAL);
    osDelay(10);
    return ok;
}

/**
 * @brief BQ27441_EnterConfigMode 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_EnterConfigMode(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if ((read_word(BQ27441_COMMAND_FLAGS) & BQ27441_FLAG_CFGUPMODE) != 0U) {
        return true;
    }

    if (!BQ27441_WriteControl(BQ27441_SUBCMD_SET_CFGUPDATE)) {
        return false;
    }

    return BQ27441_WaitFlagsSet(BQ27441_FLAG_CFGUPMODE, 2000);
}

/**
 * @brief BQ27441_ExitConfigMode 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_ExitConfigMode(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (!BQ27441_WriteControl(BQ27441_SUBCMD_SOFT_RESET)) {
        return false;
    }

    return BQ27441_WaitFlagsClear(BQ27441_FLAG_CFGUPMODE, 2000);
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
        LOGE("[FuelGauge] Checksum mismatch: write=0x%02X, read=0x%02X\n", checksum, verify_checksum);
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
    uint8_t value = 0xFF;

    (void)BQ27441_ReadExtendedData(classID, offset, &value);
    return value;
}

static uint16_t BQ27441_ReadQmaxRaw(void)
{
    return BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 0, 1);
}

/**
 * @brief BQ27441_ReadQmax 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint16_t BQ27441_ReadQmax(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint32_t qmaxRaw = BQ27441_ReadQmaxRaw();

    return (uint16_t)((qmaxRaw * DESIGN_CAPACITY) / 16384U);
}
/**
 * @brief BQ27441_HardwareReset 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
bool BQ27441_HardwareReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return BQ27441_WriteControl(0x0041);
}
/**
 * @brief BQ27441_DEMO 鍑芥暟瀹炵幇銆? */
bool BQ27441_InitConfig(void) {
    uint16_t flags = read_word(BQ27441_COMMAND_FLAGS);

    if (!BQ27441_Unseal()) {
        LOGE("[FuelGauge] Unseal failed\n");
        return false;
    }

    if ((flags & BQ27441_FLAG_ITPOR) == 0 && BQ27441_ConfigMatchesDesign()) {
        LOGI("[FuelGauge] Config already matches design\n");
        BQ27441_Seal();
        return true;
    }

    LOGI("[FuelGauge] Program config: DC=%d mAh, DE=%d mWh, TV=%d mV, TR=%d\n",
         DESIGN_CAPACITY, DESIGN_ENERGY, TERMINATE_VOLTAGE, TAPER_RATE);

    if (!BQ27441_EnterConfigMode()) {
        LOGE("[FuelGauge] Enter config mode failed\n");
        BQ27441_Seal();
        return false;
    }

    if (!BQ27441_WriteStateBlock_All()) {
        LOGE("[FuelGauge] Write state block failed\n");
        BQ27441_ExitConfigMode();
        BQ27441_Seal();
        return false;
    }

    if (!BQ27441_WriteRaTable(RT_TABLE, RT_TABLE_LEN)) {
        LOGE("[FuelGauge] Write RA table failed\n");
        BQ27441_ExitConfigMode();
        BQ27441_Seal();
        return false;
    }

    if (!BQ27441_ExitConfigMode()) {
        LOGE("[FuelGauge] Exit config mode failed\n");
        BQ27441_Seal();
        return false;
    }

    if (!BQ27441_ConfigMatchesDesign()) {
        LOGE("[FuelGauge] Config verify failed\n");
        BQ27441_Seal();
        return false;
    }

    BQ27441_Seal();
    osDelay(10);
    return true;
}

void BQ27441_DEMO(void) {
    (void)BQ27441_InitConfig();
}

/**
 * @brief BQ27441_VerifyConfig 鍑芥暟瀹炵幇銆? */
void BQ27441_VerifyConfig(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    BQ27441_Unseal();

    uint16_t designCap = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 10, 11);
    uint16_t designEnergy = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 12, 13);
    uint16_t termVolt = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 16, 17);
    uint16_t taperRate = BQ27441_ReadExtendedWord(BQ27441_ID_STATE, 27, 28);

    LOGI("[FuelGauge] Config verify %s\n",
         BQ27441_ConfigMatchesDesign() ? "OK" : "FAILED");
    LOGI("[FuelGauge] Design capacity=%d mAh\n", designCap);
    LOGI("[FuelGauge] Design energy=%d mWh\n", designEnergy);
    LOGI("[FuelGauge] Terminate voltage=%d mV\n", termVolt);
    LOGI("[FuelGauge] Taper rate=%d\n", taperRate);

    BQ27441_Seal();
}

/**
 * @brief BQ27441_PrintRaTable 鍑芥暟瀹炵幇銆? */
void BQ27441_PrintRaTable(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
#ifdef BQ27441_RA_TABLE_LOG
    if (!BQ27441_Unseal()) {
        LOGE("[FuelGauge] RA read unseal failed\n");
        return;
    }

    bool enteredConfig = false;
    for (uint8_t retry = 0; retry < 3U; retry++) {
        if (BQ27441_EnterConfigMode()) {
            enteredConfig = true;
            break;
        }
        osDelay(200);
    }

    if (!enteredConfig) {
        LOGE("[FuelGauge] RA read enter config mode failed, flags=0x%04X, control=0x%04X\n",
             read_word(BQ27441_COMMAND_FLAGS),
             BQ27441_ReadControlStatus());
        BQ27441_Seal();
        return;
    }

    for (uint8_t i = 0; i < 15; i++) {
        uint8_t msb = 0xFF;
        uint8_t lsb = 0xFF;

        if (!BQ27441_ReadExtendedData(BQ27441_ID_RACOMP, i * 2, &msb) ||
            !BQ27441_ReadExtendedData(BQ27441_ID_RACOMP, i * 2 + 1, &lsb)) {
            LOGE("[FuelGauge] RA[%u] read failed\n", i);
            continue;
        }

        uint16_t ra = ((uint16_t)msb << 8) | lsb;
        LOGI("[FuelGauge] RA[%u]=0x%04X\n", i, ra);
    }

    uint16_t qmaxRaw = BQ27441_ReadQmaxRaw();
    LOGI("[FuelGauge] Qmax raw=0x%04X, qmax=%d mAh\n", qmaxRaw, BQ27441_ReadQmax());

    if (!BQ27441_ExitConfigMode()) {
        LOGE("[FuelGauge] RA read exit config mode failed\n");
    }
    BQ27441_Seal();
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
        LOGE("[FuelGauge] Event\r\n");
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
                LOGW("[FuelGauge] Low voltage detected: %d mV\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_CHECK;
                batteryMonitor.lowVoltageCounter = 0;
                batteryMonitor.lastCheckTick = nowTick;
            }
            else if (BQ27441.Voltage <= WORKING_BATTERY_SOC && BQ27441.Voltage != 0)
            {

                LOGW("[FuelGauge] Event", BQ27441.Voltage);


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
                    LOGW("[FuelGauge] Event", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_CHECK;
                    batteryMonitor.lowVoltageCounter = 0;
                }
                else if (BQ27441.Voltage > WORKING_BATTERY_SOC)
                {
                    LOGI("[FuelGauge] Event", BQ27441.Voltage);
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
                    LOGW("[FuelGauge] Low voltage confirm count=%d, voltage=%d mV", batteryMonitor.lowVoltageCounter, BQ27441.Voltage);

                    if (batteryMonitor.lowVoltageCounter >= 5)
                    {
                        LOGW("[FuelGauge] Event");
                        batteryMonitor.state = BATTERY_CONFIRM_SHUTDOWN;
                    }
                }
                else
                {
                    LOGI("[FuelGauge] Event", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
            }
            break;

        case BATTERY_CONFIRM_SHUTDOWN:
            LOGW("[FuelGauge] Event\n");
        if (charging_flag==0)
        {
            BQ25895_Write(0x09, 0x64);
        }


            batteryMonitor.state = BATTERY_SHUTDOWN;
            break;

        case BATTERY_SHUTDOWN:
            LOGW("[FuelGauge] Event");
            batteryMonitor.state = BATTERY_NORMAL;
            break;

        default:
            LOGE("[FuelGauge] Event");
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
#if defined(LOG_SWITCH_OF_BQ27441)
    LOGI("[FuelGauge] Event\n");
    LOGI("[FuelGauge] Voltage=%d mV\n", BQ27441.Voltage);
    LOGI("[FuelGauge] Battery temp=%.1f C\n", (BQ27441.Temperature * 0.1f) - 273.15f);
    LOGI("[FuelGauge] Flags=0x%04X\n", BQ27441.Flags);
    LOGI("[FuelGauge] Nom available cap=%d mAh\n", BQ27441.NomAvailableCap);
    LOGI("[FuelGauge] Full available cap=%d mAh\n", BQ27441.FullAvailableCap);
    LOGI("[FuelGauge] Remaining cap=%d mAh\n", BQ27441.RemainingCap);
    LOGI("[FuelGauge] Full charge cap=%d mAh\n", BQ27441.FullChargeCap);
    LOGI("[FuelGauge] Avg current=%d mA\n", BQ27441.AvgCurrent);
    LOGI("[FuelGauge] Standby current=%d mA\n", BQ27441.StandbyCurrent);
    LOGI("[FuelGauge] Max load current=%d mA\n", BQ27441.MaxLoadCurrent);
    LOGI("[FuelGauge] Avg power=%d mW\n", BQ27441.AvgPower);
    LOGI("[FuelGauge] SOC=%d %%\n", BQ27441.SOC);
    LOGI("[FuelGauge] Internal temp=%.1f C\n", (BQ27441.InternalTemp * 0.1f) - 273.15f);
    LOGI("[FuelGauge] Health=%d %%, status=0x%02X\n", BQ27441.percent, BQ27441.status);
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
