/*
 * 鏂囦欢: ads1220.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include <stdint.h>
#include "ads1220.h"
#include "UserApp.h"
#include "interface_uart.h"
#include "pid.h"



volatile uint8_t spi_rx_buffer[3];
volatile int32_t ads1220_result = 0;
volatile uint8_t data_ready = 0;
extern SPI_HandleTypeDef hspi2;
extern PID_TypeDef MotorPID;


#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi2;




/**
 * @brief SPI2_DMA_Semaphores_Init 鍑芥暟瀹炵幇銆? */
void SPI2_DMA_Semaphores_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    spi2TxDmaSemaphoreHandle = xSemaphoreCreateBinary();
    spi2RxDmaSemaphoreHandle = xSemaphoreCreateBinary();

    if (spi2TxDmaSemaphoreHandle != NULL) {
        xSemaphoreGive(spi2TxDmaSemaphoreHandle);
    }
    if (spi2RxDmaSemaphoreHandle != NULL) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);
    }
}



/**
 * @brief SPI2_Transmit_DMA 鍑芥暟瀹炵幇銆? * @param txData 鍙傛暟銆? * @param size 鍙傛暟銆? * @param timeout 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef SPI2_Transmit_DMA(uint8_t *txData, uint16_t size, uint32_t timeout) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (xSemaphoreTake(spi2TxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_BUSY;
    }
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(&hspi2, txData, size);

    if (status != HAL_OK) {
        xSemaphoreGive(spi2TxDmaSemaphoreHandle);
    }

    return status;
}




/**
 * @brief SPI2_Receive_DMA 鍑芥暟瀹炵幇銆? * @param rxData 鍙傛暟銆? * @param size 鍙傛暟銆? * @param timeout 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef SPI2_Receive_DMA(uint8_t *rxData, uint16_t size, uint32_t timeout) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    if (xSemaphoreTake(spi2RxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_BUSY;
    }
    HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(&hspi2, rxData, size);
    if (status != HAL_OK) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);
    }
    return status;
}




/**
 * @brief ADS1220_WriteRegister 鍑芥暟瀹炵幇銆? * @param reg 鍙傛暟銆? * @param value 鍙傛暟銆? */
void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t cmd[2] = {0x40 | (reg << 2), value};
    ADS1220_CS_LOW();
    if (SPI2_Transmit_DMA(cmd, 2, 100) != HAL_OK) {
        LOGE("[Pressure] Write register failed, reg=0x%02X\n", reg);
    }
    ADS1220_CS_HIGH();
}




/**
 * @brief ADS1220_Init 鍑芥暟瀹炵幇銆? */
void ADS1220_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    HAL_Delay(1);

    uint8_t reset_cmd = ADS1220_CMD_RESET;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&reset_cmd, 1, 100);

    ADS1220_CS_HIGH();
    osDelay(50);

    uint8_t selfcal_cmd = ADS1220_CMD_SELFCAL;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&selfcal_cmd, 1, 100);

    ADS1220_CS_HIGH();
    osDelay(50);


    ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x3E);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00);
}




/**
 * @brief ADS1220_StartConversion 鍑芥暟瀹炵幇銆? */
void ADS1220_StartConversion(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t start_cmd = ADS1220_CMD_START_SYNC;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&start_cmd, 1, 100);
    ADS1220_CS_HIGH();
}




/**
 * @brief ADS1220_StopConversion 鍑芥暟瀹炵幇銆? */
void ADS1220_StopConversion(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t stop_cmd = ADS1220_CMD_POWERDOWN;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&stop_cmd, 1, 100);
    ADS1220_CS_HIGH();
}




/**
 * @brief ADS1220_ReadData 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
int32_t ADS1220_ReadData(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t read_cmd = ADS1220_CMD_RDATA;
    uint8_t rx_buffer[3] = {0};

    ADS1220_CS_LOW();


    if (SPI2_Transmit_DMA(&read_cmd, 1, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        LOGE("[Pressure] Event");
        return -1;
    }


    if (SPI2_Receive_DMA(rx_buffer, 3, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        LOGE("[Pressure] Event\n");
        return -1;
    }

    ADS1220_CS_HIGH();

    int32_t result = ((int32_t)rx_buffer[0] << 16) |
                     ((int32_t)rx_buffer[1] << 8)  |
                     rx_buffer[2];

    return (result & 0x800000) ? (result | 0xFF000000) : result;
}




/**
 * @brief ADS1220_GetSensitivityBySetpoint 鍑芥暟瀹炵幇銆? * @param pressure_setpoint_mmhg 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static float ADS1220_GetSensitivityBySetpoint(float pressure_setpoint_mmhg) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (pressure_setpoint_mmhg <= 200.0f) {
        return SENSITIVITY_150;
    }
    if (pressure_setpoint_mmhg <= 300.0f) {
        return SENSITIVITY_250;
    }
    if (pressure_setpoint_mmhg <= 400.0f) {
        return SENSITIVITY_350;
    }
    if (pressure_setpoint_mmhg <= 500.0f) {
        return SENSITIVITY_450;
    }
    return SENSITIVITY_550;
}

/**
 * @brief ADS1220_ReadPressure 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float ADS1220_ReadPressure(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    int32_t raw_data = ADS1220_ReadData();
    if (raw_data == -1) return -1.0f;

    raw_data = -raw_data;
    float V_in = (raw_data * VREF) / 8388607.0f;
    float V_load = V_in / GAIN;
    float sensitivity = ADS1220_GetSensitivityBySetpoint(MotorPID.setpoint);
    float weight = V_load / ((sensitivity / 1000.0f) * EXCITATION_VOLTAGE);

    return weight * 1000.0f;
}




/**
 * @brief Discard_dirty_data 鍑芥暟瀹炵幇銆? */
void Discard_dirty_data(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    for (int i = 0; i < 5; i++) {
        ADS1220_ReadPressure();
        HAL_Delay(10);
    }
}




/**
 * @brief Start_SPI_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Start_SPI_Task(void const *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    for (;;) {
        float pressure = ADS1220_ReadPressure();
        if (pressure >= 0) {
            LOGI("[Pressure] Value=%.2f g\n", pressure);
        } else {
            LOGE("[Pressure] Event\n");
        }
        osDelay(500);
    }
}

