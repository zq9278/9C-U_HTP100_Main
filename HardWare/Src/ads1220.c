/*
 * @Author: zhangqi
 * @Date: 2024-12-30 17:40:27
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 16:37:52
 */

// SPI接口代码，用于与MCU通信的ADS1220
#include "main.h"
#include <stdint.h>

// 定义全局变量
volatile uint8_t spi_rx_buffer[3];
volatile int32_t ads1220_result = 0;
volatile uint8_t data_ready = 0; // 标志位
extern SPI_HandleTypeDef hspi2;
/*
 * SPI2 DMA + 完成信号量封装 (ADS1220 支持，发送与接收分离信号量)
 * 作者: zhangqi
 * 日期: 2024-12-30
 * 修改日期: 2025-01-05
 */

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi2;

// ============================
// 信号量初始化
// ============================
void SPI2_DMA_Semaphores_Init(void) {
    spi2TxDmaSemaphoreHandle = xSemaphoreCreateBinary();
    spi2RxDmaSemaphoreHandle = xSemaphoreCreateBinary();

    if (spi2TxDmaSemaphoreHandle != NULL) {
        //xSemaphoreGive(spi2TxDmaSemaphoreHandle);  // 初始释放发送信号量
    }

    if (spi2RxDmaSemaphoreHandle != NULL) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);  // 初始释放接收信号量
    }
}
// ============================
// SPI 发送封装（DMA + 发送信号量）
// ============================
HAL_StatusTypeDef SPI2_Transmit_DMA(uint8_t *txData, uint16_t size, uint32_t timeout) {
    if (xSemaphoreTake(spi2TxDmaSemaphoreHandle, 10) != pdPASS) {
        return HAL_BUSY;  // 上一次发送未完成
    }
    //xSemaphoreTake(spi2TxDmaSemaphoreHandle, portMAX_DELAY);

    if (HAL_SPI_Transmit_DMA(&hspi2, txData, size) != HAL_OK) {
        xSemaphoreGive(spi2TxDmaSemaphoreHandle);  // 启动失败，立即释放信号量
        return HAL_ERROR;
    }

    if (xSemaphoreTake(spi2TxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_TIMEOUT;  // 超时处理
    }

    return HAL_OK;
}

// ============================
// SPI 接收封装（DMA + 接收信号量）
// ============================
HAL_StatusTypeDef SPI2_Receive_DMA(uint8_t *rxData, uint16_t size, uint32_t timeout) {
    if (xSemaphoreTake(spi2RxDmaSemaphoreHandle, 0) != pdPASS) {
        return HAL_BUSY;  // 上一次接收未完成
    }

    if (HAL_SPI_Receive_DMA(&hspi2, rxData, size) != HAL_OK) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);  // 启动失败立即释放
        return HAL_ERROR;
    }

    if (xSemaphoreTake(spi2RxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_TIMEOUT;  // 超时处理
    }

    return HAL_OK;
}

// ============================
// 写寄存器函数
// ============================
void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t cmd[2] = {0x40 | (reg << 2), value};  // WREG命令 + 地址 + 数据
    ADS1220_CS_LOW();
    if (SPI2_Transmit_DMA(cmd, 2, 100) != HAL_OK) {
        printf("写寄存器失败 - REG: 0x%02X\n", reg);
    }
    ADS1220_CS_HIGH();
}

// ============================
// ADS1220 初始化
// ============================
void ADS1220_Init(void) {
    HAL_Delay(1);  // 上电稳定时间

    uint8_t reset_cmd = ADS1220_CMD_RESET;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&reset_cmd, 1, 100);
    ADS1220_CS_HIGH();
    osDelay(50);  // 等待复位完成

    uint8_t selfcal_cmd = ADS1220_CMD_SELFCAL;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&selfcal_cmd, 1, 100);
    ADS1220_CS_HIGH();
    osDelay(50);  // 自校准完成等待

    // 配置寄存器
    ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x3E);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00);
}

// ============================
// 启动连续转换模式
// ============================
void ADS1220_StartConversion(void) {
    uint8_t start_cmd = ADS1220_CMD_START_SYNC;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&start_cmd, 1, 100);
    ADS1220_CS_HIGH();
}

// ============================
// 停止连续转换
// ============================
void ADS1220_StopConversion(void) {
    uint8_t stop_cmd = ADS1220_CMD_POWERDOWN;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&stop_cmd, 1, 100);
    ADS1220_CS_HIGH();
}

// ============================
// 读取数据函数 (DMA + 分离信号量处理)
// ============================
int32_t ADS1220_ReadData(void) {
    uint8_t read_cmd = ADS1220_CMD_RDATA;
    uint8_t rx_buffer[3] = {0};

    ADS1220_CS_LOW();

    // 发送读取命令
    if (SPI2_Transmit_DMA(&read_cmd, 1, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        printf("读取命令发送失败\n");
        return -1;
    }

    // 接收数据
    if (SPI2_Receive_DMA(rx_buffer, 3, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        printf("数据接收失败\n");
        return -1;
    }

    ADS1220_CS_HIGH();

    int32_t result = ((int32_t)rx_buffer[0] << 16) |
                     ((int32_t)rx_buffer[1] << 8)  |
                     rx_buffer[2];

    return (result & 0x800000) ? (result | 0xFF000000) : result;  // 符号扩展
}

// ============================
// 读取压力值函数
// ============================
float ADS1220_ReadPressure(void) {
    int32_t raw_data = ADS1220_ReadData();
    if (raw_data == -1) return -1.0f;  // 读取失败处理

    raw_data = -raw_data;  // 方向修正
    float V_in = (raw_data * VREF) / 8388607.0f;
    float V_load = V_in / GAIN;
    float weight = V_load / ((SENSITIVITY / 1000.0f) * EXCITATION_VOLTAGE);

    return weight * 1000.0f;  // 返回压力值 (g)
}

// ============================
// 丢弃无效数据 (启动后读取无效值清除)
// ============================
void Discard_dirty_data(void) {
    for (int i = 0; i < 5; i++) {
        ADS1220_ReadPressure();
        HAL_Delay(10);  // 间隔读取，避免过快
    }
}

// ============================
// 示例任务: 周期性读取压力
// ============================
void Start_SPI_Task(void const *argument) {
    for (;;) {
        float pressure = ADS1220_ReadPressure();
        if (pressure >= 0) {
            printf("压力: %.2f g\n", pressure);
        } else {
            printf("读取压力失败\n");
        }
        osDelay(500);  // 500ms间隔读取
    }
}