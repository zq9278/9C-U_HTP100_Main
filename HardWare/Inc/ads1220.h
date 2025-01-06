/*
 * @Author: zhangqi 
 * @Date: 2024-12-30 17:40:27 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 17:01:42
 */
#ifndef __ADS1220_H
#define __ADS1220_H
#include "stm32g0xx_hal.h"
// 定义ADS1220的SPI命令
#define ADS1220_CMD_RESET 0x06      // 重置命令
#define ADS1220_CMD_START_SYNC 0x08 // 开始/同步命令
#define ADS1220_CMD_RDATA 0x10      // 读取数据命令
#define ADS1220_CMD_POWERDOWN 0x02  // 低功耗模式命令
#define ADS1220_CMD_RREG 0x23       // 读取寄存器命令

// 定义ADS1220的寄存器地址
#define ADS1220_REG_CONFIG0 0x00 // 配置寄存器0
#define ADS1220_REG_CONFIG1 0x01 // 配置寄存器1
#define ADS1220_REG_CONFIG2 0x02 // 配置寄存器2
#define ADS1220_REG_CONFIG3 0x03 // 配置寄存器3

// 定义传感器参数
#define SENSITIVITY 0.365           // 灵敏度 (mV/V)越小数值越大
#define MAX_WEIGHT 2.0            // 满量程重量 (kg)
#define EXCITATION_VOLTAGE 3.3    // 激励电压 (V)
#define GAIN 128                    // 放大倍数
#define VREF 3.3               // 基准电压
extern SPI_HandleTypeDef hspi2;

// SPI片选控制（与平台相关）
#define ADS1220_CS_LOW()                                                       \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define ADS1220_CS_HIGH()                                                      \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

void ADS1220_WriteRegister(uint8_t reg, uint8_t value);
void ADS1220_Init(void);
void ADS1220_StartConversion(void);
void ADS1220_StopConversion(void);
int32_t ADS1220_ReadData(void);
float ADS1220_ReadPressure(void);
#endif /* __ADS1220_H */

