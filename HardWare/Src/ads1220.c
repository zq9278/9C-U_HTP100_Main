/*
 * @Author: zhangqi
 * @Date: 2024-12-30 17:40:27
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 16:37:52
 */

// SPI接口代码，用于与MCU通信的ADS1220
#include "main.h"
#include <stdint.h>



// void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
//   uint8_t cmd = 0x40 | (reg << 2); // WREG命令 + 地址
//   ADS1220_CS_LOW();
//   HAL_SPI_Transmit_IT(&hspi2, &cmd, 1);
//   HAL_SPI_Transmit_IT(&hspi2, &value, 1);
//   ADS1220_CS_HIGH();
// }
// void ADS1220_Init(void) {
//   // 上电后延时确保设备稳定
//   // 重置ADS1220
//   HAL_Delay(10);
//   ADS1220_CS_LOW();
//   HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)ADS1220_CMD_RESET, 1); // 发送重置命令
//   ADS1220_CS_HIGH();
//   HAL_Delay(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
//   // 配置ADS1220寄存器
//   //ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x38);//16
//   ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x36);//8
//   //ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x34);//4
//   ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94);
//   ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98);
//   ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00);
//   ADS1220_StartConversion();
// }
// void ADS1220_StartConversion(void) {
//   // 启动连续转换模式
//   ADS1220_CS_LOW();
//   HAL_SPI_Transmit(&hspi2, (uint8_t *)ADS1220_CMD_START_SYNC, 1,100);
//   ADS1220_CS_HIGH();
// }
// // void ADS1220_StopConversion(void) {
// //   // 停止转换并进入低功耗模式
// //   ADS1220_CS_LOW();
// //   HAL_SPI_Transmit(&hspi2, (uint8_t *)ADS1220_CMD_POWERDOWN, 1);
// //   ADS1220_CS_HIGH();
// // }

// int32_t ADS1220_ReadData(void) {
//   uint8_t data[3];
//   ADS1220_StartConversion();
//   ADS1220_CS_LOW();
//   HAL_Delay(1);
//   // 等待传输完成
//     // while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
//     //     // 可选择加入超时机制，避免死循环
//     // }
//   HAL_SPI_Transmit(&hspi2, (uint8_t *)ADS1220_CMD_RDATA, 1,100);                 // 发送读取数据命令
//   HAL_SPI_Transmit(&hspi2, data, 3,100); // 接收3字节的数据
//   ADS1220_CS_HIGH();

//   // 将字节组合成24位有符号整数
//   int32_t result = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
//   //printf("result: %d\n",result);
//   if (result & 0x800000) {
//     result |= 0xFF000000; // 负数符号扩展
//   }
//   return result; // 返回转换后的结果
// }








// 定义全局变量
volatile uint8_t spi_rx_buffer[3];
volatile int32_t ads1220_result = 0;
volatile uint8_t data_ready = 0; // 标志位

void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
  uint8_t cmd = 0x40 | (reg << 2); // WREG命令 + 地址
  ADS1220_CS_LOW();
  HAL_SPI_Transmit_IT(&hspi2, &cmd, 1); // 使用中断发送命令
  HAL_SPI_Transmit_IT(&hspi2, &value, 1); // 使用中断发送数据
  ADS1220_CS_HIGH();
}

void ADS1220_Init(void) {
  // 上电后延时确保设备稳定
  HAL_Delay(10);
  
  // 重置ADS1220
  ADS1220_CS_LOW();
  uint8_t reset_cmd = ADS1220_CMD_RESET;
  HAL_SPI_Transmit_IT(&hspi2, &reset_cmd, 1);
  ADS1220_CS_HIGH();
  HAL_Delay(10);

  // 配置ADS1220寄存器
  ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x3e); // 配置寄存器0
  ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94); // 配置寄存器1
  ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98); // 配置寄存器2
  ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00); // 配置寄存器3

  // 启动转换
  ADS1220_StartConversion();
}

void ADS1220_StartConversion(void) {
  // 启动连续转换模式
  ADS1220_CS_LOW();
  uint8_t start_cmd = ADS1220_CMD_START_SYNC;
  HAL_SPI_Transmit_IT(&hspi2, &start_cmd, 1); // 使用中断发送命令
  ADS1220_CS_HIGH();
}

int32_t ADS1220_ReadData(void) {
  uint8_t read_cmd = ADS1220_CMD_RDATA;
  ADS1220_CS_LOW();
  HAL_SPI_Transmit_IT(&hspi2, &read_cmd, 1); // 发送读取数据命令
  HAL_SPI_Receive_IT(&hspi2, spi_rx_buffer, 3); // 接收3字节数据
  ADS1220_CS_HIGH();
    // 等待中断完成
  while (!data_ready) {
    // 这里可以加入超时机制，防止无限循环
  }
  data_ready = 0; // 清除标志位
    // 解析数据
  int32_t result = ((int32_t)spi_rx_buffer[0] << 16) |
                   ((int32_t)spi_rx_buffer[1] << 8) |
                   spi_rx_buffer[2];
  if (result & 0x800000) {
    result |= 0xFF000000; // 负数符号扩展
  }
  return result; // 返回解析结果
}





float ADS1220_ReadPressure(void) {

  int32_t raw_data = ADS1220_ReadData(); // 获取原始数据
    raw_data = -raw_data; // 将负值变为正值
   // 1. 计算差分输入电压 V_in
    float V_in = (raw_data * VREF) / 8388607.0;
//printf("v_in: %d \n",raw_data);

//     // 2. 反推出传感器原始输出 V_LOAD
     float V_load = V_in / GAIN;
// printf("v_load: %f",V_load);

//     // 3. 根据传感器灵敏度计算重量
     float weight = V_load / (SENSITIVITY / 1000.0 * EXCITATION_VOLTAGE); // 注意灵敏度单位为 mV/V，需转换为 V/V

 //printf("weight0 %f, ",weight);

  //float max_voltage = SENSITIVITY * EXCITATION_VOLTAGE * GAIN; // 满量程放大后电压
    // float weight = (voltage1 / max_voltage) * MAX_WEIGHT;
    // printf("weight: %f\n",weight);
  return weight*1000;                                // 返回压力值
}
