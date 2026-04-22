#include "iic.h"
#include "main.h"
#include "24cxx.h"
#include <stdint.h>
#include "UserApp.h"
#include "interface_uart.h"
extern I2C_HandleTypeDef hi2c2;
extern uint8_t AD24C02_EYE[4];
volatile uint8_t i2c_dma_read_complete = 0;  // 读完成标�?
volatile uint8_t i2c_dma_write_complete = 0; // 写完成标�?
SoftwareI2C iic_24x = {EE_SDA_GPIO_Port, EE_SDA_Pin, EE_SCL_GPIO_Port, EE_SCL_Pin};
// �?件IIC初�?�化

void AT24CXX_Init(void) {
  // 初�?�化IIC接口
  // IIC_Init(); // 如果使用�?件IIC，可以调用�?�函�?

}

// 读取AT24CXX的一�?字节数据
// ReadAddr: 要�?�取的地址
// 返回�?  : 读取到的字节数据
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr) {
    taskENTER_CRITICAL();  // 禁�??任务切换
  uint8_t temp = 0;
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0); // 发送写命令
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, ReadAddr >> 8); // 发送高地址字节
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((ReadAddr / 256) << 1)); // 发送器件地址
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, ReadAddr % 256); // 发送低地址字节
  I2C_Wait_Ack(&iic_24x);
  I2C_Start(&iic_24x);
  I2C_Send_Byte(&iic_24x, 0xA1); // 发送�?�命�?
  I2C_Wait_Ack(&iic_24x);
  temp = I2C_Read_Byte(&iic_24x, 0);
  I2C_Stop(&iic_24x); // 停�??IIC通信
    taskEXIT_CRITICAL();   // 恢�?�任务切�?
  return temp;
}

// 写入AT24CXX一�?字节数据
// WriteAddr  : 写入地址
// DataToWrite: 要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
    taskENTER_CRITICAL();  // 禁�??任务切换
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0); // 发送写命令
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, WriteAddr >> 8); // 发送高地址字节
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((WriteAddr / 256) << 1)); // 发送器件地址
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, WriteAddr % 256); // 发送低地址字节
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, DataToWrite); // 写入数据
  I2C_Wait_Ack(&iic_24x);
  I2C_Stop(&iic_24x); // 停�??IIC通信
    taskEXIT_CRITICAL();   // 恢�?�任务切�?
  HAL_Delay(10); // 写入完成后延�?
}

// 写入AT24CXX多字节数�?
// WriteAddr  : 写入地址
// DataToWrite: 要写入的数据
// Len        : 要写入的字节�?
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  uint8_t t;
  for (t = 0; t < Len; t++) {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
  }
}

// 读取AT24CXX多字节数�?
// ReadAddr: 读取起�?�地址
// Len     : 要�?�取的字节数
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++) {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

// 检测AT24CXX�?否�?�常工作
// 返回�?:
// 1: 检测失�?
// 0: 检测成�?
uint8_t AT24CXX_Check(void) {
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(16383); // 测试读取AT24CXX最后一�?字节
  if (temp == 0x55)
    return 0;
  else {
    AT24CXX_WriteOneByte(16383, 0x55);
    temp = AT24CXX_ReadOneByte(16383);
    if (temp == 0x55)
      return 0;
  }
  return 1;
}

// 从AT24CXX读取多个字节
// ReadAddr  : 起�?�地址
// pBuffer   : 存储读取数据的缓冲区
// NumToRead : 要�?�取的字节数
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  while (NumToRead) {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}

// 向AT24CXX写入多个字节
// WriteAddr : 起�?�地址
// pBuffer   : 要写入的数据缓冲�?
// NumToWrite: 要写入的字节�?
void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  while (NumToWrite--) {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}

// 向AT24C02的所有地址写入相同的�?
void AT24C02_WriteAllBytes(uint8_t value) {
  for (uint16_t addr = 0; addr < 256; addr++) {
    AT24CXX_WriteOneByte(addr, value);
    HAL_Delay(5); // 每�?�写入后延时
  }
}
//void AT24C02_WriteAllBytes_eye(uint8_t value) {
//    static uint8_t read_buffer[256];  // 读取结果
//    for (uint16_t addr = 0; addr < 256; addr++) {
//        uint8_t data = value;  // 如果需要不同的数据，可以调整�?��??
//        HAL_I2C_Mem_Write(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
//        HAL_Delay(5);  // 增加延迟，确保写入完�?
//    }
//
//    // 使用 DMA 读取整片 EEPROM
//    HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, read_buffer, 256);
//
//    // 等待 DMA 读取完成
//    while (!i2c_dma_read_complete)
//    {
//        osDelay(1);  // **�?暂延迟，避免占用 CPU**
//    }
//
//    //osDelay(1000);
//    // 打印 EEPROM 读取数据
//    LOG("EEPROM 读取数据:\n");
//    for (uint16_t i = 2; i < 3; i++) {
//        LOG("Addr: 0x%02X, Data: 0x%02X\n", i, read_buffer[i]);
//    }
//}



 void AT24C02_WriteAllBytes_eye(uint8_t value) {
    static uint8_t read_buffer[256];

    // 写入所有字节（使用�?询写入）
    for (uint16_t addr = 0; addr < 256; addr++) {
        uint8_t data = value;
        if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
            HAL_I2C_Mem_Write(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
            xSemaphoreGive(i2c2_mutex);
            osDelay(5);  // 写完等待 EEPROM 内部写周期完�?
        } else {
            LOG("写入锁超�? at addr 0x%02X\n", addr);
        }
    }

// 读取整块 EEPROM（使�? DMA + 信号量）
     if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
         if (HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, read_buffer, 256) == HAL_OK) {
             if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(300)) == pdTRUE) {
                 LOG("EEPROM DMA读取成功\n");
             } else {
                 LOG("EEPROM DMA读取超时\n");
             }
         } else {
             LOG("EEPROM DMA读取�?动失�?\n");
         }

         xSemaphoreGive(i2c2_mutex);
     } else {
         LOG("读取锁超时，无法读取EEPROM\n");
     }


    // 验证读取内�?�（�?打印部分�?
    LOG("EEPROM 读取数据:\n");
    for (uint16_t i = 1; i < 255; i++) {
        LOG("Addr: 0x%02X, Data: 0x%02X\n", i, read_buffer[i]);
    }
}




// 向AT24C02的所有地址写入相同的�?
//    void AT24C02_WriteAllBytes_eye(uint8_t value) {
//    for (uint16_t addr = 0; addr < 256; addr++) {
//        HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, (uint8_t *) addr, I2C_MEMADD_SIZE_8BIT, &value, 1);
//        HAL_Delay(5); // 每�?�写入后延时
//    }
//}

// �? EEPROM 读取 uint32_t 数据，并在未初�?�化时写�? 0
uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr) {
  uint8_t buffer[2];

  // 读取 4 字节数据
  AT24CXX_Read(startAddr, buffer, 2);

  // 检查是否未初�?�化�?4 字节都是 0xFF�?
  if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
    uint8_t zeroBuffer[2] = {0x00, 0x00}; // 准�?�写入的 0 数据

    // �?初�?�化，将 0 写入
    AT24CXX_Write(startAddr, zeroBuffer, 2);

    // 返回写入的�? 0
    return 0;
  }
  // 如果已经初�?�化，直接拼接返�? uint32_t 值（大�??模式�?
  return (uint16_t)((buffer[0] << 8) | buffer[1]);
}

// �? uint16_t 数据写入 AT24CXX 的指定地址
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value) {
  uint8_t buffer[2];

  // �? uint16_t 数据分为高字节和低字�?
  buffer[0] = (uint8_t)(value >> 8);  // 高字�?
  buffer[1] = (uint8_t)(value & 0xFF);  // 低字�?

  // 调用 AT24CXX_Write 写入 2 字节
  AT24CXX_Write(WriteAddr, buffer, 2);
}

void Heating_film_Check(void) {
  // AD24C02_EYE[0]：是否启用功�?1
  // AD24C02_EYE[1]：是否启用功�?2
  // AD24C02_EYE[2]：是否启用功�?3
  // AD24C02_EYE[3]：是否启用功�?4

  uint8_t a = 0xAA;
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x20, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[0], 1);
  if (AD24C02_EYE[0] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x20, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[0] == 0xAA) {
    // �?用功�?1
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x21, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[1], 1);
  if (AD24C02_EYE[1] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x21, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[1] == 0xAA) {
    // �?用功�?2
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x22, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[2], 1);
  if (AD24C02_EYE[2] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x22, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[2] == 0xAA) {
    // �?用功�?3
  }
}
extern char *i2c2_mutex_owner; // 当前持有锁的函数/任务�?
//uint16_t EYE_AT24CXX_Read(uint16_t startAddr) {
//    uint8_t buffer[2];
//    HAL_StatusTypeDef status;
//
//    // 1. 获取 I2C2 的互斥锁，最长等�? 100ms
//    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Read：获�? I2C2 互斥锁失败！\n");
//        return 0xFFFF; // 错�??返回
//    }
//    // 2. �?�? I2C2 �? DMA 读操�?
//    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, startAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    if (status != HAL_OK) {
//        LOG("EYE_AT24CXX_Read：DMA �?动失败，状态码�?%d\n", status);
//        xSemaphoreGive(i2c2_mutex);
//        return 0xFFFF;
//    }
//    // 3. 等待 DMA 读取完成（回调中释放 xI2C2CompleteSem�?
//    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Read：DMA 读取超时！\n");
//        xSemaphoreGive(i2c2_mutex);
//        return 0xFFFF;
//    }
//    // 4. 释放互斥�?
//    xSemaphoreGive(i2c2_mutex);
//    // 5. 合成并返回结�?
//    return (uint16_t)((buffer[0] << 8) | buffer[1]);
//}
//HAL_StatusTypeDef EYE_AT24CXX_Write(uint16_t WriteAddr, uint16_t value) {
//    uint8_t buffer[2];
//    HAL_StatusTypeDef status;
//
//    buffer[0] = (uint8_t)(value >> 8);
//    buffer[1] = (uint8_t)(value & 0xFF);
//
//    // 1. 获取 I2C 互斥锁（永久等待�?
//    if (xSemaphoreTake(i2c2_mutex, 100) != pdTRUE) {
//        LOG("获取 I2C2 互斥锁失败！\n");
//        return HAL_ERROR;
//    }
//
//    // 2. 清空旧的信号量状态，避免残留
//    xSemaphoreTake(I2C2_DMA_Sem, 0);
//    // 3. 尝试�?�? DMA
//    status = HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, WriteAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    if (status != HAL_OK) {
//        LOG("DMA �?动失败！设�?�可能已�?开？状态码: %d\n", status);
//        xSemaphoreGive(i2c2_mutex);
//        return HAL_ERROR;
//    }
//    // 4. 等待 DMA 完成信号�?100ms�?
//    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Write：DMA 写入超时！�?��?�可能已拔出！\n");
//        xSemaphoreGive(i2c2_mutex);
//        return HAL_TIMEOUT;
//    }
//    // 5. 释放互斥�?
//    xSemaphoreGive(i2c2_mutex);
//    return HAL_OK;
//}
HAL_StatusTypeDef EYE_AT24CXX_WriteByte(uint16_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;

    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return HAL_ERROR;
    xSemaphoreTake(I2C2_DMA_Sem, 0);

    status = HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
    if (status != HAL_OK || xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(200)) != pdTRUE) {
        xSemaphoreGive(i2c2_mutex);
        return HAL_ERROR;
    }

    xSemaphoreGive(i2c2_mutex);
    osDelay(5); // 等待写入完成
    return HAL_OK;
}

//uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out)
//{
//    HAL_StatusTypeDef status;
//    uint8_t data = 0;
//
//    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
//        *status_out = HAL_ERROR;
//        return 0x00;
//    }
//
//    xSemaphoreTake(I2C2_DMA_Sem, 0);
//    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
//    if (status != HAL_OK || xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//        xSemaphoreGive(i2c2_mutex);
//        *status_out = HAL_ERROR;
//        return 0x00;
//    }
//
//    xSemaphoreGive(i2c2_mutex);
//    *status_out = HAL_OK;
//    return data;
//}
uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out)
{
    HAL_StatusTypeDef status;
    uint8_t data = 0;
    int retry = 1; // 重试次数

    for (int i = 0; i < retry; ++i) {
        if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            status = HAL_ERROR;
            LOG("[EYE_AT24CXX_ReadByte] �?%d次重�?: 获取i2c2_mutex失败\n", i+1);
        } else {
            xSemaphoreTake(I2C2_DMA_Sem, 0);
            status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
            if (status != HAL_OK) {
                LOG("[EYE_AT24CXX_ReadByte] �?%d次重�?: HAL_I2C_Mem_Read_DMA失败, status=%d\n", i+1, status);
            }
            if (status == HAL_OK && xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) == pdTRUE) {
                xSemaphoreGive(i2c2_mutex);
                *status_out = HAL_OK;
                return data;
            } else {
                LOG("[EYE_AT24CXX_ReadByte] �?%d次重�?: DMA信号量超时或I2C错�?�\n", i+1);
            }
            xSemaphoreGive(i2c2_mutex);
        }
        // 失败后延时再�?
        osDelay(5);
    }
    LOG("[EYE_AT24CXX_ReadByte] 所有重试失�?, addr=0x%X\n", addr);
    *status_out = HAL_ERROR;
    return 0x00;
}
HAL_StatusTypeDef EYE_AT24CXX_WriteUInt16(uint16_t addr, uint16_t value)
{
    HAL_StatusTypeDef status;

    status = EYE_AT24CXX_WriteByte(addr, value >> 8); // 高字�?
    if (status != HAL_OK) return status;

    status = EYE_AT24CXX_WriteByte(addr + 1, value & 0xFF); // 低字�?
    return status;
}

uint16_t EYE_AT24CXX_ReadUInt16(uint16_t addr)
{
    HAL_StatusTypeDef dummy;
    uint8_t high = EYE_AT24CXX_ReadByte(addr, &dummy);
    uint8_t low  = EYE_AT24CXX_ReadByte(addr + 1, &dummy);
    return ((uint16_t)high << 8) | low;
}




prepare_data my_prepare_data;
void prepare_data_set(void){
  uint16_t hot_count,crimp_count,auto_count,prepare_press,prepare_temperature,prepare_time,bee,set_prepare;

  // 初�?�化实例的成员变�?
  my_prepare_data.cmd_head_high = 0x6A;
  my_prepare_data.cmd_head_low = 0xA6;
  my_prepare_data.frame_length=0x0b;
  my_prepare_data.cmd_type_high = 0x00;
  my_prepare_data.end_high = 0xFF;
  my_prepare_data.end_low = 0xFF;

  hot_count = AT24CXX_ReadOrWriteZero(0x00);
  my_prepare_data.cmd_type_low = 0xA0;
  my_prepare_data.value = hot_count;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  crimp_count = AT24CXX_ReadOrWriteZero(0x02);
  my_prepare_data.cmd_type_low = 0xA1;
  my_prepare_data.value = crimp_count;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  auto_count = AT24CXX_ReadOrWriteZero(0x04);
  my_prepare_data.cmd_type_low = 0xA2;
  my_prepare_data.value = auto_count;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  set_prepare = AT24CXX_ReadOrWriteZero(0xFC);
  set_prepare = 0;
  my_prepare_data.cmd_type_low = 0xA3;
  my_prepare_data.value = set_prepare;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  switch (set_prepare) {
  case 1:
    prepare_press = AT24CXX_ReadOrWriteZero(0x08);
    prepare_temperature = AT24CXX_ReadOrWriteZero(0x0A);
    prepare_time = AT24CXX_ReadOrWriteZero(0x0C);
    break;
  case 2:
    prepare_press = AT24CXX_ReadOrWriteZero(0x10);
    prepare_temperature = AT24CXX_ReadOrWriteZero(0x12);
    prepare_time = AT24CXX_ReadOrWriteZero(0x14);
    break;
  case 3:
    prepare_press = AT24CXX_ReadOrWriteZero(0x18);
    prepare_temperature = AT24CXX_ReadOrWriteZero(0x1A);
    prepare_time = AT24CXX_ReadOrWriteZero(0x1C);
    break;
  }
  my_prepare_data.cmd_type_low = 0xA4;
  my_prepare_data.value = prepare_press;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  my_prepare_data.cmd_type_low = 0xA5;
  my_prepare_data.value = prepare_temperature;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  my_prepare_data.cmd_type_low = 0xA6;
  my_prepare_data.value = prepare_time;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
  bee = AT24CXX_ReadOrWriteZero(0xF8);
  my_prepare_data.cmd_type_low = 0xA7;
  my_prepare_data.value = bee;
  Eye_twitching_invalid_master(&my_prepare_data); // 将数�?发送到队列
}
void AD24C01_Factory_formatted(void){
  if(AT24CXX_ReadOrWriteZero(0xf0)==0){
    AT24CXX_WriteUInt16(0xf0,1);
    AT24CXX_WriteUInt16(0x08,42);
    AT24CXX_WriteUInt16(0x10,42);
    AT24CXX_WriteUInt16(0x18,42);

    AT24CXX_WriteUInt16(0x0A,250);
    AT24CXX_WriteUInt16(0x12,350);
    AT24CXX_WriteUInt16(0x1A,450);

    AT24CXX_WriteUInt16(0x0c,2);
    AT24CXX_WriteUInt16(0x14,3);
    AT24CXX_WriteUInt16(0x1c,4);

  AT24CXX_WriteUInt16(0xf8,1);

  AT24CXX_WriteUInt16(0x00,0);
  AT24CXX_WriteUInt16(0x02,0);
  AT24CXX_WriteUInt16(0x04,0);
  };
}