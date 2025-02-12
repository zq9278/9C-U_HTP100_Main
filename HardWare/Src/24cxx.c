#include "main.h"
#include <stdint.h>
extern I2C_HandleTypeDef hi2c2;
extern uint8_t AD24C02_EYE[4];
extern osMessageQueueId_t prepare_dataHandle;
SoftwareI2C iic_24x = {EE_SDA_GPIO_Port, EE_SDA_Pin, EE_SCL_GPIO_Port, EE_SCL_Pin};
// 软件IIC初始化

void AT24CXX_Init(void) {
  // 初始化IIC接口
  // IIC_Init(); // 如果使用硬件IIC，可以调用此函数

}

// 读取AT24CXX的一个字节数据
// ReadAddr: 要读取的地址
// 返回值  : 读取到的字节数据
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr) {
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
  I2C_Send_Byte(&iic_24x, 0xA1); // 发送读命令
  I2C_Wait_Ack(&iic_24x);
  temp = I2C_Read_Byte(&iic_24x, 0);
  I2C_Stop(&iic_24x); // 停止IIC通信
  return temp;
}

// 写入AT24CXX一个字节数据
// WriteAddr  : 写入地址
// DataToWrite: 要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
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
  I2C_Stop(&iic_24x); // 停止IIC通信
  HAL_Delay(10); // 写入完成后延时
}

// 写入AT24CXX多字节数据
// WriteAddr  : 写入地址
// DataToWrite: 要写入的数据
// Len        : 要写入的字节数
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  uint8_t t;
  for (t = 0; t < Len; t++) {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
  }
}

// 读取AT24CXX多字节数据
// ReadAddr: 读取起始地址
// Len     : 要读取的字节数
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++) {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

// 检测AT24CXX是否正常工作
// 返回值:
// 1: 检测失败
// 0: 检测成功
uint8_t AT24CXX_Check(void) {
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(16383); // 测试读取AT24CXX最后一个字节
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
// ReadAddr  : 起始地址
// pBuffer   : 存储读取数据的缓冲区
// NumToRead : 要读取的字节数
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  while (NumToRead) {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}

// 向AT24CXX写入多个字节
// WriteAddr : 起始地址
// pBuffer   : 要写入的数据缓冲区
// NumToWrite: 要写入的字节数
void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  while (NumToWrite--) {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}

// 向AT24C02的所有地址写入相同的值
void AT24C02_WriteAllBytes(uint8_t value) {
  for (uint16_t addr = 0; addr < 256; addr++) {
    AT24CXX_WriteOneByte(addr, value);
    HAL_Delay(5); // 每次写入后延时
  }
}

// 从 EEPROM 读取 uint32_t 数据，并在未初始化时写入 0
uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr) {
  uint8_t buffer[2];

  // 读取 4 字节数据
  AT24CXX_Read(startAddr, buffer, 2);

  // 检查是否未初始化（4 字节都是 0xFF）
  if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
    uint8_t zeroBuffer[2] = {0x00, 0x00}; // 准备写入的 0 数据

    // 未初始化，将 0 写入
    AT24CXX_Write(startAddr, zeroBuffer, 2);

    // 返回写入的值 0
    return 0;
  }
  // 如果已经初始化，直接拼接返回 uint32_t 值（大端模式）
  return (uint16_t)((buffer[0] << 8) | buffer[1]);
}

// 将 uint16_t 数据写入 AT24CXX 的指定地址
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value) {
  uint8_t buffer[2];

  // 将 uint16_t 数据分为高字节和低字节
  buffer[0] = (uint8_t)(value >> 8);  // 高字节
  buffer[1] = (uint8_t)(value & 0xFF);  // 低字节

  // 调用 AT24CXX_Write 写入 2 字节
  AT24CXX_Write(WriteAddr, buffer, 2);
}

void Heating_film_Check(void) {
  // AD24C02_EYE[0]：是否启用功能1
  // AD24C02_EYE[1]：是否启用功能2
  // AD24C02_EYE[2]：是否启用功能3
  // AD24C02_EYE[3]：是否启用功能4

  uint8_t a = 0xAA;
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x20, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[0], 1);
  if (AD24C02_EYE[0] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x20, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[0] == 0xAA) {
    // 启用功能1
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x21, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[1], 1);
  if (AD24C02_EYE[1] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x21, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[1] == 0xAA) {
    // 启用功能2
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x22, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[2], 1);
  if (AD24C02_EYE[2] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x22, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[2] == 0xAA) {
    // 启用功能3
  }
}

prepare_data my_prepare_data;
void prepare_data_set(void){
  uint16_t hot_count,crimp_count,auto_count,prepare_press,prepare_temperature,prepare_time,bee,set_prepare;

  // 初始化实例的成员变量
  my_prepare_data.cmd_head_high = 0x6A;
  my_prepare_data.cmd_head_low = 0xA6;
  my_prepare_data.cmd_type_high = 0x00;
  my_prepare_data.end_high = 0xFF;
  my_prepare_data.end_low = 0xFF;

  hot_count = AT24CXX_ReadOrWriteZero(0x00);
  my_prepare_data.cmd_type_low = 0xA0;
  my_prepare_data.value = hot_count;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  crimp_count = AT24CXX_ReadOrWriteZero(0x02);
  my_prepare_data.cmd_type_low = 0xA1;
  my_prepare_data.value = crimp_count;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  auto_count = AT24CXX_ReadOrWriteZero(0x04);
  my_prepare_data.cmd_type_low = 0xA2;
  my_prepare_data.value = auto_count;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  set_prepare = AT24CXX_ReadOrWriteZero(0xFC);
  my_prepare_data.cmd_type_low = 0xA3;
  my_prepare_data.value = set_prepare;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
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
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  my_prepare_data.cmd_type_low = 0xA5;
  my_prepare_data.value = prepare_temperature;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  my_prepare_data.cmd_type_low = 0xA6;
  my_prepare_data.value = prepare_time;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
  bee = AT24CXX_ReadOrWriteZero(0xF8);
  my_prepare_data.cmd_type_low = 0xA7;
  my_prepare_data.value = bee;
  xQueueSend(prepare_dataHandle, &my_prepare_data, 0); // 将数据发送到队列
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