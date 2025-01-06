#include "main.h"
#include <stdint.h>
extern I2C_HandleTypeDef hi2c2;
extern uint8_t AD24C02_EYE[4];

SoftwareI2C iic_24x={EE_SDA_GPIO_Port, EE_SDA_Pin,EE_SCL_GPIO_Port,EE_SCL_Pin};
// ��ʼ��IIC�ӿ�
void AT24CXX_Init(void) {
  //IIC_Init(); // IIC��ʼ��
}
// ��AT24CXXָ����ַ����һ������
// ReadAddr:��ʼ�����ĵ�ַ
// ����ֵ  :����������
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr) {
  uint8_t temp = 0;
  //WP(0);
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x,0XA0); // ����д����
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x,ReadAddr >> 8); // ���͸ߵ�ַ
  } else
    I2C_Send_Byte(&iic_24x,0XA0 + ((ReadAddr / 256) << 1)); // ����������ַ0XA0,д����
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x,ReadAddr % 256); // ���͵͵�ַ
  I2C_Wait_Ack(&iic_24x);
  I2C_Start(&iic_24x);
  I2C_Send_Byte(&iic_24x,0XA1); // �������ģ�?
  I2C_Wait_Ack(&iic_24x);
  temp = I2C_Read_Byte(&iic_24x,0);
  I2C_Stop(&iic_24x); // ����һ��ֹͣ����
  //WP(1);
  return temp;
}
// ��AT24CXXָ����ַд��һ������
// WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
// DataToWrite:Ҫд�������?
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
  //WP(0);
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x,0XA0); // ����д����
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x,WriteAddr >> 8); // ���͸ߵ�ַ
  } else
    I2C_Send_Byte(&iic_24x,0XA0 + ((WriteAddr / 256) << 1)); // ����������ַ0XA0,д����
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x,WriteAddr % 256); // ���͵͵�ַ
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x,DataToWrite); // �����ֽ�
  I2C_Wait_Ack(&iic_24x);
  I2C_Stop(&iic_24x); // ����һ��ֹͣ����
  HAL_Delay(10);
 // WP(1);
}
// ��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
// �ú�������д��16bit����32bit������.
// WriteAddr  :��ʼд��ĵ��?
// DataToWrite:���������׵�ַ
// Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  uint8_t t;
  for (t = 0; t < Len; t++) {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xff);
  }
}

// ��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
// �ú������ڶ���16bit����32bit������.
// ReadAddr   :��ʼ�����ĵ�ַ
// ����ֵ     :����
// Len        :Ҫ�������ݵĳ���2,4
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++) {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}
// ���AT24CXX�Ƿ�����
// ��������24XX�����һ�����?(16383)���洢��־��.
// ���������?24Cϵ��,�����ַ�?�޸�
// ����1:���ʧ��?
// ����0:���ɹ�
uint8_t AT24CXX_Check(void) {
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(16383); // ����ÿ�ο�����дAT24CXX
  if (temp == 0X55)
    return 0;
  else // �ų���һ�γ�ʼ�������?
  {
    AT24CXX_WriteOneByte(16383, 0X55);
    temp = AT24CXX_ReadOneByte(16383);
    if (temp == 0X55)
      return 0;
  }
  return 1;
}

// ��AT24CXX�����ָ����ַ��ʼ����ָ������������?
// ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
// pBuffer  :���������׵�ַ
// NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  while (NumToRead) {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}
// ��AT24CXX�����ָ����ַ��ʼд��ָ������������?
// WriteAddr :��ʼд��ĵ��? ��24c02Ϊ0~255
// pBuffer   :���������׵�ַ
// NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  while (NumToWrite--) {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}

void Heating_film_Check(void) {
  // AD24C02_EYE[0]//加热状态下使用次数
  // AD24C02_EYE[1]//脉动状态下使用次数
  // AD24C02_EYE[2]//�?动状态下使用次数
  // AD24C02_EYE[3]//**状态下使用次数

  uint8_t a = 0xAA;
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x20, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[0], 1);
  if (AD24C02_EYE[0] == 0xff) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x20, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[0] == 0xAA) {
    // 眼遁加热状态下使用
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x21, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[1], 1);
  if (AD24C02_EYE[1] == 0xff) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x21, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[1] == 0xAA) {
    // 眼遁脉动状态下使用
  }
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x22, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[2], 1);
  if (AD24C02_EYE[2] == 0xff) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x22, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[2] == 0xAA) {
    // 眼遁�?动状态下使用
  }
}
