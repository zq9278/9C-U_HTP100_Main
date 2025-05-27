#include "main.h"
#include <stdint.h>
extern I2C_HandleTypeDef hi2c2;
extern uint8_t AD24C02_EYE[4];
volatile uint8_t i2c_dma_read_complete = 0;  // ����ɱ�־
volatile uint8_t i2c_dma_write_complete = 0; // д��ɱ�־
SoftwareI2C iic_24x = {EE_SDA_GPIO_Port, EE_SDA_Pin, EE_SCL_GPIO_Port, EE_SCL_Pin};
// ���IIC��ʼ��

void AT24CXX_Init(void) {
  // ��ʼ��IIC�ӿ�
  // IIC_Init(); // ���ʹ��Ӳ��IIC�����Ե��ô˺���

}

// ��ȡAT24CXX��һ���ֽ�����
// ReadAddr: Ҫ��ȡ�ĵ�ַ
// ����ֵ  : ��ȡ�����ֽ�����
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr) {
    taskENTER_CRITICAL();  // ��ֹ�����л�
  uint8_t temp = 0;
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0); // ����д����
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, ReadAddr >> 8); // ���͸ߵ�ַ�ֽ�
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((ReadAddr / 256) << 1)); // ����������ַ
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, ReadAddr % 256); // ���͵͵�ַ�ֽ�
  I2C_Wait_Ack(&iic_24x);
  I2C_Start(&iic_24x);
  I2C_Send_Byte(&iic_24x, 0xA1); // ���Ͷ�����
  I2C_Wait_Ack(&iic_24x);
  temp = I2C_Read_Byte(&iic_24x, 0);
  I2C_Stop(&iic_24x); // ֹͣIICͨ��
    taskEXIT_CRITICAL();   // �ָ������л�
  return temp;
}

// д��AT24CXXһ���ֽ�����
// WriteAddr  : д���ַ
// DataToWrite: Ҫд�������
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
    taskENTER_CRITICAL();  // ��ֹ�����л�
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0); // ����д����
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, WriteAddr >> 8); // ���͸ߵ�ַ�ֽ�
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((WriteAddr / 256) << 1)); // ����������ַ
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, WriteAddr % 256); // ���͵͵�ַ�ֽ�
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, DataToWrite); // д������
  I2C_Wait_Ack(&iic_24x);
  I2C_Stop(&iic_24x); // ֹͣIICͨ��
    taskEXIT_CRITICAL();   // �ָ������л�
  HAL_Delay(10); // д����ɺ���ʱ
}

// д��AT24CXX���ֽ�����
// WriteAddr  : д���ַ
// DataToWrite: Ҫд�������
// Len        : Ҫд����ֽ���
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
  uint8_t t;
  for (t = 0; t < Len; t++) {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
  }
}

// ��ȡAT24CXX���ֽ�����
// ReadAddr: ��ȡ��ʼ��ַ
// Len     : Ҫ��ȡ���ֽ���
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint8_t Len) {
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++) {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}

// ���AT24CXX�Ƿ���������
// ����ֵ:
// 1: ���ʧ��
// 0: ���ɹ�
uint8_t AT24CXX_Check(void) {
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(16383); // ���Զ�ȡAT24CXX���һ���ֽ�
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

// ��AT24CXX��ȡ����ֽ�
// ReadAddr  : ��ʼ��ַ
// pBuffer   : �洢��ȡ���ݵĻ�����
// NumToRead : Ҫ��ȡ���ֽ���
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
  while (NumToRead) {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}

// ��AT24CXXд�����ֽ�
// WriteAddr : ��ʼ��ַ
// pBuffer   : Ҫд������ݻ�����
// NumToWrite: Ҫд����ֽ���
void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
  while (NumToWrite--) {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}

// ��AT24C02�����е�ַд����ͬ��ֵ
void AT24C02_WriteAllBytes(uint8_t value) {
  for (uint16_t addr = 0; addr < 256; addr++) {
    AT24CXX_WriteOneByte(addr, value);
    HAL_Delay(5); // ÿ��д�����ʱ
  }
}
void AT24C02_WriteAllBytes_eye(uint8_t value) {
    static uint8_t read_buffer[256];  // ��ȡ���
    for (uint16_t addr = 0; addr < 256; addr++) {
        uint8_t data = value;  // �����Ҫ��ͬ�����ݣ����Ե����˴�
        HAL_I2C_Mem_Write(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
        HAL_Delay(5);  // �����ӳ٣�ȷ��д�����
    }

    // ʹ�� DMA ��ȡ��Ƭ EEPROM
    HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, read_buffer, 256);

    // �ȴ� DMA ��ȡ���
    while (!i2c_dma_read_complete)
    {
        osDelay(1);  // **�����ӳ٣�����ռ�� CPU**
    }

    //osDelay(1000);
    // ��ӡ EEPROM ��ȡ����
    LOG("EEPROM ��ȡ����:\n");
    for (uint16_t i = 2; i < 3; i++) {
        LOG("Addr: 0x%02X, Data: 0x%02X\n", i, read_buffer[i]);
    }
}
// ��AT24C02�����е�ַд����ͬ��ֵ
//    void AT24C02_WriteAllBytes_eye(uint8_t value) {
//    for (uint16_t addr = 0; addr < 256; addr++) {
//        HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, (uint8_t *) addr, I2C_MEMADD_SIZE_8BIT, &value, 1);
//        HAL_Delay(5); // ÿ��д�����ʱ
//    }
//}

// �� EEPROM ��ȡ uint32_t ���ݣ�����δ��ʼ��ʱд�� 0
uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr) {
  uint8_t buffer[2];

  // ��ȡ 4 �ֽ�����
  AT24CXX_Read(startAddr, buffer, 2);

  // ����Ƿ�δ��ʼ����4 �ֽڶ��� 0xFF��
  if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
    uint8_t zeroBuffer[2] = {0x00, 0x00}; // ׼��д��� 0 ����

    // δ��ʼ������ 0 д��
    AT24CXX_Write(startAddr, zeroBuffer, 2);

    // ����д���ֵ 0
    return 0;
  }
  // ����Ѿ���ʼ����ֱ��ƴ�ӷ��� uint32_t ֵ�����ģʽ��
  return (uint16_t)((buffer[0] << 8) | buffer[1]);
}

// �� uint16_t ����д�� AT24CXX ��ָ����ַ
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value) {
  uint8_t buffer[2];

  // �� uint16_t ���ݷ�Ϊ���ֽں͵��ֽ�
  buffer[0] = (uint8_t)(value >> 8);  // ���ֽ�
  buffer[1] = (uint8_t)(value & 0xFF);  // ���ֽ�

  // ���� AT24CXX_Write д�� 2 �ֽ�
  AT24CXX_Write(WriteAddr, buffer, 2);
}

void Heating_film_Check(void) {
  // AD24C02_EYE[0]���Ƿ����ù���1
  // AD24C02_EYE[1]���Ƿ����ù���2
  // AD24C02_EYE[2]���Ƿ����ù���3
  // AD24C02_EYE[3]���Ƿ����ù���4

  uint8_t a = 0xAA;
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x20, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[0], 1);
  if (AD24C02_EYE[0] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x20, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[0] == 0xAA) {
    // ���ù���1
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x21, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[1], 1);
  if (AD24C02_EYE[1] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x21, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[1] == 0xAA) {
    // ���ù���2
  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x22, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[2], 1);
  if (AD24C02_EYE[2] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x22, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[2] == 0xAA) {
    // ���ù���3
  }
}
extern char *i2c2_mutex_owner; // ��ǰ�������ĺ���/������
//uint16_t EYE_AT24CXX_Read(uint16_t startAddr) {
//    uint8_t buffer[2];
//    HAL_StatusTypeDef status;
//
//    // 1. ��ȡ I2C2 �Ļ���������ȴ� 100ms
//    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Read����ȡ I2C2 ������ʧ�ܣ�\n");
//        return 0xFFFF; // ���󷵻�
//    }
//    // 2. ���� I2C2 �� DMA ������
//    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, startAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    if (status != HAL_OK) {
//        LOG("EYE_AT24CXX_Read��DMA ����ʧ�ܣ�״̬�룺%d\n", status);
//        xSemaphoreGive(i2c2_mutex);
//        return 0xFFFF;
//    }
//    // 3. �ȴ� DMA ��ȡ��ɣ��ص����ͷ� xI2C2CompleteSem��
//    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Read��DMA ��ȡ��ʱ��\n");
//        xSemaphoreGive(i2c2_mutex);
//        return 0xFFFF;
//    }
//    // 4. �ͷŻ�����
//    xSemaphoreGive(i2c2_mutex);
//    // 5. �ϳɲ����ؽ��
//    return (uint16_t)((buffer[0] << 8) | buffer[1]);
//}
//HAL_StatusTypeDef EYE_AT24CXX_Write(uint16_t WriteAddr, uint16_t value) {
//    uint8_t buffer[2];
//    HAL_StatusTypeDef status;
//
//    buffer[0] = (uint8_t)(value >> 8);
//    buffer[1] = (uint8_t)(value & 0xFF);
//
//    // 1. ��ȡ I2C �����������õȴ���
//    if (xSemaphoreTake(i2c2_mutex, 100) != pdTRUE) {
//        LOG("��ȡ I2C2 ������ʧ�ܣ�\n");
//        return HAL_ERROR;
//    }
//
//    // 2. ��վɵ��ź���״̬���������
//    xSemaphoreTake(I2C2_DMA_Sem, 0);
//    // 3. �������� DMA
//    status = HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, WriteAddr, I2C_MEMADD_SIZE_8BIT, buffer, 2);
//    if (status != HAL_OK) {
//        LOG("DMA ����ʧ�ܣ��豸�����ѶϿ���״̬��: %d\n", status);
//        xSemaphoreGive(i2c2_mutex);
//        return HAL_ERROR;
//    }
//    // 4. �ȴ� DMA ����źţ�100ms��
//    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
//        LOG("EYE_AT24CXX_Write��DMA д�볬ʱ���豸�����Ѱγ���\n");
//        xSemaphoreGive(i2c2_mutex);
//        return HAL_TIMEOUT;
//    }
//    // 5. �ͷŻ�����
//    xSemaphoreGive(i2c2_mutex);
//    return HAL_OK;
//}
HAL_StatusTypeDef EYE_AT24CXX_WriteByte(uint16_t addr, uint8_t data)
{
    HAL_StatusTypeDef status;

    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return HAL_ERROR;
    xSemaphoreTake(I2C2_DMA_Sem, 0);

    status = HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
    if (status != HAL_OK || xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        xSemaphoreGive(i2c2_mutex);
        return HAL_ERROR;
    }

    xSemaphoreGive(i2c2_mutex);
    osDelay(5); // �ȴ�д�����
    return HAL_OK;
}

uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out)
{
    HAL_StatusTypeDef status;
    uint8_t data = 0;

    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        *status_out = HAL_ERROR;
        return 0x00;
    }

    xSemaphoreTake(I2C2_DMA_Sem, 0);
    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
    if (status != HAL_OK || xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        xSemaphoreGive(i2c2_mutex);
        *status_out = HAL_ERROR;
        return 0x00;
    }

    xSemaphoreGive(i2c2_mutex);
    *status_out = HAL_OK;
    return data;
}
HAL_StatusTypeDef EYE_AT24CXX_WriteUInt16(uint16_t addr, uint16_t value)
{
    HAL_StatusTypeDef status;

    status = EYE_AT24CXX_WriteByte(addr, value >> 8); // ���ֽ�
    if (status != HAL_OK) return status;

    status = EYE_AT24CXX_WriteByte(addr + 1, value & 0xFF); // ���ֽ�
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

  // ��ʼ��ʵ���ĳ�Ա����
  my_prepare_data.cmd_head_high = 0x6A;
  my_prepare_data.cmd_head_low = 0xA6;
  my_prepare_data.frame_length=0x0b;
  my_prepare_data.cmd_type_high = 0x00;
  my_prepare_data.end_high = 0xFF;
  my_prepare_data.end_low = 0xFF;

  hot_count = AT24CXX_ReadOrWriteZero(0x00);
  my_prepare_data.cmd_type_low = 0xA0;
  my_prepare_data.value = hot_count;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  crimp_count = AT24CXX_ReadOrWriteZero(0x02);
  my_prepare_data.cmd_type_low = 0xA1;
  my_prepare_data.value = crimp_count;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  auto_count = AT24CXX_ReadOrWriteZero(0x04);
  my_prepare_data.cmd_type_low = 0xA2;
  my_prepare_data.value = auto_count;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  set_prepare = AT24CXX_ReadOrWriteZero(0xFC);
  my_prepare_data.cmd_type_low = 0xA3;
  my_prepare_data.value = set_prepare;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
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
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  my_prepare_data.cmd_type_low = 0xA5;
  my_prepare_data.value = prepare_temperature;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  my_prepare_data.cmd_type_low = 0xA6;
  my_prepare_data.value = prepare_time;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
  bee = AT24CXX_ReadOrWriteZero(0xF8);
  my_prepare_data.cmd_type_low = 0xA7;
  my_prepare_data.value = bee;
  Eye_twitching_invalid_master(&my_prepare_data); // �����ݷ��͵�����
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