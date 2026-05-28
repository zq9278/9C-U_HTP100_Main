/*
 * 鏂囦欢: 24cxx.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "iic.h"
#include "main.h"
#include "24cxx.h"
#include <stdint.h>
#include "UserApp.h"
#include "interface_uart.h"
extern I2C_HandleTypeDef hi2c2;
extern uint8_t AD24C02_EYE[4];
volatile uint8_t i2c_dma_read_complete = 0;
volatile uint8_t i2c_dma_write_complete = 0;
SoftwareI2C iic_24x = {EE_SDA_GPIO_Port, EE_SDA_Pin, EE_SCL_GPIO_Port, EE_SCL_Pin};

#ifndef DEFAULT_LANGUAGE_MODE
#define DEFAULT_LANGUAGE_MODE LANGUAGE_CHINESE
#endif

static uint16_t g_system_language = DEFAULT_LANGUAGE_MODE;


/**
 * @brief AT24CXX_Init 鍑芥暟瀹炵幇銆? */
void AT24CXX_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */



}




/**
 * @brief AT24CXX_ReadOneByte 鍑芥暟瀹炵幇銆? * @param ReadAddr 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    taskENTER_CRITICAL();
  uint8_t temp = 0;
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0);
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, ReadAddr >> 8);
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((ReadAddr / 256) << 1));
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, ReadAddr % 256);
  I2C_Wait_Ack(&iic_24x);
  I2C_Start(&iic_24x);
  I2C_Send_Byte(&iic_24x, 0xA1);
  I2C_Wait_Ack(&iic_24x);
  temp = I2C_Read_Byte(&iic_24x, 0);
  I2C_Stop(&iic_24x);
    taskEXIT_CRITICAL();
  return temp;
}




/**
 * @brief AT24CXX_WriteOneByte 鍑芥暟瀹炵幇銆? * @param WriteAddr 鍙傛暟銆? * @param DataToWrite 鍙傛暟銆? */
void AT24CXX_WriteOneByte(uint16_t WriteAddr, uint8_t DataToWrite) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    taskENTER_CRITICAL();
  I2C_Start(&iic_24x);
  if (EE_TYPE > AT24C16) {
    I2C_Send_Byte(&iic_24x, 0xA0);
    I2C_Wait_Ack(&iic_24x);
    I2C_Send_Byte(&iic_24x, WriteAddr >> 8);
  } else {
    I2C_Send_Byte(&iic_24x, 0xA0 + ((WriteAddr / 256) << 1));
  }
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, WriteAddr % 256);
  I2C_Wait_Ack(&iic_24x);
  I2C_Send_Byte(&iic_24x, DataToWrite);
  I2C_Wait_Ack(&iic_24x);
  I2C_Stop(&iic_24x);
    taskEXIT_CRITICAL();
  HAL_Delay(10);
}





/**
 * @brief AT24CXX_WriteLenByte 鍑芥暟瀹炵幇銆? * @param WriteAddr 鍙傛暟銆? * @param DataToWrite 鍙傛暟銆? * @param Len 鍙傛暟銆? */
void AT24CXX_WriteLenByte(uint16_t WriteAddr, uint32_t DataToWrite, uint8_t Len) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t t;
  for (t = 0; t < Len; t++) {
    AT24CXX_WriteOneByte(WriteAddr + t, (DataToWrite >> (8 * t)) & 0xFF);
  }
}




/**
 * @brief AT24CXX_ReadLenByte 鍑芥暟瀹炵幇銆? * @param ReadAddr 鍙傛暟銆? * @param Len 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr, uint8_t Len) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t t;
  uint32_t temp = 0;
  for (t = 0; t < Len; t++) {
    temp <<= 8;
    temp += AT24CXX_ReadOneByte(ReadAddr + Len - t - 1);
  }
  return temp;
}





/**
 * @brief AT24CXX_Check 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t AT24CXX_Check(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t temp;
  temp = AT24CXX_ReadOneByte(16383);
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





/**
 * @brief AT24CXX_Read 鍑芥暟瀹炵幇銆? * @param ReadAddr 鍙傛暟銆? * @param pBuffer 鍙傛暟銆? * @param NumToRead 鍙傛暟銆? */
void AT24CXX_Read(uint16_t ReadAddr, uint8_t *pBuffer, uint16_t NumToRead) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  while (NumToRead) {
    *pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
    NumToRead--;
  }
}





/**
 * @brief AT24CXX_Write 鍑芥暟瀹炵幇銆? * @param WriteAddr 鍙傛暟銆? * @param pBuffer 鍙傛暟銆? * @param NumToWrite 鍙傛暟銆? */
void AT24CXX_Write(uint16_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  while (NumToWrite--) {
    AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
    WriteAddr++;
    pBuffer++;
  }
}


/**
 * @brief AT24C02_WriteAllBytes 鍑芥暟瀹炵幇銆? * @param value 鍙傛暟銆? */
void AT24C02_WriteAllBytes(uint8_t value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  for (uint16_t addr = 0; addr < 256; addr++) {
    AT24CXX_WriteOneByte(addr, value);
    HAL_Delay(5);
  }
}


























/**
 * @brief AT24C02_WriteAllBytes_eye 鍑芥暟瀹炵幇銆? * @param value 鍙傛暟銆? */
 void AT24C02_WriteAllBytes_eye(uint8_t value) {
     /* 步骤说明：
      * 1) 处理输入参数与前置条件。
      * 2) 执行本函数核心业务逻辑。
      * 3) 输出结果/更新状态并返回。
      */
    static uint8_t read_buffer[256];


    for (uint16_t addr = 0; addr < 256; addr++) {
        uint8_t data = value;
        if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
            HAL_I2C_Mem_Write(&hi2c2, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
            xSemaphoreGive(i2c2_mutex);
            osDelay(5);
        } else {
            LOGE("[EEPROM] Write lock timeout, addr=0x%02X\n", addr);
        }
    }


     if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) == pdTRUE) {
         if (HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA0, 0x00, I2C_MEMADD_SIZE_8BIT, read_buffer, 256) == HAL_OK) {
             if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(300)) == pdTRUE) {
                 LOGI("[EEPROM] Event\n");
             } else {
                 LOGE("[EEPROM] Event\n");
             }
         } else {
             LOGE("[EEPROM] Event\n");
         }

         xSemaphoreGive(i2c2_mutex);
     } else {
         LOGE("[EEPROM] Event\n");
     }



    LOGI("[EEPROM] Event\n");
    for (uint16_t i = 1; i < 255; i++) {
        LOGI("[EEPROM] Addr=0x%02X, data=0x%02X\n", i, read_buffer[i]);
    }
}













/**
 * @brief AT24CXX_ReadOrWriteZero 鍑芥暟瀹炵幇銆? * @param startAddr 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint32_t AT24CXX_ReadOrWriteZero(uint16_t startAddr) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t buffer[2];


  AT24CXX_Read(startAddr, buffer, 2);


  if (buffer[0] == 0xFF && buffer[1] == 0xFF) {
    uint8_t zeroBuffer[2] = {0x00, 0x00};


    AT24CXX_Write(startAddr, zeroBuffer, 2);


    return 0;
  }
  return (uint16_t)((buffer[0] << 8) | buffer[1]);
}


/**
 * @brief AT24CXX_WriteUInt16 鍑芥暟瀹炵幇銆? * @param WriteAddr 鍙傛暟銆? * @param value 鍙傛暟銆? */
void AT24CXX_WriteUInt16(uint16_t WriteAddr, uint16_t value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint8_t buffer[2];


  buffer[0] = (uint8_t)(value >> 8);
  buffer[1] = (uint8_t)(value & 0xFF);


  AT24CXX_Write(WriteAddr, buffer, 2);
}

uint16_t SystemLanguage_Load(void)
{
  uint16_t language = (uint16_t)AT24CXX_ReadOrWriteZero(EEPROM_LANGUAGE_ADDR);

  if (language > LANGUAGE_ENGLISH) {
    language = DEFAULT_LANGUAGE_MODE;
    AT24CXX_WriteUInt16(EEPROM_LANGUAGE_ADDR, language);
  }

  g_system_language = language;
  return g_system_language;
}

bool SystemLanguage_Set(uint16_t language)
{
  if (language > LANGUAGE_ENGLISH) {
    return false;
  }

  g_system_language = language;
  AT24CXX_WriteUInt16(EEPROM_LANGUAGE_ADDR, language);
  return true;
}

uint16_t SystemLanguage_Get(void)
{
  return g_system_language;
}

/**
 * @brief Heating_film_Check 鍑芥暟瀹炵幇銆? */
void Heating_film_Check(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */





  uint8_t a = 0xAA;
  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x20, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[0], 1);
  if (AD24C02_EYE[0] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x20, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[0] == 0xAA) {

  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x21, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[1], 1);
  if (AD24C02_EYE[1] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x21, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[1] == 0xAA) {

  }

  HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, 0x22, I2C_MEMADD_SIZE_8BIT,
                       (uint8_t *)&AD24C02_EYE[2], 1);
  if (AD24C02_EYE[2] == 0xFF) {
    HAL_I2C_Mem_Write_DMA(&hi2c2, 0xA0, 0x22, I2C_MEMADD_SIZE_8BIT, &a, 1);
  } else if (AD24C02_EYE[2] == 0xAA) {

  }
}
extern char *i2c2_mutex_owner;


























































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
    osDelay(5);
    return HAL_OK;
}























uint8_t EYE_AT24CXX_ReadByte(uint16_t addr, HAL_StatusTypeDef* status_out)
{
    HAL_StatusTypeDef status;
    uint8_t data = 0;
    int retry = 3;

    for (int i = 0; i < retry; ++i) {
        if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(200)) != pdTRUE) {
            status = HAL_ERROR;
            LOGE("[EEPROM] Read byte retry %d: mutex timeout\n", i+1);
        } else {
            xSemaphoreTake(I2C2_DMA_Sem, 0);
            status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0xA1, addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
            if (status != HAL_OK) {
                LOGE("[EEPROM] Read byte retry %d: DMA start failed, status=%d\n", i+1, status);
            }
            if (status == HAL_OK && xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(200)) == pdTRUE) {
                xSemaphoreGive(i2c2_mutex);
                *status_out = HAL_OK;
                return data;
            } else {
                LOGE("[EEPROM] Read byte retry %d: DMA timeout or I2C error\n", i+1);
            }
            xSemaphoreGive(i2c2_mutex);
        }

        osDelay(5);
    }
    LOGE("[EEPROM] All read byte retries failed, addr=0x%X\n", addr);
    *status_out = HAL_ERROR;
    return 0x00;
}
HAL_StatusTypeDef EYE_AT24CXX_WriteUInt16(uint16_t addr, uint16_t value)
{
    HAL_StatusTypeDef status;

    status = EYE_AT24CXX_WriteByte(addr, value >> 8);
    if (status != HAL_OK) return status;

    status = EYE_AT24CXX_WriteByte(addr + 1, value & 0xFF);
    return status;
}

HAL_StatusTypeDef EYE_AT24CXX_ReadUInt16Ex(uint16_t addr, uint16_t *value_out)
{
    HAL_StatusTypeDef status_high = HAL_ERROR;
    HAL_StatusTypeDef status_low = HAL_ERROR;
    uint8_t high = EYE_AT24CXX_ReadByte(addr, &status_high);
    uint8_t low  = EYE_AT24CXX_ReadByte(addr + 1, &status_low);

    if ((status_high != HAL_OK) || (status_low != HAL_OK)) {
        if (value_out != NULL) {
            *value_out = 0;
        }
        return HAL_ERROR;
    }

    if (value_out != NULL) {
        *value_out = ((uint16_t)high << 8) | low;
    }

    return HAL_OK;
}

uint16_t EYE_AT24CXX_ReadUInt16(uint16_t addr)
{
    uint16_t value = 0xFFFF;
    (void)EYE_AT24CXX_ReadUInt16Ex(addr, &value);
    return value;
}


prepare_data my_prepare_data;
/**
 * @brief prepare_data_set 鍑芥暟瀹炵幇銆? */
void prepare_data_set(void){
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
  uint16_t hot_count,crimp_count,auto_count,prepare_press,prepare_temperature,prepare_time,bee,set_prepare;
  uint16_t product_model_value;


  my_prepare_data.cmd_head_high = 0x6A;
  my_prepare_data.cmd_head_low = 0xA6;
  my_prepare_data.frame_length=0x0b;
  my_prepare_data.cmd_type_high = 0x00;
  my_prepare_data.end_high = 0xFF;
  my_prepare_data.end_low = 0xFF;

  hot_count = AT24CXX_ReadOrWriteZero(0x00);
  my_prepare_data.cmd_type_low = 0xA0;
  my_prepare_data.value = hot_count;
  Eye_twitching_invalid_master(&my_prepare_data);
  crimp_count = AT24CXX_ReadOrWriteZero(0x02);
  my_prepare_data.cmd_type_low = 0xA1;
  my_prepare_data.value = crimp_count;
  Eye_twitching_invalid_master(&my_prepare_data);
  auto_count = AT24CXX_ReadOrWriteZero(0x04);
  my_prepare_data.cmd_type_low = 0xA2;
  my_prepare_data.value = auto_count;
  Eye_twitching_invalid_master(&my_prepare_data);
  set_prepare = AT24CXX_ReadOrWriteZero(0xFC);

#if defined(PRODUCT_MODEL_9C_U_HTP100S)
  set_prepare = 0;
#endif
  my_prepare_data.cmd_type_low = 0xA3;
  my_prepare_data.value = set_prepare;
  Eye_twitching_invalid_master(&my_prepare_data);

#if defined(PRODUCT_MODEL_9C_U_HTP100S)
  product_model_value = 0;
#elif defined(PRODUCT_MODEL_9C_U_HTP100)
  product_model_value = 1;
#else
  product_model_value = 0xFFFF;
#endif
  my_prepare_data.cmd_type_low = 0xAC;
  my_prepare_data.value = product_model_value;
  Eye_twitching_invalid_master(&my_prepare_data);

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
  Eye_twitching_invalid_master(&my_prepare_data);
  my_prepare_data.cmd_type_low = 0xA5;
  my_prepare_data.value = prepare_temperature;
  Eye_twitching_invalid_master(&my_prepare_data);
  my_prepare_data.cmd_type_low = 0xA6;
  my_prepare_data.value = prepare_time;
  Eye_twitching_invalid_master(&my_prepare_data);
  bee = AT24CXX_ReadOrWriteZero(0xF8);
  my_prepare_data.cmd_type_low = 0xA7;
  my_prepare_data.value = bee;
  Eye_twitching_invalid_master(&my_prepare_data);
}
/**
 * @brief AD24C01_Factory_formatted 鍑芥暟瀹炵幇銆? */
void AD24C01_Factory_formatted(void){
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
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
