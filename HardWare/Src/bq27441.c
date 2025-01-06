
#include "main.h"

uint8_t BQ27441_TempData[2];

uint8_t NEW_DC_MSB = 0x0B; // è®¾è?¡å?¹é‡é«˜å­—èŠ‚ï¼ˆ2900mAhï¼?
uint8_t NEW_DC_LSB = 0x54; // è®¾è?¡å?¹é‡ä½Žå­—èŠ‚ï¼ˆ2900mAhï¼?

uint8_t NEW_DE_MSB = 0x29; // è®¾è?¡èƒ½é‡é«˜å­—èŠ‚ï¼?10.73Wh = 10730mWhï¼?
uint8_t NEW_DE_LSB = 0xEA; // è®¾è?¡èƒ½é‡ä½Žå­—èŠ‚ï¼?10.73Wh = 10730mWhï¼?

uint8_t NEW_TV_MSB = 0x0B; // ç»ˆæ?¢ç”µåŽ‹é«˜å­—èŠ‚ï¼?3000mVï¼?
uint8_t NEW_TV_LSB = 0xB8; // ç»ˆæ?¢ç”µåŽ‹ä½Žå­—èŠ‚ï¼?3000mVï¼?

uint8_t NEW_TR_MSB = 0x01; // Taper Rate é«˜å­—èŠ‚ï¼ˆ453ï¼?
uint8_t NEW_TR_LSB = 0xC5; // Taper Rate ä½Žå­—èŠ‚ï¼ˆ453ï¼?
// 2700mAh/(0.1*64mA)453.125     64mAæ˜¯BQ25895å……ç”µç®¡æ–­ç”µæµ
uint8_t OLD_DC_LSB, OLD_DC_MSB, OLD_DE_LSB, OLD_DE_MSB, OLD_TV_LSB, OLD_TV_MSB,
    OLD_TR_LSB, OLD_TR_MSB;
uint8_t CHKSUM;
uint8_t BQ27441_FLAG_LSB, BQ27441_CONTROL_STATUS_MSB;

BQ27441_typedef BQ27441;

extern I2C_HandleTypeDef hi2c1;

uint8_t BQ27441_Init(void) {
  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM;
  uint8_t tempee;
  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
  //if (tempee != 0x5) {//at24ÊÇ·ñÅäÖÃ¹ýµç³Ø²ÎÊý
  if (1) {
    BQ27441_WriteWord(0x00, 0x8000);
    BQ27441_WriteWord(0x00, 0x8000);
    BQ27441_WriteWord(0x00, 0x0013);
    HAL_Delay(2000);

    BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
    HAL_Delay(1);
    while (hi2c1.State != HAL_I2C_STATE_READY) {
      ;
    }
    if ((BQ27441_FLAG_LSB & 0x10) != 0x10) {
      return 0;
    }
    BQ27441_WriteByte(0x61, 0x00);
    BQ27441_WriteByte(0x3E, 0x52);
    BQ27441_WriteByte(0x3F, 0x00);
    HAL_Delay(1);
    BQ27441_Read(0x60, &OLD_CHKSUM);
    while (hi2c1.State != HAL_I2C_STATE_READY) {
      ;
    }
    TMP_CHKSUM = 0xff - OLD_CHKSUM;
    BQ27441_Read(0x4A, &OLD_DC_MSB);
    BQ27441_Read(0x4B, &OLD_DC_LSB);
    BQ27441_Read(0x4C, &OLD_DE_MSB);
    BQ27441_Read(0x4D, &OLD_DE_LSB);
    BQ27441_Read(0x50, &OLD_TV_MSB);
    BQ27441_Read(0x51, &OLD_TV_LSB);
    BQ27441_Read(0x5B, &OLD_TR_MSB);
    BQ27441_Read(0x5C, &OLD_TR_LSB);
    while (hi2c1.State != HAL_I2C_STATE_READY) {
      ;
    }
    if (OLD_DC_MSB != NEW_DC_MSB ||
        OLD_DC_LSB !=
            NEW_DC_LSB) // ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½ï¿½ï¿½ï¿½è¶¨Ò»ï¿½ï¿½ï¿½Í²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½è¶?
    {
      TMP_CHKSUM = TMP_CHKSUM - OLD_DC_LSB - OLD_DC_MSB - OLD_DE_LSB -
                   OLD_DE_MSB - OLD_TV_LSB - OLD_TV_MSB - OLD_TR_LSB -
                   OLD_TR_MSB;
      HAL_Delay(1);
      BQ27441_WriteByte(0x4A, NEW_DC_MSB);
      BQ27441_WriteByte(0x4B, NEW_DC_LSB);
      BQ27441_WriteByte(0x4C, NEW_DE_MSB);
      BQ27441_WriteByte(0x4D, NEW_DE_LSB);
      BQ27441_WriteByte(0x50, NEW_TV_MSB);
      BQ27441_WriteByte(0x51, NEW_TV_LSB);
      BQ27441_WriteByte(0x5B, NEW_TR_MSB);
      BQ27441_WriteByte(0x5C, NEW_TR_LSB);
      TMP_CHKSUM = TMP_CHKSUM + NEW_DC_LSB + NEW_DC_MSB + NEW_DE_LSB +
                   NEW_DE_MSB + NEW_TV_LSB + NEW_TV_MSB + NEW_TR_LSB +
                   NEW_TR_MSB;
      NEW_CHKSUM = 0xff - TMP_CHKSUM;
      BQ27441_WriteByte(0x60, NEW_CHKSUM);
      HAL_Delay(1000);
      BQ27441_WriteByte(0x3E, 0x52);
      BQ27441_WriteByte(0x3F, 0x00);
      HAL_Delay(1);
      BQ27441_Read(0x60, &CHKSUM);
      while (hi2c1.State != HAL_I2C_STATE_READY) {
        ;
      }
      if (CHKSUM == NEW_CHKSUM) {
        do {
          BQ27441_WriteWord(0x00, 0x0042);
          HAL_Delay(1);
          BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
          HAL_Delay(1);
        } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
        BQ27441_WriteWord(0x00, 0x0020);
        AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add,
                             0x55); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Éºï¿½Ð´ï¿½ï¿½EEPROM
        return 1;
      } else {
        return 0;
      }
    } else {
      do {
        BQ27441_WriteWord(0x00, 0x0042);
        HAL_Delay(1);
        BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
        HAL_Delay(1);
      } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
      BQ27441_WriteWord(0x00, 0x0020);
      AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add,
                           0x55); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Éºï¿½Ð´ï¿½ï¿½EEPROM
      return 1;
    }
  } else {
    return 2; // ï¿½Ñ¾ï¿½ï¿½ï¿½ï¿½Ã¹ï¿½
  }
}

void BQ27441_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}

// void BQ27441_MultiRead(BQ27441_typedef *BQ_State) {
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x04, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->Voltage), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x02, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->Temperature), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x10, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->AvgCurrent), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x1C, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->SOC), 2);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//   HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x0E, I2C_MEMADD_SIZE_8BIT,
//                        (uint8_t *)&(BQ_State->FullChargeCapacity), 2);
// }

// È«¾Ö±äÁ¿£¬ÓÃÓÚ±ê¼Ç DMA ´«Êä×´Ì¬
extern volatile bool dma_transfer_complete;
// DMA ¶ÁÈ¡º¯Êý
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ_State)
{
    HAL_StatusTypeDef status;

    // Voltage
    dma_transfer_complete = false;
    while (hi2c1.State != HAL_I2C_STATE_READY) {
        ; // µÈ´ý I2C ×´Ì¬Îª¿ÕÏÐ
    }
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x04, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)&(BQ_State->Voltage), 2);
    if (status != HAL_OK) {
        return status;
    }
    while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
        ; // µÈ´ý´«ÊäÍê³É
    }

    // Temperature
    dma_transfer_complete = false;
    while (hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x02, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)&(BQ_State->Temperature), 2);
    if (status != HAL_OK) {
        return status;
    }
    while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }

    // AvgCurrent
    dma_transfer_complete = false;
    while (hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x10, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)&(BQ_State->AvgCurrent), 2);
    if (status != HAL_OK) {
        return status;
    }
    while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }

    // SOC
    dma_transfer_complete = false;
    while (hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x1C, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)&(BQ_State->SOC), 2);
    if (status != HAL_OK) {
        return status;
    }
    while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }

    // FullChargeCapacity
    dma_transfer_complete = false;
    while (hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }
    status = HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, 0x0E, I2C_MEMADD_SIZE_8BIT,
                                  (uint8_t *)&(BQ_State->FullChargeCapacity), 2);
    if (status != HAL_OK) {
        return status;
    }
    while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
        ;
    }

    return HAL_OK; // ËùÓÐ¶ÁÈ¡²Ù×÷³É¹¦
}
void BQ27441_WriteByte(uint8_t WriteAddr, uint8_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ27441Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ27441_TempData, 1);
}

void BQ27441_WriteWord(uint8_t WriteAddr, uint16_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  BQ27441_TempData[1] = WriteData >> 8;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ27441Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ27441_TempData, 2);
}

extern bool low_battery;
extern osMessageQueueId_t Battery_DATAHandle;
void battery_status_update_bq27441(void) {
  //BQ27441_MultiRead(&BQ27441); // ¶ÁÈ¡µç³Ø×´Ì¬
   // µ÷ÓÃ DMA ¶ÁÈ¡º¯Êý
    if (BQ27441_MultiRead_DMA(&BQ27441) == HAL_OK) {
        // ¶ÁÈ¡³É¹¦£¬¸üÐÂ×´Ì¬
        // printf("Voltage: %d mV\n", BQ27441.Voltage);
        // printf("Temperature: %d\n", BQ27441.Temperature);
        // printf("AvgCurrent: %d\n", BQ27441.AvgCurrent);
        // printf("SOC: %d %%\n", BQ27441.SOC);
        // printf("FullChargeCapacity: %d mAh\n", BQ27441.FullChargeCapacity);
    } else {
        // ¶ÁÈ¡Ê§°Ü£¬¼ÇÂ¼´íÎó
        printf("Failed to read battery status\n");
    }
  low_battery = (BQ27441.SOC < 20);
  float battery=(float)BQ27441.SOC;
   xQueueSend(Battery_DATAHandle, &battery,0); // ½«Êý¾Ý·¢ËÍµ½¶ÓÁÐ
  if (BQ27441.Voltage <= 3000) {
    BQ25895_Write(0x09, 0x64); // µçÑ¹µÍ£¬¹Ø¶Ï±£»¤
  }
}