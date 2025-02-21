
#include "main.h"

uint8_t BQ27441_TempData[2];

// 3300mAh �ĵ�ز���
uint8_t NEW_DC_MSB = 0x0C; // ����������ֽ�
uint8_t NEW_DC_LSB = 0xE4; // ����������ֽ�

uint8_t NEW_DE_MSB = 0x2F; // ����������ֽ�
uint8_t NEW_DE_LSB = 0xA2; // ����������ֽ�

uint8_t NEW_TV_MSB = 0x0B; // ��ֹ��ѹ���ֽڣ�3000mV��
uint8_t NEW_TV_LSB = 0xB8; // ��ֹ��ѹ���ֽڣ�3000mV��

uint8_t NEW_TR_MSB = 0x00; // Taper Rate ���ֽڣ�150mA��
uint8_t NEW_TR_LSB = 0x16; // Taper Rate ���ֽڣ�150mA��

uint8_t OLD_DC_LSB, OLD_DC_MSB, OLD_DE_LSB, OLD_DE_MSB, OLD_TV_LSB, OLD_TV_MSB,
    OLD_TR_LSB, OLD_TR_MSB;
uint8_t CHKSUM;
uint8_t BQ27441_FLAG_LSB, BQ27441_CONTROL_STATUS_MSB;

BQ27441_typedef BQ27441;
volatile bool i2c_transfer_success = false; // ������״̬
extern I2C_HandleTypeDef hi2c1;

//uint8_t BQ27441_Init(void) {
//  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM;
//  uint8_t tempee;
//  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
// if (tempee != 0x55) {
//   // ��������ģʽ��ͨ���� 0x00 �Ĵ���д�� 0x8000 ���� BQ27441
//   BQ27441_WriteWord(0x00, 0x8000);
//   BQ27441_WriteWord(0x00, 0x8000);
//
//   // �� 0x0013 д�� 0x00����ʾ���� UNSEAL ����������ʼĴ���
//   BQ27441_WriteWord(0x00, 0x0013);
//
//   // �ȴ� 2 �룬ȷ���������
//   HAL_Delay(200);
//
//   // ��ȡ 0x06 �Ĵ�������� FLAG ��־λ
//   BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//   HAL_Delay(1);
//
//   // �ȴ� I?C ���߿���
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // ��� FLAG ��־δ��ȷ���ã�����ʧ��
//   if ((BQ27441_FLAG_LSB & 0x10) != 0x10) {
//     return 0; // ����ʧ��
//   }
//
//   // ����������͵�ַָ��
//   BQ27441_WriteByte(0x61, 0x00); // д�� 0x61 �Ĵ��������ÿ�������
//   BQ27441_WriteByte(0x3E, 0x52); // д�� 0x3E �������ַΪ 0x52
//   BQ27441_WriteByte(0x3F, 0x00); // �����ӵ�ַΪ 0x00
//   HAL_Delay(1);
//
//   // ��ȡ�ɵ�У���
//   BQ27441_Read(0x60, &OLD_CHKSUM);
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // ������ʱУ���
//   TMP_CHKSUM = 0xFF - OLD_CHKSUM;
//
//   // ��ȡ�ɵĵ�ز���
//   BQ27441_Read(0x4A, &OLD_DC_MSB); // ����������ֽ�
//   BQ27441_Read(0x4B, &OLD_DC_LSB); // ����������ֽ�
//   BQ27441_Read(0x4C, &OLD_DE_MSB); // ����������ֽ�
//   BQ27441_Read(0x4D, &OLD_DE_LSB); // ����������ֽ�
//   BQ27441_Read(0x50, &OLD_TV_MSB); // ��ֹ��ѹ���ֽ�
//   BQ27441_Read(0x51, &OLD_TV_LSB); // ��ֹ��ѹ���ֽ�
//   BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate ���ֽ�
//   BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate ���ֽ�
//
//   // �ٴεȴ� I?C ���߿���
//   while (hi2c1.State != HAL_I2C_STATE_READY) {
//     ;
//   }
//
//   // ���ɲ����Ƿ���Ҫ����
//   if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
//     // ����У����Լ�ȥ��ֵ
//     TMP_CHKSUM = TMP_CHKSUM - OLD_DC_LSB - OLD_DC_MSB - OLD_DE_LSB -
//                  OLD_DE_MSB - OLD_TV_LSB - OLD_TV_MSB - OLD_TR_LSB -
//                  OLD_TR_MSB;
//
//     HAL_Delay(1);
//
//     // д���µĵ�ز���
//     BQ27441_WriteByte(0x4A, NEW_DC_MSB); // ����������ֽ�
//     BQ27441_WriteByte(0x4B, NEW_DC_LSB); // ����������ֽ�
//     BQ27441_WriteByte(0x4C, NEW_DE_MSB); // ����������ֽ�
//     BQ27441_WriteByte(0x4D, NEW_DE_LSB); // ����������ֽ�
//     BQ27441_WriteByte(0x50, NEW_TV_MSB); // ��ֹ��ѹ���ֽ�
//     BQ27441_WriteByte(0x51, NEW_TV_LSB); // ��ֹ��ѹ���ֽ�
//     BQ27441_WriteByte(0x5B, NEW_TR_MSB); // Taper Rate ���ֽ�
//     BQ27441_WriteByte(0x5C, NEW_TR_LSB); // Taper Rate ���ֽ�
//
//     // ����У����Լ�����ֵ
//     TMP_CHKSUM = TMP_CHKSUM + NEW_DC_LSB + NEW_DC_MSB + NEW_DE_LSB +
//                  NEW_DE_MSB + NEW_TV_LSB + NEW_TV_MSB + NEW_TR_LSB +
//                  NEW_TR_MSB;
//
//     // �����µ�У��Ͳ�д��
//     NEW_CHKSUM = 0xFF - TMP_CHKSUM;
//     BQ27441_WriteByte(0x60, NEW_CHKSUM);
//     HAL_Delay(1000);
//
//     // �ٴ�������������ӵ�ַ
//     BQ27441_WriteByte(0x3E, 0x52);
//     BQ27441_WriteByte(0x3F, 0x00);
//     HAL_Delay(1);
//
//     // ��ȡ����֤�µ�У���
//     BQ27441_Read(0x60, &CHKSUM);
//     while (hi2c1.State != HAL_I2C_STATE_READY) {
//       ;
//     }
//
//     // У�����֤�ɹ�
//     if (CHKSUM == NEW_CHKSUM) {
//       // �ȴ� DF ִ�����
//       do {
//         BQ27441_WriteWord(0x00, 0x0042);
//         HAL_Delay(1);
//         BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//         HAL_Delay(1);
//       } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//       // ���� CONTROL 0x0020 ����
//       BQ27441_WriteWord(0x00, 0x0020);
//
//       // ���� EEPROM ����
//       AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//
//       return 1; // �����ɹ�
//     } else {
//       return 0; // У��ʹ���
//     }
//   } else {
//     // �������δ���£�ֱ��ִ�� DF �������
//     do {
//       BQ27441_WriteWord(0x00, 0x0042);
//       HAL_Delay(1);
//       BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//       HAL_Delay(1);
//     } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//     // ���� CONTROL 0x0020 ����
//     BQ27441_WriteWord(0x00, 0x0020);
//
//     // ���� EEPROM ����
//     AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//
//     return 1; // �����ɹ�
//   }
// }
//}

void I2C_Semaphore_Init(void) {
    xI2CMutex = xSemaphoreCreateMutex();
    xI2CCompleteSem = xSemaphoreCreateBinary();

    if (xI2CMutex == NULL || xI2CCompleteSem == NULL) {
        printf("I2C�ź�������ʧ��!\r\n");
        Error_Handler();
    }
}

uint8_t BQ27441_Init(void) {
  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM; // У��ͱ���
  uint8_t tempee;

  // ��EEPROM��ȡ���ñ�־�������ж��Ƿ���Ҫ���³�ʼ��
  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
  printf("[LOG] Read EEPROM flag: 0x%02X\n", tempee);

  //if (tempee != 0x55) { // �����־δ���ã���ʼ��ʼ��
  if (1) { // �����־δ���ã���ʼ��ʼ��
    printf("[LOG] Starting initialization...\n");

    // ����BQ27441����������ģʽ
    BQ27441_WriteWord(0x00, 0x8000);
    BQ27441_WriteWord(0x00, 0x8000);
    HAL_Delay(1000); // ȷ������ģʽ����(����Ҫ��
    // ����UNSEAL���������Է��ʼĴ���
    BQ27441_WriteWord(0x00, 0x0013);
    // ��ȡFLAG�Ĵ�����ȷ�Ͻ���״̬
    for (int i = 0; i < 20; ++i) {
      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
      HAL_Delay(1); // �ȴ��������
    }

    printf("[LOG] FLAG register: 0x%02X\n", BQ27441_FLAG_LSB);
    if ((BQ27441_FLAG_LSB & 0x10) == 0) { // ���FLAG��־
      printf("[ERROR] Unlock failed.\n");
      return 0; // �������ʧ�ܣ�����ʧ��״̬
    }

    // ����������Ϊ0x52��Design���Զ�ȡ��д���ز���
    //BQ27441_WriteByte(0x61, 0x00); // �������ַ
    BQ27441_WriteByte(0x3E, 0x52); // �������ַ
    BQ27441_WriteByte(0x3F, 0x00); // �ӵ�ַ
    HAL_Delay(5); // �ȴ��������
    printf("[LOG] Set data class to 0x52.\n");

    // ��ȡ�ɵ�У��ͣ����ڸ��²��������¼���
    BQ27441_Read(0x60, &OLD_CHKSUM);
    printf("[LOG] Old checksum: 0x%02X\n", OLD_CHKSUM);

    // ��ȡ�ɵĵ�ز���
    BQ27441_Read(0x4A, &OLD_DC_MSB); // ����������ֽ�
    BQ27441_Read(0x4B, &OLD_DC_LSB); // ����������ֽ�
    BQ27441_Read(0x4C, &OLD_DE_MSB); // ����������ֽ�
    BQ27441_Read(0x4D, &OLD_DE_LSB); // ����������ֽ�
    BQ27441_Read(0x50, &OLD_TV_MSB); // ��ֹ��ѹ���ֽ�
    BQ27441_Read(0x51, &OLD_TV_LSB); // ��ֹ��ѹ���ֽ�
    BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate���ֽ�
    BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate���ֽ�
    printf("[LOG] Old parameters: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
           OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB, OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);

    // ����ɲ������²�����ͬ������Ҫ����
    if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
      printf("[LOG] Parameters mismatch, updating...\n");
      TMP_CHKSUM = 0xFF - OLD_CHKSUM; // ������ʱУ��ͣ��Ƴ���ֵ
      TMP_CHKSUM -= OLD_DC_MSB + OLD_DC_LSB + OLD_DE_MSB + OLD_DE_LSB;
      TMP_CHKSUM -= OLD_TV_MSB + OLD_TV_LSB + OLD_TR_MSB + OLD_TR_LSB;

      // д���µĵ�ز���
      BQ27441_WriteByte(0x4A, NEW_DC_MSB); // д������������ֽ�
      BQ27441_WriteByte(0x4B, NEW_DC_LSB); // д������������ֽ�
      BQ27441_WriteByte(0x4C, NEW_DE_MSB); // д������������ֽ�
      BQ27441_WriteByte(0x4D, NEW_DE_LSB); // д������������ֽ�
      BQ27441_WriteByte(0x50, NEW_TV_MSB); // д����ֹ��ѹ���ֽ�
      BQ27441_WriteByte(0x51, NEW_TV_LSB); // д����ֹ��ѹ���ֽ�
      BQ27441_WriteByte(0x5B, NEW_TR_MSB); // д��Taper Rate���ֽ�
      BQ27441_WriteByte(0x5C, NEW_TR_LSB); // д��Taper Rate���ֽ�

      TMP_CHKSUM += NEW_DC_MSB + NEW_DC_LSB + NEW_DE_MSB + NEW_DE_LSB;
      TMP_CHKSUM += NEW_TV_MSB + NEW_TV_LSB + NEW_TR_MSB + NEW_TR_LSB;
      NEW_CHKSUM = 0xFF - TMP_CHKSUM; // �����µ�У���

      // д���µ�У���
      BQ27441_WriteByte(0x60, NEW_CHKSUM);
      HAL_Delay(100);
      printf("[LOG] New checksum written: 0x%02X\n", NEW_CHKSUM);

      // ��֤У����Ƿ���ȷ
      BQ27441_Read(0x60, &CHKSUM);
      if (CHKSUM != NEW_CHKSUM) {
        printf("[ERROR] Checksum verification failed.\n");
        return 0; // ���У��ʹ��󣬷���ʧ��
      }
    }

    // �ȴ�DFִ�����
    do {
      BQ27441_WriteWord(0x00, 0x0042);
      HAL_Delay(5);
      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
    } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);

    // ����Control����0x0020���������
    BQ27441_WriteWord(0x00, 0x0020);
    printf("[LOG] DF execution completed.\n");

    // ����EEPROM��־����ʾ��ʼ�����
    AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
    printf("[LOG] Initialization completed successfully.\n");

    return 1; // ��ʼ���ɹ�
  }

  printf("[LOG] Initialization skipped, already configured.\n");
  return 1; // ����ѳ�ʼ������ֱ�ӷ��سɹ�
}

void BQ27441_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}


HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // ��ȡ����������ֹ�������ͬʱ����I2C
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ������ź�������ֹ�ϴβ���Ӱ��
        xSemaphoreTake(xI2CCompleteSem, 0);
        // ���� I2C ������ (ʹ���ж�ģʽ)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        if (status == HAL_OK) {
            // �ȴ� I2C ��������ź��� (����ʱ)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // �����ɹ�
            } else {
                // ��ʱ����
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }
        // �ͷ�I2C������
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // ��ȡ��ʧ�ܣ�I2C���߷�æ
    }
    return status;
}
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ������ź���
        xSemaphoreTake(xI2CCompleteSem, 0);

        // ���� I2C д���� (ʹ���ж�ģʽ)
        status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size);
        if (status == HAL_OK) {
            // �ȴ� I2C ��������ź���
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // �����ɹ�
            } else {
                // ��ʱ����
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // �ͷ�I2C������
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }

    return status;
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
// ȫ�ֱ��������ڱ�� DMA ����״̬
extern volatile bool dma_transfer_complete;
// DMA ��ȡ����
HAL_StatusTypeDef BQ27441_MultiRead_DMA(BQ27441_typedef *BQ_State) {
    BQ27441_Read_IT(0x04,(uint8_t *)&(BQ_State->Voltage), 2);
    BQ27441_Read_IT(0x02,(uint8_t *)&(BQ_State->Temperature), 2);
    BQ27441_Read_IT(0x10,(uint8_t *)&(BQ_State->AvgCurrent), 2);
    BQ27441_Read_IT(0x1C,(uint8_t *)&(BQ_State->SOC), 2);
    BQ27441_Read_IT(0x0E,(uint8_t *)&(BQ_State->FullChargeCapacity), 2);
  return HAL_OK; // ���ж�ȡ�����ɹ�
}
void BQ27441_WriteByte(uint8_t WriteAddr, uint8_t WriteData) {
  BQ27441_TempData[0] = WriteData;
    BQ27441_Write_IT( WriteAddr, BQ27441_TempData, 1);
}
void BQ27441_WriteWord(uint8_t WriteAddr, uint16_t WriteData) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ27441_TempData[0] = WriteData;
  BQ27441_TempData[1] = WriteData >> 8;
  BQ27441_Write_IT(WriteAddr,BQ27441_TempData, 2);
}

extern uint8_t low_battery;
extern uint8_t battery_flag_400ms;
//extern osMessageQueueId_t Battery_DATAHandle;
float batp;
static float lastBatteryValue = 0;// ����һ���������ڴ洢��һ�ε�ֵ
void battery_status_update_bq27441(void) {
  BQ27441_MultiRead_DMA(&BQ27441);
  low_battery = (BQ27441.SOC < 20) && (BQ27441.SOC != 0);
    float battery = (float)BQ27441.SOC;
    //ScreenUpdateSOC(battery);
    if(battery_flag_400ms){
        battery_flag_400ms=0;
        if (lastBatteryValue == 0 || fabs(battery - lastBatteryValue) <= 10.0f) {
            lastBatteryValue = battery;
            ScreenUpdateSOC(battery);
        }
    }
    if (BQ27441.Voltage <= 3000) {
      BQ25895_Write(0x09, 0x64); // ��ѹ�ͣ��ضϱ���
    }
  }