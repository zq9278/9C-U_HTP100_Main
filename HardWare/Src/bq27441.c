
#include "main.h"
#define LOW_BATTERY_SOC 3200
#define WORKING_BATTERY_SOC    3300  // �����û���ѹ
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
uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM;
uint8_t tempee;
uint8_t BQ27441_Init(void) {

// ��ȡ EEPROM ���ñ�־
    tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
    LOG("��ȡ EEPROM ���ñ�־: 0x%02X\n", tempee);

// if (tempee != 0x55) {
    if (1) {
        LOG("��ʼ BQ27441 ��ʼ��...\n");

        // ��������ģʽ��ͨ���� 0x00 �Ĵ���д�� 0x8000 ���� BQ27441
        BQ27441_WriteWord(0x00, 0x8000);
        BQ27441_WriteWord(0x00, 0x8000);
        LOG("���� BQ27441����������ģʽ\n");

        // ���� UNSEAL ��������
        BQ27441_WriteWord(0x00, 0x0013);
        LOG("���� UNSEAL ����\n");

        HAL_Delay(2000); // �ȴ��������

        // ��ȡ 0x06 FLAG �Ĵ���������־λ
        BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
        HAL_Delay(1);
        LOG("FLAG �Ĵ���: 0x%02X\n", BQ27441_FLAG_LSB);

        // �ȴ� I2C ���߿���
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }

        // ��� FLAG ��־δ��ȷ���ã�����ʧ��
        if ((BQ27441_FLAG_LSB & 0x10) != 0x10) {
            LOG("����ʧ�ܣ�\n");
            //return 0;
        }

        // ѡ�� Data Memory ��
        BQ27441_WriteByte(0x3E, 0x52); // 0x52 = Design Data Class
        BQ27441_WriteByte(0x3F, 0x00); // ƫ���� = 0x00
        HAL_Delay(1);
        LOG("����������: 0x52, ƫ����: 0x00\n");

        // ��ȡ�ɵ�У���
        BQ27441_Read(0x60, &OLD_CHKSUM);
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }
        LOG("��ȡ���ľ�У���: 0x%02X\n", OLD_CHKSUM);

        // ������ʱУ���
        TMP_CHKSUM = 0xFF - OLD_CHKSUM;

        // ��ȡ�ɵĵ�ز���
        BQ27441_Read(0x4A, &OLD_DC_MSB); // ����������ֽ�
        BQ27441_Read(0x4B, &OLD_DC_LSB); // ����������ֽ�
        BQ27441_Read(0x4C, &OLD_DE_MSB); // ����������ֽ�
        BQ27441_Read(0x4D, &OLD_DE_LSB); // ����������ֽ�
        BQ27441_Read(0x50, &OLD_TV_MSB); // ��ֹ��ѹ���ֽ�
        BQ27441_Read(0x51, &OLD_TV_LSB); // ��ֹ��ѹ���ֽ�
        BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate ���ֽ�
        BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate ���ֽ�

        // �ٴεȴ� I2C ���߿���
        while (hi2c1.State != HAL_I2C_STATE_READY) { ;
        }

        LOG("��ȡ���ľɲ���: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
            OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB,
            OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);

        // ���ɲ����Ƿ���Ҫ����
        if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
            LOG("������ͬ����ʼ����...\n");

            // ����У����Լ�ȥ��ֵ
            TMP_CHKSUM -= OLD_DC_LSB + OLD_DC_MSB + OLD_DE_LSB + OLD_DE_MSB;
            TMP_CHKSUM -= OLD_TV_LSB + OLD_TV_MSB + OLD_TR_LSB + OLD_TR_MSB;

            HAL_Delay(1);

            // д���µĵ�ز���
            BQ27441_WriteByte(0x4A, NEW_DC_MSB);
            BQ27441_WriteByte(0x4B, NEW_DC_LSB);
            BQ27441_WriteByte(0x4C, NEW_DE_MSB);
            BQ27441_WriteByte(0x4D, NEW_DE_LSB);
            BQ27441_WriteByte(0x50, NEW_TV_MSB);
            BQ27441_WriteByte(0x51, NEW_TV_LSB);
            BQ27441_WriteByte(0x5B, NEW_TR_MSB);
            BQ27441_WriteByte(0x5C, NEW_TR_LSB);

            // ����У����Լ�����ֵ
            TMP_CHKSUM += NEW_DC_LSB + NEW_DC_MSB + NEW_DE_LSB + NEW_DE_MSB;
            TMP_CHKSUM += NEW_TV_LSB + NEW_TV_MSB + NEW_TR_LSB + NEW_TR_MSB;

            // �����µ�У��Ͳ�д��
            NEW_CHKSUM = 0xFF - TMP_CHKSUM;
            BQ27441_WriteByte(0x60, NEW_CHKSUM);
            HAL_Delay(1000);

            LOG("��У��ͼ������: 0x%02X\n", NEW_CHKSUM);

            // ��ȡ����֤�µ�У���
            BQ27441_Read(0x60, &CHKSUM);
            if (CHKSUM != NEW_CHKSUM) {
                LOG("У�����֤ʧ�ܣ�ʵ��: 0x%02X, ����: 0x%02X\n", CHKSUM, NEW_CHKSUM);
                return 0;
            }

            LOG("У�����֤�ɹ���\n");

            // �ȴ� DF ִ�����
            do {
                BQ27441_WriteWord(0x00, 0x0042);
                HAL_Delay(1);
                BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
                HAL_Delay(1);
            } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);

            // ���� CONTROL 0x0020 ����
            BQ27441_WriteWord(0x00, 0x0020);

            // ���� EEPROM ����
            AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);

            LOG("�������������ɣ�\n");

            return 1; // �����ɹ�
        } else {
            LOG("����δ�����������¡�\n");
            return 1;
        }
    }
}

    void I2C_Semaphore_Init(void) {
    xI2CMutex = xSemaphoreCreateMutex();
    xI2CCompleteSem = xSemaphoreCreateBinary();

    if (xI2CMutex == NULL || xI2CCompleteSem == NULL) {
        printf("I2C�ź�������ʧ��!\r\n");
        Error_Handler();
    }
}

//uint8_t BQ27441_Init(void) {
//  uint8_t OLD_CHKSUM, NEW_CHKSUM, TMP_CHKSUM; // У��ͱ���
//  uint8_t tempee;
//
//  // ��EEPROM��ȡ���ñ�־�������ж��Ƿ���Ҫ���³�ʼ��
//  tempee = AT24CXX_ReadOneByte(EEPROM_BQ27441Config_Add);
//  printf("[LOG] Read EEPROM flag: 0x%02X\n", tempee);
//
//  //if (tempee != 0x55) { // �����־δ���ã���ʼ��ʼ��
//  if (1) { // �����־δ���ã���ʼ��ʼ��
//    printf("[LOG] Starting initialization...\n");
//
//    // ����BQ27441����������ģʽ
////    BQ27441_WriteWord(0x00, 0x8000);
////    BQ27441_WriteWord(0x00, 0x8000);
//
//    HAL_Delay(1000); // ȷ������ģʽ����(����Ҫ��
//    // ����UNSEAL���������Է��ʼĴ���
//    BQ27441_WriteWord(0x00, 0x0013);
//    // ��ȡFLAG�Ĵ�����ȷ�Ͻ���״̬
//    for (int i = 0; i < 20; ++i) {
//      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//      HAL_Delay(1); // �ȴ��������
//    }
//
//    printf("[LOG] FLAG register: 0x%02X\n", BQ27441_FLAG_LSB);
//    if ((BQ27441_FLAG_LSB & 0x10) == 0) { // ���FLAG��־
//      printf("[ERROR] Unlock failed.\n");
//      return 0; // �������ʧ�ܣ�����ʧ��״̬
//    }
//
//    // ����������Ϊ0x52��Design���Զ�ȡ��д���ز���
//    //BQ27441_WriteByte(0x61, 0x00); // �������ַ
//    BQ27441_WriteByte(0x3E, 0x52); // �������ַ
//    BQ27441_WriteByte(0x3F, 0x00); // �ӵ�ַ
//    HAL_Delay(5); // �ȴ��������
//    printf("[LOG] Set data class to 0x52.\n");
//
//    // ��ȡ�ɵ�У��ͣ����ڸ��²��������¼���
//    BQ27441_Read(0x60, &OLD_CHKSUM);
//    printf("[LOG] Old checksum: 0x%02X\n", OLD_CHKSUM);
//
//    // ��ȡ�ɵĵ�ز���
//    BQ27441_Read(0x4A, &OLD_DC_MSB); // ����������ֽ�
//    BQ27441_Read(0x4B, &OLD_DC_LSB); // ����������ֽ�
//    BQ27441_Read(0x4C, &OLD_DE_MSB); // ����������ֽ�
//    BQ27441_Read(0x4D, &OLD_DE_LSB); // ����������ֽ�
//    BQ27441_Read(0x50, &OLD_TV_MSB); // ��ֹ��ѹ���ֽ�
//    BQ27441_Read(0x51, &OLD_TV_LSB); // ��ֹ��ѹ���ֽ�
//    BQ27441_Read(0x5B, &OLD_TR_MSB); // Taper Rate���ֽ�
//    BQ27441_Read(0x5C, &OLD_TR_LSB); // Taper Rate���ֽ�
//    printf("[LOG] Old parameters: DC=0x%02X%02X, DE=0x%02X%02X, TV=0x%02X%02X, TR=0x%02X%02X\n",
//           OLD_DC_MSB, OLD_DC_LSB, OLD_DE_MSB, OLD_DE_LSB, OLD_TV_MSB, OLD_TV_LSB, OLD_TR_MSB, OLD_TR_LSB);
//
//    // ����ɲ������²�����ͬ������Ҫ����
//    if (OLD_DC_MSB != NEW_DC_MSB || OLD_DC_LSB != NEW_DC_LSB) {
//      printf("[LOG] Parameters mismatch, updating...\n");
//      TMP_CHKSUM = 0xFF - OLD_CHKSUM; // ������ʱУ��ͣ��Ƴ���ֵ
//      TMP_CHKSUM -= OLD_DC_MSB + OLD_DC_LSB + OLD_DE_MSB + OLD_DE_LSB;
//      TMP_CHKSUM -= OLD_TV_MSB + OLD_TV_LSB + OLD_TR_MSB + OLD_TR_LSB;
//
//      // д���µĵ�ز���
//      BQ27441_WriteByte(0x4A, NEW_DC_MSB); // д������������ֽ�
//      BQ27441_WriteByte(0x4B, NEW_DC_LSB); // д������������ֽ�
//      BQ27441_WriteByte(0x4C, NEW_DE_MSB); // д������������ֽ�
//      BQ27441_WriteByte(0x4D, NEW_DE_LSB); // д������������ֽ�
//      BQ27441_WriteByte(0x50, NEW_TV_MSB); // д����ֹ��ѹ���ֽ�
//      BQ27441_WriteByte(0x51, NEW_TV_LSB); // д����ֹ��ѹ���ֽ�
//      BQ27441_WriteByte(0x5B, NEW_TR_MSB); // д��Taper Rate���ֽ�
//      BQ27441_WriteByte(0x5C, NEW_TR_LSB); // д��Taper Rate���ֽ�
//
//      TMP_CHKSUM += NEW_DC_MSB + NEW_DC_LSB + NEW_DE_MSB + NEW_DE_LSB;
//      TMP_CHKSUM += NEW_TV_MSB + NEW_TV_LSB + NEW_TR_MSB + NEW_TR_LSB;
//      NEW_CHKSUM = 0xFF - TMP_CHKSUM; // �����µ�У���
//
//      // д���µ�У���
//      BQ27441_WriteByte(0x60, NEW_CHKSUM);
//      HAL_Delay(100);
//      printf("[LOG] New checksum written: 0x%02X\n", NEW_CHKSUM);
//
//      // ��֤У����Ƿ���ȷ
//      BQ27441_Read(0x60, &CHKSUM);
//      if (CHKSUM != NEW_CHKSUM) {
//        printf("[ERROR] Checksum verification failed.\n");
//        return 0; // ���У��ʹ��󣬷���ʧ��
//      }
//    }
//
//    // �ȴ�DFִ�����
//    do {
//      BQ27441_WriteWord(0x00, 0x0042);
//      HAL_Delay(5);
//      BQ27441_Read(0x06, &BQ27441_FLAG_LSB);
//    } while ((BQ27441_FLAG_LSB & 0x10) == 0x10);
//
//    // ����Control����0x0020���������
//    BQ27441_WriteWord(0x00, 0x0020);
//    printf("[LOG] DF execution completed.\n");
//
//    // ����EEPROM��־����ʾ��ʼ�����
//    AT24CXX_WriteOneByte(EEPROM_BQ27441Config_Add, 0x55);
//    printf("[LOG] Initialization completed successfully.\n");
//
//    return 1; // ��ʼ���ɹ�
//  }
//
//  printf("[LOG] Initialization skipped, already configured.\n");
//  return 1; // ����ѳ�ʼ������ֱ�ӷ��سɹ�
//}

void BQ27441_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ27441Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}


//HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
//    HAL_StatusTypeDef status = HAL_ERROR;
//
//    // ��ȡ I2C ����������ֹ�������ͬʱ���� I2C
//    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
//        // ������ź�������ֹ�ϴβ���Ӱ��
//        xSemaphoreTake(xI2CCompleteSem, 0);
//
//        // ���� I2C ������ (ʹ���ж�ģʽ)
//        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
//
//        if (status == HAL_OK) {
//
//            // �ȴ� I2C ��������ź��� (����ʱ)
//            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
//                status = HAL_OK;  // �����ɹ�
//                LOG("I2C ��ȡ�ɹ�: �Ĵ���=0x%02X, ����=[0x%02X]\n", regAddr, *pBuffer);
//            } else {
//                LOG("I2C ��ȡ��ʱ���������³�ʼ�� I2C\n");
//                HAL_I2C_DeInit(&hi2c1);
//                HAL_I2C_Init(&hi2c1);
//                status = HAL_TIMEOUT;
//            }
//        } else {
//            LOG("I2C ��ȡʧ��: HAL ״̬=%d\n", status);
//        }
//
//        // �ͷ� I2C ������
//        xSemaphoreGive(xI2CMutex);
//    } else {
//        status = HAL_BUSY;  // ��ȡ��ʧ�ܣ�I2C ���߷�æ
//        LOG("I2C ��������ȡʧ�ܣ�I2C ���߷�æ\n");
//    }
//
//    return status;
//}

HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // ��ȡ����������ֹ�������ͬʱ����I2C
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ������ź�������ֹ�ϴβ���Ӱ��
        xSemaphoreTake(xI2CCompleteSem, 0);
        // ���� I2C ������ (ʹ���ж�ģʽ)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        //status = HAL_I2C_Mem_Read(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size,0xffff);
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
//HAL_StatusTypeDef BQ27441_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
//    HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
//}
HAL_StatusTypeDef BQ27441_Write_IT(uint8_t regAddr, uint8_t *pData, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ������ź���
        xSemaphoreTake(xI2CCompleteSem, 0);

        // ���� I2C д���� (ʹ���ж�ģʽ)
        //status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size);
        status = HAL_I2C_Mem_Write(&hi2c1, BQ27441Address, regAddr, I2C_MEMADD_SIZE_8BIT, pData, size,0xffff);
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


//    bq_read((uint8_t *)&(BQ_State->Voltage), 0x04,2);
//    bq_read((uint8_t *)&(BQ_State->Temperature),0x02, 2);
//    bq_read((uint8_t *)&(BQ_State->AvgCurrent),0x10, 2);
//    bq_read((uint8_t *)&(BQ_State->SOC), 0x1C,2);
//    bq_read((uint8_t *)&(BQ_State->FullChargeCapacity),0x0E, 2);
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



typedef enum {
    BATTERY_NORMAL = 0,
    BATTERY_CHECK,
    BATTERY_CONFIRM_SHUTDOWN,
    BATTERY_SHUTDOWN,
    BATTERY_WORKING_LOW
} BatteryState_t;

typedef struct {
    BatteryState_t state;
    uint8_t lowVoltageCounter;
    TickType_t lastCheckTick;
} BatteryMonitor_t;

static BatteryMonitor_t batteryMonitor = {
        .state = BATTERY_NORMAL,
        .lowVoltageCounter = 0,
        .lastCheckTick = 0,
};

// �����еĺ���


void BatteryMonitor_Run(void)
{
    // ÿ�ν��붼��һ��ʱ���
    TickType_t nowTick = xTaskGetTickCount();
    const TickType_t checkInterval = pdMS_TO_TICKS(500); // ÿ500ms���һ��

    switch (batteryMonitor.state)
    {
        case BATTERY_NORMAL:
            if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
            {
                LOG("[Battery] [NORMAL] ��⵽���ص͵�ѹ %d mV��������...\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_CHECK;
                batteryMonitor.lowVoltageCounter = 0;
                batteryMonitor.lastCheckTick = nowTick;
            }
            else if (BQ27441.Voltage <= WORKING_BATTERY_SOC && BQ27441.Voltage != 0)
            {
                // ��ѹ���ڹ�����ѹ�ߣ���ǰUART������ʾ
                LOG("[Battery] [NORMAL] ��ص���ƫ�ͣ�%d mV���������û���\n", BQ27441.Voltage);
                // �������ͨ��UART����ȥ
                // uart_printf("Warning: Battery voltage low (%d mV)\r\n", BQ27441.Voltage);
                batteryMonitor.state = BATTERY_WORKING_LOW;
            }
            else
            {
                // ��ѹ����������
            }
            break;

        case BATTERY_WORKING_LOW:
            // �ڹ����͵�״̬�¼������
            if (nowTick - batteryMonitor.lastCheckTick >= checkInterval)
            {
                batteryMonitor.lastCheckTick = nowTick;

                if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
                {
                    LOG("[Battery] [WORKING_LOW] ��ѹ��һ���½������ص͵磨%d mV����������ػ����̡�\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_CHECK;
                    batteryMonitor.lowVoltageCounter = 0;
                }
                else if (BQ27441.Voltage > WORKING_BATTERY_SOC)
                {
                    LOG("[Battery] [WORKING_LOW] ��ѹ�ָ�������%d mV�����ص�����״̬��\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
                else
                {
                    // ������ WORKING_LOW
                }
            }
            break;

        case BATTERY_CHECK:
            if (nowTick - batteryMonitor.lastCheckTick >= checkInterval)
            {
                batteryMonitor.lastCheckTick = nowTick;

                if (BQ27441.Voltage <= LOW_BATTERY_SOC && BQ27441.Voltage != 0)
                {
                    batteryMonitor.lowVoltageCounter++;
                    LOG("[Battery] [CHECK] �͵�ѹȷ�� %d �Σ�%d mV��\n", batteryMonitor.lowVoltageCounter, BQ27441.Voltage);

                    if (batteryMonitor.lowVoltageCounter >= 5)
                    {
                        LOG("[Battery] [CHECK] ����ض�����������ȷ�Ϲض�״̬��\n");
                        batteryMonitor.state = BATTERY_CONFIRM_SHUTDOWN;
                    }
                }
                else
                {
                    LOG("[Battery] [CHECK] ��ѹ�ָ���%d mV�����ص�����״̬��\n", BQ27441.Voltage);
                    batteryMonitor.state = BATTERY_NORMAL;
                }
            }
            break;

        case BATTERY_CONFIRM_SHUTDOWN:
            LOG("[Battery] [CONFIRM_SHUTDOWN] �����͵�ȷ�ϣ�ִ�йػ��߼�...\n");
            // ִ�йػ�����������
             BQ25895_Write(0x09, 0x64); // ʾ����д�Ĵ����ػ�
            batteryMonitor.state = BATTERY_SHUTDOWN;
            break;

        case BATTERY_SHUTDOWN:
            LOG("[Battery] [SHUTDOWN] ��ǰ���ڹض�״̬������ģʽ�С�\n");
            break;

        default:
            LOG("[Battery] [ERROR] δ֪���״̬��ǿ�ƻع�������\n");
            batteryMonitor.state = BATTERY_NORMAL;
            break;
    }
}



void battery_status_update_bq27441(void) {
  BQ27441_MultiRead_DMA(&BQ27441);
    //LOG("BQ27441: Voltage=%d mV, Temperature=%d C, AvgCurrent=%d mA, SOC=%d%%, FullChargeCapacity=%d mAh\n", BQ27441.Voltage, BQ27441.Temperature, BQ27441.AvgCurrent, BQ27441.SOC, BQ27441.FullChargeCapacity);
  //low_battery = (BQ27441.SOC < 30) && (BQ27441.SOC != 0);
  low_battery = (BQ27441.SOC < 30) ;
    fully_charged=(BQ27441.SOC>=99);
    float battery = (float)BQ27441.SOC;
    //ScreenUpdateSOC(battery);
    if(battery_flag_400ms){
        battery_flag_400ms=0;
        if (lastBatteryValue == 0 || fabs(battery - lastBatteryValue) <= 10.0f) {
            lastBatteryValue = battery;
            ScreenUpdateSOC(battery);
        }
    }
//    if (BQ27441.Voltage <= 3300) {
//        LOG("��ص�ѹ%d\n", BQ27441.Voltage); // ��ѹ���ͣ��ضϱ���\n");
//      BQ25895_Write(0x09, 0x64); // ��ѹ�ͣ��ضϱ���
//    }
    BatteryMonitor_Run();
  }



#include <stdint.h>
#include <stdio.h>
#include "i2c.h"  // ��Ҫ��� I2C ��д�⣬���� STM32 �� ESP32 �� I2C ����

#define BQ27441_I2C_ADDR  0x55  // BQ27441 Ĭ�� I2C ��ַ
#define CONTROL_REG       0x00  // ���ƼĴ�����ַ
#define FLAGS_REG         0x06  // �豸��־�Ĵ���
#define BLOCK_DATA_CTRL   0x61  // ʹ�� Block Data Memory Control
#define DATA_BLOCK_CLASS  0x3E  // ���ݿ����Ĵ���
#define DATA_BLOCK        0x3F  // ���ݿ�ƫ�ƼĴ���
#define BLOCK_DATA_CHECKSUM 0x60  // У��ͼĴ���
#define DESIGN_CAP_OFFSET 0x0A  // �������ƫ�Ƶ�ַ
#define SOFT_RESET        0x0042  // ��λ����
#define SEAL_CMD          0x0020  // ���·�װ����
#define UNSEAL_KEY1       0x8000  // �����Կ��һ����
#define UNSEAL_KEY2       0x8000  // �����Կ�ڶ�����
#define TIMEOUT 2000  // ��ʱʱ��



// ʹ�� Block Data Memory Control
void BQ27441_EnableBlockDataMemoryControl() {
    uint8_t data[2] = {0x00, 0x00};
    BQ27441_Write_IT(BLOCK_DATA_CTRL, &data[0], 1);  // ʹ�����ݿ�洢����
}

// ѡ�����ݿ飨82 = 0x52��
void BQ27441_SelectDataBlock(uint8_t block) {
    uint8_t data = block;
    BQ27441_Write_IT(DATA_BLOCK_CLASS, &data, 1);  // ѡ�����ݿ�
}

// �������ݿ�ƫ��
void BQ27441_SetDataBlockOffset(uint8_t offset) {
    uint8_t data = offset;
    BQ27441_Write_IT(DATA_BLOCK, &data, 1);  // ѡ�����ݿ�ƫ����
}

// ��ȡ���ݿ�У���
uint8_t BQ27441_ReadChecksum() {
    uint8_t checksum;
   BQ27441_Read_IT( BLOCK_DATA_CHECKSUM, &checksum, 1);  // ��ȡУ���
    //HAL_I2C_Mem_Read_IT(&hi2c1, BQ27441Address, BLOCK_DATA_CHECKSUM, I2C_MEMADD_SIZE_8BIT, checksum, 2);
    return checksum;
}

// ��ȡ�豸��־�Ĵ���
uint16_t BQ27441_ReadFlags() {
    uint8_t data[2] = {0};
    BQ27441_Read_IT(FLAGS_REG, data, 2);
    return (data[1] << 8) | data[0];
}


// ��� BQ27441
void BQ27441_Unseal() {
    uint8_t data[2];

// ���͵�һ������Կ
    data[0] = 0x00; data[1] = 0x80;  // 0x8000
    BQ27441_Write_IT(0x00, data, 2);

// ���͵ڶ�������Կ
    BQ27441_Write_IT(0x00, data, 2);
    LOG("���͵ڶ����ֽ����Կ: 0x8000\n");

}

// �������ø���ģʽ
void BQ27441_SetConfigUpdate() {
    LOG("�������ø���ģʽ...\n");
    uint8_t data[2] = {0x13, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);

    int timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("�������ø���ģʽ��ʱ��\n");
            return;
        }
    }
    LOG("�ɹ��������ø���ģʽ\n");
}



// ��ȡ�������
void BQ27441_ReadDesignCapacity(uint8_t *data) {
    BQ27441_Read_IT(DESIGN_CAP_OFFSET, data, 2);
    LOG("��ȡ�����������: %d mAh\n", (data[1] << 8) | data[0]);
}

// д���µ��������
void BQ27441_WriteDesignCapacity(uint16_t new_capacity) {
    uint8_t data[2] = {new_capacity & 0xFF, (new_capacity >> 8) & 0xFF};
    BQ27441_Write_IT(DESIGN_CAP_OFFSET, data, 2);
    LOG("���������д��ɹ�: %d mAh\n", new_capacity);

    // ��ȡ������ֵ��ȷ���Ƿ�����д��ɹ�
    uint8_t check_data[2] = {0};
    BQ27441_ReadDesignCapacity(check_data);
    uint16_t written_capacity = (check_data[1] << 8) | check_data[0];

    if (written_capacity == new_capacity) {
        LOG("д����֤�ɹ��������������ȷд��: %d mAh\n", written_capacity);
    } else {
        LOG("д����֤ʧ�ܣ���ȡ����������� = %d mAh������ֵ = %d mAh\n", written_capacity, new_capacity);

        // **��������д��**
        LOG("���³���д���������...\n");
        BQ27441_Write_IT(DESIGN_CAP_OFFSET, data, 2);
        HAL_Delay(10); // **��оƬ�㹻ʱ�䴦��**

        // �ٴμ��
        BQ27441_ReadDesignCapacity(check_data);
        written_capacity = (check_data[1] << 8) | check_data[0];

        if (written_capacity == new_capacity) {
            LOG("����д��ɹ���\n");
        } else {
            LOG("����д����Ȼʧ�ܣ�������У�������\n");
        }
    }
}



// ����У���
void BQ27441_UpdateChecksum() {
    uint8_t old_checksum, new_checksum;
    uint8_t block_data[32];  // 0x40 - 0x5F ���ݿ�
    uint8_t sumX = 0;  // �洢 BlockData �� 8 λ���

    // **��ȡ��ǰУ���**
    old_checksum = BQ27441_ReadChecksum();
    LOG("��ȡ���ľ�У���: 0x%02X\n", old_checksum);

    if (old_checksum == 0x00) {
        LOG("���棺У��Ͷ�ȡʧ�ܣ����� I2C ��ȡ����\n");
    }

    // **��ȡ���� BlockData(0x40 - 0x5F)**
    BQ27441_Read_IT(0x40, block_data, 32);

    // **���� BlockData �� 8 λ��**
    for (int i = 0; i < 32; i++) {
        sumX += block_data[i];
    }

    // **������У���**
    new_checksum = 255 - (sumX % 256);
    LOG("�������У��� = 0x%02X\n", new_checksum);

    // **д���µ�У���**
    BQ27441_Write_IT(BLOCK_DATA_CHECKSUM, &new_checksum, 1);
    LOG("��У���д��ɹ�: 0x%02X\n", new_checksum);

    // **ȷ���Ƿ�ɹ�д��**
    uint8_t verify_checksum = BQ27441_ReadChecksum();
    if (verify_checksum != new_checksum) {
        LOG("У���д��ʧ�ܣ�ʵ�� = 0x%02X, ���� = 0x%02X\n", verify_checksum, new_checksum);
    } else {
        LOG("У��͸��³ɹ���\n");
    }
}




// ������λ����
void BQ27441_SoftReset() {
    uint8_t data[2] = {0x42, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);
    LOG("������λ����");
}
void BQ27441_DoubleSoftReset() {
    LOG("ִ�е�һ����λ...\n");
    BQ27441_SoftReset();
    HAL_Delay(100);  // **��оƬ�㹻ʱ��**

    LOG("ִ�еڶ�����λ...\n");
    BQ27441_SoftReset();
    HAL_Delay(100);
}

// ���·�װ BQ27441
void BQ27441_Seal() {
    uint8_t data[2] = {0x20, 0x00};
    BQ27441_Write_IT(CONTROL_REG, data, 2);
    LOG("���·�װ�豸");
}
void BQ27441_SelectAndVerifyBlock() {
    BQ27441_SelectDataBlock(0x52);  // ѡ�� 0x52
    HAL_Delay(10);

    // ��ȡ BlockData �ĵ�һλ��ȷ���Ѿ�ѡ��
    uint8_t verify_block = 0;
    BQ27441_Read_IT(DATA_BLOCK_CLASS, &verify_block, 1);
    if (verify_block != 0x52) {
        LOG("���棺BlockData ѡ��ʧ�ܣ���ǰֵ: 0x%02X\n", verify_block);
    } else {
        LOG("BlockData ѡ�гɹ�: 0x%02X\n", verify_block);
    }
}

// ���� BQ27441 �������
void BQ27441_UpdateDesignCapacity(uint16_t new_capacity) {
    BQ27441_Unseal();

    BQ27441_SetConfigUpdate();

    int timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("�ȴ� CFGUPDATE ��λ��ʱ��\n");
            return;
        }
    }
    LOG("CFGUPDATE ��λ�ɹ�\n");

    BQ27441_EnableBlockDataMemoryControl();

    // **�ֶ�ѡ�� BlockData(0x52)**
    BQ27441_SelectAndVerifyBlock();

    BQ27441_SetDataBlockOffset(0x00);

    uint8_t old_checksum = BQ27441_ReadChecksum();
    LOG("��ȡ���ľ�У���: 0x%02X\n", old_checksum);

    uint8_t old_data[2];
    BQ27441_ReadDesignCapacity(old_data);

    BQ27441_WriteDesignCapacity(new_capacity);

    // **��ȡ���ݣ�ȷ��д��ɹ�**
    uint8_t verify_data[2] = {0};
    BQ27441_ReadDesignCapacity(verify_data);
    uint16_t final_capacity = (verify_data[1] << 8) | verify_data[0];

    if (final_capacity != new_capacity) {
        LOG("�������д��ʧ�ܣ���������д��...\n");
        BQ27441_WriteDesignCapacity(new_capacity);
        HAL_Delay(10);
    }

    // **����У���**
    BQ27441_UpdateChecksum();

    // **ǿ���ύ����**
    BQ27441_SoftReset();

    timeout = TIMEOUT;
    while (!(BQ27441_ReadFlags() & 0x10)) {
        if (--timeout == 0) {
            LOG("�ȴ� CFGUPDATE ��λ��ʱ��\n");
            return;
        }
    }
    LOG("CFGUPDATE ��λ�ɹ�\n");

    // **����ȷ�������Ƿ���Ч**
    BQ27441_ReadDesignCapacity(verify_data);
    final_capacity = (verify_data[1] << 8) | verify_data[0];
    LOG("���ն�ȡ�����������: %d mAh\n", final_capacity);

    if (final_capacity != new_capacity) {
        LOG("�����������ʧ�ܣ�\n");
    } else {
        LOG("����������³ɹ���\n");
    }

    BQ27441_Seal();
}


// ��Ӧ�ú���
void main_app1(void) {
    BQ27441_Unseal();
    BQ27441_UpdateDesignCapacity(3000);
    BQ27441_SoftReset();
}

//
//
//const uint16_t terminatevoltage ={0x0C80};       //terminate voltage=3000mV,system shutdown voltage  ,      ϵͳ������������ѹ
////
//const uint16_t loadselect ={0x81};               //load select/load loadselectmode; 0x81 power-mode 0x01 current mode, normally use 0x81 ,if design capacity > 8500mAh change to 0x01
//#define BATTERY_3200MAH
//uint32_t Frack_Num;
//
///**************************************************************************************
//**************************************************************************************/
//
//
//#ifdef BATTERY_3200MAH
////3200mAH
//unsigned short const Designcapacity={0x0C80};     //Design capacity=2500mAh   ���ֵ���ŵ�ص�ʹ�û�仯
//unsigned short const DesignEnergy={0x2E40};       //Design Energy=2500*3.8mWh; Design Energy=Design capacity*3.7 for 4.2V battery,Design Energy=Design capacity*3.8 for 4.35V battery,
//#define   FCC_SIZE   950
//
////220   ���
//const uint16_t Taperrate ={0x0140};              //Taper rate=250,taper rate=Design Capacity*10/taper current mA;(2500*10/220 = 125) a little higher than charger taper current(~>20mA)
////Taperrate  ��һ������ֹ�ж�����
//
////subclass 81
////�ŵ���� ����62mA  2500*10 / 62.5ma = 400
//const uint16_t Dsgcurrentthreshold ={0x190};    // 50000/167=299mA,  Dsg current threshold(num)=500, Dsg current threshold(mAh)=Design capacity*10/Dsg current threshold(num)=80mA
//
//
////������һ������������� 100mA 2500*10/100 = 250
//const uint16_t Chgcurrentthreshold ={0xfa};    //Chg current threshold(num)=500, Chg current threshold(mAh)=Design capacity*10/Chg current threshold(num)=80mA,must smaller than charger taper current
//
//
////����͹��ĵ��� 2500 * 10 / 50ma = 500
//const uint16_t Quitcurrent ={0x01f4};  //{0x03E8};     //Quit current threshold(num)=1000,Quit current threshold(mAh)=Design capacity*10/Quit current threshold(num)=40mA
//
//#endif
//
//
//
//
//
//
//
//
//
///**************************************************************************************
//**************************************************************************************/
//
//
//#ifdef BATTERY_3200MAH
//const uint16_t  Qmax ={0x4000};
//
//const uint16_t  Ra[] ={0x66, 0x66, 0x63, 0x6b, 0x48, 0x3b, 0x3e, 0x3f, 0x35, 0x2f, 0x3c, 0x46, 0x8c, 0x171, 0x24c};
////const uint16_t  Ra[] ={0x34, 0x34, 0x32, 0x36, 0x2d, 0x32, 0x47, 0x52, 0x50, 0x4b, 0x57, 0x52, 0x72, 0x108, 0x1a4};
//#endif


#define DESIGN_CAPACITY_MAH  3300   // ֱ���������ʱ (mAh)
#define NOMINAL_VOLTAGE_MV   3600   // 3.7V ���
#define TAPER_CURRENT_MA     64    // ����ֹ���� 220mA
#define DISCHARGE_CURRENT_MA 62     // �ŵ�������� 62mA
#define CHARGE_THRESHOLD_MA  64    // ������� 100mA
#define QUIT_CURRENT_MA      50     // �˳��͹���ģʽ���� 50mA

// **��ػ�������**
const uint16_t Designcapacity = (uint16_t)(DESIGN_CAPACITY_MAH);  // 3000mAh
const uint16_t DesignEnergy   = (uint16_t)(DESIGN_CAPACITY_MAH * (NOMINAL_VOLTAGE_MV / 1000.0));  // ����mWh
const uint16_t terminatevoltage = 3000;  // ϵͳ��͹�����ѹ 3000mV
const uint16_t loadselect = 0x81;  // ����ģʽ (0x81: ����ģʽ, 0x01: ����ģʽ)

// **�����Ʋ���**
const uint16_t Taperrate = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / TAPER_CURRENT_MA);
const uint16_t Dsgcurrentthreshold = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / DISCHARGE_CURRENT_MA);
const uint16_t Chgcurrentthreshold = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / CHARGE_THRESHOLD_MA);
const uint16_t Quitcurrent = (uint16_t)((DESIGN_CAPACITY_MAH * 10) / QUIT_CURRENT_MA);
uint32_t Frack_Num;
// **���� (Ra) �� Qmax**
const uint16_t Qmax = 0x4000;  // �̶�ֵ
const uint16_t Ra[] = {0x66, 0x66, 0x63, 0x6b, 0x48, 0x3b, 0x3e, 0x3f, 0x35, 0x2f, 0x3c, 0x46, 0x8c, 0x171, 0x24c};

/**************************************************************************************
**************************************************************************************/


/**************************************************************************************
**************************************************************************************/

/**************************************************************************************
**************************************************************************************/


/********************************************************
��subaddr �����ַд��cmd���ָ�д����ָ��)
 ********************************************************/
uint32_t bq_cmdWrite(uint8_t subaddr, uint8_t cmd)
{
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&cmd,1,0xff);
}
/*************************************
  *************************************/
uint32_t bq_read(uint8_t *pBuf,  uint8_t addr, uint8_t len)
{
   return HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, addr, len, pBuf, len, 0xfff);

}
/********************************************************
*********************************************************/
uint32_t bq_MemoryWrite(uint8_t subaddr, uint16_t data )
{
    uint8_t Mdata=(data>>8);
    uint8_t Ldata=(data&0xFF);
            HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr,I2C_MEMADD_SIZE_8BIT,&Mdata,1,0xff);
    return HAL_I2C_Mem_Write(&hi2c1,BQ2744_ADDRESS,subaddr+1,I2C_MEMADD_SIZE_8BIT,&Ldata,1,0xff);
}
/********************************************************
*********************************************************/
uint8_t GetCheckSum(uint8_t  addr)
{
    uint8_t checksum = 0, i;
    uint8_t rtBuf[2];

    for (i = 0; i < 32; i++)
    {
        rtBuf[0] = 0;
        bq_read(rtBuf, addr, 1);
        checksum = checksum + rtBuf[0];
        addr++;
    }

    checksum = 0xFF - checksum;
    return checksum;
}
/********************************************************
*********************************************************/
void  bq_Read_Ta_and_Qmax(void)
{

	unsigned short  qmax=0x00;
	uint8_t  Ra_table[32]= {0x0A};
	unsigned short  cap=0x00;

	uint8_t buf[1];
 	uint8_t tbuf[2];

	uint8_t i;
        tbuf[0]=0;
        tbuf[1]=0;
        Ra_table[31]='\r';
        Ra_table[30]='\n';
	bq_cmdWrite(0x00,0x00);   //Places the device in UNSEALED access mode.
	bq_cmdWrite(0x01,0x80);

	bq_cmdWrite(0x00,0x00);
	bq_cmdWrite(0x01,0x80);


	bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

	bq_cmdWrite(0x3e,0x52);     //ѡ��0x52����      //access the state subclass (decimal 82, 0x52 hex)

	bq_cmdWrite(0x3f,0x00);         //use offset 0x00 for offsets 0 to 31


	bq_read(buf, 0x40, 2);
	qmax = (buf[0] << 8) | buf[1];  //�ߵ�λ����

	bq_read(buf, 0x4A, 2);
	cap = (buf[0] << 8) | buf[1]; //�ߵ�λ����


	bq_cmdWrite(0x3e,0x59);//ѡ��0x59����
	bq_cmdWrite(0x3f,0x00);

	for(i=0; i<30; i++)
	{
		bq_read(buf, 0x40+i, 1);
		Ra_table[i] = buf[0]; //�ߵ�λ����
	}
      

        Frack_Num++;
        tbuf[0]=Frack_Num>>24;
        tbuf[0]=((Frack_Num>>16)&0xFF);
        tbuf[0]=((Frack_Num>>8)&0xFF);
        tbuf[0]=(Frack_Num&0xFF);
//	bq_cmdWrite(0x3e,0x40);//ѡ��0x59����
//	bq_cmdWrite(0x3f,0x00);
//
//
//        bq_read(buf, 0x40, 1);
//        HAL_UART_Transmit(&huart2,buf,1,0xff);
//        bq_read(tbuf, 0x41, 1);
//        HAL_UART_Transmit(&huart2,tbuf,1,0xff);

}

/********************************************************
*********************************************************/
void bq_fullReset(void)
{
	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))   //Places the device in UNSEALED access mode.
	{
	    return ;
	}

	if (bq_cmdWrite(0x00,0x00) || bq_cmdWrite(0x01,0x80))
	{
	    return ;
	}

	if (bq_cmdWrite(0x00,0x41) || bq_cmdWrite(0x01,0x00)) //Performs a full device reset.
	{
	    return;
	}
}

/********************************************************
*********************************************************/

void bq_CONFIG_subclass82(void)
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x52); //ѡ��0x52����      //access the state subclass (decimal 82, 0x52 hex)
    bq_cmdWrite(0x3f, 0x00); //use offset 0x00 for offsets 0 to 31


    bq_MemoryWrite(0x40, Qmax);

    bq_MemoryWrite(0x50, terminatevoltage); //terminatevoltage ��ֹ��ѹ(ϵͳ�ܹ��������е���͵�ѹ)

    bq_cmdWrite(0x45, loadselect);

    bq_MemoryWrite(0x4A, Designcapacity); //�������  mAh

    bq_MemoryWrite(0x4C, DesignEnergy); //mWh

    bq_MemoryWrite(0x5B, Taperrate);

    checksum = GetCheckSum(0x40);

    bq_cmdWrite(0x60, checksum); //0xba  checksum

}
void bq_CONFIG_subclass81(void) //��ŵ���ֵ����
{
    uint8_t checksum = 0;
    bq_cmdWrite(0x3e, 0x51); //ѡ��0x51����
    bq_cmdWrite(0x3f, 0x00);
    bq_MemoryWrite(0x40, Dsgcurrentthreshold); //�ŵ����
    bq_MemoryWrite(0x42, Chgcurrentthreshold); //�����������������жϱ�׼
    bq_MemoryWrite(0x44, Quitcurrent); //��̬��Դ
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum); //0x5e
}

void bq_CONFIG_subclass89(void)//���������
{
    uint8_t checksum = 0, i = 0;
    bq_cmdWrite(0x3e, 0x59); //ѡ��0x59����
    bq_cmdWrite(0x3f, 0x00);
    for (i = 0; i < 15; i++) {
        bq_MemoryWrite(0x40 + i * 2, Ra[i]);
    }
    checksum = GetCheckSum(0x40);
    bq_cmdWrite(0x60, checksum);
}

/********************************************************
����0,  ��������ģʽ���˳��ɹ�, ����1, ��������ʧ��, ����0xFE, ��ʱ
*********************************************************/
uint32_t bq_Config(void) {
    uint8_t rtBuf[2];
    uint16_t value;
    uint32_t result;

    static uint32_t m_tryCnt = 0;

    // **��ֹ��������ѭ��**��������� 10 ��ʧ�ܣ��򷵻س�ʱ����
    if (m_tryCnt++ > 10) {
        m_tryCnt = 0;
        LOG("BQ27441 ���ó�ʱ��\n");
        return 0xFE;
    }

    // **��� (UNSEAL) BQ27441**
    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) {
        LOG("��� BQ27441 ʧ�� (���� 1)\n");
        return 1;
    }
    if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) {
        LOG("��� BQ27441 ʧ�� (���� 2)\n");
        return 2;
    }

    // **��������ģʽ (CFGUPDATE)**
    if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00)) {
        LOG("��������ģʽʧ�ܣ�\n");
        return 3;
    }

    // �ȴ� 1ms��ȷ������ģʽ��Ч
    HAL_Delay(1);
    result = 1;

    // **�ȴ� BQ27441 ���� CFGUPDATE ģʽ**
    while (result) {
        result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
        if (result == 0) {
            value = (rtBuf[1] << 8) | rtBuf[0];
            result = (value & 0x10);  // ��� CFGUPDATE ��־λ
        }
    }
    LOG("���� CFGUPDATE ģʽ�ɹ���\n");

    // **ʹ�� BlockData RAM ����**
    bq_cmdWrite(0x61, 0x00);

    // **ִ�� BQ27441 ���� (Subclass 82)**
    bq_CONFIG_subclass82();

    // **�˳�����ģʽ��Ӧ������**
    LOG("ִ����λ���˳�����ģʽ...\n");
    bq_cmdWrite(0x00, 0x42);  // ��λ (Soft Reset)
    bq_cmdWrite(0x01, 0x00);
    HAL_Delay(10);

    // ���� `0x00 0x00` ָ�ʹоƬ�ص���������ģʽ
    bq_cmdWrite(0x00, 0x00);
    bq_cmdWrite(0x01, 0x00);

    LOG("BQ27441 ������ɣ�\n");
    return 0;
}

/********************************************************
����0, ���óɹ�, ����1, ����ʧ��  ��������, ���ó�ʱ
*********************************************************/
uint32_t bq_ConfigRead(void) {
    uint8_t rtBuf[2] = {0};
    uint16_t value;
    uint32_t result;

    // ��ȡ BQ27441 `Flags(0x06)`����� CFGUPDATE ״̬
    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result != 0) {
        LOG("��ȡ Flags ʧ�ܣ�\n");
        return 0xFF;
    }

    value = (rtBuf[1] << 8) | rtBuf[0];

    // **��� CFGUPDATE ��Ȼ�� 1��˵���������ڽ���**
    if (value & 0x10) {
        LOG("CFGUPDATE ��Ȼ��λ�����ÿ���δ��ɡ�\n");
        return 1;
    }

    // **���������ɣ����� `SEAL` ָ����·�װ�豸**
    LOG("CFGUPDATE ��λ��ִ�� SEAL ����...\n");
    bq_cmdWrite(0x00, 0x20);
    bq_cmdWrite(0x01, 0x00);

    return 0;
}


/********************************************************
 ������
********************************************************/
uint32_t bq_Rdarg(uint16_t *volt, int16_t *avgCur, uint16_t *soc, uint16_t *fcc)
{
  uint8_t  lrtBuf[1];
  uint8_t  mrtBuf[1];

  uint16_t  value;
  uint16_t ret1;
  uint16_t ret2;


  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_LSB, 1, lrtBuf, 1, 0xfff);   //volt  Ϊ��ѹ
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_VOLT_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *volt = value;

  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_LSB, 1, lrtBuf, 1, 0xfff);     //avgCur Ϊƽ������
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_AI_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *avgCur = value;
  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_LSB, 1, lrtBuf, 1, 0xfff);    //soc Ϊ�����ٷݱ�
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_SOC_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *soc = value;
  }
  else
  {
    return 0xF1;
  }

  ret1=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_LSB, 1, lrtBuf, 1, 0xfff);   //FCCΪ������
  ret2=HAL_I2C_Mem_Read(&hi2c1, BQ2744_ADDRESS, bq27421CMD_FCC_MSB, 1, mrtBuf, 1, 0xfff);
  if(0 == ret1 && ret2 == 0)
  {
    value = (mrtBuf[0] << 8) |lrtBuf[0];

    *fcc = value;
  }
  else
  {
    return 0xF1;
  }
    return ret1;
}
/********************************************************
����1 ��Ҫ���� ����0 �����ù� ��������ֵ����
********************************************************/
uint16_t bq_ITPOR(void) {
    uint8_t rtBuf[2] = {0};
    uint32_t ret;

    // **��ȡ Flags(0x06)����� IT (Initial Power On Reset) ��־**
    ret = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 1);
    if (ret != 0) {
        LOG("��ȡ IT (Initial Power On) ʧ�ܣ�\n");
        return 0xFF;
    }

    // **��� `0x20` λ�Ƿ��� 1���ж��Ƿ���Ҫ��������**
    if (rtBuf[0] & 0x20) {
        LOG("ITPOR ��λ����Ҫ�������� BQ27441\n");
        return 1;
    }

    LOG("ITPOR δ��λ��BQ27441 ��������\n");
    return 0xAA;  // 0xAA ���������Ѿ���ɣ������޸�
}
uint8_t getBoxBattery(void)
{
  uint16_t volt,soc,fcc ;
  int16_t avgCur;

  if(bq_ITPOR() !=0xAA )
  {
    HAL_Delay(500);
    bq_Config();
    if(bq_ConfigRead() != 0)
      return 0xFE;

  }

  bq_Rdarg(&volt, &avgCur, &soc, &fcc);

  return soc&0xFF;
}
uint8_t Batter_Gauge_Test(){
  uint8_t data;
  uint16_t volt,soc,fcc ;
  int16_t avgCur;
  volt=bq_ITPOR();


        if(volt == 1 )
        {
          bq_Config();
          if(bq_ConfigRead() != 0)
          return 2;
        }




  bq_Read_Ta_and_Qmax();
  bq_Rdarg(&volt, &avgCur, &soc, &fcc);






  return 1;

}

/********************************************************
����0,  ��������ģʽ���˳��ɹ�, ����1, ��������ʧ��, ����0xFE, ��ʱ
*********************************************************/
uint32_t bq_Config_nww(void)
{
  uint8_t  rtBuf[2];
  uint16_t value;
  uint32_t result;

  static uint8_t m_tryCnt = 0;

  if (m_tryCnt++ > 10)
  {
    m_tryCnt = 0;
    return 0xFE;
  }

  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80)) //Places the device in UNSEALED access mode.
  {
    return 1;
  }

  if (bq_cmdWrite(0x00, 0x00) || bq_cmdWrite(0x01, 0x80))
  {
    return 2;
  }

  if (bq_cmdWrite(0x00, 0x13) || bq_cmdWrite(0x01, 0x00))
  {
    return 3;
  }

  HAL_Delay(1);
  result=1;
  while(result){
    result = bq_read(rtBuf, bq27421CMD_FLAG_LSB, 2);
    if (result == 0)
    {
      value = (rtBuf[1] << 8) | rtBuf[0];
      result=(value & 0x10);

    }

  }
  bq_cmdWrite(0x61,0x00);       // enables BlockData() to access to RAM.

  //bq_CONFIG_subclass52();

  return result;
}
uint16_t read_DesignCapacity() {
    uint8_t buf[2] = {0};
    uint16_t capacity = 0;

    // ��ȡ 0x4A ��������Ĵ�����2 �ֽڣ�
    if (bq_read(buf, 0x4A, 2) == 0) {
        capacity = (buf[1] << 8) | buf[0];  // **���ֽ���ǰ**
        LOG("��ȡ��������ɹ�: %d mAh (0x%04X)\n", capacity, capacity);
    } else {
        LOG("��ȡ�������ʧ��!\n");
    }

    return capacity;
}

void main_app(void) {
    // ��ȡ ITPOR ��־���ж��Ƿ���Ҫ�������õ�ع���оƬ
    if (bq_ITPOR() == 1) {
        LOG("��ع���оƬ��Ҫ����...\n");

        // �ӳ� 1.1 �룬ȷ�� BQ27441 �����ȶ�״̬
        HAL_Delay(1100);

        // ��������ģʽ����� BQ27441 �����ùؼ�����
        LOG("��ʼ BQ27441 ����...\n");
        uint32_t config_status = bq_Config();
read_DesignCapacity();
        // ��������Ƿ�ɹ�
        if (config_status != 0) {
            LOG("���� BQ27441 ʧ�ܣ�����ֵ: %d\n", config_status);
            return;
        }
        LOG("BQ27441 ���óɹ���\n");

        // ��ȡ����״̬��ȷ���Ƿ��������
        HAL_Delay(10);
        uint32_t config_read_status = bq_ConfigRead();

        // ȷ�����óɹ�
        if (config_read_status != 0) {
            LOG("��ȡ����ʧ�ܣ�����ֵ: %d\n", config_read_status);
        } else {


            LOG("BQ27441 ���ö�ȡ�ɹ���\n");
        }

    } else {
        LOG("��ع���оƬ�����ã������������á�\n");

    }
}

