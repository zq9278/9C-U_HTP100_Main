
#include "main.h"
uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {
    osDelay(10);
    CHG_CE(1);  // �رճ��ʹ�ܣ�׼������

//    // 0x00 - ���������������Ϊ 3.25A���ر� Watchdog
//    BQ25895_Write(0x00, 0x3F);  // 3A���� + Watchdog Timer disable
//
//    // 0x01 - ���� Boost �¶ȱ���,�����ѹƫ��Ϊ 0
//    BQ25895_Write(0x01, 0xE0);  //
//
//    // 0x02 - �ر� DPDM ��� + ���� ADC1sһ�Σ�Boost Ƶ�� 1.5MHz���ر���������Ż��㷨���ر� QC ������֣��ر� MaxCharge ����
//    BQ25895_Write(0x02, 0xC0);  // [7] EN_HIZ=1�ر�����ʹ��ʱ���� + EN_ADC=1����ADC
//
//    // 0x04 - ���ó�����Ϊ 2.048A
//    BQ25895_Write(0x04, 0x20);  // Charge Current = 2048mA
//
//    // 0x05 - ������ֹ����Ϊ 64mA
//    BQ25895_Write(0x05, 0xF1);
//
//    // 0x06 - ���ó���ѹΪ 4.2V
//    BQ25895_Write(0x06, 0x5C);  // Charge Voltage = 4.2V
//
//    // 0x07 - ���ó�綨ʱ��
//    BQ25895_Write(0x07, 0x89);  // DIS_TIMER=1
//
//    // 0x08 - ����Ĭ�ϣ��¶ȷ�Χ���ƣ�
//
//    // 0x09 - ��ʹ�� JEITA��OTG��Boost��
//    BQ25895_Write(0x09, 0x00);
//    // 0x03 - ������Сϵͳ��ѹΪ 3.5V��VINDPM��
//    BQ25895_Write(0x03, 0x1A);  // VINDPM = 3.5V





    BQ25895_Write(0x02, 0xFC); // ����ADC

    BQ25895_Write(0x04, 0x20); // ���ó�����2048mA
    BQ25895_Write(0x05, 0x11); // ���ó����ֹ����Ϊ64mA
    BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ??
    BQ25895_Write(0x06, 0x94);  // ����ѹ 4.2V
    BQ25895_Write(0x00, 0x3F); // 3.25A
    BQ25895_Write(0x03, 0x1E); // OTG�رգ���Сϵͳ��ѹ???��??3.5V
    CHG_CE(0);  // �򿪳��
}

void BQ25895_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    BQ25895_Read_IT(ReadAddr, pBuffer, 1);
}
void BQ25895_MultiRead(uint8_t *pBuffer) {
    BQ25895_Read_IT(0x00, pBuffer, 20);  // ��ȡ 20 �ֽ�
}
void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
    BQ25895_Write_IT(WriteAddr, WriteData);
}
HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t temp = WriteData;
    // ��ȡ I2C ��������ȷ�����߲����ͻ
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ���������ź�������ֹ�ϴβ���Ӱ��
        xSemaphoreTake(xI2CCompleteSem, 0);

        // ���� I2C д���� (�ж�ģʽ)
        status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, &temp, 1);
        if (status == HAL_OK) {
            // �ȴ�������� (����ʱ)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // ����ɹ�
            } else {
                // ��ʱ����
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // �ͷ� I2C ������
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // ���߷�æ
    }
    return status;
}
HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // ��ȡ I2C ����������ֹ���߳�ͻ
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // ���������ź�������ֹ�ϴβ���Ӱ��
        xSemaphoreTake(xI2CCompleteSem, 0);

        // ���� I2C ������ (�ж�ģʽ)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        if (status == HAL_OK) {
            // �ȴ�������� (����ʱ)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // ����ɹ�
            } else {
                // ��ʱ����
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // �ͷ� I2C ������
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // ���߷�æ
    }
    return status;
}

float read_battery_voltage(uint8_t * BQ25895Reg) {
  // �ϳ� 16 λ����
  uint16_t vbat_raw=(BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

  // ת��Ϊ��ѹ (mV)
  return vbat_raw * 20.0/1000;
}
uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
void UpdateChargeState_bq25895(void) {
  BQ25895_MultiRead(BQ25895Reg);
//  float battery=read_battery_voltage(BQ25895Reg);
//  vTaskDelay(100);
//  printf("��ѹ%f;",battery);
//  float bat=(4.199-(float)battery)/(0.00078);//�����ٷֱ�ȥ��%
//  printf("����%f;",bat);
//  printf("�ٷֱ�%f\n",bat/42);
   CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;
  // ��ʼ��״̬Ϊ false
  // ���� CHRG_STAT ����״̬
  switch (CHRG_STAT) {
  case 1: // Pre-charge
  case 2: // Fast Charging
    if(fully_charged==0){// ��������ʱ�̣�����2��3״̬�����л�
      charging = 1;
      fully_charged = 0;
      working = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//�ر���Ļ
        vTaskSuspend(deviceCheckHandle);

    }
    break;
//  case 3: // Charge Termination Done
//    fully_charged = 1;
//    charging = 0;
//    working = 0;
//    break;
  case 0: // Not Charging
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//������Ļ
    working = 1;
    charging = 0;
    fully_charged = 0;
    break;
  }
}
//void bq25895_reinitialize_if_vbus_inserted(void) {
//    if (is_charging_flag){
//        is_charging_flag=0;
//        static uint8_t last_vbus_status = 0x00;
//        // ��ȡ VBUS ״̬�Ĵ��� 0x0B
//        uint8_t vbus_status ;
//        BQ25895_Read(0x0B,&vbus_status);
//        // ��� VBUS �Ƿ���� (Bit7 = 1 ��ʾ����)
//        if (((vbus_status & 0x80) || (vbus_status == 0x16)) && !(last_vbus_status & 0x80)) {
//            BQ25895_Init(); // ���³�ʼ��
//            LOG("������Ѳ��룬���³�ʼ�� bq25895...\n");
//
//        }
//        last_vbus_status = vbus_status;
//    }
//}
void bq25895_reinitialize_if_vbus_inserted(void) {
    static uint8_t last_vbus_status = 0x00;  // �洢��һ�ε� VBUS ״̬
    uint8_t vbus_status;

    BQ25895_Read(0x0B, &vbus_status);

    // ��� VBUS ���� (����״̬��δ���� -> ����ʱִ��)
    if (((vbus_status & 0x80) || (vbus_status == 0x16)) &&
        !((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOG("������Ѳ��룬���³�ʼ�� bq25895...\n");

        BQ25895_Init();
    }

    // ��� VBUS �γ� (����״̬�Ӳ��� -> �γ�ʱִ��)
    if (!(vbus_status & 0x80) && !(vbus_status == 0x16) &&
        ((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {

        LOG("������Ѱγ��������ʼ�����...\n");
    }

    last_vbus_status = vbus_status;  // ����״̬
}


//    uint8_t reg00, reg0D;
//    BQ25895_Read(0x00, &reg00);
//    BQ25895_Read(0x0D, &reg0D);
//    LOG("IINLIM (�������): 0x%02X, VINDPM (�����ѹ): 0x%02X\n", reg00, reg0D);
//    uint8_t reg0B, reg0C;
//    BQ25895_Read(0x0B, &reg0B);
//    BQ25895_Read(0x0C, &reg0C);
//    LOG("VBUS״̬: 0x%02X, ���״̬: 0x%02X\n", reg0B, reg0C);
//    uint8_t reg02, reg0E;
//    BQ25895_Read(0x02, &reg02);
//    BQ25895_Read(0x0E, &reg0E);
//    LOG("�¶ȵ���: 0x%02X, �ȱ���״̬: 0x%02X\n", reg02, reg0E);
//    uint8_t reg03;
//    BQ25895_Read(0x03, &reg03);
//    LOG("������: 0x%02X\n", reg03);
//    uint8_t reg11;
//    BQ25895_Read(0x11, &reg11);
//    LOG("VBUS ��ѹ: 0x%02X\n", reg11);
//    uint8_t reg09, reg0Ba;
//    BQ25895_Read(0x03, &reg09);
//    BQ25895_Read(0x0B, &reg0Ba);
//
//    if (!(reg09 & 0x30)) {  // �����类����
//        LOG("��类����رգ����¿������...\n");
//        BQ25895_Write(0x03, 0x30);  // �����������
//    }
//
//    if (!(reg0Ba & 0x80)) {  // ��� VBUS ����
//        LOG("VBUS �Ͽ����ȴ����²���...\n");
//    }
//}
