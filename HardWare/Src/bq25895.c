
#include "main.h"
uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {
//    BQ25895_Write(0x00, 0x30);  // ��λ���мĴ���
//    HAL_Delay(1000);  // ȷ��оƬ��ɸ�λ
//
//    // ��ȡ�Ĵ�����ȷ����λ���
//    uint8_t check_reset;
//    BQ25895_Read(0x00, &check_reset);
//    if (check_reset != 0x30) {
//        LOG("��λʧ�ܣ�����δд��ɹ�\n");
//        return;
//    }
//
//    // ����д��Ĵ���
//    //BQ25895_Write(0x00, 0x3A);  // 3.25A �����������
//    BQ25895_Write(0x00, 0x28);  // ��������������Ƶ� 2A
//    BQ25895_Write(0x04, 0x28);  // ���ó�����Ϊ 2A
//    BQ25895_Write(0x06, 0x96);  // ����ѹ 4.2V
//    BQ25895_Write(0x05, 0x07);  // ��ֹ���� 250mA
//    BQ25895_Write(0x09, 0x40);  // �Զ����
//    BQ25895_Write(0x0B, 0x80);  // ���� OTG
//    BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ******��Ҫ********
//    BQ25895_Write(0x03, 0x30);  // ���������Զ����
//    CHG_CE(0);
    CHG_CE(0);                 // �رճ��ʹ��
    //BQ25895_Write(0x02, 0x7C); // ���üĴ�??0x02����AUTO_DPDM_EN����
    BQ25895_Write(0x02, 0xFC); // ����ADC
    BQ25895_Write(0x03, 0x1E); // OTG�رգ���Сϵͳ��ѹ???��??3.5V
    BQ25895_Write(0x04, 0x20); // ���ó�����2048mA
    // BQ25895_Write(0x05, 0x10); // ���ó����???����Ϊ64mA
    BQ25895_Write(0x05, 0x11); // ���ó����ֹ����Ϊ64mA
    BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ??
    BQ25895_Write(0x06, 0x94);  // ����ѹ 4.2V
    BQ25895_Write(0x00, 0x3F); // 3.25A
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
    }
    break;
//  case 3: // Charge Termination Done
//    fully_charged = 1;
//    charging = 0;
//    working = 0;
//    break;
  case 0: // Not Charging
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
