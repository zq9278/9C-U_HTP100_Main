
#include "main.h"

uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;

void BQ25895_Init(void) {
    CHG_CE(1);  // �رճ��ʹ�ܣ�׼������
    //BQ25895_Write(0x14, 0x80); // д1��REG_RSTλ�����������λ
    //osDelay(500);
    //BQ25895_Write(0x02, 0x30); // ����ADC
    BQ25895_Write(0x04, 0x40); // ���ó�����2048mA
    BQ25895_Write(0x05, 0x10); // ���ó����ֹ����Ϊ64mA
    BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ
    BQ25895_Write(0x08, 0xe7); // �������ߵ���
    BQ25895_Write(0x00, 0x3F); // 3.25A�����������
    //BQ25895_Write(0x03, 0x12); // OTG�رգ���Сϵͳ��ѹ3.1V
    osDelay(100);
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

float read_battery_voltage(uint8_t *BQ25895Reg) {
    // �ϳ� 16 λ����
    uint16_t vbat_raw = (BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

    // ת��Ϊ��ѹ (mV)
    return vbat_raw * 20.0 / 1000;
}
void BQ25895_AutoRecover(void) {
    BQ25895_MultiRead(BQ25895Reg);  // һ�ζ�ȡ0x00~0x14
    BQ25895_MultiRead(BQ25895Reg);  // һ�ζ�ȡ0x00~0x14
//
    uint8_t reg0b = BQ25895Reg[0x0B];
   // uint8_t chrg_stat = (reg0b >> 3) & 0x03;
    uint8_t chrg_stat = (reg0b & 0x18) >> 3;

    if (chrg_stat == 0x03) {
        LOG(" �����ɣ�BQ25895 ��⵽ Charge Termination Done\n");
        return;
    }

    uint8_t reg0c ;
    BQ25895_Read(0x0C,&reg0c);
    osDelay(1);
    BQ25895_Read(0x0C,&reg0c);
    bool fault = false;

    // �����������Ͳ������־
    if (reg0c & (1 << 3)) {
        LOG(" ��ع��ϣ�BAT_FAULT = 1����ؿ��ܶϿ���VBAT���ߣ�\n");
        fault = true;
    }

    uint8_t chrg_fault = (reg0c >> 4) & 0x03;
    if (chrg_fault != 0x00) {
        switch (chrg_fault) {
            case 0x01:
                LOG("�����ϣ������ѹ�쳣��VBUS��VBAT���⣩\n");
                break;
            case 0x02:
                LOG(" �����ϣ����±���\n");
                break;
            case 0x03:
                LOG(" �����ϣ���ȫ��ʱ����ʱ\n");
                break;
        }
        fault = true;
    }

    if (reg0c & (1 << 7)) {
        LOG(" ���Ź���ʱ��WATCHDOG_FAULT = 1����Ҫ����I2Cͨ�Ż�رտ��Ź���\n");
        fault = true;
    }

    // ������ڹ��ϣ�������������߼�
    if (1) {
        BQ25895_Init();
        LOG(" �������������\n");
    }
}


uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
static int charge_action_done = 0;
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
            if (!charge_action_done) {  // ִֻ��һ��
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // �ر���Ļ
                close_mianAPP();
                vTaskSuspend(deviceCheckHandle);
                charge_action_done = 1;  // ����Ѵ���
            }
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//�ر���Ļ
//            close_mianAPP();
//            vTaskSuspend(deviceCheckHandle);
            if (fully_charged == 0) {// ��������ʱ�̣�����2��3״̬�����л�
                charging = 1;
                fully_charged = 0;
                working = 0;
            } else if (fully_charged == 1) {
                fully_charged = 1;
                charging = 0;
                working = 0;
            }
            break;
        case 3: // Charge Termination Done
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//�ر���Ļ
            vTaskSuspend(deviceCheckHandle);
            fully_charged = 1;
            charging = 0;
            working = 0;
            charge_action_done = 0; // ״̬�л��������´���ִ��һ��
            break;
        case 0: // Not Charging
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//������Ļ
            working = 1;
            charging = 0;
            fully_charged = 0;
            charge_action_done = 0; // ״̬�л��������´���ִ��һ��
            break;
    }
    uint8_t PG_STAT = (BQ25895Reg[0x0B] >> 2) & 0x01;

    if (PG_STAT) {
        LOG(" PG_STAT = 1��VBUS ��Դ������Power Good��\n");
//        if ((BQ27441.Voltage < 4170)&&( BQ27441.AvgCurrent<100)&&(fully_charged==0)) {//&&(0<BQ27441.AvgCurrent )
//            LOG("low voltage or current\n");
//            BQ25895_Init();
//        }
    } else {
        LOG(" PG_STAT = 0��VBUS �����쳣��δ����\n");
    }

}

uint8_t charging_flag = 0;
void bq25895_reinitialize_if_vbus_inserted(void) {
    static uint8_t last_vbus_status = 0x00;  // �洢��һ�ε� VBUS ״̬
    uint8_t vbus_status;

    BQ25895_Read(0x0B, &vbus_status);

    // ��� VBUS ���� (����״̬��δ���� -> ����ʱִ��)
    if (((vbus_status & 0x80) || (vbus_status == 0x16)) &&
        !((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOG("������Ѳ��룬���³�ʼ�� bq25895...\n");

//        BQ25895_Init();
        charging_flag=1;
    }

    // ��� VBUS �γ� (����״̬�Ӳ��� -> �γ�ʱִ��)
    if (!(vbus_status & 0x80) && !(vbus_status == 0x16) &&
        ((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {

        LOG("������Ѱγ��������ʼ�����...\n");
        charging_flag=0;
       // NVIC_SystemReset();
    }

    last_vbus_status = vbus_status;  // ����״̬
}
