
#include "main.h"
uint8_t BQ25895Reg[21];
uint8_t BQ25895TempData[1];
extern I2C_HandleTypeDef hi2c1;
void BQ25895_Init(void) {

  CHG_CE(0);                 // �رճ��ʹ��
  //BQ25895_Write(0x02, 0x7C); // ���üĴ�??0x02����AUTO_DPDM_EN����
  BQ25895_Write(0x02, 0xFC); // ����ADC
  BQ25895_Write(0x03, 0x1E); // OTG�رգ���Сϵͳ��ѹ???��??3.5V
  BQ25895_Write(0x04, 0x20); // ���ó�����??4096mA
  // BQ25895_Write(0x05, 0x10); // ���ó����???����Ϊ64mA
  BQ25895_Write(0x05, 0x13); // ���ó����ֹ����Ϊ150mA
  BQ25895_Write(0x07, 0x8D); // �رճ�綨ʱ??
  BQ25895_Write(0x00, 0x3F); // 3.25A
}

void BQ25895_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, ReadAddr, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 1);
}
// ȫ�ֱ��������ڱ�� DMA ����״̬
extern volatile bool dma_transfer_complete;
void BQ25895_MultiRead(uint8_t *pBuffer) {

  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  HAL_I2C_Mem_Read_DMA(&hi2c1, BQ25895Address, 0x00, I2C_MEMADD_SIZE_8BIT,
                       pBuffer, 20);
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ; // �ȴ��������
  }
}

void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
  dma_transfer_complete = false;
  while (hi2c1.State != HAL_I2C_STATE_READY) {
    ;
  }
  BQ25895TempData[0] = WriteData;
  HAL_I2C_Mem_Write_DMA(&hi2c1, BQ25895Address, WriteAddr, I2C_MEMADD_SIZE_8BIT,
                        BQ25895TempData, 1);
  while (!dma_transfer_complete && hi2c1.State != HAL_I2C_STATE_READY) {
    ; // �ȴ��������
  }
}




#define VBAT_REG_LSB 0x0E
#define VBAT_REG_MSB 0x0F

float read_battery_voltage(uint8_t * BQ25895Reg) {

  // �ϳ� 16 λ����
  uint16_t vbat_raw=(BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

  // ת��Ϊ��ѹ (mV)
  return vbat_raw * 20.0/1000;
}
uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
extern uint8_t Flag_400ms;
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
  case 3: // Charge Termination Done
    fully_charged = 1;
    charging = 0;
    working = 0;
    break;
  case 0: // Not Charging
    working = 1;
    charging = 0;
    fully_charged = 0;
    break;
  }
}