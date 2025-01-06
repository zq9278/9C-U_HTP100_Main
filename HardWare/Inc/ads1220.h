/*
 * @Author: zhangqi 
 * @Date: 2024-12-30 17:40:27 
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 17:01:42
 */
#ifndef __ADS1220_H
#define __ADS1220_H
#include "stm32g0xx_hal.h"
// ����ADS1220��SPI����
#define ADS1220_CMD_RESET 0x06      // ��������
#define ADS1220_CMD_START_SYNC 0x08 // ��ʼ/ͬ������
#define ADS1220_CMD_RDATA 0x10      // ��ȡ��������
#define ADS1220_CMD_POWERDOWN 0x02  // �͹���ģʽ����
#define ADS1220_CMD_RREG 0x23       // ��ȡ�Ĵ�������

// ����ADS1220�ļĴ�����ַ
#define ADS1220_REG_CONFIG0 0x00 // ���üĴ���0
#define ADS1220_REG_CONFIG1 0x01 // ���üĴ���1
#define ADS1220_REG_CONFIG2 0x02 // ���üĴ���2
#define ADS1220_REG_CONFIG3 0x03 // ���üĴ���3

// ���崫��������
#define SENSITIVITY 0.365           // ������ (mV/V)ԽС��ֵԽ��
#define MAX_WEIGHT 2.0            // ���������� (kg)
#define EXCITATION_VOLTAGE 3.3    // ������ѹ (V)
#define GAIN 128                    // �Ŵ���
#define VREF 3.3               // ��׼��ѹ
extern SPI_HandleTypeDef hspi2;

// SPIƬѡ���ƣ���ƽ̨��أ�
#define ADS1220_CS_LOW()                                                       \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define ADS1220_CS_HIGH()                                                      \
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

void ADS1220_WriteRegister(uint8_t reg, uint8_t value);
void ADS1220_Init(void);
void ADS1220_StartConversion(void);
void ADS1220_StopConversion(void);
int32_t ADS1220_ReadData(void);
float ADS1220_ReadPressure(void);
#endif /* __ADS1220_H */

