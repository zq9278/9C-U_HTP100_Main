/*
 * @Author: zhangqi
 * @Date: 2024-12-30 17:40:27
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 16:37:52
 */

// SPI�ӿڴ��룬������MCUͨ�ŵ�ADS1220
#include "main.h"
#include "stm32g0xx_hal_gpio.h"
#include <stdint.h>

// ����ȫ�ֱ���
volatile uint8_t spi_rx_buffer[3];
volatile int32_t ads1220_result = 0;
volatile uint8_t data_ready = 0; // ��־λ

void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
  uint8_t cmd = 0x40 | (reg << 2); // WREG���� + ��ַ
  ADS1220_CS_LOW();
  HAL_SPI_Transmit_IT(&hspi2, &cmd, 1);   // ʹ���жϷ�������
  HAL_SPI_Transmit_IT(&hspi2, &value, 1); // ʹ���жϷ�������
  ADS1220_CS_HIGH();
}

void ADS1220_Init(void) {
  // �ϵ����ʱȷ���豸�ȶ�
  HAL_Delay(1);
  // ����ADS1220
  ADS1220_CS_LOW();
  uint8_t reset_cmd = ADS1220_CMD_RESET;
  HAL_SPI_Transmit_IT(&hspi2, &reset_cmd, 1);
  ADS1220_CS_HIGH();

  // ����ADS1220�Ĵ���
  ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x3e); // ���üĴ���0
  ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94); // ���üĴ���1
  ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98); // ���üĴ���2
  ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00); // ���üĴ���3
 
}

void ADS1220_StartConversion(void) {
  // ��������ת��ģʽ
  ADS1220_CS_LOW();
  uint8_t start_cmd = ADS1220_CMD_START_SYNC;
  HAL_SPI_Transmit_IT(&hspi2, &start_cmd, 1); // ʹ���жϷ�������
  ADS1220_CS_HIGH();
}
#define ADS1220_DEAD_RESPONSE {0xFF, 0xFF, 0xFF}

uint8_t response[3] = {0xFF, 0xFF, 0xFF}; // ��������ʱ�ķ���ֵ
int32_t ADS1220_ReadData(void) {
  uint8_t read_cmd = ADS1220_CMD_RDATA;
  if (HAL_GPIO_ReadPin(ADS1220_DRDY_GPIO_Port, ADS1220_DRDY_Pin) == GPIO_PIN_RESET) // ����Ƿ�����
  {
    ADS1220_CS_LOW();
    HAL_SPI_Transmit_IT(&hspi2, &read_cmd, 1);    // ���Ͷ�ȡ��������
    HAL_SPI_Receive_IT(&hspi2, spi_rx_buffer, 3); // ����3�ֽ�����
    ADS1220_CS_HIGH();
    //  // �ֶ����ֽڼ�������Ƿ�����������ֵһ��
    //   if (spi_rx_buffer[0] == 0xFF &&
    //       spi_rx_buffer[1] == 0xFF &&
    //       spi_rx_buffer[2] == 0xFF) {
    //       ADS1220_Init(); // �����⵽��������λADS1220
    //   }
    // �ȴ��ж����
    while (!data_ready) {
      // ������Լ��볬ʱ���ƣ���ֹ����ѭ��
    }
    data_ready = 0; // �����־λ
                    // ��������
  }
  int32_t result = ((int32_t)spi_rx_buffer[0] << 16) |
                   ((int32_t)spi_rx_buffer[1] << 8) | spi_rx_buffer[2];
  if (result & 0x800000) {
    result |= 0xFF000000; // ����������չ
  }
  return result; // ���ؽ������
}

float ADS1220_ReadPressure(void) {

  int32_t raw_data = ADS1220_ReadData(); // ��ȡԭʼ����
  raw_data = -raw_data;                  // ����ֵ��Ϊ��ֵ
                                         // 1. �����������ѹ V_in
  float V_in = (raw_data * VREF) / 8388607.0;
  // printf("v_in: %d \n",raw_data);

  //     // 2. ���Ƴ�������ԭʼ��� V_LOAD
  float V_load = V_in / GAIN;
  // printf("v_load: %f",V_load);

  //     // 3. ���ݴ����������ȼ�������
  float weight =
      V_load / (SENSITIVITY / 1000.0 *
                EXCITATION_VOLTAGE); // ע�������ȵ�λΪ mV/V����ת��Ϊ V/V
  return weight * 1000; // ����ѹ��ֵ
}
void ADS1220_StopConversion(void) {
  // ֹͣ����ת��ģʽ
  ADS1220_CS_LOW();
  uint8_t stop_cmd = ADS1220_CMD_POWERDOWN; // ʹ�õ�������ֹͣת��
  HAL_SPI_Transmit_IT(&hspi2, &stop_cmd, 1); // ʹ���жϷ�������
  ADS1220_CS_HIGH();
}
