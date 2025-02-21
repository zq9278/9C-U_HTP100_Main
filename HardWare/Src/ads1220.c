/*
 * @Author: zhangqi
 * @Date: 2024-12-30 17:40:27
 * @Last Modified by: zhangqi
 * @Last Modified time: 2025-01-05 16:37:52
 */

// SPI�ӿڴ��룬������MCUͨ�ŵ�ADS1220
#include "main.h"
#include <stdint.h>

// ����ȫ�ֱ���
volatile uint8_t spi_rx_buffer[3];
volatile int32_t ads1220_result = 0;
volatile uint8_t data_ready = 0; // ��־λ
extern SPI_HandleTypeDef hspi2;
/*
 * SPI2 DMA + ����ź�����װ (ADS1220 ֧�֣���������շ����ź���)
 * ����: zhangqi
 * ����: 2024-12-30
 * �޸�����: 2025-01-05
 */

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi2;

// ============================
// �ź�����ʼ��
// ============================
void SPI2_DMA_Semaphores_Init(void) {
    spi2TxDmaSemaphoreHandle = xSemaphoreCreateBinary();
    spi2RxDmaSemaphoreHandle = xSemaphoreCreateBinary();

    if (spi2TxDmaSemaphoreHandle != NULL) {
        //xSemaphoreGive(spi2TxDmaSemaphoreHandle);  // ��ʼ�ͷŷ����ź���
    }

    if (spi2RxDmaSemaphoreHandle != NULL) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);  // ��ʼ�ͷŽ����ź���
    }
}
// ============================
// SPI ���ͷ�װ��DMA + �����ź�����
// ============================
HAL_StatusTypeDef SPI2_Transmit_DMA(uint8_t *txData, uint16_t size, uint32_t timeout) {
    if (xSemaphoreTake(spi2TxDmaSemaphoreHandle, 10) != pdPASS) {
        return HAL_BUSY;  // ��һ�η���δ���
    }
    //xSemaphoreTake(spi2TxDmaSemaphoreHandle, portMAX_DELAY);

    if (HAL_SPI_Transmit_DMA(&hspi2, txData, size) != HAL_OK) {
        xSemaphoreGive(spi2TxDmaSemaphoreHandle);  // ����ʧ�ܣ������ͷ��ź���
        return HAL_ERROR;
    }

    if (xSemaphoreTake(spi2TxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_TIMEOUT;  // ��ʱ����
    }

    return HAL_OK;
}

// ============================
// SPI ���շ�װ��DMA + �����ź�����
// ============================
HAL_StatusTypeDef SPI2_Receive_DMA(uint8_t *rxData, uint16_t size, uint32_t timeout) {
    if (xSemaphoreTake(spi2RxDmaSemaphoreHandle, 0) != pdPASS) {
        return HAL_BUSY;  // ��һ�ν���δ���
    }

    if (HAL_SPI_Receive_DMA(&hspi2, rxData, size) != HAL_OK) {
        xSemaphoreGive(spi2RxDmaSemaphoreHandle);  // ����ʧ�������ͷ�
        return HAL_ERROR;
    }

    if (xSemaphoreTake(spi2RxDmaSemaphoreHandle, pdMS_TO_TICKS(timeout)) != pdPASS) {
        return HAL_TIMEOUT;  // ��ʱ����
    }

    return HAL_OK;
}

// ============================
// д�Ĵ�������
// ============================
void ADS1220_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t cmd[2] = {0x40 | (reg << 2), value};  // WREG���� + ��ַ + ����
    ADS1220_CS_LOW();
    if (SPI2_Transmit_DMA(cmd, 2, 100) != HAL_OK) {
        printf("д�Ĵ���ʧ�� - REG: 0x%02X\n", reg);
    }
    ADS1220_CS_HIGH();
}

// ============================
// ADS1220 ��ʼ��
// ============================
void ADS1220_Init(void) {
    HAL_Delay(1);  // �ϵ��ȶ�ʱ��

    uint8_t reset_cmd = ADS1220_CMD_RESET;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&reset_cmd, 1, 100);
    ADS1220_CS_HIGH();
    osDelay(50);  // �ȴ���λ���

    uint8_t selfcal_cmd = ADS1220_CMD_SELFCAL;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&selfcal_cmd, 1, 100);
    ADS1220_CS_HIGH();
    osDelay(50);  // ��У׼��ɵȴ�

    // ���üĴ���
    ADS1220_WriteRegister(ADS1220_REG_CONFIG0, 0x3E);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG1, 0x94);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG2, 0x98);
    ADS1220_WriteRegister(ADS1220_REG_CONFIG3, 0x00);
}

// ============================
// ��������ת��ģʽ
// ============================
void ADS1220_StartConversion(void) {
    uint8_t start_cmd = ADS1220_CMD_START_SYNC;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&start_cmd, 1, 100);
    ADS1220_CS_HIGH();
}

// ============================
// ֹͣ����ת��
// ============================
void ADS1220_StopConversion(void) {
    uint8_t stop_cmd = ADS1220_CMD_POWERDOWN;
    ADS1220_CS_LOW();
    SPI2_Transmit_DMA(&stop_cmd, 1, 100);
    ADS1220_CS_HIGH();
}

// ============================
// ��ȡ���ݺ��� (DMA + �����ź�������)
// ============================
int32_t ADS1220_ReadData(void) {
    uint8_t read_cmd = ADS1220_CMD_RDATA;
    uint8_t rx_buffer[3] = {0};

    ADS1220_CS_LOW();

    // ���Ͷ�ȡ����
    if (SPI2_Transmit_DMA(&read_cmd, 1, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        printf("��ȡ�����ʧ��\n");
        return -1;
    }

    // ��������
    if (SPI2_Receive_DMA(rx_buffer, 3, 100) != HAL_OK) {
        ADS1220_CS_HIGH();
        printf("���ݽ���ʧ��\n");
        return -1;
    }

    ADS1220_CS_HIGH();

    int32_t result = ((int32_t)rx_buffer[0] << 16) |
                     ((int32_t)rx_buffer[1] << 8)  |
                     rx_buffer[2];

    return (result & 0x800000) ? (result | 0xFF000000) : result;  // ������չ
}

// ============================
// ��ȡѹ��ֵ����
// ============================
float ADS1220_ReadPressure(void) {
    int32_t raw_data = ADS1220_ReadData();
    if (raw_data == -1) return -1.0f;  // ��ȡʧ�ܴ���

    raw_data = -raw_data;  // ��������
    float V_in = (raw_data * VREF) / 8388607.0f;
    float V_load = V_in / GAIN;
    float weight = V_load / ((SENSITIVITY / 1000.0f) * EXCITATION_VOLTAGE);

    return weight * 1000.0f;  // ����ѹ��ֵ (g)
}

// ============================
// ������Ч���� (�������ȡ��Чֵ���)
// ============================
void Discard_dirty_data(void) {
    for (int i = 0; i < 5; i++) {
        ADS1220_ReadPressure();
        HAL_Delay(10);  // �����ȡ���������
    }
}

// ============================
// ʾ������: �����Զ�ȡѹ��
// ============================
void Start_SPI_Task(void const *argument) {
    for (;;) {
        float pressure = ADS1220_ReadPressure();
        if (pressure >= 0) {
            printf("ѹ��: %.2f g\n", pressure);
        } else {
            printf("��ȡѹ��ʧ��\n");
        }
        osDelay(500);  // 500ms�����ȡ
    }
}