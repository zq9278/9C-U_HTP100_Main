
#include "main.h"
#include "UserApp.h"
#include "tmp112.h"
#include "interface_uart.h"

uint8_t EyeTmpRaw[2];
uint8_t IIC_EYETimeoutFlag;
uint8_t EYE_exist_Flag,EYE_exist_new_Flag,EYE_status;
uint8_t EYE_working_Flag;
float device_connected=0.0;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

void TMP112_Init(void)
{
    // ���üĴ������ã�ʾ��������ת��ģʽ+8Hz������
    uint8_t config[2] = {
            0x60 | 0x03 << 5, // ���ֽڣ���չģʽ(bit7) | ת������11(8Hz)
            0xA0              // ���ֽڣ�ALERT����(bit0)
    };
    TMP112_WriteWord( 0x01, config);
}  

//void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
// { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
//    ; // �?�?DMA�???
//	
//   // ʹ�� HAL_I2C_Mem_Read_DMA ���ж�����
//   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
//   
//}

HAL_StatusTypeDef TMP112_Read(uint8_t ReadAddr, uint8_t* pBuffer) {
    HAL_StatusTypeDef status;
    // 1. ��ȡ I2C2 �Ļ���������ȴ�100ms
    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) != pdTRUE) {
        LOG("TMP112_Read����ȡ I2C2 ������ʧ��\r\n");
        return HAL_ERROR;
    }

    // 2. ���� I2C2 �� DMA ��ȡ
    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2);
    if (status != HAL_OK) {
        LOG("TMP112_Read��DMA ����ʧ�ܣ�״̬�룺%d\r\n", status);
        xSemaphoreGive(i2c2_mutex); // ����ʧ��ҲҪ�ͷŻ�����
        return status;
    }

    // 3. �ȴ� DMA ��ȡ����źţ��ɻص��ͷţ�
    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(300)) != pdTRUE) {
        LOG("TMP112_Read��DMA��ȡ��ʱ\r\n");
        xSemaphoreGive(i2c2_mutex);
        return HAL_TIMEOUT;
    }

    // 4. ������ɣ��ͷ� I2C2 ������
    xSemaphoreGive(i2c2_mutex);
    return HAL_OK;
}



 void TMP112_WriteWord(uint8_t WriteAddr,uint8_t* WriteData)
 {
 	HAL_I2C_Mem_Write_DMA(&hi2c2, 0x92, WriteAddr,I2C_MEMADD_SIZE_8BIT, WriteData, 2);
 }
void Kalman_Init(KalmanFilter *kf, float Q, float R) {
    kf->Q = Q;
    kf->R = R;
    kf->P = 1;
    kf->x = 0;
}
float Kalman_Update(KalmanFilter *kf, float measurement) {
    // Ԥ�����
    kf->P += kf->Q;

    // ���㿨��������
    kf->K = kf->P / (kf->P + kf->R);

    // ���¹���ֵ
    kf->x += kf->K * (measurement - kf->x);

    // �������Э����
    kf->P *= (1 - kf->K);

    return kf->x;
}
KalmanFilter kf;
#define ALPHA 0.1  // ����ƽ�����ӣ�ֵԽ����������ݵ�Ȩ��Խ��
float previousEMA = 0.0;  // ֮ǰ��ƽ��ֵ
int16_t TmpData;
float TmpRaw2Ture(void)
{       HAL_StatusTypeDef status = TMP112_Read(0x00, EyeTmpRaw);
    if (status != HAL_OK) {
        LOG("�¶ȶ�ȡʧ�ܣ�������һ�ε�EMA��NAN");
        // ����ѡ��
        // 1. ���� NAN��������Ч
        // return NAN;

        // 2. �����ϴε� previousEMA��������
        return previousEMA;
    }
    TmpData=(EyeTmpRaw[0]<<8) | EyeTmpRaw[1];
    TmpData = TmpData >> 4;
    float tempature=TmpData*0.0625;
    // ��ֵͻ���⣨ʾ��α���룩
    if (previousEMA==0) {
        previousEMA = tempature; // ͻ��ʱֱ�Ӳ�����ֵ
    }
//    // ����EMA
    previousEMA = ALPHA * tempature + (1 - ALPHA) * previousEMA;
    //return previousEMA;
     //previousEMA = Kalman_Update(&kf, tempature);
    return previousEMA;
}