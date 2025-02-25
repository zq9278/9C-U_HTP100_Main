
#include "main.h"


uint8_t EyeTmpRaw[2];
uint8_t IIC_EYETimeoutFlag;
float device_connected=0.0;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;



void TMP112_Init(void)
{ 
	
}  

void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
 { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
    ; // �?�?DMA�???
	
   // ʹ�� HAL_I2C_Mem_Read_DMA ���ж�����
   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
   
}


uint8_t TMP112_IsDevicePresent(void) {

// ���� 2 �Σ�ÿ�εȴ� 10ms
    HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, 0x91, 2, 10);
    if (result == HAL_OK) {
        if (device_connected==0.0) {
            LOG("TMP112 �����ӣ�\n");
            taskENTER_CRITICAL();
            HeatInit();// �ٽ������루��ֹ�����л����жϣ�
            taskEXIT_CRITICAL();
            device_connected=1.0;
        }
  return 1;
    } else {
        if (device_connected==1.0){
            LOG("δ��⵽ TMP112��������߻��ַ��\n");
            device_connected=0.0;
        }
 return 0;
    }
     EYE_checkout( device_connected);
}
// void TMP112_MultiRead(uint8_t* pBuffer)   
// { 	
// 	HAL_I2C_Mem_Read_DMA(&hi2c2, TMP112_ADDR, 0x00,I2C_MEMADD_SIZE_8BIT, pBuffer,0x14);
// }  
 
// void TMP112_WriteByte(uint8_t WriteAddr,uint8_t WriteData)
// {
// 	uint8_t Data[1];
// 	Data[0]=WriteData;
// 	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP112_ADDR, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 1);
// } 

// void TMP112_WriteWord(uint8_t WriteAddr,u16 WriteData)
// {
// 	uint8_t Data[2];
// 	Data[0]=WriteData;
// 	Data[1]=WriteData>>8;
// 	HAL_I2C_Mem_Write_DMA(&hi2c2, TMP112_ADDR, WriteAddr,I2C_MEMADD_SIZE_8BIT, Data, 2);
// 	delay_ms(2);
// }
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
#define ALPHA 0.01  // ����ƽ�����ӣ�ֵԽ����������ݵ�Ȩ��Խ��
float previousEMA = 0.0;  // ֮ǰ��ƽ��ֵ
int16_t TmpData;
float TmpRaw2Ture(void)
{   TMP112_Read(0x00,EyeTmpRaw);
    TmpData=(EyeTmpRaw[0]<<8) | EyeTmpRaw[1];
    TmpData = TmpData >> 4;
    float tempature=TmpData*0.0625;

    if (previousEMA == 0.0)
    {
        previousEMA = tempature;
    }

    // ����EMA
    previousEMA = ALPHA * tempature + (1 - ALPHA) * previousEMA;

    return previousEMA;
     previousEMA = Kalman_Update(&kf, tempature);
    return previousEMA;
}