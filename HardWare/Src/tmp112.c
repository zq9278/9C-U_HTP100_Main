
#include "main.h"


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

void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
 { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
    ; // �?�?DMA�???
	
   // ʹ�� HAL_I2C_Mem_Read_DMA ���ж�����
   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
   
}
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    uint32_t timeout = 10;  // ÿ�μ�����ȴ� 10ms
    uint8_t fail_count = 5; // ��¼����ʧ�ܴ���

    for (uint8_t i = 0; i < retries; i++) {
        result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, timeout);

        if (result == HAL_OK) {
            // ����ɹ�������ʧ�ܼ������������سɹ�
            fail_count = 0;
            return HAL_OK;
        } else {
            fail_count++;
        }

        osDelay(20); // �ȴ� 2ms ������
    }

    // ������� `retries` �ζ�ʧ�ܣ��򷵻� HAL_ERROR
    if (fail_count >= retries) {
        // LOG("I2C �豸 0x%02X δ���� (���� %d ��ʧ��)\n", i2c_addr, retries);
        return HAL_ERROR;
    }

    return HAL_OK; // ����һ�㲻��ִ�е�
}


prepare_data my_prepare_data_times;
uint8_t TMP112_IsDevicePresent(void) {
    // ��ʼ��ʵ���ĳ�Ա����
    my_prepare_data_times.cmd_head_high = 0x6A;
    my_prepare_data_times.cmd_head_low = 0xA6;
    my_prepare_data_times.frame_length=0x0b;
    my_prepare_data_times.cmd_type_high = 0x00;
    my_prepare_data_times.end_high = 0xFF;
    my_prepare_data_times.end_low = 0xFF;

// ???? 2 ?��???��?? 10ms
    //HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, 0x91, 2, 100);
    HAL_StatusTypeDef result=I2C_CheckDevice(0x91,10);
    if (result == HAL_OK) {
        //LOG("�۶�������\n");
        EYE_exist_Flag=1;//��⵽�۶�
        uint16_t eye_time = EYE_AT24CXX_Read(0x02);//
        if (eye_time == 0xFFFF) {//û�б���ǵ��۶�
            EYE_exist_new_Flag=1;//��⵽�۶�
            if(!device_connected){
                NEW_EYE();
            }
            emergency_stop = 0; // ���ý�����־
            EYE_status=1;
            uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);//��ȡ�����˼�¼�Ĵ���
            my_prepare_data_times.cmd_type_low = 0xb0;
            my_prepare_data_times.value = eye_times;
            Eye_twitching_invalid_master(&my_prepare_data_times); // �����ݷ��͵�����
            device_connected=1.0;//�������۶�ʹ�ô���û�м�¼��ȥ
        } else {
            if(!EYE_exist_new_Flag){
                EYE_status=0;
            }
        }
        return 1;
//�۶� ����
        AT24C02_WriteAllBytes_eye(0xff);//����ee�洢
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_RESET);
    } else {
 //�۶� ����  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_SET);
        LOG("�۶�ʧЧ��γ�\n");
        EYE_exist_Flag=0;//�۶ܲ�������
        EYE_status=0;
        EYE_exist_new_Flag=0;//��⵽�۶�
        currentState=STATE_OFF;
        if ((EYE_working_Flag==1)&&(device_connected==1.0)){//�������۶�ʹ�ô���û�м�¼��ȥ
            close_mianAPP();
            ScreenTimerStop();
            emergency_stop = 1; // ���ý���ֹͣ��־
            uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
            eye_times+=1;
            AT24CXX_WriteUInt16(0xf2,eye_times);
            EYE_AT24CXX_Write(0x02, eye_workingtime_1s);//����۶���ʹ��
            osDelay(10); // **�ȴ� EEPROM д�����**
            device_connected=0.0;//�������۶�ʹ�ô����Ѿ���¼��ȥ
            EYE_working_Flag=0;//�۶ܲ�������
        }
        return 0;
    }
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
{   TMP112_Read(0x00,EyeTmpRaw);
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