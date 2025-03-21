
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
    // 配置寄存器设置（示例：连续转换模式+8Hz采样）
    uint8_t config[2] = {
            0x60 | 0x03 << 5, // 高字节：扩展模式(bit7) | 转换速率11(8Hz)
            0xA0              // 低字节：ALERT极性(bit0)
    };
    TMP112_WriteWord( 0x01, config);
}  

void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
 { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
    ; // 绛?寰?DMA瀹???
	
   // 使用 HAL_I2C_Mem_Read_DMA 进行读操作
   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
   
}
HAL_StatusTypeDef I2C_CheckDevice(uint8_t i2c_addr, uint8_t retries) {
    HAL_StatusTypeDef result;
    uint32_t timeout = 10;  // 每次检查最多等待 10ms
    uint8_t fail_count = 5; // 记录连续失败次数

    for (uint8_t i = 0; i < retries; i++) {
        result = HAL_I2C_IsDeviceReady(&hi2c2, i2c_addr, 1, timeout);

        if (result == HAL_OK) {
            // 如果成功，重置失败计数器，并返回成功
            fail_count = 0;
            return HAL_OK;
        } else {
            fail_count++;
        }

        osDelay(20); // 等待 2ms 后重试
    }

    // 如果连续 `retries` 次都失败，则返回 HAL_ERROR
    if (fail_count >= retries) {
        // LOG("I2C 设备 0x%02X 未连接 (连续 %d 次失败)\n", i2c_addr, retries);
        return HAL_ERROR;
    }

    return HAL_OK; // 这里一般不会执行到
}


prepare_data my_prepare_data_times;
uint8_t TMP112_IsDevicePresent(void) {
    // 初始化实例的成员变量
    my_prepare_data_times.cmd_head_high = 0x6A;
    my_prepare_data_times.cmd_head_low = 0xA6;
    my_prepare_data_times.frame_length=0x0b;
    my_prepare_data_times.cmd_type_high = 0x00;
    my_prepare_data_times.end_high = 0xFF;
    my_prepare_data_times.end_low = 0xFF;

// ???? 2 ?Σ???ε?? 10ms
    //HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c2, 0x91, 2, 100);
    HAL_StatusTypeDef result=I2C_CheckDevice(0x91,10);
    if (result == HAL_OK) {
        //LOG("眼盾连接中\n");
        EYE_exist_Flag=1;//检测到眼盾
        uint16_t eye_time = EYE_AT24CXX_Read(0x02);//
        if (eye_time == 0xFFFF) {//没有被标记的眼盾
            EYE_exist_new_Flag=1;//检测到眼盾
            if(!device_connected){
                NEW_EYE();
            }
            emergency_stop = 0; // 设置紧急标志
            EYE_status=1;
            uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);//读取主机端记录的次数
            my_prepare_data_times.cmd_type_low = 0xb0;
            my_prepare_data_times.value = eye_times;
            Eye_twitching_invalid_master(&my_prepare_data_times); // 将数据发送到队列
            device_connected=1.0;//主机端眼盾使用次数没有记录上去
        } else {
            if(!EYE_exist_new_Flag){
                EYE_status=0;
            }
        }
        return 1;
//眼盾 清零
        AT24C02_WriteAllBytes_eye(0xff);//清理ee存储
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_RESET);
    } else {
 //眼盾 清零  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,GPIO_PIN_SET);
        LOG("眼盾失效或拔出\n");
        EYE_exist_Flag=0;//眼盾不存在了
        EYE_status=0;
        EYE_exist_new_Flag=0;//检测到眼盾
        currentState=STATE_OFF;
        if ((EYE_working_Flag==1)&&(device_connected==1.0)){//主机端眼盾使用次数没有记录上去
            close_mianAPP();
            ScreenTimerStop();
            emergency_stop = 1; // 设置紧急停止标志
            uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
            eye_times+=1;
            AT24CXX_WriteUInt16(0xf2,eye_times);
            EYE_AT24CXX_Write(0x02, eye_workingtime_1s);//标记眼盾已使用
            osDelay(10); // **等待 EEPROM 写入完成**
            device_connected=0.0;//主机端眼盾使用次数已经记录上去
            EYE_working_Flag=0;//眼盾不工作了
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
    // 预测更新
    kf->P += kf->Q;

    // 计算卡尔曼增益
    kf->K = kf->P / (kf->P + kf->R);

    // 更新估计值
    kf->x += kf->K * (measurement - kf->x);

    // 更新误差协方差
    kf->P *= (1 - kf->K);

    return kf->x;
}
KalmanFilter kf;
#define ALPHA 0.1  // 设置平滑因子，值越大对最新数据的权重越大
float previousEMA = 0.0;  // 之前的平滑值
int16_t TmpData;
float TmpRaw2Ture(void)
{   TMP112_Read(0x00,EyeTmpRaw);
    TmpData=(EyeTmpRaw[0]<<8) | EyeTmpRaw[1];
    TmpData = TmpData >> 4;
    float tempature=TmpData*0.0625;
    // 阈值突变检测（示例伪代码）
    if (previousEMA==0) {
        previousEMA = tempature; // 突变时直接采用新值
    }
//    // 计算EMA
    previousEMA = ALPHA * tempature + (1 - ALPHA) * previousEMA;
    //return previousEMA;
     //previousEMA = Kalman_Update(&kf, tempature);
    return previousEMA;
}