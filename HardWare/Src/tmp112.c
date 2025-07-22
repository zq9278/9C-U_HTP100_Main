
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
    // 配置寄存器设置（示例：连续转换模式+8Hz采样）
    uint8_t config[2] = {
            0x60 | 0x03 << 5, // 高字节：扩展模式(bit7) | 转换速率11(8Hz)
            0xA0              // 低字节：ALERT极性(bit0)
    };
    TMP112_WriteWord( 0x01, config);
}  

//void TMP112_Read(uint8_t ReadAddr,uint8_t* pBuffer)   
// { 	while (hdma_i2c2_rx.State != HAL_DMA_STATE_READY)
//    ; // 绛?寰?DMA瀹???
//	
//   // 使用 HAL_I2C_Mem_Read_DMA 进行读操作
//   HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2) ;
//   
//}

HAL_StatusTypeDef TMP112_Read(uint8_t ReadAddr, uint8_t* pBuffer) {
    HAL_StatusTypeDef status;
    // 1. 获取 I2C2 的互斥锁，最长等待100ms
    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(300)) != pdTRUE) {
        LOG("TMP112_Read：获取 I2C2 互斥锁失败\r\n");
        return HAL_ERROR;
    }

    // 2. 启动 I2C2 的 DMA 读取
    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2);
    if (status != HAL_OK) {
        LOG("TMP112_Read：DMA 启动失败，状态码：%d\r\n", status);
        xSemaphoreGive(i2c2_mutex); // 启动失败也要释放互斥锁
        return status;
    }

    // 3. 等待 DMA 读取完成信号（由回调释放）
    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(300)) != pdTRUE) {
        LOG("TMP112_Read：DMA读取超时\r\n");
        xSemaphoreGive(i2c2_mutex);
        return HAL_TIMEOUT;
    }

    // 4. 操作完成，释放 I2C2 互斥锁
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
{       HAL_StatusTypeDef status = TMP112_Read(0x00, EyeTmpRaw);
    if (status != HAL_OK) {
        LOG("温度读取失败，返回上一次的EMA或NAN");
        // 可以选择：
        // 1. 返回 NAN，代表无效
        // return NAN;

        // 2. 返回上次的 previousEMA，不更新
        return previousEMA;
    }
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