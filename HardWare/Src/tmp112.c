/*
 * 鏂囦欢: tmp112.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
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

    uint8_t config[2] = {
            0x60 | 0x03 << 5,
            0xA0
    };
    TMP112_WriteWord( 0x01, config);
}









/**
 * @brief TMP112_Read 鍑芥暟瀹炵幇銆? * @param ReadAddr 鍙傛暟銆? * @param pBuffer 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
HAL_StatusTypeDef TMP112_Read(uint8_t ReadAddr, uint8_t* pBuffer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    HAL_StatusTypeDef status;

    if (xSemaphoreTake(i2c2_mutex, pdMS_TO_TICKS(60)) != pdTRUE) {
        LOGE("[Temp] Event\n");
        return HAL_ERROR;
    }


    status = HAL_I2C_Mem_Read_DMA(&hi2c2, 0x91, ReadAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, 2);
    if (status != HAL_OK) {
        LOGE("[Temp] DMA start failed, status=%d\r\n", status);
        xSemaphoreGive(i2c2_mutex);
        return status;
    }


    if (xSemaphoreTake(I2C2_DMA_Sem, pdMS_TO_TICKS(80)) != pdTRUE) {
        LOGE("[Temp] Event\r\n");
        xSemaphoreGive(i2c2_mutex);
        return HAL_TIMEOUT;
    }


    xSemaphoreGive(i2c2_mutex);
    return HAL_OK;
}



 void TMP112_WriteWord(uint8_t WriteAddr,uint8_t* WriteData)
 {
 	HAL_I2C_Mem_Write_DMA(&hi2c2, 0x92, WriteAddr,I2C_MEMADD_SIZE_8BIT, WriteData, 2);
 }
/**
 * @brief Kalman_Init 鍑芥暟瀹炵幇銆? * @param kf 鍙傛暟銆? * @param Q 鍙傛暟銆? * @param R 鍙傛暟銆? */
void Kalman_Init(KalmanFilter *kf, float Q, float R) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    kf->Q = Q;
    kf->R = R;
    kf->P = 1;
    kf->x = 0;
}
/**
 * @brief Kalman_Update 鍑芥暟瀹炵幇銆? * @param kf 鍙傛暟銆? * @param measurement 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float Kalman_Update(KalmanFilter *kf, float measurement) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    kf->P += kf->Q;


    kf->K = kf->P / (kf->P + kf->R);


    kf->x += kf->K * (measurement - kf->x);


    kf->P *= (1 - kf->K);

    return kf->x;
}
KalmanFilter kf;
#define TEMP_DISPLAY_TARGET_FILTER_SIZE 10u
static float temp_display_target_buffer[TEMP_DISPLAY_TARGET_FILTER_SIZE];
static uint8_t temp_display_target_count = 0;
static uint8_t temp_display_target_index = 0;
static uint8_t temp_display_target_valid = 0;
static float temp_display_last_target = 0.0f;

/**
 * @brief TempAbs 鍑芥暟瀹炵幇銆? * @param value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static float TempAbs(float value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief TempDisplayTargetFilterReset 鍑芥暟瀹炵幇銆? */
void TempDisplayTargetFilterReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    temp_display_target_count = 0;
    temp_display_target_index = 0;
    temp_display_target_valid = 0;
    temp_display_last_target = 0.0f;
}

/**
 * @brief TempDisplayTargetFilterUpdate 鍑芥暟瀹炵幇銆? * @param measured_value 鍙傛暟銆? * @param target_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float TempDisplayTargetFilterUpdate(float measured_value, float target_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    float best_value;
    float best_error;

    if (target_value <= 0.0f) {
        TempDisplayTargetFilterReset();
        return measured_value;
    }

    if (!temp_display_target_valid ||
        TempAbs(target_value - temp_display_last_target) > 0.01f) {
        TempDisplayTargetFilterReset();
        temp_display_target_valid = 1;
        temp_display_last_target = target_value;
    }

    temp_display_target_buffer[temp_display_target_index] = measured_value;
    temp_display_target_index++;
    if (temp_display_target_index >= TEMP_DISPLAY_TARGET_FILTER_SIZE) {
        temp_display_target_index = 0;
    }
    if (temp_display_target_count < TEMP_DISPLAY_TARGET_FILTER_SIZE) {
        temp_display_target_count++;
    }

    best_value = temp_display_target_buffer[0];
    best_error = TempAbs(best_value - target_value);
    for (uint8_t i = 1; i < temp_display_target_count; i++) {
        float current_error = TempAbs(temp_display_target_buffer[i] - target_value);
        if (current_error < best_error) {
            best_error = current_error;
            best_value = temp_display_target_buffer[i];
        }
    }

    return best_value;
}

#define ALPHA 0.1
float previousEMA = 0.0;
int16_t TmpData;
float TmpRaw2Ture(void)
{       HAL_StatusTypeDef status = TMP112_Read(0x00, EyeTmpRaw);
    if (status != HAL_OK) {
        LOGE("[Temp] Event");





        return previousEMA;
    }
    TmpData=(EyeTmpRaw[0]<<8) | EyeTmpRaw[1];
    TmpData = TmpData >> 4;
    float tempature=TmpData*0.0625;

    if (previousEMA==0) {
        previousEMA = tempature;
    }

    previousEMA = ALPHA * tempature + (1 - ALPHA) * previousEMA;


    return previousEMA;
}

