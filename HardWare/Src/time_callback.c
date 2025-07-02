//
// Created by zq on 2025/2/19.
//

#include "time_callback.h"
TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle, motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle,serialTimeoutTimerHandle,IIC_EYETimeoutTimerHandle,eye_is_existHandle,breathTimer;;
void ws2812_white_delay_callback(TimerHandle_t xTimer) {

    Flag_400ms = 1;
    white_delay = !white_delay;
    xTimerStart(ws2812_white_delayHandle, 0);



}
void ws2812_yellow_callback(TimerHandle_t xTimer) {
    emergency_stop = false;
    yellow_delay = 0;
}
void breath_delay_Callback(TimerHandle_t xTimer) {
    breathing_flag = 1;
}
void motor_grab3s_Callback(TimerHandle_t xTimer) {
    Flag_3s = 1;
}
void motor_back_1sCallback(TimerHandle_t xTimer) {
    Flag_1s = 1;
}
void buttton_Callback(TimerHandle_t xTimer) {
    if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
        xSemaphoreGive(BUTTON_SEMAPHOREHandle);
    }
}
void tempareture_pid_timer(TimerHandle_t xTimer) {
    flag_200ms = 1;
    tempature_flag_400ms = 1;
    press_flag_400ms = 1;
    battery_flag_400ms = 1;
    is_charging_flag= 1;
}
// ���ڳ�ʱ������
uint8_t serialTimeoutFlag = 0;//����Ļ
void SerialTimeout_Callback(TimerHandle_t xTimer)
{
    serialTimeoutFlag = 1;
}
//uint8_t IIC_EYETimeoutFlag = 0;//���۶�
//void IIC_EYETimeout_Callback(TimerHandle_t xTimer)
//{
//    IIC_EYETimeoutFlag= 1;
//}
void eye_is_exist_callback(TimerHandle_t xTimer) {//�۶ܼ��ʱ��
    eye_workingtime_1s=1;
    eye_existtime_1s=1;
    if (EYE_exist_Flag){//�۶ܴ��ڣ��ȶ�60�������������ͼӣ� �����ͷ����۶�ʧ������


    }
    else if(EYE_working_Flag){//�۶��ڹ������ȶ�30�������������ͼӣ� �����ͷ����۶�ʧ������

    }
}

static int16_t breath_val = 0;
static int8_t breath_dir = 1;
void BreathingLightCallback(TimerHandle_t xTimer)
{
    breath_val += breath_dir * 1;

    if (breath_val >= 100) {
        breath_val = 100;
        breath_dir = -1;
    } else if (breath_val <= 30) {
        breath_val = 30;
        breath_dir = 1;
    }

    uint8_t brightness = (breath_val * breath_val) / 100;

    uint32_t GRB = (brightness << 16) | (brightness << 8) | brightness;
    PWM_WS2812B_Write_24Bits(LED_NUM, GRB);
    PWM_WS2812B_Show(LED_NUM);
}