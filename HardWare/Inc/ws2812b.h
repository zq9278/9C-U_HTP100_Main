#ifndef _WS2812B_H
#define _WS2812B_H
#include "stm32g0xx_hal.h"	  
#include "bq25895.h"

#define WS2812B_HIGH   HAL_GPIO_WritePin(WS2812B_GPIO_Port,WS2812B_Pin,GPIO_PIN_SET)
#define WS2812B_LOW	   HAL_GPIO_WritePin(WS2812B_GPIO_Port,WS2812B_Pin,GPIO_PIN_RESET)


//typedef unsigned char bool;
#define true 1
#define false 0




#define WS2812B_ARR 80		
#define T0H 16					
#define T1H 52		

#define LED_NUM  15
#define DATA_SIZE  24 	    

void PWM_WS2812B_Init(void);
void WS2812B_Reset(void);
void PWM_WS2812B_Write_24Bits(uint16_t num,uint32_t GRB_Data);
void PWM_WS2812B_Show(uint16_t num);
void PWM_WS2812B_Red(uint16_t num);
void PWM_WS2812B_Green(uint16_t num);
void PWM_WS2812B_Blue(uint16_t num);
void PWM_WS2812B(uint16_t num,uint32_t RGB_data);
void LEDUpdate(void);
void UpdateState(uint8_t emergency_stop, uint8_t charging, uint8_t low_battery, uint8_t fully_charged, uint8_t working);
void UpdateLightState(ChargeState_t state);
void UpdateBreathingLight(void);
#endif