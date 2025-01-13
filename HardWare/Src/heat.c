#include "main.h"
#include "pid.h"

uint8_t HeatPWMVal = 0;
PID_TypeDef HeatPID;


extern TIM_HandleTypeDef htim14;
void HeatInit(void){
    //PID_Init(&HeatPID, 24.2, 0.1, 500, 500, 0, 254,0,0);
    PID_Init(&HeatPID, 31, 0.15, 800, 500, 0, 254,0,0);
}

void HeatPWM(uint8_t state)
{
    state ? HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) : HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
}

void HeatPWMSet(uint8_t PWMVal)
{
    TIM14->CCR1 = PWMVal;
}
