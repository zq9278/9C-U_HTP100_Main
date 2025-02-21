#include "bq25895.h"
#include "main.h"
#include <stdint.h>
uint8_t STATE_POWER_5V;
uint8_t reset=0;
IWDG_HandleTypeDef hiwdg;
void MX_IWDG_Init(void)
{

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 1;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
}
void STATE_POWER_5V_Update(void) {
  //if (HAL_GPIO_ReadPin(PWR_SENSE_GPIO_Port, PWR_SENSE_Pin) == 1) {
    if (reset==1) {
        MX_IWDG_Init();
  //NVIC_SystemReset();
  STATE_POWER_5V = 1; // 开关开
} else {
  STATE_POWER_5V = 0; // 开关关
};
}
