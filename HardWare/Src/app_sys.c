#include "bq25895.h"
#include "main.h"
#include <stdint.h>
uint8_t STATE_POWER_5V;
uint8_t reset=0;
void STATE_POWER_5V_Update(void) {
  //if (HAL_GPIO_ReadPin(PWR_SENSE_GPIO_Port, PWR_SENSE_Pin) == 1) {
    if (reset==1) {
  
  // MX_USART2_UART_Init();
  // ADS1220_Init();
  // TMC5130_Init();
  //MotorChecking();
  reset=0;
  //  }




  STATE_POWER_5V = 1; // 开关开
} else {
  STATE_POWER_5V = 0; // 开关关
};
}


