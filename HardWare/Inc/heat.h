#ifndef _HEAT_H
#define _HEAT_H
#include "stm32g0xx_hal.h"  


#define temperature_compensation 0
#define temperature_target 41.5
	void HeatInit(void);
void HeatPWM(uint8_t state);
void HeatPWMSet(uint8_t PWMVal);

#endif

