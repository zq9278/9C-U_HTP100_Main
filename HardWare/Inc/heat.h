#ifndef _HEAT_H
#define _HEAT_H
#include "stm32g0xx_hal.h"
#include "pid.h"


#define temperature_compensation 0
#define HEAT_ADAPT_STATUS_STARTUP_READY   (1u << 0)
#define HEAT_ADAPT_STATUS_STABLE_WINDOW   (1u << 1)
#define HEAT_ADAPT_STATUS_ARMED           (1u << 2)
#define HEAT_ADAPT_STATUS_ERROR_LOW       (1u << 3)
#define HEAT_ADAPT_STATUS_NEAR_TARGET     (1u << 4)
#define HEAT_ADAPT_STATUS_DROPPING        (1u << 5)
#define HEAT_ADAPT_STATUS_DROP_CONFIRMED  (1u << 6)
#define HEAT_ADAPT_STATUS_LOAD_MODE       (1u << 7)
	void HeatInit(void);
void HeatPWM(uint8_t state);
void HeatPWMSet(uint8_t PWMVal);
void HeatSegmentedPIDReset(void);
float HeatSegmentedPIDCompute(PID_TypeDef *pid, float measured_value);
void HeatStartupSoftLandingReset(void);
float HeatStartupSoftLandingApply(PID_TypeDef *pid, float measured_value, float output);
float HeatAdaptivePIDCompute(PID_TypeDef *pid, float measured_value);
void HeatAdaptiveReset(void);
uint16_t HeatAdaptiveGetStatus(void);

#endif
