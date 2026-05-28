/*
 * 文件: heat.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef _HEAT_H
#define _HEAT_H
#include "stm32g0xx_hal.h"
#include "pid.h"


#define TEMPERATURE_COMPENSATION 1.5f
#define TEMPERATURE_CONTROL_COMPENSATION TEMPERATURE_COMPENSATION
#define TEMPERATURE_DISPLAY_COMPENSATION TEMPERATURE_COMPENSATION
#define temperature_compensation TEMPERATURE_COMPENSATION
#define HEAT_PID_PERIOD_MS 150u
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
float HeatLimitMeasuredTemperature(float temperature);
void HeatSegmentedPIDReset(void);
float HeatSegmentedPIDCompute(PID_TypeDef *pid, float measured_value);
float HeatSegmentedPIDComputeDt(PID_TypeDef *pid, float measured_value, float dt_s);
void HeatStartupSoftLandingReset(void);
float HeatStartupSoftLandingApply(PID_TypeDef *pid, float measured_value, float output);
float HeatAdaptivePIDCompute(PID_TypeDef *pid, float measured_value);
float HeatAdaptivePIDComputeDt(PID_TypeDef *pid, float measured_value, float dt_s);
void HeatAdaptiveReset(void);
uint16_t HeatAdaptiveGetStatus(void);

#endif
