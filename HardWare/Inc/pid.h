// pid.h

#ifndef PID_H
#define PID_H
#include "stm32g0xx_hal.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
    float integral_max;   // 积分限幅值
    float integral_min;   // 积分下限幅值
    float output_max;     // 输出限幅值
    float output_min;     // 输出下限幅值
    float setpoint;       // 设定值
} PID_TypeDef;

// 初始化PID控制器
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_max, float integral_min, float output_max, float output_min, float setpoint);

// 计算PID控制输出
float PID_Compute(PID_TypeDef *pid, float measured_value);

// 限幅宏
#define Limit(x, min, max) ((x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))))

#endif // PID_H
