/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-07-01 18:14:23
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\pid\pid.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
// pid.c

#include "main.h"

float p, i, d;

// 微分滤波系数（0 < alpha < 1），越大越平滑
#define DERIVATIVE_FILTER_ALPHA 0.8f

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float integral_min,
              float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_measured_value = 0.0f;   // 用于微分先行
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;       // 微分滤波初值
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = setpoint;
}

float PID_Compute(PID_TypeDef *pid, float measured_value) {
    // 计算误差
    float error = pid->setpoint - measured_value;

    // 积分项计算并限幅
    pid->integral += error;
    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

    // ? 微分项（基于测量值，非误差）
    float derivative = measured_value - pid->previous_measured_value;

    // ? 微分滤波处理：带低通滤波的微分项
    pid->derivative_filtered = DERIVATIVE_FILTER_ALPHA * pid->derivative_filtered +
                               (1.0f - DERIVATIVE_FILTER_ALPHA) * derivative;

    // 保存当前测量值用于下次微分计算
    pid->previous_measured_value = measured_value;

    // PID 各项计算
    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = -pid->Kd * pid->derivative_filtered;  // 注意符号：微分项用于抑制测量值突变

    // 输出计算与限幅
    float output = p + i + d;
    float output_limited = Limit(output, pid->output_min, pid->output_max);

    // 调试输出
    LOG("%.2f,%.2f,%.2f,", p, i, d);

    return output_limited;
}
