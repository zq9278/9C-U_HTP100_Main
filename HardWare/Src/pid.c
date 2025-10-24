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
#include "pid.h"
#include "interface_uart.h"
#include <math.h>

float p, i, d;

// 微分滤波系数（0 < alpha < 1，越大越平滑）
#define DERIVATIVE_FILTER_ALPHA 0.8f

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float integral_min,
              float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_measured_value = 0.0f;   // 上一次测量值（用于微分计算）
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;       // 微分滤波结果
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = setpoint;
}

/**
 * @brief PID 计算函数（带积分分离）
 * @param pid PID结构体指针
 * @param measured_value 当前测量值
 * @return 限幅后的PID输出
 */
float PID_Compute(PID_TypeDef *pid, float measured_value) {
    // 计算误差
    float error = pid->setpoint - measured_value;

    // ================================
    //     积分分离 + 非对称积分速度
    // ================================
    float error_threshold = 4.0f;     // 积分分离阈值
    float decay_factor = 2.0f;        // 积分下降加速系数（>1 表示下降更快）

    if (fabsf(error) < error_threshold) {
        // 误差较小时才积分
        if (error > 0) {
            // 温度低于目标，正常积分上升
            pid->integral += error;
        } else {
            // 温度高于目标，积分下降更快（乘衰减系数）
            pid->integral += error * decay_factor;
        }
    }

    // 限制积分项范围，防止积分饱和
    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

    // ================================
    //          微分项（带滤波）
    // ================================
    float derivative = measured_value - pid->previous_measured_value;
    pid->derivative_filtered = DERIVATIVE_FILTER_ALPHA * pid->derivative_filtered +
                               (1.0f - DERIVATIVE_FILTER_ALPHA) * derivative;
    pid->previous_measured_value = measured_value;

    // ================================
    //          PID 各项计算
    // ================================
    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = -pid->Kd * pid->derivative_filtered;

    float output = p + i + d;
    float output_limited = Limit(output, pid->output_min, pid->output_max);

    //LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", p, i, d, measured_value, output_limited, pid->setpoint, error);

    return output_limited;
}


float PID_Compute_motor(PID_TypeDef *pid, float measured_value) {
    float error = pid->setpoint - measured_value;
    // 计算误差：目标值（setpoint）减去当前值（measured_value）

    pid->integral += error;
    // 更新积分项：累加误差，主要用于消除稳态误差

    // 限制积分范围，防止积分饱和（导致输出卡死或响应变慢）
    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

    double derivative = error - pid->previous_error;
    // 计算微分项：当前误差与上次误差之差，用于预测变化趋势

    pid->previous_error = error;
    // 保存当前误差

    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = pid->Kd * derivative;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    // PID 输出计算：P + I + D

    // 输出限幅
    float output1 = Limit(output, pid->output_min, pid->output_max);

    // LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output1,pid->setpoint);
    return output1;
    // 返回最终PID控制输出
}
