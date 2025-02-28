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
    LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output_limited,pid->setpoint);

    return output_limited;
}
float PID_Compute_motor(PID_TypeDef *pid, float measured_value) {
    float error = pid->setpoint - measured_value;
// 计算误差：目标值（setpoint）与测量值（measured_value）的差值

    pid->integral += error;
// 更新积分项：累计误差值，用于处理稳态误差

// 对积分项进行限幅，防止积分项过大（积分饱和）或过小（积分失效）
    pid->integral=Limit(pid->integral, pid->integral_min, pid->integral_max);

    double derivative = error - pid->previous_error;
// 计算微分项：当前误差与上一次误差的差值，用于预测误差的变化趋势

    pid->previous_error = error;
// 保存当前误差值，供下一次计算微分项时使用
    p=pid->Kp * error;
    i=pid->Ki * pid->integral;
    d=pid->Kd * derivative;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
// 计算PID输出：比例项（Kp）、积分项（Ki）、微分项（Kd）的加权和

// 对输出进行限幅，防止输出值超出允许范围
    float output1=Limit(output, pid->output_min, pid->output_max);
    LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output1,pid->setpoint);
    return output1;
// 返回PID控制器的最终输出值

}