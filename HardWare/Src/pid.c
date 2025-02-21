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
float p,i,d;
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float integral_max, float integral_min, float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0;
    pid->integral = 0;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = setpoint; // 锟斤拷始锟斤拷锟借定值为0
}

float PID_Compute(PID_TypeDef *pid, float measured_value) {
float error = pid->setpoint - measured_value; 
// 计算误差：目标值（setpoint）与测量值（measured_value）的差值

pid->integral += error; 
// 更新积分项：累计误差值，用于处理稳态误差

// 对积分项进行限幅，防止积分项过大（积分饱和）或过小（积分失效）
Limit(pid->integral, pid->integral_min, pid->integral_max);

double derivative = error - pid->previous_error;
// 计算微分项：当前误差与上一次误差的差值，用于预测误差的变化趋势

pid->previous_error = error; 
// 保存当前误差值，供下一次计算微分项时使用
p=pid->Kp * error;
i=pid->Ki * pid->integral;
d=pid->Kd * derivative;
     //printf("%.2f,%.2f,%.2f,",error,pid->integral,derivative);
   // LOG("%.2f,%.2f,%.2f,",p,i,d);
float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
// 计算PID输出：比例项（Kp）、积分项（Ki）、微分项（Kd）的加权和

// 对输出进行限幅，防止输出值超出允许范围
float output1=Limit(output, pid->output_min, pid->output_max);

return output1;
// 返回PID控制器的最终输出值

}
