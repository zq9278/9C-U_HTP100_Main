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
float p, i, d;

// ΢���˲�ϵ����0 < alpha < 1����Խ��Խƽ��
#define DERIVATIVE_FILTER_ALPHA 0.8f

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float integral_min,
              float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_measured_value = 0.0f;   // ����΢������
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;       // ΢���˲���ֵ
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = setpoint;
}

float PID_Compute(PID_TypeDef *pid, float measured_value) {
    // �������
    float error = pid->setpoint - measured_value;

    // ��������㲢�޷�
    pid->integral += error;
    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

    // ? ΢������ڲ���ֵ������
    float derivative = measured_value - pid->previous_measured_value;

    // ? ΢���˲�����������ͨ�˲���΢����
    pid->derivative_filtered = DERIVATIVE_FILTER_ALPHA * pid->derivative_filtered +
                               (1.0f - DERIVATIVE_FILTER_ALPHA) * derivative;

    // ���浱ǰ����ֵ�����´�΢�ּ���
    pid->previous_measured_value = measured_value;

    // PID �������
    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = -pid->Kd * pid->derivative_filtered;  // ע����ţ�΢�����������Ʋ���ֵͻ��

    // ����������޷�
    float output = p + i + d;
    float output_limited = Limit(output, pid->output_min, pid->output_max);

    // �������
    //LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output_limited,pid->setpoint,error);

    return output_limited;
}
float PID_Compute_motor(PID_TypeDef *pid, float measured_value) {
    float error = pid->setpoint - measured_value;
// ������Ŀ��ֵ��setpoint�������ֵ��measured_value���Ĳ�ֵ

    pid->integral += error;
// ���»�����ۼ����ֵ�����ڴ�����̬���

// �Ի���������޷�����ֹ��������󣨻��ֱ��ͣ����С������ʧЧ��
    pid->integral=Limit(pid->integral, pid->integral_min, pid->integral_max);

    double derivative = error - pid->previous_error;
// ����΢�����ǰ�������һ�����Ĳ�ֵ������Ԥ�����ı仯����

    pid->previous_error = error;
// ���浱ǰ���ֵ������һ�μ���΢����ʱʹ��
    p=pid->Kp * error;
    i=pid->Ki * pid->integral;
    d=pid->Kd * derivative;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
// ����PID����������Kp���������Ki����΢���Kd���ļ�Ȩ��

// ����������޷�����ֹ���ֵ����������Χ
    float output1=Limit(output, pid->output_min, pid->output_max);
    //LOG("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output1,pid->setpoint);
    return output1;
// ����PID���������������ֵ

}