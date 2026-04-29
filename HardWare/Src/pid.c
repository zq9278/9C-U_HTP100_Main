/*
 * 鏂囦欢: pid.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include "pid.h"
#include "interface_uart.h"
#include <math.h>


float p, i, d;


#define DERIVATIVE_FILTER_ALPHA 0.8f


void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float integral_max, float integral_min,
              float output_max, float output_min, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->previous_measured_value = 0.0f;
    pid->integral = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->error = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->output_limited = 0.0f;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->setpoint = setpoint;
}


/**
 * @brief PID_Compute 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float PID_Compute_dt(PID_TypeDef *pid, float measured_value, float dt_s) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
    }

    float error = pid->setpoint - measured_value;
    pid->error = error;


    float error_threshold = 4.0f;
    float decay_factor = 2.0f;

    if (fabsf(error) < error_threshold) {
        if (error > 0) {
            pid->integral += error * dt_s;
        } else {

            pid->integral += error * decay_factor * dt_s;
        }
    }


    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);


    float derivative = (measured_value - pid->previous_measured_value) / dt_s;
    pid->derivative = derivative;
    pid->derivative_filtered = DERIVATIVE_FILTER_ALPHA * pid->derivative_filtered +
                               (1.0f - DERIVATIVE_FILTER_ALPHA) * derivative;
    pid->previous_measured_value = measured_value;


    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = -pid->Kd * pid->derivative_filtered;

    float output = p + i + d;
    float output_limited = Limit(output, pid->output_min, pid->output_max);
    pid->output = output;
    pid->output_limited = output_limited;

    return output_limited;
}

float PID_Compute(PID_TypeDef *pid, float measured_value) {
    return PID_Compute_dt(pid, measured_value, 1.0f);
}


/**
 * @brief PID_Compute_motor_dt 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @param dt_s 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float PID_Compute_motor_dt(PID_TypeDef *pid, float measured_value, float dt_s) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (dt_s <= 0.0f) {

        dt_s = 0.001f;
    }

    float error = pid->setpoint - measured_value;
    pid->error = error;
    pid->integral += error * dt_s;
    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

    double derivative = (error - pid->previous_error) / dt_s;
    pid->derivative = (float)derivative;
    pid->previous_error = error;

    p = pid->Kp * error;
    i = pid->Ki * pid->integral;
    d = pid->Kd * derivative;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    float output1 = Limit(output, pid->output_min, pid->output_max);
    pid->output = output;
    pid->output_limited = output1;

    return output1;
}


/**
 * @brief PID_Compute_motor 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float PID_Compute_motor(PID_TypeDef *pid, float measured_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return PID_Compute_motor_dt(pid, measured_value, 1.0f);
}
