/*
 * 鏂囦欢: heat.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include "heat.h"
#include "pid.h"

uint8_t HeatPWMVal = 0;
PID_TypeDef HeatPID;




#define HEAT_LOAD_DROP_PER_SAMPLE      0.045f


#define HEAT_LOAD_ENTER_ERROR          0.10f


#define HEAT_LOAD_ENTER_NEAR_TARGET    0.80f


#define HEAT_LOAD_ARM_ERROR            0.25f


#define HEAT_LOAD_ARM_TICKS            20u


#define HEAT_LOAD_STARTUP_BLANK_TICKS  120u


#define HEAT_LOAD_DROP_CONFIRM_TICKS   2u


#define HEAT_LOAD_EXIT_ERROR           0.08f


#define HEAT_LOAD_MAX_TICKS            90u


#define HEAT_LOAD_MIN_PWM              185.0f


#define HEAT_LOAD_KP_SCALE             1.25f
#define HEAT_LOAD_KI_SCALE             1.15f


#define HEAT_LOAD_KD_SCALE             0.55f


#define HEAT_LOAD_INTEGRAL_PRECHARGE   12.0f


#define HEAT_STARTUP_SOFT_LANDING_MAX_TICKS       260u
#define HEAT_STARTUP_SOFT_LANDING_STABLE_TICKS    24u
#define HEAT_STARTUP_SOFT_LANDING_STABLE_ERROR    0.20f
#define HEAT_STARTUP_ZONE_2C_PWM                  230.0f
#define HEAT_STARTUP_ZONE_1C_PWM                  190.0f
#define HEAT_STARTUP_ZONE_0P5C_PWM                145.0f
#define HEAT_STARTUP_ZONE_0P2C_PWM                95.0f
#define HEAT_STARTUP_INTEGRAL_BLEED               0.70f


#define HEAT_SEGMENT_SWITCH_ERROR                 10.0f

#define HEAT_SEGMENT_FAR_KP                       8.0f
#define HEAT_SEGMENT_FAR_KI                       0.0f
#define HEAT_SEGMENT_FAR_KD                       0.0f

#define HEAT_SEGMENT_NEAR_KP                      34.0f
#define HEAT_SEGMENT_NEAR_KI                      0.8f
#define HEAT_SEGMENT_NEAR_KD                      20.0f

#define HEAT_SEGMENT_MODE_FAR                     0u
#define HEAT_SEGMENT_MODE_NEAR                    1u


static uint8_t heat_load_mode = 0;


static uint8_t heat_last_valid = 0;


static uint8_t heat_armed = 0;


static uint8_t heat_drop_confirm_ticks = 0;


static uint16_t heat_total_ticks = 0;


static uint16_t heat_stable_ticks = 0;


static uint16_t heat_load_ticks = 0;


static uint16_t heat_adaptive_status = 0;


static float heat_last_temperature = 0.0f;

static uint8_t heat_startup_soft_landing = 0;
static uint16_t heat_startup_ticks = 0;
static uint16_t heat_startup_stable_ticks = 0;
static uint8_t heat_segment_pid_mode = HEAT_SEGMENT_MODE_FAR;
static uint8_t heat_segment_pid_valid = 0;

extern TIM_HandleTypeDef htim14;
/**
 * @brief HeatInit 鍑芥暟瀹炵幇銆? */
void HeatInit(void){
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */


    PID_Init(&HeatPID, 30 , 0.5 , 20, 300, -300, 254,0,0);


}

void HeatPWM(uint8_t state)
{

    state ? HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) : HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
}

void HeatPWMSet(uint8_t PWMVal)
{

    TIM14->CCR1 = PWMVal;
}

/**
 * @brief HeatPIDClearHistory 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static void HeatPIDClearHistory(PID_TypeDef *pid, float measured_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_measured_value = measured_value;
    pid->derivative_filtered = 0.0f;
}

/**
 * @brief HeatSegmentedPIDReset 鍑芥暟瀹炵幇銆? */
void HeatSegmentedPIDReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    heat_segment_pid_mode = HEAT_SEGMENT_MODE_FAR;
    heat_segment_pid_valid = 0;
    HeatPIDClearHistory(&HeatPID, 0.0f);
}

/**
 * @brief HeatSegmentedPIDCompute 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float HeatSegmentedPIDCompute(PID_TypeDef *pid, float measured_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    float error = pid->setpoint - measured_value;
    float abs_error = (error >= 0.0f) ? error : -error;
    uint8_t next_mode = (abs_error > HEAT_SEGMENT_SWITCH_ERROR) ?
                        HEAT_SEGMENT_MODE_FAR :
                        HEAT_SEGMENT_MODE_NEAR;

    if (!heat_segment_pid_valid || next_mode != heat_segment_pid_mode) {
        heat_segment_pid_mode = next_mode;
        heat_segment_pid_valid = 1;
        HeatPIDClearHistory(pid, measured_value);
    }

    if (heat_segment_pid_mode == HEAT_SEGMENT_MODE_FAR) {
        pid->Kp = HEAT_SEGMENT_FAR_KP;
        pid->Ki = HEAT_SEGMENT_FAR_KI;
        pid->Kd = HEAT_SEGMENT_FAR_KD;
    } else {
        pid->Kp = HEAT_SEGMENT_NEAR_KP;
        pid->Ki = HEAT_SEGMENT_NEAR_KI;
        pid->Kd = HEAT_SEGMENT_NEAR_KD;
    }

    return PID_Compute(pid, measured_value);
}

/**
 * @brief HeatStartupSoftLandingReset 鍑芥暟瀹炵幇銆? */
void HeatStartupSoftLandingReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    heat_startup_soft_landing = 1;
    heat_startup_ticks = 0;
    heat_startup_stable_ticks = 0;
}

/**
 * @brief HeatStartupSoftLandingApply 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @param output 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float HeatStartupSoftLandingApply(PID_TypeDef *pid, float measured_value, float output) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    float error = pid->setpoint - measured_value;
    float max_output = pid->output_max;

    if (!heat_startup_soft_landing) {
        return output;
    }

    heat_startup_ticks++;

    if (error <= 0.0f) {
        output = 0.0f;
        pid->integral *= HEAT_STARTUP_INTEGRAL_BLEED;
        pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);
    } else if (error < 0.2f) {
        max_output = HEAT_STARTUP_ZONE_0P2C_PWM;
    } else if (error < 0.5f) {
        max_output = HEAT_STARTUP_ZONE_0P5C_PWM;
    } else if (error < 1.0f) {
        max_output = HEAT_STARTUP_ZONE_1C_PWM;
    } else if (error < 2.0f) {
        max_output = HEAT_STARTUP_ZONE_2C_PWM;
    }

    if (output > max_output) {
        output = max_output;
    }

    if (error > -HEAT_STARTUP_SOFT_LANDING_STABLE_ERROR &&
        error < HEAT_STARTUP_SOFT_LANDING_STABLE_ERROR) {
        if (heat_startup_stable_ticks < HEAT_STARTUP_SOFT_LANDING_STABLE_TICKS) {
            heat_startup_stable_ticks++;
        }
    } else {
        heat_startup_stable_ticks = 0;
    }

    if (heat_startup_stable_ticks >= HEAT_STARTUP_SOFT_LANDING_STABLE_TICKS ||
        heat_startup_ticks >= HEAT_STARTUP_SOFT_LANDING_MAX_TICKS) {
        heat_startup_soft_landing = 0;
        heat_startup_stable_ticks = 0;
    }

    return output;
}

/**
 * @brief HeatAdaptiveReset 鍑芥暟瀹炵幇銆? */
void HeatAdaptiveReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    heat_load_mode = 0;
    heat_last_valid = 0;
    heat_armed = 0;
    heat_drop_confirm_ticks = 0;
    heat_total_ticks = 0;
    heat_stable_ticks = 0;
    heat_load_ticks = 0;
    heat_adaptive_status = 0;
    heat_last_temperature = 0.0f;
}

/**
 * @brief HeatAdaptiveGetStatus 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint16_t HeatAdaptiveGetStatus(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return heat_adaptive_status;
}

/**
 * @brief HeatAdaptivePIDCompute 鍑芥暟瀹炵幇銆? * @param pid 鍙傛暟銆? * @param measured_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float HeatAdaptivePIDCompute(PID_TypeDef *pid, float measured_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    float error = pid->setpoint - measured_value;
    float temperature_drop = 0.0f;
    uint8_t startup_ready = 0;
    uint8_t stable_window = 0;
    uint8_t error_low = 0;
    uint8_t near_target = 0;
    uint8_t dropping = 0;


    if (heat_last_valid) {
        temperature_drop = heat_last_temperature - measured_value;
    }


    heat_total_ticks++;
    startup_ready = (heat_total_ticks > HEAT_LOAD_STARTUP_BLANK_TICKS);
    stable_window = (error > -HEAT_LOAD_ARM_ERROR && error < HEAT_LOAD_ARM_ERROR);
    error_low = (error > HEAT_LOAD_ENTER_ERROR);
    near_target = (measured_value > (pid->setpoint - HEAT_LOAD_ENTER_NEAR_TARGET));
    dropping = (heat_last_valid && temperature_drop > HEAT_LOAD_DROP_PER_SAMPLE);


    if (startup_ready && stable_window) {
        if (heat_stable_ticks < HEAT_LOAD_ARM_TICKS) {
            heat_stable_ticks++;
        }
    } else {
        heat_stable_ticks = 0;
    }

    heat_armed = (heat_stable_ticks >= HEAT_LOAD_ARM_TICKS);


    if (heat_armed && error_low && near_target && dropping) {
        if (heat_drop_confirm_ticks < HEAT_LOAD_DROP_CONFIRM_TICKS) {
            heat_drop_confirm_ticks++;
        }
    } else {
        heat_drop_confirm_ticks = 0;
    }


    if (!heat_load_mode &&
        heat_drop_confirm_ticks >= HEAT_LOAD_DROP_CONFIRM_TICKS) {
        heat_load_mode = 1;
        heat_load_ticks = 0;
        heat_drop_confirm_ticks = 0;
        pid->integral += HEAT_LOAD_INTEGRAL_PRECHARGE;
        pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);
    }

    heat_adaptive_status = 0;
    if (startup_ready) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_STARTUP_READY;
    }
    if (stable_window) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_STABLE_WINDOW;
    }
    if (heat_armed) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_ARMED;
    }
    if (error_low) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_ERROR_LOW;
    }
    if (near_target) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_NEAR_TARGET;
    }
    if (dropping) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_DROPPING;
    }
    if (heat_drop_confirm_ticks >= HEAT_LOAD_DROP_CONFIRM_TICKS || heat_load_mode) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_DROP_CONFIRMED;
    }
    if (heat_load_mode) {
        heat_adaptive_status |= HEAT_ADAPT_STATUS_LOAD_MODE;
    }

    if (heat_load_mode) {

        float kp = pid->Kp;
        float ki = pid->Ki;
        float kd = pid->Kd;
        float output;


        pid->Kp = kp * HEAT_LOAD_KP_SCALE;
        pid->Ki = ki * HEAT_LOAD_KI_SCALE;
        pid->Kd = kd * HEAT_LOAD_KD_SCALE;
        output = PID_Compute(pid, measured_value);


        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;


        if (error > 0.35f && output < HEAT_LOAD_MIN_PWM) {
            output = HEAT_LOAD_MIN_PWM;
        }


        heat_load_ticks++;
        if (error <= HEAT_LOAD_EXIT_ERROR || heat_load_ticks >= HEAT_LOAD_MAX_TICKS) {
            heat_load_mode = 0;
            heat_load_ticks = 0;
        }


        heat_last_temperature = measured_value;
        heat_last_valid = 1;
        return output;
    }


    heat_last_temperature = measured_value;
    heat_last_valid = 1;
    return PID_Compute(pid, measured_value);
}


