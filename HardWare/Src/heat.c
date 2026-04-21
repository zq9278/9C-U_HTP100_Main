#include "main.h"
#include "heat.h"
#include "pid.h"

uint8_t HeatPWMVal = 0;
PID_TypeDef HeatPID;

/*
 * 负载自适应加热控制说明
 * ---------------------------------------------------------------------------
 * 基础控制仍然使用 HeatPID 的普通 PID 参数。
 *
 * 负载自适应只在“温度已经稳定在目标附近”之后才会被允许触发。
 * 这样可以避免启动升温阶段、第一次过冲后的自然回落，被误判成负载进入。
 *
 * 典型流程：
 * 1. 启动后先屏蔽一段时间，不允许进入负载模式。
 * 2. 温度进入目标附近并稳定一段时间后，heat_armed 置 1。
 * 3. armed 状态下，如果检测到连续掉温，并且当前仍接近目标温度，
 *    判断为“负载进入”，临时进入 heat_load_mode。
 * 4. 负载模式中临时增强 P/I、减弱 D，并给 PWM 一个最小托底值，
 *    让温度更快回到目标附近。
 * 5. 温度恢复到目标附近，或负载模式运行超时后，自动退出负载模式。
 */

/* 单次控制周期允许判定为掉温的阈值，单位：摄氏度/采样周期。
 * Heat_Task 中约 150 ms 调用一次，所以 0.045 表示约 0.3 C/s 的下降速度。
 * 调大：更不容易触发负载模式，抗误判更好，但负载识别更慢。
 * 调小：负载识别更灵敏，但更容易把正常波动当成负载。
 */
#define HEAT_LOAD_DROP_PER_SAMPLE      0.045f

/* 进入负载模式前要求的最小正误差，单位：摄氏度。
 * error = setpoint - measured_value。
 * 只有当前温度低于目标超过该值时，才可能认为需要负载增强。
 */
#define HEAT_LOAD_ENTER_ERROR          0.10f

/* 负载识别只允许发生在目标附近，单位：摄氏度。
 * measured_value 必须高于 setpoint - HEAT_LOAD_ENTER_NEAR_TARGET。
 * 例如目标 42.5 C、该值 0.8 C，则温度必须高于 41.7 C。
 * 这样避免冷启动升温过程中误判为负载。
 */
#define HEAT_LOAD_ENTER_NEAR_TARGET    0.80f

/* armed 稳定判定窗口，单位：摄氏度。
 * 温度必须连续处在目标 +/- 0.25 C 内，才会累计稳定计数。
 */
#define HEAT_LOAD_ARM_ERROR            0.25f

/* armed 所需的连续稳定次数。
 * Heat_Task 约 150 ms 调用一次，20 次约 3 秒。
 * 只有稳定满这段时间后，才允许负载检测开始工作。
 */
#define HEAT_LOAD_ARM_TICKS            20u

/* 启动屏蔽时间，单位：控制周期。
 * 120 次 * 150 ms 约 18 秒。
 * 启动初期存在升温、过冲、回调，不允许触发负载模式。
 */
#define HEAT_LOAD_STARTUP_BLANK_TICKS  120u

/* 连续掉温确认次数。
 * 设置为 2 表示必须连续两次满足掉温条件才进入负载模式，
 * 用来过滤单点噪声和正常小波动。
 */
#define HEAT_LOAD_DROP_CONFIRM_TICKS   2u

/* 退出负载模式的误差阈值，单位：摄氏度。
 * 当温度已经恢复到目标附近，例如 error <= 0.08 C，
 * 认为负载恢复完成，退出增强模式。
 */
#define HEAT_LOAD_EXIT_ERROR           0.08f

/* 负载模式最长持续时间，单位：控制周期。
 * 90 次 * 150 ms 约 13.5 秒。
 * 防止误判后长时间保持增强加热。
 */
#define HEAT_LOAD_MAX_TICKS            90u

/* 负载模式下的最小 PWM 托底。
 * 当温度明显低于目标时，如果 PID 算出来的输出太小，
 * 会强制至少给到这个 PWM，提升负载回温速度。
 * 调大：回温更快，但更容易过冲。
 * 调小：更稳，但负载回温变慢。
 */
#define HEAT_LOAD_MIN_PWM              185.0f

/* 负载模式下临时放大的比例系数。
 * 这里只临时改 PID 结构体中的 Kp/Ki/Kd 调用一次 PID_Compute，
 * 调用结束后马上恢复原参数，所以不会永久改变用户设置的 PID。
 */
#define HEAT_LOAD_KP_SCALE             1.25f
#define HEAT_LOAD_KI_SCALE             1.15f

/* 负载回温时降低 D 项。
 * D 项能抑制过冲，但也会在回温过程中“刹车”。
 * 负载进入后希望先快速补热，所以临时减弱 D。
 */
#define HEAT_LOAD_KD_SCALE             0.55f

/* 负载模式进入瞬间给积分项预充一点量。
 * 作用是让持续负载下的补热更果断。
 * 调大：回温更快，但容易过冲和振荡。
 * 调小：更稳，但持续负载下恢复慢。
 */
#define HEAT_LOAD_INTEGRAL_PRECHARGE   12.0f

/* Startup soft landing:
 * Keep the normal PID parameters unchanged, but limit heater PWM only during
 * the first approach to setpoint. This reduces the initial overshoot caused by
 * thermal inertia when Kp/Ki are kept high for later recovery.
 */
#define HEAT_STARTUP_SOFT_LANDING_MAX_TICKS       260u
#define HEAT_STARTUP_SOFT_LANDING_STABLE_TICKS    24u
#define HEAT_STARTUP_SOFT_LANDING_STABLE_ERROR    0.20f
#define HEAT_STARTUP_ZONE_2C_PWM                  230.0f
#define HEAT_STARTUP_ZONE_1C_PWM                  190.0f
#define HEAT_STARTUP_ZONE_0P5C_PWM                145.0f
#define HEAT_STARTUP_ZONE_0P2C_PWM                95.0f
#define HEAT_STARTUP_INTEGRAL_BLEED               0.70f

/* Segmented PID parameters.
 * When the absolute error is greater than 5 C, use the far group for faster
 * heating. Once the error is within 5 C, switch to the near group to suppress
 * overshoot near the target.
 */
#define HEAT_SEGMENT_SWITCH_ERROR                 5.0f

#define HEAT_SEGMENT_FAR_KP                       12.0f
#define HEAT_SEGMENT_FAR_KI                       0.0f
#define HEAT_SEGMENT_FAR_KD                       0.0f

#define HEAT_SEGMENT_NEAR_KP                      35.0f
#define HEAT_SEGMENT_NEAR_KI                      0.8f
#define HEAT_SEGMENT_NEAR_KD                      20.0f

#define HEAT_SEGMENT_MODE_FAR                     0u
#define HEAT_SEGMENT_MODE_NEAR                    1u

/* 当前是否处于负载增强模式。 */
static uint8_t heat_load_mode = 0;

/* heat_last_temperature 是否有效。第一次采样前无上一温度。 */
static uint8_t heat_last_valid = 0;

/* 是否已经满足“稳定在目标附近一段时间”的武装条件。 */
static uint8_t heat_armed = 0;

/* 连续满足掉温条件的次数，用于确认负载进入。 */
static uint8_t heat_drop_confirm_ticks = 0;

/* 从本次 Heat_Task 开始后的总控制周期数。 */
static uint16_t heat_total_ticks = 0;

/* 连续稳定在目标附近的周期数。达到 HEAT_LOAD_ARM_TICKS 后 armed。 */
static uint16_t heat_stable_ticks = 0;

/* 负载模式已持续的周期数，用于超时退出。 */
static uint16_t heat_load_ticks = 0;

/* 负载触发条件状态位，上报给上位机用于观察每个条件是否满足。 */
static uint16_t heat_adaptive_status = 0;

/* 上一次控制周期的温度，用来计算 temperature_drop。 */
static float heat_last_temperature = 0.0f;

static uint8_t heat_startup_soft_landing = 0;
static uint16_t heat_startup_ticks = 0;
static uint16_t heat_startup_stable_ticks = 0;
static uint8_t heat_segment_pid_mode = HEAT_SEGMENT_MODE_FAR;
static uint8_t heat_segment_pid_valid = 0;

extern TIM_HandleTypeDef htim14;
void HeatInit(void){
   // PID_Init(&HeatPID, 17 , 0.028 , 120, 50, 0, 254,0,0);
    /* 基础温度 PID。
     * 这组参数主要负责正常升温和稳态控制。
     * 负载进入时会由 HeatAdaptivePIDCompute 临时增强，不需要把基础 PID 调得过猛。
     */
    PID_Init(&HeatPID, 30 , 0.5 , 20, 300, -300, 254,0,0);
    //PID_Init(&HeatPID, 28 , 0.55 , 28, 300, -300, 254,0,0);
    //PID_Init(&HeatPID, 15 , 0.4 , 50, 180, -180, 254,0,0);
}

void HeatPWM(uint8_t state)
{
    /* 打开或关闭 TIM14 CH1 PWM 输出。 */
    state ? HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) : HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
}

void HeatPWMSet(uint8_t PWMVal)   
{
    /* 设置 PWM 占空比。
     * 当前 TIM14 ARR 对应 0..254 的输出范围，和 PID output_max 保持一致。
     */
    TIM14->CCR1 = PWMVal;
}

static void HeatPIDClearHistory(PID_TypeDef *pid, float measured_value) {
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_measured_value = measured_value;
    pid->derivative_filtered = 0.0f;
}

void HeatSegmentedPIDReset(void) {
    heat_segment_pid_mode = HEAT_SEGMENT_MODE_FAR;
    heat_segment_pid_valid = 0;
    HeatPIDClearHistory(&HeatPID, 0.0f);
}

float HeatSegmentedPIDCompute(PID_TypeDef *pid, float measured_value) {
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

void HeatStartupSoftLandingReset(void) {
    heat_startup_soft_landing = 1;
    heat_startup_ticks = 0;
    heat_startup_stable_ticks = 0;
}

float HeatStartupSoftLandingApply(PID_TypeDef *pid, float measured_value, float output) {
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

void HeatAdaptiveReset(void) {
    /* 每次加热任务重新开始时调用，清掉上一次运行留下的负载识别状态。
     * 否则上一次测试的 armed/load 状态可能影响下一次加热。
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

uint16_t HeatAdaptiveGetStatus(void) {
    return heat_adaptive_status;
}

float HeatAdaptivePIDCompute(PID_TypeDef *pid, float measured_value) {
    /* error > 0 表示当前温度低于目标，需要加热。
     * error < 0 表示当前温度高于目标，需要降低或关闭加热输出。
     */
    float error = pid->setpoint - measured_value;
    float temperature_drop = 0.0f;
    uint8_t startup_ready = 0;
    uint8_t stable_window = 0;
    uint8_t error_low = 0;
    uint8_t near_target = 0;
    uint8_t dropping = 0;

    /* 根据上一周期温度计算本周期掉温量。
     * temperature_drop > 0 表示温度正在下降。
     */
    if (heat_last_valid) {
        temperature_drop = heat_last_temperature - measured_value;
    }

    /* 记录本次加热任务已经运行了多少个控制周期。 */
    heat_total_ticks++;
    startup_ready = (heat_total_ticks > HEAT_LOAD_STARTUP_BLANK_TICKS);
    stable_window = (error > -HEAT_LOAD_ARM_ERROR && error < HEAT_LOAD_ARM_ERROR);
    error_low = (error > HEAT_LOAD_ENTER_ERROR);
    near_target = (measured_value > (pid->setpoint - HEAT_LOAD_ENTER_NEAR_TARGET));
    dropping = (heat_last_valid && temperature_drop > HEAT_LOAD_DROP_PER_SAMPLE);

    /* armed 条件：
     * 1. 启动屏蔽时间已过；
     * 2. 温度已经在目标 +/- HEAT_LOAD_ARM_ERROR 内；
     * 3. 连续稳定达到 HEAT_LOAD_ARM_TICKS。
     *
     * 目的：只在系统已经进入稳定工作区后，才允许负载识别。
     * 起始升温、第一峰过冲后的自然回调，不会触发负载模式。
     */
    if (startup_ready && stable_window) {
        if (heat_stable_ticks < HEAT_LOAD_ARM_TICKS) {
            heat_stable_ticks++;
        }
    } else {
        heat_stable_ticks = 0;
    }

    heat_armed = (heat_stable_ticks >= HEAT_LOAD_ARM_TICKS);

    /* 负载进入确认：
     * armed 后，如果温度仍在目标附近，并且连续出现明显掉温，
     * 才累计 heat_drop_confirm_ticks。
     *
     * 这里要求 error > HEAT_LOAD_ENTER_ERROR，
     * 是为了确保当前确实低于目标，而不是高于目标时的正常回调。
     */
    if (heat_armed && error_low && near_target && dropping) {
        if (heat_drop_confirm_ticks < HEAT_LOAD_DROP_CONFIRM_TICKS) {
            heat_drop_confirm_ticks++;
        }
    } else {
        heat_drop_confirm_ticks = 0;
    }

    /* 连续掉温达到确认次数后，进入负载增强模式。
     * 同时给积分项一点预充，让持续负载下有更快的补热能力。
     */
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
        /* 保存原始 PID 参数。
         * 负载模式只临时增强本次计算，不永久改变 HeatPID。
         */
        float kp = pid->Kp;
        float ki = pid->Ki;
        float kd = pid->Kd;
        float output;

        /* 临时切换到负载恢复参数：
         * - Kp 增强：掉温后更快提高输出；
         * - Ki 增强：持续负载下更快消除稳态误差；
         * - Kd 减弱：避免回温时被微分项过度刹车。
         */
        pid->Kp = kp * HEAT_LOAD_KP_SCALE;
        pid->Ki = ki * HEAT_LOAD_KI_SCALE;
        pid->Kd = kd * HEAT_LOAD_KD_SCALE;
        output = PID_Compute(pid, measured_value);

        /* 立即恢复用户设置的基础 PID 参数。 */
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;

        /* PWM 托底：
         * 当温度明显低于目标时，即使 PID 输出偏小，
         * 也保证有足够加热功率用于负载恢复。
         */
        if (error > 0.35f && output < HEAT_LOAD_MIN_PWM) {
            output = HEAT_LOAD_MIN_PWM;
        }

        /* 退出条件：
         * 1. 温度已经恢复到目标附近；
         * 2. 或负载增强模式持续太久，强制退出，避免误判后一直加热。
         */
        heat_load_ticks++;
        if (error <= HEAT_LOAD_EXIT_ERROR || heat_load_ticks >= HEAT_LOAD_MAX_TICKS) {
            heat_load_mode = 0;
            heat_load_ticks = 0;
        }

        /* 更新历史温度，并返回增强后的输出。 */
        heat_last_temperature = measured_value;
        heat_last_valid = 1;
        return output;
    }

    /* 普通模式：直接使用基础 PID。 */
    heat_last_temperature = measured_value;
    heat_last_valid = 1;
    return PID_Compute(pid, measured_value);
}
