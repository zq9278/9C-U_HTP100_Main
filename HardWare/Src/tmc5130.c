/*
 * 鏂囦欢: tmc5130.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include <stdint.h>
#include "tmc5130.h"
#include "pid.h"
#include "ads1220.h"
#include "UserApp.h"
#include "time_callback.h"

#define FORCE_FILTER_ALPHA 0.9f
#define PRESSURE_DISPLAY_TARGET_FILTER_SIZE 8u


#define PRESSURE_COEFF_150_MMHG 80.0f
#define PRESSURE_COEFF_250_MMHG 80.0f
#define PRESSURE_COEFF_350_MMHG 80.0f
#define PRESSURE_COEFF_450_MMHG 80.0f
#define PRESSURE_COEFF_550_MMHG 80.0f
PID_TypeDef MotorPID;

typedef struct {

    float kp;
    float ki;
    float kd;
} PressurePidProfile_t;

#if ENABLE_PRESSURE_LEVEL_PID_TUNING
static PressurePidProfile_t g_pid_150 = {200.0f, 0.0f, 0.0f};
static PressurePidProfile_t g_pid_250 = {100.0f, 10.0f, 0.0f};
static PressurePidProfile_t g_pid_350 = {200.0f, 10.0f, 0.0f};
static PressurePidProfile_t g_pid_450 = {200.0f, 10.0f, 0.0f};
static PressurePidProfile_t g_pid_550 = {200.0f, 10.0f, 0.0f};



static PressurePidProfile_t *GetPressurePidProfile(float pressure_level_mmhg) {
    if (pressure_level_mmhg <= 200.0f) {
        return &g_pid_150;
    }
    if (pressure_level_mmhg <= 300.0f) {
        return &g_pid_250;
    }
    if (pressure_level_mmhg <= 400.0f) {
        return &g_pid_350;
    }
    if (pressure_level_mmhg <= 500.0f) {
        return &g_pid_450;
    }
    return &g_pid_550;
}




/**
 * @brief ApplyPressurePidBySetpoint 鍑芥暟瀹炵幇銆? * @param pressure_setpoint_mmhg 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static void ApplyPressurePidBySetpoint(float pressure_setpoint_mmhg) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    PressurePidProfile_t *pid_profile = GetPressurePidProfile(pressure_setpoint_mmhg);
    MotorPID.Kp = pid_profile->kp;
    MotorPID.Ki = pid_profile->ki;
    MotorPID.Kd = pid_profile->kd;
}

#if ENABLE_PRESSURE_LEVEL_PID_TUNING


/**
 * @brief PressurePIDSetByLevel 鍑芥暟瀹炵幇銆? * @param pressure_level_mmhg 鍙傛暟銆? * @param kp 鍙傛暟銆? * @param ki 鍙傛暟銆? * @param kd 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t PressurePIDSetByLevel(float pressure_level_mmhg, float kp, float ki, float kd) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    PressurePidProfile_t *pid_profile = GetPressurePidProfile(pressure_level_mmhg);
    if (pid_profile == NULL) {
        return 0;
    }

    pid_profile->kp = kp;
    pid_profile->ki = ki;
    pid_profile->kd = kd;

    if (((pressure_level_mmhg <= 200.0f) && (MotorPID.setpoint <= 200.0f)) ||
        ((pressure_level_mmhg > 200.0f) && (pressure_level_mmhg <= 300.0f) && (MotorPID.setpoint > 200.0f) && (MotorPID.setpoint <= 300.0f)) ||
        ((pressure_level_mmhg > 300.0f) && (pressure_level_mmhg <= 400.0f) && (MotorPID.setpoint > 300.0f) && (MotorPID.setpoint <= 400.0f)) ||
        ((pressure_level_mmhg > 400.0f) && (pressure_level_mmhg <= 500.0f) && (MotorPID.setpoint > 400.0f) && (MotorPID.setpoint <= 500.0f)) ||
        ((pressure_level_mmhg > 500.0f) && (MotorPID.setpoint > 500.0f))) {
        ApplyPressurePidBySetpoint(MotorPID.setpoint);
    }
    return 1;
}
#endif
#else

/**
 * @brief ApplyPressurePidBySetpoint 鍑芥暟瀹炵幇銆? * @param pressure_setpoint_mmhg 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static void ApplyPressurePidBySetpoint(float pressure_setpoint_mmhg) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (pressure_setpoint_mmhg <= 200.0f) {
        MotorPID.Kp = 200.0f;
        MotorPID.Ki = 0.0f;
        MotorPID.Kd = 0.0f;
    } else if (pressure_setpoint_mmhg <= 500.0f) {
        MotorPID.Kp = 10.0f;
        MotorPID.Ki = 0.0f;
        MotorPID.Kd = 0.0f;
    } else {
        MotorPID.Kp = 1.0f;
        MotorPID.Ki = 0.5f;
        MotorPID.Kd = 0.1f;
    }
}
#endif



/**
 * @brief GetPressureConvertCoeff 鍑芥暟瀹炵幇銆? * @param pressure_setpoint_mmhg 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static float GetPressureConvertCoeff(float pressure_setpoint_mmhg) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    ApplyPressurePidBySetpoint(pressure_setpoint_mmhg);

    if (pressure_setpoint_mmhg <= 200.0f) {
        return PRESSURE_COEFF_150_MMHG;
    }
    if (pressure_setpoint_mmhg <= 300.0f) {
        return PRESSURE_COEFF_250_MMHG;
    }
    if (pressure_setpoint_mmhg <= 400.0f) {
        return PRESSURE_COEFF_350_MMHG;
    }
    if (pressure_setpoint_mmhg <= 500.0f) {
        return PRESSURE_COEFF_450_MMHG;
    }
    return PRESSURE_COEFF_550_MMHG;
}



/**
 * @brief FilterForce 鍑芥暟瀹炵幇銆? * @param force 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static float FilterForce(float force) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    static float filtered_force = 0.0f;
    static uint8_t initialized = 0;

    if (!initialized) {
        filtered_force = force;
        initialized = 1;
        return filtered_force;
    }

    filtered_force += FORCE_FILTER_ALPHA * (force - filtered_force);
    return filtered_force;
}

static float pressure_display_target_buffer[PRESSURE_DISPLAY_TARGET_FILTER_SIZE];
static uint8_t pressure_display_target_count = 0;
static uint8_t pressure_display_target_index = 0;
static uint8_t pressure_display_target_valid = 0;
static float pressure_display_last_target = 0.0f;



/**
 * @brief PressureAbs 鍑芥暟瀹炵幇銆? * @param value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
static float PressureAbs(float value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    return (value >= 0.0f) ? value : -value;
}



/**
 * @brief PressureDisplayTargetFilterReset 鍑芥暟瀹炵幇銆? */
void PressureDisplayTargetFilterReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    pressure_display_target_count = 0;
    pressure_display_target_index = 0;
    pressure_display_target_valid = 0;
    pressure_display_last_target = 0.0f;
}



/**
 * @brief PressureDisplayTargetFilterUpdate 鍑芥暟瀹炵幇銆? * @param measured_value 鍙傛暟銆? * @param target_value 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
float PressureDisplayTargetFilterUpdate(float measured_value, float target_value) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    float best_value;
    float best_error;
    uint8_t best_found = 0;

    if (target_value <= 0.0f) {
        PressureDisplayTargetFilterReset();
        return measured_value;
    }

    if (!pressure_display_target_valid ||
        PressureAbs(target_value - pressure_display_last_target) > 0.01f) {
        PressureDisplayTargetFilterReset();
        pressure_display_target_valid = 1;
        pressure_display_last_target = target_value;
    }

    pressure_display_target_buffer[pressure_display_target_index] = measured_value;
    pressure_display_target_index++;
    if (pressure_display_target_index >= PRESSURE_DISPLAY_TARGET_FILTER_SIZE) {
        pressure_display_target_index = 0;
    }
    if (pressure_display_target_count < PRESSURE_DISPLAY_TARGET_FILTER_SIZE) {
        pressure_display_target_count++;
    }

    best_value = measured_value;
    best_error = 0.0f;
    for (uint8_t i = 0; i < pressure_display_target_count; i++) {
        float current_value = pressure_display_target_buffer[i];
        if (current_value >= target_value) {
            float current_error = PressureAbs(current_value - target_value);
            if (!best_found || current_error < best_error) {
                best_found = 1;
                best_error = current_error;
                best_value = current_value;
            }
        }
    }

    return best_found ? best_value : measured_value;
}


uint32_t MotorSpeed = 0x4000;

extern SPI_HandleTypeDef hspi1;



/**
 * @brief TMC5130_Init 鍑芥暟瀹炵幇銆? */
void TMC5130_Init(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    TMC5130_Write(0x81, 0x00000001);
    TMC5130_Write(0xec, 0x000300c3);


    TMC5130_Write(0x90, 0x00001006);

    TMC5130_Write(
            0x91,
            0x0000000a);
    TMC5130_Write(
            0x80,
            0x00000004);
    TMC5130_Write(0x93, 0x000001f4);

    TMC5130_Write(0xf0, 0x000701c8);



    TMC5130_Write(0xa5,0x00015000);


    TMC5130_Write(0xA4, 0x00000001);
    TMC5130_Write(0xA6, 0x00001001);
    TMC5130_Write(0xa7, MotorSpeed);
    TMC5130_Write(0xa8, 0x00001fff);
    TMC5130_Write(0xaa, 0x00008000);
    TMC5130_Write(0xab, 0x0000000a);


    TMC5130_Write(0xa0, 0x00000000);
    PID_Init(&MotorPID, 300, 0, 0, 5000, -5000, (float) (50000), (float) (-50000),
             0);
}


volatile uint8_t SPI_RxComplete = 0;
uint8_t TxBuffer[5];
uint8_t RxBuffer[4];



/**
 * @brief TMC5130_Read 鍑芥暟瀹炵幇銆? * @param ReadAddr 鍙傛暟銆? * @param pBuffer 鍙傛暟銆? */
void TMC5130_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    SPI_RxComplete = 0;
    TxBuffer[0] = ReadAddr;
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 5);
    TMC_CSN(1);

    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 1);
    SPI_RxComplete = 0;
    HAL_SPI_Receive_IT(&hspi1, pBuffer, 4);
    while (!SPI_RxComplete);
    TMC_CSN(1);
}



/**
 * @brief TMC5130_Write 鍑芥暟瀹炵幇銆? * @param WriteAddr 鍙傛暟銆? * @param WriteData 鍙傛暟銆? */
void TMC5130_Write(uint8_t WriteAddr, uint32_t WriteData) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t Data[5];
    Data[0] = WriteAddr;
    Data[1] = (uint8_t) ((WriteData) >> 24);
    Data[2] = (uint8_t) ((WriteData) >> 16);
    Data[3] = (uint8_t) ((WriteData) >> 8);
    Data[4] = (uint8_t) WriteData;
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, Data, 5);
    TMC_CSN(1);
}



/**
 * @brief MotorSetHome 鍑芥暟瀹炵幇銆? */
void MotorSetHome(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    TMC5130_Write(0xA1, 0);
    TMC5130_Write(0xAD, 0);
}



/**
 * @brief MotorCtrl 鍑芥暟瀹炵幇銆? * @param Step 鍙傛暟銆? */
void MotorCtrl(int32_t Step) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t Data[5];
    Data[0] = 0xad;
    Data[1] = (uint8_t) ((Step) >> 24);
    Data[2] = (uint8_t) ((Step) >> 16);
    Data[3] = (uint8_t) ((Step) >> 8);
    Data[4] = (uint8_t) Step;
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, Data, 5);
    TMC_CSN(1);
}



/**
 * @brief VelocityModeMove 鍑芥暟瀹炵幇銆? * @param direction 鍙傛暟銆? */
void VelocityModeMove(uint8_t direction) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t Data[5];
    Data[0] = 0xa0;
    Data[1] = 0x00;
    Data[2] = 0x00;
    Data[3] = 0x00;
    Data[4] = direction;
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, Data, 5);
    TMC_CSN(1);
}



/**
 * @brief StepMinMax 鍑芥暟瀹炵幇銆? * @param Step 鍙傛暟銆? * @param MinValue 鍙傛暟銆? * @param MaxValue 鍙傛暟銆? */
void StepMinMax(int32_t *Step, int32_t MinValue, int32_t MaxValue) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (*Step < MinValue) {
        *Step = MinValue;
    }
    if (*Step > MaxValue) {
        *Step = MaxValue;
    }
}



/**
 * @brief MotorChecking 鍑芥暟瀹炵幇銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t MotorChecking() {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t ReadData[4];

    TMC_ENN(0);
    TMC5130_Write(0xa7, 0x10000);
    VelocityModeMove(Positive);

    while (1) {
        TMC5130_Read(0x04, ReadData);

        if ((ReadData[3] & 0x02) == 0x02) {
            break;
        }
        vTaskDelay(100);
    }

    MotorSetHome();
    TMC5130_Write(0xa0, 0x00000000);
    TMC_ENN(1);

    return 1;
}


/**
 * @brief MotorCompare 鍑芥暟瀹炵幇銆? * @param SetData 鍙傛暟銆? * @param CompareData 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t MotorCompare(int32_t SetData, int32_t CompareData) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    int32_t SubData;
    SubData = CompareData - SetData;
    if (SubData > 0)
    {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 2;
    } else if (SubData < 0)
    {
        TMC5130_Write(0xa7, 0x4000);
        TMC5130_Write(0xa0, 1);
        return 1;
    } else {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 0;
    }
}


/**
 * @brief SetMotorposition 鍑芥暟瀹炵幇銆? * @param position 鍙傛暟銆? */
void SetMotorposition(int position) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    TMC5130_Write(0xa7, 1000);
    TMC5130_Write(0xa0, 0);
    TMC5130_Write(0xad, (uint32_t) position);
}


/**
 * @brief SetMotorSpeed 鍑芥暟瀹炵幇銆? * @param speed 鍙傛暟銆? */
void SetMotorSpeed(int speed) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    if (speed < 0) {
        TMC5130_Write(0xa0, 1);
        TMC5130_Write(0xa7, (uint32_t) (-speed));
    } else {
        TMC5130_Write(0xa0, 2);
        TMC5130_Write(0xa7, (uint32_t) speed);
    }
}

float weight1;
float weight;
float MotorPWM;
extern float weight0;

uint8_t PressureModeStart = 1;

typedef enum {
    PRESS_STAGE_A_RECOVERY = 1,
    PRESS_STAGE_B_APPROACH = 2,
    PRESS_STAGE_C_CLOSED_LOOP = 3,
    PRESS_STAGE_D_RETRACT = 4
} PressureStage_t;

typedef struct {
    float speed_a;
    float speed_b;
    float speed_d;
    float threshold_a_to_b_mmhg;
    float threshold_b_to_c_mmhg;
    uint32_t hold_ms_c;
    uint32_t retract_ms_d;
    float low_band;
    uint32_t low_enter_ms;
} PressureStageProfile_t;


static const PressureStageProfile_t g_profile_150 = {20000.0f,    4000.0f,    10000.0f, 100.0f,     110.0f,    1500u,   600u,           100.0f,      1000u};
static const PressureStageProfile_t g_profile_250 = {35000.0f,    4000.0f,    10000.0f, 100.0f,     110.5f,    1500u,   800u,           200.0f,      1000u};
static const PressureStageProfile_t g_profile_350 = {50000.0f,    4000.0f,    10000.0f, 100.0f,     110.0f,    1500u,   800u,           300.0f,      1000u};
static const PressureStageProfile_t g_profile_450 = {50000.0f,    4000.0f,    10000.0f, 100.0f,     110.0f,    1500u,   800u,           400.0f,      1000u};
static const PressureStageProfile_t g_profile_550 = {50000.0f,    4000.0f,    10000.0f, 100.0f,     110.0f,    1500u,   1000u,          500.0f,      1000u};



static const PressureStageProfile_t *GetPressureStageProfile(float pressure_setpoint_mmhg) {
    if (pressure_setpoint_mmhg <= 200.0f) {
        return &g_profile_150;
    }
    if (pressure_setpoint_mmhg <= 300.0f) {
        return &g_profile_250;
    }
    if (pressure_setpoint_mmhg <= 400.0f) {
        return &g_profile_350;
    }
    if (pressure_setpoint_mmhg <= 500.0f) {
        return &g_profile_450;
    }
    return &g_profile_550;
}

static PressureStage_t g_pressure_stage = PRESS_STAGE_A_RECOVERY;
static uint32_t g_stage_enter_ms = 0;
static uint32_t g_low_begin_ms = 0;
static uint8_t g_low_active = 0;
static uint8_t g_a_used_once = 0;

extern float p, i, d;
extern uint8_t flag_200ms;
extern uint8_t press_flag_400ms;



/**
 * @brief PressureControlReset 鍑芥暟瀹炵幇銆? */
void PressureControlReset(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    g_pressure_stage = PRESS_STAGE_A_RECOVERY;
    g_stage_enter_ms = HAL_GetTick();
    g_low_begin_ms = 0;
    g_low_active = 0;
    g_a_used_once = 0;
    PressureModeStart = (uint8_t)PRESS_STAGE_A_RECOVERY;

    MotorPID.integral = 0.0f;
    MotorPID.previous_error = 0.0f;
    MotorPID.previous_measured_value = 0.0f;
    MotorPID.derivative_filtered = 0.0f;
}



/**
 * @brief PressureControl 鍑芥暟瀹炵幇銆? * @param pid_dt_s 鍙傛暟銆? */
void PressureControl(float pid_dt_s) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    float raw = ADS1220_ReadPressure();
    float force = Limit(raw - weight0, 0, raw - weight0);
    float target = MotorPID.setpoint;
    const PressureStageProfile_t *profile = GetPressureStageProfile(target);
    float convert_coeff = GetPressureConvertCoeff(target);
    float hhmg = (force / 1000.0f) * 9.8f * convert_coeff;
    float pressure_display = PressureDisplayTargetFilterUpdate(hhmg, target);
    uint32_t now_ms = HAL_GetTick();

    if (press_flag_400ms) {
        press_flag_400ms = 0;
        ScreenUpdateForce(pressure_display);
        //ScreenUpdateForce(hhmg);

    }


    if ((g_pressure_stage == PRESS_STAGE_B_APPROACH) ||
        (g_pressure_stage == PRESS_STAGE_C_CLOSED_LOOP)) {
        if (hhmg < (target - profile->low_band)) {
            if (!g_low_active) {
                g_low_active = 1;
                g_low_begin_ms = now_ms;
            } else if ((now_ms - g_low_begin_ms) >= profile->low_enter_ms) {
                g_pressure_stage = g_a_used_once ? PRESS_STAGE_B_APPROACH : PRESS_STAGE_A_RECOVERY;
                g_stage_enter_ms = now_ms;
                g_low_active = 0;
                MotorPID.integral = 0.0f;
            }
        } else {
            g_low_active = 0;
        }
    } else {
        g_low_active = 0;
    }


    if (g_a_used_once && (g_pressure_stage == PRESS_STAGE_A_RECOVERY)) {
        g_pressure_stage = PRESS_STAGE_B_APPROACH;
        g_stage_enter_ms = now_ms;
    }

    switch (g_pressure_stage) {
        case PRESS_STAGE_A_RECOVERY:
            SetMotorSpeed((int)profile->speed_a);
            if (hhmg >= profile->threshold_a_to_b_mmhg) {
                g_pressure_stage = PRESS_STAGE_B_APPROACH;
                g_stage_enter_ms = now_ms;
                g_a_used_once = 1;
            }
            break;

        case PRESS_STAGE_B_APPROACH:
            SetMotorSpeed((int)profile->speed_b);
            if (hhmg >= profile->threshold_b_to_c_mmhg) {
                g_pressure_stage = PRESS_STAGE_C_CLOSED_LOOP;
                g_stage_enter_ms = now_ms;
                MotorPID.integral = 0.0f;
                MotorPID.previous_error = 0.0f;
                MotorPID.previous_measured_value = hhmg;
                MotorPID.derivative_filtered = 0.0f;
            }
            break;

        case PRESS_STAGE_C_CLOSED_LOOP:
            MotorPWM = PID_Compute_motor_dt(&MotorPID, hhmg, pid_dt_s);
            SetMotorSpeed((int)MotorPWM);
            if ((now_ms - g_stage_enter_ms) >= profile->hold_ms_c) {
                g_pressure_stage = PRESS_STAGE_D_RETRACT;
                g_stage_enter_ms = now_ms;
            }
            break;

        case PRESS_STAGE_D_RETRACT:
            SetMotorSpeed((int)(-profile->speed_d));
            if ((now_ms - g_stage_enter_ms) >= profile->retract_ms_d) {
                g_pressure_stage = PRESS_STAGE_B_APPROACH;
                g_stage_enter_ms = now_ms;
            }
            break;

        default:
            g_pressure_stage = PRESS_STAGE_A_RECOVERY;
            g_stage_enter_ms = now_ms;
            SetMotorSpeed((int)profile->speed_a);
            break;
    }

    PressureModeStart = (uint8_t)g_pressure_stage;
}


