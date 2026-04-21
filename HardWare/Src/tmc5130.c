
#include "main.h"
#include <stdint.h>
#include "tmc5130.h"
#include "pid.h"
#include "ads1220.h"
#include "UserApp.h"
#include "time_callback.h"

#define FORCE_FILTER_ALPHA 0.9f
#define PRESSURE_DISPLAY_TARGET_FILTER_SIZE 8u

/*
 * 不同压力挡位对应的力值换算系数�?
 *
 * 当前先保留原来的默�?��? 80.0f，避免这次重构直接改变现有标定结果�?
 * 后续你只需要按实际标定结果�?改下�? 5 �?宏，不需要再改控制代码�?
 */
#define PRESSURE_COEFF_150_MMHG 80.0f
#define PRESSURE_COEFF_250_MMHG 80.0f
#define PRESSURE_COEFF_350_MMHG 80.0f
#define PRESSURE_COEFF_450_MMHG 80.0f
#define PRESSURE_COEFF_550_MMHG 80.0f
PID_TypeDef MotorPID;

/*
 * 根据当前�?标压力挡位，返回对应的换算系数�?
 *
 * 这里直接使用 MotorPID.setpoint 作为当前压力挡位来源�?
 * 如果后续上位机下发的不是精确�? 150/250/350/450/550，也会按最近区间归档�?
 */
static float GetPressureConvertCoeff(float pressure_setpoint_mmhg) {
    if (pressure_setpoint_mmhg <= 200.0f) {
        MotorPID.Kp=280;
        return PRESSURE_COEFF_150_MMHG;
    }
    if (pressure_setpoint_mmhg <= 300.0f) {
         MotorPID.Kp=40;
        return PRESSURE_COEFF_250_MMHG;
    }
    if (pressure_setpoint_mmhg <= 400.0f) {
         MotorPID.Kp=120;
        return PRESSURE_COEFF_350_MMHG;
    }
    if (pressure_setpoint_mmhg <= 500.0f) {
         MotorPID.Kp=120;
        return PRESSURE_COEFF_450_MMHG;
    }
    MotorPID.Kp=120;
    return PRESSURE_COEFF_550_MMHG;
}

// Low-pass filter to smooth pressure readings before reporting them
static float FilterForce(float force) {
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

static float PressureAbs(float value) {
    return (value >= 0.0f) ? value : -value;
}

void PressureDisplayTargetFilterReset(void) {
    pressure_display_target_count = 0;
    pressure_display_target_index = 0;
    pressure_display_target_valid = 0;
    pressure_display_last_target = 0.0f;
}

float PressureDisplayTargetFilterUpdate(float measured_value, float target_value) {
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
/*motor*/
uint32_t MotorSpeed = 0x4000;

extern SPI_HandleTypeDef hspi1;

PID_TypeDef MotorPID;

void TMC5130_Init(void) {
    //	TMC_ENN(0);// ���õ������
    //HAL_Delay(20);
    TMC5130_Write(0x81, 0x00000001); // reset
    TMC5130_Write(0xec, 0x000300c3); // CHOPCONF: vsense=1,TOFF=3, HSTRT=4,
    // HEND=1, TBL=2, CHM=0 (spreadCycle)

    TMC5130_Write(0x90, 0x00001006); // IHOLD=6, IRUN=16, IHOLDDELAY=6
    /*
     * IHOLD: ���ֵ���ֵ
     * IRUN: ���е���ֵ
     * IHOLDDELAY: �ӳ�ʱ�䣬Խ��Խƽ����Խ��Խͻأ
     */
    TMC5130_Write(
            0x91,
            0x0000000a); // TPOWERDOWN=10: Delay before power down in stand still
    TMC5130_Write(
            0x80,
            0x00000004); // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
    TMC5130_Write(0x93, 0x000001f4); // TPWM_THRS=500 yields a switching velocity
    // about 35000 = ca. 30RPM
    TMC5130_Write(0xf0, 0x000701c8); // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch
    // amplitude limit=200, Grad=1


    TMC5130_Write(0xa5,0x00015000); // V1 = 50 000 Acceleration threshold velocity V1
    //TMC5130_Write(0xa4, 0x00011000); // A1 = 1 000 First acceleration
    //TMC5130_Write(0xa6, 0x00018fff); // AMAX = 500 Acceleration above V1
    TMC5130_Write(0xA4, 0x00000001); // A1: ��С��ʼ���ٶȣ�����Ϊ0��СΪ1
    TMC5130_Write(0xA6, 0x00001001); // AMAX: �����ٶȣ�����Ϊ0��СΪ1
    TMC5130_Write(0xa7, MotorSpeed); // VMAX = 200 000
    TMC5130_Write(0xa8, 0x00001fff); // DMAX = 700 Deceleration above V1
    TMC5130_Write(0xaa, 0x00008000); // D1 = 1400 Deceleration below V1
    TMC5130_Write(0xab, 0x0000000a); // VSTOP = 10 Stop velocity (Near to zero)
    // TMC5130_Write(0xac, 0x00000000);
    // TMC5130_Write(0xb4, 0x0000075f);
    TMC5130_Write(0xa0, 0x00000000); // ֹͣ����˶�
    PID_Init(&MotorPID, 300, 0, 0, 5000, -5000, (float) (50000), (float) (-50000),
             0); // 0.03,0.05//0.02, 0.01, 0.02,
}

// SPI ͨ�����
// ȫ�� SPI �������״̬��־
volatile uint8_t SPI_RxComplete = 0;
uint8_t TxBuffer[5];
uint8_t RxBuffer[4];

void TMC5130_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    // ������ձ�־
    SPI_RxComplete = 0;
    // ��ʼ��������
    TxBuffer[0] = ReadAddr;
    // �ȷ���һ���ֽڵĵ�ַ
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 5);
    TMC_CSN(1);
    // ��ʱȷ��Ӳ��������ɣ�Ƭѡ�źű��ֵ͵�ƽ��Ӳ���������
    // HAL_Delay(1); // ����ʵ�� TMC ָ���ͻ
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 1);
    SPI_RxComplete = 0; // �ٴ�������ձ�־
    HAL_SPI_Receive_IT(&hspi1, pBuffer, 4);
    while (!SPI_RxComplete); // �ȴ��������
    TMC_CSN(1);
}

void TMC5130_Write(uint8_t WriteAddr, uint32_t WriteData) {
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

void MotorSetHome(void) {
    // ���õ�ǰλ�üĴ��� XACTUAL ��ֵΪ 0
    TMC5130_Write(0xA1, 0);
    // ����Ŀ��λ�üĴ��� XTARGET ��ֵΪ 0
    TMC5130_Write(0xAD, 0);
}

void MotorCtrl(int32_t Step) {
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

void VelocityModeMove(uint8_t direction) {
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

void StepMinMax(int32_t *Step, int32_t MinValue, int32_t MaxValue) {
    if (*Step < MinValue) {
        *Step = MinValue;
    }
    if (*Step > MaxValue) {
        *Step = MaxValue;
    }
}


uint8_t MotorChecking() {
    uint8_t ReadData[4];

    TMC_ENN(0); //
    TMC5130_Write(0xa7, 0x10000);
    VelocityModeMove(Positive);

    while (1) {
        // ��ȡ���״̬�Ĵ���
        TMC5130_Read(0x04, ReadData);

        // ���״̬�Ĵ����ĵ�3�ֽڵ� bit1 Ϊ 1
        if ((ReadData[3] & 0x02) == 0x02) {
            // ״̬��ȷ���˳�ѭ��
            break;
        }
        vTaskDelay(100);
    }

    MotorSetHome();
    TMC5130_Write(0xa0, 0x00000000); // ֹͣ����˶�
    TMC_ENN(1);

    return 1;
}

uint8_t MotorCompare(int32_t SetData, int32_t CompareData) {
    int32_t SubData;
    SubData = CompareData - SetData;
    if (SubData > 0) // ForceSen ����
    {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 2;
    } else if (SubData < 0) // ����
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

// TMC_ENN(0); // ���õ������
// TMC5130_Write(0xad, 115000); // ����Ŀ��λ��
/* PID ���õ��λ�� */
void SetMotorposition(int position) {
    TMC5130_Write(0xa7, 1000);
    TMC5130_Write(0xa0, 0);
    TMC5130_Write(0xad, (uint32_t) position);
}

/* PID ���õ���ٶ� */
void SetMotorSpeed(int speed) {
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
float MotorPWM;       // ���ڴ洢������Ƶ� PWM ռ�ձ�
extern float weight0; // test_variable


uint8_t PressureModeStart = 1;
float control_output;
float control_output_speed;
//volatile int Flag_3s = 0, Flag_1s = 0;
//extern osMessageQueueId_t PRESS_DATAHandle; // ѹ�����ݶ���
//extern osTimerId_t motor_back_1sHandle;
//extern osTimerId_t motor_grab3sHandle;
//extern osMessageQueueId_t PressureHandle;
extern float p, i, d;
extern uint8_t flag_200ms;

extern uint8_t press_flag_400ms;
void PressureControl() {
    float weight1 = ADS1220_ReadPressure();
    float weight = Limit(weight1 - weight0, 0, weight1 - weight0);
    /*
     * ѹ�����㣺
     * ͬ�����������ڲ�ͬĿ��ѹ�������ʹ�ò�ͬ�ı궨ϵ����
     * ���� 150/250/350/450/550 mmHg �ܹ����Ծ�ȷƥ�䡣
     */
    float convert_coeff = GetPressureConvertCoeff(MotorPID.setpoint);
    float hhmg = (weight / 1000.0f) * 9.8f * convert_coeff;
    //float hhmg_filtered = FilterForce(hhmg);
    float pressure_display = PressureDisplayTargetFilterUpdate(hhmg, MotorPID.setpoint);
    // Push filtered pressure data when screen refresh flag is set
    if (press_flag_400ms) {
        press_flag_400ms = 0;
        ScreenUpdateForce(pressure_display);
        //xQueueSend(PressureHandle, &hhmg_filtered, 0);
    }
    switch (PressureModeStart) {
        case 1: { // ��ʼǰ׼���׶�
            control_output_speed = 20000;
            SetMotorSpeed((int)control_output_speed);
            //float start_force_flag = Limit(MotorPID.setpoint / 2, 80, MotorPID.setpoint / 2);
            if (hhmg >= (MotorPID.setpoint*2)/3) {
                PressureModeStart = 3;
                SetMotorSpeed(0);
                Flag_3s = 0;
                osTimerStart(motor_grab3sHandle, 1000); // ����3�붨ʱ��
            }
            break;
        }
        case 2: { // ���ٽ׶�
            SetMotorSpeed((int)control_output_speed);
            break;
        }
        case 3: { // ѹ�����ƽ׶�
            MotorPWM = PID_Compute_motor(&MotorPID, hhmg);
            SetMotorSpeed((int)MotorPWM);

            if (Flag_3s) {
                PressureModeStart = 4; // �л������˽׶�
                Flag_1s = 0;
                osTimerStart(motor_back_1sHandle, 500); // ����1�붨ʱ��
            }
            break;
        }
        case 4: { // ���˽׶�
            SetMotorSpeed(-20000);

            if (Flag_1s) {
                PressureModeStart = 1; // �ص�ǰ׼���׶�
                Flag_3s = 0;
            }
            break;
        }
        default:
            break;
    }
}
