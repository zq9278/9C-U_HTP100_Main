
#include "main.h"
#include <stdint.h>
#include "tmc5130.h"
#include "pid.h"
#include "ads1220.h"
#include "UserApp.h"
#include "time_callback.h"

#define FORCE_FILTER_ALPHA 0.2f

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
/*motor*/
uint32_t MotorSpeed = 0x4000;

extern SPI_HandleTypeDef hspi1;

PID_TypeDef MotorPID;

void TMC5130_Init(void) {
    //	TMC_ENN(0);//???��?��??�??��?��???
    //HAL_Delay(20);
    TMC5130_Write(0x81, 0x00000001); // reset
    TMC5130_Write(0xec, 0x000300c3); // CHOPCONF: vsense=1,TOFF=3, HSTRT=4,
    // HEND=1, TBL=2, CHM=0 (spreadCycle)

    TMC5130_Write(0x90, 0x00001006); // IHOLD=6, IRUN=16, IHOLDDELAY=6
    /*
     *IHOLD ���е���
     * IRUN   ���е���
     * IHOLDDELAY Խ��Խƽ����ԽСԽͻ��
     * */
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
    TMC5130_Write(0xA4, 0x00000001); // A1 ��С��ʼ���ٶȣ�����Ϊ0����СΪ1��
    TMC5130_Write(0xA6, 0x00001001); // AMAX ��С�����ٶȣ�����Ϊ0����СΪ1��
    TMC5130_Write(0xa7, MotorSpeed); // VMAX = 200 000
    TMC5130_Write(0xa8, 0x00001fff); // DMAX = 700 Deceleration above V1
    TMC5130_Write(0xaa, 0x00008000); // D1 = 1400 Deceleration below V1
    TMC5130_Write(0xab, 0x0000000a); // VSTOP = 10 Stop velocity (Near to zero)
    // TMC5130_Write(0xac, 0x00000000);
    // TMC5130_Write(0xb4, 0x0000075f);
    TMC5130_Write(0xa0, 0x00000000); // �????��?�模�?
    PID_Init(&MotorPID, 300, 0, 0, 5000, -5000, (float) (50000), (float) (-50000),
             0); // 0.03,0.05//0.02, 0.01, 0.02,
}

// �ж��շ�����
// ����ȫ�ֱ�������״̬
volatile uint8_t SPI_RxComplete = 0;
uint8_t TxBuffer[5];
uint8_t RxBuffer[4];

void TMC5130_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    // ���״̬��־
    SPI_RxComplete = 0;
    // ��ʼ������
    TxBuffer[0] = ReadAddr;
    // ��һ�η��ͣ�����5���ֽ�
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 5);
    TMC_CSN(1);
    // ��ʱȷ��Ӳ���ȶ�����ѡ�����ݾ���Ӳ������
    // HAL_Delay(1); // ���߲�����ʵ�NOPָ��
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 1);
    SPI_RxComplete = 0; // ���ý��ձ�־
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
    // ����ǰλ�üĴ�����XACTUAL�����㣬�߼���������Ϊ 0
    TMC5130_Write(0xA1, 0);
    // ��Ŀ��λ�üĴ�����XTARGET�����㣬�������ƶ�
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

        // ���Ŀ��״̬������Ŀ��״̬Ϊ״̬�ֵ�3�ֽڵ� bit1 Ϊ 1��
        if ((ReadData[3] & 0x02) == 0x02) {
            // ״̬���㣬�˳�ѭ��
            break;
        }
        vTaskDelay(100);
    }

    MotorSetHome();
    TMC5130_Write(0xa0, 0x00000000); // �????��?�模�?
    TMC_ENN(1);

    return 1;
}

uint8_t MotorCompare(int32_t SetData, int32_t CompareData) {
    int32_t SubData;
    SubData = CompareData - SetData;
    if (SubData > 0) // ForceSen  ????�?
    {
        TMC5130_Write(0xa7, 0x6000);
        TMC5130_Write(0xa0, 2);
        return 2;
    } else if (SubData < 0) // ????�?
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

// TMC_ENN(0); // 使�?��?��??
// TMC5130_Write(0xad, 115000); // �???��?�?
/*pid?��?��?�?*/
void SetMotorposition(int position) {
    TMC5130_Write(0xa7, 1000);
    TMC5130_Write(0xa0, 0);
    TMC5130_Write(0xad, (uint32_t) position);
}

/*pid?��?��??�?*/
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
float MotorPWM;       // ���ڴ洢��������PWMռ�ձ�
extern float weight0; // test_variable
PID_TypeDef MotorPID;

uint8_t PressureModeStart = 1;
float control_output;
float control_output_speed;
//volatile int Flag_3s = 0, Flag_1s = 0;
//extern osMessageQueueId_t PRESS_DATAHandle; // ���о��
//extern osTimerId_t motor_back_1sHandle;
//extern osTimerId_t motor_grab3sHandle;
//extern osMessageQueueId_t PressureHandle;
extern float p, i, d;
extern uint8_t flag_200ms;

//void PressureControl(void) {
//
//    weight1 = ADS1220_ReadPressure();
//    weight = weight1 - weight0;
//    Limit(weight, 0, weight);
//    float hhmg = (weight / 1000.0) * 9.8 * 78;
////    printf("%.2f,",hhmg);
////    printf("%.2f,",MotorPID.setpoint);
////    printf("%.2f,%.2f,%.2f\n",p,i,d);
//    //float hhmg =weight;
//
//    if (1) {
//        //flag_200ms=0;
//        xQueueSend(PressureHandle, &hhmg, 0); // �����ݷ��͵�����
//    }
//    switch (PressureModeStart) {
//        case 1: // �ʼ��ǰ���׶�
//            control_output_speed = 20000;
//            SetMotorSpeed((int) control_output_speed);
//            float start_force_flag = MotorPID.setpoint / 2;
//            start_force_flag = Limit(start_force_flag, 80, start_force_flag);
//            if (hhmg >= start_force_flag) {
//                PressureModeStart = 3;
//                control_output_speed = 0;
//                SetMotorSpeed((int) control_output_speed);
//                Flag_3s = 0;
//                osTimerStart(motor_grab3sHandle, 1000); // ����3�붨ʱ��
//            }
//            break;
//        case 2:
//            SetMotorSpeed((int) control_output_speed);
//            break;
//        case 3: // ѹ�����ֽ׶�
//
//            MotorPWM = PID_Compute(&MotorPID, hhmg);
//            SetMotorSpeed((int) MotorPWM);
//
//            if (Flag_3s == 1) {
//                PressureModeStart = 4; // �л������˽׶�
//                Flag_1s = 0;
//                osTimerStart(motor_back_1sHandle, 500); // ����1�붨ʱ��
//            }
//            break;
//        case 4: // ���˽׶�
//            control_output_speed = -20000;
//            SetMotorSpeed((int) control_output_speed);
//
//            if (Flag_1s == 1) {
//                PressureModeStart = 1; // ����ǰ���׶�
//                Flag_3s = 0;
//            }
//            break;
//        default:
//            break;
//    }
//    //printf("hhmg = %.2f,MotorPID.setpoint%.2f,MotorPWM%.2f\n",hhmg,MotorPID.setpoint,MotorPWM);
//}
extern uint8_t press_flag_400ms;
void PressureControl() {
    float weight1 = ADS1220_ReadPressure();
    float weight = Limit(weight1 - weight0, 0, weight1 - weight0);
    float hhmg = (weight / 1000.0) * 9.8 * 80;
    float hhmg_filtered = FilterForce(hhmg);

    // Push filtered pressure data when screen refresh flag is set
    if (press_flag_400ms) {
        press_flag_400ms = 0;
        ScreenUpdateForce(hhmg_filtered);
        //xQueueSend(PressureHandle, &hhmg_filtered, 0);
    }
    switch (PressureModeStart) {
        case 1: { // �ʼ��ǰ���׶�
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
        case 2: { // �����ٶȽ׶�
            SetMotorSpeed((int)control_output_speed);
            break;
        }
        case 3: { // ѹ�����ֽ׶�
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
                PressureModeStart = 1; // ����ǰ���׶�
                Flag_3s = 0;
            }
            break;
        }
        default:
            break;
    }
}
