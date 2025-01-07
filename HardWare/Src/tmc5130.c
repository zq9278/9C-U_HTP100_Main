
#include "main.h"
#include <stdint.h>
/*motor*/
uint32_t MotorSpeed = 0x4000;

extern SPI_HandleTypeDef hspi1;

PID_TypeDef MotorPID;
void TMC5130_Init(void) {
  //	TMC_ENN(0);//閿熸枻鎷烽敓缁炵櫢鎷烽敓?
  HAL_Delay(20);
  TMC5130_Write(0x81, 0x00000001); // reset
  TMC5130_Write(0xec, 0x000300c3); // CHOPCONF: vsense=1,TOFF=3, HSTRT=4,
                                   // HEND=1, TBL=2, CHM=0 (spreadCycle)
  TMC5130_Write(
      0x90,
      0x00001000); // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
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

  TMC5130_Write(0xa4, 0x00011000); // A1 = 1 000 First acceleration
  TMC5130_Write(0xa5,
                0x00015000); // V1 = 50 000 Acceleration threshold velocity V1
  TMC5130_Write(0xa6, 0x00018fff); // AMAX = 500 Acceleration above V1
  TMC5130_Write(0xa7, MotorSpeed); // VMAX = 200 000
  TMC5130_Write(0xa8, 0x00001fff); // DMAX = 700 Deceleration above V1
  TMC5130_Write(0xaa, 0x00008000); // D1 = 1400 Deceleration below V1
  TMC5130_Write(0xab, 0x0000000a); // VSTOP = 10 Stop velocity (Near to zero)
  // TMC5130_Write(0xac, 0x00000000);
  // TMC5130_Write(0xb4, 0x0000075f);
  TMC5130_Write(0xa0, 0x00000000); // 浣嶉敓鏂ゆ嫹妯″紡
  PID_Init(&MotorPID, 75, 2, 5, 1000, -1000, (float)(50000), (float)(-50000),
           0); // 0.03,0.05//0.02, 0.01, 0.02,
}

// 中断收发函数
// 定义全局变量保存状态
volatile uint8_t SPI_RxComplete = 0;
uint8_t TxBuffer[5];
uint8_t RxBuffer[4];
void TMC5130_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    // 清除状态标志
    SPI_RxComplete = 0;
    // 初始化数据
    TxBuffer[0] = ReadAddr;
    // 第一次发送，发送5个字节
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 5);
    TMC_CSN(1);
    // 延时确保硬件稳定（可选，根据具体硬件需求）
    // HAL_Delay(1); // 或者插入合适的NOP指令
    TMC_CSN(0);
    HAL_SPI_Transmit_IT(&hspi1, TxBuffer, 1);
    SPI_RxComplete = 0; // 重置接收标志
    HAL_SPI_Receive_IT(&hspi1, pBuffer, 4);
    while (!SPI_RxComplete); // 等待接收完成
    TMC_CSN(1);
}

void TMC5130_Write(uint8_t WriteAddr, uint32_t WriteData) {
  uint8_t Data[5];
  Data[0] = WriteAddr;
  Data[1] = (uint8_t)((WriteData) >> 24);
  Data[2] = (uint8_t)((WriteData) >> 16);
  Data[3] = (uint8_t)((WriteData) >> 8);
  Data[4] = (uint8_t)WriteData;
  TMC_CSN(0);
  HAL_SPI_Transmit_IT(&hspi1, Data, 5);
  TMC_CSN(1);
}

void MotorSetHome(void) {
    // 将当前位置寄存器（XACTUAL）清零，逻辑坐标重置为 0
    TMC5130_Write(0xA1, 0);
    // 将目标位置寄存器（XTARGET）清零，避免电机移动
    TMC5130_Write(0xAD, 0);
}

void MotorCtrl(int32_t Step) {
  uint8_t Data[5];
  Data[0] = 0xad;
  Data[1] = (uint8_t)((Step) >> 24);
  Data[2] = (uint8_t)((Step) >> 16);
  Data[3] = (uint8_t)((Step) >> 8);
  Data[4] = (uint8_t)Step;
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
#define TIMEOUT_LIMIT 5000  // 超时限制，单位为系统节拍（tick），假设系统节拍是1ms
uint8_t MotorChecking() {
  uint8_t ReadData[4];

  TMC_ENN(0); // 
  TMC5130_Write(0xa7, 0x10000);
  VelocityModeMove(Positive);
  uint32_t start_time = xTaskGetTickCount();  // 获取当前tick计数值
    uint32_t timeout_ticks = pdMS_TO_TICKS(TIMEOUT_LIMIT);  // 转换超时时间为tick

    while (1) {
        // 读取电机状态寄存器
        TMC5130_Read(0x04, ReadData);
        
        // 检查目标状态（假设目标状态为状态字第3字节的 bit1 为 1）
        if ((ReadData[3] & 0x02) == 0x02) {
            // 状态满足，退出循环
            break;
        }

        // 检查超时
        if ((xTaskGetTickCount() - start_time) >= timeout_ticks) {
            // 超时处理
            break;
        }
        vTaskDelay(100);
        }

  MotorSetHome();
  TMC5130_Write(0xa0, 0x00000000); // 浣嶉敓鏂ゆ嫹妯″紡
  TMC_ENN(1);

  return 1;
}

uint8_t MotorCompare(int32_t SetData, int32_t CompareData) {
  int32_t SubData;
  SubData = CompareData - SetData;
  if (SubData > 0) // ForceSen  鐡掑懍绨?
  {
    TMC5130_Write(0xa7, 0x6000);
    TMC5130_Write(0xa0, 2);
    return 2;
  } else if (SubData < 0) // 鐏忔垳绨?
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

// TMC_ENN(0); // 浣胯兘鐢垫満
// TMC5130_Write(0xad, 115000); // 缁濆?逛綅缃?
/*pid鎺у埗浣嶇疆*/
void SetMotorposition(int position) {
  TMC5130_Write(0xa7, 1000);
  TMC5130_Write(0xa0, 0);
  TMC5130_Write(0xad, (uint32_t)position);
}
/*pid鎺у埗閫熷害*/
void SetMotorSpeed(int speed) {
  if (speed < 0) {
    TMC5130_Write(0xa0, 1);
    TMC5130_Write(0xa7, (uint32_t)(-speed));
  } else {
    TMC5130_Write(0xa0, 2);
    TMC5130_Write(0xa7, (uint32_t)speed);
  }
}

float weight;
float MotorPWM;       // 用于存储加热器的PWM占空比
extern float weight0; // test_variable
PID_TypeDef pid_motor;
extern osMessageQueueId_t PRESS_DATAHandle; // 队列句柄
uint8_t PressureModeStart = 1;
float control_output;
float control_output_speed;
volatile int Flag_3s = 0, Flag_1s = 0;
extern osTimerId_t motor_back_1sHandle;
extern osTimerId_t motor_grab3sHandle;
extern osMessageQueueId_t PressureHandle;
void PressureControl(void) {
  // 非阻塞接收数据，队列为空时直接跳过
  if (xQueueReceive(PRESS_DATAHandle, &pid_motor, 0) == pdPASS) {
    // 队列中有数据，更新 last_data
  }

  weight = ADS1220_ReadPressure();
  weight = weight - weight0;
  Limit(weight, 0, weight);
  float hhmg = (weight/1000.0)*9.8*88.4;
xQueueSend(PressureHandle, &hhmg,0); // 将数据发送到队列
  switch (PressureModeStart) {
  case 1: // 最开始的前进阶段
    control_output_speed = 20000;
    SetMotorSpeed((int)control_output_speed);
    float start_force_flag = 0;
    start_force_flag = Limit(start_force_flag, 180, pid_motor.setpoint / 2);
    if (hhmg >= start_force_flag) {
      PressureModeStart = 3;
      control_output_speed = 0;
      SetMotorSpeed((int)control_output_speed);
      Flag_3s = 0;
      osTimerStart(motor_grab3sHandle, 3000); // 启动3秒定时器
    }
    break;
  case 2:
    SetMotorSpeed((int)control_output_speed);
    break;
  case 3: // 压力保持阶段
    MotorPWM = PID_Compute(&pid_motor, hhmg);
    SetMotorSpeed((int)MotorPWM);

    if (Flag_3s == 1) {
      PressureModeStart = 4; // 切换到回退阶段
      Flag_1s = 0;
      osTimerStart(motor_back_1sHandle, 1000); // 启动1秒定时器
    }
    break;
  case 4: // 回退阶段
    control_output_speed = -10000;
    SetMotorSpeed((int)control_output_speed);

    if (Flag_1s == 1) {
      PressureModeStart = 1; // 返回前进阶段
      Flag_3s = 0;
    }
    break;
  default:
    break;
  }
  //printf("hhmg = %.2f,pid_motor.setpoint%.2f,MotorPWM%.2f\n",hhmg,pid_motor.setpoint,MotorPWM);
}