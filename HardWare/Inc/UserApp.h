//
// Created by zq on 2025/2/18.
//
#ifndef __UserApp
#define __UserApp


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "queue.h"
#include "main.h"
#include "event_groups.h"
/* 变量声明（使用 extern）*/
extern uint8_t white_delay, yellow_delay, breathing_flag;
extern uint8_t Flag_400ms;
extern float weight0;
extern volatile int Flag_3s, Flag_1s;
extern ChargeState_t ChargeState;
extern uint8_t soft_button;
extern uint8_t charging, working, low_battery, fully_charged, emergency_stop;
extern PID_TypeDef HeatPID;
extern float Heat_PWM, EyeTmp;
extern uint8_t flag_200ms;
extern uint8_t tempature_flag_400ms, press_flag_400ms, battery_flag_400ms;
extern uart_data *frameData_uart;

/* FreeRTOS 句柄 */
extern TaskHandle_t UART_RECEPTHandle, HeatHandle, PressHandle, Button_StateHandle, APPHandle, Uart_sendHandle, AD24C01Handle,motor_homeHandle;
extern QueueHandle_t UART_DMA_IDLE_RECEPT_QUEUEHandle, HEAT_DATAHandle, PRESS_DATAHandle, TemperatureHandle, PressureHandle, Battery_DATAHandle, prepare_dataHandle;
extern SemaphoreHandle_t BUTTON_SEMAPHOREHandle;
extern TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle, motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle;
extern EventGroupHandle_t HEAT_ONHandle, PRESS_ONHandle;
/* Task function prototypes */
void UART_RECEPT_Task(void *argument);
void Heat_Task(void *argument);
void Press_Task(void *argument);
void Button_State_Task(void *argument);
void APP_task(void *argument);
void Uart_send_task(void *argument);
void AD24C01_WR(void *argument);
void Motor_go_home_task(void *argument);

        void ws2812_white_delay_callback(TimerHandle_t xTimer);
void ws2812_yellow_callback(TimerHandle_t xTimer);
void breath_delay_Callback(TimerHandle_t xTimer);
void motor_grab3s_Callback(TimerHandle_t xTimer);
void motor_back_1sCallback(TimerHandle_t xTimer);
void buttton_Callback(TimerHandle_t xTimer);
void tempareture_pid_timer(TimerHandle_t xTimer);

void Main(void );
#endif /* __ADS1220_H */
