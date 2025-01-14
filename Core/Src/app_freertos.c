/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t white_delay;    // 状态变量（同时作为标志位和灯珠状态）
extern uint8_t yellow_delay;   // 状态变量（同时作为标志位和灯珠状态）
extern uint8_t breathing_flag; // 标志位
uint8_t Flag_400ms = 1;        // 串口发送标志位
volatile extern int Flag_3s, Flag_1s;
extern ChargeState_t ChargeState; // 当前状态
extern bool charging, working, low_battery, fully_charged, emergency_stop;
PID_TypeDef pid_heat;
float Heat_PWM; // 用于存储加热器的PWM占空比
/* USER CODE END Variables */
/* Definitions for UART_RECEPT */
osThreadId_t UART_RECEPTHandle;
const osThreadAttr_t UART_RECEPT_attributes = {
        .name = "UART_RECEPT",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 256 * 4};
/* Definitions for Heat */
osThreadId_t HeatHandle;
const osThreadAttr_t Heat_attributes = {.name = "Heat",
        .priority =
        (osPriority_t) osPriorityHigh,
        .stack_size = 256 * 4};
/* Definitions for Press */
osThreadId_t PressHandle;
const osThreadAttr_t Press_attributes = {.name = "Press",
        .priority =
        (osPriority_t) osPriorityHigh,
        .stack_size = 256 * 4};
/* Definitions for Button_State */
osThreadId_t Button_StateHandle;
const osThreadAttr_t Button_State_attributes = {
        .name = "Button_State",
        .priority = (osPriority_t) osPriorityHigh1,
        .stack_size = 256 * 4};
/* Definitions for APP */
osThreadId_t APPHandle;
const osThreadAttr_t APP_attributes = {.name = "APP",
        .priority = (osPriority_t) osPriorityHigh,
        .stack_size = 512 * 4};
/* Definitions for Uart_send */
osThreadId_t Uart_sendHandle;
const osThreadAttr_t Uart_send_attributes = {.name = "Uart_send",
        .priority =
        (osPriority_t) osPriorityHigh,
        .stack_size = 256 * 4};
/* Definitions for UART_DMA_IDLE_RECEPT_QUEUE */
osMessageQueueId_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
const osMessageQueueAttr_t UART_DMA_IDLE_RECEPT_QUEUE_attributes = {
        .name = "UART_DMA_IDLE_RECEPT_QUEUE"};
/* Definitions for HEAT_DATA */
osMessageQueueId_t HEAT_DATAHandle;
const osMessageQueueAttr_t HEAT_DATA_attributes = {.name = "HEAT_DATA"};
/* Definitions for PRESS_DATA */
osMessageQueueId_t PRESS_DATAHandle;
const osMessageQueueAttr_t PRESS_DATA_attributes = {.name = "PRESS_DATA"};
/* Definitions for Temperature */
osMessageQueueId_t TemperatureHandle;
const osMessageQueueAttr_t Temperature_attributes = {.name = "Temperature"};
/* Definitions for Pressure */
osMessageQueueId_t PressureHandle;
const osMessageQueueAttr_t Pressure_attributes = {.name = "Pressure"};
/* Definitions for Battery_DATA */
osMessageQueueId_t Battery_DATAHandle;
const osMessageQueueAttr_t Battery_DATA_attributes = {.name = "Battery_DATA"};
/* Definitions for ws2812_white_delay */
osTimerId_t ws2812_white_delayHandle;
const osTimerAttr_t ws2812_white_delay_attributes = {.name =
"ws2812_white_delay"};
/* Definitions for ws2812_yellow_delay */
osTimerId_t ws2812_yellow_delayHandle;
const osTimerAttr_t ws2812_yellow_delay_attributes = {
        .name = "ws2812_yellow_delay"};
/* Definitions for breath_delay */
osTimerId_t breath_delayHandle;
const osTimerAttr_t breath_delay_attributes = {.name = "breath_delay"};
/* Definitions for motor_grab3s */
osTimerId_t motor_grab3sHandle;
const osTimerAttr_t motor_grab3s_attributes = {.name = "motor_grab3s"};
/* Definitions for motor_back_1s */
osTimerId_t motor_back_1sHandle;
const osTimerAttr_t motor_back_1s_attributes = {.name = "motor_back_1s"};
/* Definitions for BUTTON_SEMAPHORE */
osSemaphoreId_t BUTTON_SEMAPHOREHandle;
const osSemaphoreAttr_t BUTTON_SEMAPHORE_attributes = {.name =
"BUTTON_SEMAPHORE"};
/* Definitions for HEAT_ON */
osEventFlagsId_t HEAT_ONHandle;
const osEventFlagsAttr_t HEAT_ON_attributes = {.name = "HEAT_ON"};
/* Definitions for PRESS_ON */
osEventFlagsId_t PRESS_ONHandle;
const osEventFlagsAttr_t PRESS_ON_attributes = {.name = "PRESS_ON"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void UART_RECEPT_Task(void *argument);

void Heat_Task(void *argument);

void Press_Task(void *argument);

void Button_State_Task(void *argument);

void APP_task(void *argument);

void Uart_send_task(void *argument);

void ws2812_white_delay_callback(void *argument);

void ws2812_yellow_callback(void *argument);

void breath_delay_Callback(void *argument);

void motor_grab3s_Callback(void *argument);

void motor_back_1sCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);

unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */

/* USER CODE END 1 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of BUTTON_SEMAPHORE */
    BUTTON_SEMAPHOREHandle = osSemaphoreNew(1, 0, &BUTTON_SEMAPHORE_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* creation of ws2812_white_delay */
    ws2812_white_delayHandle =
            osTimerNew(ws2812_white_delay_callback, osTimerOnce, NULL,
                       &ws2812_white_delay_attributes);

    /* creation of ws2812_yellow_delay */
    ws2812_yellow_delayHandle = osTimerNew(ws2812_yellow_callback, osTimerOnce,
                                           NULL, &ws2812_yellow_delay_attributes);

    /* creation of breath_delay */
    breath_delayHandle = osTimerNew(breath_delay_Callback, osTimerOnce, NULL,
                                    &breath_delay_attributes);

    /* creation of motor_grab3s */
    motor_grab3sHandle = osTimerNew(motor_grab3s_Callback, osTimerOnce, NULL,
                                    &motor_grab3s_attributes);

    /* creation of motor_back_1s */
    motor_back_1sHandle = osTimerNew(motor_back_1sCallback, osTimerOnce, NULL,
                                     &motor_back_1s_attributes);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of UART_DMA_IDLE_RECEPT_QUEUE */
    UART_DMA_IDLE_RECEPT_QUEUEHandle = osMessageQueueNew(
            5, sizeof(uart_data), &UART_DMA_IDLE_RECEPT_QUEUE_attributes);

    /* creation of HEAT_DATA */
    HEAT_DATAHandle =
            osMessageQueueNew(5, sizeof(PID_TypeDef), &HEAT_DATA_attributes);

    /* creation of PRESS_DATA */
    PRESS_DATAHandle =
            osMessageQueueNew(5, sizeof(PID_TypeDef), &PRESS_DATA_attributes);

    /* creation of Temperature */
    TemperatureHandle =
            osMessageQueueNew(5, sizeof(float), &Temperature_attributes);

    /* creation of Pressure */
    PressureHandle = osMessageQueueNew(5, sizeof(float), &Pressure_attributes);

    /* creation of Battery_DATA */
    Battery_DATAHandle =
            osMessageQueueNew(5, sizeof(float), &Battery_DATA_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of UART_RECEPT */
    UART_RECEPTHandle =
            osThreadNew(UART_RECEPT_Task, NULL, &UART_RECEPT_attributes);

    /* creation of Heat */
    HeatHandle = osThreadNew(Heat_Task, NULL, &Heat_attributes);

    /* creation of Press */
    PressHandle = osThreadNew(Press_Task, NULL, &Press_attributes);

    /* creation of Button_State */
    Button_StateHandle =
            osThreadNew(Button_State_Task, NULL, &Button_State_attributes);

    /* creation of APP */
    APPHandle = osThreadNew(APP_task, NULL, &APP_attributes);

    /* creation of Uart_send */
    Uart_sendHandle = osThreadNew(Uart_send_task, NULL, &Uart_send_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the event(s) */
    /* creation of HEAT_ON */
    HEAT_ONHandle = osEventFlagsNew(&HEAT_ON_attributes);

    /* creation of PRESS_ON */
    PRESS_ONHandle = osEventFlagsNew(&PRESS_ON_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_UART_RECEPT_Task */
/**
 * @brief  Function implementing the UART_RECEPT thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_RECEPT_Task */
void UART_RECEPT_Task(void *argument) {
    /* USER CODE BEGIN UART_RECEPT_Task */
    uart_data recept_data;
    // extern UART_HandleTypeDef huart2;
    /* Infinite loop */
    for (;;) {
        osDelay(10);
        if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &recept_data,
                          portMAX_DELAY) == pdPASS) { // 从队列中接收数据
            command_parsing((uint8_t *) &recept_data);   // 处理接收到的数据
            // HAL_UART_Transmit(&huart2, (uint8_t*)&recept_data.buffer,
            // recept_data.length, 10);
        }
    }
    /* USER CODE END UART_RECEPT_Task */
}

/* USER CODE BEGIN Header_Heat_Task */
/**
 * @brief Function implementing the Heat thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Heat_Task */
void Heat_Task(void *argument) {
    /* USER CODE BEGIN Heat_Task */
    /* Infinite loop */
    HeatInit();
    float EyeTmp; // 用于存储从TMP112读取的温度值
    for (;;) {
        // 等待 HEAT_ON 标志被设置，且不清除标志位
        osEventFlagsWait(HEAT_ONHandle, (1 << 0), osFlagsWaitAny | osFlagsNoClear,
                         osWaitForever);
        // printf("Heating started\n");
        //  持续执行加热任务，直到 HEAT_ON 标志被清除
        while (osEventFlagsGet(HEAT_ONHandle) &
               (1 << 0)) { // 非阻塞接收数据，队列为空时直接跳过
            if (xQueueReceive(HEAT_DATAHandle, &pid_heat, 0) == pdPASS) {
                // 队列中有数据，更新 last_data
            }
            EyeTmp = TmpRaw2Ture();                    // 模拟加热任务
            xQueueSend(TemperatureHandle, &EyeTmp, 0); // 将数据发送到队列
            Heat_PWM = PID_Compute(&pid_heat, EyeTmp);
            HeatPWMSet((uint8_t) Heat_PWM);
            osDelay(100);
            // printf("Heat Task Running: EyeTmp=%.2f, HeatPWM=%.2f\n",
            // EyeTmp,HeatPWM);
        }
        // 标志位被清除后退出任务逻辑
        // printf("Heating stopped\n");
        // HeatPWM(0)  ; // 加热PWM
    }
    osDelay(50);
    /* USER CODE END Heat_Task */
}

/* USER CODE BEGIN Header_Press_Task */
/**
 * @brief Function implementing the Press thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Press_Task */
void Press_Task(void *argument) {
    /* USER CODE BEGIN Press_Task */
    /* Infinite loop */

    ADS1220_Init(); // 初始化ADS1220
    TMC5130_Init();
    MotorChecking();
    for (;;) {
        // 等待 HEAT_ON 标志被设置，且不清除标志位
        osEventFlagsWait(PRESS_ONHandle, (1 << 0), osFlagsWaitAny | osFlagsNoClear,
                         osWaitForever);

        // 启动转换
        ADS1220_StartConversion();
        // printf("Press started\n");
        // 持续执行加热任务，直到 HEAT_ON 标志被清除
        while (osEventFlagsGet(PRESS_ONHandle) & (1 << 0)) {
            PressureControl();
        }
        // printf("Press Task Stop\n");
        //  标志位被清除后退出任务逻辑
        ADS1220_StopConversion();
        MotorChecking();
    }
    osDelay(50);
    /* USER CODE END Press_Task */
}

/* USER CODE BEGIN Header_Button_State_Task */
/**
 * @brief Function implementing the Button_State thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Button_State_Task */
void Button_State_Task(void *argument) {
    /* USER CODE BEGIN Button_State_Task */
    /* Infinite loop */
    for (;;) {
        // 等待信号量
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            Button_detection();
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // 防止过度占用CPU
    }
    /* USER CODE END Button_State_Task */
}

/* USER CODE BEGIN Header_APP_task */
/**
 * @brief Function implementing the APP thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APP_task */
void APP_task(void *argument) {
    /* USER CODE BEGIN APP_task */
    /* Infinite loop */

    AT24CXX_Init();
    BQ27441_Init();
    BQ25895_Init();
    PWM_WS2812B_Init();
    for (;;) {
        osDelay(10);
        LEDUpdate();
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();
    }
    /* USER CODE END APP_task */
}

/* USER CODE BEGIN Header_Uart_send_task */
/**
 * @brief Function implementing the Uart_send thread.
 * @param argument: Not used
 * @retval None
 */

/* USER CODE END Header_Uart_send_task */
void Uart_send_task(void *argument) {
    /* USER CODE BEGIN Uart_send_task */
    /* Infinite loop */

    float tempture, pressure, battery;
    for (;;) {
        osDelay(50);

        // char pcStatsBuffer[512];             // 用于存储任务统计信息
        // vTaskGetRunTimeStats(pcStatsBuffer); // 获取任务统计信息
        //  HAL_UART_Transmit(&huart2, (uint8_t *)pcStatsBuffer,
        //  strlen(pcStatsBuffer),
        if (xQueueReceive(PressureHandle, &pressure, 0) == pdPASS) {
            // printf(" pressure: %.2f\n ", pressure);
            if (Flag_400ms) {
                // if (1) {
                ScreenUpdateForce(pressure);
            }
        }
        osDelay(10);
        if (xQueueReceive(TemperatureHandle, &tempture, 0) == pdPASS) {
            // printf("tempture%.2f\n ", tempture);
            if (Flag_400ms) {
                // if (1) {
                ScreenUpdateTemperature((float) (tempture - temperature_compensation));
            }
        }
        if (xQueueReceive(Battery_DATAHandle, &battery, 0) == pdPASS) {
            // printf("battery: %.2f\n ",battery);
            if (Flag_400ms) {
                // if (1) {
                ScreenUpdateSOC(battery);
            }
        }
        Flag_400ms = 0; // 串口发送标志位
    }
    /* USER CODE END Uart_send_task */
}

/* ws2812_white_delay_callback function */
void ws2812_white_delay_callback(void *argument) {
    /* USER CODE BEGIN ws2812_white_delay_callback */
    Flag_400ms = 1;                              // 串口发送标志位
    white_delay = !white_delay;                  // 切换状态
    osTimerStart(ws2812_white_delayHandle, 400); // 重新启动定时器
    /* USER CODE END ws2812_white_delay_callback */
}

/* ws2812_yellow_callback function */
void ws2812_yellow_callback(void *argument) {
    /* USER CODE BEGIN ws2812_yellow_callback */
    emergency_stop = false;
    yellow_delay = 0; // 切换状态
    /* USER CODE END ws2812_yellow_callback */
}

/* breath_delay_Callback function */
void breath_delay_Callback(void *argument) {
    /* USER CODE BEGIN breath_delay_Callback */
    breathing_flag = 1; // 设置标志位

    /* USER CODE END breath_delay_Callback */
}

/* motor_grab3s_Callback function */
void motor_grab3s_Callback(void *argument) {
    /* USER CODE BEGIN motor_grab3s_Callback */
    Flag_3s = 1;
    /* USER CODE END motor_grab3s_Callback */
}

/* motor_back_1sCallback function */
void motor_back_1sCallback(void *argument) {
    /* USER CODE BEGIN motor_back_1sCallback */
    Flag_1s = 1;
    /* USER CODE END motor_back_1sCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
