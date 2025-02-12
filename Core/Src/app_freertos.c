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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
float weight0 = 0;
volatile extern int Flag_3s, Flag_1s;
extern ChargeState_t ChargeState; // 当前状态
uint8_t soft_button;
extern uint8_t charging, working, low_battery, fully_charged, emergency_stop;
PID_TypeDef pid_heat;
extern PID_TypeDef HeatPID;
float Heat_PWM; // 用于存储加热器的PWM占空比
float EyeTmp; // 用于存储从TMP112读取的温度值
uint8_t tempareture_pid_time;
/* USER CODE END Variables */
/* Definitions for UART_RECEPT */
osThreadId_t UART_RECEPTHandle;
const osThreadAttr_t UART_RECEPT_attributes = {
  .name = "UART_RECEPT",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 512 * 4
};
/* Definitions for Heat */
osThreadId_t HeatHandle;
const osThreadAttr_t Heat_attributes = {
  .name = "Heat",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for Press */
osThreadId_t PressHandle;
const osThreadAttr_t Press_attributes = {
  .name = "Press",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for Button_State */
osThreadId_t Button_StateHandle;
const osThreadAttr_t Button_State_attributes = {
  .name = "Button_State",
  .priority = (osPriority_t) osPriorityHigh1,
  .stack_size = 256 * 4
};
/* Definitions for APP */
osThreadId_t APPHandle;
const osThreadAttr_t APP_attributes = {
  .name = "APP",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for Uart_send */
osThreadId_t Uart_sendHandle;
const osThreadAttr_t Uart_send_attributes = {
  .name = "Uart_send",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for AD24C01 */
osThreadId_t AD24C01Handle;
const osThreadAttr_t AD24C01_attributes = {
  .name = "AD24C01",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for UART_DMA_IDLE_RECEPT_QUEUE */
osMessageQueueId_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
const osMessageQueueAttr_t UART_DMA_IDLE_RECEPT_QUEUE_attributes = {
  .name = "UART_DMA_IDLE_RECEPT_QUEUE"
};
/* Definitions for HEAT_DATA */
osMessageQueueId_t HEAT_DATAHandle;
const osMessageQueueAttr_t HEAT_DATA_attributes = {
  .name = "HEAT_DATA"
};
/* Definitions for PRESS_DATA */
osMessageQueueId_t PRESS_DATAHandle;
const osMessageQueueAttr_t PRESS_DATA_attributes = {
  .name = "PRESS_DATA"
};
/* Definitions for Temperature */
osMessageQueueId_t TemperatureHandle;
const osMessageQueueAttr_t Temperature_attributes = {
  .name = "Temperature"
};
/* Definitions for Pressure */
osMessageQueueId_t PressureHandle;
const osMessageQueueAttr_t Pressure_attributes = {
  .name = "Pressure"
};
/* Definitions for Battery_DATA */
osMessageQueueId_t Battery_DATAHandle;
const osMessageQueueAttr_t Battery_DATA_attributes = {
  .name = "Battery_DATA"
};
/* Definitions for prepare_data */
osMessageQueueId_t prepare_dataHandle;
const osMessageQueueAttr_t prepare_data_attributes = {
  .name = "prepare_data"
};
/* Definitions for ws2812_white_delay */
osTimerId_t ws2812_white_delayHandle;
const osTimerAttr_t ws2812_white_delay_attributes = {
  .name = "ws2812_white_delay"
};
/* Definitions for ws2812_yellow_delay */
osTimerId_t ws2812_yellow_delayHandle;
const osTimerAttr_t ws2812_yellow_delay_attributes = {
  .name = "ws2812_yellow_delay"
};
/* Definitions for breath_delay */
osTimerId_t breath_delayHandle;
const osTimerAttr_t breath_delay_attributes = {
  .name = "breath_delay"
};
/* Definitions for motor_grab3s */
osTimerId_t motor_grab3sHandle;
const osTimerAttr_t motor_grab3s_attributes = {
  .name = "motor_grab3s"
};
/* Definitions for motor_back_1s */
osTimerId_t motor_back_1sHandle;
const osTimerAttr_t motor_back_1s_attributes = {
  .name = "motor_back_1s"
};
/* Definitions for buttton */
osTimerId_t butttonHandle;
const osTimerAttr_t buttton_attributes = {
  .name = "buttton"
};
/* Definitions for tempareture_pid */
osTimerId_t tempareture_pidHandle;
const osTimerAttr_t tempareture_pid_attributes = {
  .name = "tempareture_pid"
};
/* Definitions for BUTTON_SEMAPHORE */
osSemaphoreId_t BUTTON_SEMAPHOREHandle;
const osSemaphoreAttr_t BUTTON_SEMAPHORE_attributes = {
  .name = "BUTTON_SEMAPHORE"
};
/* Definitions for HEAT_ON */
osEventFlagsId_t HEAT_ONHandle;
const osEventFlagsAttr_t HEAT_ON_attributes = {
  .name = "HEAT_ON"
};
/* Definitions for PRESS_ON */
osEventFlagsId_t PRESS_ONHandle;
const osEventFlagsAttr_t PRESS_ON_attributes = {
  .name = "PRESS_ON"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void UART_RECEPT_Task(void *argument);
void Heat_Task(void *argument);
void Press_Task(void *argument);
void Button_State_Task(void *argument);
void APP_task(void *argument);
void Uart_send_task(void *argument);
void AD24C01_WR(void *argument);
void ws2812_white_delay_callback(void *argument);
void ws2812_yellow_callback(void *argument);
void breath_delay_Callback(void *argument);
void motor_grab3s_Callback(void *argument);
void motor_back_1sCallback(void *argument);
void buttton_Callback(void *argument);
void tempareture_pid_timer(void *argument);

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
  ws2812_white_delayHandle = osTimerNew(ws2812_white_delay_callback, osTimerOnce, NULL, &ws2812_white_delay_attributes);

  /* creation of ws2812_yellow_delay */
  ws2812_yellow_delayHandle = osTimerNew(ws2812_yellow_callback, osTimerOnce, NULL, &ws2812_yellow_delay_attributes);

  /* creation of breath_delay */
  breath_delayHandle = osTimerNew(breath_delay_Callback, osTimerOnce, NULL, &breath_delay_attributes);

  /* creation of motor_grab3s */
  motor_grab3sHandle = osTimerNew(motor_grab3s_Callback, osTimerOnce, NULL, &motor_grab3s_attributes);

  /* creation of motor_back_1s */
  motor_back_1sHandle = osTimerNew(motor_back_1sCallback, osTimerOnce, NULL, &motor_back_1s_attributes);

  /* creation of buttton */
  butttonHandle = osTimerNew(buttton_Callback, osTimerPeriodic, NULL, &buttton_attributes);

  /* creation of tempareture_pid */
  tempareture_pidHandle = osTimerNew(tempareture_pid_timer, osTimerPeriodic, NULL, &tempareture_pid_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_DMA_IDLE_RECEPT_QUEUE */
  UART_DMA_IDLE_RECEPT_QUEUEHandle = osMessageQueueNew (5, sizeof(uart_data), &UART_DMA_IDLE_RECEPT_QUEUE_attributes);

  /* creation of HEAT_DATA */
  HEAT_DATAHandle = osMessageQueueNew (5, sizeof(PID_TypeDef), &HEAT_DATA_attributes);

  /* creation of PRESS_DATA */
  PRESS_DATAHandle = osMessageQueueNew (5, sizeof(PID_TypeDef), &PRESS_DATA_attributes);

  /* creation of Temperature */
  TemperatureHandle = osMessageQueueNew (5, sizeof(float), &Temperature_attributes);

  /* creation of Pressure */
  PressureHandle = osMessageQueueNew (5, sizeof(float), &Pressure_attributes);

  /* creation of Battery_DATA */
  Battery_DATAHandle = osMessageQueueNew (5, sizeof(float), &Battery_DATA_attributes);

  /* creation of prepare_data */
  prepare_dataHandle = osMessageQueueNew (10, sizeof(prepare_data), &prepare_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_RECEPT */
  UART_RECEPTHandle = osThreadNew(UART_RECEPT_Task, NULL, &UART_RECEPT_attributes);

  /* creation of Heat */
  HeatHandle = osThreadNew(Heat_Task, NULL, &Heat_attributes);

  /* creation of Press */
  PressHandle = osThreadNew(Press_Task, NULL, &Press_attributes);

  /* creation of Button_State */
  Button_StateHandle = osThreadNew(Button_State_Task, NULL, &Button_State_attributes);

  /* creation of APP */
  APPHandle = osThreadNew(APP_task, NULL, &APP_attributes);

  /* creation of Uart_send */
  Uart_sendHandle = osThreadNew(Uart_send_task, NULL, &Uart_send_attributes);

  /* creation of AD24C01 */
  AD24C01Handle = osThreadNew(AD24C01_WR, NULL, &AD24C01_attributes);

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
void UART_RECEPT_Task(void *argument)
{
  /* USER CODE BEGIN UART_RECEPT_Task */
  uart_data recept_data;
      uart_data *frameData; // 定义数据指针
  // extern UART_HandleTypeDef huart2;
  /* Infinite loop */
  for (;;) {
//    osDelay(10);
//    if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &recept_data,
//                      portMAX_DELAY) == pdPASS) { // 从队列中接收数据
//      command_parsing((uint8_t *)&recept_data);   // 处理接收到的数据
//      // HAL_UART_Transmit(&huart2, (uint8_t*)&recept_data.buffer,
//      // recept_data.length, 10);
//    }
      if (osMessageQueueGet(UART_DMA_IDLE_RECEPT_QUEUEHandle, &frameData, NULL, osWaitForever) == osOK) { // 从消息队列中获取数据
          if (frameData == NULL) {
              printf("Error: Received NULL frameData pointer\n");
              continue;
          }
          for (uint16_t i = 0; i < frameData->length - 1; i++) { // 遍历缓冲区
              if (frameData->buffer[i] == FRAME_HEADER_BYTE1 && i + 1 < frameData->length && frameData->buffer[i + 1] == FRAME_HEADER_BYTE2) { // 检测帧头
                  for (uint16_t j = i + 2; j < frameData->length - 1; j++) { // 继续查找帧尾
                      if (frameData->buffer[j] == FRAME_TAIL_BYTE1 && j + 1 < frameData->length && frameData->buffer[j + 1] == FRAME_TAIL_BYTE2) { // 检测帧尾
                          uint16_t frame_size = j - i + 2; // 计算完整帧大小
                          if (frame_size > UART_RX_BUFFER_SIZE) {
                              printf("Error: Frame size exceeds buffer limit\n");
                              break;
                          }
                          // 解析完整帧
                          command_parsing(&frameData->buffer[i]);
                          // 跳过已解析的帧数据，避免重复解析
                          i = j + 1;
                          break; // 结束当前帧尾查找
                      }
                  }
              }
          }


      } else {
          printf("Error: Failed to get data from queue\n");
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
void Heat_Task(void *argument)
{
  /* USER CODE BEGIN Heat_Task */
  /* Infinite loop */
  HeatInit();

  for (;;) {
    // 等待 HEAT_ON 标志被设置，且不清除标志位
    osEventFlagsWait(HEAT_ONHandle, (1 << 0), osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
     printf("Heating started\n");
      osTimerStart(tempareture_pidHandle,10);
    //  持续执行加热任务，直到 HEAT_ON 标志被清除
    while (osEventFlagsGet(HEAT_ONHandle) &
           (1 << 0)) { // 非阻塞接收数据，队列为空时直接跳过
//      if (xQueueReceive(HEAT_DATAHandle, &pid_heat, 0) == pdPASS) {
//        // 队列中有数据，更新 last_data
//      }
      EyeTmp = TmpRaw2Ture();                    // 模拟加热任务
      xQueueSend(TemperatureHandle, &EyeTmp, 0); // 将数据发送到队列
//        if (tempareture_pid_time==1){
//            tempareture_pid_time=0;
//
//        }
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
      Heat_PWM = PID_Compute(&HeatPID, EyeTmp);
      //Heat_PWM=250;
//        printf("%.2f,",EyeTmp);
//        printf("%.2f\n",Heat_PWM);
      HeatPWMSet((uint8_t)Heat_PWM);
      osDelay(100);
      // printf("Heat Task Running: EyeTmp=%.2f, HeatPWM=%.2f\n",EyeTmp,HeatPWM);

    }
    // 标志位被清除后退出任务逻辑
     printf("Heating stopped\n");
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
void Press_Task(void *argument)
{
  /* USER CODE BEGIN Press_Task */
  /* Infinite loop */

  ADS1220_Init(); // 初始化ADS1220
  TMC5130_Init();
  MotorChecking();
  for (;;) {
    // 等待 HEAT_ON 标志被设置，且不清除标志位
    osEventFlagsWait(PRESS_ONHandle, (1 << 0), osFlagsWaitAny | osFlagsNoClear,osWaitForever);
    ADS1220_StartConversion();  // 启动转换
    osDelay(20);//防止ADC芯片没反应过来
      Discard_dirty_data();
    weight0 = ADS1220_ReadPressure();           // 读取初始压力值
     printf("Press started\n");
    while (osEventFlagsGet(PRESS_ONHandle) & (1 << 0)) {// 持续执行挤压任务，直到 PRESS_ONHandle 标志被清除
      PressureControl();
    }
     printf("Press Task Stop\n");
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
void Button_State_Task(void *argument)
{
  /* USER CODE BEGIN Button_State_Task */
  /* Infinite loop */
  for (;;) {
    // 等待信号量触发
    if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
      // 消抖延时，等待按键状态稳定
      osDelay(20);
      // 检查按键状态是否仍为按下
      if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET)||(soft_button==1)) {
        // 按键稳定，执行按键逻辑
        Button_detection();
        soft_button=0;
      }
    }

    // 延时，避免任务过度占用 CPU
    vTaskDelay(100);
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
void APP_task(void *argument)
{
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
void Uart_send_task(void *argument)
{
  /* USER CODE BEGIN Uart_send_task */
  /* Infinite loop */

  float tempture, pressure, battery, tempture_temp;
  static float lastBatteryValue = 0;// 声明一个变量用于存储上一次的值
  static float lastTempValue = 0;
  prepare_data myprepare_data;
  for (;;) {
    osDelay(50);
    if (xQueueReceive(PressureHandle, &pressure, 0) == pdPASS) {
      if (Flag_400ms) {
        // if (1) {
        Limit(pressure,0,pressure);
        ScreenUpdateForce(pressure);
      }
    }
    osDelay(10);
    if (xQueueReceive(TemperatureHandle, &tempture, 0) == pdPASS) {
      if (Flag_400ms) {
        // if (1) {
         tempture_temp=tempture - temperature_compensation;
//        if(lastTempValue==0){//检查是否是第一次传输温度
//          lastTempValue = tempture_temp;
//        }
//        float diff = fabs(tempture_temp - lastTempValue); // 检查差值是否小于等于 30
//        if (diff <= 30.0f) {// 差值符合条件，记录和更新当前值
//          lastTempValue = tempture_temp;//如果温度差值小于30，说明是正常更新，差值大于30说明重启了，重启的话UI界面不更新
          if (Flag_400ms&&(tempture_temp!=0.0f)) {
            Limit(tempture_temp,0,tempture_temp);
            ScreenUpdateTemperature(tempture_temp);
          }
        }

    }
    if (xQueueReceive(Battery_DATAHandle, &battery, 0) == pdPASS) {
      if(lastBatteryValue==0){
        lastBatteryValue = battery;
      }
      float diff = fabs(battery - lastBatteryValue); // 检查差值是否小于等于 10
      if (diff <= 10.0f) {// 差值符合条件，记录和更新当前值
        lastBatteryValue = battery;
        if (Flag_400ms) {
          ScreenUpdateSOC(battery);
        }
      }
    }
    if (xQueueReceive(prepare_dataHandle, &myprepare_data, 0) == pdPASS) {
      osDelay(10);
      Eye_twitching_invalid_master(&myprepare_data);

      //printf("-------------\n");
    }
    Flag_400ms = 0; // 串口发送标志位
    //printf("%.2f,%.2f,%.2f\n",pressure,tempture_temp,battery);
  }
  /* USER CODE END Uart_send_task */
}

/* USER CODE BEGIN Header_AD24C01_WR */
/**
* @brief Function implementing the AD24C01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AD24C01_WR */
void AD24C01_WR(void *argument)
{
  /* USER CODE BEGIN AD24C01_WR */
  AD24C01_Factory_formatted();
  osDelay(1000);
  prepare_data_set();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    //AT24C02_WriteAllBytes(0xff);//清理ee存储
    //printf("EEPROM is format\n");
    //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
  }
  /* USER CODE END AD24C01_WR */
}

/* ws2812_white_delay_callback function */
void ws2812_white_delay_callback(void *argument)
{
  /* USER CODE BEGIN ws2812_white_delay_callback */
  Flag_400ms = 1;                              // 串口发送标志位
  white_delay = !white_delay;                  // 切换状态
  osTimerStart(ws2812_white_delayHandle, 400); // 重新启动定时器
  /* USER CODE END ws2812_white_delay_callback */
}

/* ws2812_yellow_callback function */
void ws2812_yellow_callback(void *argument)
{
  /* USER CODE BEGIN ws2812_yellow_callback */
  emergency_stop = false;
  yellow_delay = 0; // 切换状态
  /* USER CODE END ws2812_yellow_callback */
}

/* breath_delay_Callback function */
void breath_delay_Callback(void *argument)
{
  /* USER CODE BEGIN breath_delay_Callback */
  breathing_flag = 1; // 设置标志位

  /* USER CODE END breath_delay_Callback */
}

/* motor_grab3s_Callback function */
void motor_grab3s_Callback(void *argument)
{
  /* USER CODE BEGIN motor_grab3s_Callback */
  Flag_3s = 1;
  /* USER CODE END motor_grab3s_Callback */
}

/* motor_back_1sCallback function */
void motor_back_1sCallback(void *argument)
{
  /* USER CODE BEGIN motor_back_1sCallback */
  Flag_1s = 1;
  /* USER CODE END motor_back_1sCallback */
}

/* buttton_Callback function */
void buttton_Callback(void *argument)
{
  /* USER CODE BEGIN buttton_Callback */
//  // 再次读取按键状态，确认是否稳定按下
//  if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
//    // 给信号量，通知任务
//    xSemaphoreGive(BUTTON_SEMAPHOREHandle);
//  }
  /* USER CODE END buttton_Callback */
}

/* tempareture_pid_timer function */
void tempareture_pid_timer(void *argument)
{
  /* USER CODE BEGIN tempareture_pid_timer */
    tempareture_pid_time=1;
  /* USER CODE END tempareture_pid_timer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

