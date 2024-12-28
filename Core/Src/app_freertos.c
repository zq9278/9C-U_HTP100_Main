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
#include <stdint.h>


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
 uint32_t a = 0;//test_variable
/* USER CODE END Variables */
/* Definitions for UART_RECEPT */
osThreadId_t UART_RECEPTHandle;
const osThreadAttr_t UART_RECEPT_attributes = {.name = "UART_RECEPT",
                                               .priority =
                                                   (osPriority_t)osPriorityLow,
                                               .stack_size = 256 * 4};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {.name = "test",
                                        .priority =
                                            (osPriority_t)osPriorityRealtime7,
                                        .stack_size = 256 * 4};
/* Definitions for Heat */
osThreadId_t HeatHandle;
const osThreadAttr_t Heat_attributes = {.name = "Heat",
                                        .priority = (osPriority_t)osPriorityLow,
                                        .stack_size = 256 * 4};
/* Definitions for Press */
osThreadId_t PressHandle;
const osThreadAttr_t Press_attributes = {.name = "Press",
                                         .priority =
                                             (osPriority_t)osPriorityLow,
                                         .stack_size = 256 * 4};
/* Definitions for Button_State */
osThreadId_t Button_StateHandle;
const osThreadAttr_t Button_State_attributes = {.name = "Button_State",
                                                .priority =
                                                    (osPriority_t)osPriorityLow,
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
void test_task(void *argument);
void Heat_Task(void *argument);
void Press_Task(void *argument);
void Button_State_Task(void *argument);

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

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_DMA_IDLE_RECEPT_QUEUE */
  UART_DMA_IDLE_RECEPT_QUEUEHandle = osMessageQueueNew(
      5, sizeof(uart_data), &UART_DMA_IDLE_RECEPT_QUEUE_attributes);

  /* creation of HEAT_DATA */
  HEAT_DATAHandle =
      osMessageQueueNew(16, sizeof(uint16_t), &HEAT_DATA_attributes);

  /* creation of PRESS_DATA */
  PRESS_DATAHandle =
      osMessageQueueNew(16, sizeof(uint16_t), &PRESS_DATA_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_RECEPT */
  UART_RECEPTHandle =
      osThreadNew(UART_RECEPT_Task, NULL, &UART_RECEPT_attributes);

  /* creation of test */
  testHandle = osThreadNew(test_task, NULL, &test_attributes);

  /* creation of Heat */
  HeatHandle = osThreadNew(Heat_Task, NULL, &Heat_attributes);

  /* creation of Press */
  PressHandle = osThreadNew(Press_Task, NULL, &Press_attributes);

  /* creation of Button_State */
  Button_StateHandle =
      osThreadNew(Button_State_Task, NULL, &Button_State_attributes);

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
  // extern UART_HandleTypeDef huart1;
  /* Infinite loop */
  for (;;) {
    osDelay(10);
    if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &recept_data,
                      portMAX_DELAY) == pdPASS) { // 从队列中接收数据
      command_parsing((uint8_t *)&recept_data);// 处理接收到的数据
      // HAL_UART_Transmit(&huart1, (uint8_t*)&recept_data.buffer, recept_data.length, 10);
    }
  }
  /* USER CODE END UART_RECEPT_Task */
}

/* USER CODE BEGIN Header_test_task */
/**
 * @brief Function implementing the test thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_test_task */
void test_task(void *argument) {
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  extern UART_HandleTypeDef huart1;
  for (;;) {
    osDelay(1000);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    char pcStatsBuffer[512];             // 用于存储任务统计信息
    vTaskGetRunTimeStats(pcStatsBuffer); // 获取任务统计信息
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStatsBuffer, strlen(pcStatsBuffer),
                      HAL_MAX_DELAY);
  }
  /* USER CODE END test_task */
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
  for (;;) {
    osDelay(1);
  }
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
  for (;;) {
    osDelay(1);
    
  }
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
