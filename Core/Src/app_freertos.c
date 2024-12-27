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

/* USER CODE END Variables */
/* Definitions for UART_RECEPT */
osThreadId_t UART_RECEPTHandle;
const osThreadAttr_t UART_RECEPT_attributes = {
  .name = "UART_RECEPT",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for test */
osThreadId_t testHandle;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 256 * 4
};
/* Definitions for UART_DMA_IDLE_RECEPT_QUEUE */
osMessageQueueId_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
const osMessageQueueAttr_t UART_DMA_IDLE_RECEPT_QUEUE_attributes = {
  .name = "UART_DMA_IDLE_RECEPT_QUEUE"
};
/* Definitions for UART_DMA_IDLE_RECEPT_SEMAPHORE */
osSemaphoreId_t UART_DMA_IDLE_RECEPT_SEMAPHOREHandle;
const osSemaphoreAttr_t UART_DMA_IDLE_RECEPT_SEMAPHORE_attributes = {
  .name = "UART_DMA_IDLE_RECEPT_SEMAPHORE"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void UART_RECEPT_Task(void *argument);
void test_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
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
  /* creation of UART_DMA_IDLE_RECEPT_SEMAPHORE */
  UART_DMA_IDLE_RECEPT_SEMAPHOREHandle = osSemaphoreNew(1, 0, &UART_DMA_IDLE_RECEPT_SEMAPHORE_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UART_DMA_IDLE_RECEPT_QUEUE */
  UART_DMA_IDLE_RECEPT_QUEUEHandle = osMessageQueueNew (5, sizeof(uart_data), &UART_DMA_IDLE_RECEPT_QUEUE_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_RECEPT */
  UART_RECEPTHandle = osThreadNew(UART_RECEPT_Task, NULL, &UART_RECEPT_attributes);

  /* creation of test */
  testHandle = osThreadNew(test_task, NULL, &test_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  //extern UART_HandleTypeDef huart1;
  /* Infinite loop */
  for(;;)
  {
     osDelay(100);
     if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &recept_data, portMAX_DELAY) == pdPASS) { // 从队列中接收数据
            // 处理接收到的数据
            //HAL_UART_Transmit(&huart1, (uint8_t*)&recept_data.buffer, recept_data.length, 10);
        
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
void test_task(void *argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  extern UART_HandleTypeDef huart1;
  for(;;)
  {
    osDelay(1000);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    char pcStatsBuffer[512];  // 用于存储任务统计信息
    vTaskGetRunTimeStats(pcStatsBuffer);  // 获取任务统计信息
    HAL_UART_Transmit(&huart1, (uint8_t *)pcStatsBuffer, strlen(pcStatsBuffer), HAL_MAX_DELAY);
  }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

