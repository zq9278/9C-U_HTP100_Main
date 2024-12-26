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
/* Definitions for UART_DMA_IDLE_R */
osThreadId_t UART_DMA_IDLE_RHandle;
const osThreadAttr_t UART_DMA_IDLE_R_attributes = {
  .name = "UART_DMA_IDLE_R",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 64 * 4
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

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  UART_DMA_IDLE_RECEPT_QUEUEHandle = osMessageQueueNew (10, sizeof(uart_data), &UART_DMA_IDLE_RECEPT_QUEUE_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UART_DMA_IDLE_R */
  UART_DMA_IDLE_RHandle = osThreadNew(StartDefaultTask, NULL, &UART_DMA_IDLE_R_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the UART_DMA_IDLE_R thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

