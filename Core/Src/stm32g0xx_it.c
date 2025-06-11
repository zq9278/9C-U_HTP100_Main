/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g0xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HardFault_Handler_C(uint32_t *stacked_args) {
    uint32_t r0  = stacked_args[0];
    uint32_t r1  = stacked_args[1];
    uint32_t r2  = stacked_args[2];
    uint32_t r3  = stacked_args[3];
    uint32_t r12 = stacked_args[4];
    uint32_t lr  = stacked_args[5];
    uint32_t pc  = stacked_args[6];
    uint32_t psr = stacked_args[7];

    LOG("? [HardFault] ϵͳ�������쳣�Ĵ������£�\r\n");
    LOG(" R0  = 0x%08lX\r\n", r0);
    LOG(" R1  = 0x%08lX\r\n", r1);
    LOG(" R2  = 0x%08lX\r\n", r2);
    LOG(" R3  = 0x%08lX\r\n", r3);
    LOG(" R12 = 0x%08lX\r\n", r12);
    LOG(" LR  = 0x%08lX\r\n", lr);
    LOG(" PC  = 0x%08lX\r\n", pc);
    LOG(" PSR = 0x%08lX\r\n", psr);

    // ���������ͣ�����������Լ��϶ϵ�ָ��
    __asm volatile("BKPT #0");

    // ͣסϵͳ���ȴ�����
    while (1);
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_tim16_ch1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
    __asm volatile
            (
            "movs r0, #4            \n"
            "mov r1, lr             \n"
            "tst r0, r1             \n"
            "beq _msp_used          \n"
            "mrs r0, psp            \n"
            "b HardFault_Handler_C  \n"
            "_msp_used:                 \n"
            "mrs r0, msp            \n"
            "b HardFault_Handler_C  \n"
            );



  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 2 and line 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(SW_CNT_Pin);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PWR_SENSE_Pin);
  HAL_GPIO_EXTI_IRQHandler(CHG_INT_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 4, channel 5, channel 6, channel 7 and DMAMUX1 interrupts.
  */
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  HAL_DMA_IRQHandler(&hdma_tim16_ch1);
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */


  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR))
  {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  }
  else
  {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/**
  * @brief This function handles I2C2 global interrupt.
  */
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR))
  {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  }
  else
  {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles SPI2 global interrupt.
  */
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//extern osSemaphoreId_t BUTTON_SEMAPHOREHandle; // �����ź������
 void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
 {
    if (GPIO_Pin == SW_CNT_Pin) {  // ����Ƿ���Ŀ�갴��
         BaseType_t xHigherPriorityTaskWoken = pdFALSE;
         // ���ж����ͷ��ź�����֪ͨ���񰴼�����
         xSemaphoreGiveFromISR(BUTTON_SEMAPHOREHandle,&xHigherPriorityTaskWoken);
         // �����Ҫ�л������ģ�����ô˺���
         portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//�Ƿ������ж�isr����ִ�и������ȼ�������
     }
 }

//extern osTimerId_t butttonHandle;
//void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
//  if (GPIO_Pin == SW_CNT_Pin) {
//    // ����ֹͣ��ʱ������������У�
//    //osTimerStop(butttonHandle);
//    // ������ʱ��
//    osTimerStart(butttonHandle, 5);
//    xTimerStartFromISR
//  }
//}

volatile uint32_t last_button_press_time_PWR = 0; // ��¼�ϴΰ������µ�ʱ���
extern uint8_t reset;
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

  if (GPIO_Pin == PWR_SENSE_Pin) {
      NVIC_SystemReset();
//    reset = 1;
//      hiwdg.Init.Reload = 1;
//      HAL_IWDG_Init(&hiwdg);

//      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//      vTaskNotifyGiveFromISR(pwrTaskHandle, &xHigherPriorityTaskWoken);
//      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
extern DMA_HandleTypeDef hdma_tim16_ch1;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM16) {
    hdma_tim16_ch1.State = HAL_DMA_STATE_READY;
  }
}


//extern uint8_t usart1_tx;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(usart2_dmatxSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }


}
extern volatile uint8_t i2c_dma_read_complete;  // ����ɱ�־
extern volatile uint8_t i2c_dma_write_complete; // д��ɱ�־

//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c1.Instance) {
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        xSemaphoreGiveFromISR(xI2CCompleteSem, &xHigherPriorityTaskWoken);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//    if (hi2c->Instance == hi2c2.Instance) {
//        i2c_dma_read_complete = 1;  // ��� DMA �������
//    }
//}
//
//void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c1.Instance) {
//        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//        xSemaphoreGiveFromISR(xI2CCompleteSem, &xHigherPriorityTaskWoken);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//    }
//    if (hi2c->Instance == hi2c2.Instance) {
//        i2c_dma_write_complete = 1;  // ��� DMA �������
//    }
//
//}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == hi2c1.Instance) {
        xSemaphoreGiveFromISR(xI2CCompleteSem, &xHigherPriorityTaskWoken);
    } else if (hi2c->Instance == hi2c2.Instance) {
        xSemaphoreGiveFromISR(I2C2_DMA_Sem, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (hi2c->Instance == hi2c1.Instance) {
        xSemaphoreGiveFromISR(xI2CCompleteSem, &xHigherPriorityTaskWoken);
    } else if (hi2c->Instance == hi2c2.Instance) {
        xSemaphoreGiveFromISR(I2C2_DMA_Sem, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c1.Instance) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xI2CCompleteSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        __HAL_RCC_I2C1_CLK_DISABLE();// �ر� I2C ʱ��
        //osDelay(10);// ������ʱ
        __HAL_RCC_I2C1_CLK_ENABLE(); // ����ʹ�� I2C ʱ��
        HAL_I2C_DeInit(&hi2c1);// ���³�ʼ�� I2C ����
        HAL_I2C_Init(&hi2c1);
    }
    if (hi2c->Instance == hi2c2.Instance) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (i2c2_recovery_task_handle != NULL) {
            vTaskNotifyGiveFromISR(i2c2_recovery_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        } else {
            LOG("DEVICE_STATE_EXPIRED: IIC恢复任务-句柄未初始化！`\n");

        }
    }

}


// ============================
// DMA ��ɻص�����
// ============================
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spi2TxDmaSemaphoreHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
extern volatile uint8_t SPI_RxComplete;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        SPI_RxComplete = 1; // ���ý�����ɱ�־
    }
    if (hspi->Instance == SPI2) {
        hspi2.State = HAL_SPI_STATE_READY;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spi2RxDmaSemaphoreHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END 1 */
