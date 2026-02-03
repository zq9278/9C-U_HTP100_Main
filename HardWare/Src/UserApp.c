
#include "UserApp.h"
#include "interface_uart.h"
#include "24cxx.h"
#include "tmp112.h"
#include "Button.h"
#include "heat.h"
#include "ads1220.h"
#include "tmc5130.h"
#include "device_lifetime.h"
#include "bq25895.h"
#include "bq27441.h"
#include "time_callback.h"
#include "ws2812b.h"
#include "app_sys.h"
extern uint8_t white_delay, yellow_delay, breathing_flag;
uint8_t Flag_400ms = 1;
float weight0 = 0;
volatile int Flag_3s, Flag_1s, eye_workingtime_1s, eye_existtime_1s;
extern ChargeState_t ChargeState;
uint8_t soft_button;
extern uint8_t charging, working, low_battery, fully_charged, emergency_stop;
extern PID_TypeDef HeatPID;
float Heat_PWM, EyeTmp;
uint8_t flag_200ms;
uint8_t tempature_flag_400ms, press_flag_400ms, battery_flag_400ms, is_charging_flag;
uart_data *frameData_uart;
uint8_t i2c2_error_flag = 0;
//#define DEBUG_LOG
/* FreeRTOS Handles */
TaskHandle_t UART_RECEPTHandle, HeatHandle, PressHandle, Button_StateHandle, APPHandle, motor_homeHandle, deviceCheckHandle, i2c2_recovery_task_handle, pwrTaskHandle,bq25895_recovery_homeHandle;
QueueHandle_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore, spi2RxDmaSemaphoreHandle, spi2TxDmaSemaphoreHandle;  // SPI2 DMA ����ź���;  // ������־�ź���;
SemaphoreHandle_t xI2CMutex;       // I2C���߻�����
SemaphoreHandle_t i2c2_mutex, I2C2_DMA_Sem;
SemaphoreHandle_t xI2CCompleteSem; // ��������ź���




void UART_RECEPT_Task(void *argument) {
    (void)argument;
    for (;;) {
        if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &frameData_uart, portMAX_DELAY) == pdTRUE) {
            Serial_data_stream_parsing(frameData_uart);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

uint32_t notify = 0;

void Heat_Task(void *argument) {
    (void)argument;

    for (;;) {
        LOG("heat_start");
        Kalman_Init(&kf, 0.1f, 1.0f);  // Q: ԽСԽƽ��, R: ԽСԽ���β���
        while (1) {
            // 1. ����˳�֪ͨ
            //if ((ulTaskNotifyTake(pdTRUE, 0) > 0)||(EYE_status==0)) {
            //if (ulTaskNotifyTake(pdTRUE, 0) > 0) {

            notify = ulTaskNotifyTake(pdTRUE, 0);
            //LOG("ulTaskNotifyTake return: %lu\n", notify);
            if (notify > 0) {
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }
                currentState = STATE_OFF;
                HeatPWM(0); // �رռ���PWM
                LOG("[Heat] �յ��˳�֪ͨ��׼���ͷ���Դ���˳�...\n");
                // 2. ������л��������ͷ���
                if (xSemaphoreGetMutexHolder(i2c2_mutex) == xTaskGetCurrentTaskHandle()) {
                    xSemaphoreGive(i2c2_mutex);
                    LOG("[Heat] ���ͷ� i2c2_mutex\n");
                }

                // 3. ִ�б�Ҫ�������˳�
                break;  // ���� inner while���ص���� for ѭ����������
            }
            if (i2c2_error_flag == 0) {
#ifdef DEBUG_LOG
                LOG("Read_tmp112/n");
#endif
                EyeTmp = TmpRaw2Ture();
                EyeTmp=(EyeTmp>43)?43:EyeTmp;
                if (tempature_flag_400ms) {
                    tempature_flag_400ms = 0;
                    if (EyeTmp != 0.0f) {
                        ScreenUpdateTemperature(EyeTmp - temperature_compensation);
                    }
                }
//        HeatPID.integral_max = 40;
//        if ((EyeTmp>=38)&&(EyeTmp<=41.5)){
//            HeatPID.Ki=1;
//            HeatPID.integral_max = 100;
//        }
                Heat_PWM = PID_Compute(&HeatPID, EyeTmp);
                HeatPWMSet((uint8_t) Heat_PWM);
            }

            vTaskDelay(pdMS_TO_TICKS(150));
            //HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

        }
        vTaskSuspend(NULL);  // �Լ��ȹ���
    }

}

void Press_Task(void *argument) {
    (void)argument;

    for (;;) {
        LOG("Press started\n");
        ADS1220_StartConversion();  // ����ת��
        osDelay(20);//��ֹADCоƬû��Ӧ����
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // ��ȡ��ʼѹ��ֵ
        while (1) {
            // 1. ����˳�֪ͨ
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                //if ((ulTaskNotifyTake(pdTRUE, 0) > 0)||(EYE_status==0)) {
                LOG("[Press_Task] �յ��˳�֪ͨ��׼���ͷ���Դ���˳�...\n");
                currentState = STATE_OFF;
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }
                // 2. ������л��������ͷ���
                // 3. ִ�б�Ҫ�������˳�
                break;  // ���� inner while���ص���� for ѭ����������
            }
            PressureControl();
            osDelay(50);
        }
        vTaskSuspend(NULL);  // �Լ��ȹ���
    }
    /* USER CODE END Press_Task */
}
extern volatile uint8_t button_pressed;
void Button_State_Task(void *argument) {
    (void)argument;
    /* USER CODE BEGIN Button_State_Task */
    /* Infinite loop */
    for (;;) {
        // �ȴ��ź�������
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            osDelay(100); // ������ʱ���ȴ�����״̬�ȶ�
            // ��鰴��״̬�Ƿ���Ϊ����
            if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) || (soft_button == 1)) {
                // �����ȶ���ִ�а����߼�
                if (EYE_status == 1) {
                    Button_detection();
                    // �ȴ��ͷ�
                    while (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
                        osDelay(10);
                    }
                    button_pressed = 0; // ׼����һ�ΰ���
                }
                soft_button = 0;
            }
        }
        // ��ʱ�������������ռ�� CPU
        // vTaskDelay(100);
    }
    /* USER CODE END Button_State_Task */
}

void APP_task(void *argument) {
    (void)argument;
    osDelay(1000);//the breath of frequency
    BQ25895_Init();
    for (;;) {
        //HAL_IWDG_Refresh(&hiwdg);  // ��������ʱι��
        osDelay(100);//the breath of frequency
       bq25895_reinitialize_if_vbus_inserted();//�����������
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
        BQ27441_PrintRaTable();
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();

    }
}

void Motor_go_home_task(void *argument) {
    (void)argument;
    vTaskDelay(100);//TMC5130_Init();����ͬһ���̣߳���Ҫ�ȴ�tmc5130��λ
    for (;;) {
        LOG("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;
        vTaskDelete(NULL);  // �Լ��ȹ���
        //vTaskDelay(1000);
    }
}


// ���������
void Device_Check_Task(void *argument) {
    (void)argument;
    AD24C01_Factory_formatted();
    EYE_checkout(1.0);
    vTaskDelay(1200);
    xTimerStart(eye_is_existHandle, 0);
    //AT24C02_WriteAllBytes_eye(0xff);//����ee�洢
    Device_Init();
    for (;;) {
        //HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        //Test_EYE_AT24CXX_ReadWrite_FullCycle();
        DeviceStateMachine_Update();
        //EYE_status=1;
        EYE_checkout(EYE_status);
        osDelay(100);

        //EYE_AT24CXX_WriteUInt16(super_eyes, 0x0202);//��������۶�
    }
}


extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

void I2C2_RecoveryTask(void *argument) {
    (void)argument;
    for (;;) {
        // һֱ�ȴ�֪ͨ�źţ���������
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        i2c2_error_flag = 1;

        uint32_t isr = hi2c2.Instance->ISR;
        LOG("[I2C2 ����] ISR=0x%08lX\n", isr);
        // ��� I2C �����־�Ա�����������
        if (xSemaphoreTake(i2c2_mutex, portMAX_DELAY) == pdTRUE) {
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_BERR);  // ������ߴ����־
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ARLO);  // ����ٲö�ʧ��־������У�
            //
            // // ��������ʱ�Ӻ� I2C ������
            // __HAL_RCC_I2C2_CLK_DISABLE();
            // __HAL_RCC_I2C2_CLK_ENABLE();

            // 1. �ر� I2C ����
            __HAL_I2C_DISABLE(&hi2c2);

            // 2. Ӳ����λ I2C ����
            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();

            // ���³�ʼ�� I2C2
            HAL_I2C_DeInit(&hi2c2);
            HAL_I2C_Init(&hi2c2);

            // ��ѡ������� DMA�����³�ʼ�� DMA
            HAL_DMA_DeInit(&hdma_i2c2_rx);
            HAL_DMA_Init(&hdma_i2c2_rx);

            xSemaphoreGive(i2c2_mutex);  // �ͷŻ�����
        }

        LOG("[�ָ�����] I2C2 ���߻ָ���ɣ�\n");

        // �ȴ�һ��ʱ��������
        //osDelay(10);
        i2c2_error_flag = 0;
    }
}


extern
void bq25895_recovery_task(void *argument) {
    (void)argument;
    for (;;) {

        // һֱ�ȴ�֪ͨ�źţ���������
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//        BQ25895_AutoRecover();
//        osDelay(10);
//        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}
void PowerOnDelayTask(void *argument) {
    (void)argument;
//    AD24C01_Factory_formatted();//���flashû�г�ʼ�������ʼ��
//    // �ϵ���ӳ�1��
//   // vTaskDelay(pdMS_TO_TICKS(200));
//    // ����PA10
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//
//    // ɾ���Լ�
//    vTaskDelete(NULL);
//    LOG("�ϵ����\n");
}


// ���������д���
void PowerReboot_Task(void *argument) {
    (void)argument;
//    for (;;) {
//       ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//        LOG("����\n");
//            vTaskDelay(pdMS_TO_TICKS(10));  // �ӳ�100ms
//            if(HAL_GPIO_ReadPin(PWR_SENSE_GPIO_Port, PWR_SENSE_Pin) == 0){
//               // reset = 1;
////                hiwdg.Init.Reload = 1;
////                HAL_IWDG_Init(&hiwdg);
//            }
//
//        }
}
#define LOG_TASK_INOF_DEBUG
void TaskMonitor_Task(void *argument)
{
    (void)argument;
    for (;;)
    {

#ifdef LOG_TASK_INOF_DEBUG
        const UBaseType_t maxTasks = 20;  // ����ʵ�����������޸�
        TaskStatus_t taskStatusArray[maxTasks];
        UBaseType_t taskCount;
        uint32_t totalRunTime;

        // ��ȡ����������Ϣ
        taskCount = uxTaskGetSystemState(taskStatusArray, maxTasks, &totalRunTime);

        //LOG("������        ���       ״̬ ���ȼ� ջ���� ջ��С\r\n");
        // for (UBaseType_t i = 0; i < taskCount; i++) {
        //     TaskStatus_t *ts = &taskStatusArray[i];
        //     LOG("%-12s %p    %lu    %lu    %lu    %lu\r\n",
        //            ts->pcTaskName,
        //            ts->xHandle,
        //            (unsigned long)ts->eCurrentState,
        //            (unsigned long)ts->uxCurrentPriority,
        //            (unsigned long)ts->usStackHighWaterMark,
        //            (unsigned long)ts->usStackHighWaterMark * sizeof(StackType_t));  // ջʣ���ֽ���
        // }

#endif
        vTaskDelay(1000);
    }
}


#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configASSERT(x) if( (x) == 0 ) vAssertCalled(__FILE__, __LINE__)

/**
 * @brief  FreeRTOS ��⵽����ջ���ʱ����
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // ��ֹ�Ż�������
    (void)xTask;

    LOG("[ERROR] Stack overflow detected! Task=%s\n", pcTaskName);

    // ���Լ��ϳ�����ָʾ
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // ���� PC13 LED

    // ����ѡ�������ѭ������ֹ�����ں˱���
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // ����ɼ���С��ʱ���ƣ���ʾ��������
    }
}

/**
 * @brief  FreeRTOS ����ʧ�ܴ�������
 */
void vAssertCalled(const char *file, int line)
{
    LOG("[ASSERT] Failed in file %s, line %d\n", file, line);

    taskDISABLE_INTERRUPTS();
    for(;;) {
        // ͬ����������ʾ
    }
}
void Main(void) {
    //HAL_IWDG_Refresh(&hiwdg);  // ��������ʱι��
    logSemaphore = xSemaphoreCreateMutex();  // ����LOG�����ź���
    i2c2_mutex = xSemaphoreCreateMutex();  // ����LOG�����ź���
    I2C2_DMA_Sem = xSemaphoreCreateBinary();
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();//����������
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();        // usart2DMA�ź���
    USART2_DMA_Init();
    SPI2_DMA_Semaphores_Init();
    I2C_Semaphore_Init();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                            ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                             ws2812_yellow_callback);
    breathTimer = xTimerCreate("BreathTimer",
                               pdMS_TO_TICKS(30),   // ÿ 30ms ��һ��
                               pdTRUE,              // �Զ���װ
                               NULL,
                               BreathingLightCallback);
    //xTimerStart(breathTimer, 20); // ? ������������
    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);
    eye_is_existHandle = xTimerCreate("eye_is_exist_delay", pdMS_TO_TICKS(1000), pdTRUE, NULL, eye_is_exist_callback);
    xTimerStart(tempareture_pidHandle, 0); // ������ʱ��
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,  //
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0); // ������ʱ��

    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));
    configASSERT(UART_DMA_IDLE_RECEPT_QUEUEHandle != NULL);

    AT24CXX_Init();
    BQ27441_DEMO();
    BQ27441_VerifyConfig();

    PWM_WS2812B_Init();
    ADS1220_Init(); // ��ʼ��ADS1220
    TMC5130_Init();
    HeatInit();
    //TMP112_Init();


    xTaskCreate(UART_RECEPT_Task, "UART_RECEPT", 350, NULL, 10, &UART_RECEPTHandle);
    xTaskCreate(Button_State_Task, "Button_State", 256, NULL, 9, &Button_StateHandle);
    xTaskCreate(APP_task, "APP", 512, NULL, 6, &APPHandle);
    xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle);
    xTaskCreate(bq25895_recovery_task, "bq25895_recovery", 128, NULL, 2, &bq25895_recovery_homeHandle);
     if (xTaskCreate(Device_Check_Task, "Device_Check", 256, NULL, 7, &deviceCheckHandle) == pdPASS) {
         vTaskSuspend(deviceCheckHandle);
     };
     if (xTaskCreate(Press_Task, "Press", 256, NULL, 3, &PressHandle) == pdPASS) { vTaskSuspend(PressHandle); };
     if (xTaskCreate(Heat_Task, "Heat", 256, NULL, 4, &HeatHandle) == pdPASS) { vTaskSuspend(HeatHandle); };
    xTaskCreate(I2C2_RecoveryTask, "I2C2Recover", 256, NULL, 11, &i2c2_recovery_task_handle);
    //xTaskCreate(TaskMonitor_Task, "TaskMonitor", 512, NULL, 1, NULL);

    //xTaskCreate(PowerOnDelayTask, "PowerOnDelay", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    //xTaskCreate(PowerReboot_Task, "PowerReboot", 128, NULL,  8, pwrTaskHandle);

}