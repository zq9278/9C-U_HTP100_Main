
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
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore, spi2RxDmaSemaphoreHandle, spi2TxDmaSemaphoreHandle;  // SPI2 DMA 完成信号量;  // 定义日志信号量;
SemaphoreHandle_t xI2CMutex;       // I2C总线互斥量
SemaphoreHandle_t i2c2_mutex, I2C2_DMA_Sem;
SemaphoreHandle_t xI2CCompleteSem; // 传输完成信号量




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
        Kalman_Init(&kf, 0.1f, 1.0f);  // Q: 越小越平滑, R: 越小越信任测量
        while (1) {
            // 1. 检查退出通知
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
                HeatPWM(0); // 关闭加热PWM
                LOG("[Heat] 收到退出通知，准备释放资源并退出...\n");
                // 2. 如果持有互斥锁，释放它
                if (xSemaphoreGetMutexHolder(i2c2_mutex) == xTaskGetCurrentTaskHandle()) {
                    xSemaphoreGive(i2c2_mutex);
                    LOG("[Heat] 已释放 i2c2_mutex\n");
                }

                // 3. 执行必要清理后退出
                break;  // 跳出 inner while，回到外层 for 循环重新启动
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
        vTaskSuspend(NULL);  // 自己先挂起
    }

}

void Press_Task(void *argument) {
    (void)argument;

    for (;;) {
        LOG("Press started\n");
        ADS1220_StartConversion();  // 启动转换
        osDelay(20);//防止ADC芯片没反应过来
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // 读取初始压力值
        while (1) {
            // 1. 检查退出通知
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                //if ((ulTaskNotifyTake(pdTRUE, 0) > 0)||(EYE_status==0)) {
                LOG("[Press_Task] 收到退出通知，准备释放资源并退出...\n");
                currentState = STATE_OFF;
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }
                // 2. 如果持有互斥锁，释放它
                // 3. 执行必要清理后退出
                break;  // 跳出 inner while，回到外层 for 循环重新启动
            }
            PressureControl();
            osDelay(50);
        }
        vTaskSuspend(NULL);  // 自己先挂起
    }
    /* USER CODE END Press_Task */
}
extern volatile uint8_t button_pressed;
void Button_State_Task(void *argument) {
    (void)argument;
    /* USER CODE BEGIN Button_State_Task */
    /* Infinite loop */
    for (;;) {
        // 等待信号量触发
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            osDelay(100); // 消抖延时，等待按键状态稳定
            // 检查按键状态是否仍为按下
            if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) || (soft_button == 1)) {
                // 按键稳定，执行按键逻辑
                if (EYE_status == 1) {
                    Button_detection();
                    // 等待释放
                    while (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
                        osDelay(10);
                    }
                    button_pressed = 0; // 准备下一次按键
                }
                soft_button = 0;
            }
        }
        // 延时，避免任务过度占用 CPU
        // vTaskDelay(100);
    }
    /* USER CODE END Button_State_Task */
}

void APP_task(void *argument) {
    (void)argument;
    osDelay(1000);//the breath of frequency
    BQ25895_Init();
    for (;;) {
        //HAL_IWDG_Refresh(&hiwdg);  // 正常运行时喂狗
        osDelay(100);//the breath of frequency
       bq25895_reinitialize_if_vbus_inserted();//充电器插入检测
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
    vTaskDelay(100);//TMC5130_Init();不在同一个线程，需要等待tmc5130复位
    for (;;) {
        LOG("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;
        vTaskDelete(NULL);  // 自己先挂起
        //vTaskDelay(1000);
    }
}


// 检测任务函数
void Device_Check_Task(void *argument) {
    (void)argument;
    AD24C01_Factory_formatted();
    EYE_checkout(1.0);
    vTaskDelay(1200);
    xTimerStart(eye_is_existHandle, 0);
    //AT24C02_WriteAllBytes_eye(0xff);//清理ee存储
    Device_Init();
    for (;;) {
        //HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        //Test_EYE_AT24CXX_ReadWrite_FullCycle();
        DeviceStateMachine_Update();
        //EYE_status=1;
        EYE_checkout(EYE_status);
        osDelay(100);

        //EYE_AT24CXX_WriteUInt16(super_eyes, 0x0202);//标记特殊眼盾
    }
}


extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

void I2C2_RecoveryTask(void *argument) {
    (void)argument;
    for (;;) {
        // 一直等待通知信号（错误发生）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        i2c2_error_flag = 1;

        uint32_t isr = hi2c2.Instance->ISR;
        LOG("[I2C2 错误] ISR=0x%08lX\n", isr);
        // 清除 I2C 错误标志以保持总线正常
        if (xSemaphoreTake(i2c2_mutex, portMAX_DELAY) == pdTRUE) {
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_BERR);  // 清除总线错误标志
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ARLO);  // 清除仲裁丢失标志（如果有）
            //
            // // 重新启用时钟和 I2C 控制器
            // __HAL_RCC_I2C2_CLK_DISABLE();
            // __HAL_RCC_I2C2_CLK_ENABLE();

            // 1. 关闭 I2C 外设
            __HAL_I2C_DISABLE(&hi2c2);

            // 2. 硬件复位 I2C 外设
            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();

            // 重新初始化 I2C2
            HAL_I2C_DeInit(&hi2c2);
            HAL_I2C_Init(&hi2c2);

            // 可选：如果有 DMA，重新初始化 DMA
            HAL_DMA_DeInit(&hdma_i2c2_rx);
            HAL_DMA_Init(&hdma_i2c2_rx);

            xSemaphoreGive(i2c2_mutex);  // 释放互斥锁
        }

        LOG("[恢复任务] I2C2 总线恢复完成！\n");

        // 等待一段时间再重试
        //osDelay(10);
        i2c2_error_flag = 0;
    }
}


extern
void bq25895_recovery_task(void *argument) {
    (void)argument;
    for (;;) {

        // 一直等待通知信号（错误发生）
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//        BQ25895_AutoRecover();
//        osDelay(10);
//        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}
void PowerOnDelayTask(void *argument) {
    (void)argument;
//    AD24C01_Factory_formatted();//如果flash没有初始化，则初始化
//    // 上电后延迟1秒
//   // vTaskDelay(pdMS_TO_TICKS(200));
//    // 拉低PA10
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//
//    // 删除自己
//    vTaskDelete(NULL);
//    LOG("上电完成\n");
}


// 独立任务中处理
void PowerReboot_Task(void *argument) {
    (void)argument;
//    for (;;) {
//       ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//        LOG("重启\n");
//            vTaskDelay(pdMS_TO_TICKS(10));  // 延迟100ms
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
        const UBaseType_t maxTasks = 20;  // 根据实际任务数量修改
        TaskStatus_t taskStatusArray[maxTasks];
        UBaseType_t taskCount;
        uint32_t totalRunTime;

        // 获取所有任务信息
        taskCount = uxTaskGetSystemState(taskStatusArray, maxTasks, &totalRunTime);

        //LOG("任务名        句柄       状态 优先级 栈余量 栈大小\r\n");
        // for (UBaseType_t i = 0; i < taskCount; i++) {
        //     TaskStatus_t *ts = &taskStatusArray[i];
        //     LOG("%-12s %p    %lu    %lu    %lu    %lu\r\n",
        //            ts->pcTaskName,
        //            ts->xHandle,
        //            (unsigned long)ts->eCurrentState,
        //            (unsigned long)ts->uxCurrentPriority,
        //            (unsigned long)ts->usStackHighWaterMark,
        //            (unsigned long)ts->usStackHighWaterMark * sizeof(StackType_t));  // 栈剩余字节数
        // }

#endif
        vTaskDelay(1000);
    }
}


#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configASSERT(x) if( (x) == 0 ) vAssertCalled(__FILE__, __LINE__)

/**
 * @brief  FreeRTOS 检测到任务栈溢出时调用
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 防止优化掉参数
    (void)xTask;

    LOG("[ERROR] Stack overflow detected! Task=%s\n", pcTaskName);

    // 可以加上出错灯指示
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // 假设 PC13 LED

    // 可以选择进入死循环，防止后续内核崩溃
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // 这里可加入小延时闪灯，表示错误类型
    }
}

/**
 * @brief  FreeRTOS 断言失败处理函数
 */
void vAssertCalled(const char *file, int line)
{
    LOG("[ASSERT] Failed in file %s, line %d\n", file, line);

    taskDISABLE_INTERRUPTS();
    for(;;) {
        // 同样可闪灯提示
    }
}
void Main(void) {
    //HAL_IWDG_Refresh(&hiwdg);  // 正常运行时喂狗
    logSemaphore = xSemaphoreCreateMutex();  // 创建LOG互斥信号量
    i2c2_mutex = xSemaphoreCreateMutex();  // 创建LOG互斥信号量
    I2C2_DMA_Sem = xSemaphoreCreateBinary();
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();//按键互斥量
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();        // usart2DMA信号量
    USART2_DMA_Init();
    SPI2_DMA_Semaphores_Init();
    I2C_Semaphore_Init();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                            ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                             ws2812_yellow_callback);
    breathTimer = xTimerCreate("BreathTimer",
                               pdMS_TO_TICKS(30),   // 每 30ms 调一次
                               pdTRUE,              // 自动重装
                               NULL,
                               BreathingLightCallback);
    //xTimerStart(breathTimer, 20); // ? 启动呼吸动画
    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);
    eye_is_existHandle = xTimerCreate("eye_is_exist_delay", pdMS_TO_TICKS(1000), pdTRUE, NULL, eye_is_exist_callback);
    xTimerStart(tempareture_pidHandle, 0); // 启动定时器
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,  //
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0); // 启动定时器

    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));
    configASSERT(UART_DMA_IDLE_RECEPT_QUEUEHandle != NULL);

    AT24CXX_Init();
    BQ27441_DEMO();
    BQ27441_VerifyConfig();

    PWM_WS2812B_Init();
    ADS1220_Init(); // 初始化ADS1220
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
    xTaskCreate(TaskMonitor_Task, "TaskMonitor", 512, NULL, 1, NULL);

    //xTaskCreate(PowerOnDelayTask, "PowerOnDelay", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    //xTaskCreate(PowerReboot_Task, "PowerReboot", 128, NULL,  8, pwrTaskHandle);

}