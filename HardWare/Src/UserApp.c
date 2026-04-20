
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
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore, spi2RxDmaSemaphoreHandle, spi2TxDmaSemaphoreHandle;  // 按钮信号量、日志输出互斥量、USART2 DMA 传输完成信号量、SPI2 DMA RX/TX 传输完成信号量
SemaphoreHandle_t xI2CMutex;       // I2C 总线互斥锁，用于保护对 I2C 总线的并发访问
SemaphoreHandle_t i2c2_mutex, I2C2_DMA_Sem;
SemaphoreHandle_t xI2CCompleteSem; // I2C 传输完成信号量，用于等待 I2C 操作完成




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
        Kalman_Init(&kf, 0.1f, 1.0f);  // Q: 越小滤波结果越平滑，R: 越小则对测量噪声更敏感
        while (1) {
            // 1. 检查是否收到停止任务的通知
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
                HeatPWM(0); // 立即关闭加热 PWM 输出
                LOG("[Heat] 接收到停止加热通知，准备释放资源并退出...\n");
                // 2. 如果当前任务持有 I2C 互斥锁，则释放它
                if (xSemaphoreGetMutexHolder(i2c2_mutex) == xTaskGetCurrentTaskHandle()) {
                    xSemaphoreGive(i2c2_mutex);
                    LOG("[Heat] 释放 i2c2_mutex\n");
                }

                // 3. 执行结束处理并退出循环
                break;  // 跳出内部 while 循环，进入任务挂起流程
            }
            if (i2c2_error_flag == 0) {
#ifdef DEBUG_LOG
                LOG("Read_tmp112/n");
#endif
                EyeTmp = TmpRaw2Ture();
                //EyeTmp=(EyeTmp>43)?43:EyeTmp;
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
        vTaskSuspend(NULL);  // 挂起当前任务
    }

}

void Press_Task(void *argument) {
    (void)argument;

    for (;;) {
        LOG("Press started\n");
        ADS1220_StartConversion();  // 启动 ADS1220 压力转换
        osDelay(20); // 延时 20ms，等待 ADC 芯片响应
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // 读取初始压力基准值
        while (1) {
            // 1. 检查是否收到停止任务的通知
            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
                //if ((ulTaskNotifyTake(pdTRUE, 0) > 0)||(EYE_status==0)) {
                LOG("[Press_Task] 接收到停止控制通知，准备退出...\n");
                currentState = STATE_OFF;
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }
                // 2. 释放当前任务可能持有的资源
                // 3. 执行结束处理并跳出循环
                break;  // 跳出内部 while 循环，进入任务挂起流程
            }
            PressureControl();
            osDelay(50);
        }
        vTaskSuspend(NULL);  // 挂起当前任务
    }
    /* USER CODE END Press_Task */
}
extern volatile uint8_t button_pressed;
void Button_State_Task(void *argument) {
    (void)argument;
    /* USER CODE BEGIN Button_State_Task */
    /* Infinite loop */
    for (;;) {
        // 等待按钮中断信号
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            osDelay(100); // 延时 100ms，避免按键抖动造成误触发
            // 读取按键状态，如果仍然按下则认为按键有效
            if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) || (soft_button == 1)) {
                // 按键有效后执行相应处理逻辑
                if (EYE_status == 1) {
                    Button_detection();
                    // 等待按键释放
                    while (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
                        osDelay(10);
                    }
                    button_pressed = 0; // 准备处理下一次按键事件
                }
                soft_button = 0;
            }
        }
        // 这里不使用延时，以避免占用过多 CPU 时间
        // vTaskDelay(100);
    }
    /* USER CODE END Button_State_Task */
}

void APP_task(void *argument) {
    (void)argument;
    osDelay(1000);//the breath of frequency
    BQ25895_Init();
    for (;;) {
        //HAL_IWDG_Refresh(&hiwdg);  // 如果启用独立看门狗，则在这里刷新看门狗计数器
        osDelay(100);//the breath of frequency
       bq25895_reinitialize_if_vbus_inserted(); // 如果检测到 VBUS 插入，则重新初始化充电管理器
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
    vTaskDelay(100); // 延时 100ms，避免与 TMC5130 初始化过程冲突，等待电机回零
    for (;;) {
        LOG("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;
        vTaskDelete(NULL);  // 删除并退出当前任务
        //vTaskDelay(1000);
    }
}


// 设备自检任务
void Device_Check_Task(void *argument) {
    (void)argument;
    AD24C01_Factory_formatted();
    EYE_checkout(1.0);
    vTaskDelay(1200);
    xTimerStart(eye_is_existHandle, 0);
    //AT24C02_WriteAllBytes_eye(0xff);// 擦除 EEPROM 存储
    Device_Init();
    for (;;) {
        //HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        //Test_EYE_AT24CXX_ReadWrite_FullCycle();
        DeviceStateMachine_Update();
        //EYE_status=1;
        EYE_checkout(EYE_status);
        osDelay(100);

        //EYE_AT24CXX_WriteUInt16(super_eyes, 0x0202);// 写入眼部数据到 EEPROM
    }
}


extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

void I2C2_RecoveryTask(void *argument) {
    (void)argument;
    for (;;) {
        // 等待通知以执行 I2C 错误恢复
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        i2c2_error_flag = 1;

        uint32_t isr = hi2c2.Instance->ISR;
        LOG("[I2C2 错误] ISR=0x%08lX\n", isr);
        // 处理 I2C 错误或总线异常后，清除相关错误标志
        if (xSemaphoreTake(i2c2_mutex, portMAX_DELAY) == pdTRUE) {
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_BERR);  // 清除总线错误标志
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ARLO);  // 清除仲裁丢失标志
            //
            // // 如果需要，可禁用并重新使能 I2C 时钟以强制复位总线
            // __HAL_RCC_I2C2_CLK_DISABLE();
            // __HAL_RCC_I2C2_CLK_ENABLE();

            // 1. 禁用 I2C 外设
            __HAL_I2C_DISABLE(&hi2c2);

            // 2. 硬件复位 I2C 外设
            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();

            // 重新初始化 I2C2
            HAL_I2C_DeInit(&hi2c2);
            HAL_I2C_Init(&hi2c2);

            // 重新初始化与 I2C2 关联的 DMA
            HAL_DMA_DeInit(&hdma_i2c2_rx);
            HAL_DMA_Init(&hdma_i2c2_rx);

            xSemaphoreGive(i2c2_mutex);  // 释放 I2C 互斥锁
        }

        LOG("[错误恢复] I2C2 已恢复并重新初始化\n");

        // 等待短暂时间，让 I2C 总线恢复稳定
        //osDelay(10);
        i2c2_error_flag = 0;
    }
}


extern
void bq25895_recovery_task(void *argument) {
    (void)argument;
    for (;;) {

        // 等待通知以恢复 bq25895 充电管理器
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//        BQ25895_AutoRecover();
//        osDelay(10);
//        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
}
void PowerOnDelayTask(void *argument) {
    (void)argument;
//    AD24C01_Factory_formatted();// 仅用于未初始化 Flash 设备的工厂格式化
//    // 校准电压显示值
//   // vTaskDelay(pdMS_TO_TICKS(200));
//    // 控制 PA10 引脚
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//    vTaskDelay(pdMS_TO_TICKS(500));
//    prepare_data_set();
//
//    // 删除错误任务
//    vTaskDelete(NULL);
//    LOG("定时任务结束\n");
}


// 电源重启任务
void PowerReboot_Task(void *argument) {
    (void)argument;
//    for (;;) {
//       ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//        LOG("重启\n");
//            vTaskDelay(pdMS_TO_TICKS(10));  // 延时 100ms
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
        const UBaseType_t maxTasks = 20;  // 根据实际任务数量分配状态信息缓存
        TaskStatus_t taskStatusArray[maxTasks];
        UBaseType_t taskCount;
        uint32_t totalRunTime;

        // 获取任务状态信息
        taskCount = uxTaskGetSystemState(taskStatusArray, maxTasks, &totalRunTime);

        //LOG("任务名        句柄       状态 优先级 栈剩余 栈字节\r\n");
        // for (UBaseType_t i = 0; i < taskCount; i++) {
        //     TaskStatus_t *ts = &taskStatusArray[i];
        //     LOG("%-12s %p    %lu    %lu    %lu    %lu\r\n",
        //            ts->pcTaskName,
        //            ts->xHandle,
        //            (unsigned long)ts->eCurrentState,
        //            (unsigned long)ts->uxCurrentPriority,
        //            (unsigned long)ts->usStackHighWaterMark,
        //            (unsigned long)ts->usStackHighWaterMark * sizeof(StackType_t));  // 计算剩余栈字节数
        // }

#endif
        vTaskDelay(1000);
    }
}


#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configASSERT(x) if( (x) == 0 ) vAssertCalled(__FILE__, __LINE__)

/**
 * @brief  FreeRTOS 堆栈溢出时的钩子函数
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 停止当前任务并进入错误处理循环
    (void)xTask;

    LOG("[ERROR] Stack overflow detected! Task=%s\n", pcTaskName);

    // 可以在此处点亮错误指示灯
    //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // 点亮 PC13 LED

    // 选择合适方式停止任务，防止继续运行
    taskDISABLE_INTERRUPTS();
    for(;;) {
        // 进入死循环，等待外部复位或调试干预
    }
}

/**
 * @brief  FreeRTOS 断言失败时的处理函数
 */
void vAssertCalled(const char *file, int line)
{
    LOG("[ASSERT] Failed in file %s, line %d\n", file, line);

    taskDISABLE_INTERRUPTS();
    for(;;) {
        // 保持此处，以便调试断言失败原因
    }
}
void Main(void) {
    //HAL_IWDG_Refresh(&hiwdg);  // 刷新独立看门狗计数器
    logSemaphore = xSemaphoreCreateMutex();  // 创建日志输出互斥量
    i2c2_mutex = xSemaphoreCreateMutex();  // 创建 I2C 互斥量
    I2C2_DMA_Sem = xSemaphoreCreateBinary();
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary(); // 创建按钮事件二值信号量
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();        // USART2 DMA 传输完成信号量
    USART2_DMA_Init();
    SPI2_DMA_Semaphores_Init();
    I2C_Semaphore_Init();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                            ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                             ws2812_yellow_callback);
    breathTimer = xTimerCreate("BreathTimer",
                               pdMS_TO_TICKS(30),   // 每 30ms 触发一次
                               pdTRUE,              // 自动重新加载
                               NULL,
                               BreathingLightCallback);
    //xTimerStart(breathTimer, 20); // ? 未使用的定时器启动参数
    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);
    press_updateHandle = xTimerCreate("press_update", pdMS_TO_TICKS(200), pdTRUE, NULL, press_update_timer);
    eye_is_existHandle = xTimerCreate("eye_is_exist_delay", pdMS_TO_TICKS(1000), pdTRUE, NULL, eye_is_exist_callback);
    xTimerStart(tempareture_pidHandle, 0); // 启动温度 PID 定时器
    xTimerStart(press_updateHandle, 0);
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,  //
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0); // 启动串口超时定时器

    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));
    configASSERT(UART_DMA_IDLE_RECEPT_QUEUEHandle != NULL);

    AT24CXX_Init();
    BQ27441_DEMO();
    BQ27441_VerifyConfig();

    PWM_WS2812B_Init();
    ADS1220_Init(); // 初始化 ADS1220 压力传感器
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