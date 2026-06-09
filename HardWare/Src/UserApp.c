/*
 * 鏂囦欢: UserApp.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
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
extern float p, i, d;
float Heat_PWM, EyeTmp;
uint8_t flag_200ms;
uint8_t tempature_flag_400ms, press_flag_400ms, battery_flag_400ms, is_charging_flag;
uart_data *frameData_uart;
uint8_t i2c2_error_flag = 0;

#define ENABLE_HEAT_ADAPTIVE_LOAD 0
#define ENABLE_HEAT_SEGMENTED_PID 1
#define ENABLE_HEAT_STARTUP_SOFT_LANDING 0

TaskHandle_t UART_RECEPTHandle, HeatHandle, PressHandle, Button_StateHandle, APPHandle, motor_homeHandle, deviceCheckHandle, i2c2_recovery_task_handle, pwrTaskHandle,bq25895_recovery_homeHandle;
QueueHandle_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore, spi2RxDmaSemaphoreHandle, spi2TxDmaSemaphoreHandle;
SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t i2c2_mutex, I2C2_DMA_Sem;
SemaphoreHandle_t xI2CCompleteSem;




/**
 * @brief UART_RECEPT_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void UART_RECEPT_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    for (;;) {
        if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &frameData_uart, portMAX_DELAY) == pdTRUE) {
            Serial_data_stream_parsing(frameData_uart);
        }
    }
}

uint32_t notify = 0;

/**
 * @brief Heat_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Heat_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;

    for (;;) {
        TickType_t heat_last_wake_tick;
        TempDisplayTargetFilterReset();
#if ENABLE_HEAT_ADAPTIVE_LOAD
        HeatAdaptiveReset();
#endif
#if ENABLE_HEAT_SEGMENTED_PID
        HeatSegmentedPIDReset();
#endif
#if ENABLE_HEAT_STARTUP_SOFT_LANDING
        HeatStartupSoftLandingReset();
#endif
        heat_last_wake_tick = xTaskGetTickCount();
        while (1) {




            notify = ulTaskNotifyTake(pdTRUE, 0);

            if (notify > 0) {
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }
                currentState = STATE_OFF;
                HeatPWM(0);
                LOGW("[Task] Event\n");

                if (xSemaphoreGetMutexHolder(i2c2_mutex) == xTaskGetCurrentTaskHandle()) {
                    xSemaphoreGive(i2c2_mutex);
                }


                break;
            }
            if (i2c2_error_flag == 0) {
#ifdef DEBUG_LOG
                LOGI("[Task] Event\n");
#endif
                EyeTmp = TmpRaw2Ture();
                EyeTmp = TempDisplayTargetFilterUpdate(
                    EyeTmp,
                    HeatPID.setpoint - TEMPERATURE_CONTROL_COMPENSATION
                );
               float Eye_Tmp = HeatLimitMeasuredTemperature(EyeTmp - TEMPERATURE_DISPLAY_COMPENSATION);
                uint8_t send_heat_power = tempature_flag_400ms;
                if (tempature_flag_400ms) {
                    tempature_flag_400ms = 0;
                    if (Eye_Tmp != 0.0f) {
                        ScreenUpdateTemperature(Eye_Tmp);
                        //ScreenUpdateTemperature(EyeTmp);
                    }
                }





#if ENABLE_HEAT_ADAPTIVE_LOAD
                Heat_PWM = HeatAdaptivePIDComputeDt(&HeatPID, EyeTmp, (float)HEAT_PID_PERIOD_MS / 1000.0f);
#elif ENABLE_HEAT_SEGMENTED_PID
                Heat_PWM = HeatSegmentedPIDComputeDt(&HeatPID, EyeTmp, (float)HEAT_PID_PERIOD_MS / 1000.0f);
#else
                Heat_PWM = PID_Compute_dt(&HeatPID, EyeTmp, (float)HEAT_PID_PERIOD_MS / 1000.0f);
#endif
#if ENABLE_HEAT_STARTUP_SOFT_LANDING
                Heat_PWM = HeatStartupSoftLandingApply(&HeatPID, EyeTmp, Heat_PWM);
#endif
                HeatPWMSet((uint8_t) Heat_PWM);
                if (send_heat_power) {
                    ScreenUpdateHeatPower((Heat_PWM / 254.0f) * 100.0f);
#if ENABLE_HEAT_ADAPTIVE_LOAD
                    ScreenUpdateHeatLoadStatus((float)HeatAdaptiveGetStatus());
#else
                    ScreenUpdateHeatLoadStatus(0.0f);
#endif
                    
                }
            }

            vTaskDelayUntil(&heat_last_wake_tick, pdMS_TO_TICKS(HEAT_PID_PERIOD_MS));


        }
        vTaskSuspend(NULL);
    }

}

/**
 * @brief Press_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Press_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;

    for (;;) {
        ADS1220_StartConversion();
        osDelay(20);
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();
        PressureDisplayTargetFilterReset();
        PressureControlReset();
        press_pid_tick_flag = 0;
        while (1) {

            if (ulTaskNotifyTake(pdTRUE, 0) > 0) {

                LOGW("[Task] Press_Task Event\n");
                currentState = STATE_OFF;
                if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
                    ScreenWorkModeQuit();
                    ScreenTimerStop();
                }


                break;
            }
            if (press_pid_tick_flag) {
                press_pid_tick_flag = 0;
                PressureControl((float)PRESS_PID_TIMER_PERIOD_MS / 1000.0f);
            }
            osDelay(5);
        }
        vTaskSuspend(NULL);
    }

}
extern volatile uint8_t button_pressed;
/**
 * @brief Button_State_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Button_State_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;


    for (;;) {
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            if (soft_button == 1) {
                soft_button = 0;
                if (EYE_status == 1) {
                    Button_detection();
                }
                button_pressed = 0;
                continue;
            }

            osDelay(50);
            if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
                if (EYE_status == 1) {
                    Button_detection();
                }
                while (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
                    osDelay(10);
                }
            }
            button_pressed = 0;
        }

    }

}
/**
 * @brief APP_task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void APP_task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    //osDelay(500);

    for (;;) {

        osDelay(100);
       bq25895_reinitialize_if_vbus_inserted();
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
#ifdef BQ27441_RA_TABLE_LOG
        static uint32_t lastRaLogTick = 0;
        uint32_t nowTick = HAL_GetTick();

        if ((nowTick - lastRaLogTick) >= 5000U) {
            lastRaLogTick = nowTick;
            BQ27441_PrintRaTable();
        }
#endif
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();

    }
}
/**
 * @brief Motor_go_home_task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Motor_go_home_task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    vTaskDelay(100);
    for (;;) {
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;
        vTaskDelete(NULL);
    }
}

/**
 * @brief Device_Check_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void Device_Check_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    AD24C01_Factory_formatted();
    EYE_checkout(1.0);
    //vTaskDelay(5000);
    xTimerStart(eye_is_existHandle, 0);
    Device_Init();
    for (;;) {
        DeviceStateMachine_Update();
        Device_HandlePendingMarkRequest();
        EYE_checkout(EYE_status);
        osDelay(100);
    }
}
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;

/**
 * @brief I2C2_RecoveryTask 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void I2C2_RecoveryTask(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        i2c2_error_flag = 1;
        uint32_t isr = hi2c2.Instance->ISR;
        LOGE("[Task] I2C2 error ISR=0x%08lX\n", isr);
        if (xSemaphoreTake(i2c2_mutex, portMAX_DELAY) == pdTRUE) {
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_BERR);
            __HAL_I2C_CLEAR_FLAG(&hi2c2, I2C_FLAG_ARLO);
            __HAL_I2C_DISABLE(&hi2c2);
            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();
            HAL_I2C_DeInit(&hi2c2);
            HAL_I2C_Init(&hi2c2);
            HAL_DMA_DeInit(&hdma_i2c2_rx);
            HAL_DMA_Init(&hdma_i2c2_rx);
            xSemaphoreGive(i2c2_mutex);
        }

        LOGI("[Task] Event");



        i2c2_error_flag = 0;
    }
}


extern
/**
 * @brief bq25895_recovery_task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void bq25895_recovery_task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;
    for (;;) {


        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);





    }
}
/**
 * @brief PowerOnDelayTask 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void PowerOnDelayTask(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;




}



/**
 * @brief PowerReboot_Task 鍑芥暟瀹炵幇銆? * @param argument 鍙傛暟銆? */
void PowerReboot_Task(void *argument) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    (void)argument;






}
#define LOG_TASK_INOF_DEBUG
void TaskMonitor_Task(void *argument)
{
    (void)argument;
    for (;;)
    {

#ifdef LOG_TASK_INOF_DEBUG
        const UBaseType_t maxTasks = 20;
        TaskStatus_t taskStatusArray[maxTasks];
        UBaseType_t taskCount;
        uint32_t totalRunTime;


        taskCount = uxTaskGetSystemState(taskStatusArray, maxTasks, &totalRunTime);



#endif
        vTaskDelay(1000);
    }
}


#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK    1
#define configASSERT(x) if( (x) == 0 ) vAssertCalled(__FILE__, __LINE__)


void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{

    (void)xTask;

    LOGI("[Task] Stack overflow task=%s\n", pcTaskName);


    taskDISABLE_INTERRUPTS();
    for(;;) {

    }
}


void vAssertCalled(const char *file, int line)
{
    LOGE("[Task] Assert failed: file=%s, line=%d\n", file, line);

    taskDISABLE_INTERRUPTS();
    for(;;) {

    }
}
/**
 * @brief Main 鍑芥暟瀹炵幇銆? */
void Main(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    logSemaphore = xSemaphoreCreateMutex();
    i2c2_mutex = xSemaphoreCreateMutex();
    I2C2_DMA_Sem = xSemaphoreCreateBinary();
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();
    USART2_DMA_Init();
    SPI2_DMA_Semaphores_Init();
    I2C_Semaphore_Init();
    BQ25895_Init();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                            ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                             ws2812_yellow_callback);
    breathTimer = xTimerCreate("BreathTimer",
                               pdMS_TO_TICKS(30),
                               pdTRUE,
                               NULL,
                               BreathingLightCallback);

    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);
    press_updateHandle = xTimerCreate("press_update", pdMS_TO_TICKS(PRESS_PID_TIMER_PERIOD_MS), pdTRUE, NULL, press_update_timer);
    eye_is_existHandle = xTimerCreate("eye_is_exist_delay", pdMS_TO_TICKS(1000), pdTRUE, NULL, eye_is_exist_callback);
    xTimerStart(tempareture_pidHandle, 0);
    xTimerStart(press_updateHandle, 0);
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0);

    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(UART_BUFFER_QUANTITY, sizeof(uart_data *));
    configASSERT(UART_DMA_IDLE_RECEPT_QUEUEHandle != NULL);

    BQ27441_InitConfig();
    BQ27441_VerifyConfig();

    PWM_WS2812B_Init();
    ADS1220_Init();
    TMC5130_Init();
    HeatInit();



    xTaskCreate(UART_RECEPT_Task, "UART_RECEPT", 500, NULL, 10, &UART_RECEPTHandle);
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





}
