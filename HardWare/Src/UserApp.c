
#include "UserApp.h"

extern uint8_t white_delay, yellow_delay, breathing_flag;
uint8_t Flag_400ms = 1;
float weight0 = 0;
volatile extern int Flag_3s, Flag_1s;
extern ChargeState_t ChargeState;
uint8_t soft_button;
extern uint8_t charging, working, low_battery, fully_charged, emergency_stop;
extern PID_TypeDef HeatPID;
float Heat_PWM, EyeTmp;
uint8_t flag_200ms;
uint8_t tempature_flag_400ms, press_flag_400ms, battery_flag_400ms;
uart_data *frameData_uart;

/* FreeRTOS Handles */
TaskHandle_t UART_RECEPTHandle, HeatHandle, PressHandle, Button_StateHandle, APPHandle, motor_homeHandle, deviceCheckHandle;
QueueHandle_t UART_DMA_IDLE_RECEPT_QUEUEHandle;
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore,spi2RxDmaSemaphoreHandle,spi2TxDmaSemaphoreHandle;  // SPI2 DMA 完成信号量;  // 定义日志信号量;
SemaphoreHandle_t xI2CMutex;       // I2C总线互斥量
SemaphoreHandle_t xI2CCompleteSem; // 传输完成信号量




void UART_RECEPT_Task(void *argument) {
    for (;;) {
        if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &frameData_uart, portMAX_DELAY) == pdTRUE) {
            Serial_data_stream_parsing(frameData_uart);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void Heat_Task(void *argument) {
    LOG("heat_start");
    Kalman_Init(&kf, 0.1f, 1.0f);  // Q: 越小越平滑, R: 越小越信任测量
    for (;;) {
        EyeTmp = TmpRaw2Ture();
        if (tempature_flag_400ms) {
            tempature_flag_400ms = 0;
            if (EyeTmp != 0.0f) {
                ScreenUpdateTemperature(EyeTmp);
            }
        }
//        HeatPID.integral_max = 40;
//        if ((EyeTmp>=38)&&(EyeTmp<=41.5)){
//            HeatPID.Ki=1;
//            HeatPID.integral_max = 100;
//        }
        Heat_PWM = PID_Compute(&HeatPID, EyeTmp);
        HeatPWMSet((uint8_t) Heat_PWM);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void Press_Task(void *argument) {

    for (;;) {
        printf("Press started\n");
        ADS1220_StartConversion();  // 启动转换
        osDelay(20);//防止ADC芯片没反应过来
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // 读取初始压力值
        while (1) {
            PressureControl();
            osDelay(50);
        }

    }
    /* USER CODE END Press_Task */
}

void Button_State_Task(void *argument) {
    /* USER CODE BEGIN Button_State_Task */
    /* Infinite loop */
    for (;;) {
        // 等待信号量触发
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            osDelay(100); // 消抖延时，等待按键状态稳定
            // 检查按键状态是否仍为按下
            if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) || (soft_button == 1)) {
                // 按键稳定，执行按键逻辑
                Button_detection();
                soft_button = 0;
            }
        }
        // 延时，避免任务过度占用 CPU
        // vTaskDelay(100);
    }
    /* USER CODE END Button_State_Task */
}

void APP_task(void *argument) {
uint16_t Voltage;
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);  // 正常运行时喂狗
        osDelay(20);//the breath of frequency
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();
    }
}


void Motor_go_home_task(void *argument) {
    vTaskDelay(100);//TMC5130_Init();不在同一个线程，需要等待tmc5130复位
    for (;;) {
        printf("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;  // 在删除前或后清空句柄
        vTaskDelete(motor_homeHandle);
    }
}

// 检测任务函数
void Device_Check_Task(void *pvParameters) {
    while (1) {
        if (serialTimeoutFlag)
           // LOG("请检查屏幕连接！\n");
        IIC_EYETimeoutFlag = TMP112_IsDevicePresent();//检查iic是否存在
        // 延时一段时间，避免任务占用过多CPU资源（例如延时100毫秒）
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void Main(void) {
    HAL_IWDG_Refresh(&hiwdg);  // 正常运行时喂狗

    logSemaphore = xSemaphoreCreateMutex();  // 创建LOG互斥信号量
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();//按键互斥量
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();        // usart2DMA信号量
    USART2_DMA_Init();
    SPI2_DMA_Semaphores_Init();
    I2C_Semaphore_Init();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                            ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL,
                                             ws2812_yellow_callback);
    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);
    xTimerStart(tempareture_pidHandle, 0); // 启动定时器
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,  //
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0); // 启动定时器


    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));

    AT24CXX_Init();
    BQ27441_Init();
    BQ25895_Init();
    PWM_WS2812B_Init();
    ADS1220_Init(); // 初始化ADS1220
    TMC5130_Init();
    HeatInit();
    TMP112_Init();
    AD24C01_Factory_formatted();//如果flash没有初始化，则初始化

    xTaskCreate(UART_RECEPT_Task, "UART_RECEPT", 256, NULL, 6, &UART_RECEPTHandle);
    xTaskCreate(Button_State_Task, "Button_State", 256, NULL, 4, &Button_StateHandle);
    xTaskCreate(APP_task, "APP", 256, NULL, 3, &APPHandle);
    xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle);
    xTaskCreate(Device_Check_Task, "Device_Check", 256, NULL, 1, &deviceCheckHandle);

}