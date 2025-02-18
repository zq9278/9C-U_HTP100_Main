
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
TaskHandle_t UART_RECEPTHandle, HeatHandle, PressHandle, Button_StateHandle, APPHandle, Uart_sendHandle, AD24C01Handle,motor_homeHandle;
QueueHandle_t UART_DMA_IDLE_RECEPT_QUEUEHandle, HEAT_DATAHandle, PRESS_DATAHandle, TemperatureHandle, PressureHandle, Battery_DATAHandle, prepare_dataHandle;
SemaphoreHandle_t BUTTON_SEMAPHOREHandle;
TimerHandle_t ws2812_white_delayHandle, ws2812_yellow_delayHandle, breath_delayHandle, motor_grab3sHandle, motor_back_1sHandle, butttonHandle, tempareture_pidHandle;



void UART_RECEPT_Task(void *argument) {
    for (;;) {
        if (xQueueReceive(UART_DMA_IDLE_RECEPT_QUEUEHandle, &frameData_uart, portMAX_DELAY) == pdTRUE) {
            Serial_data_stream_parsing(frameData_uart);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void Heat_Task(void *argument) {
    for (;;) {
        EyeTmp = TmpRaw2Ture();
        if (tempature_flag_400ms) {
            tempature_flag_400ms = 0;
            xQueueSend(TemperatureHandle, &EyeTmp, 0);
        }
        Heat_PWM = PID_Compute(&HeatPID, EyeTmp);
        HeatPWMSet((uint8_t) Heat_PWM);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void Press_Task(void *argument)
{

    for (;;) {
        printf("Press started\n");
        ADS1220_StartConversion();  // 启动转换
        osDelay(20);//防止ADC芯片没反应过来
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // 读取初始压力值
        while (1){
            PressureControl();
            osDelay(50);
        }

    }
    /* USER CODE END Press_Task */
}
void Button_State_Task(void *argument)
{
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
void APP_task(void *argument)
{
    AT24CXX_Init();
    BQ27441_Init();
    BQ25895_Init();
    PWM_WS2812B_Init();
    ADS1220_Init(); // 初始化ADS1220
    TMC5130_Init();
    HeatInit();
    for (;;) {
        osDelay(20);//the breath of frequency
        LEDUpdate();
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();

    }
}
void Uart_send_task(void *argument)
{
    float tempture, pressure, battery, tempture_temp;
    static float lastBatteryValue = 0;// 声明一个变量用于存储上一次的值
    static float lastTempValue = 0;

    osTimerStart(tempareture_pidHandle, 400);
    for (;;) {
        if (osMessageQueueGet(TemperatureHandle, &tempture, NULL, 0) == osOK) {
            tempture_temp = tempture - temperature_compensation;
            if (tempture_temp != 0.0f) {
                Limit(tempture_temp, 0, tempture_temp);
                ScreenUpdateTemperature(tempture_temp);
            }
        }
        // 立即从队列接收数据，避免阻塞
        // osDelay(10);
        if (osMessageQueueGet(PressureHandle, &pressure, NULL, 0) == osOK) {
                ScreenUpdateForce(pressure);
        }
        if (osMessageQueueGet(Battery_DATAHandle, &battery, NULL, 0) == osOK) {
            if (lastBatteryValue == 0 || fabs(battery - lastBatteryValue) <= 10.0f){
                lastBatteryValue = battery;
                ScreenUpdateSOC(battery);
            }
        }
        osDelay(20);
        osThreadYield();  // 让 FreeRTOS 调度其他任务
    }
}
void AD24C01_WR(void *argument)
{
    AD24C01_Factory_formatted();
    osDelay(1000);
    prepare_data_set();
    prepare_data myprepare_data;
    for (;;) {
        //osDelay(10000);
        //prepare_data_set();
        //AT24C02_WriteAllBytes(0xff);//清理ee存储
        //printf("EEPROM is format\n");
        //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        if (osMessageQueueGet(prepare_dataHandle, &myprepare_data, NULL, 0) == osOK) {
            Eye_twitching_invalid_master(&myprepare_data);
        }
        osDelay(20);
    }
}
void Motor_go_home_task(void *argument) {
    vTaskDelay(100);//TMC5130_Init();不在同一个线程，需要等待tmc5130复位
    for (;;) {
        printf("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        vTaskDelete(motor_homeHandle);
        motor_homeHandle=NULL;


    }
}
void ws2812_white_delay_callback(TimerHandle_t xTimer) {
    Flag_400ms = 1;
    white_delay = !white_delay;
    xTimerStart(ws2812_white_delayHandle, 0);
}
void ws2812_yellow_callback(TimerHandle_t xTimer) {
    emergency_stop = false;
    yellow_delay = 0;
}
void breath_delay_Callback(TimerHandle_t xTimer) {
    breathing_flag = 1;
}
void motor_grab3s_Callback(TimerHandle_t xTimer) {
    Flag_3s = 1;
}
void motor_back_1sCallback(TimerHandle_t xTimer) {
    Flag_1s = 1;
}
void buttton_Callback(TimerHandle_t xTimer) {
    if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) {
        xSemaphoreGive(BUTTON_SEMAPHOREHandle);
    }
}
void tempareture_pid_timer(TimerHandle_t xTimer) {
    flag_200ms = 1;
    tempature_flag_400ms = 1;
    press_flag_400ms = 1;
    battery_flag_400ms = 1;
}


void Main(void ){
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();
    ws2812_white_delayHandle = xTimerCreate("ws2812_white_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, ws2812_white_delay_callback);
    ws2812_yellow_delayHandle = xTimerCreate("ws2812_yellow_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, ws2812_yellow_callback);
    breath_delayHandle = xTimerCreate("breath_delay", pdMS_TO_TICKS(400), pdFALSE, NULL, breath_delay_Callback);
    motor_grab3sHandle = xTimerCreate("motor_grab3s", pdMS_TO_TICKS(3000), pdFALSE, NULL, motor_grab3s_Callback);
    motor_back_1sHandle = xTimerCreate("motor_back_1s", pdMS_TO_TICKS(1000), pdFALSE, NULL, motor_back_1sCallback);
    butttonHandle = xTimerCreate("buttton", pdMS_TO_TICKS(100), pdTRUE, NULL, buttton_Callback);
    tempareture_pidHandle = xTimerCreate("tempareture_pid", pdMS_TO_TICKS(400), pdTRUE, NULL, tempareture_pid_timer);

    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));
    HEAT_DATAHandle = xQueueCreate(5, sizeof(PID_TypeDef));
    PRESS_DATAHandle = xQueueCreate(5, sizeof(PID_TypeDef));
    TemperatureHandle = xQueueCreate(5, sizeof(float));
    PressureHandle = xQueueCreate(5, sizeof(float));
    Battery_DATAHandle = xQueueCreate(5, sizeof(float));
    prepare_dataHandle = xQueueCreate(10, sizeof(prepare_data));

    xTaskCreate(UART_RECEPT_Task, "UART_RECEPT", 256, NULL, 4, &UART_RECEPTHandle);
    xTaskCreate(Button_State_Task, "Button_State", 256, NULL, 4, &Button_StateHandle);
    xTaskCreate(APP_task, "APP", 256, NULL, 3, &APPHandle);
    xTaskCreate(Uart_send_task, "Uart_send", 256, NULL, 2, &Uart_sendHandle);
    xTaskCreate(AD24C01_WR, "AD24C01", 128, NULL, 2, &AD24C01Handle);
    xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 1, &motor_homeHandle);

}