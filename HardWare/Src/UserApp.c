
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
SemaphoreHandle_t BUTTON_SEMAPHOREHandle, logSemaphore, usart2_dmatxSemaphore,spi2RxDmaSemaphoreHandle,spi2TxDmaSemaphoreHandle;  // SPI2 DMA ����ź���;  // ������־�ź���;
SemaphoreHandle_t xI2CMutex;       // I2C���߻�����
SemaphoreHandle_t xI2CCompleteSem; // ��������ź���




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
    Kalman_Init(&kf, 0.1f, 1.0f);  // Q: ԽСԽƽ��, R: ԽСԽ���β���
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
        ADS1220_StartConversion();  // ����ת��
        osDelay(20);//��ֹADCоƬû��Ӧ����
        Discard_dirty_data();
        weight0 = ADS1220_ReadPressure();           // ��ȡ��ʼѹ��ֵ
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
        // �ȴ��ź�������
        if (xSemaphoreTake(BUTTON_SEMAPHOREHandle, portMAX_DELAY) == pdTRUE) {
            osDelay(100); // ������ʱ���ȴ�����״̬�ȶ�
            // ��鰴��״̬�Ƿ���Ϊ����
            if ((HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == GPIO_PIN_RESET) || (soft_button == 1)) {
                // �����ȶ���ִ�а����߼�
                Button_detection();
                soft_button = 0;
            }
        }
        // ��ʱ�������������ռ�� CPU
        // vTaskDelay(100);
    }
    /* USER CODE END Button_State_Task */
}

void APP_task(void *argument) {
uint16_t Voltage;
    for (;;) {
        HAL_IWDG_Refresh(&hiwdg);  // ��������ʱι��
        osDelay(20);//the breath of frequency
        UpdateChargeState_bq25895();
        battery_status_update_bq27441();
        UpdateState(emergency_stop, charging, low_battery, fully_charged, working);
        UpdateLightState(ChargeState);
        STATE_POWER_5V_Update();
    }
}


void Motor_go_home_task(void *argument) {
    vTaskDelay(100);//TMC5130_Init();����ͬһ���̣߳���Ҫ�ȴ�tmc5130��λ
    for (;;) {
        printf("motor go home\n");
        ADS1220_StopConversion();
        MotorChecking();
        motor_homeHandle = NULL;  // ��ɾ��ǰ�����վ��
        vTaskDelete(motor_homeHandle);
    }
}

// ���������
void Device_Check_Task(void *pvParameters) {
    while (1) {
        if (serialTimeoutFlag)
           // LOG("������Ļ���ӣ�\n");
        IIC_EYETimeoutFlag = TMP112_IsDevicePresent();//���iic�Ƿ����
        // ��ʱһ��ʱ�䣬��������ռ�ù���CPU��Դ��������ʱ100���룩
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void Main(void) {
    HAL_IWDG_Refresh(&hiwdg);  // ��������ʱι��

    logSemaphore = xSemaphoreCreateMutex();  // ����LOG�����ź���
    BUTTON_SEMAPHOREHandle = xSemaphoreCreateBinary();//����������
    usart2_dmatxSemaphore = xSemaphoreCreateBinary();        // usart2DMA�ź���
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
    xTimerStart(tempareture_pidHandle, 0); // ������ʱ��
    serialTimeoutTimerHandle = xTimerCreate("SerialTimeout",
                                            pdMS_TO_TICKS(2000),
                                            pdTRUE,  //
                                            NULL,
                                            SerialTimeout_Callback);
    xTimerStart(serialTimeoutTimerHandle, 0); // ������ʱ��


    UART_DMA_IDLE_RECEPT_QUEUEHandle = xQueueCreate(3, sizeof(uart_data *));

    AT24CXX_Init();
    BQ27441_Init();
    BQ25895_Init();
    PWM_WS2812B_Init();
    ADS1220_Init(); // ��ʼ��ADS1220
    TMC5130_Init();
    HeatInit();
    TMP112_Init();
    AD24C01_Factory_formatted();//���flashû�г�ʼ�������ʼ��

    xTaskCreate(UART_RECEPT_Task, "UART_RECEPT", 256, NULL, 6, &UART_RECEPTHandle);
    xTaskCreate(Button_State_Task, "Button_State", 256, NULL, 4, &Button_StateHandle);
    xTaskCreate(APP_task, "APP", 256, NULL, 3, &APPHandle);
    xTaskCreate(Motor_go_home_task, "Motor_go_home", 128, NULL, 2, &motor_homeHandle);
    xTaskCreate(Device_Check_Task, "Device_Check", 256, NULL, 1, &deviceCheckHandle);

}