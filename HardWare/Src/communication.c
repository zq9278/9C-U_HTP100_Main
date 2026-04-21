
#include "main.h"
#include <stdint.h>
#include "communication.h"
#include "pid.h"
#include "interface_uart.h"
#include "Button.h"
#include "24cxx.h"
#include "tmp112.h"
#include "heat.h"
#include "UserApp.h"
#include "app_sys.h"
#include "time_callback.h"

//extern TaskHandle_t HeatHandle;
//extern volatile SystemState_t currentState;
extern PID_TypeDef HeatPID;
extern PID_TypeDef MotorPID;
extern uint8_t emergency_stop;
uint8_t heat_finish = 0, press_finish = 0, auto_finish = 0;
extern prepare_data my_prepare_data;
extern uint8_t soft_button;
uint16_t save_prepare, set_prepare;
uint8_t factory_mode = 0;
/* <<<<<<<<<<<<<<  ? Windsurf Command ? >>>>>>>>>>>>>>>> */
/**
 * @brief Handles UART1 commands by processing the received message and executing corresponding actions.
 *
 * This function processes incoming messages received via UART1, determines the command type from the message,
 * and performs actions based on the command type. It handles various command types such as starting/stopping
 * heating, pressing, and auto modes, updating task statuses, and managing emergency stops. The function also
 * communicates with other components through data queues and semaphores.
 *
 * @param msg Pointer to the received message structure of type recept_data_p.
 *
 * Note: The function checks for a NULL message pointer and returns early with an error message if the pointer is NULL.
 */

/* <<<<<<<<<<  c3e7c9e2-cbba-4bb0-8638-581c28d7cc08  >>>>>>>>>>> */
void UART1_CMDHandler(recept_data_p msg) {
    if (msg == NULL) {
        LOG("[ERROR] CMDHandler: msg is NULL!\n");
        return;
    }

    uint16_t hot_count, crimp_count, auto_count;
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    float data = (float) msg->data;

    LOG("[CMD] Received: cmd_type=0x%04X, data=%.2f, currentState=%d\n",
        cmd_type, data, currentState);

    switch (cmd_type) {
    case 0x8900:  // 完成信号
        LOG("[INFO] Finish signal received\n");
        if (currentState == STATE_HEAT) {
            heat_finish = 1;
            hot_count = AT24CXX_ReadOrWriteZero(0x00);
            AT24CXX_WriteUInt16(0x00, hot_count + 1);
            my_prepare_data.cmd_type_low = 0xA0;
            my_prepare_data.value = hot_count + 1;
            Eye_twitching_invalid_master(&my_prepare_data);
            LOG("[STATE] HEAT finished, total=%d\n", hot_count + 1);
        }
        else if (currentState == STATE_PRESS) {
            press_finish = 1;
            crimp_count = AT24CXX_ReadOrWriteZero(0x02);
            AT24CXX_WriteUInt16(0x02, crimp_count + 1);
            my_prepare_data.cmd_type_low = 0xA1;
            my_prepare_data.value = crimp_count + 1;
            Eye_twitching_invalid_master(&my_prepare_data);
            LOG("[STATE] PRESS finished, total=%d\n", crimp_count + 1);
        }
        else if (currentState == STATE_AUTO) {
            auto_finish = 1;
            auto_count = AT24CXX_ReadOrWriteZero(0x04);
            AT24CXX_WriteUInt16(0x04, auto_count + 1);
            my_prepare_data.cmd_type_low = 0xA2;
            my_prepare_data.value = auto_count + 1;
            Eye_twitching_invalid_master(&my_prepare_data);
            LOG("[STATE] AUTO finished, total=%d\n", auto_count + 1);
        }
        else {
            LOG("[WARN] Finish signal ignored (state=%d)\n", currentState);
        }
        break;

    case 0x1041:  // 屏幕加热开始
        if (EYE_status == 0) {
            LOG("[WARN] Skip HEAT start: EYE_status=0\n");
            break;
        }
        currentState = STATE_PRE_HEAT;
        emergency_stop = 0;
        HeatPWM(1);
        HeatPID.setpoint = 40 + temperature_compensation;
        if (HeatHandle != NULL) {
            vTaskResume(HeatHandle);
            LOG("[TASK] HeatHandle resumed\n");
        }
        LOG("[STATE] -> PRE_HEAT, setpoint=%.2f\n", HeatPID.setpoint);
        break;

    case 0x1030:  // 屏幕加热停止
        if (currentState == STATE_HEAT || currentState == STATE_PRE_HEAT) {
            HeatPWM(0);
            LOG("[ACTION] HeatPWM stopped\n");
            if (HeatHandle != NULL) {
                xTaskNotifyGive(HeatHandle);
                LOG("[TASK] HeatHandle notified to exit\n");
            }
            if (heat_finish == 0 && currentState == STATE_HEAT) {
                emergency_stop = 1;
                LOG("[WARN] Emergency stop triggered in HEAT unfinished\n");
            }
        }
        currentState = STATE_OFF;
        LOG("[STATE] -> OFF (from HEAT stop)\n");
        break;

    case 0x1005:  // 屏幕挤压开始
        if (EYE_status == 0) {
            LOG("[WARN] Skip PRESS start: EYE_status=0\n");
            break;
        }
        currentState = STATE_PRE_PRESS;
        emergency_stop = 0;
        MotorPID.setpoint = data;
        LOG("[STATE] -> PRE_PRESS, MotorPID.setpoint=%.2f\n", MotorPID.setpoint);
        break;

    case 0x1034:  // 屏幕挤压停止
        if (currentState == STATE_PRESS) {
            if (press_finish == 0) {
                emergency_stop = 1;
                LOG("[WARN] Emergency stop triggered in PRESS unfinished\n");
            }
            if (PressHandle != NULL) {
                xTaskNotifyGive(PressHandle);
                LOG("[TASK] PressHandle notified to suspend\n");
            }
            if (motor_homeHandle == NULL) {
                if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2,
                                &motor_homeHandle) == pdPASS) {
                    LOG("[TASK] Motor_go_home created\n");
                } else {
                    LOG("[ERROR] Failed to create Motor_go_home task\n");
                }
            } else {
                LOG("[INFO] Motor_go_home already running\n");
            }
        }
        currentState = STATE_OFF;
        LOG("[STATE] -> OFF (from PRESS stop)\n");
        break;

    case 0x1037:  // 屏幕自动模式开始
        if (EYE_status == 0) {
            LOG("[WARN] Skip AUTO start: EYE_status=0\n");
            break;
        }
        currentState = STATE_PRE_AUTO;
        emergency_stop = 0;
        HeatPWM(1);
        HeatPID.integral = 0;
        HeatPID.setpoint = 40 + temperature_compensation;
        MotorPID.setpoint = data;
        if (HeatHandle != NULL) {
            vTaskResume(HeatHandle);
            LOG("[TASK] HeatHandle resumed\n");
        }
        LOG("[STATE] -> PRE_AUTO, HeatSP=%.2f, MotorSP=%.2f\n",
            HeatPID.setpoint, MotorPID.setpoint);
        break;

    case 0x1038:  // 屏幕自动模式停止
        if (currentState == STATE_PRE_AUTO) {
            HeatPWM(0);
            if (HeatHandle != NULL) {
                xTaskNotifyGive(HeatHandle);
                LOG("[TASK] HeatHandle notified to exit\n");
            }
        }
        if (currentState == STATE_AUTO) {
            HeatPWM(0);
            close_mianAPP();
            if (auto_finish == 0) {
                emergency_stop = 1;
                LOG("[WARN] Emergency stop in AUTO unfinished\n");
            }
        }
        currentState = STATE_OFF;
        LOG("[STATE] -> OFF (from AUTO stop)\n");
        break;

    case 0x1040:  // 软按钮事件
    case 0x1006:
    case 0x1036:
        soft_button = 1;
        xSemaphoreGive(BUTTON_SEMAPHOREHandle);
        LOG("[EVENT] Soft button pressed, semaphore released\n");
        break;

    case 0x1051:  // 屏幕存在信号
        serialTimeoutFlag = 0;
        xTimerReset(serialTimeoutTimerHandle, 0);
        LOG("[EVENT] Screen alive signal, timer reset\n");
        break;

    case 0x1050:  // 屏幕开机信号
        serialTimeoutFlag = 0;
        prepare_data_set();
        currentState = STATE_OFF;
        vTaskResume(deviceCheckHandle);
        LOG("[EVENT] Screen power-on, deviceCheck resumed\n");
        break;

    case 0x1052:  // 屏幕应答
        LOG("[EVENT] Screen is open\n");
        break;

    case 0x1053:  // 设置目标温度
        HeatPID.setpoint = data + temperature_compensation;
        LOG("[PARAM] HeatPID.setpoint=%.2f\n", HeatPID.setpoint);
        break;

    case 0x1054:  // 设置目标压力
        MotorPID.setpoint = data;
        LOG("[PARAM] MotorPID.setpoint=%.2f\n", MotorPID.setpoint);
        break;

    case 0x1055:  // 工厂模式
        factory_mode = 1;
        currentState = STATE_PRE_AUTO;
        emergency_stop = 0;
        HeatPWM(1);
        if (HeatHandle != NULL) {
            vTaskResume(HeatHandle);
            LOG("[TASK] HeatHandle resumed (factory)\n");
        }
        if (motor_homeHandle == NULL) {
            if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2,
                            &motor_homeHandle) == pdPASS) {
                LOG("[TASK] Motor_go_home created (factory)\n");
            } else {
                LOG("[ERROR] Failed to create Motor_go_home task (factory)\n");
            }
        } else {
            LOG("[INFO] Motor_go_home already running (factory)\n");
        }
        LOG("[STATE] -> PRE_AUTO (factory)\n");
        break;

    case 0x1056:  // 上报次数
    {
        prepare_data my_prepare_data_times;
        my_prepare_data_times.cmd_head_high = 0x6A;
        my_prepare_data_times.cmd_head_low = 0xA6;
        my_prepare_data_times.frame_length = 0x0b;
        my_prepare_data_times.cmd_type_high = 0x00;
        my_prepare_data_times.end_high = 0xFF;
        my_prepare_data_times.end_low = 0xFF;

        uint16_t eye_times = AT24CXX_ReadOrWriteZero(0xf2);
        my_prepare_data_times.cmd_type_low = 0xb0;
        my_prepare_data_times.value = eye_times;
        Eye_twitching_invalid_master(&my_prepare_data_times);
        LOG("[REPORT] eye_times=%d\n", eye_times);
    }
        break;

    default:
        LOG("[WARN] Unknown cmd_type=0x%04X\n", cmd_type);
        break;
    }
}

void UART1_CMDHandler_prepare(prepare_data_p msg) {
    if (msg == NULL) {
        LOG("[ERROR] prepare: msg NULL\n");
        return;
    }

    uint16_t prepare_press_pre = 0, prepare_temperature_pre = 0, prepare_time_pre = 0;
    uint16_t cmd_type = ((msg->cmd_type_high) << 8) | (msg->cmd_type_low);
    uint16_t data = (uint16_t) msg->value;

    // 初始化实例
    my_prepare_data.cmd_head_high = 0x6A;
    my_prepare_data.cmd_head_low = 0xA6;
    my_prepare_data.cmd_type_high = 0x00;
    my_prepare_data.end_high = 0xFF;
    my_prepare_data.end_low = 0xFF;

    LOG("[CMD_PREPARE] cmd_type=0x%04X, data=%u, save_prepare=%u set_prepare=%u\n",
        cmd_type, data, save_prepare, set_prepare);

    switch (cmd_type) {
    case 0x1042:  // 设置预设值读取（小箭头）
        save_prepare = data;
        LOG("[STATE] Save prepare=%u\n", save_prepare);
        switch (save_prepare) {
        case 0:
            prepare_press_pre = 150;
            prepare_temperature_pre = 42;
            prepare_time_pre = 1;
            LOG("[PRESET] Default: press=150, temp=42, time=1\n");
            break;
        case 1:
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x08);
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x0A);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x0C);
            LOG("[PRESET] Read slot1: temp=%u, press=%u, time=%u\n",
                prepare_temperature_pre, prepare_press_pre, prepare_time_pre);
            break;
        case 2:
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x10);
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x12);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x14);
            LOG("[PRESET] Read slot2: temp=%u, press=%u, time=%u\n",
                prepare_temperature_pre, prepare_press_pre, prepare_time_pre);
            break;
        case 3:
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x18);
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x1A);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x1C);
            LOG("[PRESET] Read slot3: temp=%u, press=%u, time=%u\n",
                prepare_temperature_pre, prepare_press_pre, prepare_time_pre);
            break;
        default:
            LOG("[WARN] Invalid save_prepare=%u\n", save_prepare);
            break;
        }
        // 下发三条数据
        my_prepare_data.cmd_type_low = 0xA9;
        my_prepare_data.value = prepare_press_pre;
        Eye_twitching_invalid_master(&my_prepare_data);

        my_prepare_data.cmd_type_low = 0xA8;
        my_prepare_data.value = prepare_temperature_pre;
        Eye_twitching_invalid_master(&my_prepare_data);

        my_prepare_data.cmd_type_low = 0xAA;
        my_prepare_data.value = prepare_time_pre;
        Eye_twitching_invalid_master(&my_prepare_data);
        break;

    case 0x1044:  // 选择预设值（大框）
        set_prepare = data;
        AT24CXX_WriteUInt16(0xFC, set_prepare);
        LOG("[STATE] Set prepare=%u saved @0xFC\n", set_prepare);
        switch (set_prepare) {
        case 0:
            prepare_press_pre = 150;
            prepare_temperature_pre = 42;
            prepare_time_pre = 1;
            LOG("[PRESET] Default: press=150, temp=42, time=1\n");
            break;
        case 1:
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x08);
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x0A);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x0C);
            break;
        case 2:
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x10);
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x12);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x14);
            break;
        case 3:
            prepare_press_pre = AT24CXX_ReadOrWriteZero(0x18);
            prepare_temperature_pre = AT24CXX_ReadOrWriteZero(0x1A);
            prepare_time_pre    = AT24CXX_ReadOrWriteZero(0x1C);
            break;
        default:
            LOG("[WARN] Invalid set_prepare=%u\n", set_prepare);
            break;
        }
        LOG("[PRESET] Apply slot%u: press=%u, temp=%u, time=%u\n",
            set_prepare, prepare_press_pre, prepare_temperature_pre, prepare_time_pre);
        // 下发三条数据
        my_prepare_data.cmd_type_low = 0xA4;
        my_prepare_data.value = prepare_press_pre;
        Eye_twitching_invalid_master(&my_prepare_data);

        my_prepare_data.cmd_type_low = 0xA5;
        my_prepare_data.value = prepare_temperature_pre;
        Eye_twitching_invalid_master(&my_prepare_data);

        my_prepare_data.cmd_type_low = 0xA6;
        my_prepare_data.value = prepare_time_pre;
        Eye_twitching_invalid_master(&my_prepare_data);
        break;

    case 0x1039:  // 预设温度
        switch (save_prepare) {
        case 1: AT24CXX_WriteUInt16(0x08, data); break;
        case 2: AT24CXX_WriteUInt16(0x10, data); break;
        case 3: AT24CXX_WriteUInt16(0x18, data); break;
        default: LOG("[WARN] Invalid save_prepare=%u for temp\n", save_prepare); break;
        }
        LOG("[PARAM] Save slot%u temp=%u\n", save_prepare, data);
        break;

    case 0x1040:  // 预设压力
        switch (save_prepare) {
        case 1: AT24CXX_WriteUInt16(0x0A, data); break;
        case 2: AT24CXX_WriteUInt16(0x12, data); break;
        case 3: AT24CXX_WriteUInt16(0x1A, data); break;
        default: LOG("[WARN] Invalid save_prepare=%u for press\n", save_prepare); break;
        }
        LOG("[PARAM] Save slot%u press=%u\n", save_prepare, data);
        break;

    case 0x1041:  // 预设时间
        switch (save_prepare) {
        case 1: AT24CXX_WriteUInt16(0x0C, data); break;
        case 2: AT24CXX_WriteUInt16(0x14, data); break;
        case 3: AT24CXX_WriteUInt16(0x1C, data); break;
        default: LOG("[WARN] Invalid save_prepare=%u for time\n", save_prepare); break;
        }
        LOG("[PARAM] Save slot%u time=%u\n", save_prepare, data);
        break;

    case 0x1043:  // 其他预设参数
        AT24CXX_WriteUInt16(0xF8, data);
        LOG("[PARAM] Saved param@0xF8=%u\n", data);
        break;

    case 0x1046:  // 清理 EEPROM
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        AT24C02_WriteAllBytes(0xFF);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        LOG("[ACTION] Cleared EEPROM all bytes\n");
        break;

    case 0x1047:  // 清理眼部 EEPROM
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        AT24C02_WriteAllBytes_eye(0xFF);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        LOG("[ACTION] Cleared EEPROM eye bytes\n");
        break;

    default:
        LOG("[WARN] Unknown prepare cmd_type=0x%04X\n", cmd_type);
        break;
    }
}

void command_parsing(uart_data *received_data) {
    if (received_data == NULL) {
        LOG("[ERROR] command_parsing: received_data NULL\n");
        return;
    }

    if (received_data->length < 2) {
        LOG("[ERROR] command_parsing: invalid length=%u\n", received_data->length);
        return;
    }

    uint16_t cmd_type =
        (received_data->buffer[0] << 8) |
        received_data->buffer[1];

    LOG("[CMD_PARSE] New frame: cmd_type=0x%04X, length=%u\n",
        cmd_type, received_data->length);

    switch (cmd_type) {
    case 0x5aa5: {  // 工作命令
        LOG("[DISPATCH] -> UART1_CMDHandler\n");
        UART1_CMDHandler((recept_data *)received_data->buffer);
        break;
    }

    case 0x6aa6: {  // prepare 命令
        LOG("[DISPATCH] -> UART1_CMDHandler_prepare\n");
        UART1_CMDHandler_prepare((prepare_data *)received_data->buffer);
        break;
    }

    case 0x7aa7: {  // 更新电机 PID
        recept_data_debug_p press_pid_data = (recept_data_debug *) received_data->buffer;
        LOG("[PID] Motor update: P=%.3f, I=%.3f, D=%.3f, setpoint=%.2f\n",
            press_pid_data->p,
            press_pid_data->i,
            press_pid_data->d,
            press_pid_data->setpoint);

        PID_Init(&MotorPID,
                 press_pid_data->p, press_pid_data->i, press_pid_data->d,
                 5000, -5000, 50000, -50000, press_pid_data->setpoint);
        LOG("[PID] MotorPID initialized\n");
        break;
    }

    case 0x9aa9: {  // 更新加热 PID
        recept_data_debug_p heat_pid_data = (recept_data_debug *) received_data->buffer;
        LOG("[PID] Heat update: P=%.3f, I=%.3f, D=%.3f, setpoint=%.2f\n",
            heat_pid_data->p,
            heat_pid_data->i,
            heat_pid_data->d,
            heat_pid_data->setpoint);

        PID_Init(&HeatPID,
                 heat_pid_data->p, heat_pid_data->i, heat_pid_data->d,
                 300, -300, 255, 0, heat_pid_data->setpoint);
        LOG("[PID] HeatPID initialized\n");
        break;
    }

    default:
        LOG("[WARN] Unknown cmd_type=0x%04X, ignored\n", cmd_type);
        break;
    }
}

extern UART_HandleTypeDef huart2;

void ScreenUpdateForce(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    if (currentState == STATE_PRESS) {
        pData.cmd_type_low = 0x05;
    } else if (currentState == STATE_AUTO) {
        pData.cmd_type_low = 0x47;
    }
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *) &pData, sizeof(pData));

}

void ScreenUpdateTemperature(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    if (currentState == STATE_HEAT || currentState == STATE_PRE_HEAT) {
        pData.cmd_type_low = 0x41;
    }
    if (currentState == STATE_AUTO || currentState == STATE_PRE_AUTO) {
        pData.cmd_type_low = 0x37;
    }
    pData.data = value;

    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));

}

void ScreenUpdateHeatPower(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x58;
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff;
    pData.end_low = 0xff;
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}

void ScreenUpdateHeatLoadStatus(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x59;
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff;
    pData.end_low = 0xff;
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}

void ScreenUpdateSOC(float value) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x50;
    pData.data = value;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));

}

void ScreenWorkModeQuit(void) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x51;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));

}

void EYE_checkout(float data) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x55;
    pData.data = data;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));

}

void ScreenTimerStart(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x52;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
if(factory_mode !=1){
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}
    //USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}
void ScreenTimerStop(void) {
    factory_mode =0;
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x56;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}
void NEW_EYE(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x20;
    pData.cmd_type_low = 0x57;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}
void Eye_twitching_invalid(void) {

    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x91;
    pData.cmd_type_low = 0x00;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)&pData, sizeof(pData),100);
//    taskEXIT_CRITICAL();
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}

void Eye_twitching_invalid_master(prepare_data_p myprepare_data) {
    myprepare_data->frame_length = 0x0b;
    myprepare_data->crc = Calculate_CRC((uint8_t *) myprepare_data, sizeof(*myprepare_data) - 4);
//    taskENTER_CRITICAL();
//    HAL_UART_Transmit(&huart2, (uint8_t *)myprepare_data, sizeof(*myprepare_data),100);
//    taskEXIT_CRITICAL();
    vTaskDelay(10);
    USART2_DMA_Send((uint8_t *) myprepare_data, sizeof(*myprepare_data));

}

void ScreenWorkMode_count(float count) {
    static recept_data pData;
    pData.cmd_head_high = 0x5A;
    pData.cmd_head_low = 0xA5;
    pData.frame_length = 0x0d;
    pData.cmd_type_high = 0x90;
    pData.cmd_type_low = 0x00;
    pData.data = count;
    pData.crc = Calculate_CRC((uint8_t *) &pData, sizeof(pData) - 4);
    pData.end_high = 0xff; // 帧尾
    pData.end_low = 0xff;  // 帧尾
    USART2_DMA_Send((uint8_t *)&pData, sizeof(pData));
}

void Serial_data_stream_parsing(uart_data *frameData) {
    if (frameData == NULL) {
        LOG("Error: Invalid frameData pointer or size\n");
    }
    for (uint16_t i = 0; i < frameData->length - 1; i++) {
        // if (frameData->buffer[i] == FRAME_HEADER_BYTE1 && i + 1 < frameData->length && frameData->buffer[i + 1] == FRAME_HEADER_BYTE2) {
        if ((frameData->buffer[i] == 0x7a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xA7) ||
            (frameData->buffer[i] == FRAME_HEADER_BYTE1 && i + 1 < frameData->length &&
             frameData->buffer[i + 1] == FRAME_HEADER_BYTE2) ||
            (frameData->buffer[i] == 0x6a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xa6) ||
            (frameData->buffer[i] == 0x9a && i + 1 < frameData->length && frameData->buffer[i + 1] == 0xa9)) {
            for (uint16_t j = i + 2; j < frameData->length - 1; j++) {
                if (frameData->buffer[j] == FRAME_TAIL_BYTE1 && j + 1 < frameData->length &&
                    frameData->buffer[j + 1] == FRAME_TAIL_BYTE2) {
                    uint16_t frame_size = j - i + 2;
                    if (frame_size > UART_RX_BUFFER_SIZE) {
                        LOG("Error: Frame size exceeds buffer limit\n");
                        break;
                    }
                    // 计算CRC并校验
                    uint16_t received_crc = (frameData->buffer[j - 2] | (frameData->buffer[j - 1] << 8));
                    uint16_t calculated_crc = Calculate_CRC(&frameData->buffer[i], frame_size - 4); // 不包含CRC和帧尾
                    if (calculated_crc == received_crc) {
                        command_parsing((uart_data *)&frameData->buffer[i]);
for(uint16_t i = 0; i < frameData->length; i++) {
                             //LOG("%02X ", frameData->buffer[i]);
                         }
                         LOG("\n");
                    } else {
                        LOG("Error: CRC mismatch\n");
                    }
                    // 跳过已解析的帧数据，避免重复解析
                    i = j + 1;
                    break;
                }
            }
        }
    }

}
