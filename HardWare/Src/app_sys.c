/*
 * 鏂囦欢: app_sys.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "bq25895.h"
#include "main.h"
#include <stdint.h>
#include "Button.h"
#include "communication.h"
#include "UserApp.h"
#include "interface_uart.h"
uint8_t STATE_POWER_5V;
uint8_t reset=0;
/**
 * @brief STATE_POWER_5V_Update 鍑芥暟瀹炵幇銆? */
void STATE_POWER_5V_Update(void) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */

    if (reset==1) {


  STATE_POWER_5V = 1;
} else {
  STATE_POWER_5V = 0;
};
}
void close_mianAPP(void)
{
    if (currentState!=STATE_OFF) {
        if(currentState!=STATE_PRE_HEAT&&currentState!=STATE_PRE_AUTO){
            ScreenWorkModeQuit();
            ScreenTimerStop();
        }

        taskENTER_CRITICAL();
        currentState = STATE_OFF;

        if (PressHandle != NULL && eTaskGetState(PressHandle) != eSuspended) {
            xTaskNotifyGive(PressHandle);
        }
        if (HeatHandle != NULL && eTaskGetState(HeatHandle) != eSuspended) {
            xTaskNotifyGive(HeatHandle);
        }
        if (motor_homeHandle == NULL) {
            if (xTaskCreate(Motor_go_home_task, "Motor_go_home", 256, NULL, 2, &motor_homeHandle)== pdPASS) {
            } else {
                LOGE("[System] Event\r\n");
            }
        } else {
            LOGW("[System] Event\r\n");
        }

        taskEXIT_CRITICAL();
    }
}


