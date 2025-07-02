
#include "main.h"

uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;

void BQ25895_Init(void) {
    CHG_CE(1);  // 关闭充电使能，准备配置
    //BQ25895_Write(0x14, 0x80); // 写1到REG_RST位，触发软件复位
    //osDelay(500);
    //BQ25895_Write(0x02, 0x30); // 开启ADC
    BQ25895_Write(0x04, 0x40); // 设置充电电流2048mA
    BQ25895_Write(0x05, 0x10); // 设置充电终止电流为64mA
    BQ25895_Write(0x07, 0x8D); // 关闭充电定时
    BQ25895_Write(0x08, 0xe7); // 补偿导线电阻
    BQ25895_Write(0x00, 0x3F); // 3.25A输入电流限制
    //BQ25895_Write(0x03, 0x12); // OTG关闭，最小系统电压3.1V
    osDelay(100);
    CHG_CE(0);  // 打开充电
}

void BQ25895_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    BQ25895_Read_IT(ReadAddr, pBuffer, 1);
}

void BQ25895_MultiRead(uint8_t *pBuffer) {
    BQ25895_Read_IT(0x00, pBuffer, 20);  // 读取 20 字节
}

void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
    BQ25895_Write_IT(WriteAddr, WriteData);
}

HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) {
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t temp = WriteData;
    // 获取 I2C 互斥锁，确保总线不会冲突
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空完成信号量，防止上次操作影响
        xSemaphoreTake(xI2CCompleteSem, 0);

        // 启动 I2C 写操作 (中断模式)
        status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, &temp, 1);
        if (status == HAL_OK) {
            // 等待传输完成 (带超时)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 传输成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // 释放 I2C 互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // 总线繁忙
    }
    return status;
}

HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    HAL_StatusTypeDef status = HAL_ERROR;
    // 获取 I2C 互斥锁，防止总线冲突
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
        // 先清空完成信号量，防止上次操作影响
        xSemaphoreTake(xI2CCompleteSem, 0);

        // 启动 I2C 读操作 (中断模式)
        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        if (status == HAL_OK) {
            // 等待传输完成 (带超时)
            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;  // 传输成功
            } else {
                // 超时处理
                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }

        // 释放 I2C 互斥锁
        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;  // 总线繁忙
    }
    return status;
}

float read_battery_voltage(uint8_t *BQ25895Reg) {
    // 合成 16 位数据
    uint16_t vbat_raw = (BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];

    // 转换为电压 (mV)
    return vbat_raw * 20.0 / 1000;
}
void BQ25895_AutoRecover(void) {
    BQ25895_MultiRead(BQ25895Reg);  // 一次读取0x00~0x14
    BQ25895_MultiRead(BQ25895Reg);  // 一次读取0x00~0x14
//
    uint8_t reg0b = BQ25895Reg[0x0B];
   // uint8_t chrg_stat = (reg0b >> 3) & 0x03;
    uint8_t chrg_stat = (reg0b & 0x18) >> 3;

    if (chrg_stat == 0x03) {
        LOG(" 充电完成：BQ25895 检测到 Charge Termination Done\n");
        return;
    }

    uint8_t reg0c ;
    BQ25895_Read(0x0C,&reg0c);
    osDelay(1);
    BQ25895_Read(0x0C,&reg0c);
    bool fault = false;

    // 解析故障类型并输出日志
    if (reg0c & (1 << 3)) {
        LOG(" 电池故障：BAT_FAULT = 1（电池可能断开或VBAT过高）\n");
        fault = true;
    }

    uint8_t chrg_fault = (reg0c >> 4) & 0x03;
    if (chrg_fault != 0x00) {
        switch (chrg_fault) {
            case 0x01:
                LOG("充电故障：输入电压异常（VBUS或VBAT问题）\n");
                break;
            case 0x02:
                LOG(" 充电故障：过温保护\n");
                break;
            case 0x03:
                LOG(" 充电故障：安全定时器超时\n");
                break;
        }
        fault = true;
    }

    if (reg0c & (1 << 7)) {
        LOG(" 看门狗超时：WATCHDOG_FAULT = 1（需要定期I2C通信或关闭看门狗）\n");
        fault = true;
    }

    // 如果存在故障，尝试重启充电逻辑
    if (1) {
        BQ25895_Init();
        LOG(" 充电已重新启用\n");
    }
}


uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
static int charge_action_done = 0;
void UpdateChargeState_bq25895(void) {

    BQ25895_MultiRead(BQ25895Reg);
//  float battery=read_battery_voltage(BQ25895Reg);
//  vTaskDelay(100);
//  printf("电压%f;",battery);
//  float bat=(4.199-(float)battery)/(0.00078);//电量百分比去掉%
//  printf("电量%f;",bat);
//  printf("百分比%f\n",bat/42);
    CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;
    // 初始化状态为 false
    // 根据 CHRG_STAT 更新状态
    switch (CHRG_STAT) {
        case 1: // Pre-charge
        case 2: // Fast Charging
            if (!charge_action_done) {  // 只执行一次
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); // 关闭屏幕
                close_mianAPP();
                vTaskSuspend(deviceCheckHandle);
                charge_action_done = 1;  // 标记已处理
            }
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//关闭屏幕
//            close_mianAPP();
//            vTaskSuspend(deviceCheckHandle);
            if (fully_charged == 0) {// 即将充满时刻，会在2和3状态来回切换
                charging = 1;
                fully_charged = 0;
                working = 0;
            } else if (fully_charged == 1) {
                fully_charged = 1;
                charging = 0;
                working = 0;
            }
            break;
        case 3: // Charge Termination Done
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//关闭屏幕
            vTaskSuspend(deviceCheckHandle);
            fully_charged = 1;
            charging = 0;
            working = 0;
            charge_action_done = 0; // 状态切换，允许下次再执行一次
            break;
        case 0: // Not Charging
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//开启屏幕
            working = 1;
            charging = 0;
            fully_charged = 0;
            charge_action_done = 0; // 状态切换，允许下次再执行一次
            break;
    }
    uint8_t PG_STAT = (BQ25895Reg[0x0B] >> 2) & 0x01;

    if (PG_STAT) {
        LOG(" PG_STAT = 1，VBUS 电源正常（Power Good）\n");
//        if ((BQ27441.Voltage < 4170)&&( BQ27441.AvgCurrent<100)&&(fully_charged==0)) {//&&(0<BQ27441.AvgCurrent )
//            LOG("low voltage or current\n");
//            BQ25895_Init();
//        }
    } else {
        LOG(" PG_STAT = 0，VBUS 输入异常或未连接\n");
    }

}

uint8_t charging_flag = 0;
void bq25895_reinitialize_if_vbus_inserted(void) {
    static uint8_t last_vbus_status = 0x00;  // 存储上一次的 VBUS 状态
    uint8_t vbus_status;

    BQ25895_Read(0x0B, &vbus_status);

    // 检测 VBUS 插入 (仅当状态从未插入 -> 插入时执行)
    if (((vbus_status & 0x80) || (vbus_status == 0x16)) &&
        !((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOG("充电器已插入，重新初始化 bq25895...\n");

//        BQ25895_Init();
        charging_flag=1;
    }

    // 检测 VBUS 拔出 (仅当状态从插入 -> 拔出时执行)
    if (!(vbus_status & 0x80) && !(vbus_status == 0x16) &&
        ((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {

        LOG("充电器已拔出，清除初始化标记...\n");
        charging_flag=0;
       // NVIC_SystemReset();
    }

    last_vbus_status = vbus_status;  // 更新状态
}
