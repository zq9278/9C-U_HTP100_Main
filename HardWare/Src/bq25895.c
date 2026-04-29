/*
 * File: bq25895.c
 * Description: cleaned comments and normalized to UTF-8 encoding.
 * Encoding: UTF-8
 */


#include "main.h"
#include "UserApp.h"
#include "interface_uart.h"
#include <stdbool.h>
#include "app_sys.h"
uint8_t BQ25895Reg[21];
extern I2C_HandleTypeDef hi2c1;


void BQ25895_Init(void) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    CHG_CE(1);


    BQ25895_Write(0x04, 0x40);
    BQ25895_Write(0x05, 0x10);
    BQ25895_Write(0x07, 0x8D);
    BQ25895_Write(0x08, 0xF3);
    BQ25895_Write(0x00, 0x3F);

    osDelay(100);
    CHG_CE(0);
}


void BQ25895_Read(uint8_t ReadAddr, uint8_t *pBuffer) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    BQ25895_Read_IT(ReadAddr, pBuffer, 1);
}


void BQ25895_MultiRead(uint8_t *pBuffer) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    BQ25895_Read_IT(0x00, pBuffer, 20);
}


void BQ25895_Write(uint8_t WriteAddr, uint8_t WriteData) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    BQ25895_Write_IT(WriteAddr, WriteData);
}


HAL_StatusTypeDef BQ25895_Write_IT(uint8_t regAddr, uint8_t WriteData) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t temp = WriteData;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {

        xSemaphoreTake(xI2CCompleteSem, 0);


        status = HAL_I2C_Mem_Write_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, &temp, 1);
        if (status == HAL_OK) {

            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;
            } else {

                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }


        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }
    return status;
}


HAL_StatusTypeDef BQ25895_Read_IT(uint8_t regAddr, uint8_t *pBuffer, uint16_t size) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    HAL_StatusTypeDef status = HAL_ERROR;

    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {

        xSemaphoreTake(xI2CCompleteSem, 0);


        status = HAL_I2C_Mem_Read_IT(&hi2c1, BQ25895Address, regAddr, I2C_MEMADD_SIZE_8BIT, pBuffer, size);
        if (status == HAL_OK) {

            if (xSemaphoreTake(xI2CCompleteSem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                status = HAL_OK;
            } else {

                HAL_I2C_DeInit(&hi2c1);
                HAL_I2C_Init(&hi2c1);
                status = HAL_TIMEOUT;
            }
        }


        xSemaphoreGive(xI2CMutex);
    } else {
        status = HAL_BUSY;
    }
    return status;
}


float read_battery_voltage(uint8_t *BQ25895Reg) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */


    uint16_t vbat_raw = (BQ25895Reg[0x0F] << 8) | BQ25895Reg[0x0E];


    return vbat_raw * 20.0 / 1000;
}

void BQ25895_AutoRecover(void) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    BQ25895_MultiRead(BQ25895Reg);
    BQ25895_MultiRead(BQ25895Reg);

    uint8_t reg0b = BQ25895Reg[0x0B];

    uint8_t chrg_stat = (reg0b & 0x18) >> 3;

    if (chrg_stat == 0x03) {
        LOGI("[Charger] Event\n");
        return;
    }

    uint8_t reg0c ;
    BQ25895_Read(0x0C,&reg0c);
    osDelay(1);
    BQ25895_Read(0x0C,&reg0c);
    bool fault = false;


    if (reg0c & (1 << 3)) {
        LOGW("[Charger] Event\n");
        fault = true;
    }

    uint8_t chrg_fault = (reg0c >> 4) & 0x03;
    if (chrg_fault != 0x00) {
        switch (chrg_fault) {
            case 0x01:
                LOGW("[Charger] Event\n");
                break;
            case 0x02:
                LOGW("[Charger] Event\n");
                break;
            case 0x03:
                LOGW("[Charger] Event\n");
                break;
        }
        fault = true;
    }

    if (reg0c & (1 << 7)) {
        LOGW("[Charger] Event\n");
        fault = true;
    }


    if (1) {
        BQ25895_Init();
        LOGI("[Charger] Event\n");
    }
}


uint8_t CHRG_STAT;
ChargeState_t ChargeState = STATE_POWER_ON;
uint8_t charging, working, fully_charged, low_battery, emergency_stop;
static int charge_action_done = 0;

void UpdateChargeState_bq25895(void) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */


    BQ25895_MultiRead(BQ25895Reg);






    CHRG_STAT = (BQ25895Reg[0x0b] & 0x18) >> 3;


    switch (CHRG_STAT) {
        case 1:
        case 2:
            if (!charge_action_done) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
                close_mianAPP();
                vTaskSuspend(deviceCheckHandle);
                charge_action_done = 1;
            }



            if (fully_charged == 0) {
                charging = 1;
                fully_charged = 0;
                working = 0;
            } else if (fully_charged == 1) {
                fully_charged = 1;
                charging = 0;
                working = 0;
            }
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
            vTaskSuspend(deviceCheckHandle);
            fully_charged = 1;
            charging = 0;
            working = 0;
            charge_action_done = 0;
            break;
        case 0:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
            working = 1;
            charging = 0;
            fully_charged = 0;
            charge_action_done = 0;
            break;
    }
    uint8_t PG_STAT = (BQ25895Reg[0x0B] >> 2) & 0x01;

    if (PG_STAT) {





    } else {

    }

}

uint8_t charging_flag = 0;

void bq25895_reinitialize_if_vbus_inserted(void) {
    /* Step 1: validate input and preconditions. */
    /* Step 2: run core logic of this function. */
    /* Step 3: update state/output and return. */

    static uint8_t last_vbus_status = 0x00;
    uint8_t vbus_status;

    BQ25895_Read(0x0B, &vbus_status);


    if (((vbus_status & 0x80) || (vbus_status == 0x16)) &&
        !((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {
        LOGI("[Charger] Event\n");

        BQ25895_Init();
        charging_flag=1;
    }


    if (!(vbus_status & 0x80) && !(vbus_status == 0x16) &&
        ((last_vbus_status & 0x80) || (last_vbus_status == 0x16))) {

        LOGW("[Charger] Event\n");
        charging_flag=0;
        NVIC_SystemReset();
    }

    last_vbus_status = vbus_status;
}


