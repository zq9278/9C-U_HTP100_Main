/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-13 10:38:11
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-15 08:51:59
 * @FilePath: \software\HardWare\Inc\IIC.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef __iic_H
#define __iic_H

#include "stm32g0xx_hal.h"

// I2C结构体定义
typedef struct
{
    GPIO_TypeDef *SDA_Port;
    uint16_t SDA_Pin;
    GPIO_TypeDef *SCL_Port;
    uint16_t SCL_Pin;
} SoftwareI2C;

// GPIO初始化宏定义
#define SDA_IN(i2c)                                                            \
    {                                                                          \
        (i2c)->SDA_Port->MODER &= ~(GPIO_MODER_MODE0 << ((i2c)->SDA_Pin * 2)); \
        (i2c)->SDA_Port->MODER |= 0 << (i2c)->SDA_Pin * 2;                     \
    }

#define SDA_OUT(i2c)                                                           \
    {                                                                          \
        (i2c)->SDA_Port->MODER &= ~(GPIO_MODER_MODE0 << ((i2c)->SDA_Pin * 2)); \
        (i2c)->SDA_Port->MODER |= 1 << (i2c)->SDA_Pin * 2;                     \
    }

// I2C函数声明
void I2C_Start(SoftwareI2C *i2c);
void I2C_Stop(SoftwareI2C *i2c);
uint8_t I2C_Wait_Ack(SoftwareI2C *i2c);
void I2C_Send_Byte(SoftwareI2C *i2c, uint8_t byte);
uint8_t I2C_Read_Byte(SoftwareI2C *i2c, uint8_t ack);
void I2C_Ack(SoftwareI2C *i2c);
void I2C_NAck(SoftwareI2C *i2c);
uint8_t WR_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len);
void RD_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len);



/*SoftwareI2C i2c1 = {EYE_SDA1_GPIO_Port, EYE_SDA1_Pin, EYE_SCL1_GPIO_Port, EYE_SCL1_Pin};*/
#endif // SOFTWARE_I2C_H
