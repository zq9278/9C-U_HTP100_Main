/*
 * 文件: iic.h
 * 说明: HardWare 模块源码文件，编码统一为 UTF-8。
 * 注释规范: 中文注释统一使用 UTF-8。
 */
#ifndef __iic_H
#define __iic_H

#include "stm32g0xx_hal.h"


typedef struct
{
    GPIO_TypeDef *SDA_Port;
    uint16_t SDA_Pin;
    GPIO_TypeDef *SCL_Port;
    uint16_t SCL_Pin;
} SoftwareI2C;


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


void I2C_Start(SoftwareI2C *i2c);
void I2C_Stop(SoftwareI2C *i2c);
uint8_t I2C_Wait_Ack(SoftwareI2C *i2c);
void I2C_Send_Byte(SoftwareI2C *i2c, uint8_t byte);
uint8_t I2C_Read_Byte(SoftwareI2C *i2c, uint8_t ack);
void I2C_Ack(SoftwareI2C *i2c);
void I2C_NAck(SoftwareI2C *i2c);
uint8_t WR_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len);
void RD_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len);




#endif


