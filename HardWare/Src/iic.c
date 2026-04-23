/*
 * 鏂囦欢: iic.c
 * 璇存槑: HardWare 妯″潡婧愮爜鏂囦欢锛岀紪鐮佺粺涓€涓?UTF-8銆? * 娉ㄩ噴瑙勮寖: 涓枃娉ㄩ噴缁熶竴浣跨敤 UTF-8銆? */
#include "main.h"
#include "iic.h"
#include "delay.h"

#define I2C_SDA_H(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_SET)
#define I2C_SDA_L(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_RESET)
#define I2C_SCL_H(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_SET)
#define I2C_SCL_L(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_RESET)
#define I2C_SDA_READ(i2c) HAL_GPIO_ReadPin((i2c)->SDA_Port, (i2c)->SDA_Pin)


/**
 * @brief I2C_Start 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? */
void I2C_Start(SoftwareI2C *i2c) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_L(i2c);
    delay_us(4);
    I2C_SCL_L(i2c);
}


/**
 * @brief I2C_Stop 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? */
void I2C_Stop(SoftwareI2C *i2c) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    SDA_OUT(i2c);
    I2C_SCL_L(i2c);
    I2C_SDA_L(i2c);

    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_H(i2c);
    delay_us(4);
}


/**
 * @brief I2C_Wait_Ack 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t I2C_Wait_Ack(SoftwareI2C *i2c) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t ErrTime = 0;
    SDA_IN(i2c);
    I2C_SDA_H(i2c);

    delay_us(1);
    I2C_SCL_H(i2c);
    delay_us(1);

    while (I2C_SDA_READ(i2c)) {
        delay_us(1);
        ErrTime++;
        if (ErrTime > 250) {
            I2C_Stop(i2c);
            return 1;
        }
    }

    I2C_SCL_L(i2c);
    delay_us(1);
    return 0;
}


/**
 * @brief I2C_Send_Byte 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? * @param byte 鍙傛暟銆? */
void I2C_Send_Byte(SoftwareI2C *i2c, uint8_t byte) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t i = 8;
    SDA_OUT(i2c);
    I2C_SCL_L(i2c);
    while (i--) {
        if (byte & 0x80) I2C_SDA_H(i2c);
        else I2C_SDA_L(i2c);
        byte <<= 1;
        delay_us(2);
        I2C_SCL_H(i2c);
        delay_us(2);
        I2C_SCL_L(i2c);
        delay_us(2);
    }
}


/**
 * @brief I2C_Read_Byte 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? * @param ack 鍙傛暟銆? * @return 杩斿洖鍊艰鍑芥暟瀹炵幇銆? */
uint8_t I2C_Read_Byte(SoftwareI2C *i2c, uint8_t ack) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    uint8_t i = 8;
    uint8_t receive = 0;

    I2C_SDA_H(i2c);
    SDA_IN(i2c);
    while (i--) {
        receive <<= 1;
        I2C_SCL_L(i2c);
        delay_us(2);
        I2C_SCL_H(i2c);
        if (I2C_SDA_READ(i2c)) receive |= 0x01;
        delay_us(2);
    }
    if (!ack) I2C_NAck(i2c);
    else I2C_Ack(i2c);

    return receive;
}


/**
 * @brief I2C_Ack 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? */
void I2C_Ack(SoftwareI2C *i2c) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_L(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}


/**
 * @brief I2C_NAck 鍑芥暟瀹炵幇銆? * @param i2c 鍙傛暟銆? */
void I2C_NAck(SoftwareI2C *i2c) {
    /* 步骤说明：
     * 1) 处理输入参数与前置条件。
     * 2) 执行本函数核心业务逻辑。
     * 3) 输出结果/更新状态并返回。
     */
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}












































