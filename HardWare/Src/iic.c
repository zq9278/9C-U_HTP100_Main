/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-13 10:38:31
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-06-15 10:39:37
 * @FilePath: \software\HardWare\Src\iic.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */


#include "main.h"
// 宏定义控制SDA和SCL引脚
#define I2C_SDA_H(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_SET)
#define I2C_SDA_L(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_RESET)
#define I2C_SCL_H(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_SET)
#define I2C_SCL_L(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_RESET)
#define I2C_SDA_READ(i2c) HAL_GPIO_ReadPin((i2c)->SDA_Port, (i2c)->SDA_Pin)

// I2C起始信号
void I2C_Start(SoftwareI2C *i2c) {
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_L(i2c);
    delay_us(4);
    I2C_SCL_L(i2c);
}

// I2C停止信号
void I2C_Stop(SoftwareI2C *i2c) {
    SDA_OUT(i2c);
    I2C_SCL_L(i2c);
    I2C_SDA_L(i2c);
    
    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_H(i2c);
    delay_us(4);
}

// I2C等待应答
uint8_t I2C_Wait_Ack(SoftwareI2C *i2c) {
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

// I2C发送一个字节
void I2C_Send_Byte(SoftwareI2C *i2c, uint8_t byte) {
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

// I2C读取一个字节
uint8_t I2C_Read_Byte(SoftwareI2C *i2c, uint8_t ack) {
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

// I2C发送应答
void I2C_Ack(SoftwareI2C *i2c) {
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_L(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}

// I2C发送不应答
void I2C_NAck(SoftwareI2C *i2c) {
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}


// // 使用软件I2C向FT5206写入数据
// uint8_t WR_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len) {
//     uint8_t i;
//     uint8_t ret = 0;
//     I2C_Start(i2c);
//    // I2C_Send_Byte(i2c, FT_CMD_WR);    // 发送写命令
//     if (I2C_Wait_Ack(i2c)) {          // 等待应答
//         ret = 1;
//     }
//     I2C_Send_Byte(i2c, reg & 0xFF);   // 发送寄存器地址
//     I2C_Wait_Ack(i2c);
//     for (i = 0; i < len; i++) {
//         I2C_Send_Byte(i2c, buf[i]);   // 发送数据
//         if (I2C_Wait_Ack(i2c)) {      // 等待应答
//             ret = 1;
//             break;
//         }
//     }
//     I2C_Stop(i2c);                    // 生成停止条件
//     return ret;
// }

// // 使用软件I2C从FT5206读取数据
// void RD_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len) {
//     I2C_Start(i2c);
//     I2C_Send_Byte(i2c, TMP114Addressw);   // 发送读命令
//     I2C_Wait_Ack(i2c);
//     I2C_Send_Byte(i2c, reg & 0xFF);  // 发送寄存器地址
//     I2C_Wait_Ack(i2c);
//     I2C_Stop(i2c);
//     I2C_Start(i2c);
//     I2C_Send_Byte(i2c, TMP114Addressr);   // 发送读命令
//     I2C_Wait_Ack(i2c);
//     while (len) {
//         len--;
//         *buf = I2C_Read_Byte(i2c, len == 0 ? 0 : 1); // 读取数据，最后一个数据发送NACK
//         buf++;
//     }
//     I2C_Stop(i2c);                  // 产生停止条件
// }
