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
// �궨�����SDA��SCL����
#define I2C_SDA_H(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_SET)
#define I2C_SDA_L(i2c) HAL_GPIO_WritePin((i2c)->SDA_Port, (i2c)->SDA_Pin, GPIO_PIN_RESET)
#define I2C_SCL_H(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_SET)
#define I2C_SCL_L(i2c) HAL_GPIO_WritePin((i2c)->SCL_Port, (i2c)->SCL_Pin, GPIO_PIN_RESET)
#define I2C_SDA_READ(i2c) HAL_GPIO_ReadPin((i2c)->SDA_Port, (i2c)->SDA_Pin)

// I2C��ʼ�ź�
void I2C_Start(SoftwareI2C *i2c) {
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_L(i2c);
    delay_us(4);
    I2C_SCL_L(i2c);
}

// I2Cֹͣ�ź�
void I2C_Stop(SoftwareI2C *i2c) {
    SDA_OUT(i2c);
    I2C_SCL_L(i2c);
    I2C_SDA_L(i2c);
    
    I2C_SCL_H(i2c);
    delay_us(4);
    I2C_SDA_H(i2c);
    delay_us(4);
}

// I2C�ȴ�Ӧ��
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

// I2C����һ���ֽ�
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

// I2C��ȡһ���ֽ�
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

// I2C����Ӧ��
void I2C_Ack(SoftwareI2C *i2c) {
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_L(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}

// I2C���Ͳ�Ӧ��
void I2C_NAck(SoftwareI2C *i2c) {
    I2C_SCL_L(i2c);
    SDA_OUT(i2c);
    I2C_SDA_H(i2c);
    delay_us(2);
    I2C_SCL_H(i2c);
    delay_us(5);
    I2C_SCL_L(i2c);
}


// // ʹ�����I2C��FT5206д������
// uint8_t WR_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len) {
//     uint8_t i;
//     uint8_t ret = 0;
//     I2C_Start(i2c);
//    // I2C_Send_Byte(i2c, FT_CMD_WR);    // ����д����
//     if (I2C_Wait_Ack(i2c)) {          // �ȴ�Ӧ��
//         ret = 1;
//     }
//     I2C_Send_Byte(i2c, reg & 0xFF);   // ���ͼĴ�����ַ
//     I2C_Wait_Ack(i2c);
//     for (i = 0; i < len; i++) {
//         I2C_Send_Byte(i2c, buf[i]);   // ��������
//         if (I2C_Wait_Ack(i2c)) {      // �ȴ�Ӧ��
//             ret = 1;
//             break;
//         }
//     }
//     I2C_Stop(i2c);                    // ����ֹͣ����
//     return ret;
// }

// // ʹ�����I2C��FT5206��ȡ����
// void RD_Reg_SoftI2C(SoftwareI2C *i2c, uint16_t reg, uint8_t *buf, uint8_t len) {
//     I2C_Start(i2c);
//     I2C_Send_Byte(i2c, TMP114Addressw);   // ���Ͷ�����
//     I2C_Wait_Ack(i2c);
//     I2C_Send_Byte(i2c, reg & 0xFF);  // ���ͼĴ�����ַ
//     I2C_Wait_Ack(i2c);
//     I2C_Stop(i2c);
//     I2C_Start(i2c);
//     I2C_Send_Byte(i2c, TMP114Addressr);   // ���Ͷ�����
//     I2C_Wait_Ack(i2c);
//     while (len) {
//         len--;
//         *buf = I2C_Read_Byte(i2c, len == 0 ? 0 : 1); // ��ȡ���ݣ����һ�����ݷ���NACK
//         buf++;
//     }
//     I2C_Stop(i2c);                  // ����ֹͣ����
// }
