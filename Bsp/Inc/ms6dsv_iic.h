/**
 ****************************************************************************************************
 * @file        ms6dsv_iic.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       iic接口
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 */
#ifndef __MS6DSV_IIC_H
#define __MS6DSV_IIC_H

#include "gpio.h"

/* 引脚定义 */
#define MS6DSV_IIC_SCL_GPIO_PORT            GPIOB
#define MS6DSV_IIC_SCL_GPIO_PIN             GPIO_PIN_10
#define MS6DSV_IIC_SCL_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)
#define MS6DSV_IIC_SDA_GPIO_PORT            GPIOB
#define MS6DSV_IIC_SDA_GPIO_PIN             GPIO_PIN_11
#define MS6DSV_IIC_SDA_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* IO操作 */
#define MS6DSV_IIC_SCL(x)                   do{ x ?                                                                                     \
                                                    HAL_GPIO_WritePin(MS6DSV_IIC_SCL_GPIO_PORT, MS6DSV_IIC_SCL_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(MS6DSV_IIC_SCL_GPIO_PORT, MS6DSV_IIC_SCL_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define MS6DSV_IIC_SDA(x)                   do{ x ?                                                                                     \
                                                    HAL_GPIO_WritePin(MS6DSV_IIC_SDA_GPIO_PORT, MS6DSV_IIC_SDA_GPIO_PIN, GPIO_PIN_SET) :    \
                                                    HAL_GPIO_WritePin(MS6DSV_IIC_SDA_GPIO_PORT, MS6DSV_IIC_SDA_GPIO_PIN, GPIO_PIN_RESET);   \
                                                }while(0)

#define MS6DSV_IIC_READ_SDA()               HAL_GPIO_ReadPin(MS6DSV_IIC_SDA_GPIO_PORT, MS6DSV_IIC_SDA_GPIO_PIN)

/* 操作函数 */
void ms6dsv_iic_start(void);                /* 产生IIC起始信号 */
void ms6dsv_iic_stop(void);                 /* 产生IIC停止信号 */
uint8_t ms6dsv_iic_wait_ack(void);          /* 等待IIC应答信号 */
void ms6dsv_iic_ack(void);                  /* 产生ACK应答信号 */
void ms6dsv_iic_nack(void);                 /* 不产生ACK应答信号 */
void ms6dsv_iic_send_byte(uint8_t dat);     /* IIC发送一个字节 */
uint8_t ms6dsv_iic_read_byte(uint8_t ack);  /* IIC接收一个字节 */
void ms6dsv_iic_init(void);                 /* 初始化IIC接口 */

#endif
