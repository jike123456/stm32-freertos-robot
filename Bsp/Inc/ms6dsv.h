/**
 ****************************************************************************************************
 * @file        ms6dsv.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       ms6dsv的驱动接口
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 */
#ifndef __MS6DSV_H
#define __MS6DSV_H

#include "lsm6dsv16x_reg.h"

/* 引脚定义 */
#define MS6DSV_SA0_GPIO_PORT            GPIOC
#define MS6DSV_SA0_GPIO_PIN             GPIO_PIN_0
#define MS6DSV_SA0_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)
#define MS6DSV_INT_GPIO_PORT            GPIOF
#define MS6DSV_INT_GPIO_PIN             GPIO_PIN_6
#define MS6DSV_INT_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* IO操作 */
#define MS6DSV_SA0(x)                   do{ x ?                                                                             \
                                                HAL_GPIO_WritePin(MS6DSV_SA0_GPIO_PORT, MS6DSV_SA0_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(MS6DSV_SA0_GPIO_PORT, MS6DSV_SA0_GPIO_PIN, GPIO_PIN_RESET);   \
                                            }while(0)
#define MS6DSV_READ_INT()               HAL_GPIO_ReadPin(MS6DSV_INT_GPIO_PORT, MS6DSV_INT_GPIO_PIN)

/* 导出MS6DSV模块对象 */
extern stmdev_ctx_t ms6dsv;

/* 函数错误代码 */
#define MS6DSV_EOK      0   /* 没有错误 */
#define MS6DSV_EID      1   /* ID错误 */
#define MS6DSV_EACK     2   /* IIC通讯ACK错误 */
#define MS6DSV_EINVAL   3   /* 传参错误 */

/* 操作函数 */
uint8_t ms6dsv_init(void);  /* MS6DSV初始化 */

#endif
