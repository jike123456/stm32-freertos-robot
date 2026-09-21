/**
 ****************************************************************************************************
 * @file        ms6dsv_iic.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信协议
 * @details
 *  本文件实现 MS6DSV/LSM6DSV16X 使用的“软件 I2C”底层时序（SCL/SDA GPIO 模拟），
 *  为上层 ms6dsv.c 的寄存器读写提供最基础的 I2C 原语，包括：
 *   - 起始/停止条件：ms6dsv_iic_start() / ms6dsv_iic_stop()
 *   - 字节发送/接收：ms6dsv_iic_send_byte() / ms6dsv_iic_read_byte()
 *   - ACK/NACK：ms6dsv_iic_wait_ack() / ms6dsv_iic_ack() / ms6dsv_iic_nack()
 *   - GPIO 初始化：ms6dsv_iic_init()
 *
 *  时序说明（I2C 标准概念）：
 *   - START：SCL=1 时，SDA 从 1 -> 0
 *   - STOP ：SCL=1 时，SDA 从 0 -> 1
 *   - 数据有效：SCL 高电平期间 SDA 稳定；SCL 低电平期间允许 SDA 变化
 *   - ACK   ：发送方释放 SDA，接收方在第 9 个时钟将 SDA 拉低表示应答
 *
 *  本实现要点：
 *   - 通过 ms6dsv_iic_delay() suggests 控制 I2C 速率（当前使用 HAL_Delay(1)）
 *   - SCL 使用推挽输出（PP）；SDA 使用开漏输出（OD）以满足 I2C “线与”特性
 *   - ACK 等待带超时（waittime>250 认为失败），失败会 stop() 释放总线
 *
 *  重要提醒（工程实践）：
 *   1) HAL_Delay(1) 是毫秒级延时，I2C 速率会非常低（约几十~几百 Hz 量级），
 *      仅适合联调；建议后续替换为 delay_us() 或 DWT 微秒延时以提升性能与实时性。
 *   2) SDA 必须外部/内部上拉（通常模块自带 4.7k~10k 上拉），否则 ACK/读数据会异常。
 *   3) 软件 I2C 非线程安全：请保证同一时刻只有一个上下文访问该接口。
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "ms6dsv_iic.h"

static inline void delay_2us(void)
{
    /* 2us 对应 CPU 周期数 */
    const uint32_t cycles = 2U * (SystemCoreClock / 1000000U);

    /* SysTick 是递减计数：VAL 从 LOAD 递减到 0 再回卷 */
    uint32_t start = SysTick->VAL;
    const uint32_t load_plus_1 = SysTick->LOAD + 1U;

    uint32_t elapsed = 0;
    while (elapsed < cycles)
    {
        uint32_t now = SysTick->VAL;

        if (now <= start)
        {
            elapsed += (start - now);
        }
        else
        {
            /* 回卷：start -> 0 + LOAD -> now */
            elapsed += (start + (load_plus_1 - now));
        }

        start = now;
    }
}
/**
 * @brief       IIC接口延时函数，用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static inline void ms6dsv_iic_delay(void)
{
	delay_2us();
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void ms6dsv_iic_start(void)
{
    MS6DSV_IIC_SDA(1);
    MS6DSV_IIC_SCL(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SDA(0);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(0);
    ms6dsv_iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void ms6dsv_iic_stop(void)
{
    MS6DSV_IIC_SDA(0);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SDA(1);
    ms6dsv_iic_delay();
}

/**
 * @brief       等待IIC应答信号
 * @param       无
 * @retval      0: 应答信号接收成功
 *              1: 应答信号接收失败
 */
uint8_t ms6dsv_iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;
    
    MS6DSV_IIC_SDA(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(1);
    ms6dsv_iic_delay();
    
    while (MS6DSV_IIC_READ_SDA())
    {
        waittime++;
        
        if (waittime > 250)
        {
            ms6dsv_iic_stop();
            rack = 1;
            break;
        }
    }
    
    MS6DSV_IIC_SCL(0);
    ms6dsv_iic_delay();
    
    return rack;
}

/**
 * @brief       产生ACK应答信号
 * @param       无
 * @retval      无
 */
void ms6dsv_iic_ack(void)
{
    MS6DSV_IIC_SDA(0);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(0);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SDA(1);
    ms6dsv_iic_delay();
}

/**
 * @brief       不产生ACK应答信号
 * @param       无
 * @retval      无
 */
void ms6dsv_iic_nack(void)
{
    MS6DSV_IIC_SDA(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(1);
    ms6dsv_iic_delay();
    MS6DSV_IIC_SCL(0);
    ms6dsv_iic_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       dat: 要发送的数据
 * @retval      无
 */
void ms6dsv_iic_send_byte(uint8_t dat)
{
    uint8_t t;
    
    for (t=0; t<8; t++)
    {
        MS6DSV_IIC_SDA((dat & 0x80) >> 7);
        ms6dsv_iic_delay();
        MS6DSV_IIC_SCL(1);
        ms6dsv_iic_delay();
        MS6DSV_IIC_SCL(0);
        dat <<= 1;
    }
    MS6DSV_IIC_SDA(1);
}

/**
 * @brief       IIC接收一个字节
 * @param       ack: ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t ms6dsv_iic_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t dat = 0;
    
    for (i = 0; i < 8; i++ )
    {
        dat <<= 1;
        MS6DSV_IIC_SCL(1);
        ms6dsv_iic_delay();
        
        if (MS6DSV_IIC_READ_SDA())
        {
            dat++;
        }
        
        MS6DSV_IIC_SCL(0);
        ms6dsv_iic_delay();
    }
    
    if (ack == 0)
    {
        ms6dsv_iic_nack();
    }
    else
    {
        ms6dsv_iic_ack();
    }

    return dat;
}

/**
 * @brief       初始化IIC接口
 * @param       无
 * @retval      无
 */
void ms6dsv_iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能SCL、SDA引脚GPIO的时钟 */
    MS6DSV_IIC_SCL_GPIO_CLK_ENABLE();
    MS6DSV_IIC_SDA_GPIO_CLK_ENABLE();
    
    /* 初始化SCL引脚 */
    gpio_init_struct.Pin    = MS6DSV_IIC_SCL_GPIO_PIN;          /* SCL引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;              /* 推挽输出 */
    gpio_init_struct.Pull   = GPIO_PULLUP;                      /* 上拉 */
    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;             /* 高速 */
    HAL_GPIO_Init(MS6DSV_IIC_SCL_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化SDA引脚 */
    gpio_init_struct.Pin    = MS6DSV_IIC_SDA_GPIO_PIN;          /* SDA引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_OD;              /* 开漏输出 */
    HAL_GPIO_Init(MS6DSV_IIC_SDA_GPIO_PORT, &gpio_init_struct);
    
    ms6dsv_iic_stop();
}
