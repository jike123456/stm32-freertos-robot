/**
 ****************************************************************************************************
 * @file        ms6dsv.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信协议
 * @details
 *  本文件实现 MS6DSV 模块（核心芯片 LSM6DSV16X）的底层通信与初始化适配，主要负责：
 *   - 板级硬件引脚初始化：SA0 地址脚、INT 中断脚等
 *   - 软件 I2C（bit-bang）读写寄存器：ms6dsv_write()/ms6dsv_read()
 *   - 适配 ST 官方驱动库（lsm6dsv16x_reg.h）所需的 ctx 回调函数：
 *       ms6dsv_i2c_write_reg() / ms6dsv_i2c_read_reg() / ms6dsv_delay()
 *   - 设备 ID 校验、软复位、BDU（Block Data Update）等基础配置：ms6dsv_init()
 *
 *  模块定位：
 *   - “传感器驱动的最底层适配层”，上层可在 SensorTask 中调用 ST 官方 API
 *     完成 ODR/FS 配置、读取加速度/角速度/温度等数据。
 *
 *  通信与地址说明：
 *   - MS6DSV 模块通过 I2C 通信（本工程为软件 I2C，接口在 ms6dsv_iic.*）
 *   - SA0 引脚决定 I2C 从机地址：
 *       SA0=0 -> LSM6DSV16X_I2C_ADD_L（示例注释中为 0xD5 的 8-bit 写地址）
 *     注意：工程内部使用 7-bit 地址，因此会对宏右移 1（>>1）。
 *
 *  初始化流程（ms6dsv_init）：
 *   1) 硬件引脚初始化 + 软件 I2C 初始化
 *   2) 读取 WHO_AM_I 并校验芯片 ID（LSM6DSV16X_ID）
 *   3) 触发 restore/reset，等待 READY
 *   4) 使能 BDU：避免读多字节数据时发生“高低字节跨采样更新”的撕裂问题
 *
 *  错误码约定：
 *   - MS6DSV_EOK  : 成功
 *   - MS6DSV_EID  : 设备 ID 校验失败
 *   - MS6DSV_EACK : I2C ACK 错误（通信失败）
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "ms6dsv.h"
#include "ms6dsv_iic.h"
#include "lsm6dsv16x_reg.h"


/* MS6DSV模块IIC通讯地址 */
static uint8_t ms6dsv_iic_addr;

/**
 * @brief       MS6DSV硬件初始化
 * @param       无
 * @retval      无
 */
static void ms6dsv_hw_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    
    /* 使能GPIO时钟 */
    MS6DSV_SA0_GPIO_CLK_ENABLE();
    MS6DSV_INT_GPIO_CLK_ENABLE();
    
    /* 初始化AD0引脚 */
    gpio_init_struct.Pin    = MS6DSV_SA0_GPIO_PIN;          /* AD0引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;          /* 推挽输出 */
    gpio_init_struct.Pull   = GPIO_PULLUP;                  /* 上拉 */
    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;         /* 高速 */
    HAL_GPIO_Init(MS6DSV_SA0_GPIO_PORT, &gpio_init_struct);
    
    /* 初始化INT引脚 */
    gpio_init_struct.Pin    = MS6DSV_INT_GPIO_PIN;          /* INT引脚 */
    gpio_init_struct.Mode   = GPIO_MODE_INPUT;              /* 输入 */
    gpio_init_struct.Pull   = GPIO_PULLDOWN;                /* 下拉 */
    HAL_GPIO_Init(MS6DSV_INT_GPIO_PORT, &gpio_init_struct);
    
    /* 控制MS6DSV的SA0引脚为低电平
     * 设置其IIC的从机地址为0xD5(LSM6DSV16X_I2C_ADD_L)
     */
    MS6DSV_SA0(0);
    ms6dsv_iic_addr = LSM6DSV16X_I2C_ADD_L >> 1;
}

/**
 * @brief       往MS6DSV的指定寄存器连续写入指定数据
 * @param       addr: MS6DSV的IIC通讯地址
 *              reg : MS6DSV寄存器地址
 *              len : 写入的长度
 *              dat : 写入的数据
 * @retval      MS6DSV_EOK : 函数执行成功
 *              MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t ms6dsv_write(uint8_t addr,uint8_t reg, uint8_t len, uint8_t *dat)
{
    uint8_t i;
    
    ms6dsv_iic_start();
    ms6dsv_iic_send_byte((addr << 1) | 0);
    if (ms6dsv_iic_wait_ack() == 1)
    {
        ms6dsv_iic_stop();
        return MS6DSV_EACK;
    }
    ms6dsv_iic_send_byte(reg);
    if (ms6dsv_iic_wait_ack() == 1)
    {
        ms6dsv_iic_stop();
        return MS6DSV_EACK;
    }
    for (i=0; i<len; i++)
    {
        ms6dsv_iic_send_byte(dat[i]);
        if (ms6dsv_iic_wait_ack() == 1)
        {
            ms6dsv_iic_stop();
            return MS6DSV_EACK;
        }
    }
    ms6dsv_iic_stop();
    return MS6DSV_EOK;
}

/**
 * @brief       连续读取MS6DSV指定寄存器的值
 * @param       addr: MS6DSV的IIC通讯地址
 *              reg : MS6DSV寄存器地址
 *              len: 读取的长度
 *              dat: 存放读取到的数据的地址
 * @retval      MS6DSV_EOK : 函数执行成功
 *              MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t ms6dsv_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *dat)
{
    ms6dsv_iic_start();
    ms6dsv_iic_send_byte((addr << 1) | 0);
    if (ms6dsv_iic_wait_ack() == 1)
    {
        ms6dsv_iic_stop();
        return MS6DSV_EACK;
    }
    ms6dsv_iic_send_byte(reg);
    if (ms6dsv_iic_wait_ack() == 1)
    {
        ms6dsv_iic_stop();
        return MS6DSV_EACK;
    }
    ms6dsv_iic_start();
    ms6dsv_iic_send_byte((addr << 1) | 1);
    if (ms6dsv_iic_wait_ack() == 1)
    {
        ms6dsv_iic_stop();
        return MS6DSV_EACK;
    }
    while (len)
    {
        *dat = ms6dsv_iic_read_byte((len > 1) ? 1 : 0);
        len--;
        dat++;
    }
    ms6dsv_iic_stop();
    return MS6DSV_EOK;
}

/**
 * @brief       MS6DSV初始化
 * @param       无
 * @retval      MS6DSV_EOK: 函数执行成功
 *              MS6DSV_EID: 获取ID错误，函数执行失败
 */
uint8_t ms6dsv_init(void)
{
    uint8_t lsm6dsv16x_id;
    lsm6dsv16x_reset_t rst;
    
    /* 初始化硬件接口 */
    ms6dsv_hw_init();
    ms6dsv_iic_init();
    
    /* 校验MS6DSV模块ID */
    lsm6dsv16x_device_id_get(&ms6dsv, &lsm6dsv16x_id);
    if (lsm6dsv16x_id != LSM6DSV16X_ID)
    {
        return MS6DSV_EID;
    }
    
    /* 复位MS6DSV模块 */
    lsm6dsv16x_reset_set(&ms6dsv, LSM6DSV16X_RESTORE_CTRL_REGS);
    do {
        lsm6dsv16x_reset_get(&ms6dsv, &rst);
    } while (rst != LSM6DSV16X_READY);
    
    /* 使能块数据更新功能 */
    lsm6dsv16x_block_data_update_set(&ms6dsv, PROPERTY_ENABLE);
    
    return MS6DSV_EOK;
}

/**
 * @brief       LSM6DSV16X写寄存器函数
 * @param       handle: 未使用
 *              reg : LSM6DSV16X寄存器地址
 *              bufp: 数据
 *              len: 数据长度
 * @retval      MS6DSV_EOK : 函数执行成功
 *              MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
static int32_t ms6dsv_i2c_write_reg(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    return ms6dsv_write(ms6dsv_iic_addr, reg, len, (uint8_t *)bufp);
}

/**
 * @brief       LSM6DSV16X读寄存器函数
 * @param       handle: 未使用
 *              reg : LSM6DSV16X寄存器地址
 *              bufp: 数据
 *              len: 数据长度
 * @retval      MS6DSV_EOK : 函数执行成功
 *              MS6DSV_EACK: IIC通讯ACK错误，函数执行失败
 */
static int32_t ms6dsv_i2c_read_reg(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    return ms6dsv_read(ms6dsv_iic_addr, reg, len, (uint8_t *)bufp);
}

/**
 * @brief       LSM6DSV16X延时函数
 * @param       millisec: 延时时间，单位：毫秒
 * @retval      无
 */
static void ms6dsv_delay(uint32_t millisec)
{
    HAL_Delay(millisec);
}

/* 定义MD模块对象 */
stmdev_ctx_t ms6dsv = {
    .write_reg = ms6dsv_i2c_write_reg,
    .read_reg = ms6dsv_i2c_read_reg,
    .mdelay = ms6dsv_delay,
    .handle = (void *)0,
};
