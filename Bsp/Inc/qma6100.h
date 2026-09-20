#ifndef __QMA6100P_H_
#define __QMA6100P_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define QMA6100P_ADDR                     0x12      /* QMA6100P地址 */
#define QMA6100P_READ                     0X25      /* 写操作 */
#define QMA6100P_WRITE                    0X24      /* 读操作 */

/* QMA6100P命令 */

/* 获取ID,默认值为0x9x */
#define QMA6100P_DEVICE_ID                0x90
#define QMA6100P_REG_CHIP_ID              0x00

/* 数据寄存器,三轴数据,默认值为0x00 */ 
#define QMA6100P_REG_XOUTL                0x01
#define QMA6100P_REG_XOUTH                0x02
#define QMA6100P_REG_YOUTL                0x03
#define QMA6100P_REG_YOUTH                0x04
#define QMA6100P_REG_ZOUTL                0x05
#define QMA6100P_REG_ZOUTH                0x06

/* 带宽寄存器 */
#define QMA6100P_REG_BW_ODR               0x10
/* 电源管理寄存器 */ 
#define QMA6100P_REG_POWER_MANAGE         0x11
/* 加速度范围,设置加速度计的满刻度 */ 
#define QMA6100P_REG_RANGE                0x0f
/* 软件复位 */  
#define QMA6100P_REG_RESET                0x36

#define QMA6100P_REG_ACC_VAL(lsb, msb)  ((int16_t)(((uint16_t)msb << 8) | ((uint16_t)lsb & 0xFC)) >> 2)

typedef struct {
    uint8_t data[2];
    float  acc_x;
    float  acc_y;
    float  acc_z;
    float  acc_g;
    float  pitch;                       /* 围绕X轴旋转,也叫做俯仰角 */
    float  roll;                        /* 围绕Z轴旋转,也叫翻滚角 */
}qma6100p_rawdata_t;

/* 函数声明 */
uint8_t qma6100p_init(void);            /* 初始化三轴加速度计 */
void qma6100p_read_rawdata(qma6100p_rawdata_t *rawdata);

/* 设置量程寄存器 */
typedef enum
{
    QMA6100P_BW_100 = 0,
    QMA6100P_BW_200 = 1,
    QMA6100P_BW_400 = 2,
    QMA6100P_BW_800 = 3,
    QMA6100P_BW_1600 = 4,
    QMA6100P_BW_50 = 5,
    QMA6100P_BW_25 = 6,
    QMA6100P_BW_12_5 = 7,
    QMA6100P_BW_OTHER = 8
}qma6100p_bw;

/* 设置加速度寄存器 */
typedef enum
{
    QMA6100P_RANGE_2G = 0x01,
    QMA6100P_RANGE_4G = 0x02,
    QMA6100P_RANGE_8G = 0x04,
    QMA6100P_RANGE_16G = 0x08,
    QMA6100P_RANGE_32G = 0x0f
}qma6100p_range;

/* 设置复位寄存器 */
typedef enum
{
    QMA6100P_RESET = 0xB6,
    QMA6100P_RESET_END = 0x00,
}qma6100p_reset;

/* 设置中断 */
typedef enum
{
    QMA6100P_MAP_INT1,
    QMA6100P_MAP_INT2,
    QMA6100P_MAP_INT_NONE
}qma6100p_int_map;

/* 设置管理寄存器 */
typedef enum
{
    QMA6100P_ACTIVE = 0x80,
    QMA6100P_ACTIVE_DIGITAL = 0x84,
    QMA6100P_STANDBY = 0x00,
}qma6100p_power;

typedef enum
{
    QMA6100P_MCLK_102_4K = 0x03,
    QMA6100P_MCLK_51_2K = 0x04,
    QMA6100P_MCLK_25_6K = 0x05,
    QMA6100P_MCLK_12_8K = 0x06,
    QMA6100P_MCLK_6_4K = 0x07,
    QMA6100P_MCLK_RESERVED = 0xff
}qma6100p_mclk;

typedef enum
{
    QMA6100P_SENSITITY_2G = 244,
    QMA6100P_SENSITITY_4G = 488,
    QMA6100P_SENSITITY_8G = 977,
    QMA6100P_SENSITITY_16G = 1950,
    QMA6100P_SENSITITY_32G = 3910
}qma6100p_sensitity;

#endif  /*  __QMA6100P_H__ */

