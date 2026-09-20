#include "qma6100.h"
#include "iic.h"
#include "log_task.h"

#define M_G                         9.80665f
#define M_PI                        3.14159265358979323846f     /* 如果无法链接 math.h，则可以这样临时定义 */
#define RAD_TO_DEG                  (180.0f / M_PI)             /* 0.017453292519943295 */


/**
 * @brief       连续读取qma6100p寄存器的数据
 * @param       reg_addr       : 要读取的寄存器地址
 * @param       data           : 读取的数据
 * @param       len           : 数据大小
 * @retval      错误值        ：0成功，其他值：错误
 */
uint8_t qma6100p_register_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    iic_start();
    iic_send_byte(QMA6100P_WRITE);
    iic_wait_ack();
    iic_send_byte(reg);
    iic_wait_ack();
    
    iic_start();
    iic_send_byte(QMA6100P_READ);
    iic_wait_ack();
    
    while (len)
    {
        if (len == 1)
        {
            *data = iic_read_byte(0);    /* 读数据,发送nACK */
        }
        else
        {
            *data = iic_read_byte(1);    /* 读数据,发送ACK */
        }
        len--;
        data++;
    }
    iic_stop();
    
    return 0;
}

/**
 * @brief       向qma6100p寄存器写数据
 * @param       reg_addr       : 要写入的寄存器地址
 * @param       data           : 要写入的数据
 * @retval      错误值        ：0成功，其他值：错误
 */
static uint8_t qma6100p_register_write_byte(uint8_t reg, uint8_t data)
{
    iic_start();
    iic_send_byte(QMA6100P_WRITE);
    iic_wait_ack();
    iic_send_byte(reg);
    iic_wait_ack();
    iic_send_byte(data);
    iic_wait_ack();
    iic_stop();
    HAL_Delay(1);
    
    return 0;
}

uint8_t xyz_data[6] = {0};
short raw_data[3] = {0};
float accl_data[3];
float acc_normal;

/**
 * @brief       读取三轴数据(原始数据、加速度、俯仰角和翻滚角)
 * @param       rawdata：qma6100p数据结构体
 * @retval      无
 */
void qma6100p_read_rawdata(qma6100p_rawdata_t *rawdata)
{
    qma6100p_register_read(QMA6100P_REG_XOUTL, xyz_data, 6);
    
    raw_data[0] = (short)(((xyz_data[1] << 8)) | (xyz_data[0]));
    raw_data[1] = (short)(((xyz_data[3] << 8)) | (xyz_data[2]));
    raw_data[2] = (short)(((xyz_data[5] << 8)) | (xyz_data[4]));
    
    rawdata->acc_x = (float)((raw_data[0] >> 2) * M_G) / (1024);
    rawdata->acc_y = (float)((raw_data[1] >> 2) * M_G) / (1024);
    rawdata->acc_z = (float)((raw_data[2] >> 2) * M_G) / (1024);
    
    rawdata->acc_g = sqrt(rawdata->acc_x*rawdata->acc_x + rawdata->acc_y * rawdata->acc_y + rawdata->acc_z*rawdata->acc_z);
    
    acc_normal = sqrtf(rawdata->acc_x * rawdata->acc_x + rawdata->acc_y * rawdata->acc_y + rawdata->acc_z * rawdata->acc_z);
    accl_data[0] = rawdata->acc_x / acc_normal;
    accl_data[1] = rawdata->acc_y / acc_normal;
    accl_data[2] = rawdata->acc_z / acc_normal;

    rawdata->pitch = -atan2f(rawdata->acc_x, rawdata->acc_z) * RAD_TO_DEG;
    
    acc_normal = sqrtf(accl_data[0] * accl_data[0] + accl_data[1] * accl_data[1] + accl_data[2] * accl_data[2]);
    
    rawdata->roll = asinf((accl_data[1]/acc_normal)) * RAD_TO_DEG ;
}

/**
 * @brief       配置中断函数
 * @param       int_map：中断模式
 * @param       enable：使能
 * @retval      mp_const_none：初始化成功
 */
void qma6100p_step_int_config(int int_map, int enable)
{
    uint8_t    reg_16 = 0;
    uint8_t    reg_19 = 0;
    uint8_t    reg_1b = 0;

    qma6100p_register_read(0x16, &reg_16, 1);
    qma6100p_register_read(0x19, &reg_19, 1);
    qma6100p_register_read(0x1b, &reg_1b, 1);
    
    if (enable)
    {
        reg_16 |= 0x08;
        reg_19 |= 0x08;
        reg_1b |= 0x08;
        qma6100p_register_write_byte(0x16, reg_16);
        
        if(int_map == QMA6100P_MAP_INT1)
        {
            qma6100p_register_write_byte(0x19, reg_19);
        }
        else if(int_map == QMA6100P_MAP_INT2)
        {
            qma6100p_register_write_byte(0x1b, reg_1b);
        }
    }
    else
    {
        reg_16 &= (~0x08);
        reg_19 &= (~0x08);
        reg_1b &= (~0x08);

        qma6100p_register_write_byte(0x16, reg_16);
        qma6100p_register_write_byte(0x19, reg_19);
        qma6100p_register_write_byte(0x1b, reg_1b);
    }
}

/**
 * @brief       初始化qma6100p
 * @param       无
 * @retval      0, 成功;
                1, 失败;
*/
uint8_t qma6100p_comfig(void)
{
    uint8_t id_data[2];
    qma6100p_register_read(QMA6100P_REG_CHIP_ID, id_data, 1);    /* 读取设备ID，正常是0x90 */
    
    /* qma6100p的初始化序列，请看手册“6.3 Initial sequence”章节 */
    qma6100p_register_write_byte(QMA6100P_REG_RESET, QMA6100P_RESET);
    HAL_Delay(5);
    qma6100p_register_write_byte(QMA6100P_REG_RESET, QMA6100P_RESET_END);
    HAL_Delay(10);
    
    qma6100p_register_read(QMA6100P_REG_CHIP_ID, id_data, 1);    /* 读取设备ID，正常是0x90 */
    
    qma6100p_register_write_byte(0x11, 0x80);
    qma6100p_register_write_byte(0x11, 0x84);
    qma6100p_register_write_byte(0x4a, 0x20);
    qma6100p_register_write_byte(0x56, 0x01);
    qma6100p_register_write_byte(0x5f, 0x80);
    HAL_Delay(1);
    qma6100p_register_write_byte(0x5f, 0x00);
    HAL_Delay(10);

    qma6100p_register_write_byte(QMA6100P_REG_RANGE, QMA6100P_RANGE_8G);
    qma6100p_register_write_byte(QMA6100P_REG_BW_ODR, QMA6100P_BW_100);
    qma6100p_register_write_byte(QMA6100P_REG_POWER_MANAGE, QMA6100P_MCLK_51_2K | 0x80);

    qma6100p_register_write_byte(0x21, 0x03);    /* default 0x1c, step latch mode */
    
    qma6100p_step_int_config(QMA6100P_MAP_INT1, 1);
    LOGINFO_T("qma6100", "id_data[0]:%d", id_data[0]);	
    if (id_data[0] == 0x90)
    {
//        printf("qma6100p success!!!");
        return 0;                                   /* qma6100p正常 */
    }
    else
    {
//        printf("qma6100p fail!!!");
        return 1;                                   /* qma6100p失败 */
    }
}

/**
 * @brief       qma6100p初始化
 * @param       无
 * @retval      无
 */
uint8_t qma6100p_init(void)
{
    uint8_t res = 0xFF;
    uint8_t retry = 10;
    
//    iic_init();
    LOGINFO_T("qma6100", "qma6100 111111");	
    while ((retry --) && res)   /* 多次尝试，直到 res == 0, 即等到QMA6100P有 ACK反应*/
    {
        res = qma6100p_comfig();
        HAL_Delay(20);
    }
	LOGINFO_T("qma6100", "qma6100 222222");	
    return res;
}
