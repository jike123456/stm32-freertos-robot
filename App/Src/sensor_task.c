/**
 ****************************************************************************************************
 * @file        sensor_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       传感器采集任务（电池 / 编码器 / IMU）+ 状态快照
 * @details
 * 本任务负责：
 *  - 初始化各类传感器（电池 ADC、编码器、IMU）
 *  - 周期性采集原始传感器数据
 *  - 做必要的单位换算与整形
 *  - 生成一份“系统状态快照（proto_status_t）”
 *  - 提供线程安全的快照读写接口，供 UplinkTask 使用
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */ 
#include "sensor_task.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "ms6dsv.h"
#include "log_task.h"
#include "battery.h"
#include "capture.h"
#include "encoder.h"
#include "control_task.h"
#include "protocol.h"
#include "monitor_task.h"
#include "uplink_proto.h"
#include "iwdg_task.h"

/* ============================================================================ */
/* 全局状态快照                                                                */
/* ============================================================================ */

#define SENSOR_TEST_FORCE_IMU_FAIL  (0u)

/**
 * @brief 系统状态快照（全局唯一）
 *
 * @note
 *  - 仅由 SensorTask 写入
 *  - 其他任务通过 SensorSnap_Get() 只读
 */
proto_status_t g_status_snap;

/* ============================================================================ */
/* 快照接口：写                                                                */
/* ============================================================================ */

/**
 * @brief   更新系统状态快照
 *
 * @param   s 指向新的状态数据
 *
 * @details
 * 使用临界区保证结构体整体写入的原子性，
 * 避免其他任务读到“撕裂数据”。
 */
void SensorSnap_Set(const proto_status_t *s)
{
    /* 防御：空指针检查 */
    if (!s) return;

    /* 进入临界区，禁止任务切换 */
    taskENTER_CRITICAL();

    /* 结构体整体拷贝 */
    g_status_snap = *s;

    /* 退出临界区 */
    taskEXIT_CRITICAL();
}

/* ============================================================================ */
/* 快照接口：读                                                                */
/* ============================================================================ */

/**
 * @brief   获取当前系统状态快照
 *
 * @param   out 输出缓冲区
 *
 * @details
 * 同样使用临界区，保证读取到的是同一时刻的一致数据。
 */
void SensorSnap_Get(proto_status_t *out)
{
	/* 防御：空指针检查 */
    if (!out) return;

    taskENTER_CRITICAL();
    *out = g_status_snap;
    taskEXIT_CRITICAL();
}

/* ============================================================================ */
/* FreeRTOS 任务：SensorTask                                                    */
/* ============================================================================ */

/**
 * @brief   传感器采集任务入口
 *
 * @details
 * 初始化各类传感器后，进入周期循环：
 *  - 采集电池电压
 *  - 采集编码器速度
 *  - 采集 IMU 数据
 *  - 单位换算 / 打包为 proto_status_t
 *  - 更新状态快照
 */
void KaoYaApp_SensorTask(void *argument) {
	
    (void)argument;

	/* ======================== 模块初始化 ======================== */

    /* 初始化电池 ADC + DMA */
    Battery_Init();

    /* 初始化编码器输入捕获 */
    EncoderCap_Init();

	uint8_t ret;
	
	/* 滤波器稳定掩码配置结构体 */
	lsm6dsv16x_filt_settling_mask_t filt_settling_mask;
	
	/* 数据就绪标志结构体 */
    lsm6dsv16x_data_ready_t drdy;
	
	/* 原始 IMU 数据（寄存器读出值） */
    int16_t data_raw_acceleration[3] = {0};
    int16_t data_raw_angular_rate[3] = {0};
    int16_t data_raw_temperature = 0;
	
	/* 转换后的物理量（float） */
    float acceleration_mg[3] = {0};
    float angular_rate_mdps[3] = {0};
    float temperature_degc = 0.0f;

	/* ======================== IMU 初始化 ======================== */
	ret = ms6dsv_init();
#if SENSOR_TEST_FORCE_IMU_FAIL
    /* 故障注入：模拟WHO_AM_I读取错误 */
    ret = MS6DSV_EID;
#endif
    if (ret != 0)
    {
		LOGERROR_T("IMU", "MS6DSV init failed, code=%u",(unsigned)ret);
		
		/* IMU 初始化失败属于致命错误，停机等待 */
        while (1){}
    }

	/* 配置加速度计和陀螺仪输出数据率（ODR = 60Hz） */
	lsm6dsv16x_xl_data_rate_set(&ms6dsv, LSM6DSV16X_ODR_AT_60Hz);
    lsm6dsv16x_gy_data_rate_set(&ms6dsv, LSM6DSV16X_ODR_AT_60Hz);
    
    /* 配置量程：±2g / ±2000dps */
    lsm6dsv16x_xl_full_scale_set(&ms6dsv, LSM6DSV16X_2g);
    lsm6dsv16x_gy_full_scale_set(&ms6dsv, LSM6DSV16X_2000dps);
    
    /* 配置滤波器稳定与低通滤波 */
    filt_settling_mask.drdy = PROPERTY_ENABLE;
    filt_settling_mask.irq_xl = PROPERTY_ENABLE;
    filt_settling_mask.irq_g = PROPERTY_ENABLE;
	
    lsm6dsv16x_filt_settling_mask_set(&ms6dsv, filt_settling_mask);
	
	/* 陀螺仪 LPF */
    lsm6dsv16x_filt_gy_lp1_set(&ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&ms6dsv, LSM6DSV16X_GY_ULTRA_LIGHT);
	
	/* 加速度计 LPF */
    lsm6dsv16x_filt_xl_lp2_set(&ms6dsv, PROPERTY_ENABLE);
    lsm6dsv16x_filt_xl_lp2_bandwidth_set(&ms6dsv, LSM6DSV16X_XL_STRONG);
	
	/* ======================== 初始化快照 ======================== */
    taskENTER_CRITICAL();
    g_status_snap.battery_mv = 0;
    g_status_snap.temp       = 0;
    g_status_snap.vx = 0; g_status_snap.vy = 0; g_status_snap.vz = 0;
    g_status_snap.gx = 0; g_status_snap.gy = 0; g_status_snap.gz = 0;
    g_status_snap.ax = 0; g_status_snap.ay = 0; g_status_snap.az = 0;
    taskEXIT_CRITICAL();
	
	/* 周期调度基准 */
	TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
		
		/* 任务心跳 */
		MON_HB_SENSOR();
		
		/* 喂狗：传感器任务存活 */
		IWDG_Heartbeat(IWDG_ID_SENSOR);

		/* ==================== 电池电压 ==================== */

		/* 获取电池电压（mV） */
		uint16_t batt_mv = Battery_VoltageMvGet();
		
		/* ==================== 车体速度（编码器） ==================== */
		float vx_f, vy_f, wz_f;
		EncoderCap_GetDiff3Axis(&vx_f, &vy_f, &wz_f);

		/* float -> int16，保持与协议一致的单位 */
		int16_t vx_mmps = (int16_t)vx_f;
		int16_t vy_mmps = (int16_t)vy_f;
		int16_t vz_mrad = (int16_t)wz_f;

		/* ==================== IMU 数据 ==================== */
		
		/* 查询 IMU 数据就绪标志 */
        lsm6dsv16x_flag_data_ready_get(&ms6dsv, &drdy);

		/* 加速度数据就绪 */
        if (drdy.drdy_xl)
        {
            lsm6dsv16x_acceleration_raw_get(&ms6dsv, data_raw_acceleration);
            acceleration_mg[0] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[0]);
            acceleration_mg[1] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[1]);
            acceleration_mg[2] = lsm6dsv16x_from_fs2_to_mg(data_raw_acceleration[2]);
        }
        
        /* 角速度数据就绪 */
        if (drdy.drdy_gy)
        {
            lsm6dsv16x_angular_rate_raw_get(&ms6dsv, data_raw_angular_rate);
            angular_rate_mdps[0] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[0]);
            angular_rate_mdps[1] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[1]);
            angular_rate_mdps[2] = lsm6dsv16x_from_fs2000_to_mdps(data_raw_angular_rate[2]);
        }
        
        /* 温度数据就绪 */
        if (drdy.drdy_temp)
        {
            lsm6dsv16x_temperature_raw_get(&ms6dsv, &data_raw_temperature);
            temperature_degc = lsm6dsv16x_from_lsb_to_celsius(data_raw_temperature);
        }

		/* ==================== 单位整形 ==================== */
        int16_t temp_x100 = (int16_t)(temperature_degc * 100.0f);

        int16_t gx_mdegps = (int16_t)(angular_rate_mdps[0]);
        int16_t gy_mdegps = (int16_t)(angular_rate_mdps[1]);
        int16_t gz_mdegps = (int16_t)(angular_rate_mdps[2]);

        int16_t ax_mg = (int16_t)acceleration_mg[0];
        int16_t ay_mg = (int16_t)acceleration_mg[1];
        int16_t az_mg = (int16_t)acceleration_mg[2];
		
		/* ==================== 组包状态快照 ==================== */
        proto_status_t st = {0};
        st.battery_mv   = batt_mv;
        st.temp         = temp_x100;

        st.vx           = vx_mmps;
        st.vy           = vy_mmps;
        st.vz           = vz_mrad;

        st.gx           = gx_mdegps;
        st.gy           = gy_mdegps;
        st.gz           = gz_mdegps;

        st.ax     		= ax_mg;
        st.ay    		= ay_mg;
        st.az     		= az_mg;
				
				/* 当前第一阶段只设置系统运行标志 */
				st.status_flags = PROTO_STATUS_FLAG_RUNNING;
				
				if (Control_IsCmdTimeout())
				{
						st.status_flags |= PROTO_STATUS_FLAG_CMD_TIMEOUT;
				}

		/* 更新全局状态快照 */
		SensorSnap_Set(&st);

#if BATTERY_KAOYATEACH_ENABLE		
		Battery_KaoYaTeaching();
#endif
#if ENCODER_KAOYATEACH_ENABLE
		Encoder_KaoYaTeaching();
		Capture_KaoYaTeaching();
#endif

#if IMU_KAOYATEACH_ENABLE		

		/* 温度*/
		LOGKAOYA_T("IMU", "【温度  】(C*100)  temp：%d", (int)temp_x100);
		/* 六轴：加速度 mg */
		LOGKAOYA_T("IMU", "【加速度】(mg   )  ax：%d ay：%d az：%d", (int)ax_mg, (int)ay_mg, (int)az_mg);
		/* 六轴：角速度 mdps */
		LOGKAOYA_T("IMU", "【角速度】(mdps )  gx：%d gy：%d gz：%d", (int)gx_mdegps, (int)gy_mdegps, (int)gz_mdegps);
				  
#endif

		/* 周期延时：当前为 1s 采样周期 */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
		
	}
}
