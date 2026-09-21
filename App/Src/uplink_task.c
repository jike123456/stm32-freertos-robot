/**
 ****************************************************************************************************
 * @file        uplink_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       上行状态上报任务（Uplink Task）
 * @details
 * 本任务用于周期性向上位机发送系统状态信息，典型流程为：
 *
 *   SensorTask / ControlTask
 *           ↓
 *   SensorSnap_Get()（获取一次完整快照）
 *           ↓
 *   UplinkProto_SendStatus()
 *           ↓
 *   UART 发送（协议帧）
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */ 
 
#include "uplink_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "log_task.h"
#include "control_task.h"
#include "uplink_proto.h"
#include "sensor_task.h"
#include "usart.h"
#include "protocol.h"
#include "monitor_task.h"

/* ============================================================================ */
/* 上行任务配置                                                                */
/* ============================================================================ */

/** 上行周期（ms）：1s 上报·一次状态 */
#define UPLINK_PERIOD_MS      (1000u)

/* ============================================================================ */
/* FreeRTOS 任务：UplinkTask                                                   */
/* ============================================================================ */

/**
 * @brief   上行状态上报任务入口
 *
 * @details
 *  - 周期性读取系统状态快照
 *  - 通过 UplinkProto 打包为协议帧
 *  - 通过串口发送给上位机
 */

void KaoYaApp_UplinkTask(void *argument)
{
    (void)argument;

	/* 帧序号（用于上位机检测丢包/乱序） */
	uint8_t seq = 0;

	/* 初始化上行协议模块（如序号、CRC、端口等） */
    UplinkProto_Init();

	/* 上行协议帧缓存 */
	proto_uplink_t tx;
	
	/* 系统状态快照结构体 */
	proto_status_t st;
	
	/* 周期调度基准 */
	TickType_t last_wake = xTaskGetTickCount();
    for (;;)
    {
		/* 任务心跳：用于 MonitorTask 监控任务存活 */
		MON_HB_UPLINK();
		
		/* 获取一次“系统状态快照”
         * - 电池电压
         * - 温度
         * - 车体速度
         * - IMU 数据等
         * 由 SensorTask 内部统一整理，避免多任务并发读取
         */
		SensorSnap_Get(&st);
		
		/* 通过上行协议打包并发送 STATUS 帧
         * seq        : 帧序号（每次递增）
         * battery_mv : 电池电压（mV）
         * temp       : 温度
         * vx,vy,vz   : 车体速度
         * gx,gy,gz   : 角速度
         * ax,ay,az   : 加速度
         */
		UplinkProto_SendStatus(&tx, seq++,
							 st.battery_mv, st.temp,
							 st.vx, st.vy, st.vz,
							 st.gx, st.gy, st.gz,
							 st.ax, st.ay, st.az,
							 st.status_flags);
#if UPLINK_KAOYATEACH_ENABLE
		/* 注意：调用里是 seq++，所以这里用 seq-1 才是本包序号 */
		uint32_t seq_sent = (uint32_t)(seq - 1u);
		LOGKAOYA_T("UPLINK", "==================UPLINK==================");
		
		LOGKAOYA_T("UPLINK", "【上报】STATUS 已发送：seq=%lu", (unsigned long)seq_sent);

		LOGKAOYA_T("UPLINK", "【电池】BAT=%u mV", (unsigned)st.battery_mv);
	   
		LOGKAOYA_T("UPLINK", "【温度】TEMP=%d C", (int)st.temp);
				   
		LOGKAOYA_T("UPLINK", "【速度】v(mm/s)=(%d,%d,%d)",
				   (int)st.vx, (int)st.vy, (int)st.vz);

		LOGKAOYA_T("UPLINK", "【角速度】g(0.01deg/s)=(%d,%d,%d)",
				   (int)st.gx, (int)st.gy, (int)st.gz);

		LOGKAOYA_T("UPLINK", "【加速度】a(mg)=(%d,%d,%d)",
				   (int)st.ax, (int)st.ay, (int)st.az);
#endif	
		/* 周期延时，保证固定 1s 上报节拍 */
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UPLINK_PERIOD_MS));
    }
}
