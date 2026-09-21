/**
 ****************************************************************************************************
 * @file        iwdg_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       软件心跳 + 硬件 IWDG 统一监控任务
 * @details
 * 本模块实现一种“软件多任务心跳 + 硬件独立看门狗”的监控机制：
 *
 *  - 各关键任务周期性调用 IWDG_Heartbeat(id) 上报心跳
 *  - IwdgTask 周期性检查各任务心跳是否超时
 *  - 若有任一被监控任务超时：
 *      → 停止喂硬件 IWDG
 *      → 等待 IWDG 超时触发系统复位
 *  - 若全部任务健康：
 *      → 正常刷新 IWDG
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */ 
#include "iwdg_task.h"
#include "iwdg.h"
#include "monitor_task.h"

/* CubeMX 生成的 IWDG 句柄 */
extern IWDG_HandleTypeDef hiwdg;

/* =========================
 * 配置区
 * ========================= */

/** IWDG 软件监控任务检查周期（ms） */
#define IWDG_CHECK_PERIOD_MS        (100u)

/* 各任务心跳超时阈值（ms）
 * 注意：阈值需明显大于该任务的正常运行周期
 */
#define IWDG_TIMEOUT_COMM_MS        (1000u)
#define IWDG_TIMEOUT_CONTROL_MS     (300u)
#define IWDG_TIMEOUT_LOG_MS         (1000u)
#define IWDG_TIMEOUT_MONITOR_MS     (1000u)
#define IWDG_TIMEOUT_SENSOR_MS      (6000u)

/* ============================================================================ */
/* 内部状态                                                                    */
/* ============================================================================ */

/**
 * @brief 被监控任务的 TaskHandle
 *
 * @note
 *  - 若为 NULL，表示该任务不参与监控
 */
static TaskHandle_t g_taskHandle[IWDG_ID_MAX] = {0};

/**
 * @brief 各任务最近一次心跳的 tick
 */
static uint32_t     g_lastKickTick[IWDG_ID_MAX] = {0};

/**
 * @brief 各任务允许的最大心跳间隔（tick）
 */
static uint32_t     g_timeoutTick[IWDG_ID_MAX] = {0};

/**
 * @brief 各任务监控使能标志
 *
 *  - 0：不检查该任务
 *  - 1：检查该任务
 */
static uint8_t      g_watchEnable[IWDG_ID_MAX]  = {0};   // 0=不检查，1=检查

/* ============================================================================ */
/* 内部工具函数                                                                */
/* ============================================================================ */

/**
 * @brief 毫秒转 RTOS tick
 */
static uint32_t ms_to_ticks(uint32_t ms)
{
    return (uint32_t)pdMS_TO_TICKS(ms);
}

/* ============================================================================ */
/* 对外接口：IWDG 监控初始化                                                   */
/* ============================================================================ */

/**
 * @brief   初始化 IWDG 软件监控系统
 *
 * @param   comm     CommTask 任务句柄（无则传 NULL）
 * @param   control  ControlTask 任务句柄
 * @param   log      LogTask 任务句柄
 * @param   monitor  MonitorTask 任务句柄
 * @param   sensor   SensorTask 任务句柄
 *
 * @details
 *  - 记录各任务句柄
 *  - 配置各任务的超时阈值
 *  - 初始化心跳时间为当前 tick，避免启动即误判
 *  - 配置默认的监控使能策略
 */
void IWDG_TaskInit(TaskHandle_t comm,
                   TaskHandle_t control,
                   TaskHandle_t log,
                   TaskHandle_t monitor,
				   TaskHandle_t sensor)
					                    
{
	/* 保存各任务句柄 */
    g_taskHandle[IWDG_ID_COMM]    = comm;
    g_taskHandle[IWDG_ID_CONTROL] = control;
    g_taskHandle[IWDG_ID_LOG]     = log;
    g_taskHandle[IWDG_ID_MONITOR] = monitor;
	g_taskHandle[IWDG_ID_SENSOR]  = sensor;

	/* 配置各任务心跳超时阈值（ms -> tick） */
    g_timeoutTick[IWDG_ID_COMM]    = ms_to_ticks(IWDG_TIMEOUT_COMM_MS);
    g_timeoutTick[IWDG_ID_CONTROL] = ms_to_ticks(IWDG_TIMEOUT_CONTROL_MS);
    g_timeoutTick[IWDG_ID_LOG]     = ms_to_ticks(IWDG_TIMEOUT_LOG_MS);
    g_timeoutTick[IWDG_ID_MONITOR] = ms_to_ticks(IWDG_TIMEOUT_MONITOR_MS);
	g_timeoutTick[IWDG_ID_SENSOR]  = ms_to_ticks(IWDG_TIMEOUT_SENSOR_MS);

    /* 启动阶段防误判：
     * 将所有任务最近心跳时间初始化为当前 tick
     */
    uint32_t now = (uint32_t)xTaskGetTickCount();
    for (int i = 0; i < (int)IWDG_ID_MAX; i++) 
	{
		g_lastKickTick[i] = now;
    }
	
	/* 默认监控策略：
     * - Comm / Control：运行阶段动态开关
     * - Log / Monitor / Sensor：始终监控
     */
	g_watchEnable[IWDG_ID_COMM]    = 0u;
    g_watchEnable[IWDG_ID_CONTROL] = 0u;
    g_watchEnable[IWDG_ID_LOG]     = 1u;
    g_watchEnable[IWDG_ID_MONITOR] = 1u;
	g_watchEnable[IWDG_ID_SENSOR]  = 1u;
}

/* ============================================================================ */
/* 对外接口：任务心跳                                                          */
/* ============================================================================ */

/**
 * @brief   上报任务心跳
 *
 * @param   id 任务 ID
 *
 * @details
 * 每个被监控任务应在其主循环中周期性调用该接口，
 * 用于刷新“最近一次存活时间”。
 */
void IWDG_Heartbeat(IwdgId_t id)
{
    if ((uint32_t)id >= (uint32_t)IWDG_ID_MAX) return;
	
	/* 记录当前 tick 作为最新心跳时间 */
    g_lastKickTick[id] = (uint32_t)xTaskGetTickCount();
}

/* ============================================================================ */
/* 对外接口：监控开关                                                          */
/* ============================================================================ */

/**
 * @brief   启用 / 禁用某任务的心跳监控
 *
 * @param   id 任务 ID
 * @param   en true：启用监控；false：禁用监控
 *
 * @details
 * 禁用监控常用于：
 *  - 任务长期阻塞
 *  - IDLE 状态
 *  - 启动 / 停机阶段
 */
void IWDG_WatchEnable(IwdgId_t id, bool en)
{
    if ((uint32_t)id >= (uint32_t)IWDG_ID_MAX) return;

    g_watchEnable[id] = en ? 1u : 0u;

    /* 启用监控时，立即刷新心跳
     * 防止刚启用就被历史时间判定超时
     */
    if (en) 
	{
		g_lastKickTick[id] = (uint32_t)xTaskGetTickCount();
    }
}


/**
 * @brief 查询某任务是否处于监控状态
 */
bool IWDG_IsWatchEnabled(IwdgId_t id)
{
    if ((uint32_t)id >= (uint32_t)IWDG_ID_MAX) return false;
    return (g_watchEnable[id] != 0u);
}

/* ============================================================================ */
/* 内部函数：致命异常处理                                                      */
/* ============================================================================ */

/**
 * @brief   IWDG 致命异常处理
 *
 * @details
 * 进入该函数后：
 *  - 禁止中断
 *  - 不再刷新硬件 IWDG
 *  - 等待 IWDG 超时触发系统复位
 *
 * @note
 *  这里不要依赖日志、通信等子系统，它们可能已经失效
 */

static void iwdg_panic_hold(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) { __NOP(); }  /* 空转，等待硬件复位 */
}

/* ============================================================================ */
/* FreeRTOS 任务：IwdgTask                                                      */
/* ============================================================================ */

/**
 * @brief   IWDG 软件监控任务入口
 *
 * @details
 * 周期性执行：
 *  1) 检查各被监控任务心跳是否超时
 *  2) 若有异常 → 停止喂狗
 *  3) 若全部正常 → 刷新硬件 IWDG
 */
void KaoYaApp_IwdgTask(void *argument)
{
    (void)argument;

	/* 周期调度基准 */
	TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {	
		/* 任务心跳（自身） */
		MON_HB_IWDG();
		
        uint32_t now = (uint32_t)xTaskGetTickCount();

        /* ================== 1) 检查各任务心跳 ================== */
        for (int i = 0; i < (int)IWDG_ID_MAX; i++)
        {
			/* 未注册任务，跳过 */
            if (g_taskHandle[i] == NULL) continue;
			
			/* 当前未启用监控，跳过 */
			if (g_watchEnable[i] == 0u)  continue;
			
			/* 计算距上次心跳的时间 */
            uint32_t dt = now - g_lastKickTick[i];
			
			/* 超时判定 */
            if (dt > g_timeoutTick[i])
            {
                /* 发现异常任务：进入 panic，等待 IWDG 复位 */
                iwdg_panic_hold();
            }
        }

        /* ================== 2) 全部任务健康 ================== */

        /* 刷新硬件 IWDG，看门狗继续运行 */
        HAL_IWDG_Refresh(&hiwdg);
		
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IWDG_CHECK_PERIOD_MS));
    }
}
