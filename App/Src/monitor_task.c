/**
 ****************************************************************************************************
 * @file        monitor_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       系统监控任务：任务注册表快照 / 栈高水位计算 / 心跳对比 / panic hook
 * @details
 * 设计目标：
 * 1) 低侵入：默认不刷日志，监控信息主要用于 LCD 展示
 * 2) 可扩展：通过注册表维护任务信息（handle + 栈总量 + alias）
 * 3) 可定位：提供高水位（HWM）计算栈使用率，LCD 可显示 worst_used%
 * 4) 可告警：异常（栈溢出/内存分配失败）进入 panic 并输出关键日志
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "monitor_task.h"
#include "log_task.h"   
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include "iwdg_task.h"

/* ============================================================
 * 心跳变量（由各任务自行递增）
 * ------------------------------------------------------------
 * 约定：每个被监控任务在其主循环里周期性 g_hb_xxx++
 * MonitorTask 用“值是否变化”判断任务是否卡住
 * ============================================================ */
volatile uint32_t g_hb_comm      = 0;  // commtask的心跳
volatile uint32_t g_hb_control   = 0;  // controltask的心跳
volatile uint32_t g_hb_led       = 0;  // ledtask的心跳
volatile uint32_t g_hb_lcd       = 0;  // lcdtask的心跳
volatile uint32_t g_hb_log       = 0;  // logtask的心跳
volatile uint32_t g_hb_iwdg      = 0;  // iwdgtask的心跳
volatile uint32_t g_hb_uplink    = 0;  // uplinktask的心跳
volatile uint32_t g_hb_sensor    = 0;  // sensortask的心跳

/* ============================================================
 * 任务注册表（外部一次性注册）
 * ============================================================ */
#define MON_MAX_TASKS  10                       // 最多支持任务数

static MonTaskInfo_t s_tasks[MON_MAX_TASKS];    // 任务信息表
static uint32_t      s_task_count = 0;          // 当前已注册任务数


/**
 * @brief 获取任务信息快照（供 LCD/其他模块读取）
 * @param out  输出数组
 * @param max  输出数组最大容量
 * @return 实际拷贝的元素个数
 *
 * @note
 * 采用临界区保护，避免与注册更新并发冲突。
 */
uint32_t Monitor_GetTaskInfoSnapshot(MonTaskInfo_t *out, uint32_t max)
{
    if (!out || max == 0u) return 0u;

    taskENTER_CRITICAL();
    uint32_t n = s_task_count;
    if (n > max) n = max;

    for (uint32_t i = 0; i < n; i++) {
        out[i] = s_tasks[i];
    }
    taskEXIT_CRITICAL();

    return n;
}

/* ============================================================
 * 栈阈值（按“最小剩余百分比 min_free%”判定）
 * ============================================================ */
#ifndef MON_ERR_MIN_FREE_PCT
#define MON_ERR_MIN_FREE_PCT   10u
#endif

#ifndef MON_WARN_MIN_FREE_PCT
#define MON_WARN_MIN_FREE_PCT  20u
#endif

/**
 * @brief 注册监控任务信息（一次性拷贝）
 * @param list  外部提供的任务信息数组
 * @param count 元素个数
 *
 * @note
 * - 本函数只做拷贝，不持有外部指针
 * - stack_words 必须与创建任务时栈大小一致（单位：word）
 * - 临界区保护：避免 Monitor 读表时并发改写
 */
void Monitor_RegisterTaskInfo(const MonTaskInfo_t *list, uint32_t count)
{
	/* 参数保护：空指针或 0 个元素，直接返回 */
    if (!list || count == 0) return;

	/* 防御：最多拷贝 MON_MAX_TASKS 个 */
    if (count > MON_MAX_TASKS) count = MON_MAX_TASKS;

	/* 进入临界区：保护 s_tasks / s_task_count 原子更新 */
    taskENTER_CRITICAL();
	
	/* 清空旧表 */
    memset(s_tasks, 0, sizeof(s_tasks));
	
	/* 拷贝新表 */
    for (uint32_t i = 0; i < count; i++) {
        s_tasks[i] = list[i];
    }
	
	/* 更新计数 */
    s_task_count = count;
	
	/* 退出临界区 */
    taskEXIT_CRITICAL();
}

/* ============================================================
 * 内部工具：获取任务栈 High Water Mark（优先用 v2）
 * ------------------------------------------------------------
 * HWM 含义：任务历史上“最小剩余栈”是多少（单位：word）
 * ============================================================ */
/**
 * @brief 获取指定任务的栈高水位（最小剩余栈）
 * @param h 任务句柄
 * @return 高水位（word）；不支持/句柄为空返回 0
 */
static UBaseType_t get_hwm(TaskHandle_t h)
{
	/* 句柄为空直接返回 0 */
    if (!h) return 0;

#if defined(INCLUDE_uxTaskGetStackHighWaterMark2) && (INCLUDE_uxTaskGetStackHighWaterMark2 == 1)
    return uxTaskGetStackHighWaterMark2(h);
#elif defined(INCLUDE_uxTaskGetStackHighWaterMark) && (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    return uxTaskGetStackHighWaterMark(h);
#else
    return 0;
#endif
}

/* ============================================================
 * 监控项：Heap（保留计算能力，但不再日常打印）
 * ============================================================ */
/**
 * @brief 读取 heap 关键指标（当前空闲/历史最小空闲）
 * @note 当前版本不打印日志，LCD 侧可直接调用 FreeRTOS API 显示
 */
static void monitor_heap(void)
{
#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
    (void)xPortGetFreeHeapSize();
    (void)xPortGetMinimumEverFreeHeapSize();
#endif
}

/* ============================================================
 * 监控项：各任务栈占用（保留计算能力，但不再日常打印）
 * ------------------------------------------------------------
 * 说明：
 * - 为避免并发读写，先将注册表拷贝到 local
 * - 计算结果主要用于你后续可能扩展（例如 LCD 更丰富展示）
 * ============================================================ */
/**
 * @brief 遍历注册任务并计算栈使用率（不打印）
 * @note 当前 LCD 侧是直接用 handle + stack_words 自己算 worst_used%
 */
static void monitor_stack_pct(void)
{
	MonTaskInfo_t local[MON_MAX_TASKS];
	
    taskENTER_CRITICAL();
    uint32_t n = s_task_count;               /* 读计数 */
    memcpy(local, s_tasks, sizeof(local));   /* 拷贝整个数组 */
    taskEXIT_CRITICAL();

	/* ---- 遍历每个注册任务并计算栈余量百分比 ---- */
    for (uint32_t i = 0; i < n; i++) {
		
        TaskHandle_t h = local[i].h;                                /* 任务句柄 */
        uint16_t total = local[i].stack_words;                      /* 栈总量（word） */
//        const char *alias = local[i].alias ? local[i].alias : "?";

		/* 参数合法性检查 */
        if (!h || total == 0) continue;

		/* 获取 High Water Mark（word） */
        UBaseType_t hwm = get_hwm(h);             
		
		/* 防御：hwm 不应大于 total，若异常则钳位 */
        if (hwm > total) hwm = total;            
		
		/* 计算百分比（全整型，避免浮点） */
        uint32_t min_free_pct  = (uint32_t)hwm * 100u / (uint32_t)total;
        uint32_t worst_used_pct = 100u - min_free_pct;

		(void)worst_used_pct;

        /* 若想在 Monitor 内部做“阈值告警”，可以在这里加开关式报警 */
        (void)MON_ERR_MIN_FREE_PCT;
        (void)MON_WARN_MIN_FREE_PCT;

    }
}


/* ============================================================
 * 监控项：心跳对比（仅更新 last，不再日常报警）
 * ------------------------------------------------------------
 * 说明：
 * - LCD 侧已经会显示 OK/STALL，这里不再重复打印
 * ============================================================ */
/**
 * @brief 更新心跳对比基准值（不打印）
 */
static void monitor_heartbeat_alarm_only(void)
{
	/* 静态变量：跨周期保存上次值 */
    static uint32_t last_comm    = 0u;
    static uint32_t last_control = 0u;
    static uint32_t last_led     = 0u;
    static uint32_t last_lcd     = 0u;
    static uint32_t last_log     = 0u;
    static uint32_t last_iwdg    = 0u;
    static uint32_t last_uplink  = 0u;
    static uint32_t last_sensor  = 0u;

    (void)last_comm;
    (void)last_control;
    (void)last_led;
    (void)last_lcd;
    (void)last_log;
    (void)last_iwdg;
    (void)last_uplink;
    (void)last_sensor;

    /* 仅更新 last_*，用于下次对比（若要加告警开关，可直接复用） */
    last_comm    = g_hb_comm;
    last_control = g_hb_control;
    last_led     = g_hb_led;
    last_lcd     = g_hb_lcd;
    last_log     = g_hb_log;
    last_iwdg    = g_hb_iwdg;
    last_uplink  = g_hb_uplink;
    last_sensor  = g_hb_sensor;
}

/* ============================================================
 * 监控项：任务列表（可选功能，默认不使用）
 * ------------------------------------------------------------
 * vTaskList 需要 FreeRTOSConfig 打开：
 * - configUSE_TRACE_FACILITY == 1
 * - configUSE_STATS_FORMATTING_FUNCTIONS == 1
 * ============================================================ */
/**
 * @brief 生成任务列表字符串（默认不打印）
 * @param seq 序号（可用于定位某次采样）
 *
 * @note
 * 如果后续确实要用日志分析，可在此处把 LOGINFO 打开。
 */
static void monitor_task_list(uint32_t seq)
{
#if (configUSE_TRACE_FACILITY == 1) && (configUSE_STATS_FORMATTING_FUNCTIONS == 1)
    /* 注意：buf 太小会截断，太大又占 RAM；按实际任务数调整 */
	static char buf[768];
	
	/* 清空缓冲区，防止残留 */
    memset(buf, 0, sizeof(buf));
	
	/* 获取任务列表字符串 */
    vTaskList(buf);
	
	/* 默认不打印：LCD 已展示核心信息 */
    /* LOGINFO_T("MON", "%s", buf); */
#else
    (void)seq;
#endif
}

/* ============================================================
 * Monitor 主任务入口
 * ------------------------------------------------------------
 * 周期：1s
 * 说明：监控逻辑仍保留，但默认不输出日志
 * ============================================================ */
/**
 * @brief MonitorTask 任务入口
 * @param argument FreeRTOS 任务参数（未使用）
 */
void KaoYaApp_MonitorTask(void *argument)
{
    (void)argument;
	TickType_t last_wake = xTaskGetTickCount();
	
	/* 序号：用于标记每次监控输出 */
    static uint32_t s_mon_seq = 0;

    for (;;) {
		
		/* 固定周期 1000ms */
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
		
		/* 1) 堆 */
        monitor_heap();

		/* 2) 栈 */
        monitor_stack_pct();
		
		/* 3) 心跳（只报警） */
        monitor_heartbeat_alarm_only();

		/* 4) 任务列表 */
        if ((s_mon_seq % 10u) == 0u) {
            monitor_task_list(s_mon_seq);
        }

		/* 序号递增 */
        s_mon_seq++;
#if LOG_KAOYATEACH_ENABLE
		LOGKAOYA_T("MON", "以下是ERROR日志");
		LOGERROR_T("MON", "烤鸭的嵌入式校招项目");		
		LOGKAOYA_T("MON", "以下是WARN 日志");
		LOGWARN_T( "MON", "烤鸭的嵌入式校招项目");
		LOGKAOYA_T("MON", "以下是INFO 日志");
		LOGINFO_T( "MON", "烤鸭的嵌入式校招项目");		
		LOGKAOYA_T("MON", "以下是DEBUG日志");
		LOGDEBUG_T("MON", "烤鸭的嵌入式校招项目");
#endif
    }
}

/* ============================================================
 * Hook：栈溢出处理（必须保留）
 * ============================================================ */
/**
 * @brief 栈溢出 Hook 转发处理
 * @param xTask     溢出的任务句柄
 * @param task_name 溢出的任务名
 *
 * @note
 * 栈溢出后系统可能已经不稳定：不要做复杂动作，进入 panic 并停机。
 */
void Monitor_OnStackOverflow(TaskHandle_t xTask, const signed char *task_name)
{
    (void)xTask;
	
	/* 进入 panic（日志/输出保护机制） */
	Log_EnterPanic();
	
    /* 栈溢出时系统可能不稳定：少做事 */
    if (task_name) 
	{
        LOGERROR_T("MON", "任务：%s的栈溢出，请重新分配栈空间！", (const char *)task_name);
    } 
	else
	{		
        LOGERROR_T("MON", "STACK OVERFLOW: (null)");
    }
	
    taskDISABLE_INTERRUPTS();
    for(;;) {}
}


/* ============================================================
 * Hook：malloc 失败处理（必须保留）
 * ============================================================ */
/**
 * @brief malloc 失败 Hook 转发处理
 * @note 内存分配失败往往是致命问题：进入 panic 并停机。
 */
void Monitor_OnMallocFailed(void)
{
	/* 进入 panic 模式 */
	Log_EnterPanic();
	
	/* 保留关键错误日志 */
    LOGERROR_T("MON", "MALLOC FAILED!");
	
	/* 关中断并死循环 */
    taskDISABLE_INTERRUPTS();
    for(;;) {}
}
