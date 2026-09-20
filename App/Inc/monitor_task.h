/**
 ****************************************************************************************************
 * @file        monitor_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       MONITOR 监控任务接口
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 * @attention
 *
 *  配套内容:
 *  - 项目文档:  <飞书文档/Notion/README 路径>
 *  - 源码仓库:  <git 地址>
 *
 ****************************************************************************************************
 */
#ifndef __MONITOR_TASK_H
#define __MONITOR_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* =========================
 * 心跳计数（各任务在循环里 ++）
 * ========================= */
extern volatile uint32_t g_hb_comm;
extern volatile uint32_t g_hb_control;
extern volatile uint32_t g_hb_led;
extern volatile uint32_t g_hb_lcd;
extern volatile uint32_t g_hb_log;
extern volatile uint32_t g_hb_iwdg;
extern volatile uint32_t g_hb_uplink;
extern volatile uint32_t g_hb_sensor;

/* 各任务里用这个宏更统一（可选） */
#define MON_HB_COMM()       (g_hb_comm++)
#define MON_HB_CONTROL()    (g_hb_control++)
#define MON_HB_LED()        (g_hb_led++)
#define MON_HB_LCD()        (g_hb_lcd++)
#define MON_HB_LOG()        (g_hb_log++)
#define MON_HB_IWDG()       (g_hb_iwdg++)
#define MON_HB_UPLINK()     (g_hb_uplink++)
#define MON_HB_SENSOR()     (g_hb_sensor++)
/* =========================
 * 任务注册信息（方案B）
 * stack_words：总栈大小（words）
 *  - STM32 上 1 word = 4 bytes
 *  - stack_words = stack_size_bytes / sizeof(StackType_t)
 * ========================= */
typedef struct {
    TaskHandle_t h;
    uint16_t     stack_words;
    const char  *alias;       // "COMM"/"CTRL"...
} MonTaskInfo_t;

/* 注册监测对象（任务创建后调用一次） */
void Monitor_RegisterTaskInfo(const MonTaskInfo_t *list, uint32_t count);

/* Monitor 任务入口 */
void KaoYaApp_MonitorTask(void *argument);

/* =========================
 * 供 freertos.c 调用的“钩子处理函数”
 * freertos.c 的 hook 函数里只要转调这些即可
 * ========================= */
void Monitor_OnStackOverflow(TaskHandle_t xTask, const signed char *task_name);
void Monitor_OnMallocFailed(void);

/* 读取已注册任务信息快照（供 LCD/其他模块显示用）
 * @param out      输出数组
 * @param max      out 最大容量
 * @return         实际拷贝数量
 */
uint32_t Monitor_GetTaskInfoSnapshot(MonTaskInfo_t *out, uint32_t max);

#endif
