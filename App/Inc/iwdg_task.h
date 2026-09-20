/**
 ****************************************************************************************************
 * @file        iwdg_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       IWDG 软件监控任务接口
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
#ifndef __IWDG_TASK_H
#define __IWDG_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief 被监控任务 ID 枚举
 */
typedef enum
{
    IWDG_ID_COMM = 0,
    IWDG_ID_CONTROL,
    IWDG_ID_LOG,
    IWDG_ID_MONITOR,
    IWDG_ID_SENSOR,
    IWDG_ID_MAX
} IwdgId_t;

/**
 * @brief 初始化 IWDG 软件监控系统
 */
void IWDG_TaskInit(TaskHandle_t comm,
                   TaskHandle_t control,
                   TaskHandle_t log,
                   TaskHandle_t monitor,
                   TaskHandle_t sensor);

/**
 * @brief 任务心跳接口（各任务周期调用）
 */
void IWDG_Heartbeat(IwdgId_t id);

/**
 * @brief 启用 / 禁用某任务的心跳监控
 */
void IWDG_WatchEnable(IwdgId_t id, bool en);

/**
 * @brief 查询某任务是否正在被监控
 */
bool IWDG_IsWatchEnabled(IwdgId_t id);

/**
 * @brief IWDG 软件监控任务入口
 */
void KaoYaApp_IwdgTask(void *argument);

#endif /* __IWDG_TASK_H */
