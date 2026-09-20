/**
 ****************************************************************************************************
 * @file        uplink_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       上行状态上报任务接口
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
#ifndef __UPLINK_TASK_H
#define __UPLINK_TASK_H

/**
 * @brief 上行状态上报任务入口
 *
 * @param argument FreeRTOS 任务参数（未使用可传 NULL）
 */
void KaoYaApp_UplinkTask(void *argument);

#endif /* __UPLINK_TASK_H */
