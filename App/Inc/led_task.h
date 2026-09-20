/**
 ****************************************************************************************************
 * @file        led_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       LED 任务对外接口声明
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
 
#ifndef __LED_TASK_H
#define __LED_TASK_H

/**
 * @brief LED 任务入口（供 freertos.c 创建任务时调用）
 *
 * @param[in] argument 任务参数（可不使用）
 */
void KaoYaApp_LedTask(void *argument);

#endif /* __LED_TASK_H */
