/**
 ****************************************************************************************************
 * @file        lcd_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       LCD 显示任务接口
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
#ifndef __LCD_TASK_H
#define __LCD_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LCD 任务入口（用于创建 FreeRTOS 任务）
 * @param argument 任务参数（可不使用）
 */
void KaoYaApp_LcdTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_TASK_H */
