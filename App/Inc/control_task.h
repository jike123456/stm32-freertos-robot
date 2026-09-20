/**
 ****************************************************************************************************
 * @file        control_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       控制任务接口定义
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
#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

/** 通知位：CommTask 解析到新 cmd_vel 后置位 */
#define CTRL_NOTIF_NEW_CMD          (1u << 0)

/** ControlTask 任务句柄（供 CommTask 使用） */
extern TaskHandle_t g_controlTaskHandle;

/**
 * @brief 控制任务入口
 */
void KaoYaApp_ControlTask(void *argument);

/**
 * @brief 读取当前轮速测量值（mm/s）
 */
void Control_GetWheelSpeedMmps(float *vL_mmps, float *vR_mmps);

/**
 * @brief 查询速度指令是否已经超时
 */
bool Control_IsCmdTimeout(void);

#endif
