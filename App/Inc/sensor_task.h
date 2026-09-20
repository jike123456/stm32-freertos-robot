/**
 ****************************************************************************************************
 * @file        sensor_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       传感器任务接口
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
#ifndef __SENSOR_TASK_H
#define __SENSOR_TASK_H

#include "protocol.h"
	
/**
 * @brief 传感器采集任务入口
 */
void KaoYaApp_SensorTask(void *argument);

/**
 * @brief 获取当前系统状态快照（线程安全）
 */
void SensorSnap_Get(proto_status_t *out);

#endif
