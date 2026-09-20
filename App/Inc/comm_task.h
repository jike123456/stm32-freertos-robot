/**
 ****************************************************************************************************
 * @file        comm_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信任务模块接口定义
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

#ifndef COMM_TASK_H
#define COMM_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * 数据结构：速度指令（上位机下发，CRC 校验通过后的最新有效值）
 * ----------------------------------------------------------------------------
 * 单位约定：
 *  - vx_mmps   : mm/s
 *  - vy_mmps   : mm/s（差速底盘一般为 0，可保留扩展）
 *  - wz_mradps : mrad/s
 * ----------------------------------------------------------------------------
 * 字段说明：
 *  - rx_tick : 最近一次收到“有效帧”的系统 tick（xTaskGetTickCount()）
 *  - valid   : 是否曾经收到过有效帧（用于启动阶段/断链判断）
 * ========================================================================== */
typedef struct
{
    int16_t  vx_mmps;
    int16_t  vy_mmps;
    int16_t  wz_mradps;

    uint32_t rx_tick;
    bool     valid;
} CmdVel_t;


/* 初始化通信模块 */
void KaoYaApp_CommInit(void);


/* CommTask 任务入口 */
void KaoYaApp_CommTask(void *argument);


/* UART1 IDLE 中断回调 */
void KaoYaApp_Uart1IdleCallback(void);


/* 获取最新速度指令（线程安全） */
CmdVel_t Comm_GetLatestCmdVel(void);


#endif /* COMM_TASK_H */
