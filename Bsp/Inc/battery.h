/**
 ****************************************************************************************************
 * @file        battery.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       电池电压采集模块接口定义
 * @license     Copyright (c) 2025-2035, Kaoya Project
 *
 ****************************************************************************************************
 */
#ifndef __BATTERY_H
#define __BATTERY_H

#include <stdint.h>
#include "log_task.h"

/**
 * @brief   初始化电池电压采集模块
 *
 * @note
 *  - 启动定时器与 ADC DMA
 *  - 一般在系统初始化阶段调用一次
 */
void Battery_Init(void);

/**
 * @brief   获取当前电池电压
 *
 * @return  电池电压（单位：mV）
 *
 * @note
 *  - 内部已进行平均滤波
 *  - 可在任务中周期性调用
 */
uint16_t Battery_VoltageMvGet(void);

#if BATTERY_KAOYATEACH_ENABLE
/* 在 ADC_IRQHandler(USER CODE) 里调用：统计每次EOC */
void Battery_Teach_OnAdcIrq(void);

/* 每500ms打印一次：显示这段时间EOC次数 */
void Battery_KaoYaTeaching(void);
#endif

#endif /* __BATTERY_H */
