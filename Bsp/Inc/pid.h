/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       car参数
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
	float kp;        /**< 比例系数 */
    float ki;        /**< 积分系数 */
    float kd;        /**< 微分系数（为 0 表示禁用 D） */

    float i_acc;     /**< 积分累加项 */
    float prev_err;  /**< 上一拍误差（用于 D 项） */

    float i_limit;   /**< 积分限幅（防 wind-up），建议 >= 0 */
    float out_limit; /**< 输出限幅，建议 >= 0（例如：对应上层逻辑输出最大值） */
} PID_t;

/**
 * @brief 初始化 PID
 * @param pid        PID 实例指针
 * @param kp         比例系数
 * @param ki         积分系数
 * @param kd         微分系数（为 0 则禁用 D）
 * @param i_limit    积分限幅（建议 >= 0）
 * @param out_limit  输出限幅（建议 >= 0）
 */
void PID_Init(PID_t* pid, float kp, float ki, float kd, float i_limit, float out_limit);

/**
 * @brief 重置 PID 内部状态（积分清零、微分历史清零）
 * @param pid PID 实例指针
 */
void PID_Reset(PID_t* pid);

/**
 * @brief PID 更新（单步计算）
 * @param pid    PID 实例指针
 * @param ref    参考值
 * @param meas   测量值
 * @param dt_s   采样周期（秒）。dt_s <= 0 时，本次计算将禁用 D 项避免除零
 * @return int16_t 输出（按 out_limit 限幅并四舍五入），不做业务层 PWM 裁剪
 */
int16_t PID_Update(PID_t* pid, float ref, float meas, float dt_s);

#endif
