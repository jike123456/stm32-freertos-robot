/**
 ****************************************************************************************************
 * @file        car.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       car参数
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 */
 
#ifndef __CAR_H
#define __CAR_H

#include <stdint.h>
#include <stddef.h>

/* =========================
 * car 默认参数（统一在这里改）
 * ========================= */
#define CAR_DIFF_WHEEL_BASE_MM          (500.0f)           // 轮距
#define CAR_DIFF_MAX_WHEEL_SPEED_MMPS   (2000.0f)          // 最大线速度
#define CAR_DIFF_WHEEL_DIAMETER_MM     (300.0f)            //轮子直径
/**
 * @brief 差速底盘参数
 */
typedef struct
{
    float wheel_base_mm;            /**< 轮距：左右轮中心距离（mm） */
    float max_wheel_speed_mmps;     /**< 轮速限幅（mm/s），用于防异常指令 */
} CarDiffParam_t;

/**
 * @brief 获取默认差速底盘参数（只读）
 * @return 指向默认参数的只读指针（参数定义在 car.c 内）
 */
const CarDiffParam_t* CarDiff_GetDefaultParam(void);

/**
 * @brief 差速逆解（指令速度 -> 左右轮线速度）
 *
 * 公式：
 *  wz(mrad/s) -> wz(rad/s) = wz / 1000
 *  vL = vx - wz * B/2
 *  vR = vx + wz * B/2
 *
 * @param p        底盘参数
 * @param vx_mmps  线速度（mm/s）
 * @param wz_mradps角速度（mrad/s）
 * @param vL_mmps  输出：左轮目标速度（mm/s）
 * @param vR_mmps  输出：右轮目标速度（mm/s）
 *
 * @note 本函数会对 vL/vR 做限幅（[-max, +max]）
 */
void CarDiff_Inverse(const CarDiffParam_t* p,
                     int16_t vx_mmps,
                     int16_t wz_mradps,
                     float* vL_mmps,
                     float* vR_mmps);


#endif /* CAR_H */
