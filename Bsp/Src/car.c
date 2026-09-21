/**
 ****************************************************************************************************
 * @file        car.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       差速小车运动学（逆解）与车辆参数管理
 * @details
 *  本文件提供差速驱动（Differential Drive）小车的基础运动学计算，主要用于：
 *   - 将“车体坐标系速度指令”转换为“左右轮线速度”（供控制闭环/PWM 输出使用）
 *   - 统一管理差速小车关键参数（轮距、最大轮速），并提供默认参数 getter
 *
 *  功能概览：
 *   1) CarDiff_GetDefaultParam()
 *      - 返回差速车型默认参数（编译期宏配置：CAR_DIFF_WHEEL_BASE_MM 等）
 *   2) CarDiff_Inverse()
 *      - 差速逆运动学：由 (vx, wz) -> (vL, vR)
 *      - 输入单位：
 *          vx_mmps   : 车体前向线速度（mm/s）
 *          wz_mradps : 车体角速度（mrad/s）
 *      - 输出单位：
 *          vL_mmps / vR_mmps : 左/右轮线速度（mm/s）
 *
 *  运动学模型（约定：左轮为 L，右轮为 R，轮距为 wheel_base_mm）：
 *      wz(rad/s) = wz(mrad/s) / 1000
 *      vL = vx - wz * (wheel_base / 2)
 *      vR = vx + wz * (wheel_base / 2)
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       STM32F407 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "car.h"
#include "log_task.h"
/* =========================
 * 内部工具：限幅（不对外暴露）
 * ========================= */
static inline float car_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 默认参数常量（外部通过 getter 访问） */
static const CarDiffParam_t s_carDiffDefault = {
    .wheel_base_mm = CAR_DIFF_WHEEL_BASE_MM,
    .max_wheel_speed_mmps = CAR_DIFF_MAX_WHEEL_SPEED_MMPS,
};

const CarDiffParam_t* CarDiff_GetDefaultParam(void)
{
    return &s_carDiffDefault;
}

void CarDiff_Inverse(const CarDiffParam_t* p,
                     int16_t vx_mmps,
                     int16_t wz_mradps,
                     float* vL_mmps,
                     float* vR_mmps)
{
    if ((p == NULL) || (vL_mmps == NULL) || (vR_mmps == NULL)) {
        return;
    }
	
    /* 参数健壮性：负值会产生歧义，统一取正 */
    const float max_v = (p->max_wheel_speed_mmps >= 0.0f) ? p->max_wheel_speed_mmps : -p->max_wheel_speed_mmps;

    /* mrad/s -> rad/s */
    const float wz_rad_s = (float)wz_mradps / 1000.0f;
    const float half_base = p->wheel_base_mm * 0.5f;

    float vL = (float)vx_mmps - (wz_rad_s * half_base);
    float vR = (float)vx_mmps + (wz_rad_s * half_base);
	
    /* 输出限幅（保证下游执行器安全） */
    vL = car_clampf(vL, -max_v, max_v);
    vR = car_clampf(vR, -max_v, max_v);

    *vL_mmps = vL;
    *vR_mmps = vR;
}
