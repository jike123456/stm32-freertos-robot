/**
 ****************************************************************************************************
 * @file        motor.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       电机执行器模块实现（方向GPIO + PWM输出 + 可选仿真反馈）
 * @details
 *  本模块封装“小车左右轮电机”的最底层执行器控制，目标是把上层控制输出
 *  （左右轮 PWM 指令）稳定、可控、可扩展地映射到硬件：
 *
 *  功能概览：
 *   1) PWM 输出（TIM3 CH1/CH2）
 *      - 逻辑 PWM 统一尺度：[-1000, +1000]
 *      - 硬件 CCR 映射：|pwm| -> CCR(0..ARR)，ARR=4199（可根据定时器配置调整）
 *   2) 方向控制（两线制 IN1/IN2）
 *      - 左轮：PB4=IN1, PB5=IN2
 *      - 右轮：PB6=IN1, PB7=IN2
 *      - 正转/反转/停机均由 GPIO 组合决定（与电机驱动芯片两线方向逻辑兼容）
 *   3) 停机策略（可配置）
 *      - COAST（滑行停止）：IN1=0 IN2=0
 *      - BRAKE（刹车停止）：IN1=1 IN2=1
 *      - 通过 MOTOR_CFG_STOP_MODE 选择，便于适配不同底盘/驱动器特性
 *   4) 可选仿真轮速反馈（用于无硬件/联调/算法调参）
 *      - MOTOR_CFG_USE_SIM：内部一阶惯性模型生成轮速测量值
 *      - MOTOR_CFG_SYNC_ENC_SIM：可将仿真轮速同步到 EncoderSim（便于其它模块复用）
 *
 *  仿真模型说明（启用 MOTOR_CFG_USE_SIM 时）：
 *      - PWM -> 期望轮速：线性映射（PWM=±1000 对应 ±MOTOR_CFG_SIM_MAX_SPEED_MMPS）
 *      - 动态：一阶惯性滤波（时间常数 tau=MOTOR_CFG_SIM_TAU_SEC）
 *      - 公式：v += alpha * (v_cmd - v)，alpha = dt / (tau + dt)
 *
 *  接口说明：
 *   - Motor_Init()
 *      启动 PWM 输出并将执行器置零（停机），同时初始化仿真状态（若启用）
 *   - Motor_ApplyPwm(pwmL, pwmR)
 *      将左右轮 PWM 指令限幅后下发到硬件（方向 + CCR）
 *   - MotorSim_Tick(pwmL, pwmR, dt_s)
 *      若启用仿真：根据 PWM 与 dt 更新“测量轮速”，并可选同步到 EncoderSim
 *   - Motor_GetWheelSpeedMmps(&vL, &vR)
 *      获取当前轮速测量值（默认来自仿真；后续可切真实编码器测量）
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "motor.h"

#include "tim.h"
#include "gpio.h"

#include <stdbool.h>
#include <stddef.h>
#include "log_task.h"
/* ============================================================
 * 编译开关（默认开启仿真）
 * - MOTOR_CFG_USE_SIM:      是否启用内部仿真轮速反馈
 * - MOTOR_CFG_SYNC_ENC_SIM: 是否同步 EncoderSim_SetSpeedMmps()
 * ============================================================ */
#ifndef MOTOR_CFG_USE_SIM
#define MOTOR_CFG_USE_SIM              (1)
#endif

#ifndef MOTOR_CFG_SYNC_ENC_SIM
#define MOTOR_CFG_SYNC_ENC_SIM         (1)
#endif

#if (MOTOR_CFG_USE_SIM || MOTOR_CFG_SYNC_ENC_SIM)
#include "encoder.h"
#endif

/* ============================================================
 * 模块配置参数
 * ============================================================ */

/* 逻辑 PWM 范围（本模块输入/输出的统一尺度） */
#define MOTOR_CFG_PWM_MAX              (1000)
#define MOTOR_CFG_PWM_MIN              (-1000)

/* TIM3 ARR = 4199，对应 CCR 0..4199 */
#define MOTOR_CFG_TIM_PWM_ARR          (4199u)
#define MOTOR_CFG_PWM_CMD_MAX          (1000)

/* PWM 硬件映射：TIM3 CH1/CH2 */
#define MOTOR_CFG_PWM_TIM              htim3
#define MOTOR_CFG_PWM_CH_L             TIM_CHANNEL_1
#define MOTOR_CFG_PWM_CH_R             TIM_CHANNEL_2

/* 方向引脚：PB4~PB7
 * 左轮：PB4=IN1, PB5=IN2
 * 右轮：PB6=IN1, PB7=IN2
 */
#define MOTOR_CFG_DIR_PORT             GPIOB
#define MOTOR_CFG_L_IN1_PIN            GPIO_PIN_4
#define MOTOR_CFG_L_IN2_PIN            GPIO_PIN_5
#define MOTOR_CFG_R_IN1_PIN            GPIO_PIN_6
#define MOTOR_CFG_R_IN2_PIN            GPIO_PIN_7

/* 停机策略：
 * - COAST：IN1=0 IN2=0（滑行停止）
 * - BRAKE：IN1=1 IN2=1（刹车停止）
 */
#define MOTOR_STOP_MODE_COAST          (0)
#define MOTOR_STOP_MODE_BRAKE          (1)
#ifndef MOTOR_CFG_STOP_MODE
#define MOTOR_CFG_STOP_MODE            MOTOR_STOP_MODE_COAST
#endif

/* 仿真电机模型参数（仅在启用仿真时使用） */
#define MOTOR_CFG_SIM_MAX_SPEED_MMPS   (2000.0f)   /* PWM=1000 -> 2000mm/s */
#define MOTOR_CFG_SIM_TAU_SEC          (0.20f)     /* 一阶惯性时间常数 */

/* ============================================================
 * 内部状态（轮速测量值，默认来自仿真）
 * ============================================================ */
static float s_motor_vL_meas_mmps = 0.0f;
static float s_motor_vR_meas_mmps = 0.0f;

/* 当前实际下发到硬件的 PWM（用于 hold / 仿真输入等） */
static int16_t s_pwmL_out = 0;
static int16_t s_pwmR_out = 0;

/* ============================================================
 * 内部工具函数
 * ============================================================ */
static inline int16_t motor_clampi16(int32_t x, int16_t lo, int16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return (int16_t)x;
}


/* 左轮：方向两线制 */
static inline void motor_apply_dir_L(int16_t pwmL)
{
    if (pwmL > 0) {
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN2_PIN, GPIO_PIN_RESET);
    } else if (pwmL < 0) {
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN2_PIN, GPIO_PIN_SET);
    } else {
#if (MOTOR_CFG_STOP_MODE == MOTOR_STOP_MODE_COAST)
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN2_PIN, GPIO_PIN_RESET);
#else
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_L_IN2_PIN, GPIO_PIN_SET);
#endif
    }
}

/* 右轮：方向两线制 */
static inline void motor_apply_dir_R(int16_t pwmR)
{
    if (pwmR > 0) {
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN2_PIN, GPIO_PIN_RESET);
    } else if (pwmR < 0) {
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN2_PIN, GPIO_PIN_SET);
    } else {
#if (MOTOR_CFG_STOP_MODE == MOTOR_STOP_MODE_COAST)
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN2_PIN, GPIO_PIN_RESET);
#else
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_CFG_DIR_PORT, MOTOR_CFG_R_IN2_PIN, GPIO_PIN_SET);
#endif
    }
}

/* PWM 幅值映射：|pwm_cmd| -> CCR(0..ARR) */
static inline uint32_t motor_pwm_to_ccr(int16_t pwm_cmd)
{
    if (pwm_cmd < 0) pwm_cmd = (int16_t)(-pwm_cmd);

    if (pwm_cmd > MOTOR_CFG_PWM_CMD_MAX) {
        pwm_cmd = MOTOR_CFG_PWM_CMD_MAX;
    }

    return (uint32_t)pwm_cmd * MOTOR_CFG_TIM_PWM_ARR / MOTOR_CFG_PWM_CMD_MAX;
}

/* 左轮：仅更新左轮硬件（GPIO方向 + CCR） */
static inline void motor_hw_apply_L(int16_t pwmL)
{
    motor_apply_dir_L(pwmL);
    __HAL_TIM_SET_COMPARE(&MOTOR_CFG_PWM_TIM, MOTOR_CFG_PWM_CH_L, motor_pwm_to_ccr(pwmL));
}

/* 右轮：仅更新右轮硬件（GPIO方向 + CCR） */
static inline void motor_hw_apply_R(int16_t pwmR)
{
    motor_apply_dir_R(pwmR);
    __HAL_TIM_SET_COMPARE(&MOTOR_CFG_PWM_TIM, MOTOR_CFG_PWM_CH_R, motor_pwm_to_ccr(pwmR));
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void Motor_Init(void)
{
    /* 启动 PWM */
    HAL_TIM_PWM_Start(&MOTOR_CFG_PWM_TIM, MOTOR_CFG_PWM_CH_L);
    HAL_TIM_PWM_Start(&MOTOR_CFG_PWM_TIM, MOTOR_CFG_PWM_CH_R);

    /* 清输出（停机） */
    s_pwmL_out = 0;
    s_pwmR_out = 0;
    motor_hw_apply_L(0);
    motor_hw_apply_R(0);

#if (MOTOR_CFG_USE_SIM || MOTOR_CFG_SYNC_ENC_SIM)
    /* 启动 EncoderSim（若工程启用仿真相关功能） */
    EncoderSim_Init();
#endif

    /* 清仿真状态 */
    s_motor_vL_meas_mmps = 0.0f;
    s_motor_vR_meas_mmps = 0.0f;
}

/* 新增：只更新左轮 PWM */
void Motor_ApplyPwmL(int16_t pwmL)
{
    pwmL = motor_clampi16((int32_t)pwmL, MOTOR_CFG_PWM_MIN, MOTOR_CFG_PWM_MAX);
    s_pwmL_out = pwmL;
    motor_hw_apply_L(pwmL);
}

/* 新增：只更新右轮 PWM */
void Motor_ApplyPwmR(int16_t pwmR)
{
    pwmR = motor_clampi16((int32_t)pwmR, MOTOR_CFG_PWM_MIN, MOTOR_CFG_PWM_MAX);
    s_pwmR_out = pwmR;
    motor_hw_apply_R(pwmR);
}
/* 兼容旧接口：同时更新左右轮 */
void Motor_ApplyPwm(int16_t pwmL, int16_t pwmR)
{
    Motor_ApplyPwmL(pwmL);
    Motor_ApplyPwmR(pwmR);
}
void MotorSim_Tick(float dt_s)
{
#if MOTOR_CFG_USE_SIM
    /* 只要启用仿真，内部测量速度就由仿真更新 */
	int16_t inL = s_pwmL_out;
    int16_t inR = s_pwmR_out;
	
    /* PWM -> 期望速度（线性映射） */
    const float vL_cmd = ((float)inL / (float)MOTOR_CFG_PWM_MAX) * MOTOR_CFG_SIM_MAX_SPEED_MMPS;
    const float vR_cmd = ((float)inR / (float)MOTOR_CFG_PWM_MAX) * MOTOR_CFG_SIM_MAX_SPEED_MMPS;

    /* 一阶惯性：alpha = dt / (tau + dt) */
    float alpha = 0.0f;
    if (dt_s > 0.0f) {
        alpha = dt_s / (MOTOR_CFG_SIM_TAU_SEC + dt_s);
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
    }

    s_motor_vL_meas_mmps += alpha * (vL_cmd - s_motor_vL_meas_mmps);
    s_motor_vR_meas_mmps += alpha * (vR_cmd - s_motor_vR_meas_mmps);
#else
    /* 未启用仿真：保持测量值不变（未来可切真实编码器填充） */
    (void)dt_s;
#endif

    /* 同步模拟编码器速度（便于其它模块复用） */
    EncoderSim_SetSpeedMmps(s_motor_vL_meas_mmps, s_motor_vR_meas_mmps);
}

void Motor_GetWheelSpeedMmps(float *vL, float *vR)
{
    if (vL) *vL = s_motor_vL_meas_mmps;
    if (vR) *vR = s_motor_vR_meas_mmps;
}
