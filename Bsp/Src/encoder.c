/**
 ****************************************************************************************************
 * @file        encoder.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       编码器仿真模块（速度 mm/s -> 脉冲频率 Hz -> 定时器 PWM 方波）
 * @details
 *  本文件用于在“无真实电机/无真实编码器”的情况下，模拟左右轮编码器脉冲输出，
 *  便于联调 ControlTask / PID / 速度闭环 / 通信链路等模块。
 *
 *  核心思路：
 *   - 以轮速 v(mm/s) 为输入，结合轮径与编码器每圈脉冲数 PPR，计算目标脉冲频率 Hz
 *   - 使用 TIM10 / TIM11 输出对应频率、50% 占空比的 PWM 方波，作为“编码器脉冲”
 *
 *  速度到频率换算：
 *   - 轮周长：circ_mm = PI * D(mm)
 *   - 轮转速：rps = v / circ_mm   (圈/s)
 *   - 脉冲频率：hz = rps * PPR = v * (PPR / circ_mm)   (脉冲/s)
 *   - 注意：本仿真只输出“脉冲快慢”，不输出方向信息（负速度内部取绝对值）
 *
 *  PWM 频率配置（定时器计数时钟 tick_hz 已知）：
 *   - F_pwm = tick_hz / (ARR + 1)
 *   - ARR   = tick_hz / F_pwm - 1
 *   - 这里设置 50% 占空比：CCR = (ARR + 1) / 2
 *   - 写 UG（更新事件）确保 ARR/CCR 立即生效
 *
 *  限幅与保护：
 *   - 低速抖动抑制：hz < SIM_FREQ_MIN_HZ 视为 0（停）
 *   - 上限保护：hz > SIM_FREQ_MAX_HZ 限制到上限，避免 ARR 过小/超硬件能力
 *   - ARR 限幅：限制在 16bit 范围与 SIM_ARR_MIN 以上，保证波形质量
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "encoder.h"
#include "tim.h"
#include <math.h>
#include "car.h"
#include "log_task.h"
/* ============================== 常量/宏定义 ============================== */
#define PI_F                  (3.1415926f)

/**
 * @brief PWM 定时器“计数时钟”(tick)频率（单位：Hz）
 *
 * @details
 * 这里的含义是：TIMx 的计数器每秒加多少次（也就是计数器时钟频率）。
 * 通常由 TIM 时钟 / (PSC+1) 得到。
 *
 * 例如：若 TIMx 经过预分频后计数时钟为 100kHz，则：
 *  - ARR = 99  => 周期 = (99+1)/100k = 1ms => 1kHz
 */
#define SIM_TIM_TICK_HZ           (100000u)

/** ARR 最小值限制，防止频率过高导致 ARR 太小（占空比/分辨率变差） */
#define SIM_ARR_MIN               (10u)

/** 16-bit 定时器 ARR 最大值（高级/通用定时器多数为 16bit） */
#define SIM_ARR_MAX_16            (65535u)

/** 低于该频率认为“速度太低/不输出”，避免抖动或无意义脉冲 */
#define SIM_FREQ_MIN_HZ           (2.0f)

/** 频率上限，保护 PWM 输出不超过仿真能力或硬件限制 */
#define SIM_FREQ_MAX_HZ           (9000.0f)

/* ============================== 初始化接口 ============================== */

/**
 * @brief   编码器仿真初始化：启动 PWM 输出并清零占空比
 *
 * @details
 * 1) 启动 TIM10 CH1、TIM11 CH1 PWM
 * 2) 将 CCR 置 0，让输出处于“无脉冲”（相当于速度=0）
 * 3) 写 UG 更新事件，使 ARR/CCR 等寄存器立即生效
 *
 */
void EncoderSim_Init(void) 
{
	/* 启动左右轮 PWM 输出 */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

	/* 初始置零：不输出脉冲 */
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0u);

	/* 触发更新事件：让寄存器配置立即加载到影子寄存器 */
    htim10.Instance->EGR = TIM_EGR_UG;
    htim11.Instance->EGR = TIM_EGR_UG;
}

/* ============================== 内部函数 ============================== */

/**
 * @brief   将轮速(mm/s)转换为编码器脉冲频率(Hz)
 *
 * @param   v_mmps  轮速，单位 mm/s（可为负，内部取绝对值）
 * @return  对应脉冲频率 Hz（若低于阈值则返回 0）
 *
 * @details
 * 轮速 v(mm/s) -> 转速 rps(圈/s) -> 脉冲频率 Hz(脉冲/s)
 *
 * 轮周长：circ_mm = PI * D
 * 转速(圈/s)：rps = v / circ_mm
 * 脉冲频率：hz = rps * PPR = v * (PPR / circ_mm)
 *
 * 限幅策略：
 *  - hz < SIM_FREQ_MIN_HZ  => 当作 0（停）
 *  - hz > SIM_FREQ_MAX_HZ  => 限制到上限
 */
static float speed_to_hz(float v_mmps) 
{
	/* 取绝对值：频率只表示脉冲快慢，不表示方向 */
    float v = fabsf(v_mmps);

	/* 轮子周长(mm) = PI * 直径(mm) */
    float circ_mm = PI_F * CAR_DIFF_WHEEL_DIAMETER_MM;
    if (circ_mm <= 1e-6f) return 0.0f; /* 防止除零或参数异常 */

	/* 频率(Hz) = v(mm/s) * (PPR / circ_mm(mm)) */
    float hz = v * (SIM_ENCODER_PPR / circ_mm);

	/* 低频直接视为停止，避免低速抖动/输出无意义脉冲 */
    if (hz < SIM_FREQ_MIN_HZ) hz = 0.0f;
	
	/* 过高频率限幅，避免 ARR 过小导致失真或超出硬件能力 */
    if (hz > SIM_FREQ_MAX_HZ) hz = SIM_FREQ_MAX_HZ;

    return hz;
}

/**
 * @brief   设置指定定时器通道输出目标频率的 PWM 方波
 *
 * @param   htim     目标定时器句柄
 * @param   channel  定时器通道（如 TIM_CHANNEL_1）
 * @param   hz       目标频率（Hz）；hz<=0 时关闭输出（CCR=0）
 *
 * @details
 * PWM 频率由计数时钟与 ARR 决定：
 *   F_pwm = tick_hz / (ARR + 1)
 * 因此：
 *   ARR = tick_hz / F_pwm - 1
 *
 * 本实现输出 50% 占空比方波：
 *   CCR = (ARR+1)/2
 *
 * 关键动作：
 *  - 写 ARR（周期）
 *  - 写 CCR（占空比）
 *  - 清 CNT（从 0 开始计数）
 *  - 写 UG（更新事件）使寄存器立即生效
 */
static void pwm_set_hz(TIM_HandleTypeDef *htim, uint32_t channel, float hz) 
{
    if (htim == NULL) return;

	/* hz<=0：视为停止，直接置 CCR=0，相当于不输出脉冲 */
    if (hz <= 0.0f) {
    
        __HAL_TIM_SET_COMPARE(htim, channel, 0u);
        htim->Instance->EGR = TIM_EGR_UG; /* 立刻生效 */
        return;
    }

	/* 根据目标频率反推 ARR：ARR = tick/hz - 1 */
    uint32_t arr = (uint32_t)((float)SIM_TIM_TICK_HZ / hz) - 1u;

	/* ARR 限幅：确保在 16bit 范围内，并避免过小导致波形不稳定 */
    if (arr > SIM_ARR_MAX_16) arr = SIM_ARR_MAX_16;
    if (arr < SIM_ARR_MIN)    arr = SIM_ARR_MIN;

	/* 写入自动重装载值（决定周期） */
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
	
	/* 50% 占空比：输出接近“编码器方波脉冲” */
    __HAL_TIM_SET_COMPARE(htim, channel, (arr + 1u) / 2u);
	
//	/* 计数器清零：让新的周期从 0 开始 */
//    __HAL_TIM_SET_COUNTER(htim, 0u);

//	/* 触发更新事件：确保 ARR/CCR 立即装载生效 */
//    htim->Instance->EGR = TIM_EGR_UG;
}

/* ============================== 对外接口 ============================== */

/**
 * @brief   设置左右轮仿真速度（mm/s）
 *
 * @param   vL_mmps  左轮线速度，单位 mm/s（允许负值，内部取绝对值 -> 频率）
 * @param   vR_mmps  右轮线速度，单位 mm/s（允许负值，内部取绝对值 -> 频率）
 *
 * @details
 * 1) 将左右轮速度转换为目标脉冲频率(Hz)
 * 2) 分别配置 TIM10 / TIM11 输出对应频率的方波
 *
 */
#if ENCODER_KAOYATEACH_ENABLE	
	static float Set_vL_mmps = 0.0f;
	static float Set_vR_mmps = 0.0f;
#endif
void EncoderSim_SetSpeedMmps(float vL_mmps, float vR_mmps)
{
	/* 速度 -> 频率 */
    float hzL = speed_to_hz(vL_mmps);
    float hzR = speed_to_hz(vR_mmps);
	
	/* 配置左右轮 PWM 输出频率 */
    pwm_set_hz(&htim10, TIM_CHANNEL_1, hzL);
    pwm_set_hz(&htim11, TIM_CHANNEL_1, hzR);
	
#if ENCODER_KAOYATEACH_ENABLE	
	Set_vL_mmps = vL_mmps;
	Set_vR_mmps = vR_mmps;
#endif

}


#if ENCODER_KAOYATEACH_ENABLE

void Encoder_KaoYaTeaching(void)
{
	float hzL = speed_to_hz(Set_vL_mmps);
    float hzR = speed_to_hz(Set_vR_mmps);
    LOGKAOYA_T("ENCODER", "【编码器设置频率】Wl=%.2f Hz | Wr=%.2f Hz", hzL, hzR);
}
#endif







