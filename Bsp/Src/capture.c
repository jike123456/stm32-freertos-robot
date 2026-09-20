/**
 ****************************************************************************************************
 * @file        capture.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       编码器输入捕获测速模块（TIM 输入捕获）
 * @details
 * 本模块用于通过定时器输入捕获（Input Capture）方式测量编码器脉冲频率，
 * 并进一步换算为轮速（mm/s）及差速车体速度（vx, wz）。
 *
 * 设计思路：
 *  - 使用 TIM2 的两个通道分别捕获左右轮编码器脉冲
 *  - 每次捕获记录当前计数值，与上一次捕获值求差得到周期 dt
 *  - 根据定时器计数频率换算得到脉冲频率（Hz）
 *  - 结合编码器 PPR 和轮径换算为线速度
 *  - 若超过一定时间未捕获到新脉冲，则认为速度为 0（超时保护）
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */ 
#include "capture.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "car.h"
#include "encoder.h"
#include "log_task.h"
/* ============================== 宏定义 ============================== */

/**
 * @brief TIM 输入捕获计数时钟频率（Hz）
 *
 * @details
 * TIM2 经过预分频后，计数器每秒递增的次数。
 * 用于将“计数差值 dt”换算为真实时间：
 *   freq = tick_hz / dt
 */
#define CAP_TIM_TICK_HZ           (1000000.0f)

/**
 * @brief 编码器超时时间（ms）
 *
 * @details
 * 若超过该时间仍未捕获到新脉冲，则认为轮子已经停止，
 * 防止速度值“卡在上一次非零结果”。
 */
#define CAP_TIMEOUT_MS            (200u)

/** 圆周率（float 精度） */
#define PI_F                  (3.1415926f)

/* ============================== 静态变量 ============================== */

/**
 * @brief 上一次捕获到的计数值（左右轮）
 *
 * @note
 *  - 在中断中更新
 *  - 使用 volatile 防止编译器优化
 */
static volatile uint32_t s_lastCapL = 0;
static volatile uint32_t s_lastCapR = 0;

/**
 * @brief 当前计算得到的脉冲频率（Hz）
 *
 * @note
 *  - 在输入捕获中断中更新
 *  - 在任务中读取
 */
static volatile float    s_freqL_hz = 0.0f;
static volatile float    s_freqR_hz = 0.0f;

/**
 * @brief 最近一次成功捕获脉冲的时间戳（ms）
 *
 * @details
 * 用于超时判断，避免长时间未更新仍返回旧速度
 */
static volatile uint32_t s_lastL_ms = 0;
static volatile uint32_t s_lastR_ms = 0;

/* ============================== 初始化 ============================== */

/**
 * @brief   编码器输入捕获初始化
 *
 * @details
 * 1. 启动 TIM2 的输入捕获中断（左右轮各占一个通道）
 * 2. 初始化最近捕获时间，用于超时检测
 */
void EncoderCap_Init(void) 
{
	/* 启动 TIM2 输入捕获中断 */
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

	/* 记录当前时间，作为初始“最近捕获时间” */
    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
	
    s_lastL_ms = now_ms;
    s_lastR_ms = now_ms;
}

/* ============================== 中断回调 ============================== */

/**
 * @brief   TIM 输入捕获中断回调函数
 *
 * @details
 * 每次捕获到编码器脉冲上升沿（或配置的边沿）时触发：
 *  - 读取当前捕获值 cap
 *  - 与上一次捕获值做差，得到周期 dt
 *  - 根据计数时钟频率换算为脉冲频率 Hz
 *
 * @note
 *  - 该函数在中断上下文中执行
 *  - 不能使用会阻塞的 RTOS API
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return;
    if (htim->Instance != TIM2) return;
	
	/* 获取当前系统时间（ms），ISR 版本 */
    uint32_t now_ms = (uint32_t)xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

	/* 左轮编码器 */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
		/* 读取当前捕获值 */
        uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		
		/* 计算两次捕获间隔 dt（tick）
         * - 正常情况：cap >= last，直接相减
         * - 溢出情况：计数器从 0xFFFFFFFF 回绕到 0，需要做回绕补偿
         */
		uint32_t dt  = (cap >= s_lastCapL) ? (cap - s_lastCapL) : (0xFFFFFFFFu - s_lastCapL + cap + 1u);

		/* 更新上一次捕获值，为下一次 dt 计算做准备 */
        s_lastCapL = cap;
		
		/* 更新“最近一次捕获时间”，用于任务侧超时判断 */
        s_lastL_ms = now_ms;
		
		/* dt > 0 才能计算频率 */
        if (dt > 0u) s_freqL_hz = (float)CAP_TIM_TICK_HZ / (float)dt;
    }
	/* 右轮编码器 */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {	
        uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uint32_t dt  = (cap >= s_lastCapR) ? (cap - s_lastCapR) : (0xFFFFFFFFu - s_lastCapR + cap + 1u);

        s_lastCapR = cap;
        s_lastR_ms = now_ms;
        if (dt > 0u) s_freqR_hz = (float)CAP_TIM_TICK_HZ / (float)dt;
    }
}

/* ============================== 内部工具函数 ============================== */

/**
 * @brief   读取频率值并进行超时判断
 *
 * @param   hz       指向频率变量（Hz）
 * @param   last_ms  最近一次捕获时间（ms）
 * @return  有效频率（Hz），超时则返回 0
 *
 * @details
 * 若当前时间与最近一次捕获时间差超过 CAP_TIMEOUT_MS，
 * 则认为编码器已停止，返回 0。
 */
static float read_hz_with_timeout(volatile float *hz, volatile uint32_t *last_ms)
{
    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

	/* 若超过 CAP_TIMEOUT_MS 没有更新捕获，则认为速度为 0 */
    if ((now_ms - *last_ms) > CAP_TIMEOUT_MS)
    {
        return 0.0f;
    }
    return *hz;
}

/* ============================== 频率接口 ============================== */

/**
 * @brief 获取左轮编码器频率（Hz）
 */
float EncoderCap_GetLeftHz(void)
{
	/* 读取左轮频率，并进行超时清零处理 */
    return read_hz_with_timeout(&s_freqL_hz, &s_lastL_ms);
}

/**
 * @brief 获取右轮编码器频率（Hz）
 */
float EncoderCap_GetRightHz(void)
{
	/* 读取右轮频率，并进行超时清零处理 */
    return read_hz_with_timeout(&s_freqR_hz, &s_lastR_ms);
}

/* ============================== 速度换算 ============================== */

/**
 * @brief   将编码器脉冲频率转换为轮速（mm/s）
 *
 * @param   hz  编码器频率（Hz）
 * @return  轮速（mm/s）
 *
 * @details
 * 轮速 = 频率 × (轮周长 / 编码器 PPR)
 */
static float hz_to_speed(float hz)
{
	/* 无脉冲频率则速度为 0 */
    if (hz <= 0.0f) return 0.0f;

	/* 计算轮周长（mm）= PI * 直径(mm) */
    float circ_mm = PI_F * CAR_DIFF_WHEEL_DIAMETER_MM;
	
	/* 参数保护：防止轮径配置为 0 造成除零/异常 */
    if (circ_mm <= 1e-6f) return 0.0f;

	/* 速度(mm/s) = 频率(脉冲/s) * (每脉冲对应的行进距离 mm/脉冲)
     * 每脉冲距离 = 周长(mm) / PPR(脉冲/圈)
     */
    return hz * (circ_mm / SIM_ENCODER_PPR);
}
/**
 * @brief 获取左轮线速度（mm/s）
 */
float EncoderCap_GetLeftMmps(void)
{
	/* 左轮：先取 Hz（带超时），再转换为 mm/s */
    return hz_to_speed(EncoderCap_GetLeftHz());
}
/**
 * @brief 获取右轮线速度（mm/s）
 */
float EncoderCap_GetRightMmps(void)
{
	/* 右轮：先取 Hz（带超时），再转换为 mm/s */
    return hz_to_speed(EncoderCap_GetRightHz());
}
/* ============================== 差速运动学 ============================== */

/**
 * @brief   获取差速车体速度（三轴形式）
 *
 * @param   vx_mmps   前向速度（mm/s）
 * @param   vy_mmps   横向速度（mm/s，差速车恒为 0）
 * @param   wz_mradps 偏航角速度（mrad/s）
 *
 * @details
 * 差速模型：
 *   vx = (vL + vR) / 2
 *   wz = (vR - vL) / wheel_base
 */
void EncoderCap_GetDiff3Axis(float *vx_mmps, float *vy_mmps, float *wz_mradps)
{

	/* 获取左右轮线速度（mm/s） */
    float vL = EncoderCap_GetLeftMmps();
    float vR = EncoderCap_GetRightMmps();

	/* 差速模型：车体前向速度 = (vL + vR)/2 */
    float vx = (vL + vR) * 0.5f;
	
	/* 差速车不具备侧向速度（忽略侧滑情况下），vy 恒为 0 */
    float vy = 0.0f;

	/* 偏航角速度 wz：wz(rad/s) = (vR - vL) / wheel_base(mm) */
    float wz = 0.0f;
		
    if (CAR_DIFF_WHEEL_BASE_MM > 1e-6f)
    {
		/* rad/s -> mrad/s：乘 1000 */
        wz = (vR - vL) / CAR_DIFF_WHEEL_BASE_MM * 1000.0f;
    }

	/* 输出指针判空保护：调用方可选择只取其中某一项 */
    if (vx_mmps)   *vx_mmps = vx;
    if (vy_mmps)   *vy_mmps = vy;
    if (wz_mradps) *wz_mradps = wz;
}

#if ENCODER_KAOYATEACH_ENABLE
void Capture_KaoYaTeaching(void)
{
    /* 1) 频率（带超时清零） */
    float wl_hz = EncoderCap_GetLeftHz();
    float wr_hz = EncoderCap_GetRightHz();

    /* 2) 左右轮线速度（mm/s） */
    float vl_mmps = EncoderCap_GetLeftMmps();
    float vr_mmps = EncoderCap_GetRightMmps();

    /* 3) 三轴速度（vx, vy, wz[mrad/s]） */
    float vx_mmps = 0.0f, vy_mmps = 0.0f, wz_mradps = 0.0f;
    EncoderCap_GetDiff3Axis(&vx_mmps, &vy_mmps, &wz_mradps);

    LOGKAOYA_T("ENCODER", "【编码器测得频率】Wl=%.2f Hz | Wr=%.2f Hz", wl_hz, wr_hz);
    LOGKAOYA_T("ENCODER", "【左右轮当前速度】Vl=%.2f mm/s | Vr=%.2f mm/s", vl_mmps, vr_mmps);
    LOGKAOYA_T("ENCODER", "【三轴当前速度  】Vx=%.2f mm/s | Vy=%.2f mm/s | Wz=%.2f mrad/s", vx_mmps, vy_mmps, wz_mradps);
}
#endif

