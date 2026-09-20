/**
 ****************************************************************************************************
 * @file        battery.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       电池电压采集模块（ADC + DMA）
 * @details
 * 本模块用于周期性采集电池电压：
 *  - 使用 ADC3 + DMA 连续采样
 *  - 通过 DMA 将多次采样结果存入缓冲区
 *  - 软件对采样结果做平均滤波，降低噪声
 *  - 最终换算得到电池电压（单位：mV）
 *  - 	TIM8 (10 Hz)
 *  - 	   ↓ TRGO
 *  - 	ADC3
 *  - 	   ↓ DMA
 *  - 	s_batteryAdcBuf[16]
 *  - 	   ↓
 *  - 	Battery_VoltageMvGet()
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */ 
#include "battery.h"
#include "adc.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "log_task.h"
/* ============================== 宏定义 ============================== */

/** ADC DMA 采样点数（用于平均滤波） */
#define BAT_ADC_BUF_LEN      16u

/** ADC 参考电压（单位：mV） */
#define BAT_VREF_MV          3300u

/** ADC 最大量化值（12bit ADC：2^12 - 1 = 4095） */
#define BAT_ADC_MAX          4095u

/* ============================ 静态变量 ============================== */

/**
 * @brief   电池电压 ADC 原始采样缓冲区
 *
 * @note
 *  - 由 DMA 自动填充
 *  - 使用 volatile 修饰，防止编译器优化
 *  - 长度为 BAT_ADC_BUF_LEN
 */
static volatile uint16_t s_batteryAdcBuf[BAT_ADC_BUF_LEN];


/* ============================ 接口函数 ============================== */

/**
 * @brief   电池电压采集模块初始化
 *
 * @details
 * 初始化流程：
 *  1. 启动定时器 TIM8
 *     - 通常用于触发 ADC 转换（TRGO）
 *  2. 启动 ADC3 的 DMA 模式
 *     - ADC 转换结果自动写入 s_batteryAdcBuf
 *     - 形成循环采样
 */
void Battery_Init(void)
{
	/* 启动定时器，用于周期性触发 ADC 转换 */
    HAL_TIM_Base_Start(&htim8);
	
	/* 启动 ADC DMA 采样 */
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)s_batteryAdcBuf, BAT_ADC_BUF_LEN);
	
#if BATTERY_KAOYATEACH_ENABLE
    /* 烤鸭教学：开启EOC中断源，这样每次转换完成都会进ADC_IRQHandler */
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);
#endif

}

/**
 * @brief   获取当前电池电压（单位：mV）
 *
 * @return  电池电压，单位 mV
 *
 * @details
 * 实现步骤：
 *  1. 对 DMA 缓冲区中的 ADC 原始数据求和
 *  2. 计算平均 ADC 值（平均滤波）
 *  3. 根据 ADC 分辨率和参考电压换算成毫伏值
 *
 * 并发安全说明：
 *  - DMA 可能在后台更新 s_batteryAdcBuf
 *  - 使用 FreeRTOS 临界区，防止读取过程中数据被修改
 */
uint16_t Battery_VoltageMvGet() 
{
	uint32_t adc_sum = 0;
	uint16_t adc_avg = 0;

	/* 进入临界区，防止 DMA 正在更新缓冲区 */
	taskENTER_CRITICAL();

	/* 对 DMA 缓冲区中的采样值求和 */
    for (uint32_t i = 0; i < BAT_ADC_BUF_LEN; i++)
    {
        adc_sum += s_batteryAdcBuf[i];
    }

	/* 退出临界区 */
    taskEXIT_CRITICAL();

	/* 计算 ADC 平均值 */
	adc_avg = (uint16_t)(adc_sum / BAT_ADC_BUF_LEN);

	/* 
     * ADC 值转换为电压（mV）
     * 电压 = ADC值 × Vref / ADC最大值
     */
	return (uint16_t)((adc_avg * BAT_VREF_MV) / BAT_ADC_MAX);
}

/* 烤鸭教学开关 */
#if BATTERY_KAOYATEACH_ENABLE
static volatile uint32_t s_adcIrqCnt  = 0;  /* EOC计数：每次转换完成 +1 */

void Battery_Teach_OnAdcIrq(void)
{
	s_adcIrqCnt++;
}
extern volatile uint32_t g_adc_irq_hit;
void Battery_KaoYaTeaching(void)
{
    uint32_t cnt;

    taskENTER_CRITICAL();
    cnt = s_adcIrqCnt;
    s_adcIrqCnt = 0;
    taskEXIT_CRITICAL();

    float mv = (float)Battery_VoltageMvGet() / 1000;
	uint16_t pct = Battery_VoltageMvGet() * 100 / BAT_VREF_MV;

//    LOGKAOYA_T("BATTERY", "【TIM-ADC-DMA】过去1s ADC采样次数为%d",(int)(10));
    LOGKAOYA_T("BATTERY", "【电压信息】电压：%fV | 电量：%d%%",(float)mv, (int)pct);          
}
#endif
