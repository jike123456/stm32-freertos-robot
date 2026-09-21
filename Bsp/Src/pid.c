/**
 ****************************************************************************************************
 * @file        protocol.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信协议
 * @details
 *  本文件实现一个轻量级 PID 控制器，面向下位机实时控制场景（如差速轮速闭环）。
 *  设计目标：可复用、可调参、默认安全（限幅保护）、便于在面试中解释“工程化控制实现”。
 *
 *  功能特性：
 *   - 基本 PID：P + I + D
 *   - 积分抗饱和：积分累加 i_acc 带限幅（i_limit）
 *   - 输出限幅：最终输出 out 带限幅（out_limit）
 *   - D 项可选：kd=0 时自动关闭微分；dt<=0 时不计算微分，避免除 0
 *   - 输出类型：float 计算，最终四舍五入输出 int16（便于直接映射到 PWM/力矩指令）
 *
 *  参数与状态：
 *   - 参数：kp / ki / kd
 *   - 状态：i_acc（积分累加器）/ prev_err（上次误差）
 *   - PID_Init() 中会将 i_limit/out_limit 的负值转为正值，避免歧义
 *
 *  使用建议：
 *   1) 控制周期 dt 必须与 ControlTask 的实际节拍一致（建议固定周期 vTaskDelayUntil）
 *   2) 速度闭环常见配置：
 *      - kd 通常可先置 0（只做 PI），稳定后再按需加入 D 项抑制超调/振荡
 *      - i_limit 需要结合执行器能力与系统误差量级设置，避免“积分顶死”
 *   3) 如需更强工程性：
 *      - 可加入积分分离、前馈、死区补偿、输出斜坡限制（限加速度）等策略
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "pid.h"
#include <math.h>
#include "log_task.h"

/* =========================
 * 内部工具函数 -> 限幅（不对外暴露）
 * ========================= */
static inline float pid_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PID_Init(PID_t* pid, float kp, float ki, float kd, float i_limit, float out_limit)
{
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->i_acc = 0.0f;
    pid->prev_err = 0.0f;

    /* 参数（负值容易引起歧义，统一转正） */
    pid->i_limit = (i_limit >= 0.0f) ? i_limit : -i_limit;
    pid->out_limit = (out_limit >= 0.0f) ? out_limit : -out_limit;
#if PID_KAOYATEACH_ENABLE
    LOGKAOYA_T("PID", "【PID参数】比例参数：%f 积分参数：%f 微分参数：%f", (double)pid->kp, (double)pid->ki, (double)pid->kd);
               
#endif	
}

void PID_Reset(PID_t* pid)
{
    if (pid == NULL) return;
    pid->i_acc = 0.0f;
    pid->prev_err = 0.0f;

}

int16_t PID_Update(PID_t* pid, float ref, float meas, float dt)
{
    if (pid == NULL) return 0;

    float err = ref - meas;

    /* P */
    float p = pid->kp * err;

    /* I（带限幅） */
    pid->i_acc += pid->ki * err * dt;
    pid->i_acc = pid_clampf(pid->i_acc, -pid->i_limit, pid->i_limit);

    /* D（可选） */
    float d = 0.0f;
    if ((pid->kd != 0.0f) && (dt > 0.0f)) {
        d = pid->kd * (err - pid->prev_err) / dt;
    }
    pid->prev_err = err;

    /* 输出合成 + 限幅 */
    float out = p + pid->i_acc + d;
    out = pid_clampf(out, -pid->out_limit, pid->out_limit);
#if PID_KAOYATEACH_ENABLE
	LOGKAOYA_T("PID", "【PID值】比例值：%f 积分值：%f 微分值：%f", (double)p, (double)pid->i_acc, (double)d);
#endif
    /* 输出：四舍五入到 int16 */
    return (int16_t)lroundf(out);
}
