/**
 ****************************************************************************************************
 * @file        motor.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       电机应用及接口
 * @license     Copyright (c) 2025-2035, Kaoya Project
 *
 ****************************************************************************************************
 */
 
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

/**
 * @brief 电机模块初始化
 * - 启动 PWM 输出
 * - 输出清零（停机）
 * - 初始化仿真状态
 * - （可选）初始化 EncoderSim
 */
void Motor_Init(void);

/**
 * @brief 应用电机 PWM（支持正负）
 * @param pwmL 左轮逻辑 PWM（正：正转；负：反转；0：停机）
 * @param pwmR 右轮逻辑 PWM（正：正转；负：反转；0：停机）
 *
 * @note 内部会完成：
 * - PWM 限幅保护
 * - 方向 GPIO 设置
 * - PWM 幅值映射并写入 TIM CCR
 */
void Motor_ApplyPwm(int16_t pwmL, int16_t pwmR);

/**
 * @brief 仿真步进（PWM -> 轮速测量值 mm/s）
 * @param pwmL   左轮逻辑 PWM
 * @param pwmR   右轮逻辑 PWM
 * @param dt_s   采样周期（秒）
 *
 * @note
 * - 若关闭仿真（编译开关），该函数将变为 no-op
 * - 若开启 EncoderSim 同步，则内部会调用 EncoderSim_SetSpeedMmps()
 */
void MotorSim_Tick(float dt_sec);

/**
 * @brief 获取轮速测量值（mm/s）
 * @param vL 输出：左轮测量速度
 * @param vR 输出：右轮测量速度
 *
 * @note 当前测量速度来自内部仿真模型；未来可替换为真实编码器
 */
void Motor_GetWheelSpeedMmps(float *vL, float *vR);

void Motor_ApplyPwmL(int16_t pwmL);
void Motor_ApplyPwmR(int16_t pwmR);


#endif /* MOTOR_H */
