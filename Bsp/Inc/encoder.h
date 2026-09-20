/**
 ****************************************************************************************************
 * @file        encoder.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       编码器仿真模块对外接口
 * @license     Copyright (c) 2025-2035, Kaoya Project
 *
 ****************************************************************************************************
 */
#ifndef __ENCODER_H
#define __ENCODER_H

/**
 * @brief   仿真编码器每圈脉冲数（Pulses Per Revolution）
 *
 * @details
 * 用于速度(mm/s)->频率(Hz)换算：
 *   hz = v * (PPR / wheel_circumference)
 */
#define SIM_ENCODER_PPR           (500.0f)

/**
 * @brief   编码器仿真初始化
 *
 * @note
 * 你当前 .c 里实现函数名是 Encoder_Init()，
 * 这里声明的是 EncoderSim_Init()，建议统一，否则会链接失败。
 */
void EncoderSim_Init(void);


/**
 * @brief   设置左右轮仿真速度（mm/s）
 *
 * @param   vL_mmps 左轮线速度 mm/s
 * @param   vR_mmps 右轮线速度 mm/s
 */
void EncoderSim_SetSpeedMmps(float vL_mmps, float vR_mmps);

void Encoder_KaoYaTeaching(void);

#endif /* __ENCODER_H */
