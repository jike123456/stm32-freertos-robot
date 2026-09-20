#ifndef __ENCODER_SIM_H
#define __ENCODER_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

void EncoderSim_Init(void);

/* 输入左右轮线速度(mm/s)，输出左右轮编码器PWM频率 */
void EncoderSim_SetSpeedMmps(float vL_mmps, float vR_mmps);

#ifdef __cplusplus
}
#endif

#endif
