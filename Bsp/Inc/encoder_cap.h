#ifndef __ENCODER_CAP_H
#define __ENCODER_CAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 初始化：启动 TIM2 CH1/CH2 输入捕获中断 */
void EncoderCap_Init(void);

/* 频率输出 */
float EncoderCap_GetLeftHz(void);
float EncoderCap_GetRightHz(void);

/* 速度输出（mm/s） */
float EncoderCap_GetLeftMmps(void);
float EncoderCap_GetRightMmps(void);


#ifdef __cplusplus
}
#endif

#endif
