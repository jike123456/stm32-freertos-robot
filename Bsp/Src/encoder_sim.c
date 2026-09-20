#include "encoder_sim.h"
#include "tim.h"
#include <math.h>
#include "stm32f4xx.h"   // 为了 TIM_EGR_UG

/* =========================
 * 默认轮参数（你后面按真实参数改）
 * ========================= */
#define PI_F               (3.1415926f)
static float s_wheel_diam_mm = 300.0f;   // 轮径 mm
static float s_enc_ppr       = 500.0f;   // 每圈脉冲数(单通道)

/* 16-bit timer ARR 最大值 */
#define ARR_MAX_16         (65535u)

/* 合理输出频率范围 */
#define FREQ_MIN_HZ        (2.0f)
#define FREQ_MAX_HZ        (9000.0f)     // tick=100kHz 且 arr>=10 => ~9kHz

/* =========================
 * 速度(mm/s) -> 频率(Hz)
 * f = v * (PPR / circumference)
 * ========================= */
static float speed_to_hz(float v_mmps)
{
    float v = fabsf(v_mmps);
    float circ = PI_F * s_wheel_diam_mm;      // mm/turn
    if (circ <= 1e-6f) return 0.0f;

    float hz = v * (s_enc_ppr / circ);        // pulses/s

    if (hz < FREQ_MIN_HZ) hz = 0.0f;
    if (hz > FREQ_MAX_HZ) hz = FREQ_MAX_HZ;
    return hz;
}

/* =========================
 * 设置 PWM 频率（TIM10/11 tick=100kHz）
 * ARR = tick/hz - 1
 * 写完后用 UG 强制立即生效
 * ========================= */
static void set_pwm_hz(TIM_HandleTypeDef *htim, uint32_t ch, float hz)
{
    /* 你的 TIM10 PSC=1680-1 -> tick = 100kHz（你已验证过） */
    const uint32_t tick_hz = 100000u;

    if (hz <= 0.0f) {
        __HAL_TIM_SET_COMPARE(htim, ch, 0);
        /* 强制更新 */
        htim->Instance->EGR = TIM_EGR_UG;
        return;
    }

    uint32_t arr = (uint32_t)((float)tick_hz / hz) - 1u;

    if (arr > ARR_MAX_16) arr = ARR_MAX_16;
    if (arr < 10u)        arr = 10u;

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, ch, (arr + 1u) / 2u); // 50% duty
    __HAL_TIM_SET_COUNTER(htim, 0);

    /* 强制更新（非常关键） */
    htim->Instance->EGR = TIM_EGR_UG;
}

/* =========================
 * API
 * ========================= */
void EncoderSim_SetWheelParams(float wheel_diam_mm, float enc_ppr)
{
    if (wheel_diam_mm > 1.0f) s_wheel_diam_mm = wheel_diam_mm;
    if (enc_ppr > 1.0f)       s_enc_ppr = enc_ppr;
}

void EncoderSim_Init(void)
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);

    /* ? 强制更新 */
    htim10.Instance->EGR = TIM_EGR_UG;
    htim11.Instance->EGR = TIM_EGR_UG;
}

void EncoderSim_SetSpeedMmps(float vL_mmps, float vR_mmps)
{
    float hzL = speed_to_hz(vL_mmps);
    float hzR = speed_to_hz(vR_mmps);

    set_pwm_hz(&htim10, TIM_CHANNEL_1, hzL);
    set_pwm_hz(&htim11, TIM_CHANNEL_1, hzR);
}
