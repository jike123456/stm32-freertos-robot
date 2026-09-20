#include "encoder_cap.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "car.h"

#define TIM2_TICK_HZ       (1000000.0f)   // TIM2 PSC=84-1 -> 1MHz
#define ENC_TIMEOUT_MS     (200u)

#define PI_F               (3.1415926f)
//static float s_wheel_diam_mm = 300.0f;
static float s_enc_ppr       = 500.0f;

static volatile uint32_t s_lastCapL = 0;
static volatile uint32_t s_lastCapR = 0;
static volatile float    s_fL = 0.0f;
static volatile float    s_fR = 0.0f;
static volatile uint32_t s_lastL_ms = 0;
static volatile uint32_t s_lastR_ms = 0;

void EncoderCap_Init(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

    uint32_t now = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_lastL_ms = now;
    s_lastR_ms = now;
}

/* 注意：HAL_TIM_IC_CaptureCallback 全工程只能定义一次！
 * 你现在没有其他地方实现它，就直接用这份即可。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    uint32_t now_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t dt  = (cap >= s_lastCapL) ? (cap - s_lastCapL)
                                           : (0xFFFFFFFFu - s_lastCapL + cap + 1u);
        s_lastCapL = cap;
        s_lastL_ms = now_ms;
        if (dt > 0) s_fL = (float)TIM2_TICK_HZ / (float)dt;
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uint32_t dt  = (cap >= s_lastCapR) ? (cap - s_lastCapR)
                                           : (0xFFFFFFFFu - s_lastCapR + cap + 1u);
        s_lastCapR = cap;
        s_lastR_ms = now_ms;
        if (dt > 0) s_fR = (float)TIM2_TICK_HZ / (float)dt;
    }
}

static float get_with_timeout(volatile float *f, volatile uint32_t *last_ms)
{
    uint32_t now = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if ((now - *last_ms) > ENC_TIMEOUT_MS) return 0.0f;
    return *f;
}

float EncoderCap_GetLeftHz(void)  { return get_with_timeout(&s_fL, &s_lastL_ms); }
float EncoderCap_GetRightHz(void) { return get_with_timeout(&s_fR, &s_lastR_ms); }

static float hz_to_mmps(float hz)
{
    if (hz <= 0.0f) return 0.0f;
    float circ = PI_F * CAR_DIFF_WHEEL_BASE_MM;
    if (circ <= 1e-6f) return 0.0f;
    return hz * (circ / s_enc_ppr);
}

float EncoderCap_GetLeftMmps(void)  { return hz_to_mmps(EncoderCap_GetLeftHz()); }
float EncoderCap_GetRightMmps(void) { return hz_to_mmps(EncoderCap_GetRightHz()); }
