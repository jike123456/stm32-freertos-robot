#include "power_task.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "log_task.h"

//volatile uint32_t g_idle_hook_cnt = 0;

//uint32_t PWRDBG_GetIdleHookCnt(void)
//{
//    /* 32-bit 读是原子的（F4），直接读即可 */
//    return (uint32_t)g_idle_hook_cnt;
//}

//static uint32_t sleep_cnt = 0;

void PreSleepProcessing(uint32_t expectedIdleTime)
{
    (void)expectedIdleTime;
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
//	LOGKAOYA_T("Sleep", "Sleep Start !");
    HAL_SuspendTick(); // 暂停HAL tick
}

void PostSleepProcessing(uint32_t expectedIdleTime)
{
    (void)expectedIdleTime;
//	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
//	LOGKAOYA_T("Sleep", "Sleep end !");
    HAL_ResumeTick(); // 恢复HAL tick
}



