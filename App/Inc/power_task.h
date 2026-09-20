#ifndef __POWER_TASK_H
#define __POWER_TASK_H

#include <stdint.h>
#include <stdbool.h>

uint32_t PWRDBG_GetIdleHookCnt(void);

/* FreeRTOS task entry */
void App_PowerTask(void *argument);

void Power_IdleHook(void);


#endif
