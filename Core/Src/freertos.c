/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Author             : Kaoya (烤鸭)
  * Version            : V1.0
  * Date               : 2026-02-04
  * Description        : Code for freertos applications
  ******************************************************************************
  * @details
  *  本文件基于 CubeMX 生成的 CMSIS-OS2 FreeRTOS 模板，承载“系统级装配”职责：
  *   1) 创建各业务任务（osThreadNew）
  *   2) 绑定项目任务入口到 KaoYaApp_*（统一入口、便于分层/单测/复用）
  *   3) 注册 Monitor 监控信息（任务句柄、栈大小、别名）
  *   4) 初始化 IWDG 看门狗监控对象（喂狗任务列表）
  *   5) 提供 FreeRTOS Hook：
  *      - vApplicationTickHook：Tick 计数/轻量统计（ISR 上下文）
  *      - vApplicationIdleHook：空闲钩子（可用于低功耗/空闲率统计等）
  *      - vApplicationStackOverflowHook：栈溢出回调 -> Monitor_OnStackOverflow()
  *      - vApplicationMallocFailedHook：堆分配失败回调 -> Monitor_OnMallocFailed()
  *   6) Tickless Idle 支持：
  *      - PreSleepProcessing/PostSleepProcessing：睡眠前后对 HAL Tick 进行挂起/恢复
  *
  *  任务关系概览（本工程）：
  *   - CommTask    : 串口 DMA/IDLE 接收 + 协议解析 + 下发命令
  *   - ControlTask : 运动学/闭环控制（PID）-> Motor 执行
  *   - SensorTask  : 传感器采集（电压/编码器/IMU 等）
  *   - UplinkTask  : 状态上报（电压/温度/姿态/速度等）
  *   - LogTask     : 异步日志输出（避免业务任务刷屏阻塞）
  *   - LcdTask     : 屏幕 UI 刷新
  *   - MonitorTask : 系统健康监控（栈水位、堆余量、任务心跳等）
  *   - IwdgTask    : 独立看门狗喂狗/复位保障（配合任务心跳）
  *   - LedTask     : 指示灯状态机（心跳/告警等）
* @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_init.h"
#include "comm_task.h"
#include "led_task.h"
#include "control_task.h"
#include "log_task.h"
#include "lcd_task.h"
#include "monitor_task.h"
#include "iwdg_task.h"
#include "uplink_task.h"
#include "sensor_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint32_t systick_cnt = 0;
/* USER CODE END Variables */
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for monitorTask */
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for iwdgTask */
osThreadId_t iwdgTaskHandle;
const osThreadAttr_t iwdgTask_attributes = {
  .name = "iwdgTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for uplinkTask */
osThreadId_t uplinkTaskHandle;
const osThreadAttr_t uplinkTask_attributes = {
  .name = "uplinkTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void CommTask(void *argument);
void LedTask(void *argument);
void ControlTask(void *argument);
void LogTask(void *argument);
void LcdTask(void *argument);
void MonitorTask(void *argument);
void IwdgTask(void *argument);
void UplinkTask(void *argument);
void SensorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
	systick_cnt++;
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	Monitor_OnStackOverflow(xTask, pcTaskName);
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{	
	Monitor_OnMallocFailed();
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
	HAL_SuspendTick();
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
	HAL_ResumeTick();
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	KaoYaApp_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of commTask */
  commTaskHandle = osThreadNew(CommTask, NULL, &commTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(LogTask, NULL, &logTask_attributes);

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(LcdTask, NULL, &lcdTask_attributes);

  /* creation of monitorTask */
  monitorTaskHandle = osThreadNew(MonitorTask, NULL, &monitorTask_attributes);

  /* creation of iwdgTask */
  iwdgTaskHandle = osThreadNew(IwdgTask, NULL, &iwdgTask_attributes);

  /* creation of uplinkTask */
  uplinkTaskHandle = osThreadNew(UplinkTask, NULL, &uplinkTask_attributes);

  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(SensorTask, NULL, &sensorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	MonTaskInfo_t mon_list[9];

	mon_list[0].h = (TaskHandle_t)commTaskHandle;
	mon_list[0].stack_words = (uint16_t)(commTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[0].alias = "COMM";

	mon_list[1].h = (TaskHandle_t)controlTaskHandle;
	mon_list[1].stack_words = (uint16_t)(controlTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[1].alias = "CTRL";

	mon_list[2].h = (TaskHandle_t)ledTaskHandle;
	mon_list[2].stack_words = (uint16_t)(ledTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[2].alias = "LED";

	mon_list[3].h = (TaskHandle_t)lcdTaskHandle;
	mon_list[3].stack_words = (uint16_t)(lcdTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[3].alias = "LCD";

	mon_list[4].h = (TaskHandle_t)logTaskHandle;
	mon_list[4].stack_words = (uint16_t)(logTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[4].alias = "LOG";

	mon_list[5].h = (TaskHandle_t)monitorTaskHandle;
	mon_list[5].stack_words = (uint16_t)(monitorTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[5].alias = "MON";
	
	mon_list[6].h = (TaskHandle_t)iwdgTaskHandle;
	mon_list[6].stack_words = (uint16_t)(iwdgTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[6].alias = "IWDG";
	
	mon_list[7].h = (TaskHandle_t)uplinkTaskHandle;
	mon_list[7].stack_words = (uint16_t)(uplinkTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[7].alias = "UPLINK";
	
	mon_list[8].h = (TaskHandle_t)sensorTaskHandle;
	mon_list[8].stack_words = (uint16_t)(sensorTask_attributes.stack_size / sizeof(StackType_t));
	mon_list[8].alias = "SENSOR";

	Monitor_RegisterTaskInfo(mon_list, 9);
	
	IWDG_TaskInit(commTaskHandle, controlTaskHandle, NULL, NULL, sensorTaskHandle);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_CommTask */
/**
  * @brief  Function implementing the commTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CommTask */
void CommTask(void *argument)
{
  /* USER CODE BEGIN CommTask */
  /* Infinite loop */
	KaoYaApp_CommTask(argument);
  /* USER CODE END CommTask */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */
  KaoYaApp_LedTask(argument);
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  /* Infinite loop */
	KaoYaApp_ControlTask(argument);
  /* USER CODE END ControlTask */
}

/* USER CODE BEGIN Header_LogTask */
/**
* @brief Function implementing the logTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogTask */
void LogTask(void *argument)
{
  /* USER CODE BEGIN LogTask */
  /* Infinite loop */
	KaoYaApp_LogTask(argument);
  /* USER CODE END LogTask */
}

/* USER CODE BEGIN Header_LcdTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LcdTask */
void LcdTask(void *argument)
{
  /* USER CODE BEGIN LcdTask */
  /* Infinite loop */
	KaoYaApp_LcdTask(argument);
  /* USER CODE END LcdTask */
}

/* USER CODE BEGIN Header_MonitorTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MonitorTask */
void MonitorTask(void *argument)
{
  /* USER CODE BEGIN MonitorTask */
  /* Infinite loop */
	KaoYaApp_MonitorTask(argument);
  /* USER CODE END MonitorTask */
}

/* USER CODE BEGIN Header_IwdgTask */
/**
* @brief Function implementing the iwdgTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IwdgTask */
void IwdgTask(void *argument)
{
  /* USER CODE BEGIN IwdgTask */
  /* Infinite loop */
	KaoYaApp_IwdgTask(argument);
  /* USER CODE END IwdgTask */
}

/* USER CODE BEGIN Header_UplinkTask */
/**
* @brief Function implementing the uplinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UplinkTask */
void UplinkTask(void *argument)
{
  /* USER CODE BEGIN UplinkTask */
  /* Infinite loop */
  KaoYaApp_UplinkTask(argument);
  /* USER CODE END UplinkTask */
}

/* USER CODE BEGIN Header_SensorTask */
/**
* @brief Function implementing the sensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTask */
void SensorTask(void *argument)
{
  /* USER CODE BEGIN SensorTask */
  /* Infinite loop */
  KaoYaApp_SensorTask(argument);
  /* USER CODE END SensorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

