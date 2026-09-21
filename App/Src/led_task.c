/**
 ****************************************************************************************************
 * @file        led_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       LED 心跳任务：周期翻转 LED0/LED1，用于指示系统运行状态
 * @details
 * - 任务以固定节拍运行（vTaskDelayUntil），避免因执行抖动导致周期漂移
 * - 每次循环：
 *   1) 更新 Monitor 心跳（用于看门狗/任务活性监测）
 *   2) 翻转两路 LED
 *   3) 延时到下一个 500ms 周期点
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "led_task.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "monitor_task.h"
#include "log_task.h"

/**
 * @brief LED 任务入口函数
 *
 * @param[in] argument 任务参数（未使用）
 *
 * @note
 * - vTaskDelayUntil() 使用“绝对时间”节拍，适合周期任务
 * - LED 闪烁频率：500ms 翻转一次（两次翻转为一个完整闪烁周期）
 */
void KaoYaApp_LedTask(void *argument)
{
    (void)argument;

    /* 记录首次唤醒时间，用于 vTaskDelayUntil 固定节拍调度 */
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        /* 任务心跳：用于 MonitorTask 统计任务是否正常运行 */
        MON_HB_LED();

        /* 翻转 LED 引脚：直观显示系统仍在运行 */
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	
        /* 固定周期：500ms */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
}
