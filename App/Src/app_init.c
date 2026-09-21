/**
 ****************************************************************************************************
 * @file        app_init.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       初始化模块
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       STM32F407 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "app_init.h"
#include "log_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "lcd_task.h"
#include "monitor_task.h"
#include "comm_task.h"
#include "sensor_task.h"

void KaoYaApp_Init(void)
{
	/* 日志队列必须先创建，后续模块初始化时产生的日志才不会被丢弃。 */
	Log_Init();

	KaoYaApp_CommInit();
	
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);	

}

