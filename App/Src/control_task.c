/**
 ****************************************************************************************************
 * @file        control_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       控制任务（差速底盘闭环控制，FreeRTOS 周期任务）
 * @details
 * 				通信架构说明：
 *
 *  			CommTask 下发 cmd_vel
 *			        ↓
 * 				ControlTask（10ms 周期）
 *        			↓
 *  			差速逆解（vx, wz -> vL, vR）
 *        			↓
 *  			轮速 PID（左右轮）
 *        			↓
 *  			PWM 输出 -> Motor
 *        			↓
 *  			仿真/反馈更新 -> 轮速测量
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "control_task.h"

#include "comm_task.h"
#include "log_task.h"
#include "monitor_task.h"
#include "iwdg_task.h"

#include "pid.h"
#include "car.h"
#include "motor.h"
#include "encoder.h"
#include "capture.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <math.h>
#include "log_task.h"
/* ============================================================
 * 控制周期配置
 * ============================================================ */
 
 /** 控制周期（ms）：10ms => 100Hz */
#define CTRL_CFG_PERIOD_MS                 (10u) 

/** 控制周期（秒）：PID 计算用 */
#define CTRL_CFG_DT_SEC                    ((float)CTRL_CFG_PERIOD_MS / 1000.0f)

/* ============================================================
 * 控制输出（逻辑 PWM 范围）
 * ============================================================ */

/** PWM 上限（逻辑值，非硬件寄存器） */
#define CTRL_CFG_PWM_MAX                   (1000)
/** PWM 下限 */
#define CTRL_CFG_PWM_MIN                   (-1000)

/* ============================================================
 * 控制策略阈值（可调）
 * ============================================================ */

/** 指令超时时间：超过该时间未收到新 cmd_vel，认为无指令 */
#define CTRL_CFG_CMD_TIMEOUT_MS            (1000u)
/** 判定“停稳”的轮速阈值（mm/s） */
#define CTRL_CFG_MEAS_STOP_TH_MMPS         (15.0f)
/** 停稳确认时间（ms）：连续多周期满足才算真正停稳 */
#define CTRL_CFG_STOP_CONFIRM_TIME_MS      (1000u)
/** 停稳确认计数（周期数） */
#define CTRL_CFG_STOP_CONFIRM_CNT          (CTRL_CFG_STOP_CONFIRM_TIME_MS / CTRL_CFG_PERIOD_MS)

/* ============================================================
 * 内部状态与类型
 * ============================================================ */
 
/** ControlTask 任务句柄（供 CommTask 通知使用） */
TaskHandle_t g_controlTaskHandle = NULL;

/*
 * true：尚未收到速度指令，或者速度指令已经超时
 * false：速度指令仍然有效
 */
static volatile bool s_cmd_timeout = true;

/**
 * @brief 控制任务运行模式
 */
typedef enum
{
    CTRL_MODE_IDLE = 0,   /**< 停机态：等待指令 */
    CTRL_MODE_RUN         /**< 运行态：周期闭环控制 */
} CtrlMode_t;

/* ============================================================
 * 内部工具函数
 * ============================================================ */

/**
 * @brief int32 -> int16 裁剪
 */
static inline int16_t ctrl_clampi16(int32_t x, int16_t lo, int16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return (int16_t)x;
}

/**
 * @brief 获取当前系统时间（ms）
 */
static inline uint32_t ctrl_now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/**
 * @brief 判断轮速是否已停稳
 *
 * @details
 * 左右轮速度绝对值均小于阈值，才认为“接近静止”
 */
static inline bool ctrl_is_stop_meas(float vL_mmps, float vR_mmps)
{
    return (fabsf(vL_mmps) <= CTRL_CFG_MEAS_STOP_TH_MMPS) &&
           (fabsf(vR_mmps) <= CTRL_CFG_MEAS_STOP_TH_MMPS);
}

/* ============================================================
 * 控制任务入口
 * ============================================================ */

/**
 * @brief   控制任务入口函数
 *
 * @details
 *  - IDLE 状态：电机停转，阻塞等待新指令
 *  - RUN 状态：10ms 周期闭环控制
 */
void KaoYaApp_ControlTask(void *argument)
{
    (void)argument;

	/* 保存当前任务句柄，供 CommTask 通知 */
    g_controlTaskHandle = xTaskGetCurrentTaskHandle();

    /* 初始化电机模块（PWM 启动、清零、仿真初始化等） */
    Motor_Init();

	/* 周期调度参数 */
    const TickType_t period_ticks = pdMS_TO_TICKS(CTRL_CFG_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

	/* 当前参考指令（来自 CommTask） */
    CmdVel_t ref_cmd = (CmdVel_t){0};

    /* 左右轮 PID 控制器 */
    PID_t pidL, pidR;
#if PID_KAOYATEACH_ENABLE
	LOGKAOYA_T("CONTROL", "【左轮】PID参数初始化");
#endif
    PID_Init(&pidL, 0.8f, 0.5f, 0.0f, 400.0f, (float)CTRL_CFG_PWM_MAX);
#if PID_KAOYATEACH_ENABLE
	LOGKAOYA_T("CONTROL", "【右轮】PID参数初始化");
#endif
    PID_Init(&pidR, 0.8f, 0.5f, 0.0f, 400.0f, (float)CTRL_CFG_PWM_MAX);

	/* 当前控制模式 */
    CtrlMode_t mode = CTRL_MODE_IDLE;

	/* 最近一次接收到有效指令的时间 */
    uint32_t last_cmd_ms = ctrl_now_ms();
	
	/* 停稳确认计数器 */
    uint32_t stop_cnt = 0;

    for (;;)
    {
        /* ============================================================
         * IDLE 模式：电机停转，阻塞等待新指令
         * ============================================================ */
        if (mode == CTRL_MODE_IDLE)
        {
			/* 确保输出清零 */
            Motor_ApplyPwm(0, 0);
			
			/* 重置 PID 积分等内部状态 */
            PID_Reset(&pidL);
            PID_Reset(&pidR);
			
			/* 清空停稳计数 */
            stop_cnt = 0;

			/* 任务心跳 */
            MON_HB_CONTROL();

            /* IDLE 时关闭看门狗监控 */
            IWDG_WatchEnable(IWDG_ID_CONTROL, false);
            IWDG_WatchEnable(IWDG_ID_COMM,    false);

			/* 阻塞等待通知（来自 CommTask 的新指令） */
            uint32_t notify_val = 0;
            (void)xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, portMAX_DELAY);

			/* 收到新 cmd_vel 通知 */
            if (notify_val & CTRL_NOTIF_NEW_CMD) 
			{	
				/* 读取最新指令 */
                ref_cmd = Comm_GetLatestCmdVel();
                last_cmd_ms = ctrl_now_ms();
								s_cmd_timeout = false;
				/* 切换到 RUN 模式 */
                mode = CTRL_MODE_RUN;
				
				/* 重置周期调度基准 */
                last_wake = xTaskGetTickCount();
            }
            continue;
        }

        /* ============================================================
         * RUN：10ms 周期闭环
         * ============================================================ */
        
				/* 任务心跳 */
				MON_HB_CONTROL();

        /* RUN 时开启监控并喂狗 */
        IWDG_WatchEnable(IWDG_ID_CONTROL, true);
        IWDG_WatchEnable(IWDG_ID_COMM,    true);
        IWDG_Heartbeat(IWDG_ID_CONTROL);

        /* 非阻塞接收新指令通知 */
        uint32_t notify_val = 0;
        (void)xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);

				/* 若有新 cmd_vel，更新参考 */
        if (notify_val & CTRL_NOTIF_NEW_CMD) 
				{
            ref_cmd = Comm_GetLatestCmdVel();
            last_cmd_ms = ctrl_now_ms();
						s_cmd_timeout = false;
        }

        /* 超时：认为无指令 => ref=0 */
        const uint32_t tnow = ctrl_now_ms();
        if ((tnow - last_cmd_ms) > CTRL_CFG_CMD_TIMEOUT_MS) 
				{
            ref_cmd.valid = false;
						s_cmd_timeout = true;
        }

        /* 轮速参考生成（差速逆解） */
        float vL_ref_mmps = 0.0f;
        float vR_ref_mmps = 0.0f;
		
        if (ref_cmd.valid) 
				{
						/* 根据 vx / wz 反算左右轮目标速度 */
            CarDiff_Inverse(CarDiff_GetDefaultParam(), ref_cmd.vx_mmps, ref_cmd.wz_mradps, &vL_ref_mmps, &vR_ref_mmps);      
        }

        /* ====================================================
         * 轮速测量（来自电机仿真/反馈）
         * ==================================================== */
        float vL_meas_mmps = 0.0f;
        float vR_meas_mmps = 0.0f;
				vL_meas_mmps = EncoderCap_GetLeftMmps();
				vR_meas_mmps = EncoderCap_GetRightMmps();
          
        /* ====================================================
         * PID 闭环计算
         * ==================================================== */		
#if PID_KAOYATEACH_ENABLE		
		static bool log_cnt_l = 0;
		static bool log_cnt_r = 0;
#endif
		float errL = vL_ref_mmps - vL_meas_mmps;
		float errR = vR_ref_mmps - vR_meas_mmps;
		if(fabsf(errL) > 10.0f)
		{	
#if PID_KAOYATEACH_ENABLE
			log_cnt_l  = 0;
			LOGKAOYA_T("CONTROL", "【左轮】PID调速：ref=%.2f meas=%.2f err=%.2f",
                   (double)vL_ref_mmps, (double)vL_meas_mmps, (double)errL);
#endif
			int16_t pwmL = PID_Update(&pidL, vL_ref_mmps, vL_meas_mmps, CTRL_CFG_DT_SEC);
			pwmL = ctrl_clampi16((int32_t)pwmL, CTRL_CFG_PWM_MIN, CTRL_CFG_PWM_MAX);
			Motor_ApplyPwmL(pwmL);
		}
#if PID_KAOYATEACH_ENABLE
		else
		{	
			if(log_cnt_l == 0)
			{
				log_cnt_l = 1;
				LOGKAOYA_T("CONTROL", "【左轮】调速完成：ref=%.2f meas=%.2f err=%.2f",
                       (double)vL_ref_mmps, (double)vL_meas_mmps, (double)errL);
			}
			
		}
#endif
		if(fabsf(errR) > 10.f){
#if PID_KAOYATEACH_ENABLE
			log_cnt_r = 0;
			LOGKAOYA_T("CONTROL", "【右轮】PID调速：ref=%.2f meas=%.2f err=%.2f",
                   (double)vR_ref_mmps, (double)vR_meas_mmps, (double)errR);
#endif
			int16_t pwmR = PID_Update(&pidR, vR_ref_mmps, vR_meas_mmps, CTRL_CFG_DT_SEC);
			pwmR = ctrl_clampi16((int32_t)pwmR, CTRL_CFG_PWM_MIN, CTRL_CFG_PWM_MAX);
			Motor_ApplyPwmR(pwmR);
		}      
#if PID_KAOYATEACH_ENABLE
		else
		{
			if(log_cnt_r == 0)
			{
				log_cnt_r = 1;
				LOGKAOYA_T("CONTROL", "【右轮】调速完成：ref=%.2f meas=%.2f err=%.2f",
                       (double)vR_ref_mmps, (double)vR_meas_mmps, (double)errR);
			}
		}
#endif

        /* ====================================================
         * 仿真反馈更新（速度/编码器）
         * ==================================================== */
					MotorSim_Tick(CTRL_CFG_DT_SEC);
	
				/* 再次读取最新测量值*/
					vL_meas_mmps = EncoderCap_GetLeftMmps();
					vR_meas_mmps = EncoderCap_GetRightMmps();
        /* ====================================================
         * 无指令时的停稳确认与模式切换
         * ==================================================== */
        if (!ref_cmd.valid) 
				{
						/* 若当前轮速已接近 0，累加停稳计数 */
            if (ctrl_is_stop_meas(vL_meas_mmps, vR_meas_mmps)) 
						{
                stop_cnt++;
            } 
			else
				{				
                stop_cnt = 0;
				}

						/* 连续多周期停稳，确认进入 IDLE */
            if (stop_cnt >= CTRL_CFG_STOP_CONFIRM_CNT) 
						{
					mode = CTRL_MODE_IDLE;
					/* 清空仿真速度，确保完全停机 */
					EncoderSim_SetSpeedMmps(0, 0);
                continue;
            }

        } 
		else
		{			
			/* 有指令时清空停稳计数 */
            stop_cnt = 0;
        }

		/* 周期延时，保证 10ms 固定节拍 */
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */

/**
 * @brief   获取当前轮速测量值
 *
 * @note
 * 当前实现直接从 Motor 模块读取，便于其他任务/上位机查询。
 */
void Control_GetWheelSpeedMmps(float *vL_mmps, float *vR_mmps)
{
    Motor_GetWheelSpeedMmps(vL_mmps, vR_mmps);
}

bool Control_IsCmdTimeout(void)
{
    return s_cmd_timeout;
}

