/**
 ****************************************************************************************************
 * @file        log_task.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       异步日志模块对外接口（队列投递 + 日志任务 + PANIC 直打）
 * @license     Copyright (c) 2025-2035, Kaoya Project
 *
 ****************************************************************************************************
 */
#ifndef __LOG_TASK_H
#define __LOG_TASK_H

#include <stdint.h>
#include <stdbool.h>

/* ====================== 烤鸭教学：模块级开关 ====================== */
#define BATTERY_KAOYATEACH_ENABLE         0   /* 0=关闭电池教学计数/打印；1=开启 */
#define IMU_KAOYATEACH_ENABLE             0   /* 0=关闭IMU教学计数/打印；1=开启 */
#define ENCODER_KAOYATEACH_ENABLE         0   /* 0=关闭编码器教学计数/打印；1=开启 */
#define COMMPARSER_KAOYATEACH_ENABLE      1   /* 0=关闭指令解析教学计数/打印；1=开启 */
#define PID_KAOYATEACH_ENABLE             0   /* 0=关闭pid调速教学计数/打印；1=开启 */
#define UPLINK_KAOYATEACH_ENABLE          0   /* 0=关闭上报数据教学计数/打印；1=开启 */
#define IWDG_KAOYATEACH_ENABLE            0   /* 0=关闭看门狗教学计数/打印；1=开启 */
#define LOG_KAOYATEACH_ENABLE             0   /* 0=关闭上报数据教学计数/打印；1=开启 */
#define DMA_KAOYATEACH_ENABLE             0   /* 0=关闭上报数据教学计数/打印；1=开启 */

/**
 * @brief 日志等级定义
 *
 * @note
 * - 编译期过滤：LOG_COMPILE_LEVEL
 * - 运行时过滤：Log_SetLevel()
 */
typedef enum {
	LOG_LVL_KAOYA = 0,    // 教学
    LOG_LVL_ERROR = 1,    // 错误
    LOG_LVL_WARN  = 2,    // 警告
    LOG_LVL_INFO  = 3,    // 信息
    LOG_LVL_DEBUG = 4,    // 调试
} LogLevel_t;

/**
 * @brief 初始化日志模块：创建队列、清空 ring/DMA 状态
 */
void Log_Init(void);

/**
 * @brief 设置运行时日志等级过滤
 * @param[in] lvl 新的运行时等级
 */
void Log_SetLevel(LogLevel_t lvl);

/**
 * @brief 获取运行时日志等级
 * @return 当前运行时等级
 */
LogLevel_t Log_GetLevel(void);

/**
 * @brief 业务侧投递日志（非阻塞，不做格式化）
 *
 * @param[in] lvl 日志等级
 * @param[in] tag 标签（NULL 或 "" 表示无 tag）
 * @param[in] fmt 格式串（建议字符串常量）
 * @param[in] ... 参数（支持 %d/%u/%x/%X/%c/%s/%f）
 *
 * @return true  投递成功或被运行时等级过滤（视为成功）
 * @return false 队列未初始化/参数错误/队列满导致投递失败
 */
bool Log_Post(LogLevel_t lvl, const char *tag, const char *fmt, ...);

/**
 * @brief 日志任务入口函数（从队列取日志并输出）
 * @param[in] argument 任务参数（未使用）
 */
void KaoYaApp_LogTask(void *argument);

/**
 * @brief UART2 DMA TX 完成 ISR 转发（推进 ring tail 并继续发送）
 *
 * @note
 * - 若工程已有 HAL_UART_TxCpltCallback()，在其中调用该函数即可
 */
void Log_Uart2TxCpltISR(void);

/* =========================
 * PANIC：保底通道控制
 * ========================= */

/**
 * @brief 进入 panic 模式（改走 UART2 轮询直打）
 *
 * @note
 * 常用于：HardFault / stack overflow / configASSERT 等场景
 */
void Log_EnterPanic(void);

/**
 * @brief 查询是否处于 panic 模式
 * @return true  panic
 * @return false 非 panic
 */
bool Log_IsPanic(void);

/**
 * @brief panic 模式下 ERROR 直打（阻塞）
 *
 * @param[in] tag 标签（可为 NULL）
 * @param[in] fmt 格式串
 * @param[in] ... 参数
 */
void Log_Error(const char *tag, const char *fmt, ...);

/* =========================
 * 编译期日志等级过滤（宏）
 * ========================= */

/**
 * @brief 编译期日志等级（大于等于该等级的宏才会编译进来）
 *
 * @note
 * - 默认设为 KAOYA（即全开）
 * - 你也可以改成 LOG_LVL_INFO/LOG_LVL_DEBUG 等
 */
#define LOG_LEVEL_KAOYA  0
#define LOG_LEVEL_ERROR  1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_INFO   3
#define LOG_LEVEL_DEBUG  4

/* 预处理器无法直接识别 enum 常量，因此编译期比较必须使用数值宏。 */
#define LOG_COMPILE_LEVEL LOG_LEVEL_DEBUG

#if (LOG_COMPILE_LEVEL >= LOG_LEVEL_ERROR)
	/**
     * @brief ERROR（无 tag）
     * @note panic 模式下走阻塞直打
     */
    #define LOGERROR(fmt, ...) do { \
        if (Log_IsPanic()) { Log_Error(NULL, (fmt), ##__VA_ARGS__); } \
        else { (void)Log_Post(LOG_LVL_ERROR, NULL, (fmt), ##__VA_ARGS__); } \
    } while(0)
	/**
     * @brief ERROR（带 tag）
     * @note panic 模式下走阻塞直打
     */
    #define LOGERROR_T(tag, fmt, ...) do { \
        if (Log_IsPanic()) { Log_Error((tag), (fmt), ##__VA_ARGS__); } \
        else { (void)Log_Post(LOG_LVL_ERROR, (tag), (fmt), ##__VA_ARGS__); } \
    } while(0)
#else
    #define LOGERROR(fmt, ...)        ((void)0)
    #define LOGERROR_T(tag, fmt, ...) ((void)0)
#endif

#if (LOG_COMPILE_LEVEL >= LOG_LEVEL_WARN)
	/** @brief WARN（无 tag） */
	#define LOGWARN(fmt, ...) (void)Log_Post(LOG_LVL_WARN, NULL, fmt, ##__VA_ARGS__)
	/** @brief WARN（带 tag） */
	#define LOGWARN_T(tag, fmt, ...) (void)Log_Post(LOG_LVL_WARN,  tag,  fmt, ##__VA_ARGS__)
#else
	#define LOGWARN(tag, fmt, ...) ((void)0)
	#define LOGWARN_T(tag, fmt, ...) ((void)0)
#endif

#if (LOG_COMPILE_LEVEL >= LOG_LEVEL_INFO)
	/** @brief INFO（无 tag） */
	#define LOGINFO(fmt, ...) (void)Log_Post(LOG_LVL_INFO, NULL, fmt, ##__VA_ARGS__)
	/** @brief INFO（带 tag） */
	#define LOGINFO_T(tag, fmt, ...) (void)Log_Post(LOG_LVL_INFO,  tag,  fmt, ##__VA_ARGS__)
#else
	#define LOGINFO(tag, fmt, ...) ((void)0)
	#define LOGINFO_T(tag, fmt, ...) ((void)0)
#endif

#if (LOG_COMPILE_LEVEL >= LOG_LEVEL_DEBUG)
	/** @brief DEBUG（无 tag） */
	#define LOGDEBUG(fmt, ...) (void)Log_Post(LOG_LVL_DEBUG, NULL, fmt, ##__VA_ARGS__)
	/** @brief DEBUG（带 tag） */
	#define LOGDEBUG_T(tag, fmt, ...) (void)Log_Post(LOG_LVL_DEBUG, tag,  fmt, ##__VA_ARGS__)
#else
	#define LOGDEBUG(tag, fmt, ...) ((void)0)
	#define LOGDEBUG_T(tag, fmt, ...) ((void)0)
#endif
	
#if (LOG_COMPILE_LEVEL >= LOG_LEVEL_KAOYA)
	/** @brief KAOYA（无 tag） */
	#define LOGKAOYA(fmt, ...) (void)Log_Post(LOG_LVL_KAOYA, NULL, fmt, ##__VA_ARGS__)
	/** @brief KAOYA（带 tag） */
	#define LOGKAOYA_T(tag, fmt, ...) (void)Log_Post(LOG_LVL_KAOYA, tag,  fmt, ##__VA_ARGS__)
#else
	#define LOGKAOYA(tag, fmt, ...) ((void)0)
	#define LOGKAOYA_T(tag, fmt, ...) ((void)0)
#endif
	
#endif /* __LOG_TASK_H */
