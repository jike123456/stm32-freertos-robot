#ifndef __BSP_LOG_H
#define __BSP_LOG_H

#include <stdint.h>
#include <stdbool.h>

/* =========================
 * 日志等级（数值越小越重要）
 * ========================= */
typedef enum {
    BSP_LOG_LVL_ERROR = 0,
    BSP_LOG_LVL_WARN  = 1,
    BSP_LOG_LVL_INFO  = 2,
    BSP_LOG_LVL_DEBUG = 3,
} BspLogLevel_t;

/* 初始化：建议在 App_Init() 调一次 */
void BspLog_Init(void);

/* 运行时等级控制（默认 INFO） */
void BspLog_SetLevel(BspLogLevel_t lvl);
BspLogLevel_t BspLog_GetLevel(void);

/* =========================
 * 写入接口（稳定 API，后期扩展不改调用方式）
 * - BspLog_Printf：最常用，支持 tag + printf
 * - BspLog_Write ：写原始字节（你自己拼好的字符串/二进制也能发）
 * - BspLog_WriteFromISR：预留（后续要在 ISR 打日志就用它）
 * ========================= */
void BspLog_Printf(BspLogLevel_t lvl, const char *tag, const char *fmt, ...);
bool BspLog_Write(const uint8_t *data, uint16_t len);
bool BspLog_WriteFromISR(const uint8_t *data, uint16_t len);

/* UART2 DMA 发送完成回调：从 HAL_UART_TxCpltCallback 转发 */
void BspLog_Uart2TxCpltISR(void);

/* 可选：统计 */
uint32_t BspLog_GetDropCount(void);
uint32_t BspLog_GetDropBytes(void);
uint32_t BspLog_GetQueuedBytes(void);

/* ========= 编译期裁剪 =========
 * 设置 BSP_LOG_COMPILE_LEVEL 来裁剪低优先级日志（0开销）
 * 例：只保留 INFO 及以上：
 *   #define BSP_LOG_COMPILE_LEVEL BSP_LOG_LVL_INFO
 */
#ifndef BSP_LOG_COMPILE_LEVEL
#define BSP_LOG_COMPILE_LEVEL BSP_LOG_LVL_DEBUG
#endif

#if BSP_LOG_COMPILE_LEVEL >= BSP_LOG_LVL_ERROR
#define LOGE(tag, fmt, ...) BspLog_Printf(BSP_LOG_LVL_ERROR, tag, fmt, ##__VA_ARGS__)
#else
#define LOGE(tag, fmt, ...) ((void)0)
#endif

#if BSP_LOG_COMPILE_LEVEL >= BSP_LOG_LVL_WARN
#define LOGW(tag, fmt, ...) BspLog_Printf(BSP_LOG_LVL_WARN,  tag, fmt, ##__VA_ARGS__)
#else
#define LOGW(tag, fmt, ...) ((void)0)
#endif

#if BSP_LOG_COMPILE_LEVEL >= BSP_LOG_LVL_INFO
#define LOGI(tag, fmt, ...) BspLog_Printf(BSP_LOG_LVL_INFO,  tag, fmt, ##__VA_ARGS__)
#else
#define LOGI(tag, fmt, ...) ((void)0)
#endif

#if BSP_LOG_COMPILE_LEVEL >= BSP_LOG_LVL_DEBUG
#define LOGD(tag, fmt, ...) BspLog_Printf(BSP_LOG_LVL_DEBUG, tag, fmt, ##__VA_ARGS__)
#else
#define LOGD(tag, fmt, ...) ((void)0)
#endif

#endif
