#include "bsp_log.h"
#include "usart.h"          // huart2
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ============================================================
 * 配置（先保证“基本功能好用”）
 * ============================================================ */
#define BSP_LOG_UART              (&huart2)   // 固定用 UART2 输出
#define BSP_LOG_RB_SIZE           2048        // ring buffer 大小
#define BSP_LOG_PRINTF_MAX        256         // 单条格式化最大长度（含前缀）

/* ============================================================
 * ring buffer 状态
 * ============================================================ */
static uint8_t  s_rb[BSP_LOG_RB_SIZE];
static volatile uint16_t s_head = 0;      // 写指针
static volatile uint16_t s_tail = 0;      // 读指针（发送指针）

static volatile uint16_t s_dma_len = 0;   // 当前 DMA 发送长度
static volatile uint8_t  s_dma_busy = 0;  // DMA 是否忙

static volatile uint32_t s_drop_cnt = 0;
static volatile uint32_t s_drop_bytes = 0;

static volatile BspLogLevel_t s_runtime_level = BSP_LOG_LVL_INFO;

/* ============================================================
 * ring 工具
 * ============================================================ */
static inline uint16_t rb_used(uint16_t head, uint16_t tail)
{
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(BSP_LOG_RB_SIZE - tail + head);
}

static inline uint16_t rb_free(uint16_t head, uint16_t tail)
{
    /* 留 1 字节空位区分满/空 */
    return (uint16_t)(BSP_LOG_RB_SIZE - 1 - rb_used(head, tail));
}

/* ============================================================
 * 内部：尝试启动 DMA（必须在临界区内调用）
 * 只发“连续段”，避免跨环；跨环部分由下一次 TxCplt 再发
 * ============================================================ */
static void try_start_dma_locked(void)
{
    if (s_dma_busy) return;

    uint16_t head = s_head;
    uint16_t tail = s_tail;
    if (head == tail) return; // 空

    uint16_t len = (head > tail) ? (uint16_t)(head - tail)
                                 : (uint16_t)(BSP_LOG_RB_SIZE - tail);
    if (len == 0) return;

    s_dma_busy = 1;
    s_dma_len  = len;

    if (HAL_UART_Transmit_DMA(BSP_LOG_UART, (uint8_t*)&s_rb[tail], len) != HAL_OK) {
        /* DMA 启动失败：释放 busy，等待后续再试 */
        s_dma_busy = 0;
        s_dma_len  = 0;
    }
}

/* ============================================================
 * 内部：写入 ring（必须在临界区内调用）
 * ============================================================ */
static bool rb_write_locked(const uint8_t *p, uint16_t len)
{
    uint16_t head = s_head;
    uint16_t tail = s_tail;

    uint16_t free = rb_free(head, tail);
    if (len > free) {
        s_drop_cnt++;
        s_drop_bytes += len;
        return false;
    }

    /* 可能跨环：分两段写 */
    uint16_t first = (uint16_t)(BSP_LOG_RB_SIZE - head);
    if (first > len) first = len;

    memcpy(&s_rb[head], p, first);
    head = (uint16_t)((head + first) % BSP_LOG_RB_SIZE);

    uint16_t remain = (uint16_t)(len - first);
    if (remain) {
        memcpy(&s_rb[head], p + first, remain);
        head = (uint16_t)(head + remain);
    }

    s_head = head;

    /* kick DMA */
    try_start_dma_locked();
    return true;
}

/* ============================================================
 * 等级字符
 * ============================================================ */
static inline char lvl_char(BspLogLevel_t lvl)
{
    switch (lvl) {
    case BSP_LOG_LVL_ERROR: return 'E';
    case BSP_LOG_LVL_WARN:  return 'W';
    case BSP_LOG_LVL_INFO:  return 'I';
    default:                return 'D';
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void BspLog_Init(void)
{
    taskENTER_CRITICAL();
    s_head = 0;
    s_tail = 0;
    s_dma_len = 0;
    s_dma_busy = 0;
    s_drop_cnt = 0;
    s_drop_bytes = 0;
    s_runtime_level = BSP_LOG_LVL_INFO;
    taskEXIT_CRITICAL();
}

void BspLog_SetLevel(BspLogLevel_t lvl) { s_runtime_level = lvl; }
BspLogLevel_t BspLog_GetLevel(void) { return s_runtime_level; }

uint32_t BspLog_GetDropCount(void) { return s_drop_cnt; }
uint32_t BspLog_GetDropBytes(void) { return s_drop_bytes; }

uint32_t BspLog_GetQueuedBytes(void)
{
    taskENTER_CRITICAL();
    uint16_t used = rb_used(s_head, s_tail);
    taskEXIT_CRITICAL();
    return used;
}

/* 写原始数据（非阻塞，空间不足则丢） */
bool BspLog_Write(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) return true;

    taskENTER_CRITICAL();
    bool ok = rb_write_locked(data, len);
    taskEXIT_CRITICAL();
    return ok;
}

/* 预留：ISR 写日志（暂时简单处理：直接禁止在 ISR 调用，返回 false）
 * 后期你要支持 ISR 打日志，我们再把它实现成 taskENTER_CRITICAL_FROM_ISR 版本即可，
 * 外部调用不需要改。
 */
bool BspLog_WriteFromISR(const uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
    return false;
}

/* printf 风格：运行时过滤 + 前缀 + 写入 ring */
void BspLog_Printf(BspLogLevel_t lvl, const char *tag, const char *fmt, ...)
{
	
    if (!fmt) return;

    /* 运行时等级过滤 */
    if (lvl > s_runtime_level) return;

    char buf[BSP_LOG_PRINTF_MAX];

    /* 前缀：例如 "I/CTRL: " */
    int off;
    if (tag && tag[0]) {
        off = snprintf(buf, sizeof(buf), "%c/%s: ", lvl_char(lvl), tag);
    } else {
        off = snprintf(buf, sizeof(buf), "%c: ", lvl_char(lvl));
    }
    if (off < 0) return;
    if (off >= (int)sizeof(buf)) off = (int)sizeof(buf) - 1;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + off, sizeof(buf) - (size_t)off, fmt, ap);
    va_end(ap);

    if (n < 0) return;

    int total = off + n;
    if (total >= (int)sizeof(buf)) total = (int)sizeof(buf) - 1;

    /* 写入 ring（非阻塞） */
    (void)BspLog_Write((const uint8_t*)buf, (uint16_t)total);
}

/* DMA TX 完成推进：推进 tail 并继续发送 */
void BspLog_Uart2TxCpltISR(void)
{
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    uint16_t tail = s_tail;
    uint16_t adv  = s_dma_len;

    tail = (uint16_t)((tail + adv) % BSP_LOG_RB_SIZE);
    s_tail = tail;

    s_dma_len = 0;
    s_dma_busy = 0;

    try_start_dma_locked();

    taskEXIT_CRITICAL_FROM_ISR(saved);
}

/* ============================================================
 * HAL 回调（你要求“回调也在这里实现”）
 *
 *   注意：全工程只能有一个 HAL_UART_TxCpltCallback 定义。
 * - 如果你工程里已经存在该函数（例如别的模块定义过），
 *   请把下面这个函数删除，并在你已有的 callback 里加上
 *   `if (huart->Instance == USART2) BspLog_Uart2TxCpltISR();`
 * ============================================================ */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart && huart->Instance == USART2) {
        BspLog_Uart2TxCpltISR();
    }
}
