/**
 ****************************************************************************************************
 * @file        log_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       异步日志模块（队列投递 + 日志任务格式化 + UART2 DMA RingBuffer 发送）
 * @details
 * - 业务任务调用 Log_Post() 时不做格式化，不阻塞（队列满直接丢弃）
 * - 日志任务统一 format 为字符串，写入 DMA ring buffer
 * - DMA 发送完成 ISR 推进 tail，继续发送下一段（不跨环）
 * - 提供 PANIC 模式：硬故障/栈溢出时可切换为 UART2 轮询直打
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "log_task.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "monitor_task.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "uplink_proto.h"

/* =========================
 * 配置
 * ========================= */
/** @brief 日志输出等级 */
#define LOG_RUNTIME_LEVEL LOG_LVL_WARN

/** @brief 日志输出串口：UART2（DMA 发送） */
#define LOG_UART                  (&huart2)

/** @brief 日志队列深度：最多缓存多少条待格式化日志 */
#define LOG_Q_DEPTH               128

/** @brief 单条日志最多支持多少个参数（从 fmt 中解析 %d/%s/... 得到） */
#define LOG_MAX_ARGS              8

/** @brief DMA 发送 ring buffer 大小（字节） */
#define LOG_RB_SIZE               4096

/** @brief 单条输出最大长度（含前缀与 \r\n） */
#define LOG_LINE_MAX              256

/* =========================
 * 队列记录：参数类型
 * ========================= */

/**
 * @brief 可序列化的参数类型（从 fmt 中提取）
 *
 * @note
 * - 这里只做“有限类型集合”，避免在业务线程里做复杂格式化
 * - 浮点统一按 double 保存（C 默认变参提升 float -> double）
 */
typedef enum {
    ARG_I32,     /**< int32_t / %d %i */
    ARG_U32,     /**< uint32_t / %u */
    ARG_HEX32,   /**< uint32_t / %x %X */
    ARG_CHAR,    /**< char / %c */
    ARG_STR,     /**< const char* / %s */
    ARG_F64,     /**< double / %f %F */
} ArgType_t;

/**
 * @brief 单个参数的存储结构
 */
typedef struct {
    ArgType_t t; /**< 参数类型 */
    union {
        int32_t     i32; /**< 有符号整型 */
        uint32_t    u32; /**< 无符号整型/HEX */
        char        ch;  /**< 字符 */
        const char *s;   /**< 字符串指针（建议传字符串常量或长生命周期指针） */
        double      f64; /**< 双精度浮点 */
    } v;
} Arg_t;

/**
 * @brief 队列中的一条“日志记录”（未格式化字符串）
 *
 * @note
 * - fmt/tag 建议传字符串常量（生命周期长）
 * - 本结构体会被拷贝进队列，因此只存指针，不拷贝字符串内容
 */
typedef struct {
    uint8_t     lvl;                 /**< 日志等级（LogLevel_t） */
    const char *tag;                 /**< 可选标签：NULL 或 "" 表示无 tag */
    const char *fmt;                 /**< 格式字符串（建议常量） */
    uint8_t     argc;                /**< 参数个数（<= LOG_MAX_ARGS） */
    Arg_t       args[LOG_MAX_ARGS];  /**< 参数数组（按 fmt 中顺序） */
} LogRec_t;

/* =========================
 * 队列（静态）
 * ========================= */

/** @brief 静态队列控制块 */
static StaticQueue_t s_qcb;
/** @brief 静态队列存储区 */
static uint8_t       s_qbuf[LOG_Q_DEPTH * sizeof(LogRec_t)];
/** @brief 队列句柄 */
static QueueHandle_t s_q = NULL;

/* =========================
 * 运行时等级
 * ========================= */

/**
 * @brief 运行时日志等级（用于过滤）
 *
 * @note
 * - 数值越大越“啰嗦”：ERROR(0) 最少，DEBUG(3) 更多
 */
static volatile LogLevel_t s_runtime_level = LOG_LVL_INFO;

/* =========================
 * 内部：DMA ring buffer
 * ========================= */
 
/** @brief DMA 发送 ring buffer 实体 */
static uint8_t s_rb[LOG_RB_SIZE];

/** @brief ring head：写指针（producer） */
static volatile uint16_t s_head = 0;
/** @brief ring tail：读指针（DMA consumer） */
static volatile uint16_t s_tail = 0;

/** @brief 当前一次 DMA 正在发送的长度（用于 TxCplt 推进 tail） */
static volatile uint16_t s_dma_len  = 0;
/** @brief DMA busy 标志：1 表示 DMA 发送中 */
static volatile uint8_t  s_dma_busy = 0;

/* =========================
 * 工具：ring 计算
 * ========================= */
 /**
 * @brief 计算 ring 已使用字节数
 * @param[in] head 写指针
 * @param[in] tail 读指针
 * @return 已使用字节数
 */
static inline uint16_t rb_used(uint16_t head, uint16_t tail)
{
    return (head >= tail) ? (uint16_t)(head - tail)
                          : (uint16_t)(LOG_RB_SIZE - tail + head);
}

/**
 * @brief 计算 ring 可用字节数
 * @param[in] head 写指针
 * @param[in] tail 读指针
 * @return 可用字节数（保留 1 字节区分空/满）
 */
static inline uint16_t rb_free(uint16_t head, uint16_t tail)
{
    /* 留 1 字节区分满/空 */
    return (uint16_t)(LOG_RB_SIZE - 1 - rb_used(head, tail));
}

/* =========================
 * 工具：等级字符串（固定宽度便于对齐）
 * ========================= */

/**
 * @brief 将日志等级转换为 5 字符宽度字符串
 * @param[in] lvl 日志等级
 * @return 等级字符串（常量指针）
 */
static inline const char* lvl_str(LogLevel_t lvl)
{
    switch (lvl) {
    case LOG_LVL_ERROR: return "ERROR";
    case LOG_LVL_WARN:  return "WARN ";
    case LOG_LVL_INFO:  return "INFO ";
	case LOG_LVL_DEBUG: return "DEBUG";
    default:            return "KAOYA";
    }
}

/* =========================
 * 内部：启动 DMA（必须在临界区内）
 * 只发送连续段，避免跨环
 * ========================= */

/**
 * @brief 尝试启动一次 UART2 DMA 发送（仅发送 ring 中的连续段）
 *
 * @note
 * - 必须在临界区调用：依赖 s_head/s_tail/s_dma_busy 的一致性
 * - 为避免跨环，若 tail 到 buffer 末尾有数据，则先发这一段
 *   后续在 TxCplt ISR 里推进 tail 再继续发下一段
 */
static void try_start_dma_locked(void)
{
    if (s_dma_busy) return;

    uint16_t head = s_head;
    uint16_t tail = s_tail;
    if (head == tail) return;

	/* 只取连续段：head>tail 直接发；否则先发 tail->末尾 */
    uint16_t len = (head > tail) ? (uint16_t)(head - tail)
                                 : (uint16_t)(LOG_RB_SIZE - tail);
    if (len == 0) return;

    s_dma_busy = 1;
    s_dma_len  = len;

	/* DMA 启动失败则回滚 busy */
    if (HAL_UART_Transmit_DMA(LOG_UART, (uint8_t*)&s_rb[tail], len) != HAL_OK) {
        s_dma_busy = 0;
        s_dma_len  = 0;
    }
}

/* =========================
 * 内部：写 ring（必须在临界区内）
 * ========================= */

/**
 * @brief 向 ring 写入一段字节流（producer）
 * @param[in] p   数据指针
 * @param[in] len 数据长度
 * @return true 写入成功
 * @return false ring 空间不足（丢弃）
 *
 * @note
 * - 必须在临界区调用：避免与 ISR 推进 tail 并发
 * - 写入成功后会 try_start_dma_locked() 尝试启动 DMA
 */
static bool rb_write_locked(const uint8_t *p, uint16_t len)
{
    uint16_t head = s_head;
    uint16_t tail = s_tail;

    if (len > rb_free(head, tail)) {
        return false; // 满就丢（后期可加统计）
    }

	/* 先写到末尾能写的那段 */
    uint16_t first = (uint16_t)(LOG_RB_SIZE - head);
    if (first > len) first = len;

    memcpy(&s_rb[head], p, first);
    head = (uint16_t)((head + first) % LOG_RB_SIZE);

	/* 如有剩余，再从头部继续写 */
    uint16_t remain = (uint16_t)(len - first);
    if (remain) {
        memcpy(&s_rb[head], p + first, remain);
        head = (uint16_t)(head + remain);
    }

    s_head = head;
	
	/* 写入后尝试启动 DMA */
    try_start_dma_locked();
    return true;
}


/* =========================
 * 内部：解析 fmt 并把参数拷贝进 record
 * 支持：%d %i %u %x %X %c %s %f %%
 * 宽度/精度/flags 会跳过（先保证稳定）
 * ========================= */

/**
 * @brief 解析格式串 fmt，将变参提取为 Arg_t 数组保存到 LogRec_t
 *
 * @param[in,out] r       目标记录（r->fmt 已设置）
 * @param[in]     ap_copy va_list 副本（函数内部会消费它）
 *
 * @note
 * - 仅支持常用 specifier，其他 spec 会被忽略（且不消费参数，避免错位）
 * - flags/width/precision/length 目前只“跳过不处理”，保证解析同步
 */
static void parse_args(LogRec_t *r, va_list ap_copy)
{
    r->argc = 0;
    const char *p = r->fmt;

    while (*p && r->argc < LOG_MAX_ARGS) {
        if (*p != '%') { p++; continue; }
        p++; // skip '%'

        if (*p == '%') { p++; continue; }

        // flags
        while (*p == '-' || *p == '+' || *p == ' ' || *p == '#' || *p == '0') p++;

        // width
        while (*p >= '0' && *p <= '9') p++;
        if (*p == '*') { (void)va_arg(ap_copy, int); p++; }

        // precision
        if (*p == '.') {
            p++;
            while (*p >= '0' && *p <= '9') p++;
            if (*p == '*') { (void)va_arg(ap_copy, int); p++; }
        }

        // length（简单跳过）
        if (*p == 'l' || *p == 'h' || *p == 'z' || *p == 't') {
            char c = *p++;
            if (*p == c) p++; // ll/hh
        }

        char spec = *p ? *p++ : '\0';
        Arg_t *a = &r->args[r->argc];

        switch (spec) {
        case 'd':
        case 'i':
            a->t = ARG_I32;
            a->v.i32 = va_arg(ap_copy, int);
            r->argc++;
            break;
        case 'u':
            a->t = ARG_U32;
            a->v.u32 = va_arg(ap_copy, unsigned int);
            r->argc++;
            break;
        case 'x':
        case 'X':
            a->t = ARG_HEX32;
            a->v.u32 = va_arg(ap_copy, unsigned int);
            r->argc++;
            break;
        case 'c':
            a->t = ARG_CHAR;
            a->v.ch = (char)va_arg(ap_copy, int);
            r->argc++;
            break;
        case 's':
            a->t = ARG_STR;
            a->v.s = va_arg(ap_copy, const char*);
            r->argc++;
            break;
        case 'f':
        case 'F':
            a->t = ARG_F64;
            a->v.f64 = va_arg(ap_copy, double); // float 提升为 double
            r->argc++;
            break;
        default:
            // 未识别：不消耗参数，避免错位
            break;
        }
    }
}


/**
 * @brief 获取当前时间戳（ms），用于日志前缀
 * @return 毫秒时间戳
 *
 * @note
 * - 调度器未启动前：xTaskGetTickCount() 可能不可靠，用 HAL_GetTick()
 * - 调度器启动后：用 RTOS tick * portTICK_PERIOD_MS
 */
static uint32_t log_now_ms_in_task(void)
{
    /* 调度器未启动时，xTaskGetTickCount() 不可靠/可能为 0 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return HAL_GetTick();
    }

    TickType_t t = xTaskGetTickCount();
    return (uint32_t)(t * (TickType_t)portTICK_PERIOD_MS);
}
/* =========================
 * 内部：record -> string（在日志任务里调用）
 * 前缀格式：
 *   无tag: [INFO ][123456] xxxx
 *   有tag: [INFO ][123456][CTRL] xxxx
 * ========================= */

/**
 * @brief 将 LogRec_t 格式化为输出字符串（带前缀与 \r\n）
 *
 * @param[out] out     输出缓冲区
 * @param[in]  out_sz  输出缓冲区大小
 * @param[in]  r       日志记录
 * @return 实际写入长度（包含 \r\n，不包含末尾 '\0' 的长度不严格要求）
 *
 * @note
 * - 这里不直接使用 vsnprintf：因为参数已被打包成 Arg_t 数组
 * - 会跳过 fmt 的 flags/width/precision/length，保证与 parse_args 同步
 */
static int format_line(char *out, int out_sz, const LogRec_t *r)
{
    if (!out || out_sz <= 0) return 0;

    uint32_t ts = log_now_ms_in_task(); // ms

    int off;
    if (r->tag && r->tag[0]) {
        off = snprintf(out, (size_t)out_sz, "[%s][%lu][%s] ",
                       lvl_str((LogLevel_t)r->lvl),
                       (unsigned long)ts,
                       r->tag);
    } else {
        off = snprintf(out, (size_t)out_sz, "[%s][%lu] ",
                       lvl_str((LogLevel_t)r->lvl),
                       (unsigned long)ts);
    }
    if (off < 0) return 0;
    if (off >= out_sz) return out_sz - 1;

    const char *p = r->fmt ? r->fmt : "";
    uint8_t ai = 0;

    while (*p && off < out_sz - 1) {
        if (*p != '%') {
            out[off++] = *p++;
            continue;
        }
        p++; // '%'

        if (*p == '%') { out[off++] = '%'; p++; continue; }

        // 跳过 flags/width/precision/length（与 parse 同步）
        while (*p == '-' || *p == '+' || *p == ' ' || *p == '#' || *p == '0') p++;
        while (*p >= '0' && *p <= '9') p++;
        if (*p == '*') p++;
        if (*p == '.') { p++; while (*p >= '0' && *p <= '9') p++; if (*p == '*') p++; }
        if (*p == 'l' || *p == 'h' || *p == 'z' || *p == 't') { char c = *p++; if (*p == c) p++; }

        char spec = *p ? *p++ : '\0';

        if (ai >= r->argc) {
            out[off++] = '?';
            continue;
        }
        const Arg_t *a = &r->args[ai++];

        int n = 0;
        switch (spec) {
        case 'd':
        case 'i':
            n = (a->t == ARG_I32)   ? snprintf(out + off, (size_t)(out_sz - off), "%d", (int)a->v.i32)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 'u':
            n = (a->t == ARG_U32)   ? snprintf(out + off, (size_t)(out_sz - off), "%u", (unsigned)a->v.u32)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 'x':
            n = (a->t == ARG_HEX32) ? snprintf(out + off, (size_t)(out_sz - off), "%x", (unsigned)a->v.u32)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 'X':
            n = (a->t == ARG_HEX32) ? snprintf(out + off, (size_t)(out_sz - off), "%X", (unsigned)a->v.u32)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 'c':
            n = (a->t == ARG_CHAR)  ? snprintf(out + off, (size_t)(out_sz - off), "%c", a->v.ch)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 's':
            n = (a->t == ARG_STR)   ? snprintf(out + off, (size_t)(out_sz - off), "%s", a->v.s ? a->v.s : "(null)")
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        case 'f':
        case 'F':
            n = (a->t == ARG_F64)   ? snprintf(out + off, (size_t)(out_sz - off), "%f", a->v.f64)
                                    : snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        default:
            n = snprintf(out + off, (size_t)(out_sz - off), "?");
            break;
        }

        if (n < 0) break;
        if (n >= (out_sz - off)) { off = out_sz - 1; break; }
        off += n;
    }

    if (off < out_sz - 2) {
        out[off++] = '\r';
        out[off++] = '\n';
    }
    out[off] = '\0';
    return off;
}

/* =========================
 * 对外接口
 * ========================= */

/**
 * @brief 日志模块初始化：创建静态队列并清空 ring 状态
 *
 * @note
 * - 若尚未创建队列，会用 xQueueCreateStatic() 创建
 * - 同时清空 ring 与 DMA 状态
 */
void Log_Init(void)
{
    if (!s_q) {
        s_q = xQueueCreateStatic(LOG_Q_DEPTH, sizeof(LogRec_t), s_qbuf, &s_qcb);
    }

    taskENTER_CRITICAL();
    s_head = 0;
    s_tail = 0;
    s_dma_len = 0;
    s_dma_busy = 0;
    s_runtime_level = LOG_RUNTIME_LEVEL;
    taskEXIT_CRITICAL();
}


/**
 * @brief 设置运行时日志等级
 * @param[in] lvl 新等级
 */
void Log_SetLevel(LogLevel_t lvl) { s_runtime_level = lvl; }

/**
 * @brief 获取运行时日志等级
 * @return 当前等级
 */
LogLevel_t Log_GetLevel(void) { return s_runtime_level; }

/**
 * @brief 业务侧投递一条日志（非阻塞）
 *
 * @param[in] lvl 日志等级
 * @param[in] tag 可选标签（NULL 或 "" 表示无 tag）
 * @param[in] fmt 格式字符串（建议字符串常量）
 * @param[in] ... 参数（支持 %d/%u/%x/%c/%s/%f）
 * @return true  投递成功或被等级过滤（视为成功）
 * @return false 队列未初始化 / fmt 为空 / 队列已满导致投递失败
 *
 * @note
 * - 本函数不进行字符串格式化，仅解析 fmt 并打包参数
 * - 队列满会丢弃：避免拖慢业务任务
 */
bool Log_Post(LogLevel_t lvl, const char *tag, const char *fmt, ...)
{
    if (!s_q || !fmt) return false;

    // 运行时等级过滤：数字越大越“啰嗦”
    if (lvl > s_runtime_level) return true;

    LogRec_t r;
    memset(&r, 0, sizeof(r));
    r.lvl = (uint8_t)lvl;
    r.tag = tag;  // 可选：NULL 表示无 tag
    r.fmt = fmt;

    va_list ap;
    va_start(ap, fmt);
    va_list ap2;
    va_copy(ap2, ap);
    parse_args(&r, ap2);
    va_end(ap2);
    va_end(ap);

    // 非阻塞：队列满就丢（不拖慢业务任务）
    return (xQueueSend(s_q, &r, 0) == pdPASS);
}

/**
 * @brief 日志任务入口：从队列取 LogRec_t，格式化为字符串，写入 DMA ring
 *
 * @param[in] argument 任务参数（未使用）
 *
 * @note
 * - line 使用 static，避免占用任务栈
 * - rb_write_locked 必须在临界区内调用
 */
void KaoYaApp_LogTask(void *argument)
{
    (void)argument;
    if (!s_q) Log_Init();

    LogRec_t r;
    static char line[LOG_LINE_MAX]; // static：更稳，不占任务栈

    for (;;) {
		MON_HB_LOG();
        if (xQueueReceive(s_q, &r, portMAX_DELAY) == pdPASS) {
            int n = format_line(line, (int)sizeof(line), &r);
            if (n > 0) {
                taskENTER_CRITICAL();
                (void)rb_write_locked((const uint8_t*)line, (uint16_t)n);
                taskEXIT_CRITICAL();
            }
        }
    }
}

/**
 * @brief UART2 DMA 发送完成 ISR 转发：推进 tail 并继续发送下一段
 *
 * @note
 * - 必须在 ISR 环境调用（或确保不会与其它上下文并发）
 * - 这里使用 taskENTER_CRITICAL_FROM_ISR 保护 ring 状态
 */
void Log_Uart2TxCpltISR(void)
{
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();

    uint16_t tail = s_tail;
    uint16_t adv  = s_dma_len;

    s_tail = (uint16_t)((tail + adv) % LOG_RB_SIZE);
    s_dma_len = 0;
    s_dma_busy = 0;

    try_start_dma_locked();

    taskEXIT_CRITICAL_FROM_ISR(saved);
}

/* ============================================================
 * HAL 回调
 *
 * 全工程只能有一个 HAL_UART_TxCpltCallback 定义 !!!
 * 如果别处已定义，请删除本函数，并在你已有回调里转调：
 *   if (huart->Instance == USART2) Log_Uart2TxCpltISR();
 *   if (huart->Instance == USART1) UplinkProto_OnUartTxCplt(huart);
 * ============================================================ */

/**
 * @brief HAL UART DMA 发送完成回调（示例实现）
 * @param[in] huart UART 句柄
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart && huart->Instance == USART2) {
        Log_Uart2TxCpltISR();
    }
	else if (huart->Instance == USART1) {
        UplinkProto_OnUartTxCplt(huart);
    }
}

/* =========================
 * PANIC：保底标志
 * ========================= */

/** @brief panic 标志：1 表示进入 panic 模式（改走轮询直打） */
static volatile uint8_t s_panic = 0;


/**
 * @brief 进入 panic 模式
 * @note 常用于 HardFault / stack overflow / assert 等场景
 */
void Log_EnterPanic(void) { s_panic = 1; }

/**
 * @brief 查询是否处于 panic 模式
 * @return true  panic 模式
 * @return false 非 panic
 */
bool Log_IsPanic(void)    { return (s_panic != 0); }

/**
 * @brief 安全计算字符串长度（最多扫描 max_len）
 * @param[in] s        字符串指针
 * @param[in] max_len  最大扫描长度
 * @return 实际长度（<= max_len）
 */
static size_t safe_strlen(const char *s, size_t max_len)
{
    size_t n = 0;
    if (!s) return 0;
    while (n < max_len && s[n] != '\0') n++;
    return n;
}

/* =========================
 * PANIC：USART2 轮询阻塞输出（不依赖RTOS/DMA/中断）
 * STM32F407: SR/DR
 * ========================= */

/**
 * @brief UART2 轮询输出单字符（阻塞）
 * @param[in] c 字符
 *
 * @note
 * - PANIC 模式使用：不依赖 DMA 与中断
 * - 直接访问 USART2 寄存器，确保最小依赖
 */
static inline void uart2_poll_putc(char c)
{
    USART_TypeDef *U = USART2;
    while ((U->SR & USART_SR_TXE) == 0) { }
    U->DR = (uint8_t)c;
    while ((U->SR & USART_SR_TC) == 0) { }
}

/**
 * @brief UART2 轮询输出字符串（阻塞）
 * @param[in] s 字符串
 */
static void uart2_poll_puts(const char *s)
{
    if (!s) return;
    while (*s) {
        char c = *s++;
        if (c == '\n') uart2_poll_putc('\r');
        uart2_poll_putc(c);
    }
}


/**
 * @brief panic 下尽力格式化一条 ERROR 并直打串口
 *
 * @param[in] tag 标签
 * @param[in] fmt 格式串
 * @param[in] ap  变参
 *
 * @note
 * - 仅用于 panic 通道（阻塞输出）
 * - 不依赖队列、任务、DMA、中断
 */
static void panic_error_vprint(const char *tag, const char *fmt, va_list ap)
{
    char line[LOG_LINE_MAX];

    int off;
    if (tag && tag[0]) {
        off = snprintf(line, sizeof(line), "[ERROR][%s] ", tag);
    } else {
        off = snprintf(line, sizeof(line), "[ERROR] ");
    }
    if (off < 0) return;
    if (off >= (int)sizeof(line)) off = (int)sizeof(line) - 1;

    vsnprintf(line + off, sizeof(line) - (size_t)off, (fmt ? fmt : ""), ap);

    size_t used = safe_strlen(line, sizeof(line));
    if (used + 2 < sizeof(line)) {
        line[used++] = '\r';
        line[used++] = '\n';
        line[used]   = '\0';
    } else {
        line[sizeof(line) - 3] = '\r';
        line[sizeof(line) - 2] = '\n';
        line[sizeof(line) - 1] = '\0';
    }

    uart2_poll_puts(line);
}

/**
 * @brief 对外：panic ERROR 直打（阻塞）
 *
 * @param[in] tag 标签
 * @param[in] fmt 格式串
 * @param[in] ... 参数
 *
 * @note
 * - 建议仅在 Log_IsPanic()==true 时调用
 * - 非 panic 日常日志请用 Log_Post()
 */
void Log_Error(const char *tag, const char *fmt, ...)
{
    if (!fmt) return;

    va_list ap;
    va_start(ap, fmt);
    panic_error_vprint(tag, fmt, ap);
    va_end(ap);
}
