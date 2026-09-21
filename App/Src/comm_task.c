/**
 ****************************************************************************************************
 * @file        comm_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信任务模块（UART1 + DMA + IDLE + StreamBuffer）
 * @details
 * 				通信架构说明：
 *
 *  			UART1 (DMA 循环接收)
 *			        ↓
 * 				IDLE 中断（判断“帧间隙”）
 *        			↓
 *  			DMA 新数据段 → StreamBuffer（ISR -> Task）
 *        			↓
 *  			CommTask（阻塞等待）
 *        			↓
 *  			CommParser（状态机解析）
 *        			↓
 *  			on_frame_cb()（处理完整有效帧）
 *        			↓
 *  			更新最新 CmdVel + 通知 ControlTask
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */

#include "comm_task.h"
#include "control_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "monitor_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "iwdg_task.h"
#include <string.h>
#include "log_task.h"
#include "protocol.h"
#include "comm_parser.h"

/* ============================================================================ */
/* UART1 DMA / StreamBuffer 配置                                                */
/* ============================================================================ */

/** UART1 DMA 环形接收缓冲区大小（字节） */
#define UART1_RX_DMA_SIZE       (256u)

/** StreamBuffer 深度（字节），需 >= DMA 缓冲 */
#define UART1_RX_SB_SIZE        (512u)

/** StreamBuffer 触发阈值（1 字节即可唤醒任务） */
#define UART1_RX_TRIG_LEVEL     (1u)

/** UART1 DMA 环形接收缓冲区（模块私有） */
static uint8_t uart1_rx_dma_buf[UART1_RX_DMA_SIZE];

/** UART1 接收 StreamBuffer 句柄（ISR -> CommTask） */
static StreamBufferHandle_t uart1_rx_stream = NULL;

/* ---------------- DMA / IDLE 教学与调试统计 ---------------- */
#if DMA_KAOYATEACH_ENABLE
/** IDLE 中断触发次数 */
static volatile uint32_t s_idle_irq_cnt = 0;

/** 累计推送到 StreamBuffer 的字节数 */
static volatile uint32_t s_idle_push_bytes = 0;

/** 最近一次推送长度 */
static volatile uint16_t s_last_len = 0;

/** 最近一次 DMA 当前指针 */
static volatile uint16_t s_last_curr = 0;

/** 上一次 DMA 指针位置 */
static volatile uint16_t s_last_last = 0;
#endif
/* ============================================================================
 * CommTask 内部状态：parser + 最新 cmd_vel
 * ========================================================================== */

/** 通信解析器实例（仅在 CommTask 上下文使用） */
static comm_parser_t s_parser;

/**
 * @brief 最新一条“已校验通过”的速度指令
 *
 * @note
 *  - 由 CommTask 更新
 *  - ControlTask / 其他任务只读
 *  - 通过临界区整体拷贝，避免撕裂读
 */
static CmdVel_t s_latest_cmd = {0};

/* ============================================================================
 * 内部函数声明
 * ========================================================================== */
/**
 * @brief 解析到完整有效帧后的回调
 */
static void on_frame_cb(const proto_frame_t *f, void *user);

/* ============================================================================ */
/* Comm 模块初始化                                                             */
/* ============================================================================ */

/**
 * @brief   Comm 模块初始化
 *
 * @details
 * 调用顺序要求：
 *  1. 创建 StreamBuffer
 *  2. 启动 UART1 DMA 循环接收
 *  3. 使能 UART1 IDLE 中断
 *
 * @note
 *  必须先创建 StreamBuffer，再开启 DMA/IDLE，
 *  否则 IDLE 中断先到会访问空句柄。
 */
void KaoYaApp_CommInit(void)
{
	/* 创建 StreamBuffer：用于 ISR -> CommTask 数据传递 */
    uart1_rx_stream = xStreamBufferCreate(UART1_RX_SB_SIZE, UART1_RX_TRIG_LEVEL);
    configASSERT(uart1_rx_stream != NULL);

    /* 启动 UART1 DMA 循环接收（环形缓冲模式） */
    (void)HAL_UART_Receive_DMA(&huart1, uart1_rx_dma_buf, UART1_RX_DMA_SIZE);
	
	LOGKAOYA_T("DMA", "UART1 DMA RX START, size=%u", (unsigned)UART1_RX_DMA_SIZE);

    /* 使能 UART IDLE 中断，用于判断一段数据“接收完成” */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* ============================================================================ */
/* 对外接口：获取最新 CmdVel（线程安全）                                       */
/* ============================================================================ */

/**
 * @brief   获取最新速度指令
 *
 * @details
 * 通过临界区保护结构体整体拷贝，避免并发撕裂读。
 */
CmdVel_t Comm_GetLatestCmdVel(void)
{
    CmdVel_t out;
	
    taskENTER_CRITICAL();
    out = s_latest_cmd;
    taskEXIT_CRITICAL();
	
    return out;
}

/* ============================================================================ */
/* 内部回调：处理完整协议帧                                                     */
/* ============================================================================ */

/**
 * @brief   收到完整、CRC 校验通过的协议帧后的回调
 *
 * @note
 *  - 回调运行在 CommTask 上下文（非中断）
 *  - 只做轻量处理：解码 + 更新缓存 + 通知控制任务
 */
static void on_frame_cb(const proto_frame_t *f, void *user)
{
    (void)user;

    /* 处理CMD_VEL消息 */
    if (f->msg_id == PROTO_MSG_CMD_VEL)
    {
        /* 严格校验 payload 长度，防止误解析 */
		if (f->len != PROTO_CMD_VEL_PAYLOAD_LEN) return;

		/* 按协议小端读取 int16：vx, vy, wz（避免结构体对齐/padding 风险） */
		const int16_t vx = proto_rd_i16(f->payload, 0);
		const int16_t vy = proto_rd_i16(f->payload, 2);
		const int16_t wz = proto_rd_i16(f->payload, 4);

		/* 构造 CmdVel */
		CmdVel_t v = {0};
		v.vx_mmps   = vx;
		v.vy_mmps   = vy;
		v.wz_mradps = wz;
		v.rx_tick   = xTaskGetTickCount();
		v.valid     = true;

		/* 更新最新指令（临界区保护） */
		taskENTER_CRITICAL();
		s_latest_cmd = v;
		taskEXIT_CRITICAL();

		/* 通知 ControlTask：有新指令到达 */
		if (g_controlTaskHandle)
		{
			xTaskNotify(g_controlTaskHandle, CTRL_NOTIF_NEW_CMD, eSetBits);
		}
			
    }
    else{/* TODO：后续可扩展处理 PROTO_MSG_STATUS / PROTO_MSG_ACK 等 */} 
}

/* ============================================================================ */
/* FreeRTOS 任务：CommTask                                                      */
/* ============================================================================ */

/**
 * @brief   通信任务主循环
 *
 * @details
 *  - 阻塞等待 StreamBuffer 中的数据
 *  - 取出字节流喂给解析器
 *  - 解析器内部通过状态机拼帧
 */
void KaoYaApp_CommTask(void *argument)
{
    (void)argument;
	
	/* 防御：StreamBuffer 必须已创建 */
    configASSERT(uart1_rx_stream != NULL);
	
    /* 初始化解析器，并注册帧回调 */
    CommParser_Init(&s_parser, on_frame_cb, NULL);

    /* 接收缓冲区：一次最多处理 64 字节 */
    uint8_t chunk[64];
	
    for (;;)
    {
		/* 任务心跳：用于监控任务存活 */
        MON_HB_COMM();
		
        /* 阻塞等待 StreamBuffer 数据 */
        size_t n = xStreamBufferReceive(uart1_rx_stream, chunk, sizeof(chunk), portMAX_DELAY);
		
#if DMA_KAOYATEACH_ENABLE                                 
		LOGKAOYA_T("DMA",
               "idle_cnt=%lu push_bytes=%lu last_len=%u last_pos=%u->%u ndtr=%u",
               s_idle_irq_cnt,
               s_idle_push_bytes,
               (unsigned)s_last_len,
               (unsigned)s_last_last,
               (unsigned)s_last_curr,
               (unsigned)__HAL_DMA_GET_COUNTER(huart1.hdmarx));
#endif
			   
        if (n > 0u)
        {
#if IWDG_KAOYATEACH_ENABLE
            HAL_Delay(5000);
#endif
			/* 正常模式：通信任务完成一次有效运行后更新心跳 */
            IWDG_Heartbeat(IWDG_ID_COMM);
			
			/* 将字节流送入解析器状态机 */
            (void)CommParser_Feed(&s_parser, chunk, n);
        }
    }
}

/* ============================================================================ */
/* UART1 IDLE 回调：DMA → StreamBuffer                                          */
/* ============================================================================ */

/**
 * @brief   UART1 IDLE 中断回调
 *
 * @details
 * 功能：
 *  - 计算 DMA 当前写指针
 *  - 找出“新接收的数据段”
 *  - 推送到 StreamBuffer
 *  - 唤醒 CommTask
 *
 * @note
 *  - 运行在 ISR 上下文
 *  - 禁止做复杂逻辑 / 阻塞操作
 */
void KaoYaApp_Uart1IdleCallback(void)
{
	/* 启动阶段防御：StreamBuffer 可能尚未创建 */
    if (uart1_rx_stream == NULL) return;
	
    static uint16_t last_pos = 0;

    /* 计算 DMA 当前写指针位置 */
    uint16_t curr_pos = (uint16_t)(UART1_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
	
    /* 指针未变化，说明没有新数据 */
    if (curr_pos == last_pos) return;
#if DMA_KAOYATEACH_ENABLE
	s_idle_irq_cnt++;
#endif
    BaseType_t hpw = pdFALSE;
	uint16_t total_len = 0;
	
    if (curr_pos > last_pos)
    {
        /* DMA 未回绕：直接发送 last_pos ~ curr_pos */
        uint16_t len = (uint16_t)(curr_pos - last_pos);
		total_len = len;
        (void)xStreamBufferSendFromISR(uart1_rx_stream, &uart1_rx_dma_buf[last_pos], len, &hpw);                                
    }
    else
    {
        /* DMA 回绕：
         * 1) last_pos ~ buffer_end
         * 2) buffer_start ~ curr_pos
         */
        uint16_t len1 = (uint16_t)(UART1_RX_DMA_SIZE - last_pos);
		total_len = len1;
        (void)xStreamBufferSendFromISR(uart1_rx_stream, &uart1_rx_dma_buf[last_pos], len1, &hpw);

        if (curr_pos > 0u)
        {
			total_len = (uint16_t)(total_len + curr_pos);
            (void)xStreamBufferSendFromISR(uart1_rx_stream, &uart1_rx_dma_buf[0], curr_pos, &hpw);                                 
        }
    }
#if DMA_KAOYATEACH_ENABLE	
	s_idle_push_bytes += total_len;
    s_last_len  = total_len;
    s_last_curr = curr_pos;
    s_last_last = last_pos;
#endif    
	/* 更新 DMA 位置基准 */
	last_pos = curr_pos;

    /* 若唤醒了更高优先级任务，立即切换 */
    portYIELD_FROM_ISR(hpw);
}
