/**
 ****************************************************************************************************
 * @file        uplink_proto.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       Uplink 协议封装：将状态帧打包到 buffer，并通过 USART1 DMA 发送（busy 则丢弃）
 * @details
 * - 本模块负责：
 *   1) 按协议格式组织 Status 帧（header + payload + crc + eof）
 *   2) 通过 USART1 DMA 发送（非阻塞）
 *   3) 提供 TxCplt/Error 回调转调接口，释放 busy
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       STM32F407 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */  
#include "uplink_proto.h"
#include "usart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"
#include "log_task.h"


/**
 * @brief USART1 DMA 发送忙标志
 *
 * @note
 * - 0：空闲；1：DMA 正在发送
 * - 使用 volatile：避免编译器优化导致的读写不一致
 * - 该标志只用于“是否允许发起一次 DMA 发送”
 */
static volatile uint8_t s_txBusy = 0;

/* ============================================================================ */
/* 内部：USART1 DMA 发送（busy 则直接丢弃，不阻塞）                              */
/* ============================================================================ */

/**
 * @brief 通过 USART1 发起 DMA 发送
 *
 * @param[in] buf  发送缓冲区指针
 * @param[in] len  发送长度（字节）
 * @return true  发起成功（DMA 已启动）
 * @return false  发起失败（busy/参数错误/HAL 返回非 OK）
 *
 * @note
 * - 该函数是“非阻塞 + 丢弃策略”：busy 时直接返回 false，不等待
 * - busy 检查必须放在临界区：避免多任务并发调用导致同时启动 DMA
 */
static bool uart1_send_dma(uint8_t *buf, uint16_t len)
{
	/* 参数检查：长度为 0 或 buf 为空都不发送 */
    if (len == 0u || !buf) return false;

    /* busy check：临界区避免多任务抢占 */
    taskENTER_CRITICAL();
    if (s_txBusy)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    s_txBusy = 1u;             /* 先置 busy，保证只有一个任务能进来 */
    taskEXIT_CRITICAL();

	/* 启动 DMA 发送：失败则回滚 busy */
    if (HAL_UART_Transmit_DMA(&huart1, buf, len) != HAL_OK)
    {
        s_txBusy = 0u;
        return false;
    }
    return true;
}

/* ============================================================================ */
/* 对外：回调释放 busy                                                           */
/* 注意：HAL 的 TxCplt/Error callback 是全局唯一的，你需要在那个地方转调          */
/* ============================================================================ */

/**
 * @brief USART Tx 完成回调转调接口（释放 busy）
 *
 * @param[in] huart HAL UART 句柄指针
 *
 * @note
 * - 需要在 `HAL_UART_TxCpltCallback()` 里调用本函数
 * - 只关心 USART1，其它串口忽略
 */
void UplinkProto_OnUartTxCplt(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART1)
    {
        s_txBusy = 0u;
    }
}

/**
 * @brief USART 错误回调转调接口（释放 busy）
 *
 * @param[in] huart HAL UART 句柄指针
 *
 * @note
 * - 需要在 `HAL_UART_ErrorCallback()` 里调用本函数
 * - 一旦 DMA/串口异常，必须释放 busy，否则会“永远发不出去”
 */
void UplinkProto_OnUartError(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == USART1)
    {
        s_txBusy = 0u;
    }
}

/**
 * @brief UplinkProto 模块初始化
 *
 * @note
 * - 目前仅清空 busy 标志
 * - 若后续需要统计丢包/发送次数等，也可以在这里初始化
 */
void UplinkProto_Init(void)
{
    s_txBusy = 0;
}

/* ============================================================================ */
/* 对外：发送 Status 帧                                                          */
/* ============================================================================ */

/**
 * @brief 发送状态帧（Status）
 *
 * @param[in,out] tx          协议发送上下文（包含 send_str 与 send_buffer）
 * @param[in]     seq         帧序号（由上层维护递增即可）
 * @param[in]     battery_mv  电池电压（mV）
 * @param[in]     temp        温度（建议：真实温度 *100）
 * @param[in]     vx          速度 x（按你协议定义的单位）
 * @param[in]     vy          速度 y（按你协议定义的单位）
 * @param[in]     vz          角速度 z / yaw rate（按你协议定义的单位）
 * @param[in]     gx          陀螺仪 x（按你协议定义的单位）
 * @param[in]     gy          陀螺仪 y（按你协议定义的单位）
 * @param[in]     gz          陀螺仪 z（按你协议定义的单位）
 * @param[in]     ax          加速度 x（按你协议定义的单位）
 * @param[in]     ay          加速度 y（按你协议定义的单位）
 * @param[in]     az          加速度 z（按你协议定义的单位）
 *
 * @return true  已成功启动 DMA 发送
 * @return false busy/参数错误/启动 DMA 失败
 *
 * @details
 * - 先填充 send_str（结构体字段），再手写序列化到 send_buffer（小端）
 * - CRC 计算范围：从 VER 开始到 PAYLOAD 结束（不含 SOF、CRC 自身、EOF）
 * - 最后调用 uart1_send_dma() 非阻塞发送
 */
bool UplinkProto_SendStatus(proto_uplink_t *tx, uint8_t seq,
                      uint16_t battery_mv, int16_t temp,
                      int16_t vx, int16_t vy, int16_t vz,
                      int16_t gx, int16_t gy, int16_t gz,
                      int16_t ax, int16_t ay, int16_t az,
											uint16_t status_flags)
{

	/* 上下文必须有效 */
    if (!tx) return 0;

    /* ====== 1) 填字段（结构体层面，便于调试/查看） ====== */
    tx->send_str.sof0   = PROTO_SOF0;
    tx->send_str.sof1   = PROTO_SOF1;
    tx->send_str.ver    = PROTO_VER;
    tx->send_str.msg_id = PROTO_MSG_STATUS;
    tx->send_str.flags  = PROTO_FLAG_DEFAULT;
    tx->send_str.seq    = seq;

	/* payload 长度（字节），由协议宏统一定义 */
    tx->send_str.len = PROTO_STATUS_PAYLOAD_LEN;

	/* payload 数据赋值 */
    tx->send_str.payload.battery_mv = battery_mv;
    tx->send_str.payload.temp       = temp;

    tx->send_str.payload.vx = vx;
    tx->send_str.payload.vy = vy;
    tx->send_str.payload.vz = vz;

    tx->send_str.payload.gx = gx;
    tx->send_str.payload.gy = gy;
    tx->send_str.payload.gz = gz;

    tx->send_str.payload.ax = ax;
    tx->send_str.payload.ay = ay;
    tx->send_str.payload.az = az;
		
		tx->send_str.payload.status_flags = status_flags;

	/* 帧尾 */
    tx->send_str.eof0 = PROTO_EOF0;
    tx->send_str.eof1 = PROTO_EOF1;

    /* ====== 2) 手写写入 send_buffer[]（线协议：全小端） ====== */
    uint8_t *send_buffer = tx->send_buffer;

    /* header：SOF0/SOF1/VER/MSG/FLAGS/SEQ */
    send_buffer[0] = tx->send_str.sof0;
    send_buffer[1] = tx->send_str.sof1;
    send_buffer[2] = tx->send_str.ver;
    send_buffer[3] = tx->send_str.msg_id;
    send_buffer[4] = tx->send_str.flags;
    send_buffer[5] = tx->send_str.seq;

    /* LEN：小端（低字节在前） */
    send_buffer[6] = (uint8_t)((uint16_t)tx->send_str.len & 0xFFu);
    send_buffer[7] = (uint8_t)(((uint16_t)tx->send_str.len >> 8) & 0xFFu);

    /* payload：从 byte[8] 开始，所有 16bit 字段均按小端写入 */
	
	/* batt */
    send_buffer[8]  = (uint8_t)((uint16_t)tx->send_str.payload.battery_mv & 0xFFu);
    send_buffer[9]  = (uint8_t)(((uint16_t)tx->send_str.payload.battery_mv >> 8) & 0xFFu);

    /* temp */
    send_buffer[10] = (uint8_t)((uint16_t)tx->send_str.payload.temp & 0xFFu);
    send_buffer[11] = (uint8_t)(((uint16_t)tx->send_str.payload.temp >> 8) & 0xFFu);

    /* vx */
    send_buffer[12] = (uint8_t)((uint16_t)tx->send_str.payload.vx & 0xFFu);
    send_buffer[13] = (uint8_t)(((uint16_t)tx->send_str.payload.vx >> 8) & 0xFFu);

    /* vy */
    send_buffer[14] = (uint8_t)((uint16_t)tx->send_str.payload.vy & 0xFFu);
    send_buffer[15] = (uint8_t)(((uint16_t)tx->send_str.payload.vy >> 8) & 0xFFu);

    /* vz */
    send_buffer[16] = (uint8_t)((uint16_t)tx->send_str.payload.vz & 0xFFu);
    send_buffer[17] = (uint8_t)(((uint16_t)tx->send_str.payload.vz >> 8) & 0xFFu);

    /* gx */
    send_buffer[18] = (uint8_t)((uint16_t)tx->send_str.payload.gx & 0xFFu);
    send_buffer[19] = (uint8_t)(((uint16_t)tx->send_str.payload.gx >> 8) & 0xFFu);

    /* gy */
    send_buffer[20] = (uint8_t)((uint16_t)tx->send_str.payload.gy & 0xFFu);
    send_buffer[21] = (uint8_t)(((uint16_t)tx->send_str.payload.gy >> 8) & 0xFFu);

    /* gz */
    send_buffer[22] = (uint8_t)((uint16_t)tx->send_str.payload.gz & 0xFFu);
    send_buffer[23] = (uint8_t)(((uint16_t)tx->send_str.payload.gz >> 8) & 0xFFu);

    /* ax */
    send_buffer[24] = (uint8_t)((uint16_t)tx->send_str.payload.ax & 0xFFu);
    send_buffer[25] = (uint8_t)(((uint16_t)tx->send_str.payload.ax >> 8) & 0xFFu);

    /* ay */
    send_buffer[26] = (uint8_t)((uint16_t)tx->send_str.payload.ay & 0xFFu);
    send_buffer[27] = (uint8_t)(((uint16_t)tx->send_str.payload.ay >> 8) & 0xFFu);

    /* az */
    send_buffer[28] = (uint8_t)((uint16_t)tx->send_str.payload.az & 0xFFu);
    send_buffer[29] = (uint8_t)(((uint16_t)tx->send_str.payload.az >> 8) & 0xFFu);
		
		/* status_flags：payload最后两个字节，小端格式 */
		send_buffer[30] =	(uint8_t)((uint16_t)tx->send_str.payload.status_flags & 0xFFu);
		send_buffer[31] =	(uint8_t)(((uint16_t)tx->send_str.payload.status_flags >> 8) & 0xFFu);

    /* ====== 3) CRC（范围：从 VER 开始到 PAYLOAD 结束） ======
     * 起始：send_buffer[2]（VER）
     * 长度：6(VER..LEN) + payload_len
     */
    uint16_t crc = Proto_Crc16CcittFalse(&send_buffer[2], (uint16_t)(6u + PROTO_STATUS_PAYLOAD_LEN));
    tx->send_str.crc = crc;

    /* CRC 小端 */
    send_buffer[32] = (uint8_t)(crc & 0xFFu);
    send_buffer[33] = (uint8_t)((crc >> 8) & 0xFFu);

    /* EOF */
    send_buffer[34] = tx->send_str.eof0;
    send_buffer[35] = tx->send_str.eof1;
	
	/* ====== 4) DMA 发送（busy 则丢弃，不阻塞） ====== */
	return uart1_send_dma(tx->send_buffer, (uint16_t)PROTO_STATUS_FRAME_LEN);
}
