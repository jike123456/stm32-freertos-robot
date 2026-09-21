/**
 ****************************************************************************************************
 * @file        comm_parser.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       串口通信协议解析器（字节流 -> 帧），状态机解析 + CRC16 校验 + 帧回调
 * @details
 *  本文件实现 STM32F407 下位机与上位机之间的统一串口协议解析（RX 侧）：
 *   - 输入：任意分片/任意长度的字节流（DMA 环形缓冲 / StreamBuffer / 中断回调等）
 *   - 输出：当完整帧通过 SOF/VER/LEN/EOF/CRC 校验后，触发 on_frame() 回调上抛业务层
 *
 *  设计要点：
 *   1) 状态机逐字节推进（支持粘包/拆包/噪声）
 *      - FeedByte()：喂 1 字节推进状态机
 *      - Feed()    ：喂一段 buffer（内部逐字节调用 FeedByte）
 *   2) 帧同步能力
 *      - SOF0/SOF1 双字节帧头同步
 *      - 支持 “AA AA 55” 重叠场景（保持 WAIT_SOF1，避免漏帧）
 *   3) CRC16 校验策略
 *      - CRC 从 VER 字段开始累计（不包含 SOF，不包含 CRC 字段本身，不包含 EOF）
 *      - 计算范围：VER/MSG/FLAGS/SEQ/LEN_L/LEN_H/PAYLOAD(LEN)
 *      - 校验在 EOF1 收到后进行最终对比，失败则复位并继续同步下一帧
 *   4) 鲁棒性与可调试
 *      - 对 VER/LEN/EOF/CRC 异常均给出错误码并 Reset
 *      - Reset 后继续解析后续字节流（适用于噪声/错位/丢字节场景）
 *      - 代码内大量 LOGKAOYA_T() 用于联调阶段定位解析过程（量产建议降级/条件编译）
 *
 *  协议帧格式（小端）：
 *    SOF0(0xAA) SOF1(0x55) VER(1) MSG_ID(1) FLAGS(1) SEQ(1)
 *    LEN(2, little-endian) PAYLOAD(LEN) CRC16(2, little-endian) EOF0(0x66) EOF1(0xBB)
 *
 *  典型使用场景：
 *   - UART DMA + IDLE 中断：从 DMA 环形缓冲取出新增数据 -> CommParser_Feed()
 *   - StreamBuffer/Queue：CommTask 收到 bytes -> CommParser_Feed()
 *   - 上层回调 on_frame() 内按 msg_id 分发到对应业务处理（cmd_vel / 参数下发 / 心跳等） 
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       STM32F407 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 ****************************************************************************************************
 */
#include "comm_parser.h"
#include "log_task.h"

/* ============================================================================
 * 内部辅助：解析成功后“出帧并复位”
 * ========================================================================== */
static void emit_frame_and_reset(comm_parser_t *parser)
{
    /* 到这里代表 SOF/VER/LEN/CRC/EOF 都已通过校验 */
    if (parser->on_frame != NULL)
    {
        parser->on_frame(&parser->frame, parser->user_ctx);
    }

    /* 继续同步解析下一帧 */
    CommParser_Reset(parser);
}

/* ============================================================================
 * 初始化
 * ========================================================================== */
void CommParser_Init(comm_parser_t *parser, comm_on_frame_fn on_frame, void *user_ctx)
{
    if (parser == NULL) return;

    parser->on_frame = on_frame;
    parser->user_ctx = user_ctx;

    CommParser_Reset(parser);
}

/* ============================================================================
 * 复位
 * ========================================================================== */
void CommParser_Reset(comm_parser_t *parser)
{
    if (parser == NULL) return;

    parser->state = COMM_ST_WAIT_SOF0;

    parser->payload_offset = 0;

    /* 真正开始算 CRC 是在 SOF 匹配完整后（进入 VER 时） */
    parser->crc_calc = Proto_Crc16CcittFalseInit();

    parser->crc_bytes[0] = 0u;
    parser->crc_bytes[1] = 0u;
    parser->crc_recv = 0u;

    /* 当前帧字段初始化（payload 不清零，len 决定有效范围） */
    parser->frame.ver   = PROTO_VER;
    parser->frame.msg_id = 0u;
    parser->frame.flags  = 0u;
    parser->frame.seq    = 0u;
    parser->frame.len    = 0u;
	
#if COMMPARSER_KAOYATEACH_ENABLE
	LOGKAOYA_T("CommParser", "【解析器】复位：进入 WAIT_SOF0，开始同步帧头(AA 55)");
#endif
}

/* ============================================================================
 * 核心：喂 1 字节推进状态机
 * ========================================================================== */
comm_parse_status_t CommParser_FeedByte(comm_parser_t *parser, uint8_t byte)
{
    if (parser == NULL) return COMM_PARSE_NEED_MORE;
	
#if COMMPARSER_KAOYATEACH_ENABLE
	LOGKAOYA_T("CommParser",
			   "【解析器】喂字节：st=%d byte=0x%02X | len=%u off=%u | crc_calc=0x%04X",
			   (int)parser->state,
			   (unsigned)byte,
			   (unsigned)parser->frame.len,
			   (unsigned)parser->payload_offset,
			   (unsigned)parser->crc_calc);
#endif
			   
    switch (parser->state)
    {
    /* ---------------------------------------------------------------------- */
    /* 1) 等待 SOF0=0xAA                                                        */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_WAIT_SOF0:
        if (byte == PROTO_SOF0)
        {	
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】同步：命中 SOF0=0x%02X -> 等待 SOF1=0x%02X",
					   (unsigned)byte, (unsigned)PROTO_SOF1);
#endif
            parser->state = COMM_ST_WAIT_SOF1;
        }
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 2) 等待 SOF1=0x55                                                        */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_WAIT_SOF1:
        if (byte == PROTO_SOF1)
        {
            /* 新帧开始：CRC 从 VER 开始算 */
            parser->crc_calc = Proto_Crc16CcittFalseInit();
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】同步：命中 SOF1=0x%02X，新帧开始 -> 读取 VER（CRC 从此处开始累计）",
					   (unsigned)byte);
#endif
            parser->state = COMM_ST_VER;
        }
        else if (byte == PROTO_SOF0)
        {
            /* 处理 “AA AA 55” 重叠：保持在等待 SOF1 */
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】同步：检测到重叠 AA AA 55，保持 WAIT_SOF1 继续等 0x55");
#endif
            parser->state = COMM_ST_WAIT_SOF1;
        }
        else
        {
            /* SOF1 不匹配，重新找 SOF0 */
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】同步失败：SOF1 期望=0x%02X，实际=0x%02X -> 回到 WAIT_SOF0 重新找帧头",
					   (unsigned)PROTO_SOF1, (unsigned)byte);
#endif

            parser->state = COMM_ST_WAIT_SOF0;
        }
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 3) VER                                                                   */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_VER:
        parser->frame.ver = byte;
		
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：VER=0x%02X", (unsigned)byte);
#endif
	
        if (parser->frame.ver != PROTO_VER)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser",
				   "【解析器】版本错误：期望 VER=0x%02X，实际=0x%02X -> 复位并重新同步",
				   (unsigned)PROTO_VER, (unsigned)byte);
#endif
			
            CommParser_Reset(parser);
            return COMM_PARSE_ERR_VER;
        }

        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】VER 校验通过 -> 读取 MSG_ID（crc_calc=0x%04X）",
				   (unsigned)parser->crc_calc);
#endif
        parser->state = COMM_ST_MSG;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 4) MSG_ID                                                                */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_MSG:
        parser->frame.msg_id = byte;
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：MSG_ID=0x%02X -> FLAGS（crc_calc=0x%04X）",
				   (unsigned)byte, (unsigned)parser->crc_calc);
#endif
        parser->state = COMM_ST_FLAGS;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 5) FLAGS                                                                 */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_FLAGS:
        parser->frame.flags = byte;
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：FLAGS=0x%02X -> SEQ（crc_calc=0x%04X）",
				   (unsigned)byte, (unsigned)parser->crc_calc);
#endif
        parser->state = COMM_ST_SEQ;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 6) SEQ                                                                   */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_SEQ:
        parser->frame.seq = byte;
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：SEQ=%u -> LEN_L（crc_calc=0x%04X）",
				   (unsigned)byte, (unsigned)parser->crc_calc);
#endif
        parser->state = COMM_ST_LEN_L;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 7) LEN_L                                                                 */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_LEN_L:
        parser->frame.len = (uint16_t)byte;
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：LEN_L=0x%02X -> LEN_H（crc_calc=0x%04X）",
				   (unsigned)byte, (unsigned)parser->crc_calc);
#endif
        parser->state = COMM_ST_LEN_H;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 8) LEN_H + 合法性检查                                                    */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_LEN_H:
        parser->frame.len |= (uint16_t)((uint16_t)byte << 8);
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
		
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：LEN_H=0x%02X -> LEN=%u（crc_calc=0x%04X）",
				   (unsigned)byte, (unsigned)parser->frame.len, (unsigned)parser->crc_calc);
#endif

        if (parser->frame.len > PROTO_MAX_PAYLOAD)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】长度错误：LEN=%u > MAX=%u -> 复位并重新同步",
				   (unsigned)parser->frame.len, (unsigned)PROTO_MAX_PAYLOAD);
#endif
            CommParser_Reset(parser);
            return COMM_PARSE_ERR_LEN;
        }

        parser->payload_offset = 0u;
		
        if (parser->frame.len == 0u)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】LEN=0：无Payload -> 读取 CRC_L");
#endif
            parser->state = COMM_ST_CRC_L;
        }
        else
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】LEN=%u：开始接收 PAYLOAD[0..%u)",
					   (unsigned)parser->frame.len, (unsigned)parser->frame.len);
#endif
            parser->state = COMM_ST_PAYLOAD;
        }
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 9) PAYLOAD                                                               */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_PAYLOAD:
        parser->frame.payload[parser->payload_offset++] = byte;
        parser->crc_calc = Proto_Crc16CcittFalseUpdateByte(parser->crc_calc, byte);
		
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】PAYLOAD[%u]=0x%02X（crc_calc=0x%04X）",
				   (unsigned)(parser->payload_offset - 1u), (unsigned)byte, (unsigned)parser->crc_calc);
#endif

        if (parser->payload_offset >= parser->frame.len)
        {	
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】PAYLOAD 接收完成：off=%u len=%u -> 读取 CRC_L",
					   (unsigned)parser->payload_offset, (unsigned)parser->frame.len);
#endif
            parser->state = COMM_ST_CRC_L;
        }
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 10) CRC_L                                                                */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_CRC_L:
        parser->crc_bytes[0] = byte;
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：CRC_L=0x%02X -> CRC_H", (unsigned)byte);
#endif
        parser->state = COMM_ST_CRC_H;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 11) CRC_H                                                                */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_CRC_H:
        parser->crc_bytes[1] = byte;
        parser->crc_recv = (uint16_t)parser->crc_bytes[0] |
                           (uint16_t)((uint16_t)parser->crc_bytes[1] << 8);
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】字段：CRC_H=0x%02X -> CRC_RECV=0x%04X -> 等待 EOF0=0x%02X",
				   (unsigned)byte, (unsigned)parser->crc_recv, (unsigned)PROTO_EOF0);
#endif
        parser->state = COMM_ST_EOF0;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 12) EOF0=0x66                                                            */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_EOF0:
        if (byte != PROTO_EOF0)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】帧尾错误：EOF0 期望=0x%02X 实际=0x%02X -> 复位并重新同步",
					   (unsigned)PROTO_EOF0, (unsigned)byte);
#endif
            CommParser_Reset(parser);
            return COMM_PARSE_ERR_EOF;
        }
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】帧尾：EOF0=0x%02X OK -> 等待 EOF1=0x%02X",
				   (unsigned)byte, (unsigned)PROTO_EOF1);
#endif
        parser->state = COMM_ST_EOF1;
        return COMM_PARSE_NEED_MORE;

    /* ---------------------------------------------------------------------- */
    /* 13) EOF1=0xBB + 最终 CRC 对比                                             */
    /* ---------------------------------------------------------------------- */
    case COMM_ST_EOF1:
        if (byte != PROTO_EOF1)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】帧尾错误：EOF1 期望=0x%02X 实际=0x%02X -> 复位并重新同步",
					   (unsigned)PROTO_EOF1, (unsigned)byte);
#endif
            CommParser_Reset(parser);
            return COMM_PARSE_ERR_EOF;
        }
		
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】帧尾：EOF1=0x%02X OK -> 开始 CRC 校验", (unsigned)byte);
#endif
		
        if (parser->crc_calc != parser->crc_recv)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】CRC错误：calc=0x%04X recv=0x%04X -> 复位并重新同步",
					   (unsigned)parser->crc_calc, (unsigned)parser->crc_recv);
#endif
            CommParser_Reset(parser);
            return COMM_PARSE_ERR_CRC;
        }
		
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser",
				   "【解析器】帧解析成功：MSG=0x%02X FLAGS=0x%02X SEQ=%u LEN=%u CRC=0x%04X",
				   (unsigned)parser->frame.msg_id,
				   (unsigned)parser->frame.flags,
				   (unsigned)parser->frame.seq,
				   (unsigned)parser->frame.len,
				   (unsigned)parser->crc_recv);
#endif
        emit_frame_and_reset(parser);
        return COMM_PARSE_OK;

    default:
#if COMMPARSER_KAOYATEACH_ENABLE
		LOGKAOYA_T("CommParser", "【解析器】未知状态：st=%d -> 复位并重新同步", (int)parser->state);
#endif
        CommParser_Reset(parser);
        return COMM_PARSE_NEED_MORE;
    }
}


/* ============================================================================
 * 喂一段 buffer
 * ========================================================================== */
size_t CommParser_Feed(comm_parser_t *parser, const uint8_t *data, size_t len)
{
    if ((parser == NULL) || (data == NULL) || (len == 0u))
    {
        return 0u;
    }

    size_t parsed_ok_count = 0u;
	
#if COMMPARSER_KAOYATEACH_ENABLE
	LOGKAOYA_T("CommParser", "【解析器】喂入缓冲区：len=%u", (unsigned)len);
#endif
	
    for (size_t idx = 0; idx < len; idx++)
    {
		comm_parse_status_t st = CommParser_FeedByte(parser, data[idx]);
        if (st == COMM_PARSE_OK)
        {
            parsed_ok_count++;
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser", "【解析器】本次缓冲解析出帧：ok_count=%u", (unsigned)parsed_ok_count);
#endif
        }
		else if (st != COMM_PARSE_NEED_MORE)
        {
#if COMMPARSER_KAOYATEACH_ENABLE
			LOGKAOYA_T("CommParser",
					   "【解析器】缓冲解析出错：err=%d idx=%u byte=0x%02X（已自动复位继续同步）",
					   (int)st, (unsigned)idx, (unsigned)data[idx]);
#endif
        }
        /* 注意：
         * - ERR_* 时 FeedByte 内部已 reset
         * - 继续循环可在同一 buffer 中继续同步并解析后续帧（粘包/噪声场景）
         */
    }
	
#if COMMPARSER_KAOYATEACH_ENABLE
	LOGKAOYA_T("CommParser", "【解析器】喂入结束：本buffer ok_count=%u", (unsigned)parsed_ok_count);
#endif
    return parsed_ok_count;
}
