/**
 ****************************************************************************************************
 * @file        comm_parser.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信帧解析器（字节流 -> 帧）
 * @license     Copyright (c) 2025-2035, Kaoya Project
 ****************************************************************************************************
 */
#ifndef COMM_PARSER_H
#define COMM_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 解析状态码
 * ========================================================================== */
typedef enum
{
    COMM_PARSE_OK = 0,          /* 成功解析出 1 帧（已触发回调并 reset） */
    COMM_PARSE_NEED_MORE = 1,   /* 还需要更多字节才能组成一帧 */

    /* 错误类：发生错误后 parser 会自动 reset 并继续找下一帧 */
    COMM_PARSE_ERR_VER = -1,    /* 协议版本不匹配 */
    COMM_PARSE_ERR_LEN = -2,    /* LEN 超过 PROTO_MAX_PAYLOAD */
    COMM_PARSE_ERR_CRC = -3,    /* CRC 校验失败 */
    COMM_PARSE_ERR_EOF = -4     /* EOF 不匹配 */
} comm_parse_status_t;

/* ============================================================================
 * 输出帧（通用结构，业务层再按 msg_id 解析 payload）
 * ========================================================================== */
typedef struct
{
    uint8_t  ver;                         /* 协议版本 */
    uint8_t  msg_id;                      /* 消息类型 */
    uint8_t  flags;                       /* 标志位 */
    uint8_t  seq;                         /* 序号 */
    uint16_t len;                         /* payload 长度（字节） */
    uint8_t  payload[PROTO_MAX_PAYLOAD];  /* payload 数据（有效长度由 len 决定） */
} proto_frame_t;

/* 解析到完整有效帧后的回调 */
typedef void (*comm_on_frame_fn)(const proto_frame_t *frame, void *user_ctx);

/* ============================================================================
 * 解析器状态机枚举（内部状态）
 * ========================================================================== */
typedef enum
{
    COMM_ST_WAIT_SOF0 = 0,  /* 等待 0xAA */
    COMM_ST_WAIT_SOF1,      /* 等待 0x55 */

    COMM_ST_VER,            /* 读取 VER */
    COMM_ST_MSG,            /* 读取 MSG_ID */
    COMM_ST_FLAGS,          /* 读取 FLAGS */
    COMM_ST_SEQ,            /* 读取 SEQ */
    COMM_ST_LEN_L,          /* 读取 LEN 低字节 */
    COMM_ST_LEN_H,          /* 读取 LEN 高字节 */

    COMM_ST_PAYLOAD,        /* 读取 PAYLOAD */

    COMM_ST_CRC_L,          /* 读取 CRC 低字节 */
    COMM_ST_CRC_H,          /* 读取 CRC 高字节 */

    COMM_ST_EOF0,           /* 读取 EOF0=0x66 */
    COMM_ST_EOF1            /* 读取 EOF1=0xBB */
} comm_parser_state_t;

/* ============================================================================
 * 解析器实例（每个串口/通道建议一个独立实例）
 * ========================================================================== */
typedef struct
{
    comm_parser_state_t state;     /* 当前状态 */

    proto_frame_t frame;           /* 当前正在组装的帧 */

    uint16_t payload_offset;       /* payload 已接收字节数（0..len） */

    /* 增量 CRC：从 VER 开始，对 VER..PAYLOAD 每字节 UpdateByte */
    uint16_t crc_calc;

    /* 接收的 CRC（帧内是小端：先 low 后 high） */
    uint8_t  crc_bytes[2];
    uint16_t crc_recv;

    /* 上层回调 */
    comm_on_frame_fn on_frame;
    void *user_ctx;

} comm_parser_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief  初始化解析器
 * @param  parser    解析器实例
 * @param  on_frame  解析到完整有效帧后的回调
 * @param  user_ctx  回调透传参数
 */
void CommParser_Init(comm_parser_t *parser, comm_on_frame_fn on_frame, void *user_ctx);

/**
 * @brief  复位解析器（回到等待帧头）
 * @param  parser 解析器实例
 */
void CommParser_Reset(comm_parser_t *parser);

/**
 * @brief  喂入 1 个字节推进状态机
 * @param  parser 解析器实例
 * @param  byte   新输入的字节
 * @return 解析状态码（OK/NEED_MORE/ERR_*）
 */
comm_parse_status_t CommParser_FeedByte(comm_parser_t *parser, uint8_t byte);

/**
 * @brief  喂入一段 buffer（常用于 DMA/环形缓冲取出的连续数据）
 * @param  parser 解析器实例
 * @param  data   连续数据指针
 * @param  len    数据长度（字节）
 * @return 成功解析出的帧数量（COMM_PARSE_OK 次数）
 */
size_t CommParser_Feed(comm_parser_t *parser, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PARSER_H */
