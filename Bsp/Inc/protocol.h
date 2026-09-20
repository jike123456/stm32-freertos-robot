/**
 ****************************************************************************************************
 * @file        protocol.h
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       统一的串口通信协议定义（收发共用）
 * @license     Copyright (c) 2025-2035, Kaoya Project
 * @details
 * 本文件只定义“协议是什么”：
 *  - 帧格式/字段含义/常量/消息ID/payload结构体/长度宏
 *  - 约定 CRC 计算范围与字节序（固定小端）
 *
 * 不做的事情：
 *  - 不依赖 HAL / FreeRTOS
 *  - 不实现具体解析器/打包器（这些放到 protocol_codec.c / comm_parser.c）
 *
 * =========================
 * 帧格式（带帧尾）
 * =========================
 *
 *  Byte Offs | Name        | Size | Desc
 * -----------+-------------+------+------------------------------------------
 *  0         | SOF0        | 1    | 帧头0：0xAA
 *  1         | SOF1        | 1    | 帧头1：0x55
 *  2         | VER         | 1    | 协议版本
 *  3         | MSG_ID      | 1    | 消息类型
 *  4         | FLAGS       | 1    | 标志位（预留）
 *  5         | SEQ         | 1    | 序号（可用于丢包检测/应答匹配）
 *  6         | LEN_L       | 1    | PAYLOAD 长度（低字节，小端）
 *  7         | LEN_H       | 1    | PAYLOAD 长度（高字节，小端）
 *  8..       | PAYLOAD     | N    | 负载（长度 = LEN）
 *  8+N       | CRC_L       | 1    | CRC16 低字节（小端）
 *  9+N       | CRC_H       | 1    | CRC16 高字节（小端）
 *  10+N      | EOF0        | 1    | 帧尾0：0x66
 *  11+N      | EOF1        | 1    | 帧尾1：0xBB
 *
 * =========================
 * 字节序约定（固定小端）
 * =========================
 * 所有多字节字段（LEN/CRC/payload 内的 int16/int32 等）均使用 little-endian（小端）编码。
 *
 * =========================
 * CRC 约定
 * =========================
 * CRC16/CCITT-FALSE:
 *  - poly: 0x1021
 *  - init: 0xFFFF
 *  - refin/refout: false/false
 *  - xorout: 0x0000
 *
 * CRC 计算范围：
 *  - 从 VER 开始，到 PAYLOAD 结束（包含 VER/MSG/FLAGS/SEQ/LEN/PAYLOAD）
 *  - 不包含 SOF，不包含 CRC 字段本身，不包含 EOF
 ****************************************************************************************************
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 固定字节常量                                                                */
/* ========================================================================== */

/* 帧头 */
#define PROTO_SOF0              (0xAAu)
#define PROTO_SOF1              (0x55u)

/* 帧尾 */
#define PROTO_EOF0              (0x66u)
#define PROTO_EOF1              (0xBBu)

/* 协议版本 */
#define PROTO_VER               (0x01u)

/* ========================================================================== */
/* 帧字段长度/偏移                                                             */
/* ========================================================================== */

#define PROTO_LEN_SOF           (2u)      /* SOF0 + SOF1 */
#define PROTO_LEN_HDR_FIXED     (6u)      /* VER(1)+MSG(1)+FLAGS(1)+SEQ(1)+LEN(2) */
#define PROTO_LEN_CRC           (2u)
#define PROTO_LEN_EOF           (2u)

/* 字段偏移（从帧起始算起） */
#define PROTO_OFF_SOF0          (0u)
#define PROTO_OFF_SOF1          (1u)
#define PROTO_OFF_VER           (2u)
#define PROTO_OFF_MSG_ID        (3u)
#define PROTO_OFF_FLAGS         (4u)
#define PROTO_OFF_SEQ           (5u)
#define PROTO_OFF_LEN_L         (6u)
#define PROTO_OFF_LEN_H         (7u)
#define PROTO_OFF_PAYLOAD       (8u)

/* ========================================================================== */
/* 载荷限制                                                                    */
/* ========================================================================== */

#ifndef PROTO_MAX_PAYLOAD
#define PROTO_MAX_PAYLOAD       (64u)
#endif

#define PROTO_FRAME_MIN_LEN     (PROTO_LEN_SOF + PROTO_LEN_HDR_FIXED + 0u + PROTO_LEN_CRC + PROTO_LEN_EOF)
#define PROTO_FRAME_MAX_LEN     (PROTO_LEN_SOF + PROTO_LEN_HDR_FIXED + PROTO_MAX_PAYLOAD + PROTO_LEN_CRC + PROTO_LEN_EOF)

/* ========================================================================== */
/* 消息 ID 定义（收发共用）                                                    */
/* ========================================================================== */
/*
 * 规则
 *  - 0x01..0x7F: Host -> MCU （上行命令）
 *  - 0x80..0xFF: MCU  -> Host（下行上报/应答）
 */
typedef enum
{
    /* Host -> MCU */
    PROTO_MSG_CMD_VEL      = 0x01,   /* 速度指令：vx, vy, wz */

    /* MCU -> Host */
    PROTO_MSG_STATUS       = 0x81,   /* 状态上报 */
    PROTO_MSG_ACK          = 0x80,   /* 通用 ACK（可选） */
} proto_msg_id_t;

/* ========================================================================== */
/* FLAGS 位定义（预留）                                                        */
/* ========================================================================== */
#define PROTO_FLAG_DEFAULT      (0u)
#define PROTO_FLAG_ACK_REQ      (1u << 0) /* 发送方请求应答 */
#define PROTO_FLAG_ENCRYPT      (1u << 1) /* 预留：加密 */
#define PROTO_FLAG_COMPRESS     (1u << 2) /* 预留：压缩 */

/* 系统状态上报位 */
#define PROTO_STATUS_FLAG_RUNNING       (1u << 0)
#define PROTO_STATUS_FLAG_CMD_TIMEOUT   (1u << 1)
#define PROTO_STATUS_FLAG_LOW_BATTERY   (1u << 2)
#define PROTO_STATUS_FLAG_IMU_ERROR     (1u << 3)
#define PROTO_STATUS_FLAG_TASK_ERROR    (1u << 4)

/* ========================================================================== */
/* payload 结构定义                                                            */
/* ========================================================================== */

/**
 * @brief 速度指令 payload（Host -> MCU）
 * 编码：int16 小端
 * 单位建议：
 *  - vx/vy: mm/s
 *  - wz:   mrad/s
 */
typedef struct
{
    int16_t vx;   /* x 方向速度，mm/s */
    int16_t vy;   /* y 方向速度，mm/s（差速底盘固定为 0） */
    int16_t wz;   /* 角速度，mrad/s */
} proto_cmd_vel_t;

#define PROTO_CMD_VEL_PAYLOAD_LEN   (6u)

/**
 * @brief 状态上报 payload（MCU -> Host）
 *
 * 设计目标：
 *  - 定长、可快速解析
 *  - 全部定点数，避免 float 跨平台差异
 *
 * 字段说明：
 *  - battery_mv: 电池电压（mV）
 *  - vl/vr:      左右轮线速度（mm/s）
 *  - gyro_*:     三轴角速度（0.01 deg/s，centi-deg/s）
 *
 */
typedef struct
{
    uint16_t battery_mv;   /* 电池电压，mV */
	
	int16_t temp;          /* 温度，℃ * 100 */

    int16_t  vx;           /* x 方向速度，mm/s */
    int16_t  vy;           /* y 方向速度，mm/s（差速底盘固定为 0） */
	int16_t  vz;           /* 角速度，mrad/s */

    int16_t  gx;  /* 角速度 X，0.01 deg/s */
    int16_t  gy;  /* 角速度 Y，0.01 deg/s */
    int16_t  gz;  /* 角速度 Z，0.01 deg/s */
	
	int16_t  ax;     /* 角速度 X，0.01 deg/s */
    int16_t  ay;     /* 角速度 Y，0.01 deg/s */
    int16_t  az;     /* 角速度 Z，0.01 deg/s */
	
	/* 新增：系统状态位，追加在末尾以保持原字段偏移不变 */
	uint16_t status_flags;
	
} proto_status_t;

#define PROTO_STATUS_PAYLOAD_LEN    (24u)
#define PROTO_STATUS_FRAME_LEN      (2 + 6 + PROTO_STATUS_PAYLOAD_LEN + 2 + 2)   /* 36 */
typedef struct
{
	uint8_t send_buffer[PROTO_STATUS_FRAME_LEN];
	struct{
		uint8_t sof0;
		uint8_t sof1;
		uint8_t ver;
		uint8_t msg_id;
		uint8_t flags;
		uint8_t seq;
		uint16_t len;
		
		proto_status_t payload;
		
		uint16_t crc;
		uint8_t eof0;
		uint8_t eof1;
	}send_str;
	
} proto_uplink_t;

/* ========================================================================== */
/* 小端读写 helper（最小必需，默认全协议固定小端）                              */
/* ========================================================================== */
/*
 * 说明：
 *  - 不建议 memcpy 结构体直接发送（有 padding/对齐风险）
 *  - 统一用这些 helper 写入/读取，避免到处手写位运算
 */
static inline void proto_wr_u8(uint8_t *buf, uint16_t *off, uint8_t v)
{
    buf[(*off)++] = v;
}

static inline void proto_wr_u16(uint8_t *buf, uint16_t *off, uint16_t v)
{
    buf[(*off)++] = (uint8_t)(v & 0xFFu);
    buf[(*off)++] = (uint8_t)((v >> 8) & 0xFFu);
}

static inline void proto_wr_i16(uint8_t *buf, uint16_t *off, int16_t v)
{
    proto_wr_u16(buf, off, (uint16_t)v);
}

static inline uint16_t proto_rd_u16(const uint8_t *buf, uint16_t off)
{
    return (uint16_t)buf[off] | ((uint16_t)buf[off + 1u] << 8);
}

static inline int16_t proto_rd_i16(const uint8_t *buf, uint16_t off)
{
    return (int16_t)proto_rd_u16(buf, off);
}

/* ========================================================================== */
/* CRC16/CCITT-FALSE                                                           */
/*  - poly=0x1021 init=0xFFFF refin=false refout=false xorout=0                */
/* 增量版                                                                             */
/* ========================================================================== */

#define PROTO_CRC16_INIT   (0xFFFFu)

static inline uint16_t Proto_Crc16CcittFalseInit(void)
{
    return (uint16_t)PROTO_CRC16_INIT;
}

static inline uint16_t Proto_Crc16CcittFalseUpdateByte(uint16_t crc, uint8_t data)
{
    crc ^= (uint16_t)((uint16_t)data << 8);
    for (uint8_t i = 0; i < 8u; i++)
    {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                             : (uint16_t)(crc << 1);
    }
    return crc;
}

/* 一次性计算版本（protocol.c 实现） */
uint16_t Proto_Crc16CcittFalse(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
