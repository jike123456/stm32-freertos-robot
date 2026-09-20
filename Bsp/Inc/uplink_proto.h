#ifndef __UPLINK_PROTO_H
#define __UPLINK_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "protocol.h"

#define UP_MSG_STATUS   (0x10u)
#define UP_FLAGS_NONE   (0x00u)

/**
 * @brief 初始化 uplink 发送封装（USART1 DMA）
 */
void UplinkProto_Init(void);

/**
 * @brief 在 HAL_UART_TxCpltCallback 中转调，释放 busy
 */
void UplinkProto_OnUartTxCplt(UART_HandleTypeDef *huart);

/**
 * @brief 在 HAL_UART_ErrorCallback 中转调，释放 busy
 */
void UplinkProto_OnUartError(UART_HandleTypeDef *huart);


/**
 * @brief 发送状态帧（Status）
 */
bool UplinkProto_SendStatus(proto_uplink_t *tx, uint8_t seq,
						uint16_t battery_mv, int16_t temp,
                        int16_t vx, int16_t vy, int16_t vz,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
												uint16_t status_flags);


#endif
