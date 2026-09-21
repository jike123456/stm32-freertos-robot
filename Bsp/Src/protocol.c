/**
 ****************************************************************************************************
 * @file        protocol.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       通信协议
 * @details
 *  CRC
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       STM32F407 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 ****************************************************************************************************
 */
#include "protocol.h"

uint16_t Proto_Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = Proto_Crc16CcittFalseInit();
    if (data == NULL || len == 0u)
    {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        crc = Proto_Crc16CcittFalseUpdateByte(crc, data[i]);
    }

    return crc;
}


