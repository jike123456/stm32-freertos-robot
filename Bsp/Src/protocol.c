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
 * @board       Kaoya Robot 下位机
 * @店铺链接： https://mall.bilibili.com/neul-next/detailshop/index.html?channel=WEIXIN&curTab=LIVE_REPLAY&loadingShow=1&msource=cps_showcase_384121683&noTitleBar=1&outsideMall=yes&page=detailshop_detail&share_mid=384121683&smallShopMid=384121683#themeType=1 
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


