/**
 ******************************************************************************
 * @file    xm_api_user_custom.c
 * @author  HyundoKim
 * @brief   In-packet User Custom data slot API — implementation
 * @details
 * User_Custom 28B 블록의 setter 들이 쓰는 백킹 스토어. UserTask 단일 스레드
 * 가정으로 lock-free 설계(쓰기/스냅샷 모두 동일 컨텍스트). 다른 컨텍스트에서
 * 호출할 일이 생기면 atomic 또는 mutex 보호를 추가해야 한다.
 *
 * @version 1.0
 * @date    May 12, 2026
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api_user_custom.h"
#include <string.h>

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static XM_UserCustomBlock_t s_block = {0};

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void XM_UserCustom_SetFloat(uint8_t idx, float value)
{
    if (idx >= XM_USER_CUSTOM_FLOAT_COUNT) {
        return;
    }
    s_block.f[idx] = value;
}

void XM_UserCustom_SetI16(uint8_t idx, int16_t value)
{
    if (idx >= XM_USER_CUSTOM_I16_COUNT) {
        return;
    }
    s_block.i16[idx] = value;
}

void XM_UserCustom_SetU8(uint8_t idx, uint8_t value)
{
    if (idx >= XM_USER_CUSTOM_U8_COUNT) {
        return;
    }
    s_block.u8[idx] = value;
}

void XM_UserCustom_SetFlags(uint16_t flags)
{
    s_block.flags = flags;
}

void XM_UserCustom_SetFlag(uint8_t bit, bool value)
{
    if (bit >= XM_USER_CUSTOM_FLAG_COUNT) {
        return;
    }
    const uint16_t mask = (uint16_t)(1U << bit);
    if (value) {
        s_block.flags |= mask;
    } else {
        s_block.flags &= (uint16_t)~mask;
    }
}

void XM_UserCustom_Reset(void)
{
    memset(&s_block, 0, sizeof(s_block));
}

void XM_UserCustom_GetBlock(XM_UserCustomBlock_t* out)
{
    if (out == NULL) {
        return;
    }
    *out = s_block;
}
