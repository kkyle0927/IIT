/**
 ******************************************************************************
 * @file    xm_api_rtc.c
 * @author  HyundoKim
 * @brief   XM10 RTC API 구현부 — MCP79510 Facade
 * @version 1.0.0
 * @date    2026-03-02
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api_rtc.h"
#include "mcp79510.h"

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

bool XM_RTC_SetDateTime(const XmDateTime_t* dt)
{
    if (dt == NULL) return false;
    if (dt->year < 2000 || dt->year > 2099) return false;

    MCP79510_DateTime_t mcp_dt = {
        .year    = (uint8_t)(dt->year - 2000),
        .month   = dt->month,
        .day     = dt->day,
        .weekday = dt->weekday,
        .hour    = dt->hour,
        .minute  = dt->minute,
        .second  = dt->second,
    };

    return MCP79510_SetDateTime(&mcp_dt) == AGRBStatus_OK;
}

bool XM_RTC_GetDateTime(XmDateTime_t* dt)
{
    if (dt == NULL) return false;

    MCP79510_DateTime_t mcp_dt;
    if (MCP79510_GetDateTime(&mcp_dt) != AGRBStatus_OK) {
        return false;
    }

    dt->year    = 2000 + mcp_dt.year;
    dt->month   = mcp_dt.month;
    dt->day     = mcp_dt.day;
    dt->weekday = mcp_dt.weekday;
    dt->hour    = mcp_dt.hour;
    dt->minute  = mcp_dt.minute;
    dt->second  = mcp_dt.second;

    return true;
}

bool XM_RTC_IsRunning(void)
{
    return MCP79510_IsRunning();
}
