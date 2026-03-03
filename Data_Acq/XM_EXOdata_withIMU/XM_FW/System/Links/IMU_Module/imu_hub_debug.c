/**
 ******************************************************************************
 * @file    imu_hub_debug.c
 * @author  HyundoKim
 * @brief   [System/Links] IMU Hub PnP 디버깅 유틸리티 구현
 * @version 1.0
 * @date    Dec 5, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_hub_debug.h"
#include "agr_pnpmgr.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/**
 * @brief Live Expression용 전역 디버깅 변수
 */
volatile ImuHub_DebugInfo_t g_imu_hub_debug;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* PnP State 문자열 매핑 */
static const char* s_pnp_state_strings[] = {
    "BOOT_UP",        // 0
    "STOPPED",        // 1
    "INVALID_2",      // 2 (사용 안 함)
    "INVALID_3",      // 3 (사용 안 함)
    "PRE_OPERATIONAL",// 4
    "OPERATIONAL"     // 5
};

/* PreOp State 문자열 매핑 */
static const char* s_pre_op_state_strings[] = {
    "PRE_OP_IDLE",
    "PRE_OP_SEND_PDO_MAP_A",
    "PRE_OP_WAIT_PDO_MAP_A_ACK",
    "PRE_OP_SEND_PDO_MAP_B",
    "PRE_OP_WAIT_PDO_MAP_B_ACK",
    "PRE_OP_SEND_NMT_OPERATIONAL",
    "PRE_OP_SEND_SYNC_STATES",
    "PRE_OP_WAIT_SYNC_ACK",
    "PRE_OP_WAIT_OPERATIONAL_HB"
};

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static const char* _GetPnPStateString(uint8_t state);
static const char* _GetPreOpStateString(uint8_t state);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void ImuHub_Debug_Init(void)
{
    memset((void*)&g_imu_hub_debug, 0, sizeof(ImuHub_DebugInfo_t));
    
    /* 초기 문자열 설정 */
    g_imu_hub_debug.xm.pnp_state_str = "BOOT_UP";
    g_imu_hub_debug.xm.pre_op_state_str = "PRE_OP_IDLE";
    g_imu_hub_debug.imu_hub.nmt_state_str = "BOOT_UP";
}

void ImuHub_Debug_UpdateXmState(uint8_t pnp_state, uint8_t pre_op_state, bool is_synced)
{
    g_imu_hub_debug.xm.pnp_state = pnp_state;
    g_imu_hub_debug.xm.pnp_state_str = _GetPnPStateString(pnp_state);
    g_imu_hub_debug.xm.pre_op_state = pre_op_state;
    g_imu_hub_debug.xm.pre_op_state_str = _GetPreOpStateString(pre_op_state);
    g_imu_hub_debug.xm.is_synced = is_synced;
}

void ImuHub_Debug_UpdateImuHubState(uint8_t nmt_state, bool is_xm_booted, bool is_xm_connected)
{
    g_imu_hub_debug.imu_hub.nmt_state = nmt_state;
    g_imu_hub_debug.imu_hub.nmt_state_str = _GetPnPStateString(nmt_state);
    g_imu_hub_debug.imu_hub.is_xm_booted = is_xm_booted;
    g_imu_hub_debug.imu_hub.is_xm_connected = is_xm_connected;
}

void ImuHub_Debug_IncrementTxStat(const char* message_type)
{
    if (message_type == NULL) {
        return;
    }
    
    if (strcmp(message_type, "XM_BOOTUP") == 0) {
        g_imu_hub_debug.tx_stats.xm_bootup_count++;
    } else if (strcmp(message_type, "IMU_BOOTUP") == 0) {
        g_imu_hub_debug.tx_stats.imu_bootup_count++;
    } else if (strcmp(message_type, "PDO_MAP_A") == 0) {
        g_imu_hub_debug.tx_stats.pdo_map_a_count++;
    } else if (strcmp(message_type, "PDO_MAP_B") == 0) {
        g_imu_hub_debug.tx_stats.pdo_map_b_count++;
    } else if (strcmp(message_type, "NMT_OPERATIONAL") == 0) {
        g_imu_hub_debug.tx_stats.nmt_operational_count++;
    } else if (strcmp(message_type, "SYNC_STATES") == 0) {
        g_imu_hub_debug.tx_stats.sync_states_count++;
    }
}

void ImuHub_Debug_IncrementRxStat(const char* message_type)
{
    if (message_type == NULL) {
        return;
    }
    
    if (strcmp(message_type, "IMU_BOOTUP") == 0) {
        g_imu_hub_debug.rx_stats.imu_bootup_count++;
    } else if (strcmp(message_type, "IMU_HEARTBEAT") == 0) {
        g_imu_hub_debug.rx_stats.imu_heartbeat_count++;
    } else if (strcmp(message_type, "XM_BOOTUP") == 0) {
        g_imu_hub_debug.rx_stats.xm_bootup_count++;
    } else if (strcmp(message_type, "XM_HEARTBEAT") == 0) {
        g_imu_hub_debug.rx_stats.xm_heartbeat_count++;
    } else if (strcmp(message_type, "PDO_MAP_ACK") == 0) {
        g_imu_hub_debug.rx_stats.pdo_map_ack_count++;
    } else if (strcmp(message_type, "SYNC_ACK") == 0) {
        g_imu_hub_debug.rx_stats.sync_ack_count++;
    }
}

void ImuHub_Debug_RecordError(uint8_t error_code)
{
    g_imu_hub_debug.errors.timeout_count++;
    g_imu_hub_debug.errors.last_error_code = error_code;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static const char* _GetPnPStateString(uint8_t state)
{
    if (state > 5) {
        return "INVALID";
    }
    return s_pnp_state_strings[state];
}

static const char* _GetPreOpStateString(uint8_t state)
{
    if (state > 8) {
        return "INVALID";
    }
    return s_pre_op_state_strings[state];
}



