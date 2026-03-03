/**
 ******************************************************************************
 * @file    imu_hub_debug.h
 * @author  HyundoKim
 * @brief   [System/Links] IMU Hub PnP 디버깅용 Live Expression 변수
 * @version 1.0
 * @date    Dec 5, 2025
 *
 * @details
 * Live Expression에 추가하여 PnP 연결 과정을 실시간 모니터링할 수 있습니다.
 * 
 * [Live Expression 사용법]
 * STM32CubeIDE에서:
 * 1. Window → Show View → Live Expressions
 * 2. Add Expression:
 *    - g_imu_hub_debug
 *    - g_imu_hub_debug.pnp_state_str
 *    - g_imu_hub_debug.pre_op_state_str
 * 
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IMU_HUB_DEBUG_H
#define IMU_HUB_DEBUG_H

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief IMU Hub PnP 디버깅 정보 (Live Expression용)
 */
typedef struct {
    /* ===== XM10 측 상태 ===== */
    struct {
        uint8_t     pnp_state;          /**< AGR_PnP_State_t (0=BOOT_UP, 1=STOPPED, 4=PRE_OP, 5=OPERATIONAL) */
        const char* pnp_state_str;      /**< 문자열 (Live Expression에 표시) */
        uint8_t     pre_op_state;       /**< PreOpState_t (세부 단계) */
        const char* pre_op_state_str;   /**< 문자열 */
        bool        is_synced;          /**< Sync States 완료 여부 */
        bool        operational_hb_received;  /**< OPERATIONAL Heartbeat 수신 플래그 */
        uint8_t     sdo_retry_count;    /**< SDO 재시도 횟수 */
        uint32_t    last_sdo_sent_ms;   /**< 마지막 SDO 전송 시각 */
        uint8_t     last_imu_hub_nmt_state;  /**< IMU Hub의 마지막 Heartbeat NMT 상태 */
    } xm;
    
    /* ===== IMU Hub 측 상태 ===== */
    struct {
        uint8_t     nmt_state;          /**< NMT 상태 (0=BOOT_UP, 1=STOPPED, 4=PRE_OP, 5=OPERATIONAL) */
        const char* nmt_state_str;      /**< 문자열 */
        bool        is_xm_booted;       /**< XM10 Boot-up 수신 여부 */
        bool        is_xm_connected;    /**< XM10 연결 완료 여부 */
        bool        is_imu_hub_bootup_sent;  /**< IMU Hub Boot-up 전송 완료 여부 */
        bool        is_pdo_mapping_set; /**< PDO Mapping 설정 완료 여부 */
        uint32_t    last_xm_heartbeat_ms;  /**< XM10 마지막 Heartbeat 수신 시각 */
    } imu_hub;
    
    /* ===== 전송 통계 ===== */
    struct {
        uint32_t    xm_bootup_count;    /**< XM10 Bootup 전송 횟수 */
        uint32_t    imu_bootup_count;   /**< IMU Hub Bootup 전송 횟수 */
        uint32_t    pdo_map_a_count;    /**< PDO Map A SDO 전송 횟수 */
        uint32_t    pdo_map_b_count;    /**< PDO Map B SDO 전송 횟수 */
        uint32_t    nmt_operational_count;  /**< NMT OPERATIONAL 전송 횟수 */
        uint32_t    sync_states_count;  /**< Sync States 전송 횟수 */
    } tx_stats;
    
    /* ===== 수신 통계 ===== */
    struct {
        uint32_t    imu_bootup_count;   /**< IMU Hub Bootup 수신 횟수 */
        uint32_t    imu_heartbeat_count;/**< IMU Hub Heartbeat 수신 횟수 */
        uint32_t    xm_bootup_count;    /**< XM10 Bootup 수신 횟수 */
        uint32_t    xm_heartbeat_count; /**< XM10 Heartbeat 수신 횟수 */
        uint32_t    pdo_map_ack_count;  /**< PDO Map ACK 수신 횟수 */
        uint32_t    sync_ack_count;     /**< Sync States ACK 수신 횟수 */
    } rx_stats;
    
    /* ===== 에러 통계 ===== */
    struct {
        uint32_t    timeout_count;      /**< 타임아웃 발생 횟수 */
        uint32_t    sdo_retry_total;    /**< SDO 재시도 총 횟수 */
        uint8_t     last_error_code;    /**< 마지막 에러 코드 */
    } errors;
    
} ImuHub_DebugInfo_t;

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES(extern)
 *-----------------------------------------------------------
 */

/**
 * @brief Live Expression용 전역 디버깅 변수
 * @note volatile로 선언하여 컴파일러 최적화 방지
 */
extern volatile ImuHub_DebugInfo_t g_imu_hub_debug;

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief 디버깅 정보 초기화
 */
void ImuHub_Debug_Init(void);

/**
 * @brief XM10 측 디버깅 정보 업데이트
 */
void ImuHub_Debug_UpdateXmState(uint8_t pnp_state, uint8_t pre_op_state, bool is_synced);

/**
 * @brief IMU Hub 측 디버깅 정보 업데이트
 */
void ImuHub_Debug_UpdateImuHubState(uint8_t nmt_state, bool is_xm_booted, bool is_xm_connected);

/**
 * @brief 전송 통계 증가
 */
void ImuHub_Debug_IncrementTxStat(const char* message_type);

/**
 * @brief 수신 통계 증가
 */
void ImuHub_Debug_IncrementRxStat(const char* message_type);

/**
 * @brief 에러 기록
 */
void ImuHub_Debug_RecordError(uint8_t error_code);

#endif /* IMU_HUB_DEBUG_H */



