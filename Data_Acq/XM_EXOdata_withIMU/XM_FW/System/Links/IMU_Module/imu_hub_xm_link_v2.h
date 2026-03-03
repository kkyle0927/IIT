/**
 ******************************************************************************
 * @file    imu_hub_xm_link_v2.h
 * @author  HyundoKim
 * @brief   [System/Link] XM10 ↔ IMU Hub Link Layer (AGR_PnP V2 기반, Master)
 * @version 2.0
 * @date    Dec 8, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IMU_HUB_XM_LINK_V2_H
#define IMU_HUB_XM_LINK_V2_H

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 * @brief IMU Hub Object Dictionary (OD) Index
 * @details XM10 (Master)이 IMU Hub (Slave)와 통신 시 사용하는 OD Index
 * 
 * [CANopen 표준 범위]
 * - 0x1600~0x17FF: TPDO Mapping Parameters
 * - 0x2000~0x5FFF: Manufacturer-specific
 */
#define IMUHUB_OD_IDX_PDO_MAPPING_A  0x1600  /**< TPDO1 Mapping (Group A: Metadata + IMU 0,1,2) */
#define IMUHUB_OD_IDX_PDO_MAPPING_B  0x1601  /**< TPDO2 Mapping (Group B: Metadata + IMU 3,4,5) */
#define IMUHUB_OD_IDX_IMU_CONN_MASK  0x2000  /**< IMU Connected Mask (RO, 1B) - bit0~5: IMU0~5 */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief XM10 Master Link 초기화 (V2)
 * @details IMU Hub Device를 AGR_PnP에 등록하고 Pre-Op 로직 설정
 */
void ImuHub_XM_Link_Init_V2(void);

/**
 * @brief XM10 Master Link 주기 실행 (V2)
 * @details AGR_PnP_RunPeriodic() + Pre-Op Timeout 체크
 */
void ImuHub_XM_Link_RunPeriodic_V2(void);

/**
 * @brief XM10 Master Link CAN 메시지 처리 (V2)
 * @param can_id CAN ID
 * @param data 데이터 포인터
 * @param len 데이터 길이
 */
void ImuHub_XM_Link_ProcessMessage_V2(uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 * @brief IMU Hub 연결 상태 확인 (V2)
 * @return true: 연결됨 (OPERATIONAL + Heartbeat OK), false: 끊김
 */
bool ImuHub_XM_Link_IsConnected_V2(void);

/**
 * @brief IMU Connected Mask 조회 (V2)
 * @return IMU Connected Mask (bit0: IMU0, ..., bit5: IMU5)
 * @details Pre-Op 단계에서 IMU Hub로부터 조회한 값
 */
uint8_t ImuHub_XM_Link_GetImuConnectedMask_V2(void);

#endif /* IMU_HUB_XM_LINK_V2_H */
