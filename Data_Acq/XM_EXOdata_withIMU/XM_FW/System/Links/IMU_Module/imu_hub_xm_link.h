/**
 ******************************************************************************
 * @file    imu_hub_xm_link.h
 * @author  HyundoKim
 * @brief   [System/Links] XM10 ↔ IMU Hub 링크 관리 (PnP State Machine)
 * @version 1.0
 * @date    Dec 4, 2025
 *
 * @details
 * IMU Hub Module과 XM10 간의 연결 상태를 관리하는 PnP State Machine입니다.
 * AGR_PnPMgr(Services Layer)를 활용하여 자동 감지 및 타임아웃 처리를 수행합니다.
 * 
 * [PnP 시퀀스]
 * 1. INITIALISING: XM10 Bootup 전송 (1s 주기)
 * 2. PRE_OPERATIONAL: PDO Mapping 설정 → NMT OPERATIONAL 전환 → Sync States
 * 3. OPERATIONAL: Heartbeat 교환 (1s 주기), 데이터 수신
 * 
 * [LED 제어]
 * - 현재는 IMU Hub Module의 Status LED만 사용
 * - 향후 XM10 LED 추가 예정 (TODO)
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_LINKS_IMU_MODULE_IMU_HUB_XM_LINK_H_
#define SYSTEM_LINKS_IMU_MODULE_IMU_HUB_XM_LINK_H_

#include <stdint.h>
#include <stdbool.h>
#include "link_interface.h"  /* LinkModule_t */

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
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief IMU Hub Link 초기화
 * @details 
 * - AGR_PnPMgr에 Device 등록
 * - ImuHub_Drv 초기화 (콜백 등록)
 */
void ImuHub_XM_Link_Init(void);

/**
 * @brief 주기적 실행 (PnP State Machine)
 * @details PnP Manager Task에서 10ms 주기로 호출됩니다.
 */
void ImuHub_XM_Link_RunPeriodic(void);

/**
 * @brief CAN 메시지 처리 (SDO)
 * @details PnP Manager에서 SDO 메시지를 라우팅할 때 호출됩니다.
 * @param can_id CAN ID
 * @param data 수신 데이터
 * @param len 데이터 길이
 */
void ImuHub_XM_Link_ProcessMessage(uint16_t can_id, uint8_t* data, uint8_t len);

/**
 * @brief IMU Hub 연결 상태 확인
 * @return true: OPERATIONAL + Sync 완료, false: 끊김
 */
bool ImuHub_XM_Link_IsConnected(void);

/**
 * @brief PnP Manager 연동용 모듈 구조체 반환
 * @return LinkModule_t 포인터
 */
LinkModule_t* ImuHub_XM_Link_GetModule(void);

#endif /* SYSTEM_LINKS_IMU_MODULE_IMU_HUB_XM_LINK_H_ */

