/**
 ******************************************************************************
 * @file    imu_hub_drv.h
 * @author  HyundoKim
 * @brief   [Device Layer] XM10 ↔ IMU Hub 통신 드라이버 (Master, CANopen 표준)
 * @version 3.0 (CANopen 표준 준수)
 * @date    Dec 8, 2025
 *
 * @details
 * [V3.0 변경사항 - CANopen 표준 기반]
 * - AGR_NMT 통합: NMT, Boot-up, Heartbeat 처리
 * - CAN ID 기반 메시지 분기 (효율적)
 * - OD Index → 제조사 영역 (0x2000~) 이동
 * - Pre-Op 상태 머신 유지 (검증된 로직)
 * - Public API: IsConnected(), GetNmtState(), RunPeriodic()
 * 
 * [메시지 처리 흐름]
 * ISR → ImuHub_Drv_ProcessCANMessage()
 *     ├─ CAN ID 0x700: Boot-up, Heartbeat → AGR_NMT
 *     ├─ CAN ID 0x580: SDO Response → Pre-Op State Machine
 *     └─ CAN ID 0x180/0x280: PDO → AGR_DOP
 * 
 * [통신 구조]
 * - IMU Hub Node ID: 0x08
 * - TPDO1 (0x188): Group A (Metadata + IMU 0,1,2)
 * - TPDO2 (0x288): Group B (Metadata + IMU 3,4,5)
 * - SDO Request (0x608): PDO Mapping (0x2010/0x2011)
 * - SDO Response (0x588): IMU Connected Mask (0x2000)
 * - Heartbeat (0x708): Heartbeat
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef DEVICES_AGR_IMU_MODULE_IMU_HUB_DRV_H_
#define DEVICES_AGR_IMU_MODULE_IMU_HUB_DRV_H_

#include <stdint.h>
#include <stdbool.h>
#include "agr_dop.h"
#include "agr_nmt.h"
#include "agr_pnp.h"  /* ✅ AGR_PnP_Inst_t forward declaration */

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** @brief IMU 센서 개수 */
#define IMUHUB_IMU_COUNT            6

/** @brief Group A IMU 개수 (IMU 0, 1, 2) */
#define IMUHUB_GROUP_A_COUNT        3

/** @brief Group B IMU 개수 (IMU 3, 4, 5) */
#define IMUHUB_GROUP_B_COUNT        3

/**
 *-----------------------------------------------------------
 * SCALING FACTORS (int16 ↔ float 변환, XM10 → User)
 *-----------------------------------------------------------
 * @details IMU Hub에서 받은 int16 데이터를 float으로 복원
 * 
 * [복원 공식]
 * - Quaternion: int16 / 10000.0f → float (-1.0 ~ 1.0)
 * - Euler: int16 / 100.0f → float (-180° ~ 180°)
 * - Accel: int16 / 100.0f → float (±16g)
 * - Gyro: int16 / 10.0f → float (±2000 deg/s)
 * - Mag: int16 / 1.0f → float (±4800 uT)
 */

#define IMUHUB_SCALE_INT16_TO_QUAT      10000.0f
#define IMUHUB_SCALE_INT16_TO_EULER     100.0f
#define IMUHUB_SCALE_INT16_TO_ACC       100.0f
#define IMUHUB_SCALE_INT16_TO_GYRO      10.0f
#define IMUHUB_SCALE_INT16_TO_MAG       1.0f

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief IMU 센서 데이터 구조체 (int16 기반, IMU Hub → XM10)
 * @details 32 bytes per IMU
 */
/**
 * @brief IMU Hub 센서 데이터 (XM10용, PDO 최적화)
 * @details SubIndex 0x60 ({q,a,g} 20B)에 맞춰진 구조체
 */
typedef struct __attribute__((packed)) {
    int16_t  q[4];       /**< 쿼터니언 (w, x, y, z) - 8 bytes */
    int16_t  a[3];       /**< 가속도 (x, y, z) - 6 bytes */
    int16_t  g[3];       /**< 자이로 (x, y, z) - 6 bytes */
} ImuHub_ImuData_t;      /* Total: 20 bytes */

/**
 * @brief IMU Hub 수신 데이터 (최종 처리 완료)
 * @note Timestamp는 24-bit 값이지만 uint32_t에 저장 (상위 8bit = 0)
 */
typedef struct {
	uint32_t         timestamp;               /**< Frame Timestamp (24-bit, 0~16777215 ms, 약 4.6시간 순환) */
	uint8_t          connected_mask;          /**< bit0~5: IMU 0~5 연결 상태 (1=connected, 0=disconnected) */
    ImuHub_ImuData_t imu[IMUHUB_IMU_COUNT];  /**< 6개 IMU 데이터 {q[4], a[3], g[3]} */
} ImuHub_RxData_t;

/* Legacy 콜백 구조체 제거 (V3.0에서 사용 안 함, AGR_PnP가 처리) */

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/**
 * @brief IMU Hub 드라이버 초기화 (XM Master)
 * @param tx_func CAN 전송 함수 (IOIF Layer에서 제공)
 * @param master_pnp Master PnP 인스턴스 (System Layer PnPManager에서 제공)
 * @return 0=성공, <0=에러
 * @note Master PnP에 IMU Hub Device 등록
 */
int ImuHub_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Inst_t* master_pnp);

/**
 * @brief CAN 메시지 처리 (TPDO/SDO)
 * @details _CanFdRxCallback에서 호출됩니다 (Task Context).
 * @param can_id CAN ID
 * @param data 수신 데이터
 * @param len 데이터 길이
 * @note 내부적으로 Mutex 보호됩니다.
 */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len);

/**
 * @brief 최신 IMU 데이터 읽기
 * @details Main Control Loop에서 호출합니다 (500Hz).
 * @param rx_data 수신 데이터를 저장할 포인터
 * @return true: 유효한 데이터 있음, false: 데이터 없음
 * @note 내부적으로 Mutex 보호됩니다.
 */
bool ImuHub_Drv_GetRxData(ImuHub_RxData_t* rx_data);

/**
 * @brief 데이터 준비 여부 확인
 * @return true: 최소 1회 이상 TPDO 수신 완료
 */
bool ImuHub_Drv_IsDataReady(void);

/**
 * @brief IMU Hub 연결 상태 확인 (V3.0: Public API)
 * @return true: 연결됨 (OPERATIONAL + Heartbeat OK)
 */
bool ImuHub_Drv_IsConnected(void);

/**
 * @brief IMU Hub NMT 상태 조회 (V3.0: Public API)
 * @return NMT 상태
 */
AGR_NMT_State_t ImuHub_Drv_GetNmtState(void);

/**
 * @brief 주기 실행 (V3.0: Public API)
 * @details 
 * Main Loop 또는 Low-Priority Task에서 호출합니다.
 * 
 * [처리 항목]
 * 1. AGR_NMT Heartbeat Timeout 체크
 * 2. AGR_PnP 주기 처리 (Pre-Op 시퀀스)
 */
void ImuHub_Drv_RunPeriodic(void);

/**
 * @brief XM10 Boot-up 전송 (PnP Step 1)
 */
void ImuHub_Drv_SendBootup(void);

/**
 * @brief IMU Hub NMT 상태 변경 명령 전송
 * @param target_state 목표 NMT 상태 (AGR_NMT_STATE_OPERATIONAL 등)
 */
void ImuHub_Drv_SendSetNMTState(uint8_t target_state);

/**
 * @brief IMU Hub PDO Mapping 설정 전송 (PnP Step 2)
 * @param group PDO 그룹 (0=Group A/TPDO1, 1=Group B/TPDO2)
 * @param pdo_map PDO Mapping 데이터 (CANopen 표준 형식)
 * @param len 데이터 길이
 * 
 * @note
 * - Group A (0x1600): TPDO1 Mapping (Metadata + IMU 0,1,2)
 * - Group B (0x1601): TPDO2 Mapping (Metadata + IMU 3,4,5)
 */
void ImuHub_Drv_SendSetPDOMapping(uint8_t group, const uint8_t* pdo_map, uint8_t len);

/**
 * @brief XM10 Heartbeat 전송 (주기적, OPERATIONAL 상태)
 * @param xm_nmt_state XM10의 현재 NMT 상태
 */
void ImuHub_Drv_SendHeartbeat(uint8_t xm_nmt_state);

/**
 * @brief IMU Connected Mask 읽기 요청 (SDO Upload)
 * @details 0x2000을 읽어서 어떤 IMU가 연결되었는지 확인
 * @note bit0~5: IMU 0~5 연결 상태 (1=connected, 0=disconnected)
 */
void ImuHub_Drv_SendReadImuConnMask(void);

/**
 * @brief Sync States 요청 전송 (PnP Step 3 - Legacy)
 * @deprecated Use ImuHub_Drv_SendReadImuConnMask() instead
 */
void ImuHub_Drv_SendSyncStates(void);

/**
 * @brief IMU Hub 연결 상태 확인 (AGR_PnP 기반)
 * @return true: 연결됨 (OPERATIONAL + Heartbeat OK), false: 끊김
 */
bool ImuHub_Drv_IsConnected(void);

/**
 * @brief IMU Hub NMT 상태 조회 (AGR_PnP 기반)
 * @return NMT 상태 (AGR_NMT_STATE_*)
 */
AGR_NMT_State_t ImuHub_Drv_GetNmtState(void);

/**
 * @brief 주기 실행 (AGR_PnP + Timeout 체크)
 * @details Main Loop에서 주기적으로 호출 (예: 10ms)
 */
void ImuHub_Drv_RunPeriodic(void);

/**
 * @brief IMU Connected Mask 조회
 * @return IMU Connected Mask (bit0: IMU0, ..., bit5: IMU5)
 * @details Pre-Op 단계에서 IMU Hub로부터 SDO로 조회한 값
 */
uint8_t ImuHub_Drv_GetImuConnectedMask(void);

#endif /* DEVICES_AGR_IMU_MODULE_IMU_HUB_DRV_H_ */

