/**
 ******************************************************************************
 * @file    system_startup.h
 * @author  HyundoKim
 * @brief   [System/Core] IMU Hub 시스템 초기화 및 부팅 시퀀스 관리
 * @version 2.0
 * @date    Dec 2, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_CORE_SYSTEM_STARTUP_H_
#define SYSTEM_CORE_SYSTEM_STARTUP_H_

#include <stdint.h>
#include <stdbool.h>
#include "ioif_agrb_fdcan.h"

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
 * @brief IMU Hub 시스템 초기화 (타이머 시작 제외)
 * @details main() 함수에서 단 한 번만 호출되어야 합니다.
 *
 * [초기화 순서]
 * 1. IOIF 계층 초기화 (FDCAN, UART, TIM, GPIO)
 * 2. 센서 리셋 시퀀스 (Power Cycle + Pin Discharge)
 * 3. System Services 초기화 (CANFD Rx, UART Rx, PnP, CDC, DOP Serial)
 * 4. IMU Power On + 안정화 대기
 *
 * [NOTE] Application Task 초기화와 타이머 시작은 main.c에서 수행
 */
void System_Startup(void);

/**
 * @brief TIM ISR 콜백 등록 + 타이머 시작
 * @param main_loop  TIM16 (1ms, Priority 2) 콜백 — Application Control Task RunLoop
 * @param sys_loop   TIM7  (1ms, Priority 4) 콜백 — Application System Task RunLoop
 * @details main.c에서 App Init 완료 후 마지막에 호출.
 *          이 함수 호출 전까지는 ISR이 발생하지 않으므로 안전한 초기화 보장.
 */
void System_StartTimers(void);

/**
 * @brief FDCAN2 전송 래퍼 함수
 * @param can_id CAN ID (11-bit 또는 29-bit)
 * @param data  전송할 데이터의 포인터 (const)
 * @param len   전송할 데이터의 길이 (0~64 bytes)
 * @return 0=성공, <0=에러
 * @note AGR_TxFunc_t 타입과 호환됩니다.
 * @details System Startup 완료 후 외부 모듈이 FDCAN2를 사용할 수 있도록 제공합니다.
 */
int System_Fdcan2_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 * @brief FDCAN2 핸들 getter (V2 아키텍처: extern 대신 getter 사용)
 * @return FDCAN2 IOIF 핸들
 */
IOIF_FDCANx_t System_GetFdcanHandle(void);

/**
 * @brief PnP Slave path Tx 통계 조회 (HB / Bootup 계측)
 * @param[out] hb_count     Heartbeat 송신 누적 (NULL 허용)
 * @param[out] bootup_count Bootup 송신 누적 (NULL 허용)
 * @note PnP Slave 는 System_Fdcan2_Transmit 을 tx_func 으로 쓰므로 xm_drv 의
 *   _Drv_TxFunc_Queue path 통계에는 잡히지 않는다. 실제 HB 계측은 여기.
 */
void System_GetFdcan2PnpTxStats(uint32_t* hb_count, uint32_t* bootup_count);

#endif /* SYSTEM_CORE_SYSTEM_STARTUP_H_ */
