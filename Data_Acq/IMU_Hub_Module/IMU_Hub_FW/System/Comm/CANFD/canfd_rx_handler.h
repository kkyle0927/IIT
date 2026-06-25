/**
 ******************************************************************************
 * @file    canfd_rx_handler.h
 * @author  HyundoKim
 * @brief   [System/Comm] CANFD 수신 핸들러 (IMU Hub Module용)
 * @version 2.2
 * @date    Dec 2, 2025
 *
 * @details
 * IMU Hub Module의 CANFD 수신 처리를 담당합니다.
 * 
 * [처리 방식 - BareMetal 최적화]
 * - 모든 메시지(PDO/Heartbeat/SDO): ISR에서 직접 처리
 * - SDO는 단발성 데이터이므로 Pending Buffer 불필요
 * - 단순한 callback만 수행하므로 ISR에서 처리 가능
 * 
 * [Main Loop 통합]
 * Main Loop에서는 추가 호출 불필요합니다.
 * 모든 처리는 ISR에서 완료됩니다.
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_
#define SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_

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
 * @brief CANFD Rx 핸들러 초기화
 * @param fdcan_id IOIF FDCAN 인스턴스 ID
 * @details System Startup에서 IOIF 초기화 후 호출합니다.
 */
void CanFdRxHandler_Init(IOIF_FDCANx_t fdcan_id);

#endif /* SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_ */
