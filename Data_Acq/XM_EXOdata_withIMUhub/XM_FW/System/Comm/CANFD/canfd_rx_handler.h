/**
 ******************************************************************************
 * @file    canfd_rx_handler.h
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Oct 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_
#define SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_

/* ✅ V3.1: Queue 제거, Task Notification만 사용 (헤더 불필요) */

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
 * @brief CAN-FD 수신 처리 서비스를 초기화하고 관련 태스크를 생성합니다.
 * @details 
 * [V3.1 최적화]
 * - Task Notification + SW RingBuffer (Queue 제거)
 * - ISR 오버헤드 10배 향상 (~5µs → ~0.5µs)
 * - Batch Processing으로 Context Switch 최소화
 * 
 * 시스템 시작 시 startup.c에서 단 한 번만 호출되어야 합니다.
 */
void CanFdRxHandler_Init(void);

#endif /* SYSTEM_COMM_CANFD_INC_CANFD_RX_HANDLER_H_ */
