/**
 ******************************************************************************
 * @file    canfd_rx_handler.c
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
 * - SDO는 단발성 데이터, 단순한 callback만 수행하므로 ISR에서 처리 가능
 * 
 * [설계 원칙]
 * 1. ISR은 최대한 짧게 (< 20μs)
 * 2. 모든 메시지를 즉시 처리 (지연 없음)
 * 3. 단순한 구조 (버퍼/큐 불필요)
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "canfd_rx_handler.h"
#include "ioif_agrb_fdcan.h"
#include "xm_drv.h"
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
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* ✅ s_fdcan_id 제거: xm_drv.c에서 관리, 여기선 콜백만 등록 */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void CanFdRxHandler_Init(IOIF_FDCANx_t fdcan_id)
{
    /* ✅ IOIF 콜백만 등록 (FDCAN ID 저장 불필요) */
    IOIF_FDCAN_REGISTER_CALLBACK(fdcan_id, _CanFdRxCallback);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IOIF FDCAN 수신 콜백 (ISR Context)
 * @details 
 * [CANopen 표준] Device Driver로 직접 전달하여 CANopen 메시지를 처리합니다.
 * 
 * [메시지 처리 흐름]
 * ISR → XM_Drv_ProcessCANMessage()
 *     ├─ CAN ID 0x000: NMT Command → AGR_NMT
 *     ├─ CAN ID 0x700: Boot-up, Heartbeat → AGR_NMT
 *     ├─ CAN ID 0x600: SDO Request → AGR_DOP
 *     └─ CAN ID 0x180/0x280: PDO → AGR_DOP
 * 
 * [BareMetal 환경 ISR 안전성]
 * - AGR_NMT_ProcessMessage는 내부 상태를 변경하므로 Main Loop과 충돌 가능
 * - NVIC 우선순위 차별화로만 보호한다 (전역 IRQ 차단 API 사용 금지).
 *   전역 IRQ 차단은 센서 샘플링 타이밍을 깨뜨리므로 Rule에서 금지됨.
 *   상태 공유가 필요한 변수는 volatile + ISR에서만 write / Main에서 read-only
 *   또는 double-buffer 패턴으로 설계할 것.
 * - ISR은 Nested 인터럽트를 우선순위로 처리
 *
 * @note ISR 실행 시간 목표: < 50μs (CANopen 메시지 파싱 포함)
 */
static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg)
{
    if (msg == NULL) {
        return;
    }
    
    /* CANopen 메시지 처리 (Device Driver 직접 호출) */
    XM_Drv_ProcessCANMessage(msg->id, msg->data, msg->len);
}
