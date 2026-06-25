/**
 ******************************************************************************
 * @file    io_manager.h
 * @author  HyundoKim
 * @brief   [System/BOARD] I/O (LED & Button) 통합 관리자
 * @version 1.0
 * @date    Dec 2, 2025
 *
 * @details
 * LED와 Button의 시간 기반 처리를 담당합니다.
 * XM10의 XM_IO_Update() 스타일을 따릅니다.
 * 
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_BOARD_IO_MANAGER_H
#define SYSTEM_BOARD_IO_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/* ===== LED Section ===== */

/**
 * @brief LED 동작 모드
 */
typedef enum {
    IO_LED_OFF           = 0,  /**< LED 끄기 */
    IO_LED_SOLID         = 1,  /**< Solid — 계속 켜기 */
    IO_LED_BLINK         = 2,  /**< Blinking — 200ms ON / 200ms OFF (2.5Hz) */
    IO_LED_FLICKERING    = 3,  /**< Flickering — 50ms ON / 50ms OFF (10Hz) */
    IO_LED_SINGLE_FLASH  = 4,  /**< Single Flash — 200ms ON / 1000ms OFF */
    IO_LED_DOUBLE_FLASH  = 5,  /**< Double Flash — 2×(200ms ON/OFF) + 1000ms OFF */
    IO_LED_TRIPLE_FLASH  = 6,  /**< Triple Flash — 3×(200ms ON/OFF) + 1000ms OFF */
} IO_LedMode_t;

/**
 * @brief XM 연결 상태 (Status LED 색상 결정용)
 */
typedef enum {
    IO_LINK_STATE_BOOT_UP          = 0x00,  /**< 부팅 중 — White Solid */
    IO_LINK_STATE_DISCONNECTED     = 0x01,  /**< Master 미감지 — Red Solid */
    IO_LINK_STATE_PRE_OPERATIONAL  = 0x02,  /**< Master 감지, Pre-Op — Green Single Flash */
    IO_LINK_STATE_OPERATIONAL      = 0x03,  /**< 정상 동작 — Green Solid */
    IO_LINK_STATE_HEARTBEAT_LOST   = 0x04,  /**< Master HB 타임아웃 — Red Double Flash */
    IO_LINK_STATE_ERROR            = 0x05,  /**< 일반 에러 — Red Solid */
} IO_LinkState_t;

/* ===== Button Section ===== */

/**
 * @brief 버튼 이벤트 타입
 */
typedef enum {
    IO_BTN_NONE       = 0,  /**< 이벤트 없음 */
    IO_BTN_PRESSED    = 1,  /**< 누른 순간 */
    IO_BTN_RELEASED   = 2,  /**< 뗀 순간 */
    IO_BTN_CLICK      = 3,  /**< 짧게 클릭 */
    IO_BTN_LONG_PRESS = 4,  /**< 길게 누름 (1초) */
} IO_ButtonEvent_t;

/**
 *-----------------------------------------------------------
 * PUBLIC API
 *-----------------------------------------------------------
 */

/**
 * @brief I/O Manager 초기화
 * @details system_startup에서 호출됩니다.
 */
void IO_Manager_Init(void);

/**
 * @brief I/O 상태 업데이트 (Tick 기반)
 * @param timestamp_ms 현재 시스템 시각 (ms)
 * @details Core_RunLoop()에서 주기적으로 호출됩니다.
 */
void IO_Manager_Update(uint32_t timestamp_ms);

/* ===== Status LED API ===== */

/**
 * @brief Status LED 상태 설정 (XM 연결 상태 기반)
 * @param state 연결 상태
 */
void IO_SetLinkState(IO_LinkState_t state);

/* ===== Function Button API ===== */

/**
 * @brief 버튼 상태 확인
 * @param btn_index 버튼 번호 (1 또는 2)
 * @return true=눌림, false=안 눌림
 */
bool IO_GetButtonState(uint8_t btn_index);

/**
 * @brief 버튼 이벤트 가져오기 (Read & Clear)
 * @param btn_index 버튼 번호 (1 또는 2)
 * @return 이벤트 타입
 */
IO_ButtonEvent_t IO_GetButtonEvent(uint8_t btn_index);

#endif /* SYSTEM_BOARD_IO_MANAGER_H */

