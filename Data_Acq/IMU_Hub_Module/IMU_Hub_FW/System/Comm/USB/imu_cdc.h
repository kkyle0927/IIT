/**
 ******************************************************************************
 * @file    imu_cdc.h
 * @author  HyundoKim
 * @brief   [System/Comm/USB] IMU Hub USB CDC (BareMetal, SPSC Ring Buffer)
 * @version 1.0 (EMG Hub emg_cdc 패턴 적용)
 * @date    2026-03-21
 *
 * @details
 * BareMetal 환경 USB CDC 통신 인터페이스.
 * EMG Hub의 emg_cdc 패턴을 SM-IMU에 동일하게 적용.
 *
 * [SM USB-CDC 역할]
 * - 메인 데이터 경로: CAN-FD → XM → PC (양산)
 * - USB-CDC: 개발/진단용 보조 채널 (P0: 터미널, P1: CSV 스트리밍)
 *
 * [Tx — SPSC Lock-Free Ring Buffer + Line-Prefix Framing]
 * ImuCdc_TermPrintf/StreamPrintf/SysPrintf → prefix 태깅 → Ring Buffer
 *   → _TryTransmit() (CAS Guard) → CDC_Transmit_FS (PMA Copy)
 *   → TxComplete ISR → Chain Transmit
 *
 * Line-Prefix: '>' 터미널 응답, '$' 스트리밍 데이터, '!' 시스템 경고
 *
 * [Rx — SPSC Ring Buffer + Line Accumulator]
 * CDC_Receive_FS() (ISR) → ImuCdc_OnRxReceived() → Rx Ring Buffer
 *   → ImuCdc_ProcessRx() (Main Loop) → 라인 누적 → 콜백 호출
 *
 * [연결 관리 — DTR + Init/DeInit 이중 보호]
 * COM 포트 열기/닫기: DTR=1/0 → ImuCdc_OnDtrChanged()
 * USB 케이블 분리/재연결: CDC_DeInit/Init_FS → ImuCdc_SetConnected()
 * 어떤 경로든 _ResetState()로 클린 리셋 보장
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_COMM_USB_IMU_CDC_H_
#define SYSTEM_COMM_USB_IMU_CDC_H_

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS
 *-----------------------------------------------------------
 */

#define IMU_CDC_TX_RING_SIZE    4096  /**< Tx SPSC 링버퍼 크기 (4KB) */
#define IMU_CDC_TX_CHUNK_SIZE   512   /**< USB 1회 전송 최대 크기 */
#define IMU_CDC_TX_FMT_SIZE     512   /**< Printf 포맷 스테이징 버퍼 크기 */
#define IMU_CDC_RX_RING_SIZE    256   /**< Rx SPSC 링버퍼 크기 */
#define IMU_CDC_CMD_LINE_SIZE   64    /**< 명령 라인 최대 길이 */

/** Line-Prefix Framing 태그 */
#define IMU_CDC_PREFIX_TERM     '>'   /**< 터미널 명령 응답 */
#define IMU_CDC_PREFIX_STREAM   '$'   /**< 스트리밍 데이터 (CSV) */
#define IMU_CDC_PREFIX_SYS      '!'   /**< 시스템 경고/에러 */

/**
 *-----------------------------------------------------------
 * PUBLIC TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief CDC Rx 수신 콜백 타입
 * @param data 수신 데이터 포인터 (NULL-terminated 명령 문자열)
 * @param len  수신 길이
 */
typedef void (*ImuCdc_RxCallback_t)(const uint8_t *data, uint16_t len);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* ===== 초기화 ===== */

/**
 * @brief CDC 초기화 (Ring Buffer + Rx 상태 리셋)
 */
void ImuCdc_Init(void);

/* ===== Rx 경로 ===== */

/**
 * @brief Rx 수신 알림 (ISR 컨텍스트 — usbd_cdc_if.c에서 호출)
 * @param data 수신 데이터
 * @param len  수신 길이
 * @note ISR에서 호출. Rx Ring Buffer에 byte 단위 복사.
 */
void ImuCdc_OnRxReceived(const uint8_t *data, uint16_t len);

/**
 * @brief Rx Ring Buffer 처리 + 라인 파싱 (Main Loop에서 호출)
 * @details Ring에서 byte를 읽어 라인 누적, \r/\n 시 콜백 호출.
 */
void ImuCdc_ProcessRx(void);

/**
 * @brief Rx 콜백 등록
 * @param callback 수신 콜백 함수 (라인 단위로 호출됨)
 */
void ImuCdc_RegisterRxCallback(ImuCdc_RxCallback_t callback);

/* ===== Tx 경로 ===== */

/**
 * @brief 데이터 전송 (Ring Buffer에 복사 후 자동 전송)
 * @param data 전송 데이터
 * @param len  전송 길이
 * @return 0=성공 (Ring Buffer에 적재), -1=Ring Full (드롭), -2=파라미터 에러
 */
int ImuCdc_Send(const uint8_t *data, uint16_t len);

/**
 * @brief Raw 포맷 문자열 전송 (prefix 없음, 하위호환)
 * @param fmt 포맷 문자열
 * @return 0=성공, <0=에러
 */
int ImuCdc_Printf(const char *fmt, ...);

/**
 * @brief 터미널 응답 전송 (> prefix)
 * @param fmt 포맷 문자열
 * @return 0=성공, <0=에러
 */
int ImuCdc_TermPrintf(const char *fmt, ...);

/**
 * @brief 스트리밍 데이터 전송 ($ prefix)
 * @param fmt 포맷 문자열
 * @return 0=성공, <0=에러
 */
int ImuCdc_StreamPrintf(const char *fmt, ...);

/**
 * @brief 시스템 경고 전송 (! prefix)
 * @param fmt 포맷 문자열
 * @return 0=성공, <0=에러
 */
int ImuCdc_SysPrintf(const char *fmt, ...);

/**
 * @brief Tx 전송 완료 콜백 (USB ISR 컨텍스트 — usbd_cdc_if.c에서 호출)
 * @details Ring Buffer에 남은 데이터가 있으면 연쇄 전송을 시작합니다.
 */
void ImuCdc_OnTxComplete(void);

/* ===== 연결 관리 ===== */

/**
 * @brief USB CDC 연결 상태 확인
 * @return true=호스트 연결됨
 */
bool ImuCdc_IsConnected(void);

/**
 * @brief USB CDC 연결 상태 설정 (CDC_Init_FS / CDC_DeInit_FS에서 호출)
 * @param connected true=연결, false=해제
 * @note 내부적으로 _ResetState() 호출 — Tx/Rx 링 클린 리셋
 */
void ImuCdc_SetConnected(bool connected);

/**
 * @brief DTR (Data Terminal Ready) 변경 콜백 (usbd_cdc_if.c에서 호출)
 * @param dtr 1=COM 포트 열림, 0=COM 포트 닫힘
 * @note 내부적으로 _ResetState() 호출 — 재연결 시 클린 시작 보장
 */
void ImuCdc_OnDtrChanged(uint8_t dtr);

/* ===== Raw Rx 경로 (DOP Serial용) ===== */

/**
 * @brief Rx Ring Buffer에서 raw 바이트를 읽어 반환 (라인 파싱 없이)
 * @param[out] buf 출력 버퍼
 * @param max_len 출력 버퍼 최대 크기
 * @return 읽은 바이트 수 (0=데이터 없음)
 * @note ImuCdc_ProcessRx()와 동시 사용 금지 — 둘 중 하나만 호출.
 */
uint32_t ImuCdc_DrainRxRaw(uint8_t *buf, uint32_t max_len);

/* ===== 통계 ===== */

/**
 * @brief Tx 드롭 카운터 조회
 * @return Ring Buffer Full로 인해 드롭된 횟수
 */
uint32_t ImuCdc_GetTxDropCount(void);

/**
 * @brief Tx 드롭 카운터 리셋
 */
void ImuCdc_ResetTxDropCount(void);

#endif /* SYSTEM_COMM_USB_IMU_CDC_H_ */