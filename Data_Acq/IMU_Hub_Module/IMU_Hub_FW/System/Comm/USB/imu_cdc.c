/**
 ******************************************************************************
 * @file    imu_cdc.c
 * @author  HyundoKim
 * @brief   [System/Comm/USB] IMU Hub USB CDC 구현 (BareMetal, SPSC Ring Buffer)
 * @version 1.0 (EMG Hub emg_cdc 패턴 적용)
 * @date    2026-03-21
 *
 * @details
 * [V1.0] EMG Hub emg_cdc.c v3.0 패턴을 SM-IMU에 동일하게 적용.
 *
 * [Tx 동시성 모델 — BareMetal SPSC]
 * Producer: Timer ISR (TIM16, 1ms) → ImuCdc_Send()에서 head 갱신
 * Consumer: USB TxCplt ISR → ImuCdc_OnTxComplete() → _TryTransmit()에서 tail 갱신
 * Guard: atomic_compare_exchange_strong (LDREX/STREX on Cortex-M4)
 *
 * [Rx 동시성 모델 — BareMetal SPSC]
 * Producer: USB Rx ISR → ImuCdc_OnRxReceived()에서 head 갱신
 * Consumer: Timer ISR (Main Loop) → ImuCdc_ProcessRx()에서 tail 갱신
 * Line Accumulator: \r/\n 단위로 명령 파싱 → 콜백 호출
 *
 * [G4 특화]
 * - D-Cache 없음 → 일반 SRAM, Clean/Invalidate 불필요
 * - USB PMA Copy → CDC_Transmit_FS 반환 후 소스 버퍼 즉시 해제
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_cdc.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdatomic.h>

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES — Tx (SPSC Lock-Free Ring Buffer)
 *-----------------------------------------------------------
 */

/** @brief Tx 링버퍼 (일반 SRAM — G4 D-Cache 없음) */
static uint8_t s_tx_ring[IMU_CDC_TX_RING_SIZE];

/** @brief Tx head (Producer 전용 쓰기, Consumer 읽기) */
static volatile atomic_uint s_tx_head = 0;

/** @brief Tx tail (Consumer 전용 쓰기 — _TryTransmit CAS Guard 내부) */
static volatile uint32_t s_tx_tail = 0;

/** @brief Tx 전송 진행 중 플래그 (CAS로 동시 진입 방지) */
static volatile atomic_bool s_tx_busy = false;

/** @brief Tx 드롭 카운터 (Ring Full 시 증가) */
static volatile atomic_uint s_tx_drop_count = 0;

/** @brief Printf 포맷 스테이징 버퍼 */
static uint8_t s_tx_fmt_buf[IMU_CDC_TX_FMT_SIZE];

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES — Rx (SPSC Ring Buffer + Line Accumulator)
 *-----------------------------------------------------------
 */

/** @brief Rx SPSC Ring Buffer */
static uint8_t s_rx_ring[IMU_CDC_RX_RING_SIZE];

/** @brief Rx head (USB Rx ISR 전용 쓰기) */
static volatile atomic_uint s_rx_head = 0;

/** @brief Rx tail (Main Loop 전용 읽기) */
static volatile uint32_t s_rx_tail = 0;

/** @brief 명령 라인 누적 버퍼 */
static uint8_t  s_cmd_line[IMU_CDC_CMD_LINE_SIZE];
static uint8_t  s_cmd_pos = 0;

/** @brief Rx 콜백 */
static ImuCdc_RxCallback_t s_rx_callback = NULL;

/** @brief 연결 상태 */
static volatile bool s_is_connected = false;

/**
 *-----------------------------------------------------------
 * PRIVATE FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void _TryTransmit(void);
static void _ResetState(bool connected);
static int  _TaggedPrintf(char prefix, const char *fmt, va_list args);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — 초기화
 *-----------------------------------------------------------
 */

void ImuCdc_Init(void)
{
    _ResetState(false);
    s_rx_callback = NULL;
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — Tx 경로
 *-----------------------------------------------------------
 */

int ImuCdc_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return -2;
    if (len > IMU_CDC_TX_RING_SIZE - 1) return -2;

    uint32_t head = atomic_load(&s_tx_head);
    uint32_t tail = s_tx_tail;

    /* Free space 계산 */
    uint32_t used = (head >= tail)
        ? (head - tail)
        : (IMU_CDC_TX_RING_SIZE - tail + head);
    uint32_t free_space = IMU_CDC_TX_RING_SIZE - 1 - used;

    if (len > free_space) {
        atomic_fetch_add(&s_tx_drop_count, 1);
        return -1;
    }

    /* Ring Buffer에 복사 (Wrap-around) */
    uint32_t space_to_end = IMU_CDC_TX_RING_SIZE - head;
    if (space_to_end >= len) {
        memcpy(&s_tx_ring[head], data, len);
    } else {
        memcpy(&s_tx_ring[head], data, space_to_end);
        memcpy(&s_tx_ring[0], data + space_to_end, len - space_to_end);
    }

    /* Head 갱신 (atomic_store = memory barrier 포함) */
    atomic_store(&s_tx_head, (head + len) % IMU_CDC_TX_RING_SIZE);

    /* 전송 트리거 (idle일 때만) */
    if (!atomic_load(&s_tx_busy)) {
        _TryTransmit();
    }

    return 0;
}

int ImuCdc_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)s_tx_fmt_buf, IMU_CDC_TX_FMT_SIZE, fmt, args);
    va_end(args);

    if (len <= 0) return -2;
    if (len > (int)IMU_CDC_TX_FMT_SIZE) len = IMU_CDC_TX_FMT_SIZE;

    return ImuCdc_Send(s_tx_fmt_buf, (uint16_t)len);
}

int ImuCdc_TermPrintf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int ret = _TaggedPrintf(IMU_CDC_PREFIX_TERM, fmt, args);
    va_end(args);
    return ret;
}

int ImuCdc_StreamPrintf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int ret = _TaggedPrintf(IMU_CDC_PREFIX_STREAM, fmt, args);
    va_end(args);
    return ret;
}

int ImuCdc_SysPrintf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int ret = _TaggedPrintf(IMU_CDC_PREFIX_SYS, fmt, args);
    va_end(args);
    return ret;
}

void ImuCdc_OnTxComplete(void)
{
    atomic_store(&s_tx_busy, false);
    _TryTransmit();
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — Rx 경로
 *-----------------------------------------------------------
 */

/**
 * @brief [ISR] Rx 수신 — Rx Ring Buffer에 byte 단위 복사
 * @note USB Rx ISR 컨텍스트에서 호출. 빠르게 ring에 복사만 수행.
 */
void ImuCdc_OnRxReceived(const uint8_t *data, uint16_t len)
{
    uint32_t head = atomic_load(&s_rx_head);

    for (uint16_t i = 0; i < len; i++) {
        uint32_t next = (head + 1) % IMU_CDC_RX_RING_SIZE;
        if (next == s_rx_tail) break;  /* Ring Full — 나머지 드롭 */
        s_rx_ring[head] = data[i];
        head = next;
    }

    atomic_store(&s_rx_head, head);

    /* 데이터 수신 = 호스트 연결 확인 */
    s_is_connected = true;
}

/**
 * @brief [Main Loop] Rx Ring Buffer에서 byte를 읽어 라인 단위로 파싱
 * @details \r 또는 \n을 만나면 누적된 명령을 콜백으로 전달.
 *          스트리밍 중에도 명령 유실 없음 (ring buffer 덕분).
 */
void ImuCdc_ProcessRx(void)
{
    uint32_t head = atomic_load(&s_rx_head);
    uint32_t tail = s_rx_tail;

    while (tail != head) {
        uint8_t ch = s_rx_ring[tail];
        tail = (tail + 1) % IMU_CDC_RX_RING_SIZE;

        if (ch == '\r' || ch == '\n') {
            if (s_cmd_pos > 0) {
                s_cmd_line[s_cmd_pos] = '\0';
                if (s_rx_callback != NULL) {
                    s_rx_callback(s_cmd_line, s_cmd_pos);
                }
                s_cmd_pos = 0;
            }
        } else if (s_cmd_pos < IMU_CDC_CMD_LINE_SIZE - 1) {
            s_cmd_line[s_cmd_pos++] = ch;
        }
        /* else: 명령이 너무 길면 초과 문자 무시 */
    }

    s_rx_tail = tail;
}

void ImuCdc_RegisterRxCallback(ImuCdc_RxCallback_t callback)
{
    s_rx_callback = callback;
}

uint32_t ImuCdc_DrainRxRaw(uint8_t *buf, uint32_t max_len)
{
    if (buf == NULL || max_len == 0) return 0;

    uint32_t head = atomic_load(&s_rx_head);
    uint32_t tail = s_rx_tail;
    uint32_t count = 0;

    while (tail != head && count < max_len) {
        buf[count++] = s_rx_ring[tail];
        tail = (tail + 1) % IMU_CDC_RX_RING_SIZE;
    }

    s_rx_tail = tail;
    return count;
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — 연결 관리
 *-----------------------------------------------------------
 */

bool ImuCdc_IsConnected(void)
{
    return s_is_connected;
}

void ImuCdc_SetConnected(bool connected)
{
    _ResetState(connected);
}

void ImuCdc_OnDtrChanged(uint8_t dtr)
{
    _ResetState(dtr ? true : false);
}

uint32_t ImuCdc_GetTxDropCount(void)
{
    return atomic_load(&s_tx_drop_count);
}

void ImuCdc_ResetTxDropCount(void)
{
    atomic_store(&s_tx_drop_count, 0);
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief 통합 상태 리셋 — 어떤 연결/해제 경로든 동일한 클린 리셋 보장
 * @param connected true=연결됨 (클린 시작), false=해제됨 (완전 중지)
 */
static void _ResetState(bool connected)
{
    s_is_connected = connected;

    /* Tx 리셋 */
    atomic_store(&s_tx_busy, false);
    atomic_store(&s_tx_head, 0);
    s_tx_tail = 0;
    atomic_store(&s_tx_drop_count, 0);

    /* Rx 리셋 */
    atomic_store(&s_rx_head, 0);
    s_rx_tail = 0;
    s_cmd_pos = 0;
}

/**
 * @brief Tagged Printf 내부 구현
 * @param prefix Line-Prefix 문자 ('>', '$', '!')
 * @param fmt    포맷 문자열
 * @param args   가변 인자 리스트
 * @return 0=성공, <0=에러
 */
static int _TaggedPrintf(char prefix, const char *fmt, va_list args)
{
    /* prefix + space */
    s_tx_fmt_buf[0] = (uint8_t)prefix;
    s_tx_fmt_buf[1] = ' ';

    int len = vsnprintf((char *)&s_tx_fmt_buf[2], IMU_CDC_TX_FMT_SIZE - 2, fmt, args);

    if (len <= 0) return -2;
    if (len > (int)(IMU_CDC_TX_FMT_SIZE - 2)) len = (int)(IMU_CDC_TX_FMT_SIZE - 2);

    return ImuCdc_Send(s_tx_fmt_buf, (uint16_t)(len + 2));
}

/**
 * @brief 실제 USB 전송 시도 (Lock-Free CAS Guard)
 * @details
 * Timer ISR (ImuCdc_Send) 또는 USB ISR (ImuCdc_OnTxComplete) 양쪽에서 호출.
 * atomic_compare_exchange_strong로 동시 진입 방지.
 *
 * Ring Buffer의 tail부터 연속된 데이터를 CDC_Transmit_FS에 전달.
 * G4 USB: PMA Copy이므로 CDC_Transmit_FS 반환 후 소스 즉시 해제.
 */
static void _TryTransmit(void)
{
    /* CAS Guard: 동시 진입 방지 (LDREX/STREX on Cortex-M4) */
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_tx_busy, &expected, true)) {
        return;
    }

    uint32_t head = atomic_load(&s_tx_head);
    uint32_t tail = s_tx_tail;

    if (head == tail) {
        atomic_store(&s_tx_busy, false);
        return;
    }

    /* Contiguous chunk 계산 */
    uint32_t len = (head > tail)
        ? (head - tail)
        : (IMU_CDC_TX_RING_SIZE - tail);
    if (len > IMU_CDC_TX_CHUNK_SIZE) len = IMU_CDC_TX_CHUNK_SIZE;

    /* USB 전송 (G4: PMA Copy → 반환 후 소스 즉시 해제) */
    if (CDC_Transmit_FS(&s_tx_ring[tail], (uint16_t)len) == USBD_OK) {
        s_tx_tail = (tail + len) % IMU_CDC_TX_RING_SIZE;
    } else {
        atomic_store(&s_tx_busy, false);
    }
}
