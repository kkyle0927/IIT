/**
 ******************************************************************************
 * @file    uart_rx_handler.c
 * @author  HyundoKim
 * @brief   [System/Comm] UART 다채널 수신 핸들러 (Auto-Detect + DataLake)
 * @version 6.0 (Parser Registry + Hot-Swap Re-Detect)
 * @date    Feb 11, 2026
 *
 * @details
 * IMU Hub의 UART 수신 관리를 담당합니다.
 *
 * [역할]
 * - 6개 UART 채널의 데이터 수신 (ISR)
 * - Parser Registry 기반 Auto-Detect: 등록된 모든 파서를 동시 probe → 첫 유효
 *   패킷의 센서로 Lock. 새 IMU 추가 = 레지스트리 테이블 1행 + 어댑터.
 * - 감지 후 Device Layer DataLake API로 데이터 전달
 * - [R2] Hot-Swap: 분리(Auto-Sense STOPPED) 감지 시 UNKNOWN 으로 리셋 + 파서 reinit
 *   → 재연결/다른 센서 교체 시 재부팅 없이 재감지
 *
 * [Auto-Detect 알고리즘 (Registry Probing)]
 * 1. 모든 채널 UNKNOWN으로 시작
 * 2. ISR에서 수신 바이트를 레지스트리의 모든 파서에 순차 투입
 * 3. 먼저 유효 패킷을 완성한 파서의 센서 타입으로 Lock
 * 4. Lock 후 해당 파서만 사용
 *
 * [ISR Priority 패턴 (BareMetal, NVIC 실측)]
 * - UART RX ISR (preempt 0): DataLake 쓰기(writer) — 최상위, Timer ISR 선점 가능
 * - Timer ISR  (preempt 1): DataLake 읽기(reader, FetchAllImus)
 * - writer(0) > reader(1) 이므로 torn-read 가능 → Device DataLake 는 seqlock(R6)로 보호
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "uart_rx_handler.h"
#include "module.h"
#include "ebimu-9dofv6.h"
#include "mti-630.h"
#include "ioif_agrb_tim.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief 채널별 관리 객체 (Registry Probing + DataLake 패턴)
 * @details UNKNOWN 상태에서 레지스트리의 모든 파서를 동시 구동, Lock 후 확정 파서만 사용.
 *   파서 상태(EbimuParser_t/XsensParser_t)는 타입이 달라 각각 보유.
 */
typedef struct {
    IOIF_UARTx_t     uart_id;
    ImuSensorType_t  type;             /**< 감지된 센서 타입 (UNKNOWN → EBIMU/XSENS) */
    volatile bool    redetect_pending; /**< [R2] Timer ISR set, UART ISR consume — 재감지 요청 */
    EbimuParser_t    ebimu_parser;     /**< EBIMU 파서 인스턴스 */
    XsensParser_t    xsens_parser;     /**< XSENS 파서 인스턴스 */
} ImuRxContext_t;

/** @brief 레지스트리 엔트리당 저장 가능한 sync(header) 바이트 최대 길이 */
#define IMU_PARSER_SYNC_MAX  (4)

/**
 * @brief 파서 레지스트리 엔트리 (확장: 새 IMU = 1행 + 어댑터 함수)
 * @details 파서별 타입이 달라 ctx/ch 기반 어댑터로 공통 인터페이스 제공.
 *
 * @note [확장 hook — sync[]/sync_len] 각 프로토콜의 프레임 선두 고정 바이트열
 *   (EBIMU=0x55 0x55, XSENS=0xFA 0xFF). 현재 Auto-Detect 는 모든 파서에 바이트를
 *   순차 투입하는 선형 probe(레지스트리 2종 → 무문제)지만, 파서가 10+ 로 늘면 첫
 *   sync 바이트로 단일 파서를 O(1) 라우팅하는 헤더 디스패치로 전환할 수 있도록
 *   데이터를 미리 노출해 둔다. **현재 probe 로직은 sync[] 를 읽지 않음**(YAGNI —
 *   활성화는 종류 급증/ISR 지연 관측 시). 참조 메모리: ebimu-checksum-and-parser-scaling-todo.
 */
typedef struct {
    ImuSensorType_t type;
    const char*     name;
    uint8_t         sync[IMU_PARSER_SYNC_MAX];                     /**< 프레임 선두 고정 바이트열 (확장 hook, 현재 미사용) */
    uint8_t         sync_len;                                      /**< 유효 sync 바이트 수 (0 = 미지정) */
    void (*InitParser)(ImuRxContext_t* ctx);                       /**< 파서 인스턴스 초기화 */
    bool (*Parse)(ImuRxContext_t* ctx, uint8_t byte, uint8_t ch);  /**< 1바이트 파싱; 패킷 완성 시 DataLake update + true */
    bool (*IsOnline)(uint8_t ch);                                  /**< Auto-Sense online 여부 */
    void (*StateInit)(uint8_t ch);                                 /**< DataLake/Auto-Sense 초기화 */
    bool (*Fetch)(uint8_t ch, EBIMU_Data_t* out);                  /**< DataLake → 공통 EBIMU_Data_t */
    void (*ConfigureOutput)(IOIF_UARTx_t uart_id);                 /**< 센서 출력 사전설정 */
} ImuParserReg_t;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static ImuRxContext_t s_imus[MAX_IMU_CHANNELS];
static uint8_t        s_channel_count = 0;

/**
 *------------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _OnUartReceive(uint8_t* rx_buf, uint32_t size, uint32_t id);

/* ===== EBIMU 어댑터 ===== */
static void _EbimuInit(ImuRxContext_t* ctx)                       { ebimu9dofv6.InitParser(&ctx->ebimu_parser); }
static bool _EbimuParse(ImuRxContext_t* ctx, uint8_t byte, uint8_t ch)
{
    EBIMU_Data_t tmp;
    if (ebimu9dofv6.ParseByte(&ctx->ebimu_parser, byte, &tmp)) {
        tmp.timestamp = IOIF_TIM_GetTick();
        tmp.imu_index = ch;
        EbimuV6_UpdateData(ch, &tmp);
        return true;
    }
    return false;
}
static bool _EbimuFetch(uint8_t ch, EBIMU_Data_t* out)           { return EbimuV6_GetLatest(ch, out); }
static void _EbimuConfigure(IOIF_UARTx_t uart_id)                { ebimu9dofv6.ConfigureOutput(uart_id); }

/* ===== XSENS 어댑터 (packet → 공통 EBIMU_Data_t 변환) ===== */
static void _XsensInit(ImuRxContext_t* ctx)                       { xsensMTi630.InitParser(&ctx->xsens_parser); }
static bool _XsensParse(ImuRxContext_t* ctx, uint8_t byte, uint8_t ch)
{
    XsensMTi_packet_t tmp;
    if (xsensMTi630.ParseByte(&ctx->xsens_parser, byte, &tmp)) {
        tmp.timestamp = IOIF_TIM_GetTick();
        tmp.imu_index = ch;
        XsensMTi_UpdateData(ch, &tmp);
        return true;
    }
    return false;
}
static bool _XsensFetch(uint8_t ch, EBIMU_Data_t* out)
{
    XsensMTi_packet_t x;
    if (!XsensMTi_GetLatest(ch, &x)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->timestamp = x.timestamp;
    out->imu_index = x.imu_index;
    out->q_w = x.q_w; out->q_x = x.q_x; out->q_y = x.q_y; out->q_z = x.q_z;
    out->acc_x = x.acc_x; out->acc_y = x.acc_y; out->acc_z = x.acc_z;
    out->gyr_x = x.gyr_x; out->gyr_y = x.gyr_y; out->gyr_z = x.gyr_z;
    /* Euler/Mag 는 PDO 미포함 (0 유지) */
    return true;
}
static void _XsensConfigure(IOIF_UARTx_t uart_id)                { xsensMTi630.ConfigureOutput(uart_id); }

/* ===== 파서 레지스트리 (probe 순서 = 배열 순서. 새 IMU = 1행 추가) =====
 * sync/sync_len = 프레임 선두 고정 바이트열(확장 hook, 현재 probe 미사용 — struct 주석 참조). */
static const ImuParserReg_t s_parser_reg[] = {
    { IMU_SENSOR_EBIMU, "EBIMU", { EBIMU_HEADER_BYTE, EBIMU_HEADER_BYTE }, 2,
      _EbimuInit, _EbimuParse, EbimuV6_IsOnline,  EbimuV6_StateInit,  _EbimuFetch, _EbimuConfigure },
    { IMU_SENSOR_XSENS, "XSENS", { XSENS_PREAMBLE,    XSENS_BID         }, 2,
      _XsensInit, _XsensParse, XsensMTi_IsOnline, XsensMTi_StateInit, _XsensFetch, _XsensConfigure },
};
#define IMU_PARSER_REG_COUNT  (sizeof(s_parser_reg) / sizeof(s_parser_reg[0]))

static const ImuParserReg_t* _FindReg(ImuSensorType_t type)
{
    for (uint8_t r = 0; r < IMU_PARSER_REG_COUNT; r++) {
        if (s_parser_reg[r].type == type) {
            return &s_parser_reg[r];
        }
    }
    return NULL;
}

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void UartRxHandler_Init(const IOIF_UARTx_t* uart_ids, uint8_t count)
{
    s_channel_count = (count > MAX_IMU_CHANNELS) ? MAX_IMU_CHANNELS : count;

    for (uint8_t i = 0; i < s_channel_count; i++) {
        s_imus[i].uart_id          = uart_ids[i];
        s_imus[i].type             = IMU_SENSOR_UNKNOWN;   /* Auto-Detect: UNKNOWN 시작 */
        s_imus[i].redetect_pending = false;

        /* 레지스트리의 모든 파서 초기화 + Device DataLake StateInit */
        for (uint8_t r = 0; r < IMU_PARSER_REG_COUNT; r++) {
            s_parser_reg[r].InitParser(&s_imus[i]);
            s_parser_reg[r].StateInit(i);
        }
    }

    /* UART Rx Idle 콜백 등록 */
    for (uint8_t i = 0; i < s_channel_count; i++) {
        IOIF_UART_SetRxIdleCallback(uart_ids[i], _OnUartReceive);
    }

    /* Note: ConfigureOutput 미호출 — 사전설정 센서는 전원 인가 시 바로 출력 → ISR 자동감지.
     * 미설정 센서는 UartRxHandler_ConfigureAllAs() 별도 호출. */
}

void UartRxHandler_FetchAllImus(EBIMU_Data_t* out_data)
{
    if (out_data == NULL) return;

    for (uint8_t i = 0; i < s_channel_count; i++) {
        const ImuParserReg_t* reg = _FindReg(s_imus[i].type);
        if (reg == NULL || !reg->Fetch(i, &out_data[i])) {
            memset(&out_data[i], 0, sizeof(EBIMU_Data_t));   /* UNKNOWN/offline/torn → 0 */
        }
    }
}

ImuSensorType_t UartRxHandler_GetSensorType(uint8_t ch)
{
    if (ch >= s_channel_count) return IMU_SENSOR_UNKNOWN;
    return s_imus[ch].type;
}

void UartRxHandler_ConfigureAllAs(ImuSensorType_t type)
{
    const ImuParserReg_t* reg = _FindReg(type);
    if (reg == NULL) return;
    for (uint8_t i = 0; i < s_channel_count; i++) {
        reg->ConfigureOutput(s_imus[i].uart_id);
    }
}

void UartRxHandler_RunPeriodic(void)
{
    /* [R2] Hot-Swap 재감지: Lock 된 채널이 offline(Auto-Sense STOPPED) 전환되면
     * UNKNOWN 으로 리셋 + 모든 파서 reinit → 재연결/다른 센서 교체 시 ISR 이 재감지.
     * (Auto-Sense timeout 체크는 호출자가 EbimuV6/XsensMTi_RunPeriodic 으로 먼저 수행) */
    for (uint8_t i = 0; i < s_channel_count; i++) {
        if (s_imus[i].type == IMU_SENSOR_UNKNOWN) {
            continue;
        }
        const ImuParserReg_t* reg = _FindReg(s_imus[i].type);
        if (reg != NULL && !reg->IsOnline(i)) {
            /* offline 감지 → flag 만 set. 실제 type/parser 리셋은 다음 수신(재연결) 시
             * UART ISR 가 수행 → type/parser 쓰기를 UART ISR 단일 컨텍스트로 일원화
             * (race-free). offline 채널은 UART ISR 가 dormant 이라 flag 가 안전히 대기. */
            s_imus[i].redetect_pending = true;
        }
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - UART 수신 콜백 (Registry Probing)
 *------------------------------------------------------------
 */

/**
 * @brief [ISR] UART 데이터 수신 콜백 (Registry Probing + DataLake)
 * @details IDLE Event 기반 패킷 단위 수신.
 *   UNKNOWN: 레지스트리 모든 파서에 바이트 투입, 먼저 완성된 파서로 Lock 후 잔여 바이트
 *            전용 처리. Locked: 확정 파서만 사용.
 */
static void _OnUartReceive(uint8_t* rx_buf, uint32_t size, uint32_t id)
{
    /* 1. ID로 인스턴스 찾기 */
    ImuRxContext_t* ctx = NULL;
    uint8_t idx = 0;
    for (uint8_t i = 0; i < s_channel_count; i++) {
        if (s_imus[i].uart_id == id) {
            ctx = &s_imus[i];
            idx = i;
            break;
        }
    }
    if (ctx == NULL) return;

    /* [R2] 재감지 대기 flag(Timer ISR set) consume — type/parser 리셋을 UART ISR(단일
     * writer)에서 수행. 재연결/센서 교체 후 첫 수신 시점에 clean reprobe → race-free. */
    if (ctx->redetect_pending) {
        ctx->redetect_pending = false;
        ctx->type = IMU_SENSOR_UNKNOWN;
        for (uint8_t r = 0; r < IMU_PARSER_REG_COUNT; r++) {
            s_parser_reg[r].InitParser(ctx);
        }
    }

    if (ctx->type == IMU_SENSOR_UNKNOWN) {
        /* ===== Registry Probing: 모든 파서 동시 투입, 첫 완성으로 Lock ===== */
        for (uint32_t k = 0; k < size; k++) {
            for (uint8_t r = 0; r < IMU_PARSER_REG_COUNT; r++) {
                if (s_parser_reg[r].Parse(ctx, rx_buf[k], idx)) {
                    ctx->type = s_parser_reg[r].type;            /* Lock */
                    /* 잔여 바이트는 확정 파서 전용 처리 */
                    for (uint32_t j = k + 1; j < size; j++) {
                        (void)s_parser_reg[r].Parse(ctx, rx_buf[j], idx);
                    }
                    return;
                }
            }
        }
        return;
    }

    /* ===== Locked: 확정 파서만 사용 ===== */
    const ImuParserReg_t* reg = _FindReg(ctx->type);
    if (reg == NULL) return;   /* 정의되지 않은 타입 — 무시 (메모리 손상 방지) */
    for (uint32_t k = 0; k < size; k++) {
        (void)reg->Parse(ctx, rx_buf[k], idx);
    }
}
