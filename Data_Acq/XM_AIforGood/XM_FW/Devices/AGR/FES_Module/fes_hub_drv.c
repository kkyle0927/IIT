/**
 ******************************************************************************
 * @file    fes_hub_drv.c
 * @author  HyundoKim
 * @brief   [Device Layer] XM10 <-> FES Hub 통신 드라이버 (Master, DOP V3 ES-vector)
 * @version 2.0
 * @date    2026-04-09
 *
 * @details
 * IMU Hub 드라이버(imu_hub_drv.c V4.0) 패턴을 기반으로 FES Hub ES-vector 모델 구현.
 *
 * [V2.0 변경 — ES-vector 모델]
 * - 기존 14B RPDO 전송 (SendRPDO1) 폐기
 * - SendESVector: SDO Download 0x6300:00 (6B, CAN-FD atomic 단일 프레임)
 * - SendMasterCommand: SDO Download 0x6310:00 (1B, 가상 ISI EXT 트리거)
 * - Pre-Op: TPDO1 Mapping + NMT START (RPDO mapping step 제거)
 * - TPDO Rx: Mutex + Snapshot (V2 Architecture) 유지
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "fes_hub_drv.h"
#include "agr_pnp_master.h"
#include "agr_pnp_identify.h"          /* IDENTIFY 상태머신 (identity 비교검증) */
#include "Core/agr_sdo_protocol.h"     /* AGR_SDO_CreateReadReq */
#include "xm_node_id.h"
#include "ioif_agrb_defs.h"
#include "ioif_agrb_tim.h"
#include <string.h>

#include "cmsis_os2.h"
#include "ioif_conf.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** @brief OD Index (FES Hub Slave side) */
#define FESHUB_OD_IDX_TPDO1_MAPPING     0x1A00
#define FESHUB_OD_IDX_ES_VECTOR         0x6300
#define FESHUB_OD_IDX_MASTER_COMMAND    0x6310

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/** @brief Pre-Op 상태 (Phase 1: IDENTIFY → Phase 2: CONFIG Step Array)
 *
 * [컨텍스트 규율]
 * IDENTIFY_PENDING 마킹은 Rx 컨텍스트(_PnP_OnBootup)에서 수행하지만,
 * 슬롯 획득 + AGR_Identify_Start(첫 SDO 송신)는 RunPeriodic(pnp_task
 * 단일 컨텍스트)에서만 수행한다 — pre_op_slot race 차단.
 * IDENTIFY 도중 Boot-up 재수신(Slave 리셋) 시 PENDING 재마킹 →
 * RunPeriodic이 Identify_Reset+Start로 시퀀스를 재시작한다 (고아 응답 방지). */
typedef enum {
    FESHUB_PRE_OP_IDLE = 0,
    FESHUB_PRE_OP_IDENTIFY_PENDING,   /**< Boot-up 마킹 (Rx 컨텍스트) — 슬롯 대기 */
    FESHUB_PRE_OP_IDENTIFY_RUNNING,   /**< 0x1000/0x1018 SDO Read 시퀀스 진행 */
    FESHUB_PRE_OP_IDENTITY_MISMATCH,  /**< 검증 실패 — NMT START 영구 보류 (fail loud) */
    FESHUB_PRE_OP_SEND_TPDO_MAP,
    FESHUB_PRE_OP_WAIT_TPDO_MAP,
    FESHUB_PRE_OP_SEND_NMT_START,
    FESHUB_PRE_OP_COMPLETE,
} FesHub_PreOpState_t;

typedef int (*FesHub_PreOpAction_t)(void);

typedef struct {
    FesHub_PreOpState_t send_state;
    FesHub_PreOpState_t wait_state;
    FesHub_PreOpAction_t action;
    uint32_t            timeout_ms;
    const char*         description;
} FesHub_PreOpStep_t;

/** @brief FES Hub 드라이버 인스턴스 */
typedef struct {
    AGR_DOP_Ctx_t       dop_ctx;
    AGR_TxFunc_t        tx_func;

    /* PnP Master (System Layer) */
    AGR_PnP_Master_t*   master_pnp;
    int                 slave_index;

    /* Pre-Op State Machine — pre_op_state/sdo_retry_count는 ISR(canfd_rx 경유
     * _PnP_OnBootup)이 쓰고 pnp_task(RunPeriodic)가 읽는 교차 컨텍스트 공유:
     * volatile 필수 (컴파일러 레지스터 캐싱으로 ISR 갱신 미관측 방지) */
    volatile FesHub_PreOpState_t pre_op_state;
    uint32_t            last_sdo_tx_time;
    volatile uint8_t    sdo_retry_count;

    /* IDENTIFY (Phase 1) */
    AGR_Identify_Ctx_t  identify;

    /* Rx Data (TPDO1: FES Hub -> XM) */
    volatile bool       is_data_ready;
} FesHub_DrvInst_t;

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static FesHub_DrvInst_t s_fes_hub_inst;

/**
 * @brief 진단 카운터 (디버거 Watch용) — 연결 불안정 원인 분리
 *
 * 2026-04-23 HW 관측: Extended 8B TPDO 전환 후 lastUpdateTick 갱신 + FSM 1→2
 * 확인됐으나 IsConnected true↔false 간헐 반복.
 *
 * [해석 매트릭스]
 *   disconnect_count ≈ reconnect_count ≈ bootup_count 동반 증가
 *     → H1 — Slave 리셋 반복 (HardFault / Watchdog / Stack 손상)
 *     → Slave 디버거로 reset reason 확인 필요
 *
 *   disconnect_count 증가, bootup_count 증가 안 함 (초기 1 유지)
 *     → H2 — Slave 는 살아있는데 HB 송신 지연 (5s timeout) 후 복구
 *     → FSM 전이 중 CPU 과부하 의심
 *
 *   last_disconnect_time, last_bootup_time 으로 간격 계산:
 *     (last_bootup_time - last_disconnect_time) < 100ms → 즉시 리셋 (H1 강력)
 *     간격 > 수초 → HB timeout 기반 (H2)
 */
static volatile uint32_t s_diag_disconnect_count   = 0;  /**< is_connected false 전이 횟수 */
static volatile uint32_t s_diag_reconnect_count    = 0;  /**< is_connected true 복귀 횟수 */
static volatile uint32_t s_diag_bootup_count       = 0;  /**< Slave Boot-up 수신 누적 (리셋 카운터) */
static volatile uint32_t s_diag_last_disconnect_ms = 0;  /**< 마지막 disconnect 시각 */
static volatile uint32_t s_diag_last_reconnect_ms  = 0;  /**< 마지막 reconnect 시각 */
static volatile uint32_t s_diag_last_bootup_ms     = 0;  /**< 마지막 Boot-up 수신 시각 */

/**
 * @brief NMT state 변화 진단 (H2 근본 원인 추적)
 * @details 2026-04-23 관측: bootup_count=1 고정 (Slave 리셋 없음), disconnect/reconnect
 *          반복 — H2 확정 (HB timeout 반복). Slave HB 누락 원인 분리 위해 state
 *          변화 패턴 기록.
 *
 * [해석]
 *   state_change_count 증가 주기 ≈ disconnect 주기 → HB 손실이 주 원인
 *   state_change_count 가 빠르게 증가 (수백 ms 주기) → PnP SM 내부 flapping
 *   min_disconnect_interval_ms 매우 짧음 (< 1000) → HB 복수 개 연속 누락
 *   min_disconnect_interval_ms 길면 (≈ 5000) → 주기적 timeout
 */
static volatile uint32_t s_diag_nmt_state_change_count = 0;  /**< NMT state 전환 누적 */
static volatile uint8_t  s_diag_nmt_last_old_state     = 0;  /**< 직전 NMT old state */
static volatile uint8_t  s_diag_nmt_last_new_state     = 0;  /**< 직전 NMT new state */
static volatile uint32_t s_diag_min_disconnect_interval_ms = 0xFFFFFFFFu; /**< disconnect 간 최소 간격 */
static volatile uint32_t s_diag_max_online_duration_ms     = 0;           /**< online 유지 최대 시간 */

/**
 * @brief SDO Write 진단 카운터 (Live Expression / 디버거 Watch 용)
 * @details
 * BTN1/BTN2 클릭 후 FES 에서 반응이 없을 때 "XM 이 송신을 실제로 했는가?
 * Slave 가 ACK 했는가? Slave 가 Abort 했는가?" 를 XM 단독으로 분리 진단하기 위함.
 *
 * 해석 매트릭스 (BTN 이벤트 직후 관찰):
 *   esv_tx=0, mcmd_tx=0           → XM 송신 자체 안 함 (상위 로직 조건 불만족)
 *   esv_tx≥1, last_ret=0          → FDCAN Tx FIFO enqueue 성공
 *   esv_tx≥1, last_ret<0          → FDCAN Tx 실패 (Bus Off / FIFO Full / HAL)
 *   sdo_ack_count 매칭             → Slave 가 ACK(0x60) 반환 — 명령 수신됨
 *   sdo_abort_count≥1 + last_code → Slave 가 Abort 반환 — Abort Code 로 원인 식별
 *   Tx 성공 + ACK 오지만 반응 없음 → Slave OD→FSM 전이 경로 문제 (XM 측 책임 아님)
 *
 * 주요 Abort Code (4B LE, CiA 301):
 *   0x06070010  TYPE_MISMATCH   — data type 불일치
 *   0x06070012  DATA_LONG       — Master 가 기대보다 길게 보냄
 *   0x06070013  DATA_SHORT      — Master 가 기대보다 짧게 보냄
 *   0x06040041  NO_MAP          — 해당 object 는 PDO 매핑 불가
 *   0x06020000  NOT_EXIST       — OD 에 해당 index 없음 (0x6300/0x6310 미정의?)
 *   0x06010002  READ_ONLY       — 쓰기 금지 object
 *   0x06010001  WRITE_ONLY      — 읽기 금지 object
 */
/* Master → Slave SDO Write 송신 카운터 */
static volatile uint32_t s_diag_esv_tx_count    = 0;  /**< ES-vector (0x6300) 송신 누적 */
static volatile int32_t  s_diag_esv_last_ret    = 0;  /**< 최근 ES-vector Tx 반환값 */
static volatile uint32_t s_diag_mcmd_tx_count   = 0;  /**< Master Cmd (0x6310) 송신 누적 */
static volatile int32_t  s_diag_mcmd_last_ret   = 0;  /**< 최근 Master Cmd Tx 반환값 */
static volatile uint8_t  s_diag_mcmd_last_value = 0;  /**< 최근 송신한 Master Cmd 값 */

/* Slave → Master SDO Response 수신 카운터 */
static volatile uint32_t s_diag_sdo_ack_count       = 0;  /**< Download ACK(0x60) 수신 누적 */
static volatile uint16_t s_diag_sdo_last_ack_index  = 0;  /**< 최근 ACK OD index */
static volatile uint32_t s_diag_sdo_abort_count     = 0;  /**< Abort(0x80) 수신 누적 */
static volatile uint32_t s_diag_sdo_last_abort_code = 0;  /**< 최근 Abort 4B LE */
static volatile uint16_t s_diag_sdo_last_abort_index= 0;  /**< 최근 Abort OD index */
static volatile uint8_t  s_diag_sdo_last_abort_subidx = 0;/**< 최근 Abort OD subindex */

/* ===== Command Vector (0x10C Req / 0x68C ACK — fes_hub_drv.h 계약) =====
 * 단일 outstanding: Send(앱 단일 컨텍스트 가정)가 frame 작성 후 pending=true,
 * ACK 매칭/타임아웃 재전송(동일 seq — Slave 멱등)/포기는 RunPeriodic(pnp_task).
 * ACK 주입은 Rx task(OnVectorAck) — volatile + flag 마지막 프로토콜. */
/* 타임아웃 검사 해상도 = RunPeriodic 호출 주기(pnp_task, TASK_PERIOD_MS_PNP_MANAGER
 * =100ms) — 그보다 작은 값은 의미 없으므로 격자에 맞춤. 실동작: 재전송 간격 ~100ms,
 * 3회 실패 포기까지 ~400ms, pending 해제도 동일 격자(연속 명령 최대 ~10Hz). */
#define FESHUB_VEC_ACK_TIMEOUT_MS  100u
#define FESHUB_VEC_MAX_RETRY       3u

static volatile bool     s_vec_pending   = false;
static uint8_t           s_vec_seq       = 0;
static uint8_t           s_vec_frame[3u + sizeof(FesHub_ESVector_t)];
static uint32_t          s_vec_sent_ms   = 0;
static uint8_t           s_vec_retry     = 0;
static volatile uint8_t  s_vec_ack_flag  = 0;   /**< Rx task: 마지막에 1 / pnp_task: 소비 후 0 */
static volatile uint8_t  s_vec_ack_seq   = 0;
static volatile uint8_t  s_vec_ack_status = 0;

static volatile uint32_t s_diag_vec_tx_count       = 0;  /**< Vector 송신(첫 전송) 수 */
static volatile uint32_t s_diag_vec_ack_ok         = 0;  /**< ACK status=0 */
static volatile uint32_t s_diag_vec_ack_err        = 0;  /**< ACK status!=0 (last_status 참조) */
static volatile uint32_t s_diag_vec_fail           = 0;  /**< 재시도 한도 초과 포기 */
static volatile uint32_t s_diag_vec_retry_count    = 0;  /**< 재전송 횟수 누적 */
static volatile uint32_t s_diag_vec_unexpected_ack = 0;  /**< pending 없음/seq 불일치 ACK */
static volatile uint8_t  s_diag_vec_last_status    = 0xFF; /**< 0xFF=미수신, 0xFE=timeout 포기 */

/**
 * @brief Pre-Op SM / NMT Start / TPDO Rx 진단 카운터
 * @details
 * 2026-04-23 관측 — IsConnected=true, ack_count 증가, lastUpdateTick=0 (TPDO 미수신)
 * 상황을 분리 진단하기 위해 추가. EMG 31bb3e6 패턴 + TPDO Rx 관측.
 *
 * 핵심 해석:
 *   pre_op_state_max < FESHUB_PRE_OP_COMPLETE → Pre-Op SM 이 COMPLETE 도달 못함
 *                                   (IDENTIFY 단계 추가로 절대값 비교 금지 — enum 참조)
 *   nmt_start_tx_count=0          → XM 이 NMT Start 송신 자체 안 함 (Pre-Op SM 멈춤)
 *   nmt_start_tx_count≥1, ret=0   → XM 송신 OK. 이후 Slave OPERATIONAL 전이 + TPDO 여부
 *   tpdo_rx_count=0               → 0x180+Node 프레임 FDCAN Rx 없음 (Slave 미송신)
 *   tpdo_rx_count≥1, rejected_short≥1, last_len<37 → Slave 가 구 포맷(16B) TPDO 송신 중
 *                                                    → Slave 측 TPDO payload 확장 필요
 *   tpdo_rx_count≥1, rejected_short=0, last_len≥37→ 정상 수신 — GetRxData 경로 점검
 */
static volatile uint32_t s_diag_nmt_start_tx_count  = 0;  /**< _Step_SendNmtStart 호출 누적 */
static volatile int32_t  s_diag_nmt_start_last_ret  = 0;  /**< 마지막 NMT Start Tx 반환값 */
static volatile uint32_t s_diag_nmt_start_last_time = 0;  /**< 마지막 NMT Start Tx 시각 (ms) */
static volatile uint8_t  s_diag_pre_op_state_now    = 0;  /**< 현재 Pre-Op SM state 값 */
static volatile uint8_t  s_diag_pre_op_state_max    = 0;  /**< Pre-Op SM 이 도달한 최대 state */

/* TPDO Rx 관측 — 0x180+Node 프레임 수신 경로 */
static volatile uint32_t s_diag_tpdo_rx_count          = 0;  /**< 0x180+Node 프레임 수신 누적 (len 체크 이전) */
static volatile uint32_t s_diag_tpdo_rx_rejected_short = 0;  /**< len < sizeof(FesHub_TpdoRaw_t) 로 drop */
static volatile uint32_t s_diag_tpdo_rx_accepted       = 0;  /**< Snapshot 갱신 성공 */
static volatile uint8_t  s_diag_tpdo_rx_last_len       = 0;  /**< 최근 수신 프레임 길이 */

/**
 * @brief IDENTIFY 진단 (Live Expression) — 시나리오 B(MISMATCH 유도) 관찰점
 */
static volatile uint8_t  s_diag_identify_result       = 0;  /**< AGR_Identify_Result_t 현재값 */
static volatile uint8_t  s_diag_identify_failed_step  = 0;  /**< 불일치 필드 (0=devtype..3=revision) */
static volatile uint32_t s_diag_identify_expected     = 0;  /**< 불일치 기대값 */
static volatile uint32_t s_diag_identify_actual       = 0;  /**< 불일치 실측값 */
static volatile uint32_t s_diag_identity_mismatch_cnt = 0;  /**< MISMATCH 진입 누적 */

/**
 * @brief FES 기대 identity — Slave 매뉴얼(fes_od_table.h) 기반 하드코딩.
 * @details revision = 와이어 계약 버전. FES OD/와이어 계약 변경 시 Slave가
 *   bump하며, 본 기대값을 함께 갱신해야 IDENTIFY가 통과한다 (drift 검출 의도).
 *   0x00030000 = OD 단일 테이블화 (2026-06-11).
 */
static const AGR_Identify_Expected_t s_fes_expected_identity = {
    .device_type  = 0x000B0000u,   /* AGR_DEVICE_TYPE(0x000B, 0) — Rule 33 §0 */
    .vendor_id    = 0x414E474Cu,   /* "ANGL" */
    .product_code = 0x00040003u,   /* Rule 22 §2.2 */
    .revision     = 0x00040000u,   /* Command Vector 채널 추가 (0x10C/0x68C, 2026-06-12) */
};

/* Forward declarations */
static int _Step_SendTpdoMap(void);
static int _Step_SendNmtStart(void);
static int _FesIdentify_SendRead(void* user, uint16_t idx, uint8_t sub);
static void _HandleIdentifyResult(AGR_Identify_Result_t r);

/** @brief Pre-Op Step Array (V2.0: RPDO mapping step 제거) */
static const FesHub_PreOpStep_t s_pre_op_steps[] = {
    { FESHUB_PRE_OP_SEND_TPDO_MAP,  FESHUB_PRE_OP_WAIT_TPDO_MAP,  _Step_SendTpdoMap,   5000, "TPDO1 Mapping" },
    { FESHUB_PRE_OP_SEND_NMT_START, FESHUB_PRE_OP_COMPLETE,        _Step_SendNmtStart,  5000, "NMT START" },
};

#define PRE_OP_STEP_COUNT (sizeof(s_pre_op_steps) / sizeof(s_pre_op_steps[0]))

/**
 * @brief TPDO1 Mapping (FES Feedback -> XM) — Extended 8B entry format
 *
 * TPDO1 (0x18C, 37B wire): 0x7018.0x00 FES Feedback BLOB (Legacy 16B + KHJ 21B)
 *
 * [왜 Extended 8B 인가]
 * CiA 301 Legacy 4B entry 는 bit_length 필드가 uint8_t (최대 255 bits = 31.875 B).
 * FES TPDO payload 37B (= 296 bits) 는 Legacy 4B entry 로 표현 불가능.
 * AGR_MW (agr_pdo_engine.c:114-122) 는 이를 위해 Extended 8B entry 를 제공:
 *   Header byte bit7 = Extended flag (1), bit0..6 = count (max 127)
 *   Entry 8B       = Index(2 LE) + SubIndex(1) + Flags(1) + Length(4 LE uint32)
 *
 * [Wire layout — 9 bytes total]
 *   byte[0]   = 0x81    → count=1, Extended flag (bit7)
 *   byte[1-2] = 0x18 0x70 → Index 0x7018 (LE)
 *   byte[3]   = 0x00    → SubIndex
 *   byte[4]   = 0x00    → Flags (reserved)
 *   byte[5-8] = 0x25 0x00 0x00 0x00 → Length = 37 B (LE uint32, 0x25 = 37)
 *
 * [AS-WAS 버그 히스토리 2026-04-23]
 * 초기 구현: Legacy 4B entry 에 bit_length=0x80 (128 bits = 16B) 기재.
 * AGR_PDO_Encode 는 map 의 length_bytes 값을 우선 적용하여 wire 에 16B 만 인코딩.
 * XM 측 FesHub_Drv_ProcessCANMessage 는 len<sizeof(FesHub_TpdoRaw_t)=37 drop
 * → is_data_ready=false 유지 → lastUpdateTick=0 고정 (TPDO 전량 손실).
 * HW 실측(BTN 클릭 시 esv_tx/mcmd_tx/ack_count 정상 증가 + lastUpdateTick=0)
 * 으로 확정 후 Extended 8B format 으로 정식 전환. bit_length 8-bit 한계
 * 정면 회피 + CiA 301 표준 + AGR 확장 준수.
 *
 * [FES Slave 영향] 없음. xm_drv.c:_OnTpdoMapping_Set 이 AGR_PDO_ApplyMapFromSDO
 * 호출 → agr_pdo_engine.c 가 header bit7=1 감지 → Extended 8B 분기 자동 파싱.
 * FES AGR_MW 도 f8bad0f 동일 버전 (동일 엔진), 코드 수정 불필요.
 */
static const uint8_t s_tpdo_map[] = {
    /* Header: count=1 | Extended flag (bit7) */
    0x81,
    /* Entry 1: Index=0x7018 LE */
    0x18, 0x70,
    /* SubIndex / Flags */
    0x00, 0x00,
    /* Length = 37 bytes (LE uint32) */
    0x25, 0x00, 0x00, 0x00,
};

/** @brief TPDO1 Raw 페이로드 (디코딩 중간 구조, 37B — FES Hub xm_drv.h와 동일) */
typedef struct __attribute__((packed)) {
    /* Legacy 16B */
    uint8_t  ch_state[FESHUB_CH_COUNT];
    int16_t  ch_current_x10[FESHUB_CH_COUNT];
    uint8_t  ch_fault_code[FESHUB_CH_COUNT];
    uint16_t hv_voltage_x100;
    uint8_t  digipot_pos;
    uint8_t  es_state_packed;       /**< [3:0]=CH0 ESState, [7:4]=CH1 */
    uint8_t  timestamp_low;
    uint8_t  timestamp_mid;
    uint8_t  timestamp_high;
    uint8_t  error_register;
    /* KHJ telemetry extension (21B) */
    uint8_t  fsm_state;
    uint8_t  fsm_state_prev;
    uint8_t  isi_packed;
    uint8_t  ch_es_error_lo[FESHUB_CH_COUNT];
    int16_t  ch_target_amp_x10[FESHUB_CH_COUNT];
    int16_t  ch_impedance_x10[FESHUB_CH_COUNT];
    uint16_t ch_pulse_cnt[FESHUB_CH_COUNT];
    int16_t  ch_voltage_diff_x100[FESHUB_CH_COUNT];
} FesHub_TpdoRaw_t;  /* 37 bytes */

/** @brief PDO 공유 데이터 + Mutex (V2 Architecture) */
static osMutexId_t      s_pdo_mutex = NULL;
static FesHub_RxData_t  s_pdo_shared = {0};

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* PnP Callbacks */
static void _PnP_OnBootup(uint8_t slave_node_id);
static void _PnP_OnStateChanged(uint8_t slave_node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
static void _PnP_OnOnline(uint8_t slave_node_id);
static void _PnP_OnOffline(uint8_t slave_node_id);

/* SDO Response */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response);

/* Pre-Op State Machine */
static void _RunPreOpStateMachine(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

int FesHub_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Master_t* master_pnp)
{
    if (tx_func == NULL || master_pnp == NULL) {
        return -1;
    }

    memset(&s_fes_hub_inst, 0, sizeof(FesHub_DrvInst_t));
    memset(&s_pdo_shared, 0, sizeof(FesHub_RxData_t));

    if (s_pdo_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "FesPdoMtx",
            .attr_bits = osMutexPrioInherit,
        };
        s_pdo_mutex = osMutexNew(&mutex_attr);
    }

    s_fes_hub_inst.tx_func = tx_func;
    s_fes_hub_inst.master_pnp = master_pnp;
    s_fes_hub_inst.slave_index = -1;

    /* DOP 초기화 (Master: OD 불필요) */
    AGR_CANFD_Init(&s_fes_hub_inst.dop_ctx,
                   NULL,
                   AGR_NODE_ID_XM,
                   tx_func);
    AGR_CANFD_SetTargetNodeId(&s_fes_hub_inst.dop_ctx, AGR_NODE_ID_FES_HUB);

    /* PnP Master에 Slave 등록 */
    AGR_PnP_SlaveConfig_t slave_config = {
        .name = "FES Hub",
        .node_id = AGR_NODE_ID_FES_HUB,
        .heartbeat_timeout_ms = 10000,   /* 2026-04-23 H2 완화책: 5s→10s. Slave HB 1s 주기 가정 시
                                          * 9 개 연속 누락 허용. disconnect/reconnect 반복 증상 완화.
                                          * 근본 원인은 Slave HB 누락 또는 FSM 전이 중 CPU 과부하. */
        .callbacks = {
            .on_slave_bootup        = _PnP_OnBootup,
            .on_slave_state_changed = _PnP_OnStateChanged,
            .on_slave_online        = _PnP_OnOnline,
            .on_slave_offline       = _PnP_OnOffline,
        },
        .expected_identity    = &s_fes_expected_identity,
        .identity_check_flags = AGR_IDENTIFY_FLAG_ALL,
    };

    int idx = AGR_PnP_Master_AddSlave(master_pnp, &slave_config);
    if (idx < 0) {
        return -1;
    }
    s_fes_hub_inst.slave_index = idx;

    /* IDENTIFY 컴포넌트 — STRICT: 불일치 시 NMT START 보류 (fail loud) */
    AGR_Identify_Init(&s_fes_hub_inst.identify,
                      _FesIdentify_SendRead, NULL,
                      &s_fes_expected_identity,
                      AGR_IDENTIFY_FLAG_ALL,
                      AGR_IDENTIFY_MODE_STRICT);

    return 0;
}

bool FesHub_Drv_IsIdentityMismatch(void)
{
    return (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTITY_MISMATCH);
}

void FesHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    uint16_t fnc_code = can_id & 0x780;

    /* 1. SDO Response (0x580 + Node ID) */
    if (fnc_code == 0x580) {
        AGR_SDO_Msg_t sdo_msg;
        if (AGR_SDO_Decode(data, len, &sdo_msg) == 0) {
            _OnSdoResponse(&sdo_msg);
        }
        return;
    }

    /* 2. TPDO1 (0x180 + Node ID = 0x18C) — FES Feedback, 37B — Mutex + Snapshot */
    if (fnc_code == 0x180) {
        /* 진단: TPDO Rx 계측 (H2 가설 증거 수집 — Slave 가 구 포맷 16B 송신 중인지).
         * len 체크 이전 카운트 + last_len 기록으로 "수신은 되는데 크기 부족" 판정. */
        s_diag_tpdo_rx_count++;
        s_diag_tpdo_rx_last_len = len;

        if (len < sizeof(FesHub_TpdoRaw_t)) {
            s_diag_tpdo_rx_rejected_short++;
            return;
        }

        {
            const FesHub_TpdoRaw_t* raw = (const FesHub_TpdoRaw_t*)data;
        if (osMutexAcquire(s_pdo_mutex, 1) == osOK) {
            /* Legacy 16B */
            for (int i = 0; i < FESHUB_CH_COUNT; i++) {
                s_pdo_shared.ch_state[i] = (FesHub_ChState_t)raw->ch_state[i];
                s_pdo_shared.ch_current_mA[i] = (float)raw->ch_current_x10[i] / FESHUB_SCALE_AMPLITUDE;
                s_pdo_shared.ch_fault_code[i] = raw->ch_fault_code[i];
            }
            s_pdo_shared.hv_voltage_V = (float)raw->hv_voltage_x100 / FESHUB_SCALE_VOLTAGE;
            s_pdo_shared.digipot_pos = raw->digipot_pos;
            s_pdo_shared.es_state_packed = raw->es_state_packed;
            s_pdo_shared.timestamp = (uint32_t)raw->timestamp_low |
                                     ((uint32_t)raw->timestamp_mid << 8) |
                                     ((uint32_t)raw->timestamp_high << 16);
            s_pdo_shared.error_register = raw->error_register;

            /* KHJ telemetry extension */
            s_pdo_shared.fsm_state      = raw->fsm_state;
            s_pdo_shared.fsm_state_prev = raw->fsm_state_prev;
            s_pdo_shared.isi_packed      = raw->isi_packed;
            for (int i = 0; i < FESHUB_CH_COUNT; i++) {
                s_pdo_shared.ch_es_error_lo[i]        = raw->ch_es_error_lo[i];
                s_pdo_shared.ch_target_amplitude_mA[i] = (float)raw->ch_target_amp_x10[i] / FESHUB_SCALE_AMPLITUDE;
                s_pdo_shared.ch_impedance[i]           = (float)raw->ch_impedance_x10[i] / FESHUB_SCALE_IMPEDANCE;
                s_pdo_shared.ch_pulse_cnt[i]           = raw->ch_pulse_cnt[i];
                s_pdo_shared.ch_voltage_diff_V[i]      = (float)raw->ch_voltage_diff_x100[i] / FESHUB_SCALE_VOLTAGE;
            }

            osMutexRelease(s_pdo_mutex);
            s_fes_hub_inst.is_data_ready = true;
            s_diag_tpdo_rx_accepted++;
        }
        }
        return;
    }
}

bool FesHub_Drv_GetRxData(FesHub_RxData_t* rx_data)
{
    if (rx_data == NULL) {
        return false;
    }

    if (!s_fes_hub_inst.is_data_ready) {
        return false;
    }

    if (osMutexAcquire(s_pdo_mutex, 0) == osOK) {
        memcpy(rx_data, &s_pdo_shared, sizeof(FesHub_RxData_t));
        osMutexRelease(s_pdo_mutex);
        return true;
    }
    return false;
}

bool FesHub_Drv_IsDataReady(void)
{
    return s_fes_hub_inst.is_data_ready;
}

bool FesHub_Drv_IsConnected(void)
{
    return AGR_PnP_Master_IsSlaveOnline(s_fes_hub_inst.master_pnp,
                                         AGR_NODE_ID_FES_HUB);
}

AGR_NMT_State_t FesHub_Drv_GetNmtState(void)
{
    return AGR_PnP_Master_GetSlaveState(s_fes_hub_inst.master_pnp,
                                         AGR_NODE_ID_FES_HUB);
}

/**
 * @brief ES-vector 전송 (SDO Download 0x6300:00, 6B atomic)
 */
int FesHub_Drv_SendESVector(const FesHub_ESVector_t* esv)
{
    if (esv == NULL || s_fes_hub_inst.tx_func == NULL) {
        return -1;
    }

    int ret = AGR_CANFD_SendSDOWrite(&s_fes_hub_inst.dop_ctx,
                                     FESHUB_OD_IDX_ES_VECTOR, 0x00,
                                     esv, sizeof(FesHub_ESVector_t));
    /* 진단: ES-vector Tx 증적 (Live Expression).
     * ret=0 이면 IOIF Tx Queue enqueue 성공 (HW Tx FIFO 는 별도).
     * ret<0 이면 이미 Tx 단계 실패 — Slave 응답 이전 문제. */
    s_diag_esv_tx_count++;
    s_diag_esv_last_ret = ret;
    return ret;
}

int FesHub_Drv_SendESVectorCmd(const FesHub_ESVector_t* esv)
{
    if (esv == NULL || s_fes_hub_inst.tx_func == NULL) {
        return -2;
    }
    if (s_vec_pending) {
        return -1;   /* 단일 outstanding — 이전 vector ACK 대기 중 */
    }

    s_vec_seq++;   /* u8 순환 — Slave는 직전 seq 와의 동일성만 검사 (멱등) */
    s_vec_frame[0] = (uint8_t)FESHUB_VEC_TYPE_ES;
    s_vec_frame[1] = s_vec_seq;
    s_vec_frame[2] = (uint8_t)sizeof(FesHub_ESVector_t);
    memcpy(&s_vec_frame[3], esv, sizeof(FesHub_ESVector_t));

    s_vec_retry    = 0;
    s_vec_ack_flag = 0;
    s_vec_sent_ms  = IOIF_TIM_GetTick();

    int ret = s_fes_hub_inst.tx_func(FESHUB_VEC_REQ_FNC_BASE + AGR_NODE_ID_FES_HUB,
                                     s_vec_frame, (uint8_t)sizeof(s_vec_frame));
    if (ret < 0) {
        return -2;
    }
    s_vec_pending = true;   /* frame 작성 완료 후 마지막 — RunPeriodic 가시화 */
    s_diag_vec_tx_count++;
    return 0;
}

void FesHub_Drv_OnVectorAck(const uint8_t* data, uint8_t len)
{
    if (data == NULL || len < 2u) {
        return;
    }
    if (!s_vec_pending) {
        s_diag_vec_unexpected_ack++;   /* 고아 ACK (재시도 후 늦은 응답 등) */
        return;
    }
    s_vec_ack_seq    = data[0];
    s_vec_ack_status = data[1];
    s_vec_ack_flag   = 1;   /* 항상 마지막 — RunPeriodic(pnp_task)이 소비 */
}

/**
 * @brief Master Command 전송 (SDO Download 0x6310:00, 1B)
 */
int FesHub_Drv_SendMasterCommand(FesHub_MasterCmd_t cmd)
{
    if (s_fes_hub_inst.tx_func == NULL) {
        return -1;
    }

    uint8_t cmd_byte = (uint8_t)cmd;
    int ret = AGR_CANFD_SendSDOWrite(&s_fes_hub_inst.dop_ctx,
                                     FESHUB_OD_IDX_MASTER_COMMAND, 0x00,
                                     &cmd_byte, 1);
    /* 진단: Master Command Tx 증적. last_value 로 enum mismatch 판별 (XM=1 vs Slave=?). */
    s_diag_mcmd_tx_count++;
    s_diag_mcmd_last_ret = ret;
    s_diag_mcmd_last_value = cmd_byte;
    return ret;
}

void FesHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();

    /* 진단: Pre-Op state 폴링 (EMG 동일 패턴, Live Expression 용). */
    s_diag_pre_op_state_now = (uint8_t)s_fes_hub_inst.pre_op_state;
    if (s_diag_pre_op_state_now > s_diag_pre_op_state_max) {
        s_diag_pre_op_state_max = s_diag_pre_op_state_now;
    }

    /* 0-b. Command Vector ACK 소비 / 타임아웃 재전송 (단일 outstanding) */
    if (s_vec_pending) {
        if (s_vec_ack_flag == 1u) {
            /* flag clear 전 스냅샷 — clear~read 사이 ISR 재주입 시 seq/status
             * 가 서로 다른 ACK 에서 섞이는(mixed-pair) 창 제거 */
            uint8_t ack_seq    = s_vec_ack_seq;
            uint8_t ack_status = s_vec_ack_status;
            s_vec_ack_flag = 0;
            if (ack_seq == s_vec_frame[1]) {
                s_diag_vec_last_status = ack_status;
                if (ack_status == (uint8_t)FESHUB_VEC_ACK_OK) {
                    s_diag_vec_ack_ok++;
                } else {
                    s_diag_vec_ack_err++;
                }
                s_vec_pending = false;
            } else {
                s_diag_vec_unexpected_ack++;   /* seq 불일치 고아 ACK — 무시, 대기 지속 */
            }
        } else if (current_ms - s_vec_sent_ms >= FESHUB_VEC_ACK_TIMEOUT_MS) {
            if (s_vec_retry < FESHUB_VEC_MAX_RETRY) {
                s_vec_retry++;
                s_diag_vec_retry_count++;
                s_vec_sent_ms = current_ms;
                /* 동일 seq 재전송 — Slave 멱등(중복 자극 방지), ACK 유실 시 재-ACK */
                (void)s_fes_hub_inst.tx_func(
                    FESHUB_VEC_REQ_FNC_BASE + AGR_NODE_ID_FES_HUB,
                    s_vec_frame, (uint8_t)sizeof(s_vec_frame));
            } else {
                s_diag_vec_fail++;
                s_diag_vec_last_status = 0xFE;   /* timeout 포기 표식 */
                s_vec_pending = false;
            }
        }
    }

    /* 0. IDENTIFY Phase 구동 (pnp_task 단일 컨텍스트 — 슬롯 계약) */
    if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_PENDING) {
        if (AGR_PnP_Master_TryAcquirePreOpSlot(s_fes_hub_inst.master_pnp,
                                               AGR_NODE_ID_FES_HUB)) {
            AGR_Identify_Reset(&s_fes_hub_inst.identify);
            AGR_Identify_Result_t r =
                AGR_Identify_Start(&s_fes_hub_inst.identify, current_ms);
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDENTIFY_RUNNING;
            _HandleIdentifyResult(r);   /* flags=0이면 즉시 PASS 가능 */
        }
        /* 슬롯 점유 중(타 Slave Pre-Op) — 다음 주기 재시도 */
    } else if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_RUNNING) {
        /* 타임아웃/재시도 + NonRealtime task가 주입한 terminal 결과 수거 */
        (void)AGR_Identify_RunPeriodic(&s_fes_hub_inst.identify, current_ms);
        _HandleIdentifyResult(AGR_Identify_GetResult(&s_fes_hub_inst.identify));
    }
    s_diag_identify_result = (uint8_t)AGR_Identify_GetResult(&s_fes_hub_inst.identify);

    /* 1. Pre-Op Timeout 체크 (SDO ACK 대기) */
    if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_WAIT_TPDO_MAP) {
        if (current_ms - s_fes_hub_inst.last_sdo_tx_time > 5000) {
            if (s_fes_hub_inst.sdo_retry_count < 3) {
                s_fes_hub_inst.sdo_retry_count++;
                s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_SEND_TPDO_MAP;
            } else {
                s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDLE;
                s_fes_hub_inst.sdo_retry_count = 0;
            }
        }
    }

    /* 2. Pre-Op State Machine */
    _RunPreOpStateMachine();
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — PnP Callbacks
 *-----------------------------------------------------------
 */

static void _PnP_OnBootup(uint8_t slave_node_id)
{
    (void)slave_node_id;

    /* 진단: Slave Boot-up 수신 증적 (H1 Slave 리셋 반복 판정용).
     * Pre-Op SM 진입 이전에 증적 — bootup_count 가 disconnect_count 비율로
     * 증가 하면 Slave 리셋, 1 고정이면 HB timeout 복구. */
    s_diag_bootup_count++;
    s_diag_last_bootup_ms = IOIF_TIM_GetTick();

    /* Rx 컨텍스트 — 마킹만. 슬롯 획득 + Identify_Start는 RunPeriodic(pnp_task).
     * IDENTIFY 도중 재수신(Slave 리셋)도 PENDING 재마킹 → 시퀀스 재시작. */
    switch (s_fes_hub_inst.pre_op_state) {
        case FESHUB_PRE_OP_IDLE:
        case FESHUB_PRE_OP_IDENTITY_MISMATCH:   /* 재부팅 = 재검증 기회 */
        case FESHUB_PRE_OP_IDENTIFY_PENDING:
        case FESHUB_PRE_OP_IDENTIFY_RUNNING:
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDENTIFY_PENDING;
            s_fes_hub_inst.sdo_retry_count = 0;
            break;
        default:
            /* CONFIG phase(SEND/WAIT_TPDO_MAP, NMT_START) 또는 COMPLETE 중
             * Boot-up = Slave 재부팅 — 즉시 재-IDENTIFY (마킹만, 슬롯 ops 없음.
             * CONFIG 중이면 슬롯 보유 상태 → TryAcquire 재진입 허용으로 계속,
             * COMPLETE면 해제 상태 → RunPeriodic이 재획득).
             * 이전: 무동작 → HB timeout(10s)까지 Pre-Op 재시작 지연
             * (벤치 D 오진 위험 — 사전 검증 MEDIUM 발견) */
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDENTIFY_PENDING;
            s_fes_hub_inst.sdo_retry_count = 0;
            break;
    }
}

static void _PnP_OnStateChanged(uint8_t slave_node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state)
{
    (void)slave_node_id;

    /* 진단: NMT state 전환 증적 (H2 flapping 패턴 관찰용) */
    s_diag_nmt_state_change_count++;
    s_diag_nmt_last_old_state = (uint8_t)old_state;
    s_diag_nmt_last_new_state = (uint8_t)new_state;
}

static void _PnP_OnOnline(uint8_t slave_node_id)
{
    (void)slave_node_id;
    s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDLE;
    s_fes_hub_inst.sdo_retry_count = 0;
    s_diag_reconnect_count++;
    s_diag_last_reconnect_ms = IOIF_TIM_GetTick();
}

static void _PnP_OnOffline(uint8_t slave_node_id)
{
    (void)slave_node_id;
    s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDLE;
    s_fes_hub_inst.sdo_retry_count = 0;
    s_fes_hub_inst.is_data_ready = false;

    /* IDENTIFY 정리 + 슬롯 반환 (Offline은 pnp_task의 RunPeriodic 경유 —
     * 슬롯 계약 컨텍스트 내) */
    AGR_Identify_Reset(&s_fes_hub_inst.identify);
    AGR_PnP_Master_ReleasePreOpSlot(s_fes_hub_inst.master_pnp, AGR_NODE_ID_FES_HUB);

    /* 진단: 이번 offline 이전에 얼마나 online 유지됐는지 측정.
     * last_reconnect_ms ~ 현재 tick 차이 = 이번 online 세션 길이.
     *   → 짧으면(<1s) flapping, 길면(>5s) 주기적 timeout. */
    uint32_t now = IOIF_TIM_GetTick();
    if (s_diag_last_reconnect_ms > 0 && now > s_diag_last_reconnect_ms) {
        uint32_t online_duration = now - s_diag_last_reconnect_ms;
        if (online_duration > s_diag_max_online_duration_ms) {
            s_diag_max_online_duration_ms = online_duration;
        }
    }
    /* 이전 disconnect 와의 간격 (reconnect → 이번 disconnect 구간과 다름. 순수 disconnect 사이).
     * last_disconnect_ms 가 0 이 아니면 직전 disconnect 이후 경과 시간. */
    if (s_diag_last_disconnect_ms > 0 && now > s_diag_last_disconnect_ms) {
        uint32_t interval = now - s_diag_last_disconnect_ms;
        if (interval < s_diag_min_disconnect_interval_ms) {
            s_diag_min_disconnect_interval_ms = interval;
        }
    }

    s_diag_disconnect_count++;
    s_diag_last_disconnect_ms = now;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — Pre-Op State Machine
 *-----------------------------------------------------------
 */

static void _RunPreOpStateMachine(void)
{
    if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDLE) {
        /* Boot-up을 놓친 경우: Slave가 이미 PRE_OP이면 IDENTIFY부터 진입 */
        AGR_NMT_State_t slave_state = AGR_PnP_Master_GetSlaveState(
            s_fes_hub_inst.master_pnp, AGR_NODE_ID_FES_HUB);
        if (slave_state == AGR_NMT_PRE_OPERATIONAL) {
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDENTIFY_PENDING;
            s_fes_hub_inst.sdo_retry_count = 0;
        }
        return;
    }

    /* IDENTIFY/MISMATCH/COMPLETE는 Step Array 비대상 —
     * NMT START는 IDENTIFY PASS 후에만 도달 가능 */
    if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_PENDING ||
        s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_RUNNING ||
        s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTITY_MISMATCH ||
        s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_COMPLETE) {
        return;
    }

    /* Step Array 기반 실행 */
    for (uint8_t i = 0; i < PRE_OP_STEP_COUNT; i++) {
        const FesHub_PreOpStep_t* step = &s_pre_op_steps[i];

        if (s_fes_hub_inst.pre_op_state == step->send_state) {
            int result = step->action();
            if (result >= 0) {
                s_fes_hub_inst.pre_op_state = step->wait_state;
                s_fes_hub_inst.last_sdo_tx_time = IOIF_TIM_GetTick();
                s_fes_hub_inst.sdo_retry_count = 0;

                /* Pre-Op 완료 — 직렬화 슬롯 반환 (타 Slave 진입 허용) */
                if (step->wait_state == FESHUB_PRE_OP_COMPLETE) {
                    AGR_PnP_Master_ReleasePreOpSlot(s_fes_hub_inst.master_pnp,
                                                    AGR_NODE_ID_FES_HUB);
                }
            }
            return;
        }

        if (s_fes_hub_inst.pre_op_state == step->wait_state) {
            return;
        }
    }
}

/**
 * @brief IDENTIFY 결과 처리 — PASS→CONFIG 진행 / MISMATCH→NMT START 보류
 * @note pnp_task 컨텍스트 전용 (RunPeriodic 경유)
 */
static void _HandleIdentifyResult(AGR_Identify_Result_t r)
{
    switch (r) {
        case AGR_IDENTIFY_RESULT_PASS:
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_SEND_TPDO_MAP;
            s_fes_hub_inst.sdo_retry_count = 0;
            break;

        case AGR_IDENTIFY_RESULT_MISMATCH:
            /* fail loud: NMT START 영구 보류 (Boot-up 재수신 시에만 재검증).
             * 슬롯은 반환 — 타 Slave Pre-Op 진입 허용. */
            s_diag_identity_mismatch_cnt++;
            s_diag_identify_failed_step = (uint8_t)s_fes_hub_inst.identify.last_mismatch.failed_step;
            s_diag_identify_expected    = s_fes_hub_inst.identify.last_mismatch.expected;
            s_diag_identify_actual      = s_fes_hub_inst.identify.last_mismatch.actual;
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDENTITY_MISMATCH;
            AGR_PnP_Master_ReleasePreOpSlot(s_fes_hub_inst.master_pnp,
                                            AGR_NODE_ID_FES_HUB);
            break;

        case AGR_IDENTIFY_RESULT_TIMEOUT:
        case AGR_IDENTIFY_RESULT_ABORT:
            /* Slave 무응답/거부 — IDLE 복귀 (Boot-up/PRE_OP 감지로 재시도) */
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_IDLE;
            s_fes_hub_inst.sdo_retry_count = 0;
            AGR_PnP_Master_ReleasePreOpSlot(s_fes_hub_inst.master_pnp,
                                            AGR_NODE_ID_FES_HUB);
            break;

        default:
            break;  /* IDLE / IN_PROGRESS — 대기 */
    }
}

static int _FesIdentify_SendRead(void* user, uint16_t idx, uint8_t sub)
{
    (void)user;
    AGR_SDO_Msg_t req;
    AGR_SDO_CreateReadReq(&req, idx, sub);
    return AGR_CANFD_SendSDO(&s_fes_hub_inst.dop_ctx,
                             AGR_NODE_ID_FES_HUB, &req);
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — Step Array Actions
 *-----------------------------------------------------------
 */

static int _Step_SendTpdoMap(void)
{
    return AGR_CANFD_SendSDOWrite(&s_fes_hub_inst.dop_ctx,
                                FESHUB_OD_IDX_TPDO1_MAPPING, 0,
                                s_tpdo_map, sizeof(s_tpdo_map));
}

static int _Step_SendNmtStart(void)
{
    int ret = AGR_PnP_Master_SendNmt(s_fes_hub_inst.master_pnp,
                                      AGR_NODE_ID_FES_HUB,
                                      AGR_NMT_CMD_START);
    /* 진단: NMT Start Tx 증적 (Live Expression).
     * ret=0 이면 Tx enqueue OK. Slave 가 OPERATIONAL 로 전이 안 하면 Slave 측 문제. */
    s_diag_nmt_start_tx_count++;
    s_diag_nmt_start_last_ret = ret;
    s_diag_nmt_start_last_time = IOIF_TIM_GetTick();
    return ret;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — SDO Response
 *-----------------------------------------------------------
 */

static void _OnSdoResponse(const AGR_SDO_Msg_t* response)
{
    if (response == NULL) return;

    uint8_t cs_raw = response->cs;
    uint8_t cs = cs_raw & 0xE0;

    /* SDO Abort (cs==0x80) — AGR_SDO_Decode 가 abort code 4B 를 data[0..3] 에
     * LE 로 copy. 상태 변경 없이 진단 카운터만 갱신 → timeout/retry 경로 유지.
     * 0x6300/0x6310 에 대한 Abort 는 "Slave OD 또는 validity check 거절" 증거. */
    if (cs_raw == AGR_SDO_CS_ABORT) {
        s_diag_sdo_abort_count++;
        if (response->data_len >= 4) {
            s_diag_sdo_last_abort_code = (uint32_t)response->data[0]
                                       | ((uint32_t)response->data[1] << 8)
                                       | ((uint32_t)response->data[2] << 16)
                                       | ((uint32_t)response->data[3] << 24);
        }
        s_diag_sdo_last_abort_index  = response->index;
        s_diag_sdo_last_abort_subidx = response->subindex;

        /* IDENTIFY 진행 중 Abort — 컴포넌트에 주입만 (NonRealtime task).
         * 상태 전이/슬롯 처리는 RunPeriodic(pnp_task)이 GetResult 폴링으로
         * 일원화 — pre_op_slot 계약(pnp_task 한정) 준수. */
        if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_RUNNING) {
            (void)AGR_Identify_OnSdoResponse(&s_fes_hub_inst.identify,
                                             response, IOIF_TIM_GetTick());
        }
        return;
    }

    /* SDO Upload Response (0x40/0x41) — IDENTIFY SDO Read 응답 (주입만, 상동) */
    if (cs == AGR_SDO_CS_UPLOAD_INIT_RSP) {
        if (s_fes_hub_inst.pre_op_state == FESHUB_PRE_OP_IDENTIFY_RUNNING) {
            (void)AGR_Identify_OnSdoResponse(&s_fes_hub_inst.identify,
                                             response, IOIF_TIM_GetTick());
        }
        return;
    }

    /* SDO Download Response (ACK) — cs 체크 강화 (Abort 선분기 후 엄격 매칭).
     * ACK 카운터는 모든 WO OD (0x1A00 / 0x6300 / 0x6310) 공용 — index 로 구분. */
    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP) {
        s_diag_sdo_ack_count++;
        s_diag_sdo_last_ack_index = response->index;

        if (response->index == FESHUB_OD_IDX_TPDO1_MAPPING) {
            /* TPDO Mapping 완료 -> NMT START */
            s_fes_hub_inst.pre_op_state = FESHUB_PRE_OP_SEND_NMT_START;
            s_fes_hub_inst.sdo_retry_count = 0;
        }
    }
}
