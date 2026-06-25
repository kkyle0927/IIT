/**
 ******************************************************************************
 * @file    emg_hub_drv.c
 * @author  HyundoKim
 * @brief   [Device Layer] XM10 ↔ EMG Hub 통신 드라이버 (Master, CANopen 표준)
 * @version 1.1
 * @date    2026-03-20
 *
 * @details
 * [V2 Architecture Compliance]
 * - PDO: Mutex + Snapshot 패턴 (timeout=0, non-blocking)
 * - SDO: Pre-Op SM에서 직접 처리 (Device Layer 범위)
 * - ISR: IOIF RxTask 콜백 체인 (Semaphore Give는 IOIF가 처리)
 *
 * [동기화 패턴] (Rule 13: comm-core-patterns)
 * ┌─────────────────────────────────────────────────────────┐
 * │ IOIF RxTask (RT4, callback)                             │
 * │   → EmgHub_Drv_ProcessCANMessage()                      │
 * │   → osMutexAcquire(s_pdo_mutex, 1) + memcpy            │
 * │   → osMutexRelease()                                    │
 * │                                                         │
 * │ User Task / core_process (RT6, IPO Input)               │
 * │   → EmgHub_Drv_GetRxData()                              │
 * │   → osMutexAcquire(s_pdo_mutex, 0) + memcpy (Snapshot) │
 * │   → osMutexRelease()                                    │
 * │   → Use snapshot without lock (no jitter)               │
 * └─────────────────────────────────────────────────────────┘
 *
 * [메시지 처리 흐름]
 * canfd_rx_handler.c:
 *     ├─ 0x70F → AGR_PnP_Master_ProcessMessage() [PnP Master]
 *     ├─ 0x58F → EmgHub_Drv_ProcessCANMessage() [여기]
 *     └─ 0x18F → EmgHub_Drv_ProcessCANMessage() [여기]
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "emg_hub_drv.h"
#include "agr_pnp_master.h"
#include "agr_pnp_identify.h"   /* IDENTIFY 상태머신 (identity 비교검증) */
#include "xm_node_id.h"
#include "ioif_conf.h"
#include "Transport/CAN_FD/agr_dop_canfd.h"
#include "ioif_agrb_defs.h"
#include "ioif_agrb_tim.h"
#include <string.h>

/* FreeRTOS (Mutex) */
#include "cmsis_os2.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define EMGHUB_MAX_PDO_MAP_SIZE     64

/**
 * @brief Object Dictionary Index (EMG Hub Slave와 동일)
 *
 * [OD 구조]
 * 0x2100: ADC Status (RO, 1B)
 * 0x1A00: TPDO1 Mapping (WO, BLOB, CiA 301 표준 — 구 vendor 0x2110)
 * 0x3000: PDO Metadata (4B)
 * 0x6010: EMG Sensor Data (SubIndex 0x60 = 10B processed set)
 * 0x6050: ctrl_tick_ms (UINT32 4B, Slave Control Task tick) — TPDO1 에 포함
 */
#define EMGHUB_OD_IDX_PDO_MAPPING       0x1A00
#define EMGHUB_OD_IDX_METADATA          0x3000
#define EMGHUB_OD_IDX_EMG_DATA          0x6010
#define EMGHUB_OD_IDX_CTRL_TICK_MS      0x6050
#define EMGHUB_OD_IDX_CAL_COMMAND       0x2103  /**< EMG 캘리브 명령 (RW, 1B): offset(2)/mvc(3) */
#define EMGHUB_OD_IDX_CAL_MVC           0x2104  /**< EMG MVC 기준값 직접 주입 (RW, float32 µV) */

/* EMG Data SubIndex */
#define EMGHUB_OD_SUBIDX_PROCESSED      0x60  /**< {raw,voltage,rms,envelope,mvc,active} 10B */

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief Pre-Operational 단계 (Phase 1: IDENTIFY → Phase 2: CONFIG Step Array)
 * @details EMG Hub는 단일 TPDO만 사용 (PDO Map B 단계 없음).
 *
 * [컨텍스트 규율 — FES/IMU 정합]
 * IDENTIFY_PENDING 마킹은 Rx 컨텍스트(_PnP_OnBootup)에서 하지만, 슬롯 획득 +
 * AGR_Identify_Start(첫 SDO 송신)는 RunPeriodic(pnp_task 단일 컨텍스트)에서만
 * 수행한다 — pre_op_slot race 차단. IDENTIFY 도중 Boot-up 재수신 시 PENDING
 * 재마킹 → RunPeriodic이 Identify_Reset+Start로 재시작 (고아 응답 방지).
 */
typedef enum {
    EMGHUB_PRE_OP_IDLE = 0,
    EMGHUB_PRE_OP_IDENTIFY_PENDING,   /**< Boot-up 마킹 (Rx 컨텍스트) — 슬롯 대기 */
    EMGHUB_PRE_OP_IDENTIFY_RUNNING,   /**< 0x1000/0x1018 SDO Read 시퀀스 진행 */
    EMGHUB_PRE_OP_IDENTITY_MISMATCH,  /**< 검증 실패 — NMT START 영구 보류 (fail loud) */
    EMGHUB_PRE_OP_SEND_PDO_MAP,
    EMGHUB_PRE_OP_WAIT_PDO_MAP,
    EMGHUB_PRE_OP_SEND_NMT_START,
    EMGHUB_PRE_OP_COMPLETE
} EmgHub_PreOpState_t;

typedef int (*EmgHub_PreOpAction_t)(void);

typedef struct {
    EmgHub_PreOpState_t send_state;
    EmgHub_PreOpState_t wait_state;
    EmgHub_PreOpAction_t action;
    uint32_t            timeout_ms;
    const char*         description;
} EmgHub_PreOpStep_t;

/**
 * @brief EMG Hub 드라이버 인스턴스
 */
typedef struct {
    AGR_DOP_Ctx_t       dop_ctx;
    AGR_TxFunc_t        tx_func;

    AGR_PnP_Master_t*   master_pnp;
    int                 slave_index;

    /* Pre-Op State Machine — pre_op_state/sdo_retry_count는 ISR(canfd_rx 경유
     * _PnP_OnBootup)이 쓰고 pnp_task(RunPeriodic)가 읽는 교차 컨텍스트 공유:
     * volatile 필수 (컴파일러 레지스터 캐싱으로 ISR 갱신 미관측 방지). FES 정합 */
    volatile EmgHub_PreOpState_t pre_op_state;
    uint32_t            last_sdo_tx_time;
    volatile uint8_t    sdo_retry_count;

    /* IDENTIFY (Phase 1) */
    AGR_Identify_Ctx_t  identify;

    volatile bool       is_data_ready;
} EmgHub_DrvInst_t;

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static EmgHub_DrvInst_t s_emg_hub_inst;

/**
 * @brief PDO 공유 데이터 + Mutex (V2 Architecture: Mutex + Snapshot)
 * @details
 * - Producer: IOIF RxTask → EmgHub_Drv_ProcessCANMessage() (osMutexAcquire, timeout=1)
 * - Consumer: core_process → EmgHub_Drv_GetRxData() (osMutexAcquire, timeout=0 → Snapshot)
 * - Critical Section: memcpy만 (≤ 1µs for 16B)
 */
static osMutexId_t      s_pdo_mutex = NULL;
static EmgHub_RxData_t  s_pdo_shared = {0};

/**
 * @brief SDO Abort 진단 카운터 (Live Expression / 디버거 Watch 용)
 * @details PRE-OP stuck 근본 원인 진단. Slave 가 SDO 응답으로 Abort(0x80) 를
 *          보내면 여기에 기록된다. 특히 PDO Mapping(0x2110) 에 대한 Abort 가
 *          누적되면 XM 의 s_pdo_map[] 길이(10B)와 Slave OD 0x6010.0x60 실제
 *          data_size 불일치 가능성이 높다.
 *
 * 주요 Abort Code (4B LE, CiA 301):
 *   0x06070010  TYPE_MISMATCH   — data type 불일치
 *   0x06070012  DATA_LONG       — Master 가 기대보다 길게 보냄
 *   0x06070013  DATA_SHORT      — Master 가 기대보다 짧게 보냄  ← H1(a) 증거
 *   0x06040041  NO_MAP          — 해당 object 는 PDO 매핑 불가
 *   0x06040042  MAP_LEN         — PDO 총 길이 한도 초과
 *   0x06020000  NOT_EXIST       — OD 에 해당 index 없음
 *   0x06010002  READ_ONLY       — 쓰기 금지 object
 */
static volatile uint32_t s_diag_sdo_abort_count       = 0;  /**< Abort 수신 누적 */
static volatile uint32_t s_diag_sdo_last_abort_code   = 0;  /**< 최근 Abort 4B LE */
static volatile uint16_t s_diag_sdo_last_abort_index  = 0;  /**< 최근 Abort OD index */
static volatile uint8_t  s_diag_sdo_last_abort_subidx = 0;  /**< 최근 Abort OD subindex */

/**
 * @brief Pre-Op 진행 진단 카운터 (Live Expression / 디버거 Watch 용)
 * @details EMG PRE-OP stuck 분기 판정용. EMG 팀이 Slave 측 ACK 0x60 송신 확인 후
 *          "XM 이 NMT Start 를 실제로 보냈는가?" 를 XM 단독으로 입증하기 위함.
 *
 * 해석 매트릭스 (플래시 후 30초 관찰):
 *   pdo_map_tx=0       → Boot-up 미수신 or Pre-Op SM 진입 실패 (HB path 확인 필요)
 *   pdo_map_tx≥1, ret=0, nmt_start_tx=0 → SDO ACK 수신 못함 → NRT queue / SDO Resp routing 의심
 *                                          또는 sdo_abort_count≥1 → Slave 가 Abort 로 거절
 *   pdo_map_tx≥1, ret≠0 → FDCAN Tx 실패 (Bus Off / FIFO Full / HAL error)
 *   nmt_start_tx≥1, ret=0 → XM 은 NMT Start 송신 성공. 이후 응답은 Slave 측 분석
 *                          (EMG 가 NMT Start 받고도 OP 전환 안 하는 시나리오)
 *   nmt_start_tx≥1, ret≠0 → XM FDCAN Tx 실패 — IMU 는 성공하는데 EMG 만 실패라면
 *                          타이밍 race (FIFO 포화 순간) 검토
 *   pre_op_state_max == COMPLETE(enum 끝값, 현재 7) → State Machine 전부 성공
 *   pre_op_state_max < COMPLETE → 중간 단계 멈춤 (절대값 비교 금지 — IDENTIFY
 *     단계 추가로 enum 참조. 3=IDENTITY_MISMATCH → identity drift 차단됨)
 */
static volatile uint32_t s_diag_pdo_map_tx_count    = 0;  /**< _Step_SendPdoMap 호출 누적 */
static volatile int32_t  s_diag_pdo_map_last_ret    = 0;  /**< 마지막 PDO Map SDO Tx 반환값 */
static volatile uint32_t s_diag_nmt_start_tx_count  = 0;  /**< _Step_SendNmtStart 호출 누적 */
static volatile int32_t  s_diag_nmt_start_last_ret  = 0;  /**< 마지막 NMT Start Tx 반환값 */
static volatile uint32_t s_diag_nmt_start_last_time = 0;  /**< 마지막 NMT Start Tx 시각 (ms) */
static volatile uint8_t  s_diag_pre_op_state_max    = 0;  /**< Pre-Op SM 이 도달한 최대 state 값 (enum 참조, COMPLETE=7) */
static volatile uint8_t  s_diag_pre_op_state_now    = 0;  /**< 현재 Pre-Op state (폴링용) */

/**
 * @brief IDENTIFY 진단 (Live Expression) — MISMATCH 유도 관찰점 (FES 정합)
 */
static volatile uint8_t  s_diag_identify_result       = 0;  /**< AGR_Identify_Result_t 현재값 */
static volatile uint8_t  s_diag_identify_failed_step  = 0;  /**< 불일치 필드 (0=devtype..3=revision) */
static volatile uint32_t s_diag_identify_expected     = 0;  /**< 불일치 기대값 */
static volatile uint32_t s_diag_identify_actual       = 0;  /**< 불일치 실측값 */
static volatile uint32_t s_diag_identity_mismatch_cnt = 0;  /**< MISMATCH 진입 누적 */

/**
 * @brief EMG 기대 identity — Slave 매뉴얼(xm_drv.c OD) 기반 하드코딩.
 * @details revision = 와이어 계약 버전. EMG OD/와이어 계약 변경 시 Slave가
 *   0x1018:03 을 bump하며, 본 기대값을 함께 갱신해야 IDENTIFY가 통과한다
 *   (manual drift 를 부팅 시점에 검출). Slave OD: xm_drv.c s_device_type/
 *   s_vendor_id/s_product_code/s_revision_num 와 반드시 일치.
 */
static const AGR_Identify_Expected_t s_emg_expected_identity = {
    .device_type  = 0x000A0000u,   /* AGR_DEVICE_TYPE(0x000A, 0) — Rule 33 §0 */
    .vendor_id    = 0x414E474Cu,   /* "ANGL" */
    .product_code = 0x00040002u,   /* Rule 22 §OD allocation — EMG Hub */
    .revision     = 0x00040000u,   /* TPDO v2.1 (ctrl_tick 선두 + status 1B + Processed Set 10B + calib_valid bit3; mean/median_freq 제거 — EMG/XM FFT 안 함, MDF/MNF 는 PC 오프라인). 0x3 = Full Set 14B[freq]. 0x2 = 0x2110→0x1A00 */
};

/* Forward Declarations (Step Array Actions + IDENTIFY) */
static int  _Step_SendPdoMap(void);
static int  _Step_SendNmtStart(void);
static int  _EmgIdentify_SendRead(void* user, uint16_t idx, uint8_t sub);
static void _HandleIdentifyResult(AGR_Identify_Result_t r);

static const EmgHub_PreOpStep_t s_pre_op_steps[] = {
    { EMGHUB_PRE_OP_SEND_PDO_MAP,   EMGHUB_PRE_OP_WAIT_PDO_MAP,   _Step_SendPdoMap,    5000, "TPDO1 Mapping" },
    { EMGHUB_PRE_OP_SEND_NMT_START, EMGHUB_PRE_OP_COMPLETE,        _Step_SendNmtStart,  5000, "NMT START" },
};
#define PRE_OP_STEP_COUNT (sizeof(s_pre_op_steps) / sizeof(s_pre_op_steps[0]))

/**
 * @brief PDO Mapping 데이터 — Extended 8B entry format
 * @details TPDO v2.1: ctrl_tick_ms 0x6050.0x00 (4B) + Status 0x3000.0x00 (1B)
 *          + EMG Processed Set 0x6010.0x60 (10B: raw,volt,rms,env,mvc,active)
 *          = 15B. wire 는 DLC_16 (16B 프레임, 1B padding). (v2: 0x61 Full Set 19B 에서 freq 2B×2 제거)
 *
 * [Wire format — agr_pdo_engine.c]
 *   Header byte : bit7 = Extended flag (1), bit0..6 = count (max 127)
 *   Entry 8B    : Index(2 LE) + SubIndex(1) + Flags(1) + Length(4 LE uint32)
 *
 * [v2.1 변경] mean/median_freq 제거 — MDF/MNF FFT 는 PC/MATLAB 오프라인(raw_adc 로깅 후),
 *          EMG/XM 는 FFT 안 함. (v2: 24-bit timestamp 제거 + ctrl_tick 선두 + calib_valid bit3)
 *
 * [Blob total] 1 + 3 * 8 = 25 bytes (< AGR_SDO_MAX_DATA_SIZE = 60).
 */
static const uint8_t s_pdo_map[] = {
    /* Header: count=3 | Extended flag (bit7) */
    0x83,

    /* Entry 1: 0x6050.0x00 ctrl_tick_ms, length=4 (TPDO v2 선두) */
    0x50, 0x60,              /* Index 0x6050 LE */
    0x00,                    /* SubIndex */
    0x00,                    /* Flags (reserved) */
    0x04, 0x00, 0x00, 0x00,  /* Length = 4 B (LE uint32) */

    /* Entry 2: 0x3000.0x00 Status Flags, length=1 */
    0x00, 0x30,              /* Index 0x3000 LE */
    0x00,                    /* SubIndex */
    0x00,                    /* Flags (reserved) */
    0x01, 0x00, 0x00, 0x00,  /* Length = 1 B (LE uint32) */

    /* Entry 3: 0x6010.0x60 EMG Processed Set (raw,volt,rms,env,mvc,active), length=10 */
    0x10, 0x60,              /* Index 0x6010 LE */
    0x60,                    /* SubIndex */
    0x00,                    /* Flags (reserved) */
    0x0A, 0x00, 0x00, 0x00,  /* Length = 10 B (LE uint32) */
};

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* PnP 콜백 */
static void _PnP_OnBootup(uint8_t slave_node_id);
static void _PnP_OnStateChanged(uint8_t slave_node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
static void _PnP_OnOnline(uint8_t slave_node_id);
static void _PnP_OnOffline(uint8_t slave_node_id);

/* SDO Response 처리 */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response);

/* Pre-Op SM */
static void _RunPreOpStateMachine(void);

/* PDO 디코딩 */
static void _DecodeTpdo1(EmgHub_RxData_t* buf, const uint8_t* data, uint8_t len);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

int EmgHub_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Master_t* master_pnp)
{
    if (tx_func == NULL || master_pnp == NULL) {
        return -1;
    }

    memset(&s_emg_hub_inst, 0, sizeof(EmgHub_DrvInst_t));
    memset(&s_pdo_shared, 0, sizeof(EmgHub_RxData_t));

    s_emg_hub_inst.tx_func = tx_func;
    s_emg_hub_inst.master_pnp = master_pnp;
    s_emg_hub_inst.slave_index = -1;

    /* PDO Mutex 생성 */
    if (s_pdo_mutex == NULL) {
        const osMutexAttr_t mutex_attr = {
            .name = "EmgPdoMtx",
            .attr_bits = osMutexPrioInherit,
        };
        s_pdo_mutex = osMutexNew(&mutex_attr);
    }

    /* AGR_CANFD 초기화 (Master: OD 불필요) */
    AGR_CANFD_Init(&s_emg_hub_inst.dop_ctx,
                   NULL,
                   AGR_NODE_ID_XM,
                   tx_func);
    AGR_CANFD_SetTargetNodeId(&s_emg_hub_inst.dop_ctx, AGR_NODE_ID_EMG_HUB);

    /* PnP Master에 Slave 등록 */
    AGR_PnP_SlaveConfig_t slave_config = {
        .name = "EMG Hub",
        .node_id = AGR_NODE_ID_EMG_HUB,
        .heartbeat_timeout_ms = 3000,
        .callbacks = {
            .on_slave_bootup        = _PnP_OnBootup,
            .on_slave_state_changed = _PnP_OnStateChanged,
            .on_slave_online        = _PnP_OnOnline,
            .on_slave_offline       = _PnP_OnOffline,
        },
        .expected_identity    = &s_emg_expected_identity,
        .identity_check_flags = AGR_IDENTIFY_FLAG_ALL,
    };

    int idx = AGR_PnP_Master_AddSlave(master_pnp, &slave_config);
    if (idx < 0) {
        return -1;
    }
    s_emg_hub_inst.slave_index = idx;

    /* IDENTIFY 컴포넌트 — STRICT: 불일치 시 NMT START 보류 (fail loud) */
    AGR_Identify_Init(&s_emg_hub_inst.identify,
                      _EmgIdentify_SendRead, NULL,
                      &s_emg_expected_identity,
                      AGR_IDENTIFY_FLAG_ALL,
                      AGR_IDENTIFY_MODE_STRICT);

    return 0;
}

/**
 * @brief CAN 메시지 처리 (TPDO/SDO만, Heartbeat 제외)
 * @details
 * [V2 Architecture] IOIF RxTask(RT4) 콜백 컨텍스트에서 실행.
 * PDO 수신 시 Mutex + memcpy로 공유 데이터 갱신 (timeout=0).
 * Mutex 획득 실패 시 이번 PDO는 Drop (다음 사이클에서 갱신).
 */
void EmgHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return;
    }

    uint16_t fnc_code = can_id & 0x780;

    /* SDO Response (0x580 + Node ID) */
    if (fnc_code == 0x580) {
        AGR_SDO_Msg_t sdo_msg;
        if (AGR_SDO_Decode(data, len, &sdo_msg) == 0) {
            _OnSdoResponse(&sdo_msg);
        }
        return;
    }

    /* TPDO1 (0x180 + Node ID = 0x18F) — Mutex + Snapshot 패턴 */
    if (fnc_code == 0x180) {
        EmgHub_RxData_t decoded;
        _DecodeTpdo1(&decoded, data, len);

        if (osMutexAcquire(s_pdo_mutex, 1) == osOK) {
            memcpy(&s_pdo_shared, &decoded, sizeof(EmgHub_RxData_t));
            osMutexRelease(s_pdo_mutex);
            s_emg_hub_inst.is_data_ready = true;
        }
        return;
    }
}

bool EmgHub_Drv_IsConnected(void)
{
    return AGR_PnP_Master_IsSlaveOnline(s_emg_hub_inst.master_pnp,
                                         AGR_NODE_ID_EMG_HUB);
}

AGR_NMT_State_t EmgHub_Drv_GetNmtState(void)
{
    return AGR_PnP_Master_GetSlaveState(s_emg_hub_inst.master_pnp,
                                         AGR_NODE_ID_EMG_HUB);
}

bool EmgHub_Drv_IsIdentityMismatch(void)
{
    return (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTITY_MISMATCH);
}

/* ===== EMG 캘리브레이션 트리거 (XM master → EMG SDO Write) =====
 * EMG OD 0x2103/0x2104 에 SDO Write → Slave 가 deferred-SDO main loop 에서
 * EmgProc_CalibrateOffset/SetMvcValue 호출. (USB sensor-studio 경로와 동일 결과) */
int EmgHub_Drv_SendCalCommand(uint8_t cmd)
{
    return AGR_CANFD_SendSDOWrite(&s_emg_hub_inst.dop_ctx,
                                  EMGHUB_OD_IDX_CAL_COMMAND, 0x00,
                                  &cmd, 1);
}

int EmgHub_Drv_SetMvcValue(float mvc_uv)
{
    uint8_t buf[4];
    memcpy(buf, &mvc_uv, sizeof(buf));   /* float32 LE wire */
    return AGR_CANFD_SendSDOWrite(&s_emg_hub_inst.dop_ctx,
                                  EMGHUB_OD_IDX_CAL_MVC, 0x00,
                                  buf, sizeof(buf));
}

void EmgHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();

    /* 진단: Pre-Op state 폴링 (Live Expression 용, 100ms 주기 tick 에서 갱신) */
    s_diag_pre_op_state_now = (uint8_t)s_emg_hub_inst.pre_op_state;
    if (s_diag_pre_op_state_now > s_diag_pre_op_state_max) {
        s_diag_pre_op_state_max = s_diag_pre_op_state_now;
    }

    /* 0. IDENTIFY Phase 구동 (pnp_task 단일 컨텍스트 — 슬롯 계약) */
    if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_PENDING) {
        if (AGR_PnP_Master_TryAcquirePreOpSlot(s_emg_hub_inst.master_pnp,
                                               AGR_NODE_ID_EMG_HUB)) {
            AGR_Identify_Reset(&s_emg_hub_inst.identify);
            AGR_Identify_Result_t r =
                AGR_Identify_Start(&s_emg_hub_inst.identify, current_ms);
            s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDENTIFY_RUNNING;
            _HandleIdentifyResult(r);   /* flags=0이면 즉시 PASS 가능 */
        }
        /* 슬롯 점유 중(타 Slave Pre-Op) — 다음 주기 재시도 */
    } else if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_RUNNING) {
        /* 타임아웃/재시도 + Rx task가 주입한 terminal 결과 수거 */
        (void)AGR_Identify_RunPeriodic(&s_emg_hub_inst.identify, current_ms);
        _HandleIdentifyResult(AGR_Identify_GetResult(&s_emg_hub_inst.identify));
    }
    s_diag_identify_result = (uint8_t)AGR_Identify_GetResult(&s_emg_hub_inst.identify);

    /* 1. Pre-Op Timeout 체크 (SDO ACK 대기 상태) */
    if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_WAIT_PDO_MAP) {
        if (current_ms - s_emg_hub_inst.last_sdo_tx_time > 5000) {
            if (s_emg_hub_inst.sdo_retry_count < 3) {
                s_emg_hub_inst.sdo_retry_count++;
                s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_SEND_PDO_MAP;
            } else {
                s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDLE;
                s_emg_hub_inst.sdo_retry_count = 0;
            }
        }
    }

    /* 2. Pre-Op State Machine 구동 */
    _RunPreOpStateMachine();
}

/**
 * @brief 최신 EMG 데이터 읽기 (Mutex + Snapshot, V2 Architecture)
 * @details
 * [V2 Mutex + Snapshot Pattern]
 * - osMutexAcquire(timeout=0): Non-blocking
 * - 성공 시: memcpy로 Snapshot 생성 → Mutex 즉시 해제
 * - 실패 시: 이전 Snapshot 재사용 (Graceful Degradation)
 * - Snapshot 이후: Lock 없이 안전하게 사용 (no jitter)
 */
bool EmgHub_Drv_GetRxData(EmgHub_RxData_t* rx_data)
{
    if (rx_data == NULL) {
        return false;
    }

    if (!s_emg_hub_inst.is_data_ready) {
        return false;
    }

    if (osMutexAcquire(s_pdo_mutex, 0) == osOK) {
        memcpy(rx_data, &s_pdo_shared, sizeof(EmgHub_RxData_t));
        osMutexRelease(s_pdo_mutex);
        return true;
    }
    return false;
}

bool EmgHub_Drv_IsDataReady(void)
{
    return s_emg_hub_inst.is_data_ready;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — PnP Callbacks
 *-----------------------------------------------------------
 */

static void _PnP_OnBootup(uint8_t slave_node_id)
{
    (void)slave_node_id;

    /* Rx 컨텍스트 — IDENTIFY_PENDING 마킹만. 슬롯 획득 + Identify_Start는
     * RunPeriodic(pnp_task) 단일 컨텍스트에서 (슬롯 계약). Boot-up = Slave 리셋
     * → 어느 state 든 항상 재-IDENTIFY (FES 정합). CONFIG 중이면 슬롯 보유
     * 상태에서 PENDING → RunPeriodic이 TryAcquire 재진입으로 시퀀스 재시작,
     * COMPLETE면 해제 상태 → RunPeriodic이 재획득. IDENTIFY 도중 재수신도
     * PENDING 재마킹 → Identify_Reset+Start로 재시작 (고아 응답 방지). */
    s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDENTIFY_PENDING;
    s_emg_hub_inst.sdo_retry_count = 0;
}

static void _PnP_OnStateChanged(uint8_t slave_node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state)
{
    (void)slave_node_id;
    (void)old_state;
    (void)new_state;
}

static void _PnP_OnOnline(uint8_t slave_node_id)
{
    (void)slave_node_id;
    /* 재연결 시 Pre-Op SM 재시작 힌트. Bootup Recovery 로직(_RunPreOpStateMachine
     * IDLE 상태에서 Slave NMT=PRE_OP 감지 시 SEND_PDO_MAP 전이) 이 이미 있으므로
     * 필수는 아니지만 FES/IMU 패턴 정합 + 재연결 명시성 확보. */
    s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDLE;
    s_emg_hub_inst.sdo_retry_count = 0;
}

static void _PnP_OnOffline(uint8_t slave_node_id)
{
    (void)slave_node_id;

    s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDLE;
    s_emg_hub_inst.sdo_retry_count = 0;
    s_emg_hub_inst.is_data_ready = false;

    /* IDENTIFY 정리 + 슬롯 반환 (Offline은 pnp_task의 RunPeriodic 경유 —
     * 슬롯 계약 컨텍스트 내) */
    AGR_Identify_Reset(&s_emg_hub_inst.identify);
    AGR_PnP_Master_ReleasePreOpSlot(s_emg_hub_inst.master_pnp, AGR_NODE_ID_EMG_HUB);
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
     * LE 로 copy. 상태 변경 없이 진단 카운터만 갱신 → timeout/retry 경로 유지. */
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

        /* IDENTIFY 진행 중 Abort — 컴포넌트에 주입만 (Rx task 컨텍스트).
         * 상태 전이/슬롯 처리는 RunPeriodic(pnp_task)이 GetResult 폴링으로
         * 일원화 — pre_op_slot 계약(pnp_task 한정) 준수. */
        if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_RUNNING) {
            (void)AGR_Identify_OnSdoResponse(&s_emg_hub_inst.identify,
                                             response, IOIF_TIM_GetTick());
        }
        return;
    }

    /* SDO Upload Response (0x40/0x41) — IDENTIFY SDO Read 응답 (주입만, 상동) */
    if (cs == AGR_SDO_CS_UPLOAD_INIT_RSP) {
        if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_RUNNING) {
            (void)AGR_Identify_OnSdoResponse(&s_emg_hub_inst.identify,
                                             response, IOIF_TIM_GetTick());
        }
        return;
    }

    /* SDO Download Response (ACK) — cs 체크 누락 시 Abort(0x80) 를 ACK 로 오판
     * → NMT_START 조기 전송 → Slave PRE_OP 잔류 → TPDO 미수신. IMU/FES 패턴 정합.
     * index 는 EMGHUB_OD_IDX_PDO_MAPPING (0x1A00) — TPDO 매핑 ACK 만 수용. */
    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP) {
        if (response->index == EMGHUB_OD_IDX_PDO_MAPPING) {
            if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_WAIT_PDO_MAP) {
                s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_SEND_NMT_START;
                s_emg_hub_inst.sdo_retry_count = 0;
            }
        }
    }
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — Pre-Op State Machine
 *-----------------------------------------------------------
 */

static void _RunPreOpStateMachine(void)
{
    /* Boot-up Recovery: IDLE 상태에서 Slave가 PRE_OP이면 IDENTIFY부터 진입
     * → Boot-up 콜백을 놓쳤을 때 복구 (FES/IMU 동일 패턴) */
    if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDLE) {
        AGR_NMT_State_t slave_state = AGR_PnP_Master_GetSlaveState(
            s_emg_hub_inst.master_pnp, AGR_NODE_ID_EMG_HUB);
        if (slave_state == AGR_NMT_PRE_OPERATIONAL) {
            s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDENTIFY_PENDING;
            s_emg_hub_inst.sdo_retry_count = 0;
        }
        return;
    }

    /* IDENTIFY/MISMATCH/COMPLETE는 Step Array 비대상 —
     * NMT START는 IDENTIFY PASS 후에만 도달 가능 (CONFIG phase) */
    if (s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_PENDING ||
        s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTIFY_RUNNING ||
        s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_IDENTITY_MISMATCH ||
        s_emg_hub_inst.pre_op_state == EMGHUB_PRE_OP_COMPLETE) {
        return;
    }

    for (uint8_t i = 0; i < PRE_OP_STEP_COUNT; i++) {
        if (s_emg_hub_inst.pre_op_state == s_pre_op_steps[i].send_state) {
            if (s_pre_op_steps[i].action() == 0) {
                s_emg_hub_inst.last_sdo_tx_time = IOIF_TIM_GetTick();
                s_emg_hub_inst.pre_op_state = s_pre_op_steps[i].wait_state;
                s_emg_hub_inst.sdo_retry_count = 0;

                /* Pre-Op 완료 — 직렬화 슬롯 반환 (타 Slave Pre-Op 진입 허용) */
                if (s_pre_op_steps[i].wait_state == EMGHUB_PRE_OP_COMPLETE) {
                    AGR_PnP_Master_ReleasePreOpSlot(s_emg_hub_inst.master_pnp,
                                                    AGR_NODE_ID_EMG_HUB);
                }
            }
            return;
        }
    }
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — Pre-Op Step Actions
 *-----------------------------------------------------------
 */

static int _Step_SendPdoMap(void)
{
    int ret = AGR_CANFD_SendSDOWrite(&s_emg_hub_inst.dop_ctx,
                                     EMGHUB_OD_IDX_PDO_MAPPING, 0x00,
                                     s_pdo_map, sizeof(s_pdo_map));
    s_diag_pdo_map_tx_count++;
    s_diag_pdo_map_last_ret = ret;
    return ret;
}

static int _Step_SendNmtStart(void)
{
    int ret = AGR_PnP_Master_SendNmt(s_emg_hub_inst.master_pnp,
                                      AGR_NODE_ID_EMG_HUB,
                                      AGR_NMT_CMD_START);
    /* 진단: NMT Start Tx 실제 호출 증적 (Live Expression).
     * ret=0 이면 FDCAN Tx FIFO 에 정상 enqueue — Slave 가 Frame 을 놓쳤다면
     * Slave 측 HW Filter (0x000) 또는 AGR_NMT_ProcessMessage 분기 점검. */
    s_diag_nmt_start_tx_count++;
    s_diag_nmt_start_last_ret = ret;
    s_diag_nmt_start_last_time = IOIF_TIM_GetTick();

    /* COMPLETE 전이 + 직렬화 슬롯 반환은 _RunPreOpStateMachine 이 wait_state
     * (COMPLETE) 도달 시 일원화 처리 (FES 정합) — 여기서 직접 설정 금지. */
    return ret;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — IDENTIFY (Phase 1)
 *-----------------------------------------------------------
 */

/**
 * @brief IDENTIFY 결과 처리 — PASS→CONFIG 진행 / MISMATCH→NMT START 보류
 * @note  pnp_task 컨텍스트 전용 (RunPeriodic 경유 — 슬롯 계약)
 */
static void _HandleIdentifyResult(AGR_Identify_Result_t r)
{
    switch (r) {
        case AGR_IDENTIFY_RESULT_PASS:
            s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_SEND_PDO_MAP;
            s_emg_hub_inst.sdo_retry_count = 0;
            break;

        case AGR_IDENTIFY_RESULT_MISMATCH:
            /* fail loud: NMT START 영구 보류 (Boot-up 재수신 시에만 재검증).
             * 슬롯은 반환 — 타 Slave Pre-Op 진입 허용. */
            s_diag_identity_mismatch_cnt++;
            s_diag_identify_failed_step = (uint8_t)s_emg_hub_inst.identify.last_mismatch.failed_step;
            s_diag_identify_expected    = s_emg_hub_inst.identify.last_mismatch.expected;
            s_diag_identify_actual      = s_emg_hub_inst.identify.last_mismatch.actual;
            s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDENTITY_MISMATCH;
            AGR_PnP_Master_ReleasePreOpSlot(s_emg_hub_inst.master_pnp,
                                            AGR_NODE_ID_EMG_HUB);
            break;

        case AGR_IDENTIFY_RESULT_TIMEOUT:
        case AGR_IDENTIFY_RESULT_ABORT:
            /* Slave 무응답/거부 — IDLE 복귀 (Boot-up/PRE_OP 감지로 재시도) */
            s_emg_hub_inst.pre_op_state = EMGHUB_PRE_OP_IDLE;
            s_emg_hub_inst.sdo_retry_count = 0;
            AGR_PnP_Master_ReleasePreOpSlot(s_emg_hub_inst.master_pnp,
                                            AGR_NODE_ID_EMG_HUB);
            break;

        default:
            break;  /* IDLE / IN_PROGRESS — 대기 */
    }
}

/**
 * @brief IDENTIFY SDO Read 송신 (DI — AGR_Identify 가 단계별 호출)
 */
static int _EmgIdentify_SendRead(void* user, uint16_t idx, uint8_t sub)
{
    (void)user;
    AGR_SDO_Msg_t req;
    AGR_SDO_CreateReadReq(&req, idx, sub);
    return AGR_CANFD_SendSDO(&s_emg_hub_inst.dop_ctx,
                             AGR_NODE_ID_EMG_HUB, &req);
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — PDO Decoding
 *-----------------------------------------------------------
 */

/**
 * @brief TPDO v2.1 데이터 디코딩 (ctrl_tick + status + EMG Processed Set)
 * @details
 * Byte Layout (15B total, wire 는 DLC_16 으로 16B 프레임, 1B padding):
 *   [0-3]   ctrl_tick_ms (uint32, LE) — Slave 제어 틱 (송신 1kHz 시점, gap 감지)
 *   [4]     status_flags (bit0 ADC_OK / bit1 IS_ACTIVE / bit2 SATURATED / bit3 CALIB_VALID)
 *   [5-6]   raw_adc (uint16, LE)
 *   [7-8]   voltage_uv_x10 (int16, LE)
 *   [9-10]  rms_uv_x10 (int16, LE)
 *   [11-12] envelope_uv_x10 (int16, LE)
 *   [13]    mvc_percent (uint8)
 *   [14]    is_active (uint8)
 * (v2.1: mean/median_freq 제거 — MDF/MNF FFT 는 PC/MATLAB 오프라인, EMG/XM FFT 안 함)
 */
static void _DecodeTpdo1(EmgHub_RxData_t* buf, const uint8_t* data, uint8_t len)
{
    if (len < 15) return;   /* v2.1 고정 15B (ctrl_tick4 + status1 + ProcessedSet10) */

    /* [0-3] ctrl_tick_ms (4B LE) — Slave 제어 틱 ground truth (송신 1kHz 시점) */
    buf->ctrl_tick_ms = (uint32_t)data[0]
                      | ((uint32_t)data[1] << 8)
                      | ((uint32_t)data[2] << 16)
                      | ((uint32_t)data[3] << 24);

    /* [4] status_flags */
    buf->status_flags = data[4];

    /* [5-14] EMG Processed Set (0x6010:0x60, 10B, Little-Endian) */
    buf->raw_adc         = (uint16_t)data[5]  | ((uint16_t)data[6] << 8);
    buf->voltage_uv_x10  = (int16_t)((uint16_t)data[7]  | ((uint16_t)data[8] << 8));
    buf->rms_uv_x10      = (int16_t)((uint16_t)data[9]  | ((uint16_t)data[10] << 8));
    buf->envelope_uv_x10 = (int16_t)((uint16_t)data[11] | ((uint16_t)data[12] << 8));
    buf->mvc_percent      = data[13];
    buf->is_active        = data[14];
}
