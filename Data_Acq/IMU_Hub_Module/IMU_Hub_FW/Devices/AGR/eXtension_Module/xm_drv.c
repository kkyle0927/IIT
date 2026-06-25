/**
 ******************************************************************************
 * @file    xm_drv.c
 * @author  HyundoKim
 * @brief   [Device Layer] IMU Hub ↔ XM10 통신 드라이버 (DOP V3 + PnP V2 Slave)
 * @version 5.0
 * @date    2026-02-25
 *
 * @details
 * IMU Hub Module이 XM10(Master)과 FDCAN으로 통신하기 위한 드라이버입니다.
 * AGR_PnP_Slave API를 사용하여 Master 추적, Boot-up/Heartbeat를 자동 관리합니다.
 * 
 * [주요 변경사항 V5.0 - DOP V3 직접 마이그레이션]
 * - Facade(agr_dop.h) 미사용, Core/Transport API 직접 사용
 * - Core: AGR_OD_, AGR_SDO_, AGR_PDO_ (순수 C, Transport-Agnostic)
 * - Transport: AGR_CANFD_ (CAN-FD 전용)
 * - _OnSdoRequest 콜백 제거 → AGR_SDO_ProcessRequest()가 OD 기반 자동 처리
 * - PDO Mapping: AGR_PDO_ApplyMapFromSDO() 직접 호출
 * 
 * [V4.0 변경사항 (유지)]
 * - AGR_PnP_Slave_t* 외부 주입 (System Layer에서 생성)
 * - 0x700, 0x000 메시지 → AGR_PnP_Slave_ProcessMessage()로 라우팅
 * - Boot-up/Heartbeat: AGR_PnP_Slave_RunPeriodic() 자동 처리
 * 
 * [데이터 흐름 - BareMetal, FDCAN Tx single-writer]
 * - PDO Tx: s_imu_hub_tx_data.imu[] → TPDO encode → Timer ISR direct (1kHz)
 * - PDO/Heartbeat Rx: ISR에서 직접 변수 업데이트 (volatile/flag)
 * - SDO Rx: FDCAN RX ISR 은 defer-ring 적재만 → main() XM_Drv_ProcessDeferredSdo
 *   에서 decode + OD write, 응답은 tx-defer-ring → Timer ISR(단일 송신자)가 drain.
 *   (BareMetal TX_LOCK no-op → HW Tx 를 Timer ISR 한 컨텍스트로 일원화)
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_drv.h"
#include "imu_node_id.h"
#include "agr_dop_config.h"     /* AGR_CAN_ID_SYNC */
#include "agr_nmt.h"
#include "agr_pnp_slave.h"    /* V4.0: AGR_PnP_Slave API (agr_pnp.h 대체) */
#include "ioif_agrb_tim.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** @brief Heartbeat 타임아웃 (ms) */
#define XM_DRV_HEARTBEAT_TIMEOUT_MS     1000

/** @brief PDO 페이로드 최대 크기 */
#define XM_DRV_PDO_MAX_SIZE             64

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PUBLIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* ===== FDCAN Instance ID (Software Queue 사용) ===== */
static IOIF_FDCANx_t s_fdcan_id = IOIF_FDCAN_INVALID_ID;  /**< FDCAN Instance ID */

/* ===== AGR_PnP Slave 인스턴스 (V4.0: System Layer에서 주입) =====
 * [변경 이유]
 * - 기존: 내부 AGR_NMT_Inst_t + AGR_PnP_Inst_t 별도 관리
 * - 변경: AGR_PnP_Slave_t* 외부 주입 (NMT 인스턴스 내장)
 * - 효과: NMT, Heartbeat, Boot-up을 PnP Slave가 통합 관리
 */
static AGR_PnP_Slave_t* s_slave_pnp = NULL;  /**< PnP Slave 포인터 (System에서 주입) */

/* ===== Rx 데이터 (XM10 → IMU Hub) - Lock-free Double Buffering ===== */
/**
 * @brief Lock-free Double Buffer (Bare-metal 최적화)
 * @details 
 * [동작 원리]
 * - ISR: write_idx 버퍼에 쓰고, 완료 후 read_idx ↔ write_idx 스왑
 * - Main: read_idx 버퍼를 읽음 (ISR이 쓰는 버퍼와 독립)
 * 
 * [안전성 보장]
 * - volatile uint8_t: ARM Cortex-M에서 원자적 읽기/쓰기
 * - Data Tearing 불가능 (Main이 읽는 버퍼는 ISR이 절대 건드리지 않음)
 * - IRQ Disable 불필요 (Zero Blocking Time)
 * 
 * [미래 확장]
 * - 현재는 XM으로부터 실시간 제어 입력을 받지 않음
 * - Motor Driver 등에서 Continuous Control Input을 받을 경우 사용
 */
typedef struct {
    XM_RxData_t buffers[2];      /**< 2개의 버퍼 */
    volatile uint8_t write_idx;  /**< ISR이 쓸 버퍼 (0 or 1) */
    volatile uint8_t read_idx;   /**< Main이 읽을 버퍼 (1 or 0) */
} XM_DoubleBuffer_t;

static XM_DoubleBuffer_t s_xm_double_buf = {
    .write_idx = 0,
    .read_idx = 1
};

/* ===== 센서 상태 (OD Entry용, Double Buffer 밖에 별도 관리) ===== */
static uint8_t s_imu_connected_mask = 0;  /**< IMU 연결 상태 (Main Loop에서 업데이트) */

/* ===== SYNC Phase-Lock (XM → SM, CAN-ID 0x080) ===== */
static volatile bool     s_sync_pending    = false;  /**< ISR에서 set, Control Task에서 consume */
static volatile uint32_t s_sync_counter_rx = 0;      /**< 디버깅: 총 SYNC 수신 횟수 */

/* ===== master-online abort 지연 (RX ISR → Timer ISR, single-writer) =====
 * _OnMasterOnline(FDCAN RX ISR)이 AbortAllTx 를 직접 호출하면 RX ISR 이 HW Tx 를
 * 건드려 Timer ISR 의 TPDO 송신과 경합(BareMetal TX_LOCK no-op). RX ISR 은 flag 만
 * set, Timer ISR(XM_Drv_ServiceMasterOnlineAbort)이 RunLoop 맨 앞(TPDO 前) consume. */
static volatile bool     s_master_online_abort_pending = false;

/* ===== SDO main-loop defer ring (SM Safety Checklist: "SDO: ISR에서 플래그만") =====
 * FDCAN Rx ISR 은 0x600 프레임을 ring 에 적재만 하고, 실제 처리(decode + OD write
 * = PDO 매핑 재구성 + 응답 Tx)는 main() while(1) 의 XM_Drv_ProcessDeferredSdo() 에서
 * 수행한다. 무거운 SDO decode 가 1kHz Timer ISR 예산을 먹지 않도록 분리(결정성 보호).
 * SPSC: producer=FDCAN Rx ISR(head), consumer=main loop(tail). uint8 인덱스 =
 * Cortex-M4 단일워드 원자적. IMU SDO 는 비주기(PnP TPDO 매핑 + 진단 cmd) — burst
 * 없음, 4 slot 충분, full=drop-newest (XM 측 재시도 존재). EMG/FES 패턴 정합. */
#define XM_SDO_DEFER_SLOTS   4U
typedef struct {
    uint32_t can_id;
    uint8_t  len;
    uint8_t  data[64];
} XmSdoDeferFrame_t;
static XmSdoDeferFrame_t s_sdo_defer_ring[XM_SDO_DEFER_SLOTS];
static volatile uint8_t  s_sdo_defer_head = 0;  /**< FDCAN ISR 가 증가 (producer) */
static volatile uint8_t  s_sdo_defer_tail = 0;  /**< main loop 가 증가 (consumer) */
static volatile uint32_t s_sdo_defer_drop = 0;  /**< ring full drop 진단 */

/* ===== FDCAN Tx defer ring (main → Timer ISR, single-writer 일원화) =====
 * main() 컨텍스트(XM_Drv_ProcessDeferredSdo → SDO 응답)에서 생성된 비-PDO 프레임을
 * 직접 HW 송신하지 않고 여기 적재 → Timer ISR(XM_Drv_DrainTxDeferRing)이 단일 송신.
 * BareMetal TX_LOCK 이 no-op 이라 HW Tx 는 Timer ISR 한 곳으로 일원화해야 안전.
 * SPSC: producer=main(head), consumer=Timer ISR(tail). XmSdoDeferFrame_t 재사용.
 * leave-in-place backpressure: HW full 시 ring 잔류(tail 미advance) 다음 tick 재시도
 * → 비-PDO 무손실. main 발 EMCY 도 이 ring 경유(EMG 정합) — 현재 EMCY 트리거가
 * SDO write(main) 뿐이라 single-writer 유지. ISR 발 EMCY 만 직접경로(Timer ISR). */
#define XM_TX_DEFER_SLOTS    8U
static XmSdoDeferFrame_t s_tx_defer_ring[XM_TX_DEFER_SLOTS];
static volatile uint8_t  s_tx_defer_head = 0;  /**< main loop 가 증가 (producer) */
static volatile uint8_t  s_tx_defer_tail = 0;  /**< Timer ISR 가 증가 (consumer) */
static volatile uint32_t s_tx_defer_drop = 0;  /**< ring full drop 진단 (no-loss 위반 감지=0 기대) */

/* ===== Tx 데이터 (IMU Hub → XM10, cm_drv 패턴) ===== */
static XM_TxData_t s_imu_hub_tx_data;  /**< 송신 데이터 (PDO + SDO 통합 관리) */

/* ===== [Snapshot/HD/IMU_Resolution_For_Swiss] IMU float32 원본 슬롯 (40B) =====
 * Swiss 학회 시연 전용 — 첫 연결 IMU 1개를 다운스케일 없이 float32 로 송신한다.
 * OD 0x6000.0x70 에 매핑(int16 0x60 과 가산 공존). _FlushOutputs 가 매 tick 갱신.
 * write(TIM16 ISR) → encode(동일 TIM16 ISR) 단일 컨텍스트 → 추가 동시성 race 없음. */
static XM_Drv_ImuDataF_t s_imu_float_slot = {0};

/**
 * Forward declarations (OD Entry WriteCb)
 */
static void _OnPdoMappingA_Set(void);
static void _OnPdoMappingB_Set(void);
static void _OnEmcyTestTrigger(void);  /* 더미 EMCY 주입 (SDO 0x7F02.01 write) */

/* EMCY 테스트 주입용 버퍼 (SDO write target, 3B: code_lo, code_hi, error_reg) */
static uint8_t s_emcy_test_payload[3] = {0};

/* 
 * ✅ CANopen 표준 WriteCb 활용
 * 
 * [WriteCb는 SDO Download (Write) 완료 후에만 호출됨]
 * - PDO Mapping: SDO로 전송 (CAN ID 0x600) → WriteCb 호출 ✅
 * 
 * [별도 프로토콜 메시지는 AGR_NMT가 처리]
 * - Boot-up: CAN ID 0x700 + Node ID → AGR_NMT_ProcessMessage ✅
 * - NMT: CAN ID 0x000 → AGR_NMT_ProcessMessage ✅
 * - Heartbeat: CAN ID 0x700 + Node ID → AGR_NMT_ProcessMessage ✅
 */

/* ===== PDO Mapping 데이터 ===== */
static uint8_t s_pdo_mapping_a[64];      /**< TPDO1 Mapping (Group A: IMU 0,1,2) */
static uint8_t s_pdo_mapping_b[64];      /**< TPDO2 Mapping (Group B: IMU 3,4,5) */

/* ===== PDO map 갱신 race 가드 (main WriteCb ↔ Timer ISR SendTPDO encode) =====
 * _OnPdoMappingA/B_Set(main, deferred SDO)이 AGR_PDO_ApplyMapFromSDO 로 tx_pdo_map 을
 * 변이하는 동안 Timer ISR 의 SendTPDO encode 가 half-updated map 을 읽지 않도록 게이트.
 * 갱신 중이면 해당 tick TPDO 송신 스킵(다음 tick 최신 map 으로 송신). */
static volatile bool s_pdo_map_updating = false;

/* ===== PDO 메타데이터 (OD에 등록됨) ===== */
static XM_Drv_PdoMetadata_t s_pdo_metadata = {0};  /**< PDO Metadata: Timestamp (24-bit) + Valid Mask (8-bit), 4B total */

/* ===== Communication Profile OD 변수 (CiA 301 Mandatory) ===== */
static uint32_t s_device_type    = 0x00000008;   /**< 0x1000: Device Type — IMU Hub (Rule 33 §0) */
static uint8_t  s_error_register = 0;            /**< 0x1001: Error Register */
static uint16_t s_hb_time_ms     = 1000;         /**< 0x1017: Producer Heartbeat Time (ms) */
static uint8_t  s_identity_count = 4;            /**< 0x1018:0 Number of entries */
static uint32_t s_vendor_id      = 0x414E474C;   /**< 0x1018:1 Vendor ID ("ANGL") */
static uint32_t s_product_code   = 0x00040001;   /**< 0x1018:2 Product Code (Sensor 0x0004, IMU Hub 0x0001) */
static uint32_t s_revision_num   = 0x00050000;   /**< 0x1018:3 Revision Number (V5.0) */
static uint32_t s_serial_num     = 0x00000001;   /**< 0x1018:4 Serial Number */

/* 
 * ===== Object Dictionary 정의 ===== 
 * 
 * [CANopen 표준 OD 영역 구분 (CiA 301)]
 * ┌────────────────────────────────────────────────────────┐
 * │ 0x0000 ~ 0x0FFF: Data Type Definitions (표준)          │
 * │ 0x1000 ~ 0x1FFF: Communication Profile (표준)          │
 * │ 0x2000 ~ 0x5FFF: Manufacturer Specific (제조사 정의) ✅ │
 * │ 0x6000 ~ 0x9FFF: Device Profile (CiA 401, 402 등)      │
 * │ 0xA000 ~ 0xFFFF: Reserved                              │
 * └────────────────────────────────────────────────────────┘
 * 
 * [중요] CANopen 메시지 타입별 처리
 * ┌─────────────────────────────────────────────────────────┐
 * │ NMT, Boot-up, Heartbeat → 별도 CAN ID (SDO 아님)        │
 * │ - CAN ID 0x000: NMT Command                             │
 * │ - CAN ID 0x700 + Node ID: Boot-up, Heartbeat           │
 * │ → AGR_NMT_ProcessMessage()가 처리 ✅                     │
 * │                                                         │
 * │ SDO, PDO → OD Entry를 통해 처리                          │
 * │ - CAN ID 0x600 + Node ID: SDO Request                  │
 * │ - CAN ID 0x180/0x280 + Node ID: PDO                    │
 * │ → AGR_DOP_ProcessRxMessage()가 처리 ✅                   │
 * └─────────────────────────────────────────────────────────┘
 */
static const AGR_OD_Entry_t s_od_entries[] = {
    // Index,   SubIdx, Type,            Size, Access,         DataPtr,                      WriteCb
    
    /* ===== 0x1000 ~ 0x1FFF: Communication Profile (CiA 301 Mandatory) ===== */
    { 0x1000,   0x00,   AGR_TYPE_UINT32, 4,    AGR_ACCESS_RO, &s_device_type,    NULL },  /* Device Type */
    { 0x1001,   0x00,   AGR_TYPE_UINT8,  1,    AGR_ACCESS_RO, &s_error_register, NULL },  /* Error Register */
    { 0x1017,   0x00,   AGR_TYPE_UINT16, 2,    AGR_ACCESS_RW, &s_hb_time_ms,     NULL },  /* Producer Heartbeat Time (ms) */
    { 0x1018,   0x00,   AGR_TYPE_UINT8,  1,    AGR_ACCESS_RO, &s_identity_count, NULL },  /* Identity: Number of entries */
    { 0x1018,   0x01,   AGR_TYPE_UINT32, 4,    AGR_ACCESS_RO, &s_vendor_id,      NULL },  /* Identity: Vendor ID */
    { 0x1018,   0x02,   AGR_TYPE_UINT32, 4,    AGR_ACCESS_RO, &s_product_code,   NULL },  /* Identity: Product Code */
    { 0x1018,   0x03,   AGR_TYPE_UINT32, 4,    AGR_ACCESS_RO, &s_revision_num,   NULL },  /* Identity: Revision Number */
    { 0x1018,   0x04,   AGR_TYPE_UINT32, 4,    AGR_ACCESS_RO, &s_serial_num,     NULL },  /* Identity: Serial Number */
    
    /* TPDO Mapping Parameter (CiA 301 표준: 0x1A00~0x1A01) */
    { 0x1A00,   0x00,   AGR_TYPE_BLOB,   64,   AGR_ACCESS_WO, s_pdo_mapping_a,               _OnPdoMappingA_Set },  /* TPDO1 Mapping (Group A: IMU 0,1,2) */
    { 0x1A01,   0x00,   AGR_TYPE_BLOB,   64,   AGR_ACCESS_WO, s_pdo_mapping_b,               _OnPdoMappingB_Set },  /* TPDO2 Mapping (Group B: IMU 3,4,5) */
    
    /* ===== 0x2000 ~ 0x5FFF: Manufacturer Specific (Angel Robotics) ===== */
    
    /* 센서 상태 (Read Only) */
    { 0x2000,   0x00,   AGR_TYPE_UINT8,  1,    AGR_ACCESS_RO, &s_imu_connected_mask, NULL },  /* IMU Connected Mask (bit0~5: IMU0~5) */
    
    /* ========== PDO Metadata (Index 0x3000) ========== */
    { 0x3000, 0x00, AGR_TYPE_BLOB, 4, AGR_ACCESS_RO, &s_pdo_metadata, NULL },  /* Metadata: Timestamp(3B) + Valid Mask(1B) */
    
    /* ===== Tx: IMU Hub → XM10 (상태 데이터 - SDO) ===== */
    { 0x3100,   0x00,   AGR_TYPE_UINT8,  1,    AGR_ACCESS_RO, &s_imu_hub_tx_data.calibration_status, NULL },
    { 0x3101,   0x00,   AGR_TYPE_UINT16, 2,    AGR_ACCESS_RO, &s_imu_hub_tx_data.error_code,         NULL },
    { 0x3102,   0x00,   AGR_TYPE_BLOB,   6,    AGR_ACCESS_RO, &s_imu_hub_tx_data.sensor_health,      NULL },
    
    /* ===== Tx: IMU Hub → XM10 (IMU 데이터 - PDO/SDO) ===== */
    /* 
     * Object Dictionary 구조:
     * 
     * ========== PDO Metadata (Index 0x3000) ==========
     * Index 0x3000: PDO Metadata (4 bytes, BLOB)
     *   SubIndex 0x00: Timestamp (3B) + Valid Mask (1B)
     *     - Byte 0-2: Frame Timestamp (24-bit, ms)
     *     - Byte 3:   IMU Valid Mask (bit0~5 = IMU 0~5)
     * 
     * ========== IMU Data (Index 0x6000~0x6005) ==========
     * Index: 0x6000~0x6005 (IMU 0~5)
     * 
     * SubIndex (개별 항목, int16, 2 bytes):
     *   0x00~0x03: Quaternion (q[0~3])
     *   0x10~0x12: Euler (rpy[0~2])
     *   0x20~0x22: Accel (a[0~2])
     *   0x30~0x32: Gyro (g[0~2])
     *   0x40~0x42: Mag (m[0~2])
     * 
     * SubIndex (뭉탱이, BLOB):
     *   0x50: q[4] (8B)
     *   0x51: rpy[3] (6B)
     *   0x52: a[3] (6B)
     *   0x53: g[3] (6B)
     *   0x54: m[3] (6B)
     * 
     * SubIndex (조합, BLOB):
     *   0x60: {q,a,g} (20B) - 기본 세트
     *   0x61: {q,a,g,m} (26B)
     *   0x62: {q,rpy,a,g,m} (32B) - 전체
     */
    
    /* ========== IMU 0 (Index 0x6000) ========== */
    /* 개별 항목 */
    { 0x6000, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q[0],    NULL },
    { 0x6000, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q[1],    NULL },
    { 0x6000, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q[2],    NULL },
    { 0x6000, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q[3],    NULL },
    // { 0x6000, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].rpy[0],  NULL },
    // { 0x6000, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].rpy[1],  NULL },
    // { 0x6000, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].rpy[2],  NULL },
    { 0x6000, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].a[0],    NULL },
    { 0x6000, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].a[1],    NULL },
    { 0x6000, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].a[2],    NULL },
    { 0x6000, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].g[0],    NULL },
    { 0x6000, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].g[1],    NULL },
    { 0x6000, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].g[2],    NULL },
    // { 0x6000, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].m[0],    NULL },
    // { 0x6000, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].m[1],    NULL },
    // { 0x6000, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].m[2],    NULL },
    /* 뭉탱이 */
    { 0x6000, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q,       NULL },
    // { 0x6000, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].rpy,     NULL },
    { 0x6000, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].a,       NULL },
    { 0x6000, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].g,       NULL },
    // { 0x6000, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].m,       NULL },
    /* 조합 */
    { 0x6000, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q,       NULL },  // {q,a,g}
    /* [Snapshot/HD/IMU_Resolution_For_Swiss] float32 원본 (40B) — 다운스케일 없음.
     * XM 이 TPDO1 에 [Metadata 4B + 본 40B] 로 매핑. 단일 IMU(첫 연결) 전용. */
    { 0x6000, 0x70, AGR_TYPE_BLOB, 40, AGR_ACCESS_RO, &s_imu_float_slot,                 NULL },  // {q,a,g} float32
    /* [R5] 0x61(26B {q,a,g,m})·0x62(32B {q,rpy,a,g,m}) 주석처리 — 구조체는 20B(q,a,g)만
     * 보유(mag/rpy 미사용, 형제 개별항목 0x10-12/0x40-42/0x51/0x54 와 동일). XM 이 SDO 로
     * 이 항목을 매핑하면 20B 구조체 너머 over-read 발생 → 차단. 추후 mag/rpy 구현 시 일괄 부활. */
    // { 0x6000, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q,       NULL },  // {q,a,g,m}
    // { 0x6000, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0].q,       NULL },  // {q,rpy,a,g,m}
    
    /* ========== IMU 1 (Index 0x6001) ========== */
    { 0x6001, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q[0],    NULL },
    { 0x6001, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q[1],    NULL },
    { 0x6001, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q[2],    NULL },
    { 0x6001, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q[3],    NULL },
    // { 0x6001, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].rpy[0],  NULL },
    // { 0x6001, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].rpy[1],  NULL },
    // { 0x6001, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].rpy[2],  NULL },
    { 0x6001, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].a[0],    NULL },
    { 0x6001, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].a[1],    NULL },
    { 0x6001, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].a[2],    NULL },
    { 0x6001, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].g[0],    NULL },
    { 0x6001, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].g[1],    NULL },
    { 0x6001, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].g[2],    NULL },
    // { 0x6001, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].m[0],    NULL },
    // { 0x6001, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].m[1],    NULL },
    // { 0x6001, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].m[2],    NULL },
    { 0x6001, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q,       NULL },
    // { 0x6001, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].rpy,     NULL },
    { 0x6001, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].a,       NULL },
    { 0x6001, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].g,       NULL },
    // { 0x6001, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].m,       NULL },
    { 0x6001, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q,       NULL },
    // { 0x6001, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q,       NULL },  // [R5] over-read 방지
    // { 0x6001, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1].q,       NULL },  // [R5] over-read 방지
    
    /* ========== IMU 2 (Index 0x6002) ========== */
    { 0x6002, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q[0],    NULL },
    { 0x6002, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q[1],    NULL },
    { 0x6002, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q[2],    NULL },
    { 0x6002, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q[3],    NULL },
    // { 0x6002, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].rpy[0],  NULL },
    // { 0x6002, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].rpy[1],  NULL },
    // { 0x6002, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].rpy[2],  NULL },
    { 0x6002, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].a[0],    NULL },
    { 0x6002, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].a[1],    NULL },
    { 0x6002, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].a[2],    NULL },
    { 0x6002, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].g[0],    NULL },
    { 0x6002, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].g[1],    NULL },
    { 0x6002, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].g[2],    NULL },
    // { 0x6002, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].m[0],    NULL },
    // { 0x6002, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].m[1],    NULL },
    // { 0x6002, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].m[2],    NULL },
    { 0x6002, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q,       NULL },
    // { 0x6002, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].rpy,     NULL },
    { 0x6002, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].a,       NULL },
    { 0x6002, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].g,       NULL },
    // { 0x6002, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].m,       NULL },
    { 0x6002, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q,       NULL },
    // { 0x6002, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q,       NULL },  // [R5] over-read 방지
    // { 0x6002, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2].q,       NULL },  // [R5] over-read 방지
    
    /* ========== IMU 3 (Index 0x6003) ========== */
    { 0x6003, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q[0],    NULL },
    { 0x6003, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q[1],    NULL },
    { 0x6003, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q[2],    NULL },
    { 0x6003, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q[3],    NULL },
    // { 0x6003, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].rpy[0],  NULL },
    // { 0x6003, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].rpy[1],  NULL },
    // { 0x6003, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].rpy[2],  NULL },
    { 0x6003, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].a[0],    NULL },
    { 0x6003, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].a[1],    NULL },
    { 0x6003, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].a[2],    NULL },
    { 0x6003, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].g[0],    NULL },
    { 0x6003, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].g[1],    NULL },
    { 0x6003, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].g[2],    NULL },
    // { 0x6003, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].m[0],    NULL },
    // { 0x6003, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].m[1],    NULL },
    // { 0x6003, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].m[2],    NULL },
    { 0x6003, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q,       NULL },
    // { 0x6003, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].rpy,     NULL },
    { 0x6003, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].a,       NULL },
    { 0x6003, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].g,       NULL },
    // { 0x6003, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].m,       NULL },
    { 0x6003, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q,       NULL },
    // { 0x6003, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q,       NULL },  // [R5] over-read 방지
    // { 0x6003, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[3].q,       NULL },  // [R5] over-read 방지
    
    /* ========== IMU 4 (Index 0x6004) ========== */
    { 0x6004, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q[0],    NULL },
    { 0x6004, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q[1],    NULL },
    { 0x6004, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q[2],    NULL },
    { 0x6004, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q[3],    NULL },
    // { 0x6004, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].rpy[0],  NULL },
    // { 0x6004, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].rpy[1],  NULL },
    // { 0x6004, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].rpy[2],  NULL },
    { 0x6004, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].a[0],    NULL },
    { 0x6004, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].a[1],    NULL },
    { 0x6004, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].a[2],    NULL },
    { 0x6004, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].g[0],    NULL },
    { 0x6004, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].g[1],    NULL },
    { 0x6004, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].g[2],    NULL },
    // { 0x6004, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].m[0],    NULL },
    // { 0x6004, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].m[1],    NULL },
    // { 0x6004, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].m[2],    NULL },
    { 0x6004, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q,       NULL },
    // { 0x6004, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].rpy,     NULL },
    { 0x6004, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].a,       NULL },
    { 0x6004, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].g,       NULL },
    // { 0x6004, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].m,       NULL },
    { 0x6004, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q,       NULL },
    // { 0x6004, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q,       NULL },  // [R5] over-read 방지
    // { 0x6004, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[4].q,       NULL },  // [R5] over-read 방지
    
    /* ========== IMU 5 (Index 0x6005) ========== */
    { 0x6005, 0x00, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q[0],    NULL },
    { 0x6005, 0x01, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q[1],    NULL },
    { 0x6005, 0x02, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q[2],    NULL },
    { 0x6005, 0x03, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q[3],    NULL },
    // { 0x6005, 0x10, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].rpy[0],  NULL },
    // { 0x6005, 0x11, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].rpy[1],  NULL },
    // { 0x6005, 0x12, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].rpy[2],  NULL },
    { 0x6005, 0x20, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].a[0],    NULL },
    { 0x6005, 0x21, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].a[1],    NULL },
    { 0x6005, 0x22, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].a[2],    NULL },
    { 0x6005, 0x30, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].g[0],    NULL },
    { 0x6005, 0x31, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].g[1],    NULL },
    { 0x6005, 0x32, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].g[2],    NULL },
    // { 0x6005, 0x40, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].m[0],    NULL },
    // { 0x6005, 0x41, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].m[1],    NULL },
    // { 0x6005, 0x42, AGR_TYPE_INT16, 2, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].m[2],    NULL },
    { 0x6005, 0x50, AGR_TYPE_BLOB,  8, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q,       NULL },
    // { 0x6005, 0x51, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].rpy,     NULL },
    { 0x6005, 0x52, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].a,       NULL },
    { 0x6005, 0x53, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].g,       NULL },
    // { 0x6005, 0x54, AGR_TYPE_BLOB,  6, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].m,       NULL },
    { 0x6005, 0x60, AGR_TYPE_BLOB, 20, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q,       NULL },
    // { 0x6005, 0x61, AGR_TYPE_BLOB, 26, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q,       NULL },  // [R5] over-read 방지
    // { 0x6005, 0x62, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[5].q,       NULL },  // [R5] over-read 방지

    /* ========== Control Loop Tick (1ms counter, seq gap detection) ========== */
    /* Index: 0x6050, TPDO mapping 가능 (XM 수신 측에서 gap 감지용) */
    { 0x6050, 0x00, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_imu_hub_tx_data.ctrl_tick_ms, NULL },

    /* ========== EMCY Test Trigger (dummy injection via SDO Write) ==========
     * OD 0x7F02.01 write (3 bytes: err_code_lo, err_code_hi, err_reg) →
     * _OnEmcyTestTrigger() → XM_Drv_SendEmcy(). 실제 EMCY 트리거 조건은 TBD.
     */
    { 0x7F02, 0x01, AGR_TYPE_BLOB, 3, AGR_ACCESS_WO, s_emcy_test_payload, _OnEmcyTestTrigger },
};

static const AGR_OD_Table_t s_od_table = {
    .entries = s_od_entries,
    .entry_count = AGR_OD_ENTRY_COUNT(s_od_entries)
};

/* ===== DOP Context ===== */
static AGR_DOP_Ctx_t s_dop_ctx;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * ===== [Layer 3] AGR_PnP_Slave 콜백 (V4.0 - Device Driver에서 구현) =====
 * 
 * [Slave 역할]
 * - Master(XM10) 연결/끊김 이벤트 처리
 * - 나의 NMT 상태 변경 알림
 * - NMT 명령 수신 처리
 * 
 * [콜백 호출 흐름 (V4.0)]
 * Master Heartbeat 수신 → AGR_PnP_Slave_ProcessMessage()
 *   → on_master_online 콜백 (Device Driver) ✅
 * Master Timeout → AGR_PnP_Slave_RunPeriodic()
 *   → on_master_offline 콜백 (Device Driver) ✅
 * NMT 명령 수신 → AGR_PnP_Slave_ProcessMessage()
 *   → on_nmt_command + on_my_state_changed 콜백 (Device Driver) ✅
 */
static void _OnMasterOnline(void);               /* Master 연결 (Heartbeat 수신) */
static void _OnMasterOffline(void);              /* Master 끊김 (Heartbeat Timeout) */
static void _OnMyStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);  /* 내 NMT 상태 변경 */
static void _OnNmtCommand(AGR_NMT_Cmd_t cmd);   /* NMT 명령 수신 */

/**
 * ===== [Layer 2] DOP 콜백 (V3: Core SDO가 OD 기반 SDO를 자동 처리) =====
 * 
 * [V3 변경]
 * - _OnSdoRequest 콜백 제거: AGR_SDO_ProcessRequest()가 OD 검색/읽기/쓰기/WriteCb 자동 처리
 * - _OnPdoReceived 콜백 유지: PDO 수신 이벤트 처리
 */
static void _OnPdoReceived(uint32_t cob_id, const uint8_t* data, uint8_t len);

static int _Drv_TxFunc_Queue(uint32_t can_id, const uint8_t* data, uint8_t len);
static int _TxFunc_DeferToRing(uint32_t can_id, const uint8_t* data, uint8_t len);  /* main → tx-defer-ring */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief XM Driver 초기화 (CANopen 표준 기반)
 * @param fdcan_id FDCAN Instance ID (Software Queue 사용)
 * @param slave_pnp System Layer 에서 생성한 PnP Slave 인스턴스 (주입)
 * @return 0: 성공, <0: 실패
 *
 * @details
 * [초기화 순서]
 * 1. AGR_CANFD(DOP V3 Transport) 초기화 — Tx 는 내부 _Drv_TxFunc_Queue 로 일원화
 *    (BareMetal single-writer: IPSR 분기 라우팅이 내부 전용이라 tx_func 주입 안 받음)
 * 2. PnP Slave 포인터 저장 (NMT/Boot-up/Heartbeat 위임)
 */
int XM_Drv_Init(IOIF_FDCANx_t fdcan_id, AGR_PnP_Slave_t* slave_pnp)
{
    /* ===== 파라미터 검증 ===== */
    if (fdcan_id == IOIF_FDCAN_INVALID_ID || slave_pnp == NULL) {
        return -1;
    }

    /* FDCAN ID 저장 (Software Queue 사용) */
    s_fdcan_id = fdcan_id;
    
    /* ===== PnP Slave 포인터 저장 (V4.0: System Layer에서 주입) =====
     * [변경 이유]
     * - 기존: 내부에서 AGR_PnP_Inst_t + AGR_NMT_Inst_t 생성 및 초기화
     * - 변경: System Layer에서 초기화된 AGR_PnP_Slave_t* 포인터만 저장
     * - 효과: PnP 라이프사이클을 System이 관리, Driver는 콜백만 등록
     */
    s_slave_pnp = slave_pnp;

    /* 프레임 메타데이터 초기화 */
    memset(&s_pdo_metadata, 0, sizeof(XM_Drv_PdoMetadata_t));
    
    /* ✅ Rx 데이터 초기화 (XM10 → IMU Hub) - Lock-free Double Buffer */
    memset(&s_xm_double_buf, 0, sizeof(XM_DoubleBuffer_t));
    s_xm_double_buf.write_idx = 0;
    s_xm_double_buf.read_idx = 1;
    s_imu_connected_mask = 0;
    
    /* Tx 데이터 초기화 (IMU Hub → XM10) */
    memset(&s_imu_hub_tx_data, 0, sizeof(XM_TxData_t));
    s_imu_hub_tx_data.calibration_status = 0;
    s_imu_hub_tx_data.error_code = 0;
    
    /* ===================================================================
     * [Layer 1] DOP V3 CAN-FD Transport 초기화
     * ===================================================================
     * [역할] SDO/PDO 처리 (Core + CAN-FD Transport 직접 사용)
     * [V5.0 변경] Facade 미사용, AGR_CANFD_ API 직접 호출
     */
    int ret = AGR_CANFD_Init(&s_dop_ctx, &s_od_table, AGR_NODE_ID_IMU_HUB, _Drv_TxFunc_Queue);
    if (ret != 0) {
        return ret;
    }
    
    AGR_CANFD_SetTargetNodeId(&s_dop_ctx, AGR_NODE_ID_XM);

    /* EMCY 송신 활성화 (수신 콜백 NULL — IMU 는 EMCY 송신만 담당) */
    AGR_CANFD_EnableEmergency(&s_dop_ctx, NULL);

    /* V3: Core SDO가 OD 기반 SDO를 자동 처리 (on_sdo_request 불필요) */
    s_dop_ctx.on_pdo_received = _OnPdoReceived;
    
    /* ===================================================================
     * [Layer 2] AGR_PnP_Slave 콜백 등록 (V4.0)
     * ===================================================================
     * [변경 이유]
     * - 기존: AGR_PnP_Config_t에 콜백 직접 설정 (Init에서 일괄)
     * - 변경: AGR_PnP_Slave_SetCallbacks()로 명시적 등록
     * - 효과: 콜백 시그니처 명확화, Master/Slave 혼동 방지
     *
     * [1:1 콜백 대조 - V3.0 → V4.0]
     * - _PnP_OnConnected(node_id)    → _OnMasterOnline(void)
     * - _PnP_OnDisconnected(node_id) → _OnMasterOffline(void)
     * - _PnP_OnNmtChange(node_id, old, new) → _OnMyStateChanged(old, new)
     * - _PnP_OnError(node_id, error) → 제거 (PnP Slave 내부 처리)
     * - (없음)                       → _OnNmtCommand(cmd) ✅ 신규
     */
    AGR_PnP_Slave_SetCallbacks(s_slave_pnp, &(AGR_PnP_Slave_Callbacks_t){
        .on_master_online    = _OnMasterOnline,      /* Master Heartbeat 수신 → 연결 */
        .on_master_offline   = _OnMasterOffline,     /* Master Timeout → 끊김 */
        .on_my_state_changed = _OnMyStateChanged,    /* 나의 NMT 상태 변경 알림 */
        .on_nmt_command      = _OnNmtCommand,        /* NMT 명령 수신 (START 등) */
    });
    
    /* ===== Boot-up 전송은 AGR_PnP_Slave_RunPeriodic()에서 자동 처리 =====
     * [.cursorrules 준수] Init에서 Boot-up 전송 금지
     * - BOOT_UP 상태에서 1초마다 자동 재전송 (Master 대기)
     * - FDCAN 시작 후 첫 RunPeriodic() 호출 시 자동 전송
     */
    
    return 0;
}

/**
 * @brief XM10 연결 상태 확인 (V4.0: PnP Slave API 사용)
 * @return true: 연결됨 (Slave가 OPERATIONAL + Master 온라인)
 * 
 * [V4.0 변경]
 * - 기존: s_imu_hub_nmt.state + AGR_PnP_IsMasterConnected()
 * - 변경: AGR_PnP_Slave_GetMyState() + AGR_PnP_Slave_IsMyMasterOnline()
 * - 효과: 단일 API 사용, 구현 상세 은닉
 */
bool XM_Drv_IsConnected(void)
{
    if (s_slave_pnp == NULL) return false;
    
    /* ✅ 두 조건 모두 충족해야 연결됨 */
    /* 1. Slave 자신이 OPERATIONAL (NMT START 받음) */
    /* 2. Master 온라인 (Heartbeat 수신 중) */
    return (AGR_PnP_Slave_GetMyState(s_slave_pnp) == AGR_NMT_OPERATIONAL) &&
           AGR_PnP_Slave_IsMyMasterOnline(s_slave_pnp);
}

/**
 * @brief XM10 NMT 상태 조회 (V4.0: PnP Slave API 사용)
 * @return NMT 상태
 * 
 * [V4.0 변경]
 * - 기존: AGR_PnP_GetMyState(&s_imu_hub_pnp_inst)
 * - 변경: AGR_PnP_Slave_GetMyState(s_slave_pnp)
 */
AGR_NMT_State_t XM_Drv_GetNmtState(void)
{
    if (s_slave_pnp == NULL) return AGR_NMT_STOPPED;
    
    return AGR_PnP_Slave_GetMyState(s_slave_pnp);
}

/**
 * @brief SYNC pending 플래그 소비 (TIM16 Phase-Lock용)
 * @return 직전 SYNC 수신 여부. true 반환 후 플래그 자동 클리어.
 */
bool XM_Drv_ConsumeSyncPending(void)
{
    if (s_sync_pending) {
        s_sync_pending = false;
        return true;
    }
    return false;
}

uint32_t XM_Drv_GetSyncCount(void)
{
    return s_sync_counter_rx;
}

/**
 * @brief 주기 실행 (PnP Slave 위임)
 * @details ImuSysService_Run()(TIM16 ISR 의 500ms 분기)가 호출. 내부에서 Master
 *   Heartbeat Timeout 체크, BOOT_UP 상태 시 Bootup 재전송, PRE_OP/OPERATIONAL 시
 *   Heartbeat 자동 전송. LED 는 건드리지 않음(System/Board Layer 단독 관리 — 아래 주석).
 */
void XM_Drv_RunPeriodic(void)
{
    if (s_slave_pnp == NULL) return;

    /* 1. AGR_PnP Slave 주기 처리 (Master Timeout + Boot-up/Heartbeat 자동) */
    AGR_PnP_Slave_RunPeriodic(s_slave_pnp);

    /* 2. LED 는 System/Board Layer (imu_sys_service._UpdateLedState, 100 ms)
     *    가 단독 관리. Device Layer 에서 LED 를 건드리지 않는다.
     *    과거 여기 있던 IO_SetLinkState((IO_LinkState_t)NMT_state) 는
     *    AGR_NMT_OPERATIONAL(0x05) → IO_LINK_STATE_ERROR(0x05) 로 오매핑되어
     *    LED 가 500ms 주기 ERROR ↔ OPERATIONAL 플리킹을 유발했음. */
}

/**
 * @brief CAN 메시지 처리 (V4.0: PnP Slave API로 NMT/HB 위임)
 * @details 
 * ISR (canfd_rx_handler)에서 직접 호출됩니다.
 * 
 * [V4.0 변경]
 * - 기존: 0x000 → AGR_NMT_ProcessMessage(&s_imu_hub_nmt, ...) 직접 호출
 *         0x700 → AGR_NMT_ProcessMessage(&s_imu_hub_pnp_inst.config.nmt, ...) 직접 호출
 * - 변경: 0x000, 0x700 → AGR_PnP_Slave_ProcessMessage(s_slave_pnp, ...) 위임
 * - 효과: NMT Command/Heartbeat 처리를 PnP Slave가 통합 관리
 *         → on_nmt_command, on_master_online/offline 콜백 자동 호출
 * 
 * [메시지 라우팅]
 * 1. NMT Command (0x000) → PnP Slave (나의 NMT 상태 변경)
 * 2. Heartbeat (0x700)   → PnP Slave (Master 추적)
 * 3. SDO (0x600)         → AGR_CANFD (Core SDO로 OD 접근)
 * 4. PDO (0x180/0x280)   → AGR_CANFD (Core PDO로 데이터 업데이트)
 * 
 * @param can_id CAN ID (11-bit)
 * @param data   데이터 버퍼
 * @param len    데이터 길이
 */
void XM_Drv_ProcessCANMessage(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (data == NULL) {
        return;
    }

    /* ===== CAN ID 기반 분기 (CANopen 표준) ===== */

    /* 0. SYNC (0x080) → Flag Set (ISR-Safe, BareMetal volatile write) */
    if (can_id == AGR_CAN_ID_SYNC) {
        s_sync_pending = true;
        s_sync_counter_rx++;
        return;
    }

    /* 1. NMT Command (CAN ID 0x000) → PnP Slave가 처리
     * [V4.0 변경]
     * - 기존: AGR_NMT_ProcessMessage(&s_imu_hub_nmt, ...) 직접
     * - 변경: AGR_PnP_Slave_ProcessMessage() → 내부에서 my_nmt 자동 업데이트
     *         + on_nmt_command 콜백 호출 + on_my_state_changed 콜백 호출
     */
    if (can_id == 0x000) {
        if (s_slave_pnp != NULL) {
            AGR_PnP_Slave_ProcessMessage(s_slave_pnp, can_id, data, len);
        }
        return;
    }
    
    /* 2. Boot-up / Heartbeat (CAN ID 0x700 + Node ID) → PnP Slave가 처리
     * [V4.0 변경]
     * - 기존: Master Node ID 필터링 후 AGR_NMT_ProcessMessage() 직접
     * - 변경: AGR_PnP_Slave_ProcessMessage() → 내부에서 Master 필터링 + 추적
     *         + on_master_online 콜백 호출 (첫 수신 시)
     */
    uint16_t fnc_code = can_id & 0x780;  /* Function Code 추출 */
    if (fnc_code == 0x700) {
        if (s_slave_pnp != NULL) {
            AGR_PnP_Slave_ProcessMessage(s_slave_pnp, can_id, data, len);
        }
        return;
    }
    
    /* 3. SDO Request (0x600 + Node ID) → main loop 처리로 defer
     * (SM Safety Checklist "SDO: ISR에서 플래그만, main loop에서 처리".
     *  decode + OD write(PDO 매핑 재구성) + 응답 Tx 가 ISR 밖으로 이동). EMG 정합. */
    if (fnc_code == 0x600) {
        uint8_t head = s_sdo_defer_head;
        uint8_t next = (uint8_t)((head + 1U) % XM_SDO_DEFER_SLOTS);
        if (next == s_sdo_defer_tail) {
            s_sdo_defer_drop++;   /* ring full — drop-newest (XM 재시도 존재) */
            return;
        }
        XmSdoDeferFrame_t* slot = &s_sdo_defer_ring[head];
        slot->can_id = can_id;
        slot->len    = (len > 64U) ? 64U : len;
        memcpy(slot->data, data, slot->len);
        __DMB();                  /* 데이터 가시화 후 인덱스 공개 (producer 배리어) */
        s_sdo_defer_head = next;
        return;
    }
    
    /* 4. PDO (CAN ID 0x180/0x280 + Node ID) */
    if (fnc_code == 0x180 || fnc_code == 0x280) {
        AGR_CANFD_ProcessRxMessage(&s_dop_ctx, can_id, data, len);
        return;
    }
    
    /* 5. Unknown Message (로그 또는 무시) */
}

/**
 * @brief Deferred SDO 처리 (main() while(1) thread-level 전용)
 * @details FDCAN Rx ISR 이 ring 에 적재한 0x600 프레임을 decode + OD write(PDO 매핑
 *          재구성) + 응답 Tx 한다. SDO 응답 Tx 는 _Drv_TxFunc_Queue 의 IPSR==0(thread)
 *          분기를 타 tx-defer-ring 에 적재되고, Timer ISR(XM_Drv_DrainTxDeferRing)이
 *          단일 디스패치한다 → BareMetal FDCAN Tx single-writer 유지.
 * @note main() while(1) 에서 호출. ISR 에서 호출 금지.
 */
void XM_Drv_ProcessDeferredSdo(void)
{
    while (s_sdo_defer_tail != s_sdo_defer_head) {
        XmSdoDeferFrame_t* slot = &s_sdo_defer_ring[s_sdo_defer_tail];
        __DMB();   /* 인덱스 확인 후 데이터 읽기 (consumer 배리어) */
        AGR_CANFD_ProcessRxMessage(&s_dop_ctx, slot->can_id, slot->data, slot->len);
        __DMB();
        s_sdo_defer_tail = (uint8_t)((s_sdo_defer_tail + 1U) % XM_SDO_DEFER_SLOTS);
    }
}

/**
 * @brief SDO defer ring full-drop 누적 횟수 (진단)
 */
uint32_t XM_Drv_GetSdoDeferDropCount(void)
{
    return s_sdo_defer_drop;
}

/**
 * @brief Tx defer ring drain — Timer ISR(단일 송신자) 전용
 * @details main() 이 적재한 비-PDO 프레임(SDO 응답)을 HW 직접 송신한다.
 *          PDO-first: 호출자(_FlushOutputs)가 TPDO direct 송신 후 호출.
 *          cap=2/tick (TPDO 슬롯·ISR 예산 보호). HW full 시 ring 잔류(tail 미advance)
 *          → 다음 tick 재시도(leave-in-place, 비-PDO 무손실).
 * @note Timer ISR 에서만 호출 (IPSR != 0). HW Tx single-writer 일원화의 핵심.
 */
void XM_Drv_DrainTxDeferRing(void)
{
    if (s_fdcan_id == IOIF_FDCAN_INVALID_ID) {
        return;
    }
    for (uint8_t n = 0U; n < 2U; n++) {   /* cap 2 frame/tick */
        if (s_tx_defer_tail == s_tx_defer_head) {
            break;   /* empty */
        }
        XmSdoDeferFrame_t* slot = &s_tx_defer_ring[s_tx_defer_tail];
        __DMB();   /* 인덱스 확인 후 데이터 읽기 (consumer 배리어) */
        if (IOIF_FDCAN_Transmit(s_fdcan_id, slot->can_id, slot->data, slot->len) != AGRBStatus_OK) {
            break;   /* HW full → tail 미advance, 다음 tick 재시도 (no-loss) */
        }
        __DMB();
        s_tx_defer_tail = (uint8_t)((s_tx_defer_tail + 1U) % XM_TX_DEFER_SLOTS);
    }
}

/**
 * @brief master-online abort 처리 — Timer ISR 전용 (RunLoop 맨 앞)
 * @details _OnMasterOnline(FDCAN RX ISR)이 set 한 flag 를 consume 하여 AbortAllTx 수행.
 *          마스터 연결 시 stuck TX 버퍼 flush 를 Timer ISR(단일 writer)로 일원화.
 *          RunLoop 맨 앞(TPDO 송신 前) 호출 → 새 TPDO 가 abort 되지 않음.
 * @note Timer ISR 에서만 호출.
 */
void XM_Drv_ServiceMasterOnlineAbort(void)
{
    if (s_master_online_abort_pending) {
        s_master_online_abort_pending = false;
        if (s_fdcan_id != IOIF_FDCAN_INVALID_ID) {
            IOIF_FDCAN_AbortAllTx(s_fdcan_id);
        }
    }
}

/* [REMOVED V5.0] XM_Drv_CheckAndClearHeartbeat / XM_Drv_GetXmNmtState / XM_Drv_HandleSDORequest
 * (deprecated, 호출처 0건) — 대체: XM_Drv_IsConnected() / XM_Drv_GetNmtState() / SDO 자동처리
 * (FDCAN RX ISR defer-ring → main() XM_Drv_ProcessDeferredSdo → AGR_SDO_ProcessRequest). */

void XM_Drv_UpdateImuData(uint8_t imu_index, const XM_Drv_ImuData_t* data)
{
    if (imu_index >= XM_DRV_IMU_COUNT || data == NULL) {
        return;
    }
    
    /* IMU 데이터 복사 */
    memcpy(&s_imu_hub_tx_data.imu[imu_index], data, sizeof(XM_Drv_ImuData_t));
}

void XM_Drv_UpdateImuFloat(const XM_Drv_ImuDataF_t* data)
{
    if (data == NULL) {
        return;
    }
    /* [Snapshot] float32 원본 슬롯 복사 (OD 0x6000.0x70, 40B) */
    memcpy(&s_imu_float_slot, data, sizeof(XM_Drv_ImuDataF_t));
}

void XM_Drv_SetFrameTimestamp(uint32_t time_ms)
{
    /* 24-bit timestamp를 3 bytes로 분리 */
    s_pdo_metadata.timestamp_low  = (time_ms >> 0) & 0xFF;
    s_pdo_metadata.timestamp_mid  = (time_ms >> 8) & 0xFF;
    s_pdo_metadata.timestamp_high = (time_ms >> 16) & 0xFF;
}

void XM_Drv_ClearValidMask(void)
{
    s_pdo_metadata.valid_mask = 0;  /* PDO Metadata의 valid_mask 클리어 */
}

int XM_Drv_SendTPDO1(void)
{
    if (s_dop_ctx.tx_func == NULL) {
        return -1;
    }

    /* map 갱신 중(main WriteCb)이면 이번 tick 송신 스킵 — half-updated map encode 방지 */
    if (s_pdo_map_updating) {
        return -3;
    }

    /*
     * [동적 PDO Mapping 사용 - TPDO1]
     * XM10이 PnP 단계에서 설정한 PDO Mapping에 따라 전송합니다.
     * AGR_DOP_EncodeTxPDON()이 s_dop_ctx.tx_pdo_map[0]를 참조하여
     * 필요한 데이터만 선택적으로 인코딩합니다.
     */
    
    /* PDO Payload 동적 구성 (TPDO1) - V3 Core API 직접 사용 */
    uint8_t pdo_buf[64];
    int len = AGR_PDO_Encode(&s_dop_ctx.tx_pdo_map[0], &s_dop_ctx.od,
                             pdo_buf, sizeof(pdo_buf));
    
    if (len <= 0) {
        return -2;
    }
    
    uint32_t can_id = AGR_CANFD_GetTPDOID(AGR_NODE_ID_IMU_HUB, 1);
    return s_dop_ctx.tx_func(can_id, pdo_buf, (uint8_t)len);
}

int XM_Drv_SendTPDO2(void)
{
    if (s_dop_ctx.tx_func == NULL) {
        return -1;
    }

    /* map 갱신 중(main WriteCb)이면 이번 tick 송신 스킵 — half-updated map encode 방지 */
    if (s_pdo_map_updating) {
        return -3;
    }

    /*
     * [동적 PDO Mapping 사용 - TPDO2]
     * XM10이 PnP 단계에서 설정한 PDO Mapping에 따라 전송합니다.
     * AGR_DOP_EncodeTxPDON()이 s_dop_ctx.tx_pdo_map[1]를 참조하여
     * 필요한 데이터만 선택적으로 인코딩합니다.
     */
    
    /* PDO Payload 동적 구성 (TPDO2) - V3 Core API 직접 사용 */
    uint8_t pdo_buf[64];
    int len = AGR_PDO_Encode(&s_dop_ctx.tx_pdo_map[1], &s_dop_ctx.od,
                             pdo_buf, sizeof(pdo_buf));
    
    if (len <= 0) {
        return -2;
    }
    
    uint32_t can_id = AGR_CANFD_GetTPDOID(AGR_NODE_ID_IMU_HUB, 2);
    return s_dop_ctx.tx_func(can_id, pdo_buf, (uint8_t)len);
}

/* [DEPRECATED V4.0] Boot-up/Heartbeat는 AGR_PnP_Slave_RunPeriodic()에서 자동 관리
 * 수동 호출이 필요한 경우: AGR_PnP_Slave_SendBootupNow() / SendHeartbeatNow() 사용
 *
 * 아래 함수들은 하위 호환성을 위해 유지하되, 내부적으로 PnP Slave API를 호출합니다.
 */
int XM_Drv_SendBootup(void)
{
    if (s_slave_pnp == NULL) return -1;
    return AGR_PnP_Slave_SendBootupNow(s_slave_pnp);
}

int XM_Drv_SendHeartbeat(void)
{
    if (s_slave_pnp == NULL) return -1;
    return AGR_PnP_Slave_SendHeartbeatNow(s_slave_pnp);
}

void XM_Drv_SetPDOInhibitTime(uint8_t pdo_num, uint32_t inhibit_time_us)
{
    if (pdo_num >= 1 && pdo_num <= 4) {
        AGR_PDO_SetInhibitTime(&s_dop_ctx.pdo_inhibit[pdo_num - 1], inhibit_time_us);
    }
}

void XM_Drv_DisablePDOInhibit(uint8_t pdo_num)
{
    if (pdo_num >= 1 && pdo_num <= 4) {
        AGR_PDO_DisableInhibit(&s_dop_ctx.pdo_inhibit[pdo_num - 1]);
    }
}

/* [DEPRECATED V4.0] Timeout 체크는 AGR_PnP_Slave_RunPeriodic()에서 자동 처리
 * 하위 호환성을 위해 유지하되, 내부적으로 PnP Slave API가 처리합니다.
 */
void XM_Drv_CheckTimeout(uint32_t current_ms)
{
    (void)current_ms;
    /* V4.0: AGR_PnP_Slave_RunPeriodic()에서 Master Timeout 자동 처리 */
    /* 이 함수를 별도로 호출할 필요 없음 */
}

void XM_Drv_SetImuConnectedMask(uint8_t mask)
{
    s_imu_connected_mask = mask;
    s_pdo_metadata.valid_mask = mask;  /* OD 0x3000.0x00에 반영 (TPDO에 포함됨) */
}

uint8_t XM_Drv_GetImuConnectedMask(void)
{
    return s_imu_connected_mask;
}

/**
 * @brief FDCAN Rx 데이터 가져오기 (Main Control Loop용)
 * @details 
 * ✅ Lock-free Double Buffering으로 안전하게 데이터를 읽어옵니다.
 * 
 * [Bare-metal 최적화]
 * - IRQ Disable 불필요 (Zero Blocking Time)
 * - Data Tearing 완벽 방지
 * - Control Loop Timer ISR 방해 없음
 * 
 * [동작 원리]
 * - ISR: write_idx 버퍼에 쓰고 완료 후 인덱스 스왑
 * - Main: read_idx 버퍼를 읽음 (ISR이 쓰는 버퍼와 독립)
 * - volatile uint8_t: ARM Cortex-M에서 원자적 읽기/쓰기 보장
 */
bool XM_Drv_GetRxData(XM_RxData_t* out_data)
{
    if (out_data == NULL) {
        return false;
    }
    
    /* ✅ Lock-free: read_idx 버퍼를 안전하게 읽음 */
    /* 
     * [중요] volatile 읽기는 atomic (ARM Cortex-M)
     * ISR이 write_idx에 쓰는 동안에도 read_idx는 변하지 않음
     * → Data Tearing 불가능
     */
    uint8_t safe_idx = s_xm_double_buf.read_idx;  /* volatile 읽기 (atomic) */
    
    /* [Zero-Copy 최적화] 직접 버퍼 복사 (IRQ 차단 불필요) */
    *out_data = s_xm_double_buf.buffers[safe_idx];
    
    return true;
}

/**
 * @brief Tx 데이터 가져오기 (디버깅/모니터링용)
 */
bool XM_Drv_GetTxData(XM_TxData_t* out_data)
{
    if (out_data == NULL) {
        return false;
    }
    
    /* Tx 데이터는 Main Loop에서만 업데이트되므로 보호 불필요 */
    memcpy(out_data, &s_imu_hub_tx_data, sizeof(XM_TxData_t));
    
    return true;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief XM10 연결 완료 콜백 (AGR_PnP)
 */
/**
 * @brief Master 온라인 콜백 (V4.0: AGR_PnP_Slave)
 * @details Master Heartbeat 첫 수신 또는 재연결 시 호출됩니다.
 * 
 * [1:1 대조] V3.0 _PnP_OnConnected(uint8_t master_node_id)
 * - LED 제어는 Application Layer (IO_Manager)에서 NMT 상태 기반으로 관리
 */
static void _OnMasterOnline(void)
{
    /* 부팅 시 ACK 없이 stuck된 TX 버퍼 전체 취소(HW FIFO flush)가 필요하다
     * (AutoRetransmission=ENABLE 에서 Master 없이 전송된 프레임이 영원히 재시도하여
     * TX FIFO 점유). 단 본 콜백은 FDCAN RX ISR 컨텍스트라 여기서 AbortAllTx(HW Tx
     * 조작)를 직접 호출하면 Timer ISR 의 TPDO 송신과 경합(BareMetal TX_LOCK no-op).
     * → flag 만 set, Timer ISR(XM_Drv_ServiceMasterOnlineAbort)이 RunLoop 맨 앞
     *   (TPDO 송신 前)에서 단일 수행 = HW Tx single-writer 유지 + 새 TPDO 무손실. */
    s_master_online_abort_pending = true;
}

/**
 * @brief Master 오프라인 콜백 (V4.0: AGR_PnP_Slave)
 * @details Master Heartbeat Timeout 시 호출됩니다.
 * 
 * [1:1 대조] V3.0 _PnP_OnDisconnected(uint8_t master_node_id)
 * - LED 제어는 Application Layer (IO_Manager)에서 NMT 상태 기반으로 관리
 */
static void _OnMasterOffline(void)
{
    /* LED 제어는 Application Layer (IO_Manager)에서 NMT 상태 기반으로 처리 */
}

/**
 * @brief 나의 NMT 상태 변경 콜백 (V4.0: AGR_PnP_Slave)
 * @param old_state 이전 NMT 상태
 * @param new_state 새로운 NMT 상태
 * 
 * [1:1 대조] V3.0 _PnP_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old, AGR_NMT_State_t new)
 * - 기존 동작: 필요시 상태 변경 로그 (빈 함수)
 * - V4.0 동작: 동일 (확장 가능)
 * 
 * [참고] V3.0에서는 Master의 NMT 변경을 알렸지만,
 *        V4.0에서는 "나의" NMT 변경을 알립니다 (더 직관적).
 *        Master 상태는 on_master_online/offline으로 관리.
 */
static void _OnMyStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state)
{
    (void)old_state;
    (void)new_state;
    
    /* 필요시 상태 변경 로그 또는 추가 처리 */
}

/**
 * @brief NMT 명령 수신 콜백 (V4.0: AGR_PnP_Slave - 신규)
 * @param cmd NMT 명령 (AGR_NMT_CMD_START, AGR_NMT_CMD_STOP 등)
 * 
 * [V4.0 신규] V3.0에는 없던 콜백
 * - Master가 보낸 NMT 명령을 알림
 * - 상태 변경 자체는 PnP Slave 내부에서 자동 처리
 * - 여기서는 추가 로직 (로그, 특수 처리) 수행 가능
 */
static void _OnNmtCommand(AGR_NMT_Cmd_t cmd)
{
    (void)cmd;
    
    /* 필요시 NMT 명령에 따른 추가 처리 */
}

/* V3: _FindODEntry, _OnSdoRequest 제거
 * AGR_SDO_ProcessRequest() (Core)가 OD 조회/읽기/쓰기/WriteCb를 자동 처리합니다.
 * AGR_CANFD_ProcessRxMessage() → AGR_CANFD_ProcessSDORequest() → AGR_SDO_ProcessRequest()
 */

/**
 * @brief 에러 발생 콜백 (AGR_PnP)
 */
/* [REMOVED V4.0] _PnP_OnError 콜백 제거
 * [변경 이유]
 * - 기존: AGR_PnP_RecoveryAction_t _PnP_OnError() → 재시도 전략 반환
 * - 변경: AGR_PnP_Slave가 Boot-up 재전송, Timeout 처리를 자동으로 수행
 * - 효과: 에러 복구 로직이 PnP 라이브러리 내부에서 일관되게 처리
 */

static void _OnPdoReceived(uint32_t cob_id, const uint8_t* data, uint8_t len)
{
    (void)cob_id;
    (void)data;
    (void)len;
    
    /* 현재 IMU Hub는 RPDO를 사용하지 않음 */
}

/**
 * @brief TPDO1 Mapping 설정 콜백 (OD Entry 0x1A00 WriteCb)
 * @note XM10이 SDO를 통해 PDO Mapping A를 설정하면 자동 호출됩니다.
 */
static void _OnPdoMappingA_Set(void)
{
    /* Group A PDO Mapping 설정 → TPDO1 (V3 Core API 직접 사용)
     * race 가드: 갱신 구간 동안 Timer ISR SendTPDO1 encode 스킵 (s_pdo_map_updating). */
    s_pdo_map_updating = true;
    __DMB();
    AGR_PDO_ApplyMapFromSDO(&s_dop_ctx.tx_pdo_map[0], &s_dop_ctx.od,
                            s_pdo_mapping_a, 64);
    __DMB();
    s_pdo_map_updating = false;
}

/**
 * @brief TPDO2 Mapping 설정 콜백 (OD Entry 0x1A01 WriteCb)
 * @note XM10이 SDO를 통해 PDO Mapping B를 설정하면 자동 호출됩니다.
 */
static void _OnPdoMappingB_Set(void)
{
    /* Group B PDO Mapping 설정 → TPDO2 (V3 Core API 직접 사용)
     * race 가드: 갱신 구간 동안 Timer ISR SendTPDO2 encode 스킵 (s_pdo_map_updating). */
    s_pdo_map_updating = true;
    __DMB();
    AGR_PDO_ApplyMapFromSDO(&s_dop_ctx.tx_pdo_map[1], &s_dop_ctx.od,
                            s_pdo_mapping_b, 64);
    __DMB();
    s_pdo_map_updating = false;
}

/*
 * CANopen 메시지 처리 흐름 (V5.0 — FDCAN RX ISR 진입점 = XM_Drv_ProcessCANMessage)
 *
 * [컨텍스트 분리: RX ISR = 적재/위임만, decode/Tx 는 main()·Timer ISR]
 * 1. NMT (0x000)            → AGR_PnP_Slave_ProcessMessage()            (RX ISR inline)
 * 2. Boot-up/Heartbeat(0x700+ID) → AGR_PnP_Slave_ProcessMessage()      (RX ISR inline)
 * 3. SYNC (0x080)           → s_sync_pending flag set                   (RX ISR inline)
 * 4. SDO (0x600+ID)         → s_sdo_defer_ring 적재만(decode 금지)      (RX ISR)
 *    → main() XM_Drv_ProcessDeferredSdo → AGR_CANFD_ProcessRxMessage
 *      → AGR_SDO_ProcessRequest(OD 자동 조회/RW/WriteCb 예: _OnPdoMappingA_Set)
 *      → 응답은 tx-defer-ring → Timer ISR XM_Drv_DrainTxDeferRing 단일 송신
 * 5. PDO (0x180/0x280+ID)   → _OnPdoReceived (RPDO 현재 미사용)
 *
 * TPDO/HB Tx 는 Timer ISR(TIM16) 단일 컨텍스트. 상세: 파일 헤더 @details.
 */

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - Software Tx Queue Wrapper
 *------------------------------------------------------------
 */

/**
 * CANopen 메시지 우선순위 정의 (Device Layer)
 * ✅ IOIF Layer가 아닌 Device Layer에서 정의
 * ✅ CANopen 프로토콜 지식은 Device Layer에 국한
 */
#define CAN_PRIORITY_NMT        0   /**< NMT (최고 우선순위) */
#define CAN_PRIORITY_BOOTUP     0   /**< Boot-up/Heartbeat (0x700, 최고 — CAN-ID 로 구분 불가) */
#define CAN_PRIORITY_TPDO       1   /**< TPDO (실시간 데이터) */
#define CAN_PRIORITY_SDO        5   /**< SDO (설정 메시지) */
/* ⚠️ CAN_PRIORITY_HEARTBEAT 는 현재 _GetCanPriority 에서 미사용 — 0x700 은 Boot-up 과
 * CAN-ID 가 동일해 구분 불가하므로 둘 다 BOOTUP(최고)로 처리. payload(0x00/0x05) 기반
 * 분리 우선순위 도입 시 활성화할 예약 매크로(HB 는 본래 지연 허용 = 저우선). */
#define CAN_PRIORITY_HEARTBEAT  10  /**< (예약, 현재 미사용) Heartbeat 지연 허용 저우선 */

/**
 * @brief CAN ID별 우선순위 결정
 */
static uint8_t _GetCanPriority(uint32_t can_id)
{
    uint16_t fnc_code = can_id & 0x780;
    
    /* NMT Command (0x000) */
    if (can_id == 0x000) {
        return CAN_PRIORITY_NMT;  /* 0: 최고 우선순위 */
    }
    
    /* Boot-up, Heartbeat (0x700 + Node ID) */
    if (fnc_code == 0x700) {
        /* Boot-up은 첫 바이트가 0x00, Heartbeat는 0x05 */
        /* 여기서는 CAN ID만으로 구분 불가, 동일 우선순위 처리 */
        return CAN_PRIORITY_BOOTUP;  /* 0: Boot-up/Heartbeat (최고) */
    }
    
    /* SDO (0x600/0x580 + Node ID) */
    if (fnc_code == 0x600 || fnc_code == 0x580) {
        return CAN_PRIORITY_SDO;  /* 5: SDO */
    }
    
    /* PDO (0x180/0x280 + Node ID) */
    if (fnc_code == 0x180 || fnc_code == 0x280) {
        return CAN_PRIORITY_TPDO;  /* 1: TPDO (실시간) */
    }
    
    return 50;  /* 기타: 낮은 우선순위 */
}

/**
 * @brief Tx Wrapper 함수 — 메시지 유형별 분리 전송
 * @details
 * AGR_DOP, AGR_PnP가 호출하는 tx_func입니다.
 *
 * [양산 안전 설계 — TPDO/HB/SDO 경로 분리]
 *
 * TPDO (0x180, 0x280):
 *   → HW Tx FIFO 직접 전송만 시도
 *   → FIFO Full 시 Drop (stale data, 다음 tick에 최신 데이터 전송)
 *   → SW Queue 사용 금지 (Queue에 쌓인 과거 PDO가 최신 PDO보다 먼저 나가는 역전 방지)
 *
 * HB/SDO/NMT (기타 전부):
 *   → 항상 SW Queue로 전송 (HW FIFO 직접 사용 금지)
 *   → ProcessQueue()가 TPDO 예약 slot 보호하며 drain
 *   → HB/SDO가 TPDO slot을 선점하는 사고 원천 차단
 *
 * @return 0=성공, <0=실패 (AGR_TxFunc_t 시그니처 준수)
 */
/* Backpressure threshold: HB/SDO가 HW Queue를 직접 사용하려면 최소 이만큼 슬롯 여유
 * 1kHz Dual-Send 모드에서 TPDO가 매 tick 2 slot 사용 → HB는 1 slot만 남겨둬도 안전.
 * 값: 2 → HW Queue에 2 slot 이상 여유일 때만 HB/SDO 직접 전송, 아니면 SW Queue. */
#define DRV_HB_SDO_DIRECT_MIN_FREE   2U

/* Tx 경로 통계 (Device 레이어 내부 유지, Application이 Getter로 조회) */
static volatile uint32_t s_hb_direct_count   = 0;
static volatile uint32_t s_hb_queued_count   = 0;
static volatile uint32_t s_sdo_direct_count  = 0;
static volatile uint32_t s_sdo_queued_count  = 0;
static volatile uint32_t s_emcy_direct_count = 0;
static volatile uint32_t s_emcy_queued_count = 0;

/**
 * @brief main → Tx defer ring 적재 (producer=main, SPSC)
 * @details thread(main) 컨텍스트의 비-PDO·비-EMCY 프레임(SDO 응답)을 ring 에 적재만
 *          한다. 실제 HW 송신은 Timer ISR(XM_Drv_DrainTxDeferRing)이 단일 수행하여
 *          BareMetal FDCAN Tx single-writer 를 보장. full=drop (no-loss 설계상 발생
 *          시 사이징 부족 → s_tx_defer_drop 으로 감지).
 */
static int _TxFunc_DeferToRing(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    uint8_t head = s_tx_defer_head;
    uint8_t next = (uint8_t)((head + 1U) % XM_TX_DEFER_SLOTS);
    if (next == s_tx_defer_tail) {
        s_tx_defer_drop++;   /* ring full — no-loss 위반(사이징 부족) */
        return -1;
    }
    XmSdoDeferFrame_t* slot = &s_tx_defer_ring[head];
    slot->can_id = can_id;
    slot->len    = (len > 64U) ? 64U : len;
    memcpy(slot->data, data, slot->len);
    __DMB();                  /* 데이터 가시화 후 인덱스 공개 (producer 배리어) */
    s_tx_defer_head = next;
    return 0;
}

static int _Drv_TxFunc_Queue(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (s_fdcan_id == IOIF_FDCAN_INVALID_ID || data == NULL) {
        return -1;
    }

    uint16_t fnc_code = can_id & 0x780;
    bool is_tpdo    = (fnc_code == 0x180 || fnc_code == 0x280);
    bool is_sdo_rsp = (fnc_code == 0x580);
    bool is_emcy    = (fnc_code == 0x080) && (can_id != AGR_CAN_ID_SYNC);

    /* ===== Tx single-writer (BareMetal): thread(main) 컨텍스트는 tx-defer-ring 으로 =====
     * __get_IPSR()==0 → main() while(1)(deferred SDO 응답 + EMCY 트리거). HW Tx 는
     * Timer ISR 만 하도록 ring 에 적재 후 반환 → Timer ISR(XM_Drv_DrainTxDeferRing)이
     * 단일 송신. EMCY 도 포함(EMG 정합): 현재 EMCY 트리거는 SDO write(=main 컨텍스트)
     * 뿐이라 ring 경유가 곧 single-writer 보장. 아래 EMCY 직접경로는 ISR 컨텍스트
     * (IPSR!=0, 현재는 Timer ISR 만 = 단일 writer)에서만 도달. TPDO 는 항상 Timer ISR.
     * ⚠️ EMCY>TPDO 우선순위 격상(ring 선두 점프/HW priority)은 미구현 — XM master 의
     *    EMCY 수신 처리 기획 확정 후 도입(현재는 ring FIFO, 테스트 주입 latency 무관). */
    if (__get_IPSR() == 0U) {
        return _TxFunc_DeferToRing(can_id, data, len);
    }

    if (is_tpdo) {
        /* ===== TPDO: HW FIFO 직접 전송 (Full 시 Drop = stale data 방지) ===== */
        AGRBStatusDef result = IOIF_FDCAN_Transmit(s_fdcan_id, can_id, data, len);
        return (result == AGRBStatus_OK) ? 0 : -1;
    }

    /* ===== HW free slot 계산 =====
     * Queue 모드에서 IOIF_FDCAN_GetTxFifoFreeLevel 은 TFQF(full flag) 만 노출
     * (0/1)하므로 backpressure 판정에 쓸 수 없다. TXBRP popcount 로 실제
     * pending 을 구하고 3 - in_flight 로 free slot 수를 얻는다. */
    uint32_t in_flight = IOIF_FDCAN_GetTxInFlightCount(s_fdcan_id);
    uint32_t hw_free   = (in_flight < 3U) ? (3U - in_flight) : 0U;

    /* ===== EMCY (ISR 컨텍스트 전용 — main 발 EMCY 는 위에서 ring 경유) =====
     * IPSR!=0 (현재 Timer ISR 만 = 단일 writer) 에서만 도달. hw_free ≥ 1 면 HW direct,
     * hw_free == 0 시 SW Queue fallback. ⚠️ 비-Timer ISR 에서 EMCY 트리거 추가 시
     * 2nd writer 가 되므로 그 경로도 ring(또는 Timer ISR 일원화)로 보내야 함. */
    if (is_emcy) {
        if (hw_free >= 1U) {
            AGRBStatusDef result = IOIF_FDCAN_Transmit(s_fdcan_id, can_id, data, len);
            if (result == AGRBStatus_OK) {
                s_emcy_direct_count++;
                return 0;
            }
        }
        AGRBStatusDef result = IOIF_FDCAN_QueueMessage(s_fdcan_id, can_id, data, len, _GetCanPriority(can_id));
        if (result == AGRBStatus_OK) {
            s_emcy_queued_count++;
            return 0;
        }
        return -1;
    }

    /* ===== HB / SDO Response / NMT: Backpressure 기반 라우팅 =====
     * hw_free ≥ DRV_HB_SDO_DIRECT_MIN_FREE (=2): HW FIFO 직접 (TPDO 슬롯 보호)
     * hw_free <  2: SW Queue로 대기 (다음 ProcessQueue에서 drain) */
    if (hw_free >= DRV_HB_SDO_DIRECT_MIN_FREE) {
        AGRBStatusDef result = IOIF_FDCAN_Transmit(s_fdcan_id, can_id, data, len);
        if (result == AGRBStatus_OK) {
            if (is_sdo_rsp) s_sdo_direct_count++;
            else            s_hb_direct_count++;
            return 0;
        }
        /* 직접 전송 실패 시 SW Queue fallback */
    }

    /* SW Queue 경로 (backpressure or fallback) */
    uint8_t priority = _GetCanPriority(can_id);
    AGRBStatusDef result = IOIF_FDCAN_QueueMessage(s_fdcan_id, can_id, data, len, priority);
    if (result == AGRBStatus_OK) {
        if (is_sdo_rsp) s_sdo_queued_count++;
        else            s_hb_queued_count++;
        return 0;
    }
    return -1;
}

/**
 * @brief Tx 경로 통계 조회 (Application이 Live Expression용 g_fdcan_diag 에 주기 반영)
 * @param[out] hb_direct, hb_queued, sdo_direct, sdo_queued, emcy_direct, emcy_queued  각 NULL 허용
 */
void XM_Drv_GetTxPathStats(uint32_t* hb_direct, uint32_t* hb_queued,
                            uint32_t* sdo_direct, uint32_t* sdo_queued,
                            uint32_t* emcy_direct, uint32_t* emcy_queued)
{
    if (hb_direct   != NULL) *hb_direct   = s_hb_direct_count;
    if (hb_queued   != NULL) *hb_queued   = s_hb_queued_count;
    if (sdo_direct  != NULL) *sdo_direct  = s_sdo_direct_count;
    if (sdo_queued  != NULL) *sdo_queued  = s_sdo_queued_count;
    if (emcy_direct != NULL) *emcy_direct = s_emcy_direct_count;
    if (emcy_queued != NULL) *emcy_queued = s_emcy_queued_count;
}

/**
 * @brief 1ms 루프 카운터 갱신 (OD 0x6050, TPDO mapping)
 */
void XM_Drv_UpdateCtrlTickMs(uint32_t tick_ms)
{
    s_imu_hub_tx_data.ctrl_tick_ms = tick_ms;
}

/**
 * @brief Emergency (EMCY) 메시지 전송 (CiA 301)
 * @details AGR_CANFD_SendEmergency 래퍼. Tx 는 _Drv_TxFunc_Queue 경로로 흘러간다.
 *   main 컨텍스트(현재 트리거: SDO 0x7F02.01 write → _OnEmcyTestTrigger)에서 호출 시
 *   tx-defer-ring 적재 → Timer ISR(XM_Drv_DrainTxDeferRing) 단일 송신(single-writer).
 *   ISR 컨텍스트(IPSR!=0, 현재 Timer ISR 만) 호출 시 EMCY 분기 HW direct + SW Queue
 *   fallback. EMCY>TPDO 우선순위 격상은 XM master EMCY 기획 후 도입(현재 ring FIFO).
 */
int XM_Drv_SendEmcy(uint16_t error_code, uint8_t error_register)
{
    return AGR_CANFD_SendEmergency(&s_dop_ctx, error_code, error_register);
}

/**
 * @brief EMCY 테스트 주입 WriteCb (SDO 0x7F02.01 write 완료 시 호출)
 * @details buffer layout: byte0 = code_lo, byte1 = code_hi, byte2 = error_reg.
 *   더미 MfgSpec 5B 는 AGR_CANFD_SendEmergency 내부에서 0x00 로 고정.
 */
static void _OnEmcyTestTrigger(void)
{
    uint16_t code = (uint16_t)s_emcy_test_payload[0] |
                    ((uint16_t)s_emcy_test_payload[1] << 8);
    uint8_t  reg  = s_emcy_test_payload[2];
    (void)XM_Drv_SendEmcy(code, reg);
}

