/**
 ******************************************************************************
 * @file    imu_hub_drv.c
 * @author  HyundoKim
 * @brief   [Device Layer] XM10 ↔ IMU Hub 통신 드라이버 (Master, CANopen 표준)
 * @version 3.0 (CANopen 표준 준수)
 * @date    Dec 8, 2025
 *
 * @details
 * [V3.0 변경사항 - CANopen 표준 기반]
 * - AGR_NMT 통합: NMT, Boot-up, Heartbeat 처리
 * - CAN ID 기반 메시지 분기 (효율적)
 * - OD Index → 제조사 영역 (0x2000~) 이동
 * - Pre-Op 상태 머신 유지 (검증된 로직)
 * 
 * [메시지 처리 흐름]
 * ISR → ImuHub_Drv_ProcessCANMessage()
 *     ├─ CAN ID 0x700: Boot-up, Heartbeat → AGR_NMT
 *     ├─ CAN ID 0x580: SDO Response → Pre-Op State Machine
 *     └─ CAN ID 0x180/0x280: PDO → AGR_DOP
 * 
 * Device Layer는 제품 독립적이므로 module.h를 포함하지 않습니다.
 * RTOS/BareMetal 환경은 ioif_agrb_defs.h에서 자동 감지합니다.
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_hub_drv.h"
#include "agr_pnp.h"         /* ✅ AGR_PnP 통합 */
#include "agr_dop_node_id.h"
#include "ioif_agrb_defs.h"  /* ✅ RTOS/BareMetal 자동 감지 */
#include "ioif_agrb_tim.h"   /* IOIF_TIM_GetTick() */
#include <string.h>

/* ✅ Lock-Free Double Buffering: FreeRTOS/Bare-metal 헤더 불필요 */

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define IMUHUB_MAX_PDO_MAP_SIZE     64  /**< PDO Mapping 최대 크기 */

/**
 * @brief Object Dictionary Index (IMU Hub Slave와 동일)
 * @details XM10 (Master)이 IMU Hub (Slave)와 SDO 통신 시 사용하는 OD Index
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
 * │ SDO, PDO → OD Index를 통해 처리                          │
 * │ - CAN ID 0x580 + Node ID: SDO Response                 │
 * │ - CAN ID 0x180/0x280 + Node ID: PDO                    │
 * │ → AGR_DOP_ProcessRxMessage()가 처리 ✅                   │
 * └─────────────────────────────────────────────────────────┘
 */

/* ===== 0x2000 ~ 0x5FFF: Manufacturer Specific (Angel Robotics) ===== */

/* 센서 상태 (Read Only) */
#define IMUHUB_OD_IDX_IMU_CONN_MASK     0x2000  /**< IMU Connected Mask (RO, 1B) - bit0~5: IMU0~5 */

/* PDO Mapping (Write Only) */
#define IMUHUB_OD_IDX_PDO_MAPPING_A     0x2010  /**< TPDO1 Mapping (Group A: Metadata + IMU 0,1,2) */
#define IMUHUB_OD_IDX_PDO_MAPPING_B     0x2011  /**< TPDO2 Mapping (Group B: Metadata + IMU 3,4,5) */

/* PDO Metadata */
#define IMUHUB_OD_IDX_METADATA          0x2020  /**< PDO Metadata (4B BLOB): Timestamp(3B) + Valid Mask(1B) */

/** @brief IMU Data OD Index (0x6000~0x6005) */
#define IMUHUB_OD_IDX_IMU_BASE          0x6000

/* SubIndex 정의 (IMU Hub xm_drv.h와 동일) */
#define IMUHUB_OD_SUBIDX_QUAT_W         0x00
#define IMUHUB_OD_SUBIDX_QUAT_X         0x01
#define IMUHUB_OD_SUBIDX_QUAT_Y         0x02
#define IMUHUB_OD_SUBIDX_QUAT_Z         0x03

#define IMUHUB_OD_SUBIDX_EULER_ROLL     0x10
#define IMUHUB_OD_SUBIDX_EULER_PITCH    0x11
#define IMUHUB_OD_SUBIDX_EULER_YAW      0x12

#define IMUHUB_OD_SUBIDX_ACC_X          0x20
#define IMUHUB_OD_SUBIDX_ACC_Y          0x21
#define IMUHUB_OD_SUBIDX_ACC_Z          0x22

#define IMUHUB_OD_SUBIDX_GYR_X          0x30
#define IMUHUB_OD_SUBIDX_GYR_Y          0x31
#define IMUHUB_OD_SUBIDX_GYR_Z          0x32

#define IMUHUB_OD_SUBIDX_MAG_X          0x40
#define IMUHUB_OD_SUBIDX_MAG_Y          0x41
#define IMUHUB_OD_SUBIDX_MAG_Z          0x42

#define IMUHUB_OD_SUBIDX_QUAT_ARRAY     0x50  // q[4] (8B)
#define IMUHUB_OD_SUBIDX_EULER_ARRAY    0x51  // rpy[3] (6B)
#define IMUHUB_OD_SUBIDX_ACC_ARRAY      0x52  // a[3] (6B)
#define IMUHUB_OD_SUBIDX_GYR_ARRAY      0x53  // g[3] (6B)
#define IMUHUB_OD_SUBIDX_MAG_ARRAY      0x54  // m[3] (6B)

#define IMUHUB_OD_SUBIDX_QAG            0x60  // {q,a,g} (20B)
#define IMUHUB_OD_SUBIDX_QAGM           0x61  // {q,a,g,m} (26B)
#define IMUHUB_OD_SUBIDX_FULL           0x62  // {q,rpy,a,g,m} (32B)

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief Pre-Operational 단계 (Device Layer 전용)
 * @details Master Pre-Op 시퀀스 상태 관리
 */
typedef enum {
    IMUHUB_PRE_OP_IDLE = 0,          /**< 대기 (Boot-up 전 또는 완료 후) */
    IMUHUB_PRE_OP_SEND_PDO_MAP_A,    /**< TPDO1 Mapping 전송 */
    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,    /**< TPDO1 Mapping ACK 대기 */
    IMUHUB_PRE_OP_SEND_PDO_MAP_B,    /**< TPDO2 Mapping 전송 */
    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,    /**< TPDO2 Mapping ACK 대기 */
    IMUHUB_PRE_OP_SEND_IMU_MASK_REQ, /**< IMU Connected Mask 조회 (Optional) */
    IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP, /**< IMU Connected Mask 응답 대기 */
    IMUHUB_PRE_OP_SEND_NMT_START,    /**< NMT START 전송 */
    IMUHUB_PRE_OP_COMPLETE           /**< Pre-Op 완료 (OPERATIONAL 대기) */
} ImuHub_PreOpState_t;

/**
 * @brief Pre-Op Step Action 함수 타입 (Step Array 패턴)
 * @param node_id   대상 노드 ID
 * @param inst      AGR_PnP 인스턴스
 * @return 0: 성공 (다음 단계), -1: 실패 또는 대기
 */
typedef int (*ImuHub_PreOpAction_t)(uint8_t node_id, AGR_PnP_Inst_t* inst);

/**
 * @brief Pre-Op Step 정의 (Step Array 패턴)
 * @details 선언적 Pre-Op 시퀀스 정의 (향후 확장성)
 */
typedef struct {
    ImuHub_PreOpState_t send_state;     /**< 전송 상태 */
    ImuHub_PreOpState_t wait_state;     /**< 대기 상태 */
    ImuHub_PreOpAction_t action;        /**< 실행 함수 */
    uint32_t            timeout_ms;     /**< SDO Timeout (ms) */
    const char*         description;    /**< 단계 설명 (디버깅용) */
} ImuHub_PreOpStep_t;

/**
 * @brief IMU Hub 드라이버 인스턴스 (내부 관리)
 */
typedef struct {
    /* CANopen 표준 프로토콜 */
    AGR_DOP_Ctx_t       dop_ctx;            /**< DOP V2 Context (SDO, PDO) */
    AGR_TxFunc_t        tx_func;            /**< FDCAN Tx 함수 */
    
    /* Master PnP (System Layer) */
    AGR_PnP_Inst_t*     master_pnp;         /**< ✅ Master PnP 인스턴스 (System Layer에서 제공) */
    uint8_t             device_index;       /**< ✅ Master PnP devices[] 배열 인덱스 (NMT 접근용) */
    
    /* Pre-Op 상태 머신 (검증된 로직 유지) */
    ImuHub_PreOpState_t pre_op_state;       /**< Pre-Op 상태 */
    uint32_t            last_sdo_tx_time;   /**< 마지막 SDO 전송 시간 (Timeout 체크용) */
    uint8_t             sdo_retry_count;    /**< SDO 재시도 횟수 */
    uint8_t             imu_connected_mask; /**< IMU Connected Mask (SDO 조회 결과) */
    
    /* Rx Data (IMU Hub → XM10) */
    ImuHub_RxData_t     rx_data;            /**< 최종 처리된 데이터 (int16) */
    volatile bool       is_data_ready;      /**< TPDO 수신 플래그 */
    
} ImuHub_DrvInst_t;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

static ImuHub_DrvInst_t s_imu_hub_inst;

/* ===== Forward Declarations (Step Array Actions) ===== */
static int _Step_SendPdoMapA(uint8_t node_id, AGR_PnP_Inst_t* inst);
static int _Step_SendPdoMapB(uint8_t node_id, AGR_PnP_Inst_t* inst);
static int _Step_SendImuMaskReq(uint8_t node_id, AGR_PnP_Inst_t* inst);
static int _Step_SendNmtStart(uint8_t node_id, AGR_PnP_Inst_t* inst);

/**
 * @brief Pre-Op Step Array (선언적 시퀀스 정의)
 * @details 확장성을 위한 Step Array 패턴
 */
static const ImuHub_PreOpStep_t s_pre_op_steps[] = {
    { IMUHUB_PRE_OP_SEND_PDO_MAP_A,    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,    _Step_SendPdoMapA,     5000,    "TPDO1 Mapping" },
    { IMUHUB_PRE_OP_SEND_PDO_MAP_B,    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,    _Step_SendPdoMapB,     5000,    "TPDO2 Mapping" },
    { IMUHUB_PRE_OP_SEND_IMU_MASK_REQ, IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP, _Step_SendImuMaskReq,  5000,    "IMU Mask Read" },
    { IMUHUB_PRE_OP_SEND_NMT_START,    IMUHUB_PRE_OP_COMPLETE,          _Step_SendNmtStart,    5000,    "NMT START" },
};

#define PRE_OP_STEP_COUNT (sizeof(s_pre_op_steps) / sizeof(s_pre_op_steps[0]))

/** @brief PDO Mapping 데이터 (CANopen 표준 형식) */
/* 
 * TPDO1 (Group A): Metadata + IMU 0,1,2 (64B)
 * [CANopen 표준 PDO Mapping Format]
 * - Byte 0: Number of mapped objects
 * - Entry Format: Length(bits), SubIndex, Index(LE)
 */
static const uint8_t s_pdo_map_a[] = {
    0x04,  /* Number of mapped objects = 4 */
    /* Entry 1: 0x3000.0x00 - Metadata (4B = 32 bits) */
    0x20, 0x00, 0x00, 0x30,
    /* Entry 2: 0x6000.0x60 - IMU0 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x00, 0x60,
    /* Entry 3: 0x6001.0x60 - IMU1 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x01, 0x60,
    /* Entry 4: 0x6002.0x60 - IMU2 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x02, 0x60,
};

/* TPDO2 (Group B): Metadata + IMU 3,4,5 (64B) */
static const uint8_t s_pdo_map_b[] = {
    0x04,  /* Number of mapped objects = 4 */
    /* Entry 1: 0x3000.0x00 - Metadata (4B = 32 bits) */
    0x20, 0x00, 0x00, 0x30,
    /* Entry 2: 0x6003.0x60 - IMU3 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x03, 0x60,
    /* Entry 3: 0x6004.0x60 - IMU4 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x04, 0x60,
    /* Entry 4: 0x6005.0x60 - IMU5 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x05, 0x60,
};

/**
 * ✅ Lock-Free Double Buffer (Real-Time 완벽 보장)
 * - ISR: write_idx 버퍼에 쓰고 Atomic Swap
 * - Main Task: read_idx 버퍼에서 읽기 (반대 버퍼)
 * - Lock 불필요, 데이터 손실 없음, 실시간성 완벽
 * 
 * [설계 V2] TPDO1과 TPDO2 완전 독립
 * - TPDO1: 독립적인 Double Buffer (IMU 0,1,2)
 * - TPDO2: 독립적인 Double Buffer (IMU 3,4,5)
 * - 메모리: 4개 버퍼 (각 PDO당 2개)
 * - 장점: 완전히 독립적, 하나가 늦어도 다른 것 업데이트, 실시간성 극대화
 */

/* TPDO1 전용 데이터 (IMU 0, 1, 2) */
typedef struct {
    uint32_t         timestamp;
    uint8_t          connected_mask;
    ImuHub_ImuData_t imu[IMUHUB_GROUP_A_COUNT];  /* IMU 0,1,2 */
} ImuHub_Tpdo1Data_t;

/* TPDO2 전용 데이터 (IMU 3, 4, 5) */
typedef struct {
    uint32_t         timestamp;
    uint8_t          connected_mask;
    ImuHub_ImuData_t imu[IMUHUB_GROUP_B_COUNT];  /* IMU 3,4,5 */
} ImuHub_Tpdo2Data_t;

/* TPDO1 Double Buffer */
typedef struct {
    ImuHub_Tpdo1Data_t buffers[2];
    volatile uint8_t write_idx;
} ImuHub_Tpdo1Buffer_t;

/* TPDO2 Double Buffer */
typedef struct {
    ImuHub_Tpdo2Data_t buffers[2];
    volatile uint8_t write_idx;
} ImuHub_Tpdo2Buffer_t;

static ImuHub_Tpdo1Buffer_t s_tpdo1_buf = { .buffers = {{0}, {0}}, .write_idx = 0 };
static ImuHub_Tpdo2Buffer_t s_tpdo2_buf = { .buffers = {{0}, {0}}, .write_idx = 0 };

/**
 * ✅ Lock-Free 설계 (Critical Section 불필요)
 * 
 * [이유]
 * 1. volatile uint8_t write_idx: 단일 바이트
 * 2. ARM Cortex-M: 단일 바이트 쓰기는 하드웨어 레벨에서 Atomic
 * 3. ISR: write_idx 버퍼에 쓰고 → write_idx 토글 (Atomic)
 * 4. Main Task: (1 - write_idx) 버퍼에서 읽음 (항상 반대 버퍼)
 * 
 * [결론]
 * - __disable_irq() / taskENTER_CRITICAL 불필요
 * - 완벽한 Lock-Free Double Buffering
 */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * ===== [Layer 3] AGR_PnP 콜백 (Device Driver에서 구현) =====
 * 
 * [역할 분리]
 * - AGR_NMT: CANopen NMT 프로토콜 처리 (Boot-up, Heartbeat)
 * - AGR_PnP: PnP 추상화 (State Machine, Timeout)
 * - Device Driver: 응용 로직 (Pre-Op 시퀀스, 데이터 처리)
 * 
 * [콜백 호출 흐름]
 * Boot-up 수신 → AGR_NMT_ProcessMessage → AGR_NMT_UpdateActivity
 *   → _OnNmtStateChanged (agr_pnp.c 내부)
 *   → _PnP_OnBootup (Device Driver) ✅
 */

/* Master 전용 콜백 (Slave 모니터링) */
static void _PnP_OnBootup(uint8_t node_id);        /* Slave Boot-up 수신 → Pre-Op 초기화 */
static void _PnP_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);  /* Slave NMT 변경 → Pre-Op State Machine */
static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst);  /* Pre-Op 단계 실행 (Step Array) */
static AGR_PnP_RecoveryAction_t _PnP_OnError(uint8_t node_id, AGR_PnP_Error_t error_code);  /* 에러 처리 */

/* 사용 안 함 (Slave 전용) */
static void _PnP_OnConnected(uint8_t node_id);     /* Master는 사용 안 함 (NULL) */
static void _PnP_OnDisconnected(uint8_t node_id);  /* Master는 사용 안 함 (NULL) */

/**
 * ===== [Layer 2] AGR_DOP 콜백 (Device Driver에서 구현) =====
 * 
 * [역할]
 * - AGR_DOP: SDO/PDO 인코딩/디코딩
 * - Device Driver: SDO Response 처리 (Pre-Op 전이)
 * 
 * [Master는 SDO Request 보내고 Response 받음]
 */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response);  /* SDO Response 처리 → Pre-Op 상태 전이 */

/* Legacy 콜백 제거 (AGR_PnP가 처리):
 * - _ImuHub_OnSdoBootup → _PnP_OnBootup
 * - _ImuHub_OnSdoHeartbeat → AGR_PnP Heartbeat 처리
 * - _ImuHub_OnSdoSyncStates → 사용 안함
 * - _ImuHub_OnPdoMappingAck → _PnP_RunPreOp에서 처리
 */

/* ===== PDO 처리 (Double Buffer용) ===== */
static void _ImuHub_DecodeTpdo1_ToBuf(ImuHub_Tpdo1Data_t* buf, const uint8_t* data, uint8_t len);
static void _ImuHub_DecodeTpdo2_ToBuf(ImuHub_Tpdo2Data_t* buf, const uint8_t* data, uint8_t len);

/* Helper 함수 제거 (AGR_PnP API 사용) */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

int ImuHub_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Inst_t* master_pnp)
{
    if (tx_func == NULL || master_pnp == NULL) {
        return -1;  /* ✅ int 반환 */
    }
    
    memset(&s_imu_hub_inst, 0, sizeof(ImuHub_DrvInst_t));
    
    /* 1. Tx 함수 저장 */
    s_imu_hub_inst.tx_func = tx_func;
    
    /* 2. Master PnP 인스턴스 저장 */
    s_imu_hub_inst.master_pnp = master_pnp;
    
    /* ✅ Lock-Free Double Buffer 사용: Mutex 불필요 */
    
    /* ===================================================================
     * [Layer 2] AGR_DOP 초기화 (CANopen SDO/PDO 프로토콜)
     * ===================================================================
     * [역할] SDO Request/Response, PDO 송수신
     * [Master] OD 불필요 (Slave의 OD만 접근)
     */
    AGR_DOP_Init(&s_imu_hub_inst.dop_ctx,
                 NULL,              /* OD Table: Master는 불필요 (Slave OD만 접근) */
                 AGR_NODE_ID_XM,    /* XM10 Node ID (0x02) */
                 tx_func);
    
    /* ===================================================================
     * [Layer 3] AGR_PnP에 Device 등록 (Master PnP)
     * ===================================================================
     * [역할] Slave(IMU Hub) 연결 상태 관리, Pre-Op State Machine
     * [콜백] Device Driver에서 구현한 Master 전용 콜백 등록
     * 
     * [Master PnP는 System Layer에서 하나만 관리]
     * - pnp_manager.c에서 s_master_pnp 생성
     * - 각 Device Driver는 Device만 등록
     */
    AGR_PnP_Device_t imu_hub_device = {
        .name = "IMU Hub",              /* 디버깅용 이름 (Live Expression) */
        .node_id = AGR_NODE_ID_IMU_HUB,
        .heartbeat_timeout = 3000,
        .callbacks = {
            /* Master 전용 콜백 */
            .on_bootup = _PnP_OnBootup,        /* Slave Boot-up → Pre-Op 초기화 */
            .on_nmt_change = _PnP_OnNmtChange,  /* Slave NMT 변경 → Pre-Op State Machine */
            .on_run_pre_op = _PnP_RunPreOp,    /* Pre-Op 단계 실행 (Step Array) */
            .on_error = _PnP_OnError,          /* 에러 처리 (Timeout, SDO Abort) */
            
            /* Slave 전용 콜백 (Master는 사용 안 함) */
            .on_connected = NULL,      /* Master는 사용 안 함 */
            .on_disconnected = NULL,   /* Master는 사용 안 함 */
        }
    };
    int dev_idx = AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device);
    if (dev_idx < 0) {
        return -1;  /* 등록 실패 */
    }
    s_imu_hub_inst.device_index = (uint8_t)dev_idx;  /* ✅ Device Index 저장 (NMT 접근용) */
    
    return 0;  /* ✅ 성공 */
}

void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ===== CAN ID 기반 분기 (CANopen 표준) ===== */
    
    /* 1. Boot-up / Heartbeat (CAN ID 0x700 + Node ID) */
    uint16_t fnc_code = can_id & 0x780;  /* Function Code 추출 */
    if (fnc_code == 0x700) {
        /* ✅ Master PnP의 Device NMT로 전달 */
        AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
        AGR_NMT_ProcessMessage(&dev->nmt, can_id, data, len, current_ms);
        return;
    }
    
    /* 2. SDO Response (CAN ID 0x580 + Node ID) */
    if (fnc_code == 0x580) {
        /* ✅ AGR_DOP가 SDO 디코딩 (CANopen 표준 프로토콜 처리) */
        AGR_SDO_Msg_t sdo_msg;
        if (AGR_DOP_DecodeSDO(data, len, &sdo_msg) == 0) {
            /* Device Driver가 Pre-Op 상태 전환 (응용 로직) */
            _OnSdoResponse(&sdo_msg);
        }
        return;
    }
    
    /* 3. PDO (CAN ID 0x180/0x280 + Node ID) */
    if (fnc_code == 0x180) {  /* TPDO1 (Group A: IMU 0,1,2) - 완전 독립 */
        /* ✅ TPDO1 전용 Double Buffer */
        uint8_t write_idx = s_tpdo1_buf.write_idx;
        ImuHub_Tpdo1Data_t* write_buf = &s_tpdo1_buf.buffers[write_idx];
        
        _ImuHub_DecodeTpdo1_ToBuf(write_buf, data, len);
        
        /* ✅ Lock-Free Swap (TPDO1 독립적으로 토글) */
        s_tpdo1_buf.write_idx = 1 - write_idx;
        s_imu_hub_inst.is_data_ready = true;
        
        return;
    }
    
    if (fnc_code == 0x280) {  /* TPDO2 (Group B: IMU 3,4,5) - 완전 독립 */
        /* ✅ TPDO2 전용 Double Buffer */
        uint8_t write_idx = s_tpdo2_buf.write_idx;
        ImuHub_Tpdo2Data_t* write_buf = &s_tpdo2_buf.buffers[write_idx];
        
        _ImuHub_DecodeTpdo2_ToBuf(write_buf, data, len);
        
        /* ✅ Lock-Free Swap (TPDO2 독립적으로 토글) */
        s_tpdo2_buf.write_idx = 1 - write_idx;
        s_imu_hub_inst.is_data_ready = true;
        
        return;
    }
}

/**
 * @brief IMU Hub 연결 상태 확인 (V3.0: Public API)
 * @return true: 연결됨 (OPERATIONAL + Heartbeat OK)
 */
bool ImuHub_Drv_IsConnected(void)
{
    AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
    return AGR_NMT_IsConnected(&dev->nmt);
}

/**
 * @brief IMU Hub NMT 상태 조회 (V3.0: Public API)
 * @return NMT 상태
 */
AGR_NMT_State_t ImuHub_Drv_GetNmtState(void)
{
    AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
    return AGR_NMT_GetState(&dev->nmt);
}

/**
 * @brief 주기 실행 (CANopen 표준)
 * @details 
 * Main Loop 또는 Low-Priority Task에서 호출합니다.
 * 
 * [처리 항목]
 * 1. AGR_NMT Heartbeat Timeout 체크
 * 2. AGR_PnP 주기 처리 (Pre-Op 시퀀스)
 */
void ImuHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ✅ NMT/PnP는 System Layer (PnPManager)에서 관리 */
    /* Device Driver는 Pre-Op Timeout만 체크 */
    
    /* 1. Pre-Op Timeout 체크 */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
        
        if (current_ms - s_imu_hub_inst.last_sdo_tx_time > 5000) {  /* 5s Timeout (Step Array에 맞춤) */
            if (s_imu_hub_inst.sdo_retry_count < 3) {
                /* ✅ 재시도: WAIT → SEND (같은 단계 재시도) */
                s_imu_hub_inst.sdo_retry_count++;
                if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A) {
                    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;  /* ✅ A 재시도 */
                } else if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B) {
                    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;  /* ✅ B 재시도 */
                } else if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
                    /* IMU Mask는 Optional → Skip하고 다음 단계 */
                    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
                    s_imu_hub_inst.sdo_retry_count = 0;
                }
            } else {
                /* 최대 재시도 초과 → Error */
                s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_IDLE;
                s_imu_hub_inst.sdo_retry_count = 0;
            }
        }
    }
}

/**
 * @brief IMU Hub PDO 데이터 가져오기 (Lock-Free Double Buffer)
 * @details Main Task에서 호출, ISR이 쓰지 않는 반대 버퍼에서 읽음
 */
bool ImuHub_Drv_GetRxData(ImuHub_RxData_t* rx_data)
{
    if (rx_data == NULL) {
        return false;
    }
    
    if (!s_imu_hub_inst.is_data_ready) {
        return false;
    }
    
    /* ✅ Lock-Free: 각 TPDO의 반대 버퍼에서 읽어서 병합 */
    uint8_t tpdo1_read_idx = 1 - s_tpdo1_buf.write_idx;
    uint8_t tpdo2_read_idx = 1 - s_tpdo2_buf.write_idx;
    
    ImuHub_Tpdo1Data_t* tpdo1_data = &s_tpdo1_buf.buffers[tpdo1_read_idx];
    ImuHub_Tpdo2Data_t* tpdo2_data = &s_tpdo2_buf.buffers[tpdo2_read_idx];
    
    /* TPDO1 데이터 복사 (IMU 0,1,2) */
    rx_data->timestamp = tpdo1_data->timestamp;  /* 최신 timestamp 사용 */
    rx_data->connected_mask = tpdo1_data->connected_mask | tpdo2_data->connected_mask;
    memcpy(&rx_data->imu[0], tpdo1_data->imu, sizeof(ImuHub_ImuData_t) * IMUHUB_GROUP_A_COUNT);
    
    /* TPDO2 데이터 복사 (IMU 3,4,5) */
    memcpy(&rx_data->imu[3], tpdo2_data->imu, sizeof(ImuHub_ImuData_t) * IMUHUB_GROUP_B_COUNT);
    
    return true;
}

bool ImuHub_Drv_IsDataReady(void)
{
    return s_imu_hub_inst.is_data_ready;
}

/* Legacy 함수들 제거 (AGR_PnP와 AGR_DOP가 처리) */

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - AGR_PnP Callbacks (검증된 Pre-Op 로직)
 *------------------------------------------------------------
 */

/**
 * @brief IMU Hub 연결 완료 콜백
 */
static void _PnP_OnConnected(uint8_t node_id)
{
    (void)node_id;
    
    /* Pre-Op 완료 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_imu_hub_inst.sdo_retry_count = 0;
    
    /* TODO: User 콜백 호출 */
}

/**
 * @brief IMU Hub 연결 끊김 콜백
 */
static void _PnP_OnDisconnected(uint8_t node_id)
{
    (void)node_id;
    
    /* Pre-Op 상태 초기화 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_imu_hub_inst.sdo_retry_count = 0;
    s_imu_hub_inst.imu_connected_mask = 0x00;
    
    /* TODO: User 콜백 호출 */
}

/**
 * @brief IMU Hub Boot-up 수신 콜백
 */
static void _PnP_OnBootup(uint8_t node_id)
{
    (void)node_id;
    
    /* ✅ Boot-up 수신 → Pre-Op 상태 머신 시작 (IDLE 또는 COMPLETE 상태에서만) */
    /* 이미 Pre-Op 진행 중이면 무시 (중복 Boot-up 방지) */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_IDLE ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_COMPLETE) {
        s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
        s_imu_hub_inst.sdo_retry_count = 0;
    }
}

/**
 * @brief IMU Hub NMT 상태 변경 콜백
 */
static void _PnP_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state)
{
    (void)node_id;
    (void)old_state;
    (void)new_state;
    
    /* TODO: 상태 변경 로그 */
}

/**
 * @brief Pre-Op 단계 실행 콜백 (✅ Step Array 패턴, 향후 확장성)
 * @details AGR_PnP가 PRE_OPERATIONAL 상태일 때 주기적으로 호출
 */
static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    /* IDLE 상태이고 Slave가 PRE_OPERATIONAL이면 Pre-Op 시작 (Boot-up 놓친 경우) */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_IDLE) {
        AGR_PnP_Device_t* dev = &inst->devices[s_imu_hub_inst.device_index];
        if (dev->nmt.state == AGR_NMT_PRE_OPERATIONAL) {
            /* XM 리셋 시 Slave가 이미 PRE_OP 상태라면 능동적으로 Pre-Op 시작 */
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
        return;
    }
    
    /* COMPLETE 상태는 아무것도 안함 */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_COMPLETE) {
        return;
    }
    
    /* ✅ Step Array 기반 실행 (확장성) */
    for (uint8_t i = 0; i < PRE_OP_STEP_COUNT; i++) {
        const ImuHub_PreOpStep_t* step = &s_pre_op_steps[i];
        
        /* 현재 상태가 SEND 상태인 경우 */
        if (s_imu_hub_inst.pre_op_state == step->send_state) {
            /* Action 실행 */
            int result = step->action(node_id, inst);
            if (result >= 0) {
                /* 성공 → WAIT 상태로 전환 */
                s_imu_hub_inst.pre_op_state = step->wait_state;
                s_imu_hub_inst.last_sdo_tx_time = IOIF_TIM_GetTick();
                s_imu_hub_inst.sdo_retry_count = 0;
            }
            return;
        }
        
        /* 현재 상태가 WAIT 상태인 경우 (SDO Response 대기) */
        if (s_imu_hub_inst.pre_op_state == step->wait_state) {
            /* 대기 중 (SDO Response는 _OnSdoResponse에서 처리) */
            return;
        }
    }
}

/**
 *-----------------------------------------------------------
 * STEP ARRAY - Action 함수 구현
 *-----------------------------------------------------------
 */

/**
 * @brief Step 1: TPDO1 Mapping 전송
 */
static int _Step_SendPdoMapA(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    return AGR_PnP_SendSDOWrite(inst, node_id, IMUHUB_OD_IDX_PDO_MAPPING_A, 0,
                                s_pdo_map_a, sizeof(s_pdo_map_a));
}

/**
 * @brief Step 2: TPDO2 Mapping 전송
 */
static int _Step_SendPdoMapB(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    return AGR_PnP_SendSDOWrite(inst, node_id, IMUHUB_OD_IDX_PDO_MAPPING_B, 0,
                                s_pdo_map_b, sizeof(s_pdo_map_b));
}

/**
 * @brief Step 3: IMU Connected Mask 조회 (Optional)
 */
static int _Step_SendImuMaskReq(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    return AGR_PnP_SendSDORead(inst, node_id, IMUHUB_OD_IDX_IMU_CONN_MASK, 0);
}

/**
 * @brief Step 4: NMT START 전송
 */
static int _Step_SendNmtStart(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    return AGR_PnP_SendNmtCommand(inst, node_id, AGR_NMT_CMD_START);
}

/**
 * @brief 에러 발생 콜백
 */
static AGR_PnP_RecoveryAction_t _PnP_OnError(uint8_t node_id, AGR_PnP_Error_t error_code)
{
    (void)node_id;
    
    if (error_code == AGR_PNP_ERROR_SDO_TIMEOUT) {
        return AGR_PNP_RECOVERY_RETRY;
    }
    else if (error_code == AGR_PNP_ERROR_SDO_ABORT) {
        return AGR_PNP_RECOVERY_ABORT;
    }
    else {
        return AGR_PNP_RECOVERY_ABORT;
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - AGR_DOP Callbacks
 *------------------------------------------------------------
 */

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS - Device Driver 콜백 (응용 로직)
 *------------------------------------------------------------
 */

/**
 * @brief SDO Response 처리 (Pre-Op 상태 전환)
 * @details 
 * ✅ 레이어 분리:
 * - AGR_DOP: CANopen 표준 프로토콜 처리 (SDO 디코딩)
 * - Device Driver: 응용 로직 (Pre-Op 상태 머신)
 */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response)
{
    uint8_t cs = response->cs & 0xE0;
    
    /* SDO Download Response (ACK) */
    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP) {
        if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_A) {
            /* TPDO1 Mapping 완료 → TPDO2로 */
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
        else if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_B) {
            /* TPDO2 Mapping 완료 → IMU Mask 조회 */
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
    }
    /* SDO Upload Response (Data) */
    else if ((response->cs & 0xE0) == 0x40) {
        if (response->index == IMUHUB_OD_IDX_IMU_CONN_MASK) {
            /* IMU Connected Mask 수신 */
            s_imu_hub_inst.imu_connected_mask = response->data[0];
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
    }
}

/* ===== PDO 디코딩 (Double Buffer용) ===== */

static void _ImuHub_DecodeTpdo1_ToBuf(ImuHub_Tpdo1Data_t* buf, const uint8_t* data, uint8_t len)
{
    /* 
     * [TPDO1 구조]
     * PDO Mapping (64B):
     * - 0x3000.0x00: Metadata (4B) = {Timestamp(24bit) + Valid Mask(8bit)}
     * - 0x6000.0x60: IMU 0 {q,a,g} (20B)
     * - 0x6001.0x60: IMU 1 {q,a,g} (20B)
     * - 0x6002.0x60: IMU 2 {q,a,g} (20B)
     * 
     * [Payload 구조]
     * Offset  | Data                 | Size
     * --------|----------------------|-----
     * 0-2     | Timestamp (24bit)    | 3B
     * 3       | Valid Mask (8bit)    | 1B
     * 4-23    | IMU 0 {q,a,g}        | 20B
     * 24-43   | IMU 1 {q,a,g}        | 20B
     * 44-63   | IMU 2 {q,a,g}        | 20B
     * 
     * [Metadata 구조]
     * Byte 0: Timestamp bits 0-7
     * Byte 1: Timestamp bits 8-15
     * Byte 2: Timestamp bits 16-23
     * Byte 3: Valid Mask (bit0~5 = IMU 0~5)
     */
    
    (void)len;  /* Unused */
    
    /* ✅ TPDO1 전용 Double Buffer에 직접 쓰기 (Lock 불필요) */
    /* Metadata (4B): Timestamp (24-bit) + Valid Mask (8-bit) */
    buf->timestamp = (data[0]) | (data[1] << 8) | (data[2] << 16);
    buf->connected_mask = data[3];  /* bit0~5: IMU 0~5 */
    
    /* IMU Data (20B × 3) - Group A */
    int offset = 4;  /* Metadata(4B) 이후 */
    for (int i = 0; i < IMUHUB_GROUP_A_COUNT; i++) {
        memcpy(&buf->imu[i], data + offset, 20);
        offset += 20;
    }
}

static void _ImuHub_DecodeTpdo2_ToBuf(ImuHub_Tpdo2Data_t* buf, const uint8_t* data, uint8_t len)
{
    /* 
     * [TPDO2 구조]
     * PDO Mapping (64B):
     * - 0x3000.0x00: Metadata (4B) = {Timestamp(24bit) + Valid Mask(8bit)}
     * - 0x6003.0x60: IMU 3 {q,a,g} (20B)
     * - 0x6004.0x60: IMU 4 {q,a,g} (20B)
     * - 0x6005.0x60: IMU 5 {q,a,g} (20B)
     * 
     * [Payload 구조]
     * Offset  | Data                 | Size
     * --------|----------------------|-----
     * 0-2     | Timestamp (24bit)    | 3B
     * 3       | Valid Mask (8bit)    | 1B
     * 4-23    | IMU 3 {q,a,g}        | 20B
     * 24-43   | IMU 4 {q,a,g}        | 20B
     * 44-63   | IMU 5 {q,a,g}        | 20B
     * 
     * [Metadata 구조]
     * Byte 0: Timestamp bits 0-7
     * Byte 1: Timestamp bits 8-15
     * Byte 2: Timestamp bits 16-23
     * Byte 3: Valid Mask (bit0~5 = IMU 0~5)
     */
    
    (void)len;  /* Unused */
    
    /* ✅ TPDO2 전용 Double Buffer에 직접 쓰기 (Lock 불필요) */
    /* Timestamp (3B → 24bit) */
    buf->timestamp = (data[0]) | (data[1] << 8) | (data[2] << 16);
    buf->connected_mask = data[3];  /* Valid Mask */
    
    /* IMU Data (20B × 3) - Group B */
    int offset = 4;  /* Metadata(4B) 이후 */
    for (int i = 0; i < IMUHUB_GROUP_B_COUNT; i++) {
        memcpy(&buf->imu[i], data + offset, 20);  /* buf->imu[0,1,2] = IMU 3,4,5 */
        offset += 20;
    }
}

/* ===== Utility ===== */

static void _ImuHub_SendSdo(const AGR_SDO_Msg_t* sdo_msg)
{
    if (sdo_msg == NULL || s_imu_hub_inst.tx_func == NULL) {
        return;
    }
    
    uint8_t sdo_buf[64];
    int len = AGR_DOP_EncodeSDO(sdo_msg, sdo_buf);
    
    if (len > 0) {
        /* [DOP V2] XM10 → IMU Hub SDO Request (Node ID만 사용) */
        uint32_t can_id = AGR_DOP_GetSDORequestCANID(AGR_NODE_ID_IMU_HUB);  /* 0x608 */
        s_imu_hub_inst.tx_func(can_id, sdo_buf, (uint8_t)len);
    }
}

