/**
 ******************************************************************************
 * @file    imu_hub_xm_link_v2.c
 * @author  HyundoKim
 * @brief   [System/Link] XM10 ↔ IMU Hub Link Layer (AGR_PnP V2 기반, Master)
 * @version 2.0
 * @date    Dec 8, 2025
 *
 * @details
 * XM10 (Master)와 IMU Hub (Slave) 간 PnP 연결을 관리합니다.
 * AGR_PnP V2 API를 사용하여 CANopen 표준 PnP 시퀀스를 구현합니다.
 * 
 * [PnP 시퀀스]
 * 1. IMU Hub Boot-up 수신 (0x708)
 * 2. PRE_OPERATIONAL 전환 (AGR_PnP 자동)
 * 3. TPDO1 Mapping 설정 (SDO Write 0x1600)
 * 4. TPDO1 Mapping ACK 대기
 * 5. TPDO2 Mapping 설정 (SDO Write 0x1601)
 * 6. TPDO2 Mapping ACK 대기
 * 7. IMU Connected Mask 조회 (SDO Read 0x2000) [Optional]
 * 8. NMT START 전송
 * 9. OPERATIONAL Heartbeat 대기
 * 10. 연결 완료!
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_hub_xm_link_v2.h"
#include "imu_hub_drv.h"
#include "agr_pnp.h"
#include "agr_dop_node_id.h"
#include "system_startup.h"  /* System_Fdcan1_Transmit */
#include "ioif_agrb_tim.h"   /* IOIF_TIM_GetTick */
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 * @brief Pre-Operational 단계 (Link Layer 전용)
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
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

/** @brief AGR_PnP 인스턴스 (Master) */
static AGR_PnP_Inst_t s_pnp_inst;

/** @brief Pre-Op 상태 (Link Layer 관리) */
static ImuHub_PreOpState_t s_pre_op_state = IMUHUB_PRE_OP_IDLE;

/** @brief 마지막 SDO 전송 시간 (Timeout 체크용) */
static uint32_t s_last_sdo_tx_time = 0;

/** @brief SDO 재시도 횟수 */
static uint8_t s_sdo_retry_count = 0;

/** @brief PDO Mapping 데이터 (CANopen 표준 형식) */
/* 
 * TPDO1 (Group A): Metadata + IMU 0,1,2 (64B)
 * Offset  | Data              | Size | OD Index
 * --------|-------------------|------|----------
 * 0-3     | Metadata (packed) | 4B   | 0x3000.0x00
 *         |  - Timestamp      | 3B   |   (bits 0-23)
 *         |  - Valid Mask     | 1B   |   (bits 24-31)
 * 4-23    | IMU 0: {q,a,g}    | 20B  | 0x6000.0x60
 * 24-43   | IMU 1: {q,a,g}    | 20B  | 0x6001.0x60
 * 44-63   | IMU 2: {q,a,g}    | 20B  | 0x6002.0x60
 * 
 * [CANopen 표준 PDO Mapping Format]
 * - Byte 0: Object Length (bits) - 예: 32 (4B × 8), 160 (20B × 8)
 * - Byte 1: Object SubIndex
 * - Byte 2-3: Object Index (Little Endian)
 * - Total: 4 bytes per entry
 */
static const uint8_t s_pdo_map_a[] = {
    0x04,  /* Number of mapped objects = 4 */
    
    /* Entry 1: 0x3000.0x00 - Metadata (4B = 32 bits) */
    0x20, 0x00, 0x00, 0x30,  /* Length=32 bits, SubIndex=0x00, Index=0x3000 */
    
    /* Entry 2: 0x6000.0x60 - IMU0 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x00, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6000 */
    
    /* Entry 3: 0x6001.0x60 - IMU1 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x01, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6001 */
    
    /* Entry 4: 0x6002.0x60 - IMU2 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x02, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6002 */
};

/* TPDO2 (Group B): Metadata + IMU 3,4,5 (64B) */
static const uint8_t s_pdo_map_b[] = {
    0x04,  /* Number of mapped objects = 4 */
    
    /* Entry 1: 0x3000.0x00 - Metadata (4B = 32 bits) */
    0x20, 0x00, 0x00, 0x30,  /* Length=32 bits, SubIndex=0x00, Index=0x3000 */
    
    /* Entry 2: 0x6003.0x60 - IMU3 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x03, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6003 */
    
    /* Entry 3: 0x6004.0x60 - IMU4 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x04, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6004 */
    
    /* Entry 4: 0x6005.0x60 - IMU5 {q,a,g} (20B = 160 bits) */
    0xA0, 0x60, 0x05, 0x60,  /* Length=160 bits, SubIndex=0x60, Index=0x6005 */
};

/** @brief IMU Connected Mask (XM10이 조회한 값) */
static uint8_t s_imu_connected_mask = 0x00;

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* AGR_PnP 콜백 */
static void _Link_OnConnected(uint8_t node_id);
static void _Link_OnDisconnected(uint8_t node_id);
static void _Link_OnBootup(uint8_t node_id);
static void _Link_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
static void _Link_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst);
static void _Link_OnSDOResponse(uint8_t node_id, const AGR_SDO_Msg_t* response);
static AGR_PnP_RecoveryAction_t _Link_OnError(uint8_t node_id, AGR_PnP_Error_t error_code);

/* Dependency Injection: FDCAN Tx Wrapper */
static int _Link_TxFunc(uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IMU Hub Link 초기화 (V2)
 */
void ImuHub_XM_Link_Init_V2(void)
{
    /* 변수 초기화 */
    memset(&s_pnp_inst, 0, sizeof(AGR_PnP_Inst_t));
    s_pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_last_sdo_tx_time = 0;
    s_sdo_retry_count = 0;
    s_imu_connected_mask = 0x00;
    
    /* AGR_PnP 설정 */
    AGR_PnP_Config_t config = {
        .role = AGR_PNP_ROLE_MASTER,
        .my_node_id = AGR_NODE_ID_XM,
        .tx_func = _Link_TxFunc,
        .callbacks = {
            .on_connected = NULL,      /* Device별 콜백 사용 */
            .on_disconnected = NULL,
            .on_bootup = NULL,
            .on_nmt_change = NULL,
            .on_sdo_request = NULL,    /* Master는 SDO Request 받지 않음 */
            .on_sdo_response = NULL,   /* Device별 콜백 사용 */
            .on_run_pre_op = NULL,     /* Device별 콜백 사용 */
            .on_error = NULL
        }
    };
    
    /* AGR_PnP 초기화 */
    AGR_PnP_Init(&s_pnp_inst, &config, IOIF_TIM_GetTick);
    
    /* IMU Hub Driver 초기화 (Mutex, DOP Context, OD 초기화) */
    ImuHub_Drv_Init(_Link_TxFunc, NULL);  /* ✅ callbacks=NULL (V2는 PnP 콜백 사용) */
    
    /* IMU Hub Device 등록 */
    AGR_PnP_Device_t imu_hub_device = {
        .node_id = AGR_NODE_ID_IMU_HUB,
        .mode = AGR_PNP_MODE_PROTOCOL,
        .heartbeat_timeout = 5000,  /* 5s */
        .callbacks = {
            .on_connected = _Link_OnConnected,
            .on_disconnected = _Link_OnDisconnected,
            .on_bootup = _Link_OnBootup,
            .on_nmt_change = _Link_OnNmtChange,
            .on_sdo_request = NULL,      /* Master는 SDO Request 받지 않음 */
            .on_sdo_response = _Link_OnSDOResponse,
            .on_run_pre_op = _Link_RunPreOp,  /* Pre-Op 로직 구현 */
            .on_error = _Link_OnError
        }
    };
    AGR_PnP_RegisterDevice(&s_pnp_inst, &imu_hub_device);
}

/**
 * @brief 주기 실행 (V2)
 */
void ImuHub_XM_Link_RunPeriodic_V2(void)
{
    AGR_PnP_RunPeriodic(&s_pnp_inst);
    
    /* Pre-Op Timeout 체크 (Link Layer 관리) */
    if (s_pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A ||
        s_pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B ||
        s_pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
        
        uint32_t current_time = IOIF_TIM_GetTick();
        if (current_time - s_last_sdo_tx_time > AGR_PNP_SDO_TIMEOUT_MS) {
            if (s_sdo_retry_count < AGR_PNP_MAX_SDO_RETRIES) {
                /* 재시도 */
                s_sdo_retry_count++;
                if (s_pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A) {
                    s_pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
                } else if (s_pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B) {
                    s_pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
                } else if (s_pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
                    s_pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
                }
            } else {
                /* 최대 재시도 초과 → Error */
                s_pre_op_state = IMUHUB_PRE_OP_IDLE;
                s_sdo_retry_count = 0;
                /* Error는 AGR_PnP_OnError 콜백으로 전달됨 */
            }
        }
    }
}

/**
 * @brief CAN 메시지 처리 (V2)
 */
void ImuHub_XM_Link_ProcessMessage_V2(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    AGR_PnP_ProcessMessage(&s_pnp_inst, can_id, data, len);
}

/**
 * @brief IMU Hub 연결 상태 확인 (V2)
 */
bool ImuHub_XM_Link_IsConnected_V2(void)
{
    return AGR_PnP_IsDeviceConnected(&s_pnp_inst, AGR_NODE_ID_IMU_HUB);
}

/**
 * @brief IMU Connected Mask 조회 (V2)
 */
uint8_t ImuHub_XM_Link_GetImuConnectedMask_V2(void)
{
    return s_imu_connected_mask;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS (AGR_PnP Callbacks)
 *------------------------------------------------------------
 */

/**
 * @brief FDCAN Tx Wrapper (Dependency Injection)
 */
static int _Link_TxFunc(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    return System_Fdcan1_Transmit(can_id, (uint8_t*)data, len);
}

/**
 * @brief IMU Hub 연결 완료 콜백
 */
static void _Link_OnConnected(uint8_t node_id)
{
    (void)node_id;
    
    /* Pre-Op 완료 */
    s_pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_sdo_retry_count = 0;
    
    /* TODO: User API 업데이트 (XM.status.imu_hub_connected = true) */
    /* TODO: LED 표시 */
}

/**
 * @brief IMU Hub 연결 끊김 콜백
 */
static void _Link_OnDisconnected(uint8_t node_id)
{
    (void)node_id;
    
    /* Pre-Op 상태 초기화 */
    s_pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_sdo_retry_count = 0;
    s_imu_connected_mask = 0x00;
    
    /* TODO: User API 업데이트 (XM.status.imu_hub_connected = false) */
    /* TODO: LED 표시 */
}

/**
 * @brief IMU Hub Boot-up 수신 콜백
 */
static void _Link_OnBootup(uint8_t node_id)
{
    (void)node_id;
    
    /* Boot-up 수신 → Pre-Op 상태 머신 시작 준비 */
    s_pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
    s_sdo_retry_count = 0;
}

/**
 * @brief IMU Hub NMT 상태 변경 콜백
 */
static void _Link_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state)
{
    (void)node_id;
    (void)old_state;
    (void)new_state;
    
    /* TODO: 상태 변경 로그 */
}

/**
 * @brief Pre-Op 단계 실행 콜백 (Master only, Protocol-based)
 * @details AGR_PnP가 PRE_OPERATIONAL 상태일 때 주기적으로 호출
 */
static void _Link_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    switch (s_pre_op_state) {
        case IMUHUB_PRE_OP_IDLE:
            /* Boot-up 전 또는 완료 후 대기 */
            break;
        
        case IMUHUB_PRE_OP_SEND_PDO_MAP_A:
            /* TPDO1 Mapping 전송 (CANopen 표준 형식) */
            {
                int result = AGR_PnP_SendSDOWrite(inst, node_id, IMUHUB_OD_IDX_PDO_MAPPING_A, 0,
                                                  s_pdo_map_a, sizeof(s_pdo_map_a));
                if (result >= 0) {
                    /* ✅ 전송 성공 → WAIT 상태로 */
                    s_pre_op_state = IMUHUB_PRE_OP_WAIT_PDO_MAP_A;
                    s_last_sdo_tx_time = IOIF_TIM_GetTick();
                    s_sdo_retry_count = 0;
                }
                /* ❌ 전송 실패 → 다음 주기에 재시도 (상태 유지) */
            }
            break;
        
        case IMUHUB_PRE_OP_WAIT_PDO_MAP_A:
            /* SDO Response 대기 (SDO Response 콜백에서 처리) */
            break;
        
        case IMUHUB_PRE_OP_SEND_PDO_MAP_B:
            /* TPDO2 Mapping 전송 (CANopen 표준 형식) */
            {
                int result = AGR_PnP_SendSDOWrite(inst, node_id, IMUHUB_OD_IDX_PDO_MAPPING_B, 0,
                                                  s_pdo_map_b, sizeof(s_pdo_map_b));
                if (result >= 0) {
                    /* ✅ 전송 성공 → WAIT 상태로 */
                    s_pre_op_state = IMUHUB_PRE_OP_WAIT_PDO_MAP_B;
                    s_last_sdo_tx_time = IOIF_TIM_GetTick();
                    s_sdo_retry_count = 0;
                }
                /* ❌ 전송 실패 → 다음 주기에 재시도 (상태 유지) */
            }
            break;
        
        case IMUHUB_PRE_OP_WAIT_PDO_MAP_B:
            /* SDO Response 대기 */
            break;
        
        case IMUHUB_PRE_OP_SEND_IMU_MASK_REQ:
            /* IMU Connected Mask 조회 (Optional) */
            {
                int result = AGR_PnP_SendSDORead(inst, node_id, IMUHUB_OD_IDX_IMU_CONN_MASK, 0);

                if (result >= 0) {
                    /* ✅ 전송 성공 → WAIT 상태로 */
                    s_pre_op_state = IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP;
                    s_last_sdo_tx_time = IOIF_TIM_GetTick();
                    s_sdo_retry_count = 0;
                }
                /* ❌ 전송 실패 → 다음 주기에 재시도 (상태 유지) */
            }
            break;
        
        case IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP:
            /* SDO Response 대기 */
            break;
        
        case IMUHUB_PRE_OP_SEND_NMT_START:
            /* NMT START 전송 */
            {
                int result = AGR_PnP_SendNmtCommand(inst, node_id, AGR_NMT_CMD_START);

                if (result >= 0) {
                    /* ✅ 전송 성공 → COMPLETE 상태로 */
                    s_pre_op_state = IMUHUB_PRE_OP_COMPLETE;
                }
                /* ❌ 전송 실패 → 다음 주기에 재시도 (상태 유지) */
            }
            break;
        
        case IMUHUB_PRE_OP_COMPLETE:
            /* OPERATIONAL Heartbeat 대기 (AGR_PnP가 자동 처리) */
            break;
        
        default:
            break;
    }
}

/**
 * @brief SDO Response 수신 콜백
 */
static void _Link_OnSDOResponse(uint8_t node_id, const AGR_SDO_Msg_t* response)
{
    (void)node_id;
    
    uint8_t cs = response->cs & 0xE0;
    
    /* SDO Download Response (ACK) */
    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP) {
        if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_A) {
            /* TPDO1 Mapping 완료 → TPDO2로 */
            s_pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
            s_sdo_retry_count = 0;
        }
        else if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_B) {
            /* TPDO2 Mapping 완료 → IMU Mask 조회 */
            s_pre_op_state = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ;
            s_sdo_retry_count = 0;
        }
    }
    /* SDO Upload Response (Data) */
    else if ((response->cs & 0xE0) == 0x40) {  /* Upload Initiate Response (0x40~0x5F, including Expedited) */
        if (response->index == IMUHUB_OD_IDX_IMU_CONN_MASK) {
            /* 
             * IMU Connected Mask 수신 (0x2000.0x00)
             * Expedited Upload Response:
             * - cs = 0x4F (1 byte)
             * - data[0] = IMU Connected Mask (bit0~5: IMU 0~5)
             */
            s_imu_connected_mask = response->data[0];
            s_pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
            s_sdo_retry_count = 0;
        }
    }
}

/**
 * @brief 에러 발생 콜백 (복구 전략 반환)
 */
static AGR_PnP_RecoveryAction_t _Link_OnError(uint8_t node_id, AGR_PnP_Error_t error_code)
{
    (void)node_id;
    
    /* IMU Hub는 재시도 전략 사용 */
    if (error_code == AGR_PNP_ERROR_SDO_TIMEOUT) {
        return AGR_PNP_RECOVERY_RETRY;
    }
    else if (error_code == AGR_PNP_ERROR_SDO_ABORT) {
        return AGR_PNP_RECOVERY_ABORT;  /* Abort는 재시도 불가 */
    }
    else {
        return AGR_PNP_RECOVERY_ABORT;
    }
}
