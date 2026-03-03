/**
 ******************************************************************************
 * @file    imu_hub_xm_link.c
 * @author  HyundoKim
 * @brief   [System/Links] XM10 ↔ IMU Hub 링크 관리 (PnP State Machine)
 * @version 1.0
 * @date    Dec 4, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_hub_xm_link.h"
#include "imu_hub_drv.h"
#include "agr_pnpmgr.h"
#include "agr_dop_node_id.h"
#include "system_startup.h"  /* System_Fdcan1_Transmit */
#include "ioif_agrb_tim.h"   /* IOIF_TIM_GetTick */

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
 * @brief PRE_OPERATIONAL 하위 상태
 */
typedef enum {
	PRE_OP_IDLE,
    PRE_OP_SEND_PDO_MAP_A,           /**< TPDO1 (Group A) 매핑 전송 */
    PRE_OP_WAIT_PDO_MAP_A_ACK,       /**< TPDO1 응답 대기 */
    PRE_OP_SEND_PDO_MAP_B,           /**< TPDO2 (Group B) 매핑 전송 */
    PRE_OP_WAIT_PDO_MAP_B_ACK,       /**< TPDO2 응답 대기 */
    PRE_OP_SEND_NMT_OPERATIONAL,     /**< NMT OPERATIONAL 명령 전송 */
    PRE_OP_SEND_SYNC_STATES,         /**< IMU Connected Mask 읽기 요청 */
    PRE_OP_WAIT_SYNC_ACK,            /**< IMU Connected Mask 응답 대기 */
    PRE_OP_WAIT_OPERATIONAL_HB       /**< OPERATIONAL Heartbeat 대기 */
} PreOpState_t;

/**
 * @brief IMU Hub Link 인스턴스
 */
typedef struct {
    volatile PreOpState_t  pre_op_state;
    volatile bool          is_synced;              /**< Sync States 완료 여부 */
    volatile uint32_t      last_sdo_sent_ms;       /**< 마지막 SDO 전송 시각 */
    volatile uint8_t       sdo_retry_count;        /**< SDO 재시도 횟수 */
    volatile uint8_t       last_imu_hub_nmt_state; /**< IMU Hub의 마지막 Heartbeat NMT 상태 (ISR에서 업데이트) */
    volatile bool          operational_hb_received;/**< OPERATIONAL Heartbeat 수신 플래그 (ISR → Main Task) */
} ImuHubLinkInst_t;

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

static ImuHubLinkInst_t s_imu_hub_xm_link;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/* AGR_PnPMgr 콜백 */
static void _Link_OnConnected(uint8_t device_id);
static void _Link_OnDisconnected(uint8_t device_id);

/* ImuHub_Drv 콜백 */
static void _Link_OnImuHubBootup(void);
static void _Link_OnImuHubHeartbeat(uint8_t nmt_state);
static void _Link_OnNmtStateChange(uint8_t new_state);
static void _Link_OnSyncStates(void);
static void _Link_OnPdoMappingAck(uint8_t group);

/* PnP State Machine */
static void _Link_RunPreOpStateMachine(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void ImuHub_XM_Link_Init(void)
{
    memset(&s_imu_hub_xm_link, 0, sizeof(ImuHubLinkInst_t));
    s_imu_hub_xm_link.pre_op_state = PRE_OP_IDLE;  /* ✅ 올바른 초기값 (연결 대기) */
    s_imu_hub_xm_link.last_imu_hub_nmt_state = 0;  /* NMT 상태 초기화 */
    s_imu_hub_xm_link.operational_hb_received = false;  /* 플래그 초기화 */
    
    /* 1. AGR_PnPMgr에 IMU Hub Device 등록 */
    static AGR_PnP_Device_t imu_hub_device = {
        .id = AGR_NODE_ID_IMU_HUB,  /* 0x08 */
        .timeout_ms = 5000,          /* 5s 타임아웃 */
        .mode = AGR_PNP_MODE_PROTOCOL_BASED,  /* CANopen 호환 */
        .OnConnected = _Link_OnConnected,
        .OnDisconnected = _Link_OnDisconnected,
        .user_ctx = NULL
    };
    AGR_PnPMgr_RegisterDevice(&imu_hub_device);
    
    /* 2. IMU Hub Driver 초기화 */
    ImuHub_Callbacks_t drv_callbacks = {
        .OnBootup = _Link_OnImuHubBootup,
        .OnHeartbeat = _Link_OnImuHubHeartbeat,
        .OnNmtStateChange = _Link_OnNmtStateChange,
        .OnSyncStates = _Link_OnSyncStates,
        .OnPdoMappingAck = _Link_OnPdoMappingAck
    };
    ImuHub_Drv_Init(System_Fdcan1_Transmit, &drv_callbacks);
}

void ImuHub_XM_Link_RunPeriodic(void)
{
    /* AGR_PnPMgr 상태 확인 */
    AGR_PnP_State_t pnp_state = AGR_PnPMgr_GetState(AGR_NODE_ID_IMU_HUB);
    
    switch (pnp_state) {
        case AGR_PNP_STATE_BOOT_UP:
            /* XM10 Bootup 전송 (1s 주기) */
            {
                static uint32_t last_bootup_ms = 0;
                uint32_t current_ms = IOIF_TIM_GetTick();
                
                if (current_ms - last_bootup_ms > 1000) {
                    ImuHub_Drv_SendBootup();
                    last_bootup_ms = current_ms;
                }
            }
            break;
            
        case AGR_PNP_STATE_PRE_OPERATIONAL:
            /* 복잡한 PnP 시퀀스 (PDO Mapping 등) */
            _Link_RunPreOpStateMachine();
            break;
            
        case AGR_PNP_STATE_OPERATIONAL:
            /* Heartbeat 전송 (1s 주기) */
            {
                static uint32_t last_hb_ms = 0;
                uint32_t current_ms = IOIF_TIM_GetTick();
                
                if (current_ms - last_hb_ms > 1000) {
                    ImuHub_Drv_SendHeartbeat(AGR_PNP_STATE_OPERATIONAL);  /* PnP 상태 사용 */
                    last_hb_ms = current_ms;
                }
            }
            break;
            
        case AGR_PNP_STATE_STOPPED:
            /* 타임아웃 발생 시 자동으로 BOOT_UP으로 전환 */
            AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_BOOT_UP);
            s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_A;
            s_imu_hub_xm_link.is_synced = false;
            s_imu_hub_xm_link.operational_hb_received = false;  /* 플래그 리셋 */
            s_imu_hub_xm_link.last_imu_hub_nmt_state = 0;  /* NMT 상태 리셋 */
            break;
            
        default:
            break;
    }
}

void ImuHub_XM_Link_ProcessMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    /* SDO 메시지 처리 (PnP Manager → Link → Driver) */
    ImuHub_Drv_ProcessCANMessage(can_id, data, len);
}

bool ImuHub_XM_Link_IsConnected(void)
{
    /* OPERATIONAL + Sync 완료 */
    return (AGR_PnPMgr_GetState(AGR_NODE_ID_IMU_HUB) == AGR_PNP_STATE_OPERATIONAL)
           && s_imu_hub_xm_link.is_synced;
}

LinkModule_t* ImuHub_XM_Link_GetModule(void)
{
    static LinkModule_t module = {
        .nodeId = AGR_NODE_ID_IMU_HUB,  /* 0x08 */
        .Init = ImuHub_XM_Link_Init,
        .ProcessMessage = ImuHub_XM_Link_ProcessMessage,
        .RunPeriodic = ImuHub_XM_Link_RunPeriodic,
        .IsConnected = ImuHub_XM_Link_IsConnected
    };
    return &module;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ===== AGR_PnPMgr 콜백 ===== */

static void _Link_OnConnected(uint8_t device_id)
{
    /* 
     * [안전장치] 이미 OPERATIONAL 완료 상태면 무시
     * 
     * AGR_PnPMgr의 _OnNmtStateChanged는 다음 두 전환 시 OnConnected를 호출합니다:
     * - (1) BOOT_UP → PRE_OPERATIONAL (Bootup 수신 시)
     * - (2) PRE_OPERATIONAL → OPERATIONAL (SetState 호출 시) ⚠️
     * 
     * (2)의 경우 pre_op_state가 초기화되면 안 되므로, 방어 로직 추가합니다.
     */
    if (s_imu_hub_xm_link.pre_op_state == PRE_OP_IDLE) {
        /* 이미 OPERATIONAL 완료 → 무시 */
        return;
    }
    
    /* BOOT_UP → PRE_OPERATIONAL 전환 완료 */
    s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_A;  /* ✅ TPDO1부터 시작! */
    s_imu_hub_xm_link.sdo_retry_count = 0;
    s_imu_hub_xm_link.operational_hb_received = false;  /* 플래그 리셋 */
    s_imu_hub_xm_link.last_imu_hub_nmt_state = 0;  /* NMT 상태 리셋 */
    
    /* AGR_PnPMgr 상태 전환 (중복 호출이지만 안전장치) */
    AGR_PnPMgr_SetState(device_id, AGR_PNP_STATE_PRE_OPERATIONAL);
    
    /* [디버깅] 로그 추가 (TODO: printf 또는 LED 표시) */
    // printf("[IMU Hub] Connected! Starting PDO Mapping A\n");
}

static void _Link_OnDisconnected(uint8_t device_id)
{
    /* 타임아웃 발생 → STOPPED */
    s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_A;  /* ⚠️ 다시 연결 시 OnConnected에서 재설정 */
    s_imu_hub_xm_link.is_synced = false;
    s_imu_hub_xm_link.operational_hb_received = false;  /* 플래그 리셋 */
    s_imu_hub_xm_link.last_imu_hub_nmt_state = 0;  /* NMT 상태 리셋 */
    
    /* [디버깅] 로그 추가 (TODO: printf 또는 LED 표시) */
    // printf("[IMU Hub] Disconnected! Timeout detected\n");
}

/* ===== ImuHub_Drv 콜백 ===== */

static void _Link_OnImuHubBootup(void)
{
    /* 
     * IMU Hub Boot-up 수신 → AGR_PnPMgr 활동 갱신
     * 
     * [AGR_NMT 동작]
     * - AGR_NMT_MODE_FULL: BOOT_UP → PRE_OPERATIONAL 자동 전환
     * - _OnNmtStateChanged 콜백: OPERATIONAL 진입 시에만 OnConnected 호출
     * - 따라서 PRE_OPERATIONAL 진입 시 수동으로 _Link_OnConnected 호출 필요
     */
    AGR_PnPMgr_UpdateActivity(AGR_NODE_ID_IMU_HUB);  /* BOOT_UP → PRE_OP 자동 전환 */
    
    /* PRE_OPERATIONAL 전환 확인 후 OnConnected 수동 호출 */
    AGR_PnP_State_t current_state = AGR_PnPMgr_GetState(AGR_NODE_ID_IMU_HUB);
    if (current_state == AGR_PNP_STATE_PRE_OPERATIONAL) {
        _Link_OnConnected(AGR_NODE_ID_IMU_HUB);  /* ✅ pre_op_state = PRE_OP_SEND_PDO_MAP_A */
    }
}

static void _Link_OnImuHubHeartbeat(uint8_t nmt_state)
{
    /* Heartbeat 수신 → 타임아웃 리셋 */
    AGR_PnPMgr_UpdateActivity(AGR_NODE_ID_IMU_HUB);
    
    /* 
     * [ISR Context] Heartbeat NMT 상태 저장 (Main Task에서 확인)
     * - ISR에서는 최소한의 작업만 수행합니다.
     * - 상태 전환은 RunPeriodic에서 수행합니다.
     */
    s_imu_hub_xm_link.last_imu_hub_nmt_state = nmt_state;
    
    /* OPERATIONAL Heartbeat 수신 플래그 설정 */
    if (nmt_state == AGR_PNP_STATE_OPERATIONAL) {
        s_imu_hub_xm_link.operational_hb_received = true;
    }
}

static void _Link_OnNmtStateChange(uint8_t new_state)
{
    /* NMT 상태 변경 알림 (Optional) */
    (void)new_state;
}

static void _Link_OnSyncStates(void)
{
    /* 
     * IMU Connected Mask Upload Response 수신 완료
     * 이제 IMU Hub가 OPERATIONAL Heartbeat를 보낼 때까지 대기
     */
    s_imu_hub_xm_link.is_synced = true;
    s_imu_hub_xm_link.pre_op_state = PRE_OP_WAIT_OPERATIONAL_HB;
    s_imu_hub_xm_link.last_sdo_sent_ms = IOIF_TIM_GetTick();  /* 타임아웃 타이머 시작 */
}

static void _Link_OnPdoMappingAck(uint8_t group)
{
    /* PDO Mapping 설정 완료 응답 */
    if (group == 0) {  /* Group A (TPDO1) → TPDO2 설정으로 전환 */
        s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_B;
        s_imu_hub_xm_link.sdo_retry_count = 0;  /* 재시도 카운터 리셋 */
    } else if (group == 1) {  /* Group B (TPDO2) → NMT OPERATIONAL로 전환 */
        s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_NMT_OPERATIONAL;
        s_imu_hub_xm_link.sdo_retry_count = 0;  /* 재시도 카운터 리셋 */
    }
}

/* ===== PnP State Machine ===== */

static void _Link_RunPreOpStateMachine(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    switch (s_imu_hub_xm_link.pre_op_state) {
        case PRE_OP_SEND_PDO_MAP_A:
        {
            /* TPDO1 (Group A) Mapping 설정 전송: IMU 0,1,2 → {q,a,g} (20B × 3 = 60B) */
            uint8_t pdo_map_a[] = {
                0x03,  /* 3개 항목 */
                0x60, 0x00, 0x60, 0x00,  /* IMU0: 0x6000.0x60 ({q,a,g}, 20B) */
                0x60, 0x01, 0x60, 0x00,  /* IMU1: 0x6001.0x60 */
                0x60, 0x02, 0x60, 0x00,  /* IMU2: 0x6002.0x60 */
            };
            
            ImuHub_Drv_SendSetPDOMapping(0, pdo_map_a, sizeof(pdo_map_a));  /* Group A (0) */
            
            s_imu_hub_xm_link.pre_op_state = PRE_OP_WAIT_PDO_MAP_A_ACK;
            s_imu_hub_xm_link.last_sdo_sent_ms = current_ms;
            break;
        }
        
        case PRE_OP_WAIT_PDO_MAP_A_ACK:
        {
            /* TPDO1 PDO Mapping 응답 대기 (타임아웃: 1s) */
            if (current_ms - s_imu_hub_xm_link.last_sdo_sent_ms > 1000) {
                /* 타임아웃 → 재시도 */
                if (s_imu_hub_xm_link.sdo_retry_count < 3) {
                    s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_A;
                    s_imu_hub_xm_link.sdo_retry_count++;
                } else {
                    /* 실패 → STOPPED */
                    AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_STOPPED);
                    s_imu_hub_xm_link.sdo_retry_count = 0;
                }
            }
            /* OnPdoMappingAck 콜백에서 PRE_OP_SEND_PDO_MAP_B로 전환 */
            break;
        }
        
        case PRE_OP_SEND_PDO_MAP_B:
        {
            /* TPDO2 (Group B) Mapping 설정 전송: IMU 3,4,5 → {q,a,g} (20B × 3 = 60B) */
            uint8_t pdo_map_b[] = {
                0x03,  /* 3개 항목 */
                0x60, 0x03, 0x60, 0x00,  /* IMU3: 0x6003.0x60 ({q,a,g}, 20B) */
                0x60, 0x04, 0x60, 0x00,  /* IMU4: 0x6004.0x60 */
                0x60, 0x05, 0x60, 0x00,  /* IMU5: 0x6005.0x60 */
            };
            
            ImuHub_Drv_SendSetPDOMapping(1, pdo_map_b, sizeof(pdo_map_b));  /* Group B (1) */
            
            s_imu_hub_xm_link.pre_op_state = PRE_OP_WAIT_PDO_MAP_B_ACK;
            s_imu_hub_xm_link.last_sdo_sent_ms = current_ms;
            s_imu_hub_xm_link.sdo_retry_count = 0;  /* TPDO2 재시도 카운터 리셋 */
            break;
        }
        
        case PRE_OP_WAIT_PDO_MAP_B_ACK:
        {
            /* TPDO2 PDO Mapping 응답 대기 (타임아웃: 1s) */
            if (current_ms - s_imu_hub_xm_link.last_sdo_sent_ms > 1000) {
                /* 타임아웃 → 재시도 */
                if (s_imu_hub_xm_link.sdo_retry_count < 3) {
                    s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_PDO_MAP_B;
                    s_imu_hub_xm_link.sdo_retry_count++;
                } else {
                    /* 실패 → STOPPED */
                    AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_STOPPED);
                    s_imu_hub_xm_link.sdo_retry_count = 0;
                }
            }
            /* OnPdoMappingAck 콜백에서 PRE_OP_SEND_NMT_OPERATIONAL로 전환 */
            break;
        }
        
        case PRE_OP_SEND_NMT_OPERATIONAL:
        {
            /* NMT OPERATIONAL 전환 명령 */
            ImuHub_Drv_SendSetNMTState(AGR_PNP_STATE_OPERATIONAL);  /* PnP 상태 사용 */
            
            s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_SYNC_STATES;
            s_imu_hub_xm_link.last_sdo_sent_ms = current_ms;
            break;
        }
        
        case PRE_OP_SEND_SYNC_STATES:
        {
            /* IMU Connected Mask 읽기 요청 (SDO Upload) */
            ImuHub_Drv_SendReadImuConnMask();  /* 0x2001 Upload Request */
            
            s_imu_hub_xm_link.pre_op_state = PRE_OP_WAIT_SYNC_ACK;
            s_imu_hub_xm_link.last_sdo_sent_ms = current_ms;
            s_imu_hub_xm_link.sdo_retry_count = 0;  /* 재시도 카운터 리셋 */
            break;
        }
        
        case PRE_OP_WAIT_SYNC_ACK:
        {
            /* IMU Connected Mask Upload Response 대기 (타임아웃: 1s) */
            if (current_ms - s_imu_hub_xm_link.last_sdo_sent_ms > 1000) {
                /* 타임아웃 → 재시도 */
                if (s_imu_hub_xm_link.sdo_retry_count < 3) {
                    s_imu_hub_xm_link.pre_op_state = PRE_OP_SEND_SYNC_STATES;
                    s_imu_hub_xm_link.sdo_retry_count++;
                } else {
                    /* 실패 → STOPPED */
                    AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_STOPPED);
                    s_imu_hub_xm_link.sdo_retry_count = 0;
                }
            }
            /* OnSyncStates 콜백에서 PRE_OP_WAIT_OPERATIONAL_HB로 전환 */
            break;
        }
        
        case PRE_OP_WAIT_OPERATIONAL_HB:
        {
            /* 
             * [Main Task] OPERATIONAL Heartbeat 수신 확인 (ISR에서 설정된 플래그 확인)
             * 
             * [중요] ISR Context에서 상태를 바로 전환하지 않고,
             * Main Task (RunPeriodic)에서 플래그를 확인하여 전환합니다.
             * 이렇게 하면 상태 전환이 Main Task의 흐름 내에서 이루어져
             * 동기화 문제를 방지할 수 있습니다.
             */
            if (s_imu_hub_xm_link.operational_hb_received) {
                /* ✅ OPERATIONAL Heartbeat 수신 완료! */
                s_imu_hub_xm_link.operational_hb_received = false;  /* 플래그 클리어 */
                
                /* [Step 1] 하위 상태머신 종료 (OnConnected 재호출 방지) */
                s_imu_hub_xm_link.pre_op_state = PRE_OP_IDLE;
                
                /* 
                 * PnP State를 OPERATIONAL로 전환
                 */
                AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_OPERATIONAL);
                
                /* [디버깅] 로그 추가 (TODO: printf 또는 LED 표시) */
                // printf("[XM10] IMU Hub PnP Completed! OPERATIONAL!\n");
                
                break;  /* 상태 전환 완료 후 탈출 */
            }
            
            /* 타임아웃 감시 (3초) */
            if (current_ms - s_imu_hub_xm_link.last_sdo_sent_ms > 3000) {
                /* 3초 타임아웃 → 실패 */
                AGR_PnPMgr_SetState(AGR_NODE_ID_IMU_HUB, AGR_PNP_STATE_STOPPED);
                s_imu_hub_xm_link.sdo_retry_count = 0;
            }
            break;
        }
        
        default:
            break;
    }
}

