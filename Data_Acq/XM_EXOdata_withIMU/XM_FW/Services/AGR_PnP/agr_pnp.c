/**
 ******************************************************************************
 * @file    agr_pnp.c
 * @author  HyundoKim
 * @brief   [Services] PnP Abstraction Layer (OnConnected, OnDisconnected)
 * @version 3.0 (CANopen 표준 준수)
 * @date    Dec 8, 2025
 * 
 * @details
 * CANopen 표준을 기반으로 개발 편의성을 제공하는 추상화 레이어입니다.
 * 
 * [역할]
 * - OnConnected/OnDisconnected 이벤트 추상화
 * - Master: Pre-Op 시퀀스 관리 (SDO Response 처리)
 * - Slave: Boot-up 전송, Heartbeat 전송
 * 
 * [CANopen 표준 메시지는 AGR_NMT, AGR_DOP가 처리]
 * - NMT, Boot-up, Heartbeat → AGR_NMT
 * - SDO, PDO → AGR_DOP
 * 
 * [중요] AGR_PnP는 Optional (제거 가능)
 * - Device Driver에서 직접 AGR_NMT, AGR_DOP 사용 가능
 * - OnConnected/OnDisconnected가 필요한 경우에만 사용
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_pnp.h"
#include "agr_dop.h"  /* SDO Encode/Decode, CAN ID 계산 */
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define AGR_PNP_HEARTBEAT_INTERVAL  1000  /**< Slave Heartbeat 전송 주기 (ms) */

/* CANopen CAN ID 계산 */
#define NMT_CAN_ID          0x000
#define HEARTBEAT_CAN_ID(node_id)  (0x700 + (node_id))

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static AGR_PnP_Device_t* _FindDevice(AGR_PnP_Inst_t* inst, uint8_t node_id);
static int _ProcessSDOResponse(AGR_PnP_Inst_t* inst, uint8_t node_id, const uint8_t* data, uint8_t len);
static void _CheckTimeouts(AGR_PnP_Inst_t* inst);
static void _RunMasterPreOpStateMachine(AGR_PnP_Inst_t* inst, AGR_PnP_Device_t* dev);
static void _OnNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx);
static void _OnNmtTimeout(void* user_ctx);
static void _OnSlaveNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief PnP 인스턴스 초기화 (Primary)
 */
int AGR_PnP_Init(AGR_PnP_Inst_t* inst, const AGR_PnP_Config_t* config, uint32_t (*get_tick)(void))
{
    if (inst == NULL || config == NULL || get_tick == NULL) {
        return -1;  /* Invalid parameter */
    }
    
    if (config->tx_func == NULL) {
        return -2;  /* Tx function required */
    }
    
    /* 초기화 */
    memset(inst, 0, sizeof(AGR_PnP_Inst_t));
    
    /* 설정 복사 */
    memcpy(&inst->config, config, sizeof(AGR_PnP_Config_t));
    inst->get_tick = get_tick;
    
    /* NMT 초기화 (Slave용) */
    if (config->role == AGR_PNP_ROLE_SLAVE) {
        // AGR_NMT_Init(&inst->config.nmt, config->my_node_id, get_tick);
    	AGR_NMT_Init(&inst->config.nmt, config->my_node_id);
    }
    
    inst->is_initialized = true;
    
    return 0;
}

/**
 * @brief PnP 인스턴스 초기화 (Secondary - 이중 역할용)
 */
int AGR_PnP_InitSecondary(AGR_PnP_Inst_t* inst, const AGR_PnP_Config_t* config, uint32_t (*get_tick)(void))
{
    /* Primary와 동일한 로직 (추후 확장 가능) */
    return AGR_PnP_Init(inst, config, get_tick);
}

/**
 * @brief Device 등록 (Master only)
 */
int AGR_PnP_RegisterDevice(AGR_PnP_Inst_t* inst, const AGR_PnP_Device_t* device)
{
    if (inst == NULL || device == NULL) {
        return -1;
    }
    
    if (!inst->is_initialized) {
        return -2;  /* Not initialized */
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return -3;  /* Not a Master */
    }
    
    if (inst->device_count >= AGR_PNP_MAX_DEVICES) {
        return -4;  /* Too many devices */
    }
    
    /* Device 추가 */
    AGR_PnP_Device_t* new_dev = &inst->devices[inst->device_count];
    memcpy(new_dev, device, sizeof(AGR_PnP_Device_t));
    
    /* NMT 초기화 (콜백 포함) */
    uint32_t timeout = (device->heartbeat_timeout > 0) ? device->heartbeat_timeout : AGR_PNP_HEARTBEAT_TIMEOUT;
    AGR_NMT_InitEx(&new_dev->nmt, 
                   timeout, 
                   device->node_id, 
                   _OnNmtStateChanged,  /* ✅ 상태 변경 콜백 */
                   _OnNmtTimeout,       /* ✅ 타임아웃 콜백 */
                   new_dev);            /* ✅ user_ctx로 AGR_PnP_Device_t 전달 */
    
    new_dev->is_registered = true;
    inst->device_count++;
    
    return inst->device_count - 1;  /* ✅ Device Index 반환 */
}

/**
 * @brief Slave 자신의 NMT 등록 (Slave only)
 */
void AGR_PnP_SetMyNmt(AGR_PnP_Inst_t* inst, AGR_NMT_Inst_t* my_nmt)
{
    if (inst != NULL) {
        inst->my_nmt = my_nmt;
    }
}

/**
 * @brief Master 등록 (Slave only)
 */
int AGR_PnP_RegisterMaster(AGR_PnP_Inst_t* inst, const AGR_PnP_Master_t* master)
{
    if (inst == NULL || master == NULL) {
        return -1;
    }
    
    if (!inst->is_initialized) {
        return -2;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return -3;  /* Not a Slave */
    }
    
    /* Master 정보 저장 */
    memcpy(&inst->config.master, master, sizeof(AGR_PnP_Master_t));
    
    /* ✅ Master NMT 모니터링 시작 (Heartbeat 수신) */
    AGR_NMT_InitEx(&inst->config.nmt, 
                   master->heartbeat_timeout,    /* timeout_ms */
                   master->node_id,              /* node_id */
                   _OnSlaveNmtStateChanged,      /* on_state_changed */
                   NULL,                         /* on_timeout */
                   inst);                        /* user_ctx */
    
    return 0;
}

/**
 * @brief NMT 명령 전송 (Master only)
 */
int AGR_PnP_SendNmtCommand(AGR_PnP_Inst_t* inst, uint8_t node_id, AGR_NMT_Cmd_t nmt_cmd)
{
    if (inst == NULL || !inst->is_initialized) {
        return -1;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return -2;  /* Not a Master */
    }
    
    if (inst->config.tx_func == NULL) {
        return -3;  /* Tx function not set */
    }
    
    /* NMT 명령 CAN 메시지 전송 */
    /* CANopen 표준: CAN ID = 0x000, Data = [cs, node_id] */
    uint8_t nmt_msg[2] = {
        (uint8_t)nmt_cmd,  /* Command Specifier */
        node_id            /* Target Node ID (0 = Broadcast) */
    };
    
    return inst->config.tx_func(NMT_CAN_ID, nmt_msg, 2);
}

/**
 * @brief SDO Write 요청 전송 (Master only)
 */
int AGR_PnP_SendSDOWrite(AGR_PnP_Inst_t* inst, 
                         uint8_t node_id, 
                         uint16_t index, 
                         uint8_t subindex, 
                         const uint8_t* data, 
                         uint8_t data_len)
{
    if (inst == NULL || !inst->is_initialized || data == NULL) {
        return -1;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return -2;  /* Not a Master */
    }
    
    if (inst->config.tx_func == NULL) {
        return -3;  /* Tx function not set */
    }
    
    if (data_len > AGR_SDO_MAX_DATA_SIZE) {
        return -4;  /* Data too large */
    }
    
    /* SDO Download Initiate Request 생성 */
    AGR_SDO_Msg_t sdo_req = {
        .cs = AGR_SDO_CS_DOWNLOAD_INIT_REQ,  /* 0x20 */
        .index = index,
        .subindex = subindex,
        .data_len = data_len
    };
    memcpy(sdo_req.data, data, data_len);
    
    /* SDO 인코딩 */
    uint8_t sdo_buf[8];
    int encoded_len = AGR_DOP_EncodeSDO(&sdo_req, sdo_buf);
    if (encoded_len <= 0) {
        return -5;  /* Encode failed */
    }
    
    /* CAN 전송 (SDO Request: 0x600 + node_id) */
    uint32_t can_id = AGR_DOP_GetSDORequestCANID(node_id);
    return inst->config.tx_func(can_id, sdo_buf, (uint8_t)encoded_len);
}

/**
 * @brief SDO Read 요청 전송 (Master only)
 */
int AGR_PnP_SendSDORead(AGR_PnP_Inst_t* inst, 
                        uint8_t node_id, 
                        uint16_t index, 
                        uint8_t subindex)
{
    if (inst == NULL || !inst->is_initialized) {
        return -1;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return -2;  /* Not a Master */
    }
    
    if (inst->config.tx_func == NULL) {
        return -3;  /* Tx function not set */
    }
    
    /* SDO Upload Initiate Request 생성 */
    AGR_SDO_Msg_t sdo_req = {
        .cs = AGR_SDO_CS_UPLOAD_INIT_REQ,  /* 0x40 */
        .index = index,
        .subindex = subindex,
        .data_len = 0  /* Upload Request는 데이터 없음 */
    };
    
    /* SDO 인코딩 */
    uint8_t sdo_buf[8];
    int encoded_len = AGR_DOP_EncodeSDO(&sdo_req, sdo_buf);
    if (encoded_len <= 0) {
        return -4;  /* Encode failed */
    }
    
    /* CAN 전송 (SDO Request: 0x600 + node_id) */
    uint32_t can_id = AGR_DOP_GetSDORequestCANID(node_id);
    return inst->config.tx_func(can_id, sdo_buf, (uint8_t)encoded_len);
}

/* AGR_PnP_SetDevicePDOMapping 함수 제거됨 (Link Layer가 직접 AGR_PnP_SendSDOWrite 사용) */

/**
 * @brief Device 연결 상태 확인 (Master only)
 */
bool AGR_PnP_IsDeviceConnected(const AGR_PnP_Inst_t* inst, uint8_t node_id)
{
    if (inst == NULL || !inst->is_initialized) {
        return false;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return false;
    }
    
    /* Device 찾기 */
    for (uint8_t i = 0; i < inst->device_count; i++) {
        if (inst->devices[i].node_id == node_id && inst->devices[i].is_registered) {
            const AGR_NMT_Inst_t* nmt = &inst->devices[i].nmt;
            uint32_t current_time = inst->get_tick();
            
            /* NMT 상태가 OPERATIONAL이고 Heartbeat가 유효하면 연결됨 */
            bool is_timeout = (nmt->timeout_ms > 0) && 
                              ((current_time - nmt->last_activity_ms) > nmt->timeout_ms);
            
            return (nmt->state == AGR_NMT_OPERATIONAL) && !is_timeout;
        }
    }
    
    return false;
}

/**
 * @brief Device NMT 상태 조회 (Master only)
 */
AGR_NMT_State_t AGR_PnP_GetDeviceState(const AGR_PnP_Inst_t* inst, uint8_t node_id)
{
    if (inst == NULL || !inst->is_initialized) {
        return AGR_NMT_BOOT_UP;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_MASTER) {
        return AGR_NMT_BOOT_UP;
    }
    
    /* Device 찾기 */
    for (uint8_t i = 0; i < inst->device_count; i++) {
        if (inst->devices[i].node_id == node_id && inst->devices[i].is_registered) {
            return inst->devices[i].nmt.state;
        }
    }
    
    return AGR_NMT_BOOT_UP;
}

/**
 * @brief Master 연결 상태 확인 (Slave only)
 */
bool AGR_PnP_IsMasterConnected(const AGR_PnP_Inst_t* inst)
{
    if (inst == NULL || !inst->is_initialized) {
        return false;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return false;
    }
    
    /* ✅ Master가 연결되어 있으면 true (Slave 자신의 NMT 상태는 무관) */
    /* 주의: Slave가 OPERATIONAL이어야 PDO 전송이 의미 있음 */
    return inst->is_master_connected;
}

/**
 * @brief 자신의 NMT 상태 조회 (Slave only)
 */
AGR_NMT_State_t AGR_PnP_GetMyState(const AGR_PnP_Inst_t* inst)
{
    if (inst == NULL || !inst->is_initialized) {
        return AGR_NMT_BOOT_UP;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return AGR_NMT_BOOT_UP;
    }
    
    return inst->config.nmt.state;
}

/**
 * @brief Boot-up 전송 (Slave only)
 */
int AGR_PnP_SendBootup(AGR_PnP_Inst_t* inst)
{
    if (inst == NULL || !inst->is_initialized) {
        return -1;
    }
    
    if (inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return -2;  /* Not a Slave */
    }
    
    if (inst->config.tx_func == NULL) {
        return -3;  /* Tx function not set */
    }
    
    /* NMT 상태를 BOOT_UP으로 설정 */
    AGR_NMT_SetState(&inst->config.nmt, AGR_NMT_BOOT_UP);
    
    /* Boot-up CAN 메시지 전송 */
    /* CANopen 표준: CAN ID = 0x700 + node_id, Data = [0x00] */
    uint8_t bootup_msg[1] = { 0x00 };  /* Boot-up state */
    uint32_t can_id = HEARTBEAT_CAN_ID(inst->config.my_node_id);
    
    return inst->config.tx_func(can_id, bootup_msg, 1);
}

/**
 * @brief Heartbeat 전송 (Master/Slave 모두 가능)
 * @note CANopen 표준: 모든 노드가 Heartbeat Producer 가능
 */
int AGR_PnP_SendHeartbeat(AGR_PnP_Inst_t* inst)
{
    if (inst == NULL || !inst->is_initialized) {
        return -1;
    }
    
    if (inst->config.tx_func == NULL) {
        return -3;  /* Tx function not set */
    }
    
    /* Heartbeat CAN 메시지 전송 */
    /* CANopen 표준: CAN ID = 0x700 + node_id, Data = [current_nmt_state] */
    uint8_t nmt_state;
    
    if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
        /* ✅ Slave는 자신의 NMT 상태 전송 (my_nmt가 등록되어 있으면 사용) */
        nmt_state = (inst->my_nmt != NULL) 
                    ? (uint8_t)inst->my_nmt->state 
                    : (uint8_t)inst->config.nmt.state;  /* Fallback */
    } else {
        /* Master는 항상 OPERATIONAL로 전송 */
        nmt_state = AGR_NMT_OPERATIONAL;
    }
    
    uint8_t hb_msg[1] = { nmt_state };
    uint32_t can_id = HEARTBEAT_CAN_ID(inst->config.my_node_id);
    
    return inst->config.tx_func(can_id, hb_msg, 1);
}

/**
 * @brief CAN 메시지 처리 (Legacy - AGR_NMT/AGR_DOP 사용 권장)
 * 
 * @deprecated 이 함수는 하위 호환성을 위해 유지됩니다.
 * 
 * [권장 사항]
 * Device Driver에서 직접 AGR_NMT, AGR_DOP 사용:
 * - AGR_NMT_ProcessMessage() → NMT, Boot-up, Heartbeat
 * - AGR_DOP_ProcessRxMessage() → SDO, PDO
 * 
 * [현재 처리 항목]
 * - SDO Response (0x580) - Master Pre-Op 시퀀스용
 * - SDO Request (0x600) - Slave (하지만 AGR_DOP 사용 권장)
 * 
 * @note NMT, Boot-up, Heartbeat는 AGR_NMT가 처리하므로 제거됨
 */
int AGR_PnP_ProcessMessage(AGR_PnP_Inst_t* inst, uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (inst == NULL || !inst->is_initialized || data == NULL || len == 0) {
        return -1;
    }
    
    /* ===== [Deprecated] NMT, Heartbeat는 AGR_NMT가 처리 ===== */
    /* AGR_NMT_ProcessMessage()를 사용하세요 */
    
    /* SDO Response (0x580 + node_id) - Master only */
    if ((can_id & 0x780) == 0x580 && len >= 4) {
        if (inst->config.role == AGR_PNP_ROLE_MASTER) {
            uint8_t node_id = can_id & 0x7F;
            return _ProcessSDOResponse(inst, node_id, data, len);
        }
    }
    
    /* ===== [Deprecated] SDO Request는 AGR_DOP가 처리 ===== */
    /* AGR_DOP_ProcessRxMessage()를 사용하세요 */
    
    return -2;  /* Not processed - Use AGR_NMT or AGR_DOP */
}

/**
 * @brief 주기 실행
 */
void AGR_PnP_RunPeriodic(AGR_PnP_Inst_t* inst)
{
    if (inst == NULL || !inst->is_initialized) {
        return;
    }
    
    /* Timeout 체크 */
    _CheckTimeouts(inst);
    
    /* Master: Device State Machine 실행 + Heartbeat 전송 */
    if (inst->config.role == AGR_PNP_ROLE_MASTER) {
        /* 1. Device State Machine */
        for (uint8_t i = 0; i < inst->device_count; i++) {
            AGR_PnP_Device_t* dev = &inst->devices[i];
            if (!dev->is_registered) continue;
            
            /* Protocol-based: Pre-Op State Machine */
            if (dev->nmt.state == AGR_NMT_PRE_OPERATIONAL) {
                _RunMasterPreOpStateMachine(inst, dev);
            }
        }
        
        /* 2. Master Heartbeat 전송 (CANopen 표준) */
        uint32_t current_time = inst->get_tick();
        if (current_time - inst->last_hb_sent_time >= AGR_PNP_HEARTBEAT_INTERVAL) {
            /* Master는 항상 Heartbeat 전송 (Slave가 연결 상태 모니터링) */
			int result = AGR_PnP_SendHeartbeat(inst);
			if (result >= 0) {
				/* ✅ 전송 성공 시에만 타임스탬프 갱신 */
				inst->last_hb_sent_time = current_time;
			}
			/* ❌ 전송 실패 시: 타임스탬프 갱신 안함 → 다음 주기에 재시도 */
        }
    }
    
    /* Slave: Heartbeat/Boot-up 전송 (주기적) */
    if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
        /* 1초마다 Heartbeat 또는 Boot-up 전송 */
        uint32_t current_time = inst->get_tick();
        
        if (current_time - inst->last_hb_sent_time >= AGR_PNP_HEARTBEAT_INTERVAL) {
            int result = -1;
            /* ✅ Slave 자신의 NMT 상태 확인 (my_nmt 사용) */
            AGR_NMT_State_t my_state = inst->my_nmt->state;
            
            if (my_state == AGR_NMT_BOOT_UP) {
                /* BOOT_UP 상태 + Master 미연결 → Boot-up 재전송 */
                if (!inst->is_master_connected) {
                    result = AGR_PnP_SendBootup(inst);
                }
            } else {
                /* PRE_OP/OPERATIONAL 상태 → Heartbeat 전송 */
                result = AGR_PnP_SendHeartbeat(inst);
            }
            
            /* ✅ 전송 성공 시에만 타임스탬프 갱신 */
            if (result >= 0) {
                inst->last_hb_sent_time = current_time;
            }
            /* ❌ 전송 실패 시: 타임스탬프 갱신 안함 → 다음 주기에 재시도 */
        }
    }
}

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief Device 찾기
 */
static AGR_PnP_Device_t* _FindDevice(AGR_PnP_Inst_t* inst, uint8_t node_id)
{
    for (uint8_t i = 0; i < inst->device_count; i++) {
        if (inst->devices[i].node_id == node_id && inst->devices[i].is_registered) {
            return &inst->devices[i];
        }
    }
    return NULL;
}

/**
 * @brief SDO Response 처리 (Master only)
 * @details Slave로부터 SDO Response 수신 시 호출
 */
static int _ProcessSDOResponse(AGR_PnP_Inst_t* inst, uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (inst == NULL || data == NULL || len < 4) {
        return -1;
    }
    
    /* Device 찾기 */
    AGR_PnP_Device_t* dev = _FindDevice(inst, node_id);
    if (dev == NULL) {
        return -2;  /* Device not registered */
    }
    
    /* SDO 디코딩 */
    AGR_SDO_Msg_t sdo_msg;
    if (AGR_DOP_DecodeSDO(data, len, &sdo_msg) != 0) {
        return -3;  /* Decode failed */
    }
    
    /* SDO Command Specifier 확인 */
    uint8_t cs = sdo_msg.cs & 0xE0;
    
    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP || cs == AGR_SDO_CS_UPLOAD_INIT_RSP) {
        /* [Deprecated] SDO Response는 AGR_DOP가 처리합니다 */
        (void)sdo_msg;
    } 
    else if (sdo_msg.cs == 0x80) {
        /* Abort (Error) */
        AGR_PnP_RecoveryAction_t recovery = AGR_PNP_RECOVERY_ABORT;
        if (dev->callbacks.on_error != NULL) {
            recovery = dev->callbacks.on_error(node_id, AGR_PNP_ERROR_SDO_ABORT);
        }
        
        /* Recovery 전략에 따라 처리 (Link Layer가 결정) */
        (void)recovery;  /* Link Layer가 Pre-Op 로직 제어 */
    }
    
    return 0;
}

/**
 * @brief Master Pre-Op State Machine 실행 (Protocol-based Device)
 */
/**
 * @brief Master Pre-Op 콜백 호출 (Protocol-based)
 * @note Link Layer에서 Device별 Pre-Op 로직 구현
 *       AGR_PnP는 단순히 PRE_OPERATIONAL 상태일 때 이 함수 호출
 *       Link Layer가 완료 시 AGR_PnP_SendNmtCommand(START) 호출
 */
static void _RunMasterPreOpStateMachine(AGR_PnP_Inst_t* inst, AGR_PnP_Device_t* dev)
{
    /* Link Layer에게 Pre-Op 로직 실행 요청 */
    if (dev->callbacks.on_run_pre_op != NULL) {
        dev->callbacks.on_run_pre_op(dev->node_id, inst);
    }
}

/**
 * @brief Timeout 체크
 */
static void _CheckTimeouts(AGR_PnP_Inst_t* inst)
{
    uint32_t current_time = inst->get_tick();
    
    /* Master: Device Timeout 체크 */
    if (inst->config.role == AGR_PNP_ROLE_MASTER) {
        for (uint8_t i = 0; i < inst->device_count; i++) {
            AGR_PnP_Device_t* dev = &inst->devices[i];
            if (!dev->is_registered) continue;
            
            /* Timeout 체크 전 연결 상태 확인 */
            bool was_connected = (dev->nmt.state == AGR_NMT_OPERATIONAL) && 
                                 (current_time - dev->nmt.last_activity_ms <= dev->heartbeat_timeout);
            
            /* Timeout 여부 업데이트 */
            AGR_NMT_CheckTimeout(&dev->nmt, current_time);
            
            /* Timeout 체크 후 연결 상태 확인 */
            bool is_connected = (dev->nmt.state == AGR_NMT_OPERATIONAL) && 
                                (current_time - dev->nmt.last_activity_ms <= dev->heartbeat_timeout);
            
            /* 연결 → 끊김 */
            if (was_connected && !is_connected) {
                if (dev->callbacks.on_disconnected != NULL) {
                    dev->callbacks.on_disconnected(dev->node_id);
                }
            }
            
            /* 끊김 → 연결 */
            if (!was_connected && is_connected &&
                dev->nmt.state == AGR_NMT_OPERATIONAL) {
                if (dev->callbacks.on_connected != NULL) {
                    dev->callbacks.on_connected(dev->node_id);
                }
            }
        }
    }
    
    /* Slave: Master Timeout 체크 */
    if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
        bool was_connected = inst->is_master_connected;
        
        if (current_time - inst->last_master_hb_time > inst->config.master.heartbeat_timeout) {
            inst->is_master_connected = false;
        }
        
        /* 연결 → 끊김 */
        if (was_connected && !inst->is_master_connected) {
            if (inst->config.callbacks.on_disconnected != NULL) {
                inst->config.callbacks.on_disconnected(inst->config.master.node_id);
            }
        }
        
        /* 끊김 → 연결 */
        if (!was_connected && inst->is_master_connected &&
            inst->config.nmt.state == AGR_NMT_OPERATIONAL) {
            if (inst->config.callbacks.on_connected != NULL) {
                inst->config.callbacks.on_connected(inst->config.master.node_id);
            }
        }
    }
}

/**
 * ===================================================================
 * AGR_PnP 내부 핸들러 (콜백 체인의 Bridge 역할)
 * ===================================================================
 * 
 * [역할]
 * - AGR_NMT 콜백을 받아서 AGR_PnP_Device_t 콜백으로 전파
 * - AGR_NMT는 범용 모듈 (PnP 몰라도 됨)
 * - AGR_PnP는 NMT 위에서 동작 (중간 연결 필요)
 * 
 * [콜백 흐름]
 * [1] AGR_NMT → [2] _OnNmtStateChanged → [3] Device Driver 콜백
 */

/**
 * @brief NMT 상태 변경 콜백 (Bridge)
 * @details AGR_NMT → AGR_PnP → Device Driver 콜백 체인
 * 
 * [호출 경로]
 * AGR_NMT_ProcessMessage → AGR_NMT_UpdateActivity → _NotifyStateChange
 *   → _OnNmtStateChanged (여기!) → dev->callbacks.on_bootup / on_nmt_change
 */
static void _OnNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx)
{
    AGR_PnP_Device_t* dev = (AGR_PnP_Device_t*)user_ctx;
    if (dev == NULL) {
        return;
    }
    
    /* Boot-up 진입 → on_bootup 콜백 (Master only) */
    if (new_state == AGR_NMT_BOOT_UP && old_state != AGR_NMT_BOOT_UP) {
        if (dev->callbacks.on_bootup != NULL) {
            dev->callbacks.on_bootup(dev->node_id);  /* ✅ Pre-Op 초기화 */
        }
    }
    
    /* 모든 상태 변경 → on_nmt_change 콜백 */
    if (dev->callbacks.on_nmt_change != NULL) {
        dev->callbacks.on_nmt_change(dev->node_id, old_state, new_state);  /* ✅ Pre-Op State Machine */
    }
}

/**
 * @brief NMT Timeout 콜백 (Bridge)
 * @details AGR_NMT → AGR_PnP → Device Driver 에러 콜백
 * 
 * [호출 경로]
 * AGR_NMT_CheckTimeout → _OnNmtTimeout (여기!) → dev->callbacks.on_error
 */
static void _OnNmtTimeout(void* user_ctx)
{
    AGR_PnP_Device_t* dev = (AGR_PnP_Device_t*)user_ctx;
    if (dev == NULL) {
        return;
    }
    
    /* Heartbeat Timeout → on_error 콜백 */
    if (dev->callbacks.on_error != NULL) {
        dev->callbacks.on_error(dev->node_id, AGR_PNP_ERROR_HEARTBEAT_TIMEOUT);
    }
}

/**
 * @brief Slave용 NMT 상태 변경 콜백 (Master Heartbeat 수신)
 * @details Slave가 Master의 Heartbeat/Boot-up을 받았을 때 시간 갱신
 */
static void _OnSlaveNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx)
{
    AGR_PnP_Inst_t* inst = (AGR_PnP_Inst_t*)user_ctx;
    if (inst == NULL || inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return;
    }
    
    /* Master의 Heartbeat/Boot-up 수신 → 연결 시간 갱신 */
    inst->last_master_hb_time = inst->get_tick();
    inst->is_master_connected = true;
    
    (void)old_state;
    (void)new_state;
}

