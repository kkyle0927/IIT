/**
 ******************************************************************************
 * @file    agr_pnp.h
 * @author  HyundoKim
 * @brief   [Services] 통합 PnP + NMT API (CANopen 표준 준수)
 * @version 2.0
 * @date    Dec 8, 2025
 *
 * @details
 * CANopen NMT 표준을 준수하면서 Angel Robotics 프로젝트에 최적화된
 * 통합 PnP + NMT API입니다.
 * 
 * [주요 특징]
 * - CANopen NMT 표준 준수 (Boot-up, Pre-Operational, Operational, Stopped)
 * - Master/Slave 역할 통합 지원
 * - RTOS/BareMetal 투명성
 * - Protocol-based (FDCAN+DOP) vs Stream-based (UART) 구분
 * - 이중 역할 (Dual Role) 지원 (e.g., IMU Hub)
 * 
 * [아키텍처]
 * - 기존 AGR_NMT를 내부적으로 사용
 * - AGR_PnPMgr 기능을 흡수하여 통합
 * - 재사용성과 직관성 극대화
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef AGR_PNP_H
#define AGR_PNP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "agr_nmt.h"       /* 내부적으로 NMT 사용 */
#include "agr_dop_types.h" /* SDO/PDO 타입 정의 */

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define AGR_PNP_MAX_DEVICES        16  /**< Master가 관리할 수 있는 최대 Device 수 */
#define AGR_PNP_HEARTBEAT_TIMEOUT  3000 /**< Heartbeat Timeout (ms) - 기본값 */

/* ===== SDO 관련 설정 ===== */
#define AGR_PNP_SDO_TIMEOUT_MS     1000 /**< SDO 응답 대기 Timeout (ms) */
#define AGR_PNP_MAX_SDO_RETRIES    3    /**< SDO 최대 재시도 횟수 */
#define AGR_PNP_PDO_MAP_MAX_SIZE   64   /**< PDO Mapping 데이터 최대 크기 (bytes) */

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief CAN 전송 함수 타입 (Dependency Injection)
 * @param can_id CAN ID
 * @param data 데이터 포인터
 * @param len 데이터 길이
 * @return 0: 성공, <0: 실패
 */
typedef int (*AGR_PnP_TxFunc_t)(uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 * @brief PnP 역할 (Master vs Slave)
 */
typedef enum {
    AGR_PNP_ROLE_SLAVE  = 0,  /**< Slave: Master에 연결 */
    AGR_PNP_ROLE_MASTER = 1   /**< Master: Devices 관리 */
} AGR_PnP_Role_t;

/**
 * @brief PnP 에러 코드
 */
typedef enum {
    AGR_PNP_ERROR_NONE              = 0,  /**< 에러 없음 */
    AGR_PNP_ERROR_SDO_TIMEOUT       = 1,  /**< SDO 응답 타임아웃 */
    AGR_PNP_ERROR_SDO_ABORT         = 2,  /**< SDO Abort 수신 */
    AGR_PNP_ERROR_HEARTBEAT_TIMEOUT = 3,  /**< Heartbeat 타임아웃 */
    AGR_PNP_ERROR_INVALID_STATE     = 4,  /**< 잘못된 상태 전환 */
    AGR_PNP_ERROR_QUEUE_FULL        = 5   /**< SDO Queue Full */
} AGR_PnP_Error_t;

/**
 * @brief PnP 상태 (CANopen NMT + 연결 플래그)
 */
typedef struct {
    AGR_NMT_State_t nmt_state;  /**< CANopen NMT 상태 */
    bool            is_connected; /**< 최종 연결 완료 여부 (OPERATIONAL + Heartbeat OK) */
} AGR_PnP_State_t;

/**
 * @brief Error Recovery 전략
 */
typedef enum {
    AGR_PNP_RECOVERY_RETRY,  /**< 재시도 */
    AGR_PNP_RECOVERY_SKIP,   /**< 건너뛰기 (다음 단계로) */
    AGR_PNP_RECOVERY_ABORT   /**< 포기 (PRE_OP_IDLE로) */
} AGR_PnP_RecoveryAction_t;

/* Forward declaration for AGR_PnP_Inst_t */
typedef struct AGR_PnP_Inst_s AGR_PnP_Inst_t;

/**
 * @brief Error 콜백 함수 타입
 */
typedef AGR_PnP_RecoveryAction_t (*AGR_PnP_OnError_t)(uint8_t node_id, AGR_PnP_Error_t error_code);

/**
 * @brief Pre-Operational 단계 실행 콜백 (Master only, Protocol-based)
 * @param node_id Device Node ID
 * @param inst AGR_PnP 인스턴스 (SDO 전송용)
 * @details 
 * - Link Layer에서 Device별 Pre-Op 로직 구현
 * - AGR_PnP_SendSDOWrite/Read를 사용하여 SDO 전송
 * - 완료 시 AGR_PnP_SendNmtCommand(START) 호출
 * - AGR_PnP는 단순히 PRE_OPERATIONAL 상태일 때 이 콜백 호출
 */
typedef void (*AGR_PnP_OnRunPreOp_t)(uint8_t node_id, AGR_PnP_Inst_t* inst);

/**
 * @brief PnP 콜백 (Device Driver에서 구현)
 * 
 * @details
 * [레이어별 역할 분리]
 * - [1] AGR_NMT: CANopen NMT 프로토콜 처리 (Boot-up, Heartbeat, NMT Command)
 * - [2] AGR_DOP: CANopen SDO/PDO 프로토콜 처리 (데이터 통신)
 * - [3] AGR_PnP: PnP 추상화 (State Machine, Timeout 관리)
 * - [4] Device Driver: 응용 로직 (Pre-Op 시퀀스, 데이터 변환)
 * 
 * [콜백 호출 흐름]
 * AGR_NMT (상태 변경) → AGR_PnP 내부 핸들러 → Device Driver 콜백
 * 
 * [Master vs Slave 사용 콜백]
 * - Master: on_bootup, on_nmt_change, on_run_pre_op, on_error
 * - Slave: on_connected, on_disconnected, on_error
 */
typedef struct {
    /* ===== Master 전용 콜백 ===== */
    void (*on_bootup)(uint8_t node_id);        /**< Slave Boot-up 수신 시 (Pre-Op 초기화) */
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state); /**< Slave NMT 상태 변경 시 */
    AGR_PnP_OnRunPreOp_t on_run_pre_op;        /**< Pre-Op 단계 실행 (Step Array 실행) */
    
    /* ===== Slave 전용 콜백 ===== */
    void (*on_connected)(uint8_t node_id);     /**< Master 연결 완료 (OPERATIONAL 진입) */
    void (*on_disconnected)(uint8_t node_id);  /**< Master 연결 끊김 (Heartbeat Timeout) */
    
    /* ===== 공통 콜백 ===== */
    AGR_PnP_OnError_t on_error;                /**< 에러 발생 (Timeout, SDO Abort 등) */
} AGR_PnP_Callbacks_t;

/**
 * @brief Pre-Operational 진행 상태 (Link Layer 전용)
 * @note AGR_PnP는 이 enum을 사용하지 않습니다.
 *       Link Layer에서 자유롭게 정의하여 사용하세요.
 * 
 * @example
 * // IMU Hub용 Pre-Op 상태
 * typedef enum {
 *     IMUHUB_PRE_OP_IDLE,
 *     IMUHUB_PRE_OP_SEND_PDO_MAP_A,
 *     IMUHUB_PRE_OP_WAIT_PDO_MAP_A,
 *     IMUHUB_PRE_OP_SEND_PDO_MAP_B,
 *     IMUHUB_PRE_OP_WAIT_PDO_MAP_B,
 *     IMUHUB_PRE_OP_SEND_NMT_START
 * } ImuHub_PreOpState_t;
 */
typedef enum {
    PRE_OP_IDLE          = 0,  /**< 대기 (Boot-up 전 또는 완료 후) */
    PRE_OP_SEND_SDO_A    = 1,  /**< SDO Group A 전송 */
    PRE_OP_WAIT_SDO_A    = 2,  /**< SDO Group A 응답 대기 */
    PRE_OP_SEND_SDO_B    = 3,  /**< SDO Group B 전송 */
    PRE_OP_WAIT_SDO_B    = 4,  /**< SDO Group B 응답 대기 */
    PRE_OP_SEND_NMT_START = 5, /**< NMT START 전송 */
    PRE_OP_COMPLETE      = 6   /**< Pre-Op 완료 (OPERATIONAL로 전환 대기) */
} AGR_PnP_PreOpState_t;

/**
 * @brief Device 정보 (Master가 관리하는 Slave)
 * @note Pre-Op 시퀀스, PDO Mapping 등 Device별 로직은 Link Layer에서 관리합니다.
 *       AGR_PnP_Device_t는 NMT 상태와 Heartbeat만 관리합니다.
 * @note AGR_PnP는 CANopen Protocol 전용입니다 (FDCAN+DOP).
 *       UART 센서는 Device Layer에서 직접 타임아웃 관리.
 */
typedef struct {
    const char*       name;              /**< Device 이름 (디버깅용, e.g., "IMU Hub") */
    uint8_t           node_id;           /**< CANopen Node ID */
    uint32_t          heartbeat_timeout; /**< Heartbeat Timeout (ms) */
    AGR_PnP_Callbacks_t callbacks;       /**< 이벤트 콜백 */
    
    /* 내부 상태 (Private) */
    AGR_NMT_Inst_t         nmt;          /**< NMT 인스턴스 */
    bool                   is_registered; /**< 등록 완료? */
} AGR_PnP_Device_t;

/**
 * @brief Master 정보 (Slave가 연결할 Master)
 */
typedef struct {
    uint8_t  node_id;           /**< Master의 CANopen Node ID */
    uint32_t heartbeat_timeout; /**< Master Heartbeat Timeout (ms) */
} AGR_PnP_Master_t;

/**
 * @brief PnP 설정
 */
typedef struct {
    AGR_PnP_Role_t      role;           /**< Master vs Slave */
    uint8_t             my_node_id;     /**< 자신의 CANopen Node ID */
    AGR_PnP_TxFunc_t    tx_func;        /**< CAN 전송 함수 (Dependency Injection) */
    AGR_PnP_Callbacks_t callbacks;      /**< 이벤트 콜백 (Slave용) */
    
    /* Slave 전용 */
    AGR_PnP_Master_t    master;         /**< 연결할 Master 정보 */
    
    /* 내부 상태 (Private) */
    AGR_NMT_Inst_t      nmt;            /**< 자신의 NMT 인스턴스 (Slave용) */
} AGR_PnP_Config_t;

/**
 * @brief PnP 인스턴스 (Primary or Secondary)
 * @note Pre-Op 시퀀스는 Link Layer에서 관리합니다.
 *       AGR_PnP는 NMT 상태와 Heartbeat만 관리합니다.
 */
struct AGR_PnP_Inst_s {
    AGR_PnP_Config_t    config;         /**< 설정 */
    
    /* Master 전용: Device 관리 */
    AGR_PnP_Device_t    devices[AGR_PNP_MAX_DEVICES]; /**< 등록된 Devices */
    uint8_t             device_count;   /**< 등록된 Device 수 */
    
    /* Slave 전용: Master 연결 상태 */
    bool                is_master_connected; /**< Master와 연결 완료? */
    uint32_t            last_master_hb_time; /**< 마지막 Master Heartbeat 수신 시간 */
    uint32_t            last_hb_sent_time;   /**< 마지막 Heartbeat 전송 시간 (Slave) */
    AGR_NMT_Inst_t*     my_nmt;         /**< ✅ Slave 자신의 NMT 포인터 (Heartbeat 전송용) */
    
    /* 공통 */
    bool                is_initialized; /**< 초기화 완료? */
    uint32_t (*get_tick)(void);         /**< Tick 함수 포인터 */
};

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* ===== 초기화 및 설정 ===== */

/**
 * @brief PnP 인스턴스 초기화 (Primary)
 * @param inst PnP 인스턴스
 * @param config 설정 (Role, Node ID, Callbacks)
 * @param get_tick Tick 함수 포인터 (e.g., IOIF_TIM_GetTick)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_Init(AGR_PnP_Inst_t* inst, const AGR_PnP_Config_t* config, uint32_t (*get_tick)(void));

/**
 * @brief PnP 인스턴스 초기화 (Secondary - 이중 역할용)
 * @details IMU Hub처럼 Slave(XM)이면서 Master(EBIMU)인 경우 사용
 * @param inst PnP 인스턴스 (Secondary)
 * @param config 설정 (Role, Node ID, Callbacks)
 * @param get_tick Tick 함수 포인터
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_InitSecondary(AGR_PnP_Inst_t* inst, const AGR_PnP_Config_t* config, uint32_t (*get_tick)(void));

/* ===== Master 전용: Device 관리 ===== */

/**
 * @brief Device 등록 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param device Device 정보 (Node ID, Mode, Timeout, Callbacks)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_RegisterDevice(AGR_PnP_Inst_t* inst, const AGR_PnP_Device_t* device);

/**
 * @brief NMT 명령 전송 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param node_id 대상 Node ID (0 = Broadcast)
 * @param nmt_cmd NMT 명령 (START, STOP, PRE_OP, RESET_NODE, RESET_COMM)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_SendNmtCommand(AGR_PnP_Inst_t* inst, uint8_t node_id, AGR_NMT_Cmd_t nmt_cmd);

/**
 * @brief SDO Write 요청 전송 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param node_id 대상 Slave Node ID
 * @param index OD Index
 * @param subindex OD Sub-Index
 * @param data 쓸 데이터
 * @param data_len 데이터 길이
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_SendSDOWrite(AGR_PnP_Inst_t* inst, 
                         uint8_t node_id, 
                         uint16_t index, 
                         uint8_t subindex, 
                         const uint8_t* data, 
                         uint8_t data_len);

/**
 * @brief SDO Read 요청 전송 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param node_id 대상 Slave Node ID
 * @param index OD Index
 * @param subindex OD Sub-Index
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_SendSDORead(AGR_PnP_Inst_t* inst, 
                        uint8_t node_id, 
                        uint16_t index, 
                        uint8_t subindex);

/**
 * @brief Device 연결 상태 확인 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param node_id Device Node ID
 * @return true: 연결됨 (OPERATIONAL + Heartbeat OK), false: 끊김
 */
bool AGR_PnP_IsDeviceConnected(const AGR_PnP_Inst_t* inst, uint8_t node_id);

/**
 * @brief Device NMT 상태 조회 (Master only)
 * @param inst PnP 인스턴스 (Master)
 * @param node_id Device Node ID
 * @return NMT 상태
 */
AGR_NMT_State_t AGR_PnP_GetDeviceState(const AGR_PnP_Inst_t* inst, uint8_t node_id);

/* ===== Slave 전용: Master 연결 ===== */

/**
 * @brief Master 등록 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @param master Master 정보 (Node ID, Timeout)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_RegisterMaster(AGR_PnP_Inst_t* inst, const AGR_PnP_Master_t* master);
/**
 * @brief Slave 자신의 NMT 등록 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @param my_nmt Slave 자신의 NMT 인스턴스 포인터
 * @details Heartbeat 전송 시 Slave 자신의 NMT 상태를 전송하기 위함
 */
void AGR_PnP_SetMyNmt(AGR_PnP_Inst_t* inst, AGR_NMT_Inst_t* my_nmt);

/**
 * @brief Master 연결 상태 확인 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @return true: Master 연결됨 (OPERATIONAL + Heartbeat OK), false: 끊김
 */
bool AGR_PnP_IsMasterConnected(const AGR_PnP_Inst_t* inst);

/**
 * @brief 자신의 NMT 상태 조회 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @return 자신의 NMT 상태
 */
AGR_NMT_State_t AGR_PnP_GetMyState(const AGR_PnP_Inst_t* inst);

/**
 * @brief Boot-up 전송 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_SendBootup(AGR_PnP_Inst_t* inst);

/**
 * @brief Heartbeat 전송 (Slave only)
 * @param inst PnP 인스턴스 (Slave)
 * @return 0: 성공, <0: 실패
 */
int AGR_PnP_SendHeartbeat(AGR_PnP_Inst_t* inst);

/* ===== 공통: 메시지 처리 및 주기 실행 ===== */

/**
 * @brief CAN 메시지 처리 (Heartbeat, NMT, Boot-up)
 * @param inst PnP 인스턴스
 * @param can_id CAN ID
 * @param data 데이터 (최소 1 byte)
 * @param len 데이터 길이
 * @return 0: 처리됨, <0: 처리 안됨
 */
int AGR_PnP_ProcessMessage(AGR_PnP_Inst_t* inst, uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 * @brief 주기 실행 (Heartbeat Timeout 체크, Heartbeat 전송)
 * @param inst PnP 인스턴스
 * @details Master: Device Heartbeat Timeout 체크
 *          Slave: Master Heartbeat Timeout 체크, 자신의 Heartbeat 전송
 */
void AGR_PnP_RunPeriodic(AGR_PnP_Inst_t* inst);

#ifdef __cplusplus
}
#endif

#endif /* AGR_PNP_H */

