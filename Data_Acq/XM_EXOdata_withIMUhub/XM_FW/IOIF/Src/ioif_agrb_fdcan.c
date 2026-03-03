/**
 *-----------------------------------------------------------
 *                 FDCAN Communication Driver
 *-----------------------------------------------------------
 * @date Created on: Aug 23, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the FDCAN communication.
 *
 * This source file provides functionality to interface
 *
 * @ref FDCAN reference
 */

#include "ioif_agrb_fdcan.h"
#if defined(AGRB_IOIF_FDCAN_ENABLE)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if defined(USE_FREERTOS_DMA)
#include "cmsis_os2.h" // For xSemaphore...
#endif

// #include "can_bus_monitor.h"  /* CAN Bus 디버깅 - File not found, disabled */

/**
 *-----------------------------------------------------------
 *              PRIVATE DEFINITIONS AND TYPES
 *-----------------------------------------------------------
 */

/* Software Tx Queue 크기 (HW FIFO 3개 보완) */
#define SW_TX_QUEUE_SIZE  16  /**< 16 slots (3개 HW FIFO 보완) */

/* ✅ CAN 메시지 우선순위는 IOIF에서 정의하지 않음 (CANopen 프로토콜 독립성) */
/* ⚠️ Device Layer (xm_drv.c)에서 정의해야 함 */

/**
 * @brief Software Tx Queue 아이템
 */
typedef struct {
    uint32_t can_id;
    uint8_t  data[64];
    uint8_t  len;
    uint8_t  priority;  /**< 0=최고, 255=최저 */
    bool     valid;     /**< true: 유효한 메시지 */
} CanTxQueueItem_t;

// FDCAN 인스턴스의 내부 상태를 관리하는 구조체 (Private)
typedef struct {
    bool                    is_assigned;
    FDCAN_HandleTypeDef*    hfdcan;
    IOIF_FDCAN_RxCallback_t rx_callback;
    
    /* ✅ RTOS/Bare-metal 공통: ISR에서 직접 콜백 호출 (Task 불필요) */
    
    /* Software Tx Queue (Bus Off 방지) */
    CanTxQueueItem_t        tx_queue[SW_TX_QUEUE_SIZE];
    uint8_t                 tx_queue_head;
    uint8_t                 tx_queue_tail;
    uint8_t                 tx_queue_count;
} IOIF_FDCAN_Inst_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 */
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 */

// 인스턴스들을 관리할 정적 배열
static IOIF_FDCAN_Inst_t s_fdcan_instances[IOIF_FDCAN_MAX_INSTANCES] = {0};
static uint32_t s_instance_count = 0;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static IOIF_FDCAN_Inst_t* _FindInstanceByHandle(FDCAN_HandleTypeDef* hfdcan);
static uint32_t _Len2DLC(uint8_t len);
static uint8_t _DLC2Len(uint8_t dlc);
#if defined(USE_FREERTOS_DMA)
static void _IOIF_FDCAN_RxTask(void* argument);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

 // Init 함수에서 rx_queue 인자 제거
AGRBStatusDef IOIF_FDCAN_AssignInstance(IOIF_FDCANx_t* id, FDCAN_HandleTypeDef* hfdcan)
{
    if (s_instance_count >= IOIF_FDCAN_MAX_INSTANCES || hfdcan == NULL || id == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }
    // 이미 등록된 핸들인지 확인
    if (_FindInstanceByHandle(hfdcan) != NULL) {
        return AGRBStatus_BUSY;
    }

    // 새 인스턴스에 정보 할당
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[s_instance_count];
    memset(inst, 0, sizeof(IOIF_FDCAN_Inst_t));
    inst->is_assigned = true;
    inst->hfdcan = hfdcan;
    inst->rx_callback = NULL; // 콜백을 NULL로 초기화
    /* ✅ RTOS/Bare-metal 공통: ISR에서 직접 콜백 호출 (Task 불필요) */
    
    *id = s_instance_count; // 사용자에게 핸들(ID) 반환
    s_instance_count++;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_FDCAN_Start(IOIF_FDCANx_t id)
{
    /* ✅ Task 불필요 (ISR에서 직접 콜백 호출) */
    
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;

    /* ✅ RTOS/Bare-metal 공통: Task 불필요, ISR에서 직접 콜백 호출 */

    /* 
     * [FDCAN Hardware Filter 설정 제거]
     * 
     * [설계 변경]
     * - IOIF는 범용 모듈이므로 기본 필터를 설정하지 않습니다.
     * - System Layer에서 IOIF_FDCAN_ConfigFilter()를 호출하여 모듈에 맞는 필터를 설정해야 합니다.
     * - 이유: System Layer의 필터 설정이 덮어써지는 문제 방지
     * 
     * [초기화 순서]
     * 1. System Layer: IOIF_FDCAN_ConfigFilter() (필터 설정)
     * 2. System Layer: IOIF_FDCAN_START() (FDCAN 시작)
     * 3. IOIF Layer: Rx FIFO Overwrite + Global Filter + TDC + Start (하드웨어 활성화)
     */
    
    // 2. Rx FIFO Overwrite Mode 활성화 (최신 메시지 우선)
    if (HAL_FDCAN_ConfigRxFifoOverwrite(hfdcan, FDCAN_RX_FIFO0, 
                                        FDCAN_RX_FIFO_OVERWRITE) != HAL_OK) {
        return AGRBStatus_ERROR;
    }
    
    // 3. 글로벌 필터 설정
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return AGRBStatus_ERROR;
    
    // 4. 수신 인터럽트 활성화
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return AGRBStatus_ERROR;

    // Bus Off Auto Recovery 활성화 (STM32G4/H7 공통)
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) return AGRBStatus_ERROR;
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_ERROR_PASSIVE, 0) != HAL_OK) return AGRBStatus_ERROR;

    // 5. Tx Complete Interrupt 활성화 (CAN Bus Off 방지)
#if defined(STM32H743xx)
    // 모든 Tx 버퍼 (STM32H743: 32개)에 ISR 활성화
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0xFFFFFFFF) != HAL_OK) return AGRBStatus_ERROR;
#elif defined(STM32G474xx)
    // 모든 Tx 버퍼 (STM32G474: 3개)에 ISR 활성화
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0x00000007) != HAL_OK) return AGRBStatus_ERROR;
#endif
    
    // 6. 송신 지연 보상(TDC) 설정
    if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1, IOIF_TDC_FILTER) != HAL_OK) return AGRBStatus_ERROR;
    
    // 7. 송신 지연 보상(TDC) 활성화
    if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    // 8. FDCAN 시작
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_FDCAN_Transmit(IOIF_FDCANx_t id, uint32_t can_id, const uint8_t* txData, uint8_t len)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }

    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;
    
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = can_id,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = _Len2DLC(len),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_ON,
        .FDFormat = FDCAN_FD_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, txData) != HAL_OK) {
        // CanBusMon_RecordTx(can_id, len, false);  /* ❌ Tx 실패 기록 - Disabled */
        return AGRBStatus_ERROR;
    }

    // CanBusMon_RecordTx(can_id, len, true);  /* ✅ Tx 성공 기록 - Disabled */
    return AGRBStatus_OK;
}

/**
 * @brief Tx FIFO 여유 공간 확인
 * @details CAN Bus Off 방지를 위해 전송 전 FIFO 상태를 확인합니다.
 * @return 여유 공간 (0~3, 0=Full, 3=Empty)
 */
uint32_t IOIF_FDCAN_GetTxFifoFreeLevel(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;  /* 인스턴스 없음 → Full로 간주 */
    }

    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;
    
    /* HAL_FDCAN_GetTxFifoFreeLevel 사용 */
    return HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
}

/**
 * @brief Software Tx Queue에 메시지 추가 (FIFO)
 * @details Tx FIFO Full 시 Software Queue에 저장 (Bus Off 방지)
 * 
 * [변경 이유]
 * - 우선순위 기반 중간 삽입 로직이 Circular Queue 상태를 망가뜨림
 * - Boot-up 시 Queue Full 오류 발생
 * - CM-XM처럼 단순 FIFO로 변경 (검증된 방식)
 * 
 * @note priority 파라미터는 호환성을 위해 유지하지만 사용하지 않음
 */
AGRBStatusDef IOIF_FDCAN_QueueMessage(IOIF_FDCANx_t id, uint32_t can_id, const uint8_t* data, uint8_t len, uint8_t priority)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned || data == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    
    /* Queue Full 체크 */
    if (inst->tx_queue_count >= SW_TX_QUEUE_SIZE) {
        return AGRBStatus_ERROR;  /* Queue Full (Drop) */
    }
    
    /* ✅ 단순 FIFO: tail 위치에 삽입 (CM-XM 방식) */
    inst->tx_queue[inst->tx_queue_tail].can_id = can_id;
    memcpy(inst->tx_queue[inst->tx_queue_tail].data, data, len);
    inst->tx_queue[inst->tx_queue_tail].len = len;
    inst->tx_queue[inst->tx_queue_tail].priority = priority;  /* 저장만 하고 사용 안 함 */
    inst->tx_queue[inst->tx_queue_tail].valid = true;
    
    inst->tx_queue_tail = (inst->tx_queue_tail + 1) % SW_TX_QUEUE_SIZE;
    inst->tx_queue_count++;
    
    return AGRBStatus_OK;
}

/**
 * @brief Software Tx Queue 처리 (Main Loop에서 주기 호출)
 * @details Queue에서 Tx FIFO로 전송 (FIFO 순서)
 * @return 처리된 메시지 개수
 */
uint32_t IOIF_FDCAN_ProcessQueue(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;
    uint32_t sent_count = 0;
    
    /* Queue에서 Tx FIFO로 전송 시도 */
    while (inst->tx_queue_count > 0) {
        /* Tx FIFO 여유 확인 */
        uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
        if (free_level == 0) {
            break;  /* Tx FIFO Full → 다음 주기에 재시도 */
        }
        
        /* Queue에서 최우선 메시지 가져오기 (head) */
        CanTxQueueItem_t* item = &inst->tx_queue[inst->tx_queue_head];
        if (!item->valid) {
            break;  /* Invalid item */
        }
        
        /* Tx Header 설정 */
        FDCAN_TxHeaderTypeDef TxHeader = {
            .Identifier = item->can_id,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = _Len2DLC(item->len),
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_ON,
            .FDFormat = FDCAN_FD_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0
        };
        
        /* Tx FIFO에 추가 */
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, item->data) == HAL_OK) {
            /* ✅ 전송 성공 */
            CanBusMon_RecordTx(item->can_id, item->len, true);
            
            /* Queue에서 제거 */
            item->valid = false;
            inst->tx_queue_head = (inst->tx_queue_head + 1) % SW_TX_QUEUE_SIZE;
            inst->tx_queue_count--;
            sent_count++;
        } else {
            /* ❌ 전송 실패 (Tx FIFO Full?) → 다음 주기에 재시도 */
            CanBusMon_RecordTx(item->can_id, item->len, false);
            break;
        }
    }
    
    return sent_count;
}

AGRBStatusDef IOIF_FDCAN_ConfigFilter(IOIF_FDCANx_t id, FDCAN_FilterTypeDef* filter_config)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    
    if (filter_config == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }
    
    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;
    
    if (HAL_FDCAN_ConfigFilter(hfdcan, filter_config) != HAL_OK) {
        return AGRBStatus_ERROR;
    }
    
    return AGRBStatus_OK;
}

// 수신 콜백 등록 함수 구현
void IOIF_FDCAN_RegisterRxCallback(IOIF_FDCANx_t id, IOIF_FDCAN_RxCallback_t callback)
{
    if (id < s_instance_count && s_fdcan_instances[id].is_assigned) {
        s_fdcan_instances[id].rx_callback = callback;
    }
}

/**
 *------------------------------------------------------------
 *                        HAL CALLBACKS
 *------------------------------------------------------------
 */

/**
 * @brief FDCAN Error Status Callback (Bus Off Auto Recovery)
 * @details HAL에서 Bus Off 또는 Error Passive 발생 시 호출
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    /* Bus Off 자동 복구 */
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
        /* Bus Off 복구: INIT 비트 클리어 (즉시 재시작) */
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        
        /* TODO: Bus Off 로그 기록 (선택 사항) */
        // CanBusMon_RecordBusOff();
    }
    
    /* Error Passive 경고 */
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != RESET) {
        /* TEC >= 128: Error Passive 상태 (경고) */
        uint32_t tec = (hfdcan->Instance->ECR & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos;
        
        /* TODO: Error Passive 로그 (선택 사항) */
        // CanBusMon_RecordErrorPassive(tec);
        (void)tec;
    }
}

/**
 * @brief FDCAN 수신 FIFO 0에 새 메시지가 도착했을 때 HAL 라이브러리에 의해 호출되는 콜백 함수.
 * @details 이 함수의 유일한 역할은 하드웨어로부터 메시지를 꺼내 파싱한 후,
 * 상위 계층에서 등록한 콜백 함수로 그대로 전달하는 것입니다.
 * PDO/SDO 구분, 필터링, 큐 전송 등 어떠한 정책 결정도 하지 않습니다.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs; // Unused parameter warning 방지
    
    // 이 인터럽트를 발생시킨 인스턴스를 찾음
    IOIF_FDCAN_Inst_t* instance = _FindInstanceByHandle(hfdcan);

    /* ✅ RTOS/Bare-metal 공통: ISR에서 즉시 콜백 호출 (정책은 System Layer에서 결정) */
    /* [방어 로직] 콜백이 NULL이면 FIFO를 읽어서 버림 */
    // 1. 인스턴스 검증
    if (instance == NULL || instance->rx_callback == NULL) {
        // [Fast Drain] 주인 없는 데이터는 빠르게 비워서 버림
        FDCAN_RxHeaderTypeDef dummyHeader;
        uint8_t dummyData[64];
        // FIFO가 빌 때까지 반복해서 읽고 버림
        while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &dummyHeader, dummyData) == HAL_OK);
        return;
    }
    
    /* HW FIFO가 빌 때까지 반복 처리 (Batch Processing) */
    // 2. [Batch Processing] 유효 데이터 처리
    IOIF_FDCAN_Msg_t received_msg;
    FDCAN_RxHeaderTypeDef rxHeader;

    /* * [최적화] GetRxFifoFillLevel 호출 제거
     * GetRxMessage가 HAL_OK를 리턴하는 동안 계속 돔 (데이터가 있으면 OK, 없으면 Error 리턴)
     */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, received_msg.data) == HAL_OK)
    {
        // 메시지 파싱
        received_msg.id = rxHeader.Identifier;
        received_msg.len = _DLC2Len(rxHeader.DataLength);
        
        // 기록
        // CanBusMon_RecordRx(received_msg.id, received_msg.len);  // Disabled - can_bus_monitor.h not found
        
        // 상위 계층 전달 (PDO:즉시처리, SDO, NMT, Hearbeat, ...:큐잉)
        instance->rx_callback(&received_msg);
    }
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static IOIF_FDCAN_Inst_t* _FindInstanceByHandle(FDCAN_HandleTypeDef* hfdcan)
{
    for (uint32_t i = 0; i < s_instance_count; ++i) {
        if (s_fdcan_instances[i].hfdcan == hfdcan) {
            return &s_fdcan_instances[i];
        }
    }
    return NULL;
}

static uint32_t _Len2DLC(uint8_t len)
{
    // CAN-FD Length to DLC conversion table
    static const uint32_t len_to_dlc[] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5, FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    if (len <= 8) return len_to_dlc[len];
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

static uint8_t _DLC2Len(uint8_t dlc)
{
    static const uint8_t dlc_to_len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    if (dlc > 15) return 64;
    return dlc_to_len[dlc];
}

// // RTOS 사용 시에만 Rx 태스크 함수 구현
// #if defined(USE_FREERTOS_DMA)
// /**
//  * @brief FDCAN Rx Task (Batch Processing 패턴)
//  * @details 
//  * [성능 최적화 전략]
//  * 1. Task Notification 사용 (Semaphore보다 10배 빠름)
//  * 2. 한 번 깨어나면 FIFO가 빌 때까지 반복 처리 (Context Switch 최소화)
//  * 3. Critical Section 불필요 (Queue/Mutex 대신 빠른 메모리 접근)
//  */
// static void _IOIF_FDCAN_RxTask(void* argument)
// {
//     IOIF_FDCAN_Inst_t* inst = (IOIF_FDCAN_Inst_t*)argument;
//     FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;

//     IOIF_FDCAN_Msg_t received_msg;
//     FDCAN_RxHeaderTypeDef rxHeader;

//     for (;;) {
//         /* ✅ Step 1: Task Notification 대기 (ISR이 알림을 보낼 때까지 블로킹) */
//         /* pdTRUE: 알림 값을 0으로 초기화 (Binary Semaphore처럼 동작) */
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
//         /* ✅ Step 2: Batch Processing (FIFO가 빌 때까지 반복 처리) */
//         /* 
//          * [중요] 한 번 깨어났을 때 쌓인 메시지를 모두 처리해야 합니다!
//          * - Context Switch 비용 절감 (알림 1번 → 메시지 N개 처리)
//          * - 처리 속도가 수신 속도를 따라잡을 수 있음
//          */
//         while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
//             /* FIFO에서 메시지 읽기 */
//             if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, received_msg.data) != HAL_OK) {
//                 break;  /* 에러 발생 시 중단 */
//             }

//             /* 메시지 파싱 */
//             received_msg.id = rxHeader.Identifier;
//             received_msg.len = _DLC2Len(rxHeader.DataLength);

//             /* CAN Bus Monitor 기록 */
//             CanBusMon_RecordRx(received_msg.id, received_msg.len);

//             /* Rx 콜백 호출 (상위 계층으로 전달) */
//             if (inst->rx_callback != NULL) {
//                 inst->rx_callback(&received_msg);
//             }
//         }
//         /* Loop 종료: FIFO가 비었음 → 다시 대기 상태로 */
//     }
// }
// #endif // USE_FREERTOS_DMA

#endif /* AGRB_IOIF_FDCAN_ENABLE */
