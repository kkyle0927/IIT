/**
 *-----------------------------------------------------------
 *                 FDCAN Communication Driver
 *-----------------------------------------------------------
 * @file ioif_agrb_fdcan.c
 * @version Common Library
 * @date Created on: Aug 23, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the FDCAN communication.
 *
 * This source file provides functionality to interface
 * with FDCAN hardware. Common Library version - shared
 * across XM, IMU Hub, and other Angel Robotics projects.
 *
 * @ref FDCAN reference
 */

#include "ioif_agrb_fdcan.h"
#if defined(AGRB_IOIF_FDCAN_ENABLE)

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#if defined(USE_FREERTOS)
#include "cmsis_os2.h" // For xSemaphore...
#endif

/**
 *-----------------------------------------------------------
 *              PRIVATE DEFINITIONS AND TYPES
 *-----------------------------------------------------------
 */

/* Software Tx Queue 크기 (HW FIFO 3개 보완) */
/**
 * @brief Software Tx Queue depth — HW Tx FIFO 보완용
 * @note  Project-level override: ioif_conf.h 에서 `#define IOIF_FDCAN_SW_TX_QUEUE_SIZE N` 으로 재정의 가능
 */
#ifndef IOIF_FDCAN_SW_TX_QUEUE_SIZE
#define IOIF_FDCAN_SW_TX_QUEUE_SIZE  (16)
#endif

#if defined(USE_FREERTOS)
/** @brief FDCAN RxTask 설정 (세마포어 Give/Take 동일 소스 원칙)
 *  @note ioif_conf.h에서 #ifndef 오버라이드 가능
 *  @note FDCAN Rx > SDO Processor(osPriorityRealtime3): FDCAN이 SDO에 데이터를 공급
 */
#ifndef IOIF_FDCAN_RX_TASK_STACK_SIZE
    #define IOIF_FDCAN_RX_TASK_STACK_SIZE    (2048U)              /**< RxTask 스택 크기 (bytes) */
#endif
#ifndef IOIF_FDCAN_RX_TASK_PRIORITY
    #define IOIF_FDCAN_RX_TASK_PRIORITY      (osPriorityRealtime4) /**< RxTask 우선순위 */
#endif
#define FDCAN_RX_TASK_STACK_SIZE    IOIF_FDCAN_RX_TASK_STACK_SIZE
#define FDCAN_RX_TASK_PRIORITY      IOIF_FDCAN_RX_TASK_PRIORITY
#endif

/**
 * @brief Tx Mutex Timeout (ms)
 * @details 
 * HAL_FDCAN_AddMessageToTxFifoQ 소요 시간: ~2µs
 * SW Queue 조작 소요 시간: ~5µs
 * 5ms Timeout은 매우 보수적 (실제 Hold Time 대비 1000배 마진)
 */
#define TX_MUTEX_TIMEOUT_MS     5

/* ===== Tx Mutex 매크로 (.cursorrules: RTOS/BareMetal 양립) ===== */
#if defined(USE_FREERTOS)
    #define TX_LOCK(inst)   ((inst)->tx_mutex != NULL && \
                             xSemaphoreTake((inst)->tx_mutex, pdMS_TO_TICKS(TX_MUTEX_TIMEOUT_MS)) == pdTRUE)
    #define TX_UNLOCK(inst) xSemaphoreGive((inst)->tx_mutex)
#else
    /* BareMetal: ISR 우선순위 차별화로 보호 (Critical Section 금지) */
    #define TX_LOCK(inst)   (true)
    #define TX_UNLOCK(inst) ((void)0)
#endif

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
    
#if defined(USE_FREERTOS)
    /* ===== RTOS Mode ===== */
    SemaphoreHandle_t       rx_semaphore;   /**< ISR → RxTask 신호용 */
    TaskHandle_t            rx_task_handle; /**< IOIF 내부 RxTask */
    SemaphoreHandle_t       tx_mutex;       /**< Tx 경로 Thread-Safety 보호 */
#endif
    
    /* Software Tx Queue (Bus Off 방지) */
    CanTxQueueItem_t        tx_queue[IOIF_FDCAN_SW_TX_QUEUE_SIZE];
    uint8_t                 tx_queue_head;
    uint8_t                 tx_queue_tail;
    uint8_t                 tx_queue_count;

    /* SW Tx Queue 사용량 통계 (QueueMessage enqueue 경로에서 갱신) */
    volatile uint8_t        tx_queue_peak;         /**< 관측된 최대 count (high-water mark) */
    volatile uint32_t       tx_queue_drop_count;   /**< Queue Full 로 인한 drop 누적 횟수 */

    /* Error event cumulative counters (HAL_FDCAN_ErrorStatusCallback 에서 갱신)
     * - 실시간 HW TEC/REC 은 IOIF_FDCAN_GetErrorCounters() 로 조회
     * - 본 필드들은 IOIF 가 관측한 이벤트 누적 통계 */
    volatile uint32_t       bus_off_count;
    volatile uint32_t       error_passive_count;
    volatile uint8_t        last_tec_at_error_passive;

    /* Bus-Off 복구 시 stale Tx 버스트 방지 (ISR set → main loop service)
     * ISR(ErrorStatusCallback) 은 플래그만 세트, 실제 Tx abort/flush 는
     * ProcessQueue 에서 수행 — AbortAllTx 가 TX_LOCK mutex 를 잡아 ISR 직접 호출 불가. */
    volatile bool           busoff_flush_pending;
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
static void _FlushAllTxLocked(IOIF_FDCAN_Inst_t* inst);
static uint32_t _Len2DLC(uint8_t len);
static uint8_t _DLC2Len(uint8_t dlc);
static uint8_t _LenRoundUpToDlcBytes(uint8_t len);
static HAL_StatusTypeDef _FdcanTxWithZeroPad(FDCAN_HandleTypeDef* hfdcan,
                                             FDCAN_TxHeaderTypeDef* header,
                                             const uint8_t* src,
                                             uint8_t src_len);

#if defined(USE_FREERTOS)
/**
 * @brief [IOIF 내부] FDCAN RxTask (세마포어 Give/Take 동일 소스 원칙)
 */
static void _IOIF_FDCAN_RxTask(void* argument);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* [REMOVED] IOIF_FDCAN_GetRxSemaphore 
 * 세마포어 Give/Take 동일 소스 원칙에 따라 제거됨.
 * RxTask가 IOIF 내부로 이동되어 semaphore 외부 노출 불필요.
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
    
#if defined(USE_FREERTOS)
    /* ===== Rx Semaphore 생성 (ISR → RxTask 신호) ===== */
    inst->rx_semaphore = xSemaphoreCreateBinary();
    if (inst->rx_semaphore == NULL) {
        return AGRBStatus_SEMAPHORE_ERROR;
    }
    /* ⚠️ 초기 상태: 대기 (Give 안함, ISR에서 첫 신호) */

    /* ===== Tx Mutex 생성 (Priority Inheritance 지원) =====
     * - 문제: UserTask(54), PnP(25), SDO Handler(51)가 동시에 Transmit 호출 가능
     * - 해결: Mutex로 Tx 경로 전체 보호 (HW Tx FIFO + SW Tx Queue)
     */
    inst->tx_mutex = xSemaphoreCreateMutex();
    if (inst->tx_mutex == NULL) {
        return AGRBStatus_SEMAPHORE_ERROR;
    }
#endif
    
    *id = s_instance_count; // 사용자에게 핸들(ID) 반환
    s_instance_count++;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_FDCAN_Start(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;

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

    // 4-1. RxFIFO0 Message Lost 모니터링 (overflow 검출)
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK) return AGRBStatus_ERROR;

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

#if defined(USE_FREERTOS)
    /**
     * 9. [RTOS] IOIF 내부 RxTask 생성
     * - 세마포어 Give/Take 동일 소스 원칙 적용
     * - ISR(Give) → RxTask(Take) → Batch Read → Callback
     */
    if (inst->rx_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(
            _IOIF_FDCAN_RxTask,
            (id == 0) ? "IOIF_FDCRx0" : "IOIF_FDCRx1",
            FDCAN_RX_TASK_STACK_SIZE / sizeof(StackType_t),
            (void*)(uintptr_t)id,  /* 인스턴스 ID 전달 */
            FDCAN_RX_TASK_PRIORITY,
            &inst->rx_task_handle
        );
        if (ret != pdPASS) {
            HAL_FDCAN_Stop(hfdcan);  /* HW 상태 복구 — 재시도 가능하도록 */
            return AGRBStatus_INITIAL_FAILED;
        }
    }
#endif

    return AGRBStatus_OK;
}

/**
 * @brief FDCAN Restricted Operation Mode 전환 (CCCR.ASM 비트만 토글)
 * @param id         IOIF_FDCANx_t 핸들
 * @param restricted true=Restricted Operation Mode 진입, false=Normal Mode 복원
 * @return AGRBStatus_OK 성공 / AGRBStatus_ERROR invalid id / AGRBStatus_FAILED Stop·Start 실패
 *
 * @details Restricted Operation Mode (CCCR.ASM=1): 외부 CAN_TX/RX 라인은 정상 신호를
 *          유지하되 ACK 미수신·Tx 오류에도 TEC 누적 / Bus-Off 진입이 HW 차원에서 차단된다.
 *          peer 미연결 환경(오실로 SI 측정, 단독 보드)에서 무한 송신해도 안전.
 *          시퀀스: HAL_FDCAN_Stop(INIT=1) → CCCR.CCE=1 → ASM 설정 → CCCR.CCE=0 →
 *          HAL_FDCAN_Start. filter/notification config 는 보존(HAL_FDCAN_Init 재호출 없음),
 *          전환 ~수십µs.
 *
 * @note 진단/stimulus 전용. 평시 PDO 경로에서 호출 금지(retransmit 차단 → 통신 fragility).
 * @note 단일 컨텍스트 가정 — Stop/Start 윈도우 중 동일 인스턴스 Transmit 금지(짧은 idle 구간).
 */
AGRBStatusDef IOIF_FDCAN_SetRestrictedMode(IOIF_FDCANx_t id, bool restricted)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_ERROR;
    }
    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;

    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
        return AGRBStatus_FAILED;
    }

    /* CCE (Configure Change Enable): INIT 모드에서 CCCR 의 protected 비트(ASM 포함)
     * 변경을 허용. ASM 은 protected bit 이므로 CCE 없이는 write 무시됨. */
    SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);
    if (restricted) {
        SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_ASM);
    } else {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_ASM);
    }
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_CCE);

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return AGRBStatus_FAILED;
    }
    return AGRBStatus_OK;
}

/**
 * @brief 지정된 FDCAN 인스턴스를 통해 메시지를 전송합니다.
 *
 * [변경 이유] .cursorrules "통신 아키텍처 철학" 섹션 기반
 * - AS-IS: Mutex 없이 HAL_FDCAN_AddMessageToTxFifoQ 호출 (Race Condition)
 * - TO-BE: Tx Mutex로 보호하여 Thread-Safety 보장
 * 
 * [Race Condition 시나리오]
 * 1. UserTask(54): Tx FIFO Put Index=5 읽음
 * 2. PnP Task(25): 선점 → 같은 Put Index=5 읽음 → RAM[5]에 Heartbeat 쓰기
 * 3. UserTask: RAM[5]에 토크 PDO 덮어쓰기 → CAN 프레임 손상
 * 
 * @note Thread-Safe: 내부 Tx Mutex 보호
 */
AGRBStatusDef IOIF_FDCAN_Transmit(IOIF_FDCANx_t id, uint32_t can_id, const uint8_t* txData, uint8_t len)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    if (len > 64) {
        return AGRBStatus_PARAM_ERROR;
    }

    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    AGRBStatusDef result = AGRBStatus_ERROR;
    
    /* ===== Tx Mutex Lock ===== */
    if (!TX_LOCK(inst)) {
        return AGRBStatus_TIMEOUT;  /* Mutex 획득 실패 */
    }
    
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

    if (_FdcanTxWithZeroPad(inst->hfdcan, &TxHeader, txData, len) == HAL_OK) {
        result = AGRBStatus_OK;
    }
    
    /* ===== Tx Mutex Unlock ===== */
    TX_UNLOCK(inst);
    
    return result;
}

/**
 * @brief Tx FIFO 여유 공간 확인 (HW 레지스터 Read-Only)
 * @details CAN Bus Off 방지를 위해 전송 전 FIFO 상태를 확인합니다.
 * @return 여유 공간 (0~3, 0=Full, 3=Empty)
 * 
 * @note Mutex 불필요: HW 레지스터 Read-Only (Atomic, Side-effect 없음)
 */
uint32_t IOIF_FDCAN_GetTxFifoFreeLevel(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;  /* 인스턴스 없음 → Full로 간주 */
    }

    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;

    /* Queue 모드: TFFL은 항상 0이므로 TFQF (Full flag)로 판단 */
    if (hfdcan->Init.TxFifoQueueMode == FDCAN_TX_QUEUE_OPERATION) {
        return ((hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U) ? 0U : 1U;
    }
    /* FIFO 모드: TFFL 필드 사용 (기존 동작) */
    return HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);
}

/**
 * @brief Tx pending buffer 개수 (TXBRP popcount)
 * @details Queue 모드에서 GetTxFifoFreeLevel 이 0/1 만 노출하는 한계를 보완.
 *          TXBRP 의 각 비트 = 한 Tx buffer slot 의 pending 여부.
 */
uint32_t IOIF_FDCAN_GetTxInFlightCount(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;
    }
    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;
    return (uint32_t)__builtin_popcount(hfdcan->Instance->TXBRP);
}

/**
 * @brief Rx FIFO0 채움 수준 확인 (HW 레지스터 Read-Only)
 */
uint32_t IOIF_FDCAN_GetRxFifo0FillLevel(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;
    }
    return HAL_FDCAN_GetRxFifoFillLevel(s_fdcan_instances[id].hfdcan, FDCAN_RX_FIFO0);
}

/**
 * @brief FDCAN 에러 카운터 조회 (ECR 레지스터)
 */
AGRBStatusDef IOIF_FDCAN_GetErrorCounters(IOIF_FDCANx_t id, uint8_t* tec, uint8_t* rec)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_ERROR;
    }
    FDCAN_ErrorCountersTypeDef err_cnt;
    if (HAL_FDCAN_GetErrorCounters(s_fdcan_instances[id].hfdcan, &err_cnt) != HAL_OK) {
        return AGRBStatus_ERROR;
    }
    if (tec) *tec = (uint8_t)err_cnt.TxErrorCnt;
    if (rec) *rec = (uint8_t)err_cnt.RxErrorCnt;
    return AGRBStatus_OK;
}

/**
 * @brief IOIF 관측 FDCAN 이벤트 누적 통계 조회
 */
AGRBStatusDef IOIF_FDCAN_GetErrorStats(IOIF_FDCANx_t id, IOIF_FDCAN_ErrorStats_t* stats)
{
    if (stats == NULL) return AGRBStatus_PARAM_ERROR;
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_ERROR;
    }
    const IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    stats->bus_off_count             = inst->bus_off_count;
    stats->error_passive_count       = inst->error_passive_count;
    stats->last_tec_at_error_passive = inst->last_tec_at_error_passive;
    return AGRBStatus_OK;
}

/**
 * @brief SW Tx Queue 사용량 통계 조회
 * @details now/peak/drop 각 포인터는 NULL 허용 (필요한 항목만 수신 가능).
 */
AGRBStatusDef IOIF_FDCAN_GetQueueStats(IOIF_FDCANx_t id,
                                        uint8_t* now,
                                        uint8_t* peak,
                                        uint32_t* drop)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_ERROR;
    }
    const IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    if (now)  *now  = inst->tx_queue_count;
    if (peak) *peak = inst->tx_queue_peak;
    if (drop) *drop = inst->tx_queue_drop_count;
    return AGRBStatus_OK;
}

/**
 * @brief FDCAN 프로토콜 상태 조회 (PSR 레지스터)
 */
AGRBStatusDef IOIF_FDCAN_GetBusStatus(IOIF_FDCANx_t id, uint8_t* lec, uint8_t* bus_status)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_ERROR;
    }
    FDCAN_ProtocolStatusTypeDef proto_sts;
    if (HAL_FDCAN_GetProtocolStatus(s_fdcan_instances[id].hfdcan, &proto_sts) != HAL_OK) {
        return AGRBStatus_ERROR;
    }
    if (lec) *lec = (uint8_t)proto_sts.LastErrorCode;
    if (bus_status) {
        *bus_status =
            ((proto_sts.BusOff)       ? 0x01 : 0) |
            ((proto_sts.Warning)      ? 0x02 : 0) |
            ((proto_sts.ErrorPassive) ? 0x04 : 0);
    }
    return AGRBStatus_OK;
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
/**
 * @brief Software Tx Queue에 메시지 추가 (Thread-Safe)
 * 
 * @note Thread-Safe: 내부 Tx Mutex 보호 (head/tail/count 무결성 보장)
 */
AGRBStatusDef IOIF_FDCAN_QueueMessage(IOIF_FDCANx_t id, uint32_t can_id, const uint8_t* data, uint8_t len, uint8_t priority)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned || data == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }
    if (len > 64) return AGRBStatus_PARAM_ERROR;

    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    AGRBStatusDef result = AGRBStatus_ERROR;
    
    /* ===== Tx Mutex Lock ===== */
    if (!TX_LOCK(inst)) {
        return AGRBStatus_TIMEOUT;
    }
    
    /* Queue Full 체크 */
    if (inst->tx_queue_count >= IOIF_FDCAN_SW_TX_QUEUE_SIZE) {
        inst->tx_queue_drop_count++;
        TX_UNLOCK(inst);
        return AGRBStatus_ERROR;  /* Queue Full (Drop) */
    }

    /* ✅ 단순 FIFO: tail 위치에 삽입 (CM-XM 방식) */
    inst->tx_queue[inst->tx_queue_tail].can_id = can_id;
    memcpy(inst->tx_queue[inst->tx_queue_tail].data, data, len);
    inst->tx_queue[inst->tx_queue_tail].len = len;
    inst->tx_queue[inst->tx_queue_tail].priority = priority;  /* 저장만 하고 사용 안 함 */
    inst->tx_queue[inst->tx_queue_tail].valid = true;

    inst->tx_queue_tail = (inst->tx_queue_tail + 1) % IOIF_FDCAN_SW_TX_QUEUE_SIZE;
    inst->tx_queue_count++;
    if (inst->tx_queue_count > inst->tx_queue_peak) {
        inst->tx_queue_peak = inst->tx_queue_count;
    }
    result = AGRBStatus_OK;
    
    /* ===== Tx Mutex Unlock ===== */
    TX_UNLOCK(inst);
    
    return result;
}

/**
 * @brief Software Tx Queue 처리 (Main Loop에서 주기 호출, Thread-Safe)
 * @details Queue에서 Tx FIFO로 전송 (FIFO 순서)
 * @return 처리된 메시지 개수
 * 
 * @note Thread-Safe: 내부 Tx Mutex 보호 (Queue 조작 + HAL Tx 원자적 수행)
 */
uint32_t IOIF_FDCAN_ProcessQueue(IOIF_FDCANx_t id, uint8_t reserved_slots)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return 0;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    uint32_t sent_count = 0;
    
    /* ===== Tx Mutex Lock ===== */
    if (!TX_LOCK(inst)) {
        return 0;  /* Mutex 획득 실패 → 다음 주기에 재시도 */
    }

    /* Bus-Off 복구 직후: 결함 이전에 적재된 stale 프레임을 비워
     * 복구된 버스로 버스트 송신되는 것을 방지 (ISR 은 flag 만 세트).
     * 클리어를 먼저 수행 → flush 도중 ISR 재세트 시 다음 주기에 재처리. */
    if (inst->busoff_flush_pending) {
        inst->busoff_flush_pending = false;
        _FlushAllTxLocked(inst);
        TX_UNLOCK(inst);
        return 0;  /* 이번 주기는 flush 만 수행, 정상 Tx 는 다음 주기부터 */
    }

    /* Queue에서 Tx FIFO로 전송 시도 */
    while (inst->tx_queue_count > 0) {
        /* Tx FIFO 여유 확인 — Queue 모드에서 GetTxFifoFreeLevel 은 TFQF(0/1) 만 노출해
         * backpressure 판정 불가. TXBRP popcount(in_flight)로 실제 free slot 수 산출
         * (3 - in_flight). reserved_slots 만큼 TPDO direct 용으로 남김. ⚠️ 구버전
         * (binary free_level ≤ reserved=1)은 연결 시 free_level 이 항상 0/1 → 거의 항상
         * break → SW queue drain 영구 차단 (HB/SDO/EMCY stranded). */
        uint32_t in_flight = IOIF_FDCAN_GetTxInFlightCount(id);
        uint32_t hw_free   = (in_flight < 3U) ? (3U - in_flight) : 0U;
        if (hw_free <= reserved_slots) {
            break;  /* Tx FIFO Full 또는 reserved 보호 → 다음 주기에 재시도 */
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
        
        /* Tx FIFO에 추가 (len→DLC gap 자동 zero-fill) */
        if (_FdcanTxWithZeroPad(inst->hfdcan, &TxHeader, item->data, item->len) == HAL_OK) {
            /* ✅ 전송 성공 → Queue에서 제거 */
            item->valid = false;
            inst->tx_queue_head = (inst->tx_queue_head + 1) % IOIF_FDCAN_SW_TX_QUEUE_SIZE;
            inst->tx_queue_count--;
            sent_count++;
        } else {
            /* ❌ 전송 실패 (Tx FIFO Full?) → 다음 주기에 재시도 */
            break;
        }
    }
    
    /* ===== Tx Mutex Unlock ===== */
    TX_UNLOCK(inst);
    
    return sent_count;
}

/**
 * @brief 모든 Pending Tx 요청 취소 (Thread-Safe)
 * @details Master 온라인 감지 시 stuck TX 방지를 위해 호출
 * @param id IOIF_FDCANx_t 핸들
 */
/**
 * @brief HW Tx 버퍼 abort + SW queue flush (내부 공용)
 * @details 호출자가 TX_LOCK 을 보유한 상태에서 호출해야 함 (lock 미획득).
 *          IOIF_FDCAN_AbortAllTx(외부 API) 와 ProcessQueue(Bus-Off 복구) 가 공유.
 */
static void _FlushAllTxLocked(IOIF_FDCAN_Inst_t* inst)
{
    HAL_FDCAN_AbortTxRequest(inst->hfdcan,
                              FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

    /* SW queue flush — abort된 메시지가 ProcessQueue에서 재전송되는 것 방지 */
    for (uint8_t i = 0; i < IOIF_FDCAN_SW_TX_QUEUE_SIZE; i++) {
        inst->tx_queue[i].valid = false;
    }
    inst->tx_queue_head  = 0;
    inst->tx_queue_tail  = 0;
    inst->tx_queue_count = 0;
}

void IOIF_FDCAN_AbortAllTx(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return;
    }
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    if (!TX_LOCK(inst)) {
        return;
    }
    _FlushAllTxLocked(inst);
    TX_UNLOCK(inst);
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
    IOIF_FDCAN_Inst_t* inst = _FindInstanceByHandle(hfdcan);

    /* Bus Off 자동 복구 */
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
        /* Bus Off 복구: INIT 비트 클리어 (즉시 재시작).
         * HW 는 INIT 클리어 후 129x11 recessive bit 동안 복구 대기하므로,
         * 그 사이 main loop 가 아래 flush 를 수행할 시간 창이 확보됨. */
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);

        if (inst != NULL) {
            inst->bus_off_count++;
            /* pending Tx (HW 버퍼 + SW queue) flush 를 main loop 로 defer.
             * ISR 에서 직접 abort 불가 — AbortAllTx 가 TX_LOCK mutex 사용. */
            inst->busoff_flush_pending = true;
        }
    }

    /* Error Passive 경고 */
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != RESET) {
        /* TEC >= 128: Error Passive 상태 (경고) */
        uint8_t tec = (uint8_t)((hfdcan->Instance->ECR & FDCAN_ECR_TEC_Msk) >> FDCAN_ECR_TEC_Pos);

        if (inst != NULL) {
            inst->last_tec_at_error_passive = tec;
            inst->error_passive_count++;
        }
    }
}

/**
 * @brief FDCAN 수신 FIFO 0에 새 메시지가 도착했을 때 HAL 라이브러리에 의해 호출되는 콜백 함수.
 * @details
 * [RTOS Mode - Semaphore Give ONLY]
 * - xSemaphoreGiveFromISR()만 호출 (~0.5µs)
 * - HW FIFO 읽기는 RxTask에서 처리 (Batch Processing)
 * - ISR Latency 최소화
 *
 * [BareMetal Mode - Direct Callback]
 * - HW FIFO에서 1개만 읽고 Callback 호출
 * - HW가 자동으로 인터럽트 재발생
 *
 * [성능 (STM32H743, 480MHz, Cache ON)]
 * - RTOS: ~0.5µs (Semaphore Give)
 * - BareMetal: ~5µs (GetRxMessage + Callback)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) {
        return;
    }
    
    IOIF_FDCAN_Inst_t* instance = _FindInstanceByHandle(hfdcan);
    if (instance == NULL || !instance->is_assigned) {
        return;
    }

#if defined(USE_FREERTOS)
    /* ===== RTOS: Semaphore Give ONLY ===== */
    if (instance->rx_semaphore != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->rx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

#else
    /* ===== BareMetal Mode: Direct Callback ===== */
    /* [방어 로직] Callback NULL 체크 */
    if (instance->rx_callback == NULL) {
        // 주인 없는 데이터는 1개만 읽고 버림
        FDCAN_RxHeaderTypeDef dummyHeader;
        uint8_t dummyData[64];
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &dummyHeader, dummyData);
        return;
    }
    
    IOIF_FDCAN_Msg_t received_msg;
    FDCAN_RxHeaderTypeDef rxHeader;

    /* 1개만 읽고 즉시 ISR 종료 (HW가 자동 재발생) */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, received_msg.data) == HAL_OK)
    {
        received_msg.id = rxHeader.Identifier;
        received_msg.len = _DLC2Len(rxHeader.DataLength);
        
        /* Callback 호출 (Device Layer) */
        instance->rx_callback(&received_msg);
    }
#endif
}

/**
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

/**
 * @brief len 을 CAN-FD DLC 가 실제 전송할 byte 수로 올림
 * @details `_Len2DLC` 가 반환하는 HAL 매크로는 register bit pattern 이라 산술 불가.
 *          본 helper 는 동일 분기를 byte 단위로 표현하여 padding 계산에 사용한다.
 */
static uint8_t _LenRoundUpToDlcBytes(uint8_t len)
{
    if (len <= 8)  return len;
    if (len <= 12) return 12;
    if (len <= 16) return 16;
    if (len <= 20) return 20;
    if (len <= 24) return 24;
    if (len <= 32) return 32;
    if (len <= 48) return 48;
    return 64;
}

/**
 * @brief HAL tx + len→DLC gap 자동 zero-fill
 * @details CAN-FD DLC 올림으로 인해 caller 버퍼 경계 내 uninit 영역이 wire 에
 *          실리는 것을 차단. 호출자는 logical len 만 선언하고 padding 은 IOIF 책임.
 *          Stack-local 64B 버퍼 사용 (FDCAN Message RAM 접근은 DMA 아님).
 */
static HAL_StatusTypeDef _FdcanTxWithZeroPad(FDCAN_HandleTypeDef* hfdcan,
                                             FDCAN_TxHeaderTypeDef* header,
                                             const uint8_t* src,
                                             uint8_t src_len)
{
    uint8_t padded[64];
    uint8_t dlc_bytes = _LenRoundUpToDlcBytes(src_len);

    if (src_len > 0 && src != NULL) {
        memcpy(padded, src, src_len);
    }
    if (dlc_bytes > src_len) {
        memset(padded + src_len, 0, (size_t)(dlc_bytes - src_len));
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, header, padded);
}


#if defined(USE_FREERTOS)
/**
 * @brief [IOIF 내부] FDCAN RxTask (세마포어 Give/Take 동일 소스 원칙)
 */
static void _IOIF_FDCAN_RxTask(void* argument)
{
    IOIF_FDCANx_t id = (IOIF_FDCANx_t)(uintptr_t)argument;

    if (id >= IOIF_FDCAN_MAX_INSTANCES) {
        vTaskDelete(NULL);
        return;
    }

    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    IOIF_FDCAN_Msg_t msg;
    FDCAN_RxHeaderTypeDef rxHeader;

    for (;;) {
        if (xSemaphoreTake(inst->rx_semaphore, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        while (HAL_FDCAN_GetRxMessage(inst->hfdcan, FDCAN_RX_FIFO0,
                                       &rxHeader, msg.data) == HAL_OK)
        {
            msg.id  = rxHeader.Identifier;
            msg.len = _DLC2Len(rxHeader.DataLength);

            if (inst->rx_callback != NULL) {
                inst->rx_callback(&msg);
            }
        }
    }
}
#endif /* USE_FREERTOS */

#if defined(USE_FREERTOS)
/**
 * @brief [RTOS Only] FDCAN 메시지 수신 (Non-Blocking) - 공개 API
 * @param id FDCAN 인스턴스 ID
 * @param msg 수신 메시지 구조체 포인터
 * @return AGRBStatus_OK (메시지 수신), AGRBStatus_ERROR (FIFO 비어있음)
 * 
 * @note IOIF 내부 RxTask가 기본 처리. 필요시 외부에서도 호출 가능.
 */
AGRBStatusDef IOIF_FDCAN_Receive(IOIF_FDCANx_t id, IOIF_FDCAN_Msg_t* msg)
{
    if (id >= s_instance_count || msg == NULL) {
        return AGRBStatus_ERROR;
    }

    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    if (inst->hfdcan == NULL) {
        return AGRBStatus_ERROR;
    }

    /* HW FIFO에서 메시지 1개 읽기 (Non-Blocking) */
    FDCAN_RxHeaderTypeDef rxHeader;
    HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(
        inst->hfdcan, 
        FDCAN_RX_FIFO0, 
        &rxHeader, 
        msg->data
    );

    if (status != HAL_OK) {
        /* HW FIFO 비어있음 */
        return AGRBStatus_ERROR;
    }

    /* CAN-ID 및 길이 변환 */
    msg->id = rxHeader.Identifier;
    msg->len = _DLC2Len(rxHeader.DataLength);

    return AGRBStatus_OK;
}
#endif

#endif /* AGRB_IOIF_FDCAN_ENABLE */
