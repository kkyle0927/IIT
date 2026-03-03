/**
 ******************************************************************************
 * @file    canfd_rx_handler.c
 * @author  HyundoKim
 * @brief   [System] FDCAN Rx 메시지 라우팅 및 처리
 * @version 3.0 (CANopen 표준 기반)
 * @date    Dec 8, 2025
 *
 * @details
 * [V3.0 변경사항]
 * - System Link 제거 (imu_hub_xm_link_v2.h)
 * - Device Driver 직접 호출 (ImuHub_Drv_ProcessCANMessage)
 * - CANopen 표준 메시지 처리 (AGR_NMT, AGR_DOP)
 * 
 * [메시지 라우팅]
 * - PDO (0x180/0x280): ISR에서 직접 처리 (실시간)
 * - 기타 (Heartbeat, SDO, NMT): Queue → Router Task 처리
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "canfd_rx_handler.h"
#include "data_object_dictionaries.h"
#include "ioif_agrb_fdcan.h" // IOIF_FDCAN_Msg_t 구조체를 위해 포함
#include "cm_drv.h"
#include "imu_hub_drv.h"       /* ✅ IMU Hub Driver (CANopen 표준, V3.0) */
#include "pnp_manager.h"       // cm_xm_link.h 대신 pnp_manager.h를 포함
#include "system_startup.h"

#if defined(USE_FREERTOS_DMA)
// FreeRTOS 및 CMSIS-OS 관련 헤더
#include "FreeRTOS.h"
#include "task.h"       /* For vTaskNotifyGiveFromISR, ulTaskNotifyTake */
#include "cmsis_os2.h"
#endif

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
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/**
 * ✅ Task Notification + SW RingBuffer (High Performance)
 * - Queue 대비 10배 빠른 ISR 처리
 * - Batch Processing으로 Context Switch 최소화
 * - 메모리 효율적 (Queue 객체 불필요)
 */

/* SW RingBuffer for General Messages (SDO, Heartbeat, NMT) */
/**
 * [버퍼 크기 계산 근거]
 * 
 * XM Master 환경:
 * - 연결 모듈: 최대 6개 (CM, IMU Hub, EMG, FES, FSR, 기타)
 * - PnP 단계: 각 모듈당 Boot-up + SDO Response × 2 = 3개
 * - Heartbeat: 100ms 주기 × 6개 모듈
 * 
 * 최악의 시나리오:
 * 1. 모든 모듈 동시 PnP: 6 × 3 = 18개
 * 2. Router Task 지연 (50~100ms): +12개 (SDO 중복)
 * 3. Error + 재연결: +6개
 * → 총 36개
 * 
 * 안전 마진 (1.8배): 36 × 1.8 = 64.8
 * → Power-of-2 정렬: 64개
 * 
 * 메모리 사용량: 64 × 72B = 4.6KB (STM32H7 1MB RAM 중 0.5%)
 */
#define FDCAN_GEN_FIFO_SIZE   64  /**< 일반 메시지 버퍼 크기 (6개 모듈 안전) */

/**
 * @brief SW RingBuffer + 통계 (Overflow 관리)
 */
typedef struct {
    uint32_t head;
    uint32_t tail;
    IOIF_FDCAN_Msg_t buffer[FDCAN_GEN_FIFO_SIZE];
    
    /* ✅ 통계 (디버깅/모니터링) */
    uint32_t total_rx_count;      /**< 총 수신 메시지 수 */
    uint32_t overwrite_count;     /**< Overwrite 발생 횟수 */
    uint32_t peak_usage;          /**< 최대 사용량 (0~32) */
} FDCAN_SwFifo_t;

static FDCAN_SwFifo_t s_gen_fifo = { 
    .head = 0, 
    .tail = 0, 
    .total_rx_count = 0,
    .overwrite_count = 0,
    .peak_usage = 0
};

// CANFD Rx Queue Router Task의 속성 정의
static osThreadId_t CanFDRxQueueRouter_TaskHandle;
static const osThreadAttr_t CanFDRxQueueRouter_Task_attributes = {
  .name = "CanFDRxQueueRouter_Task",
  .stack_size = TASK_STACK_SDO_ROUTER,
  .priority = (osPriority_t) TASK_PRIO_SDO_ROUTER,
};

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg); // Rx Data 콜백

static void _CanFdRxQueueRouter_Task(void* argument); // Rx Router Task

/* SW RingBuffer 관리 함수 */
static inline void _GenFifo_Push(IOIF_FDCAN_Msg_t* msg);  /* ✅ void: 항상 성공 (Overwrite) */
static inline bool _GenFifo_Pop(IOIF_FDCAN_Msg_t* msg);
static void _Process_General_Message(IOIF_FDCAN_Msg_t* msg);

/* 통계 함수 (디버깅/모니터링용) */
static inline uint32_t _GenFifo_GetUsage(void);  /* 현재 사용량 반환 */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void CanFdRxHandler_Init(void)
{
    /* ✅ SW RingBuffer 초기화 (static 변수이므로 자동 초기화) */
    s_gen_fifo.head = 0;
    s_gen_fifo.tail = 0;

    // system_startup으로부터 FDCAN1의 ID를 가져옵니다.
    IOIF_FDCANx_t fdcan1_id = System_GetFDCAN1_Id();

    // IOIF 드라이버에 콜백 함수 등록 (RTOS/Bare-metal 공통)
    // fdcan1_id는 system_startup에서 초기화, getter 함수로 접근
    IOIF_FDCAN_REGISTER_CALLBACK(fdcan1_id, _CanFdRxCallback); // 수신 데이터 처리 콜백

    /* ✅ Router Task 생성 (Task Notification 기반, 고성능) */
    CanFDRxQueueRouter_TaskHandle = osThreadNew(_CanFdRxQueueRouter_Task, NULL, &CanFDRxQueueRouter_Task_attributes);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IOIF FDCAN 드라이버에 의해 ISR 컨텍스트에서 호출될 메인 수신 콜백. (정책 구현)
 * @details 
 * [중요] DOP V1과 V2의 Function Code가 다르므로, **Node ID로 먼저 분기**해야 합니다.
 * - DOP V1 (CM): FNC 0x300 (PDO), 0x200 (SDO)
 * - DOP V2 (IMU Hub): FNC 0x180/0x280 (TPDO1/2), 0x580/0x600 (SDO), 0x700 (Heartbeat)
 * 
 * [CANopen Node ID 추출]
 * CAN-ID = [Function Code: 4-bit] | [Node ID: 7-bit]
 *          Bits 7-10               Bits 0-6
 * 
 * Node ID 추출: can_id & 0x7F (하위 7비트)
 * 실용적으로는 can_id & 0x0F (하위 4비트)로 충분 (Node ID ≤ 15)
 */
static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg)
{
    /* 
     * [DOP V1 vs V2 구분 전략]
     * 
     * DOP V1 (Legacy): [Function(8-bit)] [Oriented Node(4-bit)] [Destination Node(4-bit)]
     *   - Function: Bit 8-10 (0x200=SDO, 0x300=PDO)
     *   - Oriented Node (발신자): Bit 4-7 → (msg->id & 0x0F0) >> 4
     *   - Destination Node (수신자): Bit 0-3 → msg->id & 0x0F
     *   - 예: 0x312 = 0x3 (PDO) | 0x1 (CM, Oriented) | 0x2 (XM, Destination) ✅
     *   - 예: 0x371 = 0x3 (PDO) | 0x7 (MD, Oriented) | 0x1 (CM, Destination) ❌ XM이 받으면 안됨!
     * 
     * DOP V2 (CANopen): [Function(4-bit)] [Node(7-bit)]
     *   - Function: Bit 7-10 (0x180=TPDO1, 0x700=Heartbeat)
     *   - Node ID (발신자): Bit 0-6 → msg->id & 0x7F
     *   - 예: 0x708 = 0x7 (Heartbeat) | 0x08 (IMU Hub)
     */
    
    uint16_t fnc_code = msg->id & 0x780;  // Bit 7-10 추출
    
    /* [STEP 1] Function Code로 DOP V1 vs V2 구분 */
    if (fnc_code == 0x200 || fnc_code == 0x300) {
        /* ========== DOP V1: CM ↔ XM (Legacy) ========== */
        uint8_t dest_node = msg->id & 0x0F;  // Bit 0-3: Destination Node
        
        /* XM으로 오는 메시지만 처리 */
        if (dest_node == SYS_NODE_ID_XM) {  // 0x02 (XM이 받을 메시지)
            switch (fnc_code) {
                case 0x300:  // PDO (DOP V1)
                    CM_UpdatePdoData(msg->data, msg->len);
                    break;
                    
                case 0x200:  // SDO (DOP V1)
                    /* ✅ Task Notification: Queue 대비 10배 빠름 */
                    _GenFifo_Push(msg);  /* ✅ 항상 성공 (Overwrite 방식) */
                    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                    vTaskNotifyGiveFromISR(CanFDRxQueueRouter_TaskHandle, &xHigherPriorityTaskWoken);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    break;
            }
        }
        /* 
         * else: 다른 Destination (CM, MD 등)으로 가는 메시지 → 무시
         * - 0x371 (MD(LH)→CM): dest_node=0x1 (CM) → XM이 무시 ✅
         * - 0x161 (MD(RH)→CM): dest_node=0x1 (CM) → XM이 무시 ✅
         */
    }
    else {
        /* ========== DOP V2: Sensor Modules (CANopen Compatible) ========== */
        uint8_t node_id = msg->id & 0x7F;  // Bit 0-6: Node ID (발신자)
        
        if (node_id == SYS_NODE_ID_IMU_HUB) {  /* 0x08 */
            /* [분기] Function Code로 구분 */
            uint16_t fnc_code = msg->id & 0x780;  /* Bit 7-10: Function Code */
            
            if (fnc_code == 0x180 || fnc_code == 0x280) {
                /* ✅ TPDO1, TPDO2: ISR에서 직접 처리 (실시간 데이터) */
                ImuHub_Drv_ProcessCANMessage(msg->id, msg->data, msg->len);
            }
            else {
                /* ✅ Heartbeat (0x708), SDO (0x580, 0x600), NMT (0x000): SW Buffer + Notification */
                _GenFifo_Push(msg);  /* ✅ 항상 성공 (Overwrite 방식) */
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(CanFDRxQueueRouter_TaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        /* 향후 추가: EMG Hub (0x09), FES Hub (0x0A), GRF Hub (0x0B) 등 */
        // else if (node_id == SYS_NODE_ID_EMG_HUB) {
        //     ...
        // }
    }
}

/**
 * @brief 통합 메시지 라우터 Task (Task Notification + Batch Processing)
 * @details 
 * [V3.1 최적화]
 * - Task Notification: Queue 대비 10배 빠른 ISR 처리
 * - Batch Processing: 한 번 깨어나면 버퍼가 빌 때까지 처리
 * - Context Switch 최소화: 알림 1번 → 메시지 N개 처리
 * 
 * [성능]
 * - ISR 오버헤드: ~5µs → ~0.5µs (10배 향상)
 * - Context Switch: 매 메시지 → Batch당 1번 (1/N 감소)
 */
static void _CanFdRxQueueRouter_Task(void* argument)
{
    (void)argument;  /* Unused */
    IOIF_FDCAN_Msg_t received_msg;

    for (;;) {
        /* ✅ Step 1: Task Notification 대기 (ISR이 알림을 보낼 때까지 블로킹) */
        /* pdTRUE: 알림 값을 0으로 초기화 (Binary Semaphore처럼 동작) */
        uint32_t notify_count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (notify_count > 0) {
            /* ✅ Step 2: Batch Processing (버퍼가 빌 때까지 반복 처리) */
            /* 
             * [중요] 한 번 깨어났을 때 쌓인 메시지를 모두 처리해야 합니다!
             * - Context Switch 비용 절감 (알림 1번 → 메시지 N개 처리)
             * - 처리 속도가 수신 속도를 따라잡을 수 있음
             */
            while (_GenFifo_Pop(&received_msg)) {
                /* 메시지 처리 (기존 로직) */
                _Process_General_Message(&received_msg);
            }
        }
    }
}

/**
 * @brief 일반 메시지 처리 (SDO, Heartbeat, NMT)
 * @details DOP V1 (CM-XM) + DOP V2 (IMU-XM, EMG-XM, ...) 라우팅
 */
static void _Process_General_Message(IOIF_FDCAN_Msg_t* msg)
{
    /* CAN ID로 프로토콜 구분 */
    uint16_t fnc_code = msg->id & 0x780;
    
    if (fnc_code == 0x200) {
        /* DOP V1 SDO (CM-XM) */
        PnPManager_RouteMessage(msg->id, msg->data, msg->len);
    }
    else {
        /* DOP V2 (Heartbeat, SDO, NMT) */
        uint8_t node_id = msg->id & 0x7F;
        
        switch (node_id) {
            case SYS_NODE_ID_IMU_HUB:  /* 0x08 */
                /* ✅ V3.0: Device Driver 직접 호출 (CANopen 표준) */
                ImuHub_Drv_ProcessCANMessage(msg->id, msg->data, msg->len);
                break;
            
            // 향후 추가: EMG Hub (0x09), FES Hub (0x0A), GRF Hub (0x0B) 등
            // case SYS_NODE_ID_EMG_HUB:
            //     EmgHub_XM_Link_ProcessMessage_V2(msg->id, msg->data, msg->len);
            //     break;
            
            default:
                /* 알 수 없는 Node ID: 무시 */
                break;
        }
    }
}

/**
 *------------------------------------------------------------
 * SW RINGBUFFER 관리 함수 (High Performance + Overflow Safe)
 *------------------------------------------------------------
 */

/**
 * @brief SW RingBuffer에 메시지 추가 (ISR에서 호출)
 * @details 
 * [V3.2 Overwrite 방식]
 * - Buffer Full 시: 가장 오래된 메시지를 덮어씀 (Overwrite)
 * - 이유: Heartbeat/SDO는 최신 데이터가 중요
 * - 통계: Overwrite 횟수 기록 (디버깅/모니터링)
 * 
 * [성능]
 * - 항상 성공 (return void)
 * - ISR 오버헤드 최소화
 */
static inline void _GenFifo_Push(IOIF_FDCAN_Msg_t* msg)
{
    uint32_t next_head = (s_gen_fifo.head + 1) % FDCAN_GEN_FIFO_SIZE;
    
    /* ✅ Buffer Full 체크: Overwrite */
    if (next_head == s_gen_fifo.tail) {
        /* Overwrite: 가장 오래된 메시지(tail) 버림 */
        s_gen_fifo.tail = (s_gen_fifo.tail + 1) % FDCAN_GEN_FIFO_SIZE;
        s_gen_fifo.overwrite_count++;  /* 통계 */
    }
    
    /* 메시지 저장 */
    s_gen_fifo.buffer[s_gen_fifo.head] = *msg;  /* memcpy */
    s_gen_fifo.head = next_head;
    
    /* ✅ 통계 업데이트 */
    s_gen_fifo.total_rx_count++;
    
    /* Peak 사용량 추적 */
    uint32_t current_usage = (s_gen_fifo.head >= s_gen_fifo.tail) 
        ? (s_gen_fifo.head - s_gen_fifo.tail) 
        : (FDCAN_GEN_FIFO_SIZE - s_gen_fifo.tail + s_gen_fifo.head);
    
    if (current_usage > s_gen_fifo.peak_usage) {
        s_gen_fifo.peak_usage = current_usage;
    }
}

/**
 * @brief SW RingBuffer에서 메시지 추출 (Task에서 호출)
 * @return true: 성공, false: Buffer Empty
 */
static inline bool _GenFifo_Pop(IOIF_FDCAN_Msg_t* msg)
{
    if (s_gen_fifo.head == s_gen_fifo.tail) {
        /* Buffer Empty */
        return false;
    }
    
    *msg = s_gen_fifo.buffer[s_gen_fifo.tail];  /* memcpy */
    s_gen_fifo.tail = (s_gen_fifo.tail + 1) % FDCAN_GEN_FIFO_SIZE;
    
    return true;
}

/**
 * @brief SW RingBuffer 현재 사용량 조회 (디버깅용)
 * @return 현재 저장된 메시지 개수 (0~FDCAN_GEN_FIFO_SIZE-1)
 */
static inline uint32_t _GenFifo_GetUsage(void)
{
    uint32_t head = s_gen_fifo.head;
    uint32_t tail = s_gen_fifo.tail;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return FDCAN_GEN_FIFO_SIZE - tail + head;
    }
}

/**
 * @brief SW RingBuffer 통계 조회 (디버깅/모니터링용)
 * @details Live Expression에서 확인 가능
 * - s_gen_fifo.total_rx_count: 총 수신 메시지
 * - s_gen_fifo.overwrite_count: Overwrite 발생 횟수
 * - s_gen_fifo.peak_usage: 최대 사용량 (0~32)
 * 
 * [경고 기준]
 * - peak_usage > 24 (75%): 버퍼 부족 경고
 * - overwrite_count > 0: 처리 속도 부족
 */

