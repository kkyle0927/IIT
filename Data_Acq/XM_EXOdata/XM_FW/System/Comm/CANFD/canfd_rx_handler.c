/**
 ******************************************************************************
 * @file    canfd_rx_handler.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Oct 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "canfd_rx_handler.h"
#include "data_object_dictionaries.h"
#include "ioif_agrb_fdcan.h" // IOIF_FDCAN_Msg_t 구조체를 위해 포함
#include "cm_drv.h"
#include "pnp_manager.h" // cm_xm_link.h 대신 pnp_manager.h를 포함
#include "system_startup.h"

#if defined(USE_FREERTOS_DMA)
// FreeRTOS 및 CMSIS-OS 관련 헤더
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

#if defined(USE_FREERTOS_DMA)
#define CANFD_RX_QUEUE_RX_QUEUE_SIZE 20 // FDCAN Rx 메시지를 담을 RTOS 큐 크기

// CANFD Rx Queue Router Task의 속성 정의
static osThreadId_t CanFDRxQueueRouter_TaskHandle;
static const osThreadAttr_t CanFDRxQueueRouter_Task_attributes = {
  .name = "CanFDRxQueueRouter_Task",
  .stack_size = TASK_STACK_SDO_ROUTER,
  .priority = (osPriority_t) TASK_PRIO_SDO_ROUTER,
};

static QueueHandle_t s_rx_queue; // Rx queue 메시지만을 위한 큐
#endif // USE_FREERTOS_DMA

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static bool _CanFdRxFilterCallback(uint16_t canId); // 필터링 규칙 함수
static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg); // Rx Data 콜백
#if defined(USE_FREERTOS_DMA)
static void _CanFdRxQueueRouter_Task(void* argument); // Rx queue만 처리함
#endif // USE_FREERTOS_DMA

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void CanFdRxHandler_Init(void)
{
#if defined(USE_FREERTOS_DMA)
    // RTOS 모드: SDO 처리를 위한 큐 생성
    s_rx_queue = xQueueCreate(CANFD_RX_QUEUE_RX_QUEUE_SIZE, sizeof(IOIF_FDCAN_Msg_t));
#endif

    // system_startup으로부터 FDCAN1의 ID를 가져옵니다.
    IOIF_FDCANx_t fdcan1_id = System_GetFDCAN1_Id();

    // IOIF 드라이버에 콜백 함수 등록 (RTOS/Bare-metal 공통)
    // fdcan1_id는 system_startup에서 초기화, getter 함수로 접근
    IOIF_FDCAN_REGISTER_FILTER_CALLBACK(fdcan1_id, _CanFdRxFilterCallback); // ID 필터링 콜백
    IOIF_FDCAN_REGISTER_CALLBACK(fdcan1_id, _CanFdRxCallback); // 수신 데이터 처리 콜백

#if defined(USE_FREERTOS_DMA)
    // RTOS 모드: SDO 큐 처리 태스크 생성
    CanFDRxQueueRouter_TaskHandle = osThreadNew(_CanFdRxQueueRouter_Task, s_rx_queue, &CanFDRxQueueRouter_Task_attributes);
#endif
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IOIF FDCAN 드라이버에 등록될 '필터링 규칙' 콜백 함수.
 * @details 이 시스템에서 처리해야 할 메시지인지를 1차로 판별합니다.
 * XM10 자신에게 온 메시지(Unicast) 또는 모든 모듈에게 보내는
 * 메시지(Broadcast)만 통과시킵니다.
 */
static bool _CanFdRxFilterCallback(uint16_t canId)
{
    // CAN ID의 가장 낮은 4비트는 '목적지 Node ID'를 나타냅니다.
    uint8_t destination_node_id = canId & 0x00F;

    // 목적지가 '모두(Broadcast, ID=0)'이거나 '나 자신(XM10)'인 경우에만 통과시킵니다.
    if (destination_node_id == 0 || destination_node_id == SYS_NODE_ID_XM) {
        return true; // 이 메시지는 처리해야 함 (통과)
    }
    
    // 그 외의 모든 메시지는 무시합니다.
    return false; // 거부
}

/**
 * @brief IOIF FDCAN 드라이버에 의해 ISR 컨텍스트에서 호출될 메인 수신 콜백. (정책 구현)
 * @details 모든 수신 메시지의 '1차 분류'를 담당합니다.
 */
static void _CanFdRxCallback(IOIF_FDCAN_Msg_t* msg)
{
    // 발신자 Node ID를 기반으로 소프트웨어 필터링
    uint8_t source_node_id = (msg->id & 0x0F0) >> 4;
    // PDO/SDO 구분
    uint16_t fncCode = msg->id & 0x700;
    
#if defined(USE_FREERTOS_DMA)
    // --- RTOS 모드 (Task Context) ---
    switch (fncCode) {
        case PDO: // PDO -> 공유 메모리 (Mutex는 CM_UpdatePdoData 내부에서 처리)
            // 해당 Node ID를 처리하는 Devices 계층의 PDO 업데이트 함수 호출
            switch (source_node_id) {
                case SYS_NODE_ID_CM:
                    CM_UpdatePdoData(msg->data, msg->len); // Mutex 처리 등은 CM_UpdatePdoData 내부에서 담당
                    break;
                // ... 6개 모듈 ...
                // case SYS_NODE_ID_EMG:
                //     EMG_UpdatePdoData(msg->data, msg->len);
                //     break;
            }
            break;
        case SDO: // SDO -> RTOS 큐로 전송
            // SDO 메시지는 Rx Queue 태스크가 처리하도록 큐에 넣습니다.
            xQueueSend(s_rx_queue, msg, (TickType_t)0);
            break;
        // EMCY, SYNC 등 다른 메시지 유형도 여기서 처리 가능
        default:
            break;
    }
#else
    // --- Bare-metal 모드 (ISR Context) ---
    switch (fncCode) {
        case PDO: // PDO -> 공유 메모리 (직접 덮어쓰기)
            if (source_node_id == SYS_NODE_ID_CM) {
                // (가정) CM_UpdatePdoData_ISR_Safe가 ISR에서 호출 가능하도록 작성됨
                // CM_UpdatePdoData_ISR_Safe(msg->data, msg->len); 
            }
            break;
        case SDO: // SDO -> 직접 처리 또는 Ring Buffer
            // (가정) PnPManager_RouteMessage_ISR_Safe가 ISR에서 호출 가능하도록 작성됨
            // PnPManager_RouteMessage_ISR_Safe(msg->id, msg->data, msg->len);
            break;
    }
#endif
}

#if defined(USE_FREERTOS_DMA)
/**
 * @brief CAN 메시지 수신 및 분배를 담당하는 메인 태스크 함수
 * SDO Queue 처리 Task는 RTOS 사용 시에만 구현
 */
static void _CanFdRxQueueRouter_Task(void* argument)
{
    // StartupAPP에서 전달받은 큐 핸들을 캐스팅하여 사용
    QueueHandle_t rx_queue = (QueueHandle_t)argument;

    // 수신된 메시지를 저장할 로컬 변수
    IOIF_FDCAN_Msg_t received_msg;

    for (;;) {
        // 큐에 메시지가 들어올 때까지 무한정 대기 (CPU 소모 0)
        // 이벤트가 발생하면 즉시 깨어남
        if (xQueueReceive(rx_queue, &received_msg, portMAX_DELAY) == pdPASS) {
            // SDO 메시지는 PnP 매니저에게 라우팅하여 메세지 처리
            PnPManager_RouteMessage(received_msg.id, received_msg.data, received_msg.len);
        }
    }
}
#endif // USE_FREERTOS_DMA
