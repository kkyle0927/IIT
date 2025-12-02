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

/**
 *-----------------------------------------------------------
 *              PRIVATE DEFINITIONS AND TYPES
 *-----------------------------------------------------------
 */

// FDCAN 인스턴스의 내부 상태를 관리하는 구조체 (Private)
typedef struct {
    bool                    is_assigned;
    FDCAN_HandleTypeDef*    hfdcan;
    IOIF_FDCAN_FilterCallback_t filter_callback;
    IOIF_FDCAN_RxCallback_t rx_callback;
#if defined(USE_FREERTOS_DMA)
    SemaphoreHandle_t       rx_sem;
    osThreadId_t            rx_task_handle;
#endif
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
static uint32_t _Len2DLC(uint32_t len);
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
    inst->filter_callback = NULL; // 필터 콜백을 NULL로 초기화
    inst->rx_callback = NULL; // 콜백을 NULL로 초기화
    // RTOS 객체 초기화
#if defined(USE_FREERTOS_DMA)
    inst->rx_sem = NULL;
    inst->rx_task_handle = NULL;
#endif
    
    *id = s_instance_count; // 사용자에게 핸들(ID) 반환
    s_instance_count++;

    return AGRBStatus_OK;
}

// [변경] Start 함수를 RTOS 여부에 따라 다르게 구현
#if defined(USE_FREERTOS_DMA)
// --- RTOS용 Start 함수 ---
AGRBStatusDef IOIF_FDCAN_Start(IOIF_FDCANx_t id, const osThreadAttr_t* rx_task_attr)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    
    IOIF_FDCAN_Inst_t* inst = &s_fdcan_instances[id];
    FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;

    // 1. RTOS 자원 생성
    inst->rx_sem = xSemaphoreCreateBinary();
    if (inst->rx_sem == NULL) return AGRBStatus_ERROR;

    inst->rx_task_handle = osThreadNew(_IOIF_FDCAN_RxTask, (void*)inst, rx_task_attr);
    if (inst->rx_task_handle == NULL) return AGRBStatus_ERROR;

    // 2. 모든 메시지를 수신하는 기본 필터 설정
    FDCAN_FilterTypeDef sFilterConfig = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, // FIFO0만 사용하기로 결정
        .FilterID1 = 0x000,
        .FilterID2 = 0x7FF,
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) return AGRBStatus_ERROR;

    // 3. 글로벌 필터 설정
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return AGRBStatus_ERROR;
    
    // 4. 수신 인터럽트 활성화
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return AGRBStatus_ERROR;
    
    // 5. 송신 지연 보상(TDC) 설정
    if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1, IOIF_TDC_FILTER) != HAL_OK) return AGRBStatus_ERROR;
    
    // 6. 송신 지연 보상(TDC) 활성화
    if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    // 7. FDCAN 시작
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    return AGRBStatus_OK;
}
#else
// --- Bare-metal용 Start 함수 ---
AGRBStatusDef IOIF_FDCAN_Start(IOIF_FDCANx_t id)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }
    
    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;

    // 1. 모든 메시지를 수신하는 기본 필터 설정
    FDCAN_FilterTypeDef sFilterConfig = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, // FIFO0만 사용하기로 결정
        .FilterID1 = 0x000,
        .FilterID2 = 0x7FF,
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) return AGRBStatus_ERROR;

    // 2. 글로벌 필터 설정
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) return AGRBStatus_ERROR;
    
    // 3. 수신 인터럽트 활성화
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) return AGRBStatus_ERROR;
    
    // 4. 송신 지연 보상(TDC) 설정
    if (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, hfdcan->Init.DataPrescaler * hfdcan->Init.DataTimeSeg1, IOIF_TDC_FILTER) != HAL_OK) return AGRBStatus_ERROR;
    
    // 5. 송신 지연 보상(TDC) 활성화
    if (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    // 6. FDCAN 시작
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return AGRBStatus_ERROR;

    return AGRBStatus_OK;
}
#endif // IOIF_FDCAN_Start

AGRBStatusDef IOIF_FDCAN_Transmit(IOIF_FDCANx_t id, uint16_t msgId, uint8_t* txData, uint32_t len)
{
    if (id >= s_instance_count || !s_fdcan_instances[id].is_assigned) {
        return AGRBStatus_NOT_INITIALIZED;
    }

    FDCAN_HandleTypeDef* hfdcan = s_fdcan_instances[id].hfdcan;
    
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = msgId,
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
        return AGRBStatus_ERROR;
    }

    return AGRBStatus_OK;
}

// 필터 콜백 등록 함수 구현
void IOIF_FDCAN_RegisterFilterCallback(IOIF_FDCANx_t id, IOIF_FDCAN_FilterCallback_t callback)
{
    if (id < s_instance_count && s_fdcan_instances[id].is_assigned) {
        s_fdcan_instances[id].filter_callback = callback;
    }
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
    if (instance == NULL || instance->rx_callback == NULL) {
        // 주인이 없거나, 콜백이 등록되지 않은 메시지는 무시
        return;
    }
    
#if defined(USE_FREERTOS_DMA)
    // --- RTOS 모드: 세마포어만 Give ---
    if (instance->rx_sem != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#else
    // --- Bare-metal 모드: ISR에서 직접 처리 ---
    // 상위 계층으로 전달할 메시지를 담을 구조체
    IOIF_FDCAN_Msg_t received_msg;
    FDCAN_RxHeaderTypeDef rxHeader;

    // 하드웨어 FIFO0에서 메시지를 읽어옵니다.
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, received_msg.data) != HAL_OK) {
        return; // 에러 처리
    }

    // 1. 필터 콜백 수행 (System Layer 정책)
    // 등록된 소프트웨어 필터 콜백이 있다면, 필터링을 먼저 수행합니다.
    if (instance->filter_callback != NULL) {
        if (instance->filter_callback(rxHeader.Identifier) == false) {
            return; // 필터 콜백이 false를 반환하면, 여기서 즉시 ISR 종료
        }
    }

    // 2. 메시지 파싱
    // 수신된 헤더 정보를 IOIF_FDCAN_Msg_t 구조체에 맞게 파싱합니다.
    received_msg.id = rxHeader.Identifier;
    // 32비트 DataLength 필드에서 4비트 DLC 값(0~15)을 추출
    uint8_t dlc = rxHeader.DataLength >> 16;
    // DLC 값을 실제 바이트 길이(0~64)로 변환
    received_msg.len = _DLC2Len(dlc);

    // 3. Rx 콜백 호출 (System Layer 정책)
    // 수신 콜백 함수가 등록되어 있는지 확인하고, 등록된 경우에만 호출
    if (instance->rx_callback != NULL) {
        instance->rx_callback(&received_msg);
    }
#endif
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

static uint32_t _Len2DLC(uint32_t len)
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

// RTOS 사용 시에만 Rx 태스크 함수 구현
#if defined(USE_FREERTOS_DMA)
static void _IOIF_FDCAN_RxTask(void* argument)
{
    IOIF_FDCAN_Inst_t* inst = (IOIF_FDCAN_Inst_t*)argument;
    FDCAN_HandleTypeDef* hfdcan = inst->hfdcan;

    IOIF_FDCAN_Msg_t received_msg;
    FDCAN_RxHeaderTypeDef rxHeader;

    for (;;) {
        // 1. ISR이 세마포어를 줄 때까지 대기
        if (xSemaphoreTake(inst->rx_sem, portMAX_DELAY) == pdPASS) {
            // 2. FIFO 메시지 읽음 (Bare-metal ISR과 동일 로직)
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, received_msg.data) != HAL_OK) {
                break;
            }

            // 3. 필터 콜백 수행
            if (inst->filter_callback != NULL) {
                if (inst->filter_callback(rxHeader.Identifier) == false) {
                    continue;
                }
            }

            // 4. 메시지 파싱
            received_msg.id = rxHeader.Identifier;
            uint8_t dlc = (uint8_t)rxHeader.DataLength;
            received_msg.len = _DLC2Len(dlc);

            // 5. Rx 콜백 호출
            if (inst->rx_callback != NULL) {
                inst->rx_callback(&received_msg);
            }
        }
    }
}
#endif // USE_FREERTOS_DMA

#endif /* AGRB_IOIF_FDCAN_ENABLE */
