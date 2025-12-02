/**
 ******************************************************************************
 * @file    ioif_agrb_uart.c
 * @author  HyundoKim
 * @brief   [IOIF Layer] UART 하드웨어 추상화 계층 구현
 * @version 0.1
 * @date    Nov 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_uart.h"
#if defined(AGRB_IOIF_UART_ENABLE)

#include "module.h"

#include <string.h>

#if defined(USE_FREERTOS_DMA)
#include "cmsis_os2.h" // RTOS
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#if defined(USE_FREERTOS_DMA)
#define IOIF_UART_RX_BOUNCE_BUFFER_SIZE     (512)

#define IOIF_UART_TX_DMA_BUFFER_SIZE        (256)
#define IOIF_UART_RX_DMA_BUFFER_SIZE        (2048) // 링버퍼 크기

// 세마포어 타임아웃 (5초)
#define UART_TX_TIMEOUT_MS                  (5000)
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

typedef struct {
    bool allocated;
    IOIF_UARTx_t id;
    UART_HandleTypeDef* huart;
    IOIF_UART_Config_t config; //  rxMode 포함

#if defined(USE_FREERTOS_DMA)
    SemaphoreHandle_t tx_semaphore; 
    osThreadId_t rx_task_handle; // (POLLING 모드 전용)
    
    struct {
        uint32_t tail; // 링버퍼 읽기 위치
    } ringbuffer; // (POLLING 모드 전용)
    
    uint8_t* rx_dma_ptr; // 할당된 DMA 버퍼 포인터
#endif
} IOIF_UART_Instance_t;

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

static IOIF_UART_Instance_t _uart_instances[IOIF_UART_MAX_INSTANCES] = {0};
static uint32_t _uart_instance_count = 0;

// [DMA Buffer] Non-Cacheable 영역에 배치 (필수)
#if defined(USE_FREERTOS_DMA)
__attribute__((section(IOIF_DMA_SECTION), aligned(32)))
static uint8_t _uart_tx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_TX_DMA_BUFFER_SIZE];

__attribute__((section(IOIF_DMA_SECTION), aligned(32)))
static uint8_t _uart_rx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_RX_DMA_BUFFER_SIZE];
#endif

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart);
static AGRBStatusDef _convert_hal_status(HAL_StatusTypeDef hal_status);

#if defined(USE_FREERTOS_DMA)
static void _uart_rx_task(void* argument); // (POLLING 모드 전용)
#endif

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

AGRBStatusDef ioif_uart_assign_instance(IOIF_UARTx_t* id, UART_HandleTypeDef* huart, IOIF_UART_Config_t* config)
{
    if (id == NULL || huart == NULL || config == NULL) return AGRBStatus_PARAM_ERROR;

    *id = IOIF_UART_ID_NOT_ALLOCATED;

    if (_uart_instance_count >= IOIF_UART_MAX_INSTANCES) {
        return AGRBStatus_INITIAL_FAILED; 
    }

    // 중복 할당 검사
    for (uint32_t i = 0; i < _uart_instance_count; i++) {
        if (_uart_instances[i].huart == huart && _uart_instances[i].allocated) {
            return AGRBStatus_BUSY; 
        }
    }

    // 인스턴스 할당
    IOIF_UART_Instance_t* instance = &_uart_instances[_uart_instance_count];
    memset(instance, 0, sizeof(IOIF_UART_Instance_t));

    instance->huart = huart;
    memcpy(&(instance->config), config, sizeof(IOIF_UART_Config_t));
    instance->id = _uart_instance_count;

#if defined(USE_FREERTOS_DMA)
    instance->rx_dma_ptr = _uart_rx_dma_buffer[instance->id];
    
    // TX 세마포어 생성 (공통)
    instance->tx_semaphore = xSemaphoreCreateBinary();
    if (instance->tx_semaphore == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    xSemaphoreGive(instance->tx_semaphore); // 초기 상태: 사용 가능

    // [!! 핵심 수정 !!]
    // HAL_UART_Init()이 CubeMX 설정(예: Circular)으로 DMA를 초기화했으므로,
    // DeInit으로 해제하고 config->rxMode에 맞게 다시 초기화합니다.
    if (huart->hdmarx != NULL) {
        // 1. 기존 DMA 설정 해제
        HAL_DMA_DeInit(huart->hdmarx);
        
        // 2. config->rxMode에 맞는 새 DMA 설정
        if (config->rxMode == IOIF_UART_MODE_POLLING_TASK) {
            huart->hdmarx->Init.Mode = DMA_CIRCULAR; // 폴링은 Circular
        } else {
            huart->hdmarx->Init.Mode = DMA_NORMAL;   // 이벤트는 Normal
        }
        
        // 3. 새 설정으로 DMA 재초기화
        if (HAL_DMA_Init(huart->hdmarx) != HAL_OK) {
             vSemaphoreDelete(instance->tx_semaphore);
             return AGRBStatus_INITIAL_FAILED;
        }
    }

    // [수정] config.rxMode에 따라 분기
    if (config->rxMode == IOIF_UART_MODE_POLLING_TASK) {
        // Polling 모드: RX 태스크 생성
        instance->ringbuffer.tail = 0;
        const osThreadAttr_t task_attr = {
            .name = "UartRxTask",
            .priority = TASK_PRIO_UART_RX,
            .stack_size = TASK_STACK_UART_RX
        };
        instance->rx_task_handle = osThreadNew(_uart_rx_task, instance, &task_attr);
        if (instance->rx_task_handle == NULL) {
            vSemaphoreDelete(instance->tx_semaphore);
            return AGRBStatus_ERROR;
        }
    } else {
        // IDLE_EVENT 모드: 태스크 생성 안 함
        instance->rx_task_handle = NULL;
    }
#endif

    instance->allocated = true;
    *id = _uart_instance_count++;

    // 즉시 수신 시작
    return ioif_uart_start(*id);
}

AGRBStatusDef ioif_uart_start(IOIF_UARTx_t id)
{
#if defined(USE_FREERTOS_DMA)
    if (id >= _uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

    // RX 버퍼 초기화
    memset(instance->rx_dma_ptr, 0, IOIF_UART_RX_DMA_BUFFER_SIZE);

    // 에러 플래그 클리어
    __HAL_UART_CLEAR_OREFLAG(instance->huart);
    __HAL_UART_CLEAR_NEFLAG(instance->huart);
    __HAL_UART_CLEAR_FEFLAG(instance->huart);
    __HAL_UART_CLEAR_PEFLAG(instance->huart);

    // [수정] config.rxMode에 따라 다른 수신 함수 호출
    if (instance->config.rxMode == IOIF_UART_MODE_POLLING_TASK) {
        instance->ringbuffer.tail = 0;
        // 1. Polling 방식: Circular DMA 시작
        if (HAL_UART_Receive_DMA(instance->huart, instance->rx_dma_ptr, IOIF_UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
            return AGRBStatus_ERROR;
        }
    } else {
        // 2. Event 방식: Idle DMA 시작 (DMA Normal mode)
        // if (HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, instance->rx_dma_ptr, IOIF_UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
        //     return AGRBStatus_ERROR;
        // }
    }
    return AGRBStatus_OK;
#else
    return AGRBStatus_NOT_SUPPORTED;
#endif
}

AGRBStatusDef ioif_uart_write(IOIF_UARTx_t id, uint8_t* tx_buf, uint32_t size)
{
    if (id >= _uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

#if defined(USE_FREERTOS_DMA)
    if (size > IOIF_UART_TX_DMA_BUFFER_SIZE) return AGRBStatus_BUFFER_OVERFLOW;

    // 1. 세마포어 대기 (이전 전송 완료 대기)
    if (xSemaphoreTake(instance->tx_semaphore, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS)) != pdTRUE) {
        return AGRBStatus_TIMEOUT;
    }

    // 2. DMA 버퍼로 복사 (Cache Coherency를 위해)
    uint8_t* dma_buf = _uart_tx_dma_buffer[id];
    memcpy(dma_buf, tx_buf, size);

    // 3. Cache Clean (CPU -> RAM)
    SCB_CleanDCache_by_Addr((uint32_t*)dma_buf, size);

    // 4. 전송 시작
    if (HAL_UART_Transmit_DMA(instance->huart, dma_buf, size) != HAL_OK) {
        xSemaphoreGive(instance->tx_semaphore); // 실패 시 반환
        return AGRBStatus_ERROR;
    }
    // 성공 시: ISR에서 xSemaphoreGive 호출됨
    return AGRBStatus_OK;
#else
    return _convert_hal_status(HAL_UART_Transmit(instance->huart, tx_buf, size, 1000));
#endif
}

AGRBStatusDef ioif_uart_update_rx_callback(IOIF_UARTx_t id, IOIF_UART_RxEventCallback_t callback)
{
    // 1. 예외 처리
    if (id >= _uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &_uart_instances[id];

    // 2. 콜백 등록 (기존 코드)
    instance->config.rx_event_callback = callback;

    // 3. [핵심 추가] 콜백 등록과 동시에 "수신 시작(Start)" 명령 전달
    // 이 코드가 없으면 인터럽트가 절대 발생하지 않습니다.
    if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        // [디버깅 포인트] 여기서 instance->huart->RxState 값을 확인하세요.
        // 0x20(READY)이면 시작해야 하고, 0x22(BUSY_RX)면 이미 도는 중입니다.
        if (instance->huart->RxState == HAL_UART_STATE_READY) {
            // 멈춰있을 때만 시작 명령
            if (HAL_UARTEx_ReceiveToIdle_DMA(instance->huart,
                                    instance->rx_dma_ptr,
                                    IOIF_UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
                return AGRBStatus_ERROR;
            }
        }
        // else: 이미 BUSY라면, 콜백만 교체했으니 성공으로 간주하고 통과
    }
    // (참고: Polling 모드라면 여기서 시작하지 않고 태스크 루프에서 처리할 수도 있음)

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_uart_flush(IOIF_UARTx_t id)
{
#if defined(USE_FREERTOS_DMA)
    if (id >= _uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &_uart_instances[id];
    instance->ringbuffer.tail = IOIF_UART_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
    return AGRBStatus_OK;
#else
    return AGRBStatus_NOT_SUPPORTED;
#endif
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

#if defined(USE_FREERTOS_DMA)
/**
 * @brief UART 수신 처리 태스크 (High-speed polling)
 */
static void _uart_rx_task(void* argument)
{
    IOIF_UART_Instance_t* instance = (IOIF_UART_Instance_t*)argument;
    uint8_t bounce_buffer[IOIF_UART_RX_BOUNCE_BUFFER_SIZE];
    uint8_t* dma_base = instance->rx_dma_ptr;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 1ms 주기

    for (;;) {
        // 1. 주기 대기 (가장 먼저 수행하여 CPU 점유 방지)
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // 2. 수신된 데이터 양 계산 (NDTR 레지스터 활용)
        // NDTR은 감소하는 카운터임
        uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
        uint32_t head = IOIF_UART_RX_DMA_BUFFER_SIZE - dma_cnt;
        uint32_t tail = instance->ringbuffer.tail;
        
        uint32_t data_len = 0;
        if (head >= tail) {
            data_len = head - tail;
        } else {
            data_len = (IOIF_UART_RX_DMA_BUFFER_SIZE - tail) + head;
        }

        if (data_len == 0) continue;

        // 3. 데이터 처리 루프 (버퍼가 클 경우 쪼개서 처리)
        while (data_len > 0) {
            uint32_t chunk_size = (data_len > IOIF_UART_RX_BOUNCE_BUFFER_SIZE) ? IOIF_UART_RX_BOUNCE_BUFFER_SIZE : data_len;
            
            // Wrap-around 처리
            if (tail + chunk_size <= IOIF_UART_RX_DMA_BUFFER_SIZE) {
                // 연속된 데이터
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[tail], chunk_size);
                memcpy(bounce_buffer, &dma_base[tail], chunk_size);
                tail += chunk_size;
            } else {
                // 끝부분 + 앞부분
                uint32_t first_part = IOIF_UART_RX_DMA_BUFFER_SIZE - tail;
                uint32_t second_part = chunk_size - first_part;
                
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[tail], first_part);
                SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[0], second_part);
                
                memcpy(bounce_buffer, &dma_base[tail], first_part);
                memcpy(&bounce_buffer[first_part], &dma_base[0], second_part);
                tail = second_part;
            }
            
            // 콜백 호출 (System Layer로 전달)
            if (instance->config.rx_event_callback != NULL) {
                instance->config.rx_event_callback(bounce_buffer, chunk_size, instance->id);
            }
            
            data_len -= chunk_size;
        }
        
        // Tail 업데이트
        instance->ringbuffer.tail = tail;
    }
}
#endif

static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart)
{
    for (uint32_t i = 0; i < _uart_instance_count; i++) {
        if (_uart_instances[i].huart == huart) return &_uart_instances[i];
    }
    return NULL;
}

static inline AGRBStatusDef _convert_hal_status(HAL_StatusTypeDef hal_status)
{
    if (hal_status == HAL_OK) return AGRBStatus_OK;
    if (hal_status == HAL_BUSY) return AGRBStatus_BUSY;
    if (hal_status == HAL_TIMEOUT) return AGRBStatus_TIMEOUT;
    return AGRBStatus_ERROR;
}

/**
 * @brief HAL UART Tx Complete Callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
#if defined(USE_FREERTOS_DMA)
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance && instance->tx_semaphore) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#endif
}

/**
 * @brief UART IDLE Event 콜백 (Event 모드 전용)
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

    // 이 콜백은 IDLE_EVENT 모드인 인스턴스만 처리
    if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        if (Size > 0) {
            // 1. 캐시 무효화 (DMA -> CPU)
            SCB_InvalidateDCache_by_Addr((uint32_t*)instance->rx_dma_ptr, Size);
            
            // 2. System Layer로 데이터 전달
            if (instance->config.rx_event_callback != NULL) {
                instance->config.rx_event_callback(instance->rx_dma_ptr, Size, instance->id);
            }
        }
        
        // 3. [필수] 다음 Idle 이벤트를 받기 위해 수신을 즉시 재시작
        if (instance->huart->RxState == HAL_UART_STATE_READY) {
        	HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, instance->rx_dma_ptr, IOIF_UART_RX_DMA_BUFFER_SIZE);
        }
    }
}

/**
 * @brief UART 오류 콜백 (노이즈 등으로 인한 통신 오류 처리)
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance && instance->allocated) 
    {
        // 1. [필수] 모든 에러 플래그 클리어 (이걸 안 하면 통신 복구 안됨)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 2. 수신 재시작 (모드에 따라 분기)
        if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT)
        {
            // IDLE Event 모드 재시작
            HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, 
                                         instance->rx_dma_ptr, 
                                         IOIF_UART_RX_DMA_BUFFER_SIZE);
        }
        else if (instance->config.rxMode == IOIF_UART_MODE_POLLING_TASK)
        {
            // Polling 모드 재시작
            HAL_UART_Receive_DMA(instance->huart, 
                                 instance->rx_dma_ptr, 
                                 IOIF_UART_RX_DMA_BUFFER_SIZE);
        }
    }
}

#endif /* AGRB_IOIF_UART_ENABLE */
