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

/* 
 * [빌드 제어] AGRB_IOIF_UART_ENABLE 체크
 * - 목적: 프로젝트에서 사용하지 않는 IOIF 모듈이 빌드되지 않도록 제어
 *         (예: IMU Hub Module에서는 FS 모듈이 복사되어 있지만 빌드 안 함)
 * - 헤더 파일(.h)에서 먼저 체크하고, .c 파일에서 한 번 더 체크
 * - 편집기 음영 문제: ioif_agrb_defs.h를 include path에 추가하면 해결
 */
#if defined(AGRB_IOIF_UART_ENABLE)

#include <string.h>

#if defined(USE_FREERTOS_DMA)
#include "cmsis_os2.h" // RTOS
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/* 공통: TX/RX DMA 버퍼 크기 */
#define IOIF_UART_TX_DMA_BUFFER_SIZE        (128)   /**< TX DMA 버퍼 (RTOS/BareMetal 공통) */
#define IOIF_UART_RX_DMA_BUFFER_SIZE        (256)  /**< RX DMA 버퍼 (RTOS/BareMetal 공통) */

#if defined(USE_FREERTOS_DMA)
    /* RTOS 전용 */
    #define UART_TX_SEMAPHORE_TIMEOUT_MS        (5000)  /**< TX 세마포어 타임아웃 */
    #define UART_TX_POLLING_TIMEOUT_MS          (5000)  /**< TX Polling 타임아웃 */
#else
    /* BareMetal 전용 */
    #define UART_TX_POLLING_TIMEOUT_MS          (1000)  /**< TX Polling 타임아웃 */
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
    osThreadId_t rx_task_handle; // (POLLING 모드 전용)
    
    struct {
        uint32_t tail; // 링버퍼 읽기 위치
    } ringbuffer; // (POLLING 모드 전용)

    SemaphoreHandle_t tx_semaphore; 
    uint8_t* rx_dma_ptr; // 할당된 DMA 버퍼 포인터

    /* Circular DMA용 소프트웨어 Tail 포인터 추가 */
    /* HAL_UARTEx_RxEventCallback의 Size 파라미터와 비교하여 처리 */
    volatile uint16_t rx_old_pos;

#else // USE_BAREMETAL
    uint8_t* rx_dma_ptr;    // 할당된 DMA 버퍼 포인터
    volatile bool tx_busy;  // BareMetal용: Tx 상태 플래그

    /* Circular DMA용 소프트웨어 Tail 포인터 추가 */
    /* HAL_UARTEx_RxEventCallback의 Size 파라미터와 비교하여 처리 */
    volatile uint16_t rx_old_pos;
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

static IOIF_UART_Instance_t s_uart_instances[IOIF_UART_MAX_INSTANCES] = {0};
static uint32_t s_uart_instance_count = 0;

/* [디버깅용] IDLE 콜백 호출 횟수 */
static volatile uint32_t s_idle_callback_count[IOIF_UART_MAX_INSTANCES] = {0};
static volatile uint32_t s_error_callback_count[IOIF_UART_MAX_INSTANCES] = {0};

/* [디버깅용] DMA 초기화 실패 원인 추적 */
volatile uint32_t g_dma_init_fail_reason[IOIF_UART_MAX_INSTANCES] = {0};

/**
 * DMA Buffers (MPU로 Non-Cacheable 영역에 배치)
 * 
 * [STM32H7 (RTOS)]
 * - MPU 설정으로 IOIF_DMA_SECTION (.RAM_D3_data 사용 모듈에 따라 정의 영역이 다를 수 있음)이 non-cacheable
 * - 캐시 무효화 불필요
 * 
 * [STM32G4 (BareMetal)]
 * - D-Cache 없음 → 캐시 문제 없음
 * 
 * [확장성] BareMetal에서도 TX DMA 사용 가능하도록 버퍼 확보
 */
#if defined(USE_FREERTOS_DMA)
#if defined(STM32H743xx) || defined(STM32H750xx)
    /* RTOS: TX/RX 모두 DMA 버퍼 사용 (MPU 설정) */
    __attribute__((section(IOIF_DMA_SECTION), aligned(32)))
    static uint8_t s_uart_tx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_TX_DMA_BUFFER_SIZE];

    __attribute__((section(IOIF_DMA_SECTION), aligned(32)))
    static uint8_t s_uart_rx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_RX_DMA_BUFFER_SIZE];
#endif
#else
    /* BareMetal: TX/RX 모두 DMA 버퍼 사용 가능 */
#if defined(STM32H743xx) || defined(STM32H750xx)
    /* TX/RX 모두 DMA 버퍼 사용 (D-cache가 있으므로 MPU 설정으로 non-cacheable 영역 설정시 사용) */
    __attribute__((section(IOIF_DMA_SECTION), aligned(32)))
    static uint8_t s_uart_tx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_TX_DMA_BUFFER_SIZE];

    __attribute__((section(IOIF_DMA_SECTION), aligned(32)))
    static uint8_t s_uart_rx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_RX_DMA_BUFFER_SIZE];
#elif defined(STM32G474xx) || defined(STM32G431xx)
    // D-cache가 없음.
    static uint8_t s_uart_tx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_TX_DMA_BUFFER_SIZE];
    static uint8_t s_uart_rx_dma_buffer[IOIF_UART_MAX_INSTANCES][IOIF_UART_RX_DMA_BUFFER_SIZE];
#endif
#endif

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart);
static AGRBStatusDef _convert_hal_status(HAL_StatusTypeDef hal_status);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

AGRBStatusDef IOIF_UART_AssignInstance(IOIF_UARTx_t* id, UART_HandleTypeDef* huart, IOIF_UART_Config_t* config)
{
    if (id == NULL || huart == NULL || config == NULL) return AGRBStatus_PARAM_ERROR;

    *id = IOIF_UART_ID_NOT_ALLOCATED;

    if (s_uart_instance_count >= IOIF_UART_MAX_INSTANCES) {
        return AGRBStatus_INITIAL_FAILED; 
    }

    // 중복 할당 검사
    for (uint32_t i = 0; i < s_uart_instance_count; i++) {
        if (s_uart_instances[i].huart == huart && s_uart_instances[i].allocated) {
            return AGRBStatus_BUSY; 
        }
    }

    // 인스턴스 할당
    IOIF_UART_Instance_t* instance = &s_uart_instances[s_uart_instance_count];
    memset(instance, 0, sizeof(IOIF_UART_Instance_t));

    instance->huart = huart;
    memcpy(&(instance->config), config, sizeof(IOIF_UART_Config_t));
    instance->id = s_uart_instance_count;

    /* Circular DMA Buffer 포인터 초기화 */
    instance->rx_old_pos = 0;

#if defined(USE_FREERTOS_DMA)
    instance->rx_dma_ptr = s_uart_rx_dma_buffer[instance->id];
    
    // TX 세마포어 생성 (공통)
    instance->tx_semaphore = xSemaphoreCreateBinary();
    if (instance->tx_semaphore == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    xSemaphoreGive(instance->tx_semaphore); // 초기 상태: 사용 가능

    // [수정] config.rxMode에 따라 분기
    if (config->rxMode == IOIF_UART_MODE_POLLING_TASK) {
#if 0 // polling mode 레거시
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
#endif
    } else {
        // IDLE_EVENT 모드: 태스크 생성 안 함
        instance->rx_task_handle = NULL;
    }
#else // USE_BAREMETAL
    /* BareMetal: rx_dma_ptr 설정 (CRITICAL) */
    instance->rx_dma_ptr = s_uart_rx_dma_buffer[instance->id];
    instance->tx_busy = false;
#endif

    instance->allocated = true;
    *id = s_uart_instance_count++;

    // 즉시 수신 시작
    return IOIF_UART_Start(*id);
}

AGRBStatusDef IOIF_UART_Start(IOIF_UARTx_t id)
{
#if defined(USE_FREERTOS_DMA)
    if (id >= s_uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
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
        // 2. Event 방식: Idle DMA는 콜백 등록 시 시작됨 (ioif_uart_update_rx_callback)
        /* IDLE_EVENT 모드는 콜백이 등록될 때까지 대기 */
    }
    return AGRBStatus_OK;
#else // USE_BAREMETAL
    /* BareMetal: IDLE_EVENT 모드는 콜백 등록 시 시작 */
    if (id >= s_uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;
    
    /* [CRITICAL] UART 완전 리셋 (이전 상태 클리어) */
    /* 1. 진행 중인 UART/DMA 동작 중단 */
    HAL_UART_Abort(instance->huart);
    
    /* 2. UART 상태 강제 초기화 */
    instance->huart->RxState = HAL_UART_STATE_READY;
    instance->huart->ErrorCode = HAL_UART_ERROR_NONE;
    
    /* 3. 에러 플래그 클리어 */
    __HAL_UART_CLEAR_OREFLAG(instance->huart);
    __HAL_UART_CLEAR_NEFLAG(instance->huart);
    __HAL_UART_CLEAR_FEFLAG(instance->huart);
    __HAL_UART_CLEAR_PEFLAG(instance->huart);
    
    /* 4. DMA 상태 리셋 (필요 시) */
    if (instance->huart->hdmarx != NULL) {
        instance->huart->hdmarx->State = HAL_DMA_STATE_READY;
    }
    
    /* [수정] IDLE_EVENT 모드는 콜백 등록 시 시작 (ioif_uart_update_rx_callback에서) */
    if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        /* 콜백이 등록될 때까지 대기 */
        return AGRBStatus_OK;
    }
    
    /* POLLING 모드는 여기서 시작 (현재 BareMetal에서는 미지원이지만 향후 대비) */
    if (HAL_UART_Receive_DMA(instance->huart, instance->rx_dma_ptr, IOIF_UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    return AGRBStatus_OK;
#endif
}

/**
 * @brief UART 데이터 전송 (Polling 모드, Blocking)
 * @param id UART Instance ID
 * @param tx_buf 전송할 데이터 버퍼
 * @param size 전송할 데이터 크기 (bytes)
 * @return AGRBStatus_OK=성공, AGRBStatus_TIMEOUT=타임아웃, AGRBStatus_BUSY=사용 중
 * 
 * @note 
 * - RTOS: Semaphore로 동기화 (Blocking, Timeout 5초)
 * - BareMetal: Busy 체크 후 즉시 전송
 * - Polling 방식이므로 전송 완료까지 대기
 */
AGRBStatusDef IOIF_UART_Write_Polling(IOIF_UARTx_t id, const uint8_t* tx_buf, uint32_t size)
{
    if (id >= s_uart_instance_count || tx_buf == NULL) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

#if defined(USE_FREERTOS_DMA)
    /* RTOS: Semaphore로 이전 전송 완료 대기 */
    if (xSemaphoreTake(instance->tx_semaphore, pdMS_TO_TICKS(UART_TX_SEMAPHORE_TIMEOUT_MS)) != pdTRUE) {
        return AGRBStatus_TIMEOUT;
    }

    /* Polling 전송 (Blocking) */
    HAL_StatusTypeDef status = HAL_UART_Transmit(instance->huart, (uint8_t*)tx_buf, size, UART_TX_POLLING_TIMEOUT_MS);
    
    xSemaphoreGive(instance->tx_semaphore);  /* 전송 완료, Semaphore 반환 */
    
    return (status == HAL_OK) ? AGRBStatus_OK : AGRBStatus_ERROR;
#else
    /* BareMetal: Busy Check 후 Polling 전송 */
    if (instance->huart->gState != HAL_UART_STATE_READY) {
        return AGRBStatus_BUSY;
    }
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(instance->huart, (uint8_t*)tx_buf, size, 1000);
    return (status == HAL_OK) ? AGRBStatus_OK : AGRBStatus_ERROR;
#endif
}

/**
 * @brief UART 데이터 전송 (DMA 모드, Non-Blocking)
 * @param id UART Instance ID
 * @param tx_buf 전송할 데이터 버퍼
 * @param size 전송할 데이터 크기 (bytes, 최대 256)
 * @return AGRBStatus_OK=성공, AGRBStatus_TIMEOUT=타임아웃, AGRBStatus_BUSY=사용 중
 * 
 * @note 
 * - RTOS/BareMetal 공통 사용 가능
 * - DMA 사용으로 Non-Blocking (CPU 부하 최소)
 * - 내부 DMA 버퍼로 복사 후 전송
 * 
 * @details
 * [RTOS]
 * - Semaphore로 동기화
 * - 전송 완료는 HAL_UART_TxCpltCallback에서 Semaphore Give
 * 
 * [BareMetal]
 * - Volatile flag로 상태 관리
 * - 전송 완료는 HAL_UART_TxCpltCallback에서 flag 클리어
 */
AGRBStatusDef IOIF_UART_Write_DMA(IOIF_UARTx_t id, const uint8_t* tx_buf, uint32_t size)
{
    if (id >= s_uart_instance_count || tx_buf == NULL) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;
    
    if (size > IOIF_UART_TX_DMA_BUFFER_SIZE) return AGRBStatus_BUFFER_OVERFLOW;

#if defined(USE_FREERTOS_DMA)
    /* [RTOS] Semaphore로 동기화 */
    if (xSemaphoreTake(instance->tx_semaphore, pdMS_TO_TICKS(UART_TX_SEMAPHORE_TIMEOUT_MS)) != pdTRUE) {
        return AGRBStatus_TIMEOUT;
    }
#else
    /* [BareMetal] Busy Flag 체크 */
    if (instance->tx_busy) {
        return AGRBStatus_BUSY;
    }
    instance->tx_busy = true;  /* DMA 전송 시작 표시 */
#endif

    /* DMA 버퍼로 복사 (MPU로 non-cacheable 설정됨, 캐시 무효화 불필요) */
    memcpy(s_uart_tx_dma_buffer[id], tx_buf, size);

    /* DMA 전송 시작 (Non-Blocking) */
    if (HAL_UART_Transmit_DMA(instance->huart, s_uart_tx_dma_buffer[id], size) != HAL_OK) {
#if defined(USE_FREERTOS_DMA)
        xSemaphoreGive(instance->tx_semaphore);
#else
        instance->tx_busy = false;
#endif
        return AGRBStatus_ERROR;
    }
    
    /* 성공: ISR에서 Semaphore Give 또는 flag 클리어 */
    return AGRBStatus_OK;
}

/**
 * @brief RX IDLE Event 콜백 등록 및 수신 시작
 * @param id UART Instance ID
 * @param callback RX IDLE Event 발생 시 호출될 콜백 함수
 * @return AGRBStatus_OK=성공, AGRBStatus_ERROR=DMA 시작 실패
 * 
 * @note 
 * - IDLE Event 모드 전용 함수
 * - Circular DMA 모드로 수신 시작 (IDLE/HT/TC 이벤트 처리)
 * - RTOS/BareMetal 공통 사용 가능
 * 
 * @details
 * [Circular DMA + IDLE Event 동작]
 * 1. IDLE 감지: 프레임 수신 완료 (가장 일반적)
 * 2. HT (Half Transfer): 버퍼 절반 도달
 * 3. TC (Transfer Complete): 버퍼 끝 도달
 * → 모든 경우에 HAL_UARTEx_RxEventCallback 호출
 * → 소프트웨어 링버퍼로 데이터 유실 방지
 */
AGRBStatusDef IOIF_UART_SetRxIdleCallback(IOIF_UARTx_t id, IOIF_UART_RxEventCallback_t callback)
{
    if (id >= s_uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;

    /* 1. 콜백 등록 */
    instance->config.rx_event_callback = callback;

    /* 2. IDLE Event 모드인 경우에만 DMA 수신 시작 */
    if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        /* UART가 READY 상태인 경우에만 시작 (BUSY면 이미 동작 중) */
        if (instance->huart->RxState == HAL_UART_STATE_READY) {
            /* 링버퍼 포인터 초기화 */
            instance->rx_old_pos = 0;
            
            /* 에러 플래그 클리어 */
            __HAL_UART_CLEAR_OREFLAG(instance->huart);
            __HAL_UART_CLEAR_NEFLAG(instance->huart);
            __HAL_UART_CLEAR_FEFLAG(instance->huart);
            __HAL_UART_CLEAR_PEFLAG(instance->huart);

            /* Circular DMA 모드로 수신 시작 */
            /* 
             * [중요] CubeMX에서 DMA Mode를 "Circular"로 설정 필수!
             * Circular 모드: TC/HT/IDLE 이벤트마다 콜백 호출, DMA는 멈추지 않음
             * Normal 모드: 1회 전송 후 멈춤 (사용 금지)
             */
            if (HAL_UARTEx_ReceiveToIdle_DMA(instance->huart,
                                    instance->rx_dma_ptr,
                                    IOIF_UART_RX_DMA_BUFFER_SIZE) != HAL_OK) {
                return AGRBStatus_ERROR;
            }
        }
        /* else: 이미 BUSY (DMA 동작 중) → 콜백만 교체, 성공 */
    }

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_UART_Flush(IOIF_UARTx_t id)
{
#if defined(USE_FREERTOS_DMA)
    if (id >= s_uart_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    instance->ringbuffer.tail = IOIF_UART_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
    return AGRBStatus_OK;
#else
    return AGRBStatus_NOT_SUPPORTED;
#endif
}

AGRBStatusDef IOIF_UART_GetDebugInfo(IOIF_UARTx_t id, IOIF_UART_DebugInfo_t* info)
{
    if (id >= s_uart_instance_count || info == NULL) return AGRBStatus_PARAM_ERROR;
    IOIF_UART_Instance_t* instance = &s_uart_instances[id];
    if (!instance->allocated) return AGRBStatus_NOT_INITIALIZED;
    
    UART_HandleTypeDef* huart = instance->huart;
    
    // UART 상태
    info->rx_state = huart->RxState;
    info->error_code = huart->ErrorCode;
    info->isr = huart->Instance->ISR;
    
    // DMA 상태
    if (huart->hdmarx != NULL) {
        info->dma_cndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        
        /* DMA Configuration Register (MCU별 레지스터 구조 다름) */
        #if defined(STM32H743xx) || defined(STM32H750xx)
            /* STM32H7: DMA Stream uses CR register */
            info->dma_ccr = ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR;
        #elif defined(STM32G474xx) || defined(STM32G431xx)
            /* STM32G4: DMA Channel uses CCR register */
            info->dma_ccr = ((DMA_Channel_TypeDef *)huart->hdmarx->Instance)->CCR;
        #else
            info->dma_ccr = 0;  /* Unsupported MCU */
        #endif
    } else {
        info->dma_cndtr = 0;
        info->dma_ccr = 0;
    }
    
    // 인터럽트 활성화 상태
    info->idle_ie_enabled = (huart->Instance->CR1 & USART_CR1_IDLEIE) ? true : false;
    info->dmar_enabled = (huart->Instance->CR3 & USART_CR3_DMAR) ? true : false;
    
    // 콜백 카운터
    info->idle_callback_cnt = s_idle_callback_count[id];
    info->error_callback_cnt = s_error_callback_count[id];
    
    return AGRBStatus_OK;
}

/**
 *------------------------------------------------------------
 * [Phase 1 & 2] 센서 리셋 지원 함수
 *------------------------------------------------------------
 */

/**
 * @brief [범용] UART 상태를 완전히 리셋
 */
 AGRBStatusDef IOIF_UART_ResetState(UART_HandleTypeDef* huart)
 {
     if (huart == NULL) return AGRBStatus_PARAM_ERROR;
     
     /* 1. [CRITICAL] UART 완전 리셋 (이전 상태 클리어) */
     HAL_UART_Abort(huart);
     
     /* 2. UART 상태 강제 초기화 */
     huart->RxState = HAL_UART_STATE_READY;
     huart->ErrorCode = HAL_UART_ERROR_NONE;
     
     /* 3. 에러 플래그 클리어 */
     __HAL_UART_CLEAR_OREFLAG(huart);
     __HAL_UART_CLEAR_NEFLAG(huart);
     __HAL_UART_CLEAR_FEFLAG(huart);
     __HAL_UART_CLEAR_PEFLAG(huart);
     
     /* 4. [CRITICAL] DMA on RX Error 설정 변경 */
     /* 에러 발생 시에도 DMA가 계속 동작하도록 설정 */
     huart->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_DMADISABLEONERROR_INIT;
     huart->AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_ENABLE;
     huart->AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_ENABLEONRXERROR;
     HAL_UART_Init(huart); // 설정 적용
     
     /* 5. DMA 상태 리셋 */
     if (huart->hdmarx != NULL) {
         huart->hdmarx->State = HAL_DMA_STATE_READY;
     }
     
     return AGRBStatus_OK;
 }

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static inline IOIF_UART_Instance_t* _get_instance(UART_HandleTypeDef* huart)
{
    for (uint32_t i = 0; i < s_uart_instance_count; i++) {
        if (s_uart_instances[i].huart == huart) return &s_uart_instances[i];
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
 * @note RTOS/BareMetal 공통 사용 (DMA TX 전송 완료 시 호출)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

#if defined(USE_FREERTOS_DMA)
    /* RTOS: Semaphore Give */
    if (instance->tx_semaphore) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#else
    /* BareMetal: Busy Flag 클리어 */
    instance->tx_busy = false;
#endif
}

/**
 * @brief UART IDLE/TC/HT Event 콜백 (DMA Circular Mode + Rx Idle Event 모드 전용)
 * @note  이 함수는 IDLE 라인 감지 시 뿐만 아니라, 
 * 버퍼가 절반 찼을 때(HT), 가득 찼을 때(TC)도 호출됩니다.
 * 따라서 데이터 유실 없이 연속 수신이 가능합니다.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    IOIF_UART_Instance_t* instance = _get_instance(huart);
    if (instance == NULL) return;

    /* [디버깅] 콜백 카운터 증가 */
    s_idle_callback_count[instance->id]++;

    // 이 콜백은 IDLE_EVENT 모드인 인스턴스만 처리
    if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        /**
         * [Circular Buffer Logic]
         * Size: DMA가 버퍼의 처음(0)부터 현재까지 쓴 데이터의 위치 (Current Head)
         * rx_old_pos: 소프트웨어가 마지막으로 처리한 위치 (Tail)
         */
        uint16_t head = Size;
        uint16_t tail = instance->rx_old_pos;
        
        /* 데이터가 들어온 경우에만 처리 */
        if (head != tail) {
            
            // Case 1: Wrap-around 발생 (Tail이 Head보다 뒤에 있음)
            // [ ... Head ... Tail ... ]
            if (head < tail) {
                // 1. Tail -> 버퍼 끝까지 처리
                uint16_t len_first = IOIF_UART_RX_DMA_BUFFER_SIZE - tail;
                if (instance->config.rx_event_callback != NULL) {
                    instance->config.rx_event_callback(&instance->rx_dma_ptr[tail], len_first, instance->id);
                }
                
                // 2. 버퍼 처음(0) -> Head까지 처리
                if (head > 0) {
                    if (instance->config.rx_event_callback != NULL) {
                        instance->config.rx_event_callback(&instance->rx_dma_ptr[0], head, instance->id);
                    }
                }
            }
            // Case 2: 선형적인 데이터 수신
            // [ ... Tail ... Head ... ]
            else {
                uint16_t len = head - tail;
                if (instance->config.rx_event_callback != NULL) {
                    instance->config.rx_event_callback(&instance->rx_dma_ptr[tail], len, instance->id);
                }
            }
            
            // Tail 업데이트 (현재 Head 위치 기억)
            instance->rx_old_pos = head;
        }

        /* * [CRITICAL CHANGE]
         * HAL_UARTEx_ReceiveToIdle_DMA() 재호출 코드 삭제!
         * Circular 모드이므로 DMA는 멈추지 않고 계속 돕니다.
         * 재호출하면 인덱스가 0으로 초기화되어 데이터가 꼬이게 됩니다.
         */

        // if (Size > 0) {
        //     /* 1. DMA 버퍼는 MPU로 non-cacheable 설정됨(H7) - 캐시 무효화 불필요, STM32G4는 DCache 없음 */
        //     // 2. System Layer로 데이터 전달
        //     if (instance->config.rx_event_callback != NULL) {
        //         instance->config.rx_event_callback(instance->rx_dma_ptr, Size, instance->id);
        //     }
        // }
        
        // // 3. [필수] 다음 Idle 이벤트를 받기 위해 수신을 즉시 재시작
        // // [수정] RxState 체크를 제거하고 무조건 재시작 (HAL 내부에서 체크함)
        // HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, instance->rx_dma_ptr, IOIF_UART_RX_DMA_BUFFER_SIZE);
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
        /* [디버깅] 에러 콜백 카운터 증가 */
        s_error_callback_count[instance->id]++;
        // 1. [필수] 모든 에러 플래그 클리어 (이걸 안 하면 통신 복구 안됨)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 2. 수신 재시작 (모드에 따라 분기)
        /* DMA 상태 확인 및 재가동 */
        // Circular 모드에서도 심각한 에러 발생 시 DMA가 멈출 수 있음 (Ready 상태)
        // 이 경우에만 다시 살려줌
        if (instance->config.rxMode == IOIF_UART_MODE_IDLE_EVENT)
        {
            if (huart->RxState == HAL_UART_STATE_READY) {
                // IDLE Event 모드 재시작
                instance->rx_old_pos = 0; // 포인터 리셋
                HAL_UARTEx_ReceiveToIdle_DMA(instance->huart, 
                                            instance->rx_dma_ptr, 
                                            IOIF_UART_RX_DMA_BUFFER_SIZE);
            }
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

#if 0  /* ===== LEGACY CODE: RX POLLING TASK ===== */
#define IOIF_UART_RX_BOUNCE_BUFFER_SIZE     (512)   /**< Polling Task 임시 버퍼 (레거시) */

/**
 * @deprecated 레거시 코드: Polling 모드 RX Task
 * @note 현재는 RX IDLE Event 방식 사용 중 (Circular DMA + IDLE Event)
 *       향후 필요 시 복구 가능하도록 보존
 */
#if defined(USE_FREERTOS_DMA)
static void _uart_rx_task(void* argument);

/**
 * @deprecated 레거시 코드: Polling 모드 RX Task
 * @brief UART 수신 처리 태스크 (High-speed polling)
 * @note 현재는 RX IDLE Event 방식 사용 중 (Circular DMA + IDLE Event)
 *       향후 필요 시 복구 가능하도록 보존
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
                /* DCache Invalidate - STM32H7만 해당 MPU 설정으로 대체 */
                // #if defined(STM32H7)
                // SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[tail], chunk_size);
                // #endif
                memcpy(bounce_buffer, &dma_base[tail], chunk_size);
                tail += chunk_size;
            } else {
                // 끝부분 + 앞부분
                uint32_t first_part = IOIF_UART_RX_DMA_BUFFER_SIZE - tail;
                uint32_t second_part = chunk_size - first_part;
                
                /* DCache Invalidate - STM32H7만 해당 MPU 설정으로 대체 */
                // #if defined(STM32H7)
                // SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[tail], first_part);
                // SCB_InvalidateDCache_by_Addr((uint32_t*)&dma_base[0], second_part);
                // #endif
                
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
#endif  /* USE_FREERTOS_DMA */
#endif  /* ===== END OF LEGACY CODE ===== */

#endif /* AGRB_IOIF_UART_ENABLE */
