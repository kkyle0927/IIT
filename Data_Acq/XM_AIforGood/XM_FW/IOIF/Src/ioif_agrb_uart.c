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

#if defined(USE_FREERTOS)
#include "cmsis_os2.h" // RTOS
#include "stream_buffer.h"
#include "task.h"
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/* 공통: TX/RX DMA 버퍼 크기 */
#define IOIF_UART_TX_DMA_BUFFER_SIZE        (128)   /**< TX DMA 버퍼 (RTOS/BareMetal 공통) */
#define IOIF_UART_RX_DMA_BUFFER_SIZE        (256)   /**< RX DMA 버퍼 (RTOS/BareMetal 공통) */

#if defined(USE_FREERTOS)
    /** @brief StreamBuffer 크기 = DMA 버퍼의 2배 (ISR burst 수용) */
    #define IOIF_UART_STREAM_BUFFER_SIZE    (IOIF_UART_RX_DMA_BUFFER_SIZE * 2)
    /** @brief RxTask 내부 임시 버퍼 크기 */
    #define IOIF_UART_RX_TASK_TEMP_BUF_SIZE (128U)
#endif

#if defined(USE_FREERTOS)
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

#if defined(USE_FREERTOS)
    osThreadId_t rx_task_handle; // (POLLING 모드 전용)
    
    struct {
        uint32_t tail; // 링버퍼 읽기 위치
    } ringbuffer; // (POLLING 모드 전용)

    SemaphoreHandle_t tx_semaphore; 
    uint8_t* rx_dma_ptr; // 할당된 DMA 버퍼 포인터

    /* Circular DMA용 소프트웨어 Tail 포인터 추가 */
    /* HAL_UARTEx_RxEventCallback의 Size 파라미터와 비교하여 처리 */
    volatile uint16_t rx_old_pos;

    /**
     * @brief [ISR→Task] StreamBuffer (IDLE_EVENT 모드 전용)
     * @details ISR에서 raw bytes를 전송, 공유 RxTask에서 수신하여 콜백 호출
     */
    StreamBufferHandle_t rx_stream_buffer;

#else // USE_BAREMETAL
    uint8_t* rx_dma_ptr;    // 할당된 DMA 버퍼 포인터
    volatile bool tx_busy;  // BareMetal용: Tx 상태 플래그 (atomic 접근)

    /* Circular DMA용 소프트웨어 Tail 포인터 추가 */
    /* HAL_UARTEx_RxEventCallback의 Size 파라미터와 비교하여 처리 */
    volatile uint16_t rx_old_pos;
#endif
} IOIF_UART_Instance_t;

/**
 * @brief Manual-init 전용 HAL 핸들 저장소 (ADC 의 s_manual_init_info 패턴).
 * @details `IOIF_UART_InitManual` 이 stack 로컬 HAL 핸들을 쓰면 함수 종료 후
 *          dangling pointer 가 되므로, 인스턴스별로 static storage 를 보유한다.
 *          AssignInstance 경로는 외부 HAL 핸들 포인터를 받으므로 이 구조체를 사용하지 않는다.
 */
typedef struct {
    bool               is_manual_init;
    UART_HandleTypeDef huart_storage;
    DMA_HandleTypeDef  hdma_rx_storage;
    DMA_HandleTypeDef  hdma_tx_storage;
} IOIF_UART_ManualInitInfo_t;

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

/* Manual-init 경로 전용 HAL 핸들 storage (parallel array). AssignInstance 경로는 미사용. */
static IOIF_UART_ManualInitInfo_t s_uart_manual_info[IOIF_UART_MAX_INSTANCES] = {0};

#if defined(USE_FREERTOS)
/**
 * @brief 단일 공유 UART RxTask 핸들
 * @details 모든 UART 인스턴스의 StreamBuffer 데이터를 처리하는 하나의 Task.
 *          TaskNotify 비트마스크(bit N = instance N)로 어떤 인스턴스에 데이터가 있는지 식별.
 */
static TaskHandle_t s_uart_shared_rx_task = NULL;
static void _IOIF_UART_SharedRxTask(void* argument);
#endif

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
#if defined(USE_FREERTOS)
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

#if defined(USE_FREERTOS)
    instance->rx_dma_ptr = s_uart_rx_dma_buffer[instance->id];
    
    // TX 세마포어 생성 (공통)
    instance->tx_semaphore = xSemaphoreCreateBinary();
    if (instance->tx_semaphore == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    xSemaphoreGive(instance->tx_semaphore); // 초기 상태: 사용 가능

    // [수정] config.rxMode에 따라 분기
    if (config->rxMode == IOIF_UART_MODE_IDLE_EVENT) {
        /**
         * [IDLE_EVENT 모드] StreamBuffer 생성
         * - ISR → StreamBuffer → 공유 RxTask → 콜백
         * - ISR에서의 동작을 최소화하여 latency 감소
         */
        instance->rx_stream_buffer = xStreamBufferCreate(
            IOIF_UART_STREAM_BUFFER_SIZE,
            IOIF_UART_STREAM_BUFFER_TRIGGER
        );
        if (instance->rx_stream_buffer == NULL) {
            vSemaphoreDelete(instance->tx_semaphore);
            return AGRBStatus_ERROR;
        }

        /**
         * [공유 RxTask] 최초 1회만 생성
         * - 모든 UART IDLE_EVENT 인스턴스가 하나의 Task를 공유
         * - TaskNotify 비트마스크로 인스턴스 식별 (bit N = instance N)
         */
        if (s_uart_shared_rx_task == NULL) {
            BaseType_t ret = xTaskCreate(
                _IOIF_UART_SharedRxTask,
                "IOIF_UartRx",
                IOIF_UART_RX_TASK_STACK_SIZE / sizeof(StackType_t),
                NULL,
                IOIF_UART_RX_TASK_PRIORITY,
                &s_uart_shared_rx_task
            );
            if (ret != pdPASS) {
                vStreamBufferDelete(instance->rx_stream_buffer);
                instance->rx_stream_buffer = NULL;
                vSemaphoreDelete(instance->tx_semaphore);
                return AGRBStatus_INITIAL_FAILED;
            }
        }
        instance->rx_task_handle = NULL; /* 개별 task 없음 */
    } else {
        // POLLING_TASK 모드: 레거시 (현재 미사용)
        instance->rx_stream_buffer = NULL;
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
#if defined(USE_FREERTOS)
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
 * @brief UART baud rate를 런타임에 변경합니다.
 * @param id        UART Instance ID
 * @param baudrate  새 baud rate enum 값
 * @return AGRBStatus_OK=성공, AGRBStatus_ERROR=실패
 *
 * @details
 * 1. 진행 중인 DMA RX/TX Abort
 * 2. HAL UART baud rate 변경 + Re-Init
 * 3. DMA RX 재시작 (기존 RxMode 유지)
 */
AGRBStatusDef IOIF_UART_SetBaudrate(IOIF_UARTx_t id, IOIF_UART_Baudrate_e baudrate)
{
    if (id >= s_uart_instance_count) return AGRBStatus_PARAM_ERROR;

    static const uint32_t baud_map[] = {
        [IOIF_UART_Baudrate_9600]   = 9600,
        [IOIF_UART_Baudrate_19200]  = 19200,
        [IOIF_UART_Baudrate_38400]  = 38400,
        [IOIF_UART_Baudrate_57600]  = 57600,
        [IOIF_UART_Baudrate_115200] = 115200,
        [IOIF_UART_Baudrate_230400] = 230400,
        [IOIF_UART_Baudrate_460800] = 460800,
        [IOIF_UART_Baudrate_921600] = 921600,
    };

    if (baudrate >= sizeof(baud_map) / sizeof(baud_map[0])) return AGRBStatus_PARAM_ERROR;

    IOIF_UART_Instance_t* instance = &s_uart_instances[id];

    /* 1. Abort ongoing DMA */
    HAL_UART_Abort(instance->huart);

    /* 2. Change baud rate + Re-Init */
    instance->huart->Init.BaudRate = baud_map[baudrate];
    if (HAL_UART_Init(instance->huart) != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    /* 3. Update stored config */
    instance->config.baudrate = baudrate;

    /* 4. Clear error flags */
    __HAL_UART_CLEAR_OREFLAG(instance->huart);
    __HAL_UART_CLEAR_NEFLAG(instance->huart);
    __HAL_UART_CLEAR_FEFLAG(instance->huart);
    __HAL_UART_CLEAR_PEFLAG(instance->huart);

    /* 5. Restart DMA RX */
    instance->rx_old_pos = 0;
    return IOIF_UART_Start(id);
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

#if defined(USE_FREERTOS)
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

#if defined(USE_FREERTOS)
    /* [RTOS] Semaphore로 동기화 */
    if (xSemaphoreTake(instance->tx_semaphore, pdMS_TO_TICKS(UART_TX_SEMAPHORE_TIMEOUT_MS)) != pdTRUE) {
        return AGRBStatus_TIMEOUT;
    }
#else
    /* [BareMetal] Atomic check-then-set (LDREX/STREX on Cortex-M) */
    if (__atomic_exchange_n(&instance->tx_busy, true, __ATOMIC_SEQ_CST)) {
        return AGRBStatus_BUSY;  /* 이미 busy → 이전 값 true 반환 */
    }
#endif

    /* DMA 버퍼로 복사 (MPU로 non-cacheable 설정됨, 캐시 무효화 불필요) */
    memcpy(s_uart_tx_dma_buffer[id], tx_buf, size);

    /* DMA 전송 시작 (Non-Blocking) */
    if (HAL_UART_Transmit_DMA(instance->huart, s_uart_tx_dma_buffer[id], size) != HAL_OK) {
#if defined(USE_FREERTOS)
        xSemaphoreGive(instance->tx_semaphore);
#else
        __atomic_store_n(&instance->tx_busy, false, __ATOMIC_RELEASE);
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

    /* 1. IDLE Event 모드인 경우에만 DMA 수신 시작 */
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

    /* 2. DMA 시작 성공 후 콜백 등록 (DMA 실패 시 이전 콜백 유지) */
    instance->config.rx_event_callback = callback;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_UART_Flush(IOIF_UARTx_t id)
{
#if defined(USE_FREERTOS)
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
     HAL_StatusTypeDef hal_res = HAL_UART_Init(huart);
     if (hal_res != HAL_OK) {
         return IOIF_HAL_ERROR_TO_AGRB_STATUSDEF(hal_res);
     }

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

#if defined(USE_FREERTOS)
    /* RTOS: Semaphore Give */
    if (instance->tx_semaphore) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(instance->tx_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
#else
    /* BareMetal: Atomic Busy Flag 클리어 */
    __atomic_store_n(&instance->tx_busy, false, __ATOMIC_RELEASE);
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
    if (instance->config.rxMode != IOIF_UART_MODE_IDLE_EVENT) return;

    /**
     * [Circular Buffer Logic]
     * Size: DMA가 버퍼의 처음(0)부터 현재까지 쓴 데이터의 위치 (Current Head)
     * rx_old_pos: 소프트웨어가 마지막으로 처리한 위치 (Tail)
     */
    uint16_t head = Size;
    uint16_t tail = instance->rx_old_pos;
    
    /* 데이터가 들어온 경우에만 처리 */
    if (head == tail) return;

#if defined(USE_FREERTOS)
    /**
     * [RTOS] ISR 최소화 전략:
     * - ISR에서는 raw bytes를 StreamBuffer로 전송만 수행
     * - 파싱/콜백은 공유 RxTask에서 처리
     * - xTaskNotifyFromISR 비트마스크로 해당 인스턴스 식별
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (head < tail) {
        /* Case 1: Wrap-around - Tail→End, 0→Head 두 번에 걸쳐 전송 */
        uint16_t len_first = IOIF_UART_RX_DMA_BUFFER_SIZE - tail;
        xStreamBufferSendFromISR(instance->rx_stream_buffer,
                                  &instance->rx_dma_ptr[tail],
                                  len_first,
                                  &xHigherPriorityTaskWoken);
        if (head > 0) {
            xStreamBufferSendFromISR(instance->rx_stream_buffer,
                                      &instance->rx_dma_ptr[0],
                                      head,
                                      &xHigherPriorityTaskWoken);
        }
    } else {
        /* Case 2: 선형 데이터 - 한 번에 전송 */
        uint16_t len = head - tail;
        xStreamBufferSendFromISR(instance->rx_stream_buffer,
                                  &instance->rx_dma_ptr[tail],
                                  len,
                                  &xHigherPriorityTaskWoken);
    }

    /* 공유 RxTask에 알림 (비트마스크: bit N = instance N) */
    if (s_uart_shared_rx_task != NULL) {
        xTaskNotifyFromISR(s_uart_shared_rx_task,
                           (1U << instance->id),
                           eSetBits,
                           &xHigherPriorityTaskWoken);
    }

    /* Tail 업데이트 */
    instance->rx_old_pos = head;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

#else /* BareMetal: 기존 동작 유지 (ISR에서 직접 콜백) */
    if (head < tail) {
        /* Case 1: Wrap-around */
        uint16_t len_first = IOIF_UART_RX_DMA_BUFFER_SIZE - tail;
        if (instance->config.rx_event_callback != NULL) {
            instance->config.rx_event_callback(&instance->rx_dma_ptr[tail], len_first, instance->id);
        }
        if (head > 0 && instance->config.rx_event_callback != NULL) {
            instance->config.rx_event_callback(&instance->rx_dma_ptr[0], head, instance->id);
        }
    } else {
        /* Case 2: 선형 데이터 */
        uint16_t len = head - tail;
        if (instance->config.rx_event_callback != NULL) {
            instance->config.rx_event_callback(&instance->rx_dma_ptr[tail], len, instance->id);
        }
    }

    /* Tail 업데이트 */
    instance->rx_old_pos = head;
#endif /* USE_FREERTOS */

    /**
     * [CRITICAL] Circular DMA이므로 재호출하지 않음!
     * DMA는 멈추지 않고 계속 순환하며, 재호출 시 인덱스 초기화로 데이터 손실.
     */
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

#if defined(USE_FREERTOS)
/**
 * @brief [IOIF 내부] UART 공유 RxTask
 * @details
 * **ISR 최소화 원칙에 따른 핵심 구현부**
 * 
 * [동작]
 * 1. xTaskNotifyWait()로 ISR 알림 대기 (비트마스크)
 * 2. 알림 수신 시, 해당 인스턴스의 StreamBuffer에서 데이터 읽기
 * 3. 등록된 rx_event_callback 호출 (Task 컨텍스트)
 * 
 * [장점]
 * - ISR은 memcpy(StreamBuffer) + TaskNotify만 수행 → 최소 latency
 * - 파싱/DataLake 업데이트는 Task 컨텍스트에서 안전하게 수행
 * - 모든 UART IDLE_EVENT 인스턴스가 하나의 Task를 공유 → 자원 효율
 * 
 * [데이터 흐름]
 * DMA Ring Buffer → ISR(StreamBuffer Send) → RxTask(StreamBuffer Recv) → Callback → DataLake
 * 
 * @param argument 미사용 (NULL)
 */
static void _IOIF_UART_SharedRxTask(void* argument)
{
    (void)argument;
    uint8_t temp_buf[IOIF_UART_RX_TASK_TEMP_BUF_SIZE];

    for (;;) {
        /* 1. ISR 알림 대기 (비트마스크: bit N = instance N에 데이터 도착) */
        uint32_t notification_bits = 0;
        xTaskNotifyWait(0,                  /* ulBitsToClearOnEntry: 진입 시 클리어 안함 */
                        0xFFFFFFFF,         /* ulBitsToClearOnExit: 종료 시 모든 비트 클리어 */
                        &notification_bits,
                        portMAX_DELAY);     /* 무한 대기 */

        /* 2. 알림된 인스턴스별로 StreamBuffer 처리 */
        for (uint32_t i = 0; i < s_uart_instance_count; i++) {
            if (!(notification_bits & (1U << i))) continue;

            IOIF_UART_Instance_t* inst = &s_uart_instances[i];
            if (!inst->allocated) continue;
            if (inst->rx_stream_buffer == NULL) continue;

            /* 3. StreamBuffer에서 데이터를 모두 소진할 때까지 읽기 */
            size_t received;
            while ((received = xStreamBufferReceive(inst->rx_stream_buffer,
                                                     temp_buf,
                                                     sizeof(temp_buf),
                                                     0 /* 즉시 반환 */)) > 0)
            {
                /* 4. 등록된 콜백 호출 (Task 컨텍스트에서 파싱 수행) */
                if (inst->config.rx_event_callback != NULL) {
                    inst->config.rx_event_callback(temp_buf, (uint32_t)received, inst->id);
                }
            }
        }
    }
}
#endif /* USE_FREERTOS */

/**
 * ============================================================================
 * [신규] UART 런타임 동적 초기화 API 구현
 * ============================================================================
 */

/**
 * @brief [범용] UART를 런타임에 수동 초기화 (GPIO+DMA+NVIC 자동 설정)
 * @details
 * - System Layer의 `_Manual_UART4_MspInit()` 로직을 완전히 캡슐화
 * - GPIO, Clock, DMA, NVIC를 모두 IOIF가 처리
 */
AGRBStatusDef IOIF_UART_InitManual(
    USART_TypeDef* uart_instance,
    const IOIF_UART_ManualInitConfig_t* config,
    IOIF_UARTx_t* out_id
)
{
    if (uart_instance == NULL || config == NULL || out_id == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }
    if (s_uart_instance_count >= IOIF_UART_MAX_INSTANCES) {
        return AGRBStatus_NO_RESOURCE;
    }

    /* ========== [0] Instance slot + manual storage 확보 (ADC 의 s_manual_init_info 패턴) ========== */
    const IOIF_UARTx_t new_id = s_uart_instance_count;
    IOIF_UART_Instance_t*       instance    = &s_uart_instances[new_id];
    IOIF_UART_ManualInitInfo_t* manual_info = &s_uart_manual_info[new_id];
    memset(instance,    0, sizeof(*instance));
    memset(manual_info, 0, sizeof(*manual_info));
    manual_info->is_manual_init = true;

    /* ========== [1] Clock 활성화 ========== */
#if defined(IOIF_MCU_SERIES_H7)
    /* STM32H7: UART Clock 소스 설정 */
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (uart_instance == USART1) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    } else if (uart_instance == USART2 || uart_instance == USART3 || 
               uart_instance == UART4 || uart_instance == UART5 || 
               uart_instance == UART7 || uart_instance == UART8) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART234578;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    } else if (uart_instance == USART6) {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    }
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
#endif
    
    /* UART Peripheral Clock 활성화 */
    if (uart_instance == USART1) __HAL_RCC_USART1_CLK_ENABLE();
    else if (uart_instance == USART2) __HAL_RCC_USART2_CLK_ENABLE();
    else if (uart_instance == USART3) __HAL_RCC_USART3_CLK_ENABLE();
#if defined(UART4)
    else if (uart_instance == UART4) __HAL_RCC_UART4_CLK_ENABLE();
#endif
#if defined(UART5)
    else if (uart_instance == UART5) __HAL_RCC_UART5_CLK_ENABLE();
#endif
#if defined(USART6)
    else if (uart_instance == USART6) __HAL_RCC_USART6_CLK_ENABLE();
#endif
#if defined(UART7)
    else if (uart_instance == UART7) __HAL_RCC_UART7_CLK_ENABLE();
#endif
#if defined(UART8)
    else if (uart_instance == UART8) __HAL_RCC_UART8_CLK_ENABLE();
#endif
    
    /* GPIO Clock 활성화 */
    if (config->gpio_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (config->gpio_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (config->gpio_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (config->gpio_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (config->gpio_port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (config->gpio_port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (config->gpio_port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    
    /* DMA Clock 활성화 */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    /* ========== [2] GPIO 재설정 (ADC Analog → UART Alternate Function) ========== */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = config->tx_pin | config->rx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = config->alternate_function;
    HAL_GPIO_Init(config->gpio_port, &GPIO_InitStruct);
    
    /* ========== [3] DMA 설정 (RX) — storage 에 직접 작성 ========== */
#if defined(IOIF_MCU_SERIES_H7)
    if (config->enable_dma_rx && config->dma_rx_stream != NULL) {
        manual_info->hdma_rx_storage.Instance = config->dma_rx_stream;
#elif defined(IOIF_MCU_SERIES_G4)
    if (config->enable_dma_rx && config->dma_rx_channel != NULL) {
        manual_info->hdma_rx_storage.Instance = config->dma_rx_channel;
#endif
        manual_info->hdma_rx_storage.Init.Request = config->dma_rx_request;
        manual_info->hdma_rx_storage.Init.Direction = DMA_PERIPH_TO_MEMORY;
        manual_info->hdma_rx_storage.Init.PeriphInc = DMA_PINC_DISABLE;
        manual_info->hdma_rx_storage.Init.MemInc = DMA_MINC_ENABLE;
        manual_info->hdma_rx_storage.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        manual_info->hdma_rx_storage.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        manual_info->hdma_rx_storage.Init.Mode = config->dma_rx_circular ? DMA_CIRCULAR : DMA_NORMAL;
        manual_info->hdma_rx_storage.Init.Priority = DMA_PRIORITY_VERY_HIGH;

#if defined(IOIF_MCU_SERIES_H7)
        manual_info->hdma_rx_storage.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        manual_info->hdma_rx_storage.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
        manual_info->hdma_rx_storage.Init.MemBurst = DMA_MBURST_SINGLE;
        manual_info->hdma_rx_storage.Init.PeriphBurst = DMA_PBURST_SINGLE;
#endif

        if (HAL_DMA_Init(&manual_info->hdma_rx_storage) != HAL_OK) {
            return AGRBStatus_ERROR;
        }
    }

    /* ========== [4] DMA 설정 (TX, 선택사항) — storage 에 직접 작성 ========== */
#if defined(IOIF_MCU_SERIES_H7)
    if (config->enable_dma_tx && config->dma_tx_stream != NULL) {
        manual_info->hdma_tx_storage.Instance = config->dma_tx_stream;
#elif defined(IOIF_MCU_SERIES_G4)
    if (config->enable_dma_tx && config->dma_tx_channel != NULL) {
        manual_info->hdma_tx_storage.Instance = config->dma_tx_channel;
#endif
        manual_info->hdma_tx_storage.Init.Request = config->dma_tx_request;
        manual_info->hdma_tx_storage.Init.Direction = DMA_MEMORY_TO_PERIPH;
        manual_info->hdma_tx_storage.Init.PeriphInc = DMA_PINC_DISABLE;
        manual_info->hdma_tx_storage.Init.MemInc = DMA_MINC_ENABLE;
        manual_info->hdma_tx_storage.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        manual_info->hdma_tx_storage.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        manual_info->hdma_tx_storage.Init.Mode = DMA_NORMAL;
        manual_info->hdma_tx_storage.Init.Priority = DMA_PRIORITY_LOW;

#if defined(IOIF_MCU_SERIES_H7)
        manual_info->hdma_tx_storage.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        manual_info->hdma_tx_storage.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
        manual_info->hdma_tx_storage.Init.MemBurst = DMA_MBURST_SINGLE;
        manual_info->hdma_tx_storage.Init.PeriphBurst = DMA_PBURST_SINGLE;
#endif

        if (HAL_DMA_Init(&manual_info->hdma_tx_storage) != HAL_OK) {
            return AGRBStatus_ERROR;
        }
    }

    /* ========== [5] UART 핸들 초기화 — storage 에 직접 작성 ========== */
    manual_info->huart_storage.Instance = uart_instance;
    manual_info->huart_storage.Init.BaudRate = config->baudrate;
    manual_info->huart_storage.Init.WordLength = config->word_length;
    manual_info->huart_storage.Init.StopBits = config->stop_bits;
    manual_info->huart_storage.Init.Parity = config->parity;
    manual_info->huart_storage.Init.Mode = UART_MODE_TX_RX;
    manual_info->huart_storage.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    manual_info->huart_storage.Init.OverSampling = UART_OVERSAMPLING_16;

#if defined(IOIF_MCU_SERIES_H7) || defined(IOIF_MCU_SERIES_G4)
    manual_info->huart_storage.FifoMode = config->enable_fifo ? UART_FIFOMODE_ENABLE : UART_FIFOMODE_DISABLE;

    /* Advanced Features */
    manual_info->huart_storage.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT |
                                                              UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
    manual_info->huart_storage.AdvancedInit.DMADisableonRxError = config->dma_on_rx_error ?
                                                                   UART_ADVFEATURE_DMA_ENABLEONRXERROR :
                                                                   UART_ADVFEATURE_DMA_DISABLEONRXERROR;
    manual_info->huart_storage.AdvancedInit.OverrunDisable = config->overrun_disable ?
                                                              UART_ADVFEATURE_OVERRUN_DISABLE :
                                                              UART_ADVFEATURE_OVERRUN_ENABLE;
#endif

    /* DMA 연결 — storage 간 링크 (수명 = static, 안전) */
    if (config->enable_dma_rx) {
        __HAL_LINKDMA(&manual_info->huart_storage, hdmarx, manual_info->hdma_rx_storage);
    }
    if (config->enable_dma_tx) {
        __HAL_LINKDMA(&manual_info->huart_storage, hdmatx, manual_info->hdma_tx_storage);
    }

    if (HAL_UART_Init(&manual_info->huart_storage) != HAL_OK) {
        return AGRBStatus_ERROR;
    }

#if defined(IOIF_MCU_SERIES_H7) || defined(IOIF_MCU_SERIES_G4)
    /* FIFO Threshold 설정 */
    if (config->enable_fifo) {
        HAL_UARTEx_SetTxFifoThreshold(&manual_info->huart_storage, UART_TXFIFO_THRESHOLD_1_4);
        HAL_UARTEx_SetRxFifoThreshold(&manual_info->huart_storage, UART_RXFIFO_THRESHOLD_1_4);
        HAL_UARTEx_EnableFifoMode(&manual_info->huart_storage);
    }
#endif
    
    /* ========== [6] NVIC 설정 ========== */
    IRQn_Type uart_irq = USART1_IRQn;  /* 기본값 */
    if (uart_instance == USART1) uart_irq = USART1_IRQn;
    else if (uart_instance == USART2) uart_irq = USART2_IRQn;
    else if (uart_instance == USART3) uart_irq = USART3_IRQn;
#if defined(UART4_IRQn)
    else if (uart_instance == UART4) uart_irq = UART4_IRQn;
#endif
#if defined(UART5_IRQn)
    else if (uart_instance == UART5) uart_irq = UART5_IRQn;
#endif
#if defined(USART6_IRQn)
    else if (uart_instance == USART6) uart_irq = USART6_IRQn;
#endif
#if defined(UART7_IRQn)
    else if (uart_instance == UART7) uart_irq = UART7_IRQn;
#endif
#if defined(UART8_IRQn)
    else if (uart_instance == UART8) uart_irq = UART8_IRQn;
#endif
    
    HAL_NVIC_SetPriority(uart_irq, config->uart_irq_priority, 0);
    HAL_NVIC_EnableIRQ(uart_irq);
    
    /* DMA RX NVIC */
#if defined(IOIF_MCU_SERIES_H7)
    if (config->enable_dma_rx && config->dma_rx_stream != NULL) {
        IRQn_Type dma_rx_irq = DMA1_Stream0_IRQn;  /* 기본값 */
        if (config->dma_rx_stream == DMA2_Stream0) dma_rx_irq = DMA2_Stream0_IRQn;
        else if (config->dma_rx_stream == DMA2_Stream1) dma_rx_irq = DMA2_Stream1_IRQn;
        
        HAL_NVIC_SetPriority(dma_rx_irq, config->dma_rx_irq_priority, 0);
        HAL_NVIC_EnableIRQ(dma_rx_irq);
    }
    
    /* DMA TX NVIC */
    if (config->enable_dma_tx && config->dma_tx_stream != NULL) {
        IRQn_Type dma_tx_irq = DMA1_Stream0_IRQn;  /* 기본값 */
        if (config->dma_tx_stream == DMA2_Stream1) dma_tx_irq = DMA2_Stream1_IRQn;
        
        HAL_NVIC_SetPriority(dma_tx_irq, config->dma_tx_irq_priority, 0);
        HAL_NVIC_EnableIRQ(dma_tx_irq);
    }
#elif defined(IOIF_MCU_SERIES_G4)
    /* G4: DMA Channel 기반 - CubeMX가 NVIC를 자동 설정하므로 여기서는 skip */
    /* Manual Init 시 필요하면 DMA Channel에 맞는 IRQn을 직접 설정 */
    (void)config;  /* suppress unused warning */
#endif
    
    /* ========== [7] IOIF 인스턴스 풀 등록 (AssignInstance 와 호환되는 형태) ========== */
    instance->huart = &manual_info->huart_storage;
    instance->id    = new_id;
    /* Manual-init config 에는 rxMode 필드가 없어 IDLE_EVENT 를 기본 선택.
     * Xsens IMU 등 현재 모든 manual-init 경로가 IDLE 이벤트 기반 수신을 사용. */
    instance->config.rxMode             = IOIF_UART_MODE_IDLE_EVENT;
    instance->config.rx_event_callback  = NULL;  /* 이후 IOIF_UART_SetRxIdleCallback 으로 등록 */
    instance->rx_old_pos                = 0;

#if defined(USE_FREERTOS)
    instance->rx_dma_ptr = s_uart_rx_dma_buffer[new_id];

    /* TX 세마포어 생성 (AssignInstance 와 동일) */
    instance->tx_semaphore = xSemaphoreCreateBinary();
    if (instance->tx_semaphore == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    xSemaphoreGive(instance->tx_semaphore);

    /* IDLE_EVENT 모드: StreamBuffer + 공유 RxTask (AssignInstance 와 동일 경로) */
    instance->rx_stream_buffer = xStreamBufferCreate(IOIF_UART_STREAM_BUFFER_SIZE,
                                                      IOIF_UART_STREAM_BUFFER_TRIGGER);
    if (instance->rx_stream_buffer == NULL) {
        vSemaphoreDelete(instance->tx_semaphore);
        return AGRBStatus_ERROR;
    }

    if (s_uart_shared_rx_task == NULL) {
        BaseType_t ret = xTaskCreate(_IOIF_UART_SharedRxTask,
                                      "IOIF_UartRx",
                                      IOIF_UART_RX_TASK_STACK_SIZE / sizeof(StackType_t),
                                      NULL,
                                      IOIF_UART_RX_TASK_PRIORITY,
                                      &s_uart_shared_rx_task);
        if (ret != pdPASS) {
            vStreamBufferDelete(instance->rx_stream_buffer);
            instance->rx_stream_buffer = NULL;
            vSemaphoreDelete(instance->tx_semaphore);
            return AGRBStatus_INITIAL_FAILED;
        }
    }
    instance->rx_task_handle = NULL;
#else  /* USE_BAREMETAL */
    instance->rx_dma_ptr = s_uart_rx_dma_buffer[new_id];
    instance->tx_busy    = false;
#endif

    instance->allocated = true;
    *out_id = new_id;
    s_uart_instance_count++;

    /* AssignInstance 와 동일하게 즉시 수신 준비.
     * IDLE_EVENT 모드는 SetRxIdleCallback() 시점에 DMA 수신 시작됨. */
    return IOIF_UART_Start(new_id);
}

/**
 * ============================================================================
 * [신규] ISR Delegation Implementation (아키텍처 준수)
 * ============================================================================
 */

/**
 * @brief [ISR Wrapper] UART 인터럽트 핸들러 (System Layer → IOIF 위임)
 * @details
 * - ✅ HAL 완전히 숨김 - System Layer는 HAL 타입조차 모름
 * - ✅ ADC3 패턴과 동일한 위임 구조
 * 
 * @param id IOIF UART 인스턴스 ID
 */
void IOIF_UART_HandleIsr(IOIF_UARTx_t id)
{
    /* ID 유효성 검증 */
    if (id >= IOIF_UART_MAX_INSTANCES) {
        return;  /* 잘못된 ID */
    }
    
    IOIF_UART_Instance_t* inst = &s_uart_instances[id];
    if (!inst->allocated || inst->huart == NULL) {
        return;  /* 초기화되지 않은 인스턴스 */
    }
    
    /* ✅ HAL 호출 (IOIF 내부에서만) */
    HAL_UART_IRQHandler(inst->huart);
}

/**
 * @brief [ISR Wrapper] UART DMA RX 인터럽트 핸들러 (System Layer → IOIF 위임)
 * @details
 * - ✅ HAL 완전히 숨김 - System Layer는 HAL 타입조차 모름
 * - ✅ ADC3 패턴과 동일한 위임 구조
 * 
 * @param id IOIF UART 인스턴스 ID
 */
void IOIF_UART_HandleDmaRxIsr(IOIF_UARTx_t id)
{
    /* ID 유효성 검증 */
    if (id >= IOIF_UART_MAX_INSTANCES) {
        return;  /* 잘못된 ID */
    }
    
    IOIF_UART_Instance_t* inst = &s_uart_instances[id];
    if (!inst->allocated || inst->huart == NULL || inst->huart->hdmarx == NULL) {
        return;  /* 초기화되지 않은 인스턴스 또는 DMA 없음 */
    }
    
    /* ✅ HAL 호출 (IOIF 내부에서만) */
    HAL_DMA_IRQHandler(inst->huart->hdmarx);
}

/**
 * @brief [ISR Wrapper] UART DMA TX 인터럽트 핸들러 (System Layer → IOIF 위임)
 * @details
 * - ✅ HAL 완전히 숨김 - System Layer는 HAL 타입조차 모름
 * - ✅ ADC3 패턴과 동일한 위임 구조
 * 
 * @param id IOIF UART 인스턴스 ID
 */
void IOIF_UART_HandleDmaTxIsr(IOIF_UARTx_t id)
{
    /* ID 유효성 검증 */
    if (id >= IOIF_UART_MAX_INSTANCES) {
        return;  /* 잘못된 ID */
    }
    
    IOIF_UART_Instance_t* inst = &s_uart_instances[id];
    if (!inst->allocated || inst->huart == NULL || inst->huart->hdmatx == NULL) {
        return;  /* 초기화되지 않은 인스턴스 또는 DMA 없음 */
    }
    
    /* ✅ HAL 호출 (IOIF 내부에서만) */
    HAL_DMA_IRQHandler(inst->huart->hdmatx);
}

#endif /* AGRB_IOIF_UART_ENABLE */
