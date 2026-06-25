/**
 ******************************************************************************
 * @file    ioif_agrb_sai.c
 * @author  Angel Robotics Firmware Team (KimJinwoo)
 * @brief   [IOIF Layer] SAI Audio 하드웨어 추상화 구현부
 * @version 4.0 (Production Quality - Multi-Instance Redesign)
 * @date    Mar 22, 2026
 *
 * @details
 * - Multi-instance (max 2), FDCAN-style instance ID
 * - DMA 기반 오디오 전송 (Circular/Normal 모두 지원)
 * - ISR → xTaskNotifyFromISR → EventTask → 사용자 콜백
 * - State Machine: IDLE → BUSY → ERROR → (Reset) → IDLE
 * - H7 + RTOS = Full, 나머지 = NOT_SUPPORTED stub
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_sai.h"

#if defined(AGRB_IOIF_SAI_ENABLE)

/* ===== Platform gate: Full implementation only on H7+RTOS ===== */
#if defined(IOIF_MCU_SERIES_H7) && defined(USE_FREERTOS)

#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if defined(AGRB_IOIF_DMA_ENABLE)
#include "ioif_agrb_dma.h"
#endif

/**********************************************************************/
/***************** MACRO FOR SAI INTERFACE ****************************/
/**********************************************************************/
#define SAI_ERROR_LOG_SIZE               (32)
#define SAI_ERROR_LOG_MASK               (SAI_ERROR_LOG_SIZE - 1)
#if (SAI_ERROR_LOG_SIZE & (SAI_ERROR_LOG_MASK)) != 0
    #error "SAI_ERROR_LOG_SIZE must be power of 2"
#endif

#define SAI_EVENT_TASK_STACK_SIZE        (256)
#define SAI_EVENT_TASK_PRIORITY          (osPriorityAboveNormal)

/* TaskNotify bits */
#define SAI_NOTIFY_HALF_COMPLETE         (1U << 0)
#define SAI_NOTIFY_COMPLETE              (1U << 1)
#define SAI_NOTIFY_ERROR                 (1U << 2)
#define SAI_NOTIFY_ALL                   (SAI_NOTIFY_HALF_COMPLETE | SAI_NOTIFY_COMPLETE | SAI_NOTIFY_ERROR)

/** @brief 진단: HALF_COMPLETE + COMPLETE 동시 도착 횟수 (0이면 정상, >0이면 tearing 위험) */
volatile uint32_t g_dbg_sai_dual_notify = 0;

/**********************************************************************/
/***************** SAI STRUCTURE AND ENUM *****************************/
/**********************************************************************/
typedef enum {
    SAI_State_IDLE,
    SAI_State_BUSY,
    SAI_State_ERROR,
} SAI_State_e;

/**********************************************************************/
/***************** SAI INSTANCE STRUCTURE *****************************/
/**********************************************************************/

typedef struct {
    bool            is_assigned;
    IOIF_SAI_Config_t config;

    SAI_State_e     state;

    struct {
        IOIF_DMAx_t*        dma;
        uint32_t            sequence;
        size_t              send_length;
        uint8_t             last_event;    /**< 마지막 DMA 이벤트 (0=HALF, 1=COMPLETE) — BUSY half 결정용 */
    } buffer;

    struct {
        IOIF_SAI_EventCallback_t fn;
        void*                    ctx;
    } callback;

    struct {
        uint32_t counter;
        HAL_StatusTypeDef data[SAI_ERROR_LOG_SIZE];

        struct {
            struct {
                uint32_t complete;
                uint32_t half_complete;
            } callback;
            struct {
                uint32_t overflow;
                uint32_t underflow;
                uint32_t afsdet;
                uint32_t lfsdet;
                uint32_t cnready;
                uint32_t wckcfg;
                uint32_t timeout;
                uint32_t dma;
            } error;

            struct {
                uint32_t play;
                uint32_t stop;
                uint32_t reset;
            } requested;
        } event;
    } log;

    SemaphoreHandle_t   mutex;
    TaskHandle_t        event_task;

} IOIF_SAI_Instance_t;

/**********************************************************************/
/***************** PRIVATE VARIABLES **********************************/
/**********************************************************************/
static IOIF_SAI_Instance_t _sai_instances[IOIF_SAI_MAX_INSTANCES];
static uint32_t _sai_instance_count = 0;

/**********************************************************************/
/***************** PRIVATE FUNCTIONS **********************************/
/**********************************************************************/
static IOIF_SAI_Instance_t* _find_instance_by_hsai(SAI_HandleTypeDef* hsai);
static void _sai_event_task(void* arg);

/**********************************************************************/
/***************** PUBLIC API IMPLEMENTATION **************************/
/**********************************************************************/

AGRBStatusDef IOIF_SAI_AssignInstance(IOIF_SAIx_t* id, IOIF_SAI_Config_t* config)
{
    if (id == NULL || config == NULL) return AGRBStatus_PARAM_ERROR;
    if (config->hsai == NULL) return AGRBStatus_PARAM_ERROR;
    if (_sai_instance_count >= IOIF_SAI_MAX_INSTANCES) return AGRBStatus_INSTANCE_FULL;

    /* Duplicate check */
    if (_find_instance_by_hsai(config->hsai) != NULL) return AGRBStatus_ALREADY_INITIALIZED;

    IOIF_SAIx_t index = _sai_instance_count;
    IOIF_SAI_Instance_t* instance = &_sai_instances[index];

    memset(instance, 0, sizeof(IOIF_SAI_Instance_t));
    memcpy(&instance->config, config, sizeof(IOIF_SAI_Config_t));

    instance->state = SAI_State_IDLE;

    /* DMA TX 핸들 검증 (Circular/Normal 모두 허용) */
    if ( instance->config.hsai->hdmatx == NULL ) {
        return AGRBStatus_DMA_ALLOCATION_ERROR;
    }

    /* DMA 버퍼 할당 (capacity * 2 = 더블 버퍼) */
    #if defined(AGRB_IOIF_DMA_ENABLE)
    instance->buffer.dma = ioif_dma.allocate(instance->config.hsai->hdmatx,
                                              (config->capacity * 2), "SAI_DMA_TX");
    if ( instance->buffer.dma == NULL ) return AGRBStatus_DMA_ALLOCATION_ERROR;
    #else
    return AGRBStatus_NOT_SUPPORTED;
    #endif

    /* Mutex */
    instance->mutex = xSemaphoreCreateMutex();
    if ( instance->mutex == NULL ) return AGRBStatus_SEMAPHORE_ERROR;

    /* Event Task (ISR → TaskNotify → 사용자 콜백) */
    BaseType_t task_res = xTaskCreate(
        _sai_event_task,
        "SAI_Evt",
        SAI_EVENT_TASK_STACK_SIZE,
        (void*)(uintptr_t)index,
        SAI_EVENT_TASK_PRIORITY,
        &instance->event_task );

    if ( task_res != pdPASS ) {
        vSemaphoreDelete(instance->mutex);
        instance->mutex = NULL;
        return AGRBStatus_NOT_INITIALIZED;
    }

    instance->is_assigned = true;
    *id = index;
    _sai_instance_count++;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_RegisterCallback(IOIF_SAIx_t id, IOIF_SAI_EventCallback_t cb, void* ctx)
{
    if (id >= _sai_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_SAI_Instance_t* instance = &_sai_instances[id];
    if (!instance->is_assigned) return AGRBStatus_NOT_INITIALIZED;

    instance->callback.fn = cb;
    instance->callback.ctx = ctx;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_Play(IOIF_SAIx_t id, const void* buffer, size_t length)
{
    if (id >= _sai_instance_count) return AGRBStatus_PARAM_ERROR;

    IOIF_SAI_Instance_t* instance = &_sai_instances[id];

    /* length는 bytes 단위 — 실제 DMA 버퍼 크기와 비교 */
    if (instance->buffer.dma != NULL && length > instance->buffer.dma->size) {
        return AGRBStatus_BUFFER_OVERFLOW;
    }
    if (!instance->is_assigned) return AGRBStatus_NOT_INITIALIZED;

    TickType_t wait = (instance->config.timeout_ms == 0) ?
                      portMAX_DELAY : pdMS_TO_TICKS(instance->config.timeout_ms);

    instance->log.event.requested.play++;

    if (xSemaphoreTake(instance->mutex, wait) != pdTRUE) return AGRBStatus_TIMEOUT;

    AGRBStatusDef result = AGRBStatus_OK;

    switch ( instance->state )
    {
        case SAI_State_IDLE:
        {
            instance->state = SAI_State_BUSY;
            instance->buffer.sequence = 0;
            instance->buffer.last_event = 1;  /* IDLE→BUSY 전환: 다음 BUSY write는 2nd half로 */

            if ( buffer != NULL && length > 0 ) {
                memset( (uint8_t*)instance->buffer.dma->buffer, 0, instance->buffer.dma->size );
                memcpy( (uint8_t*)instance->buffer.dma->buffer, (const uint8_t*)buffer, length );
            }
            #if IOIF_HAS_DCACHE
            SCB_CleanDCache_by_Addr( (uint32_t*)instance->buffer.dma->buffer, (instance->buffer.dma->size) );
            #endif

            instance->buffer.send_length = length;

            /* Size = 16-bit 전송 횟수 (SAI 16-bit I2S: 1 sample = 2 bytes)
             * buffer.dma->size(bytes) / sizeof(uint16_t) = 전체 16-bit 전송 횟수 */
            HAL_StatusTypeDef hal_result = HAL_SAI_Transmit_DMA( instance->config.hsai,
                                   (uint8_t*)instance->buffer.dma->buffer,
                                   (instance->buffer.dma->size) / sizeof(uint16_t) );

            if ( hal_result != HAL_OK ) {
                instance->state = SAI_State_ERROR;
                result = IOIF_HAL_ERROR_TO_AGRB_STATUSDEF( hal_result );
            } else {
                instance->buffer.sequence++;
                /* ★ artificial HALF_COMPLETE 제거 — race condition 방지
                 * eSetBits merge로 real DMA HALF_COMPLETE와 합쳐져 1회 콜백 유실 가능.
                 * 호출자가 Play(IDLE) 후 즉시 Play(BUSY)로 2nd half를 직접 채워야 함. */
            }
            break;
        }
        case SAI_State_BUSY:
        {
            uint32_t half_size = instance->buffer.dma->size / 2;

            /* 이벤트 기반 inactive half 결정 (sequence 카운터 대체):
             * last_event=0 (HALF_COMPLETE): DMA가 2nd half 읽는 중 → 1st half에 쓰기 (offset=0)
             * last_event=1 (COMPLETE):      DMA가 1st half 읽는 중 → 2nd half에 쓰기 (offset=half)
             * sequence 기반은 EventTask 지연 시 DMA 위치와 어긋남 → tearing/드르르륵 발생 */
            uint32_t offset = (instance->buffer.last_event == 0) ? 0 : half_size;

            /* DMA가 다른 half를 재생하는 동안 이 half를 갱신.
             * memcpy를 먼저 수행하고, 잔여 영역만 0으로 채움.
             * ★ memset(전체)→memcpy 순서 금지: non-cacheable 영역에서
             *    DMA가 memset의 0을 읽어 팝 노이즈 발생 */
            if ( buffer != NULL && length > 0 ) {
                memcpy( (uint8_t*)instance->buffer.dma->buffer + offset, (const uint8_t*)buffer, length );
                /* 잔여 영역 zero-fill (length < half_size인 경우만) */
                if (length < half_size) {
                    memset( (uint8_t*)instance->buffer.dma->buffer + offset + length, 0, half_size - length );
                }
            } else {
                memset( (uint8_t*)&instance->buffer.dma->buffer[offset], 0, half_size );
            }
            #if IOIF_HAS_DCACHE
            SCB_CleanDCache_by_Addr( (uint32_t*)(instance->buffer.dma->buffer + offset), half_size );
            #endif

            instance->buffer.sequence++;
            instance->buffer.send_length += length;

            result = AGRBStatus_OK;
            break;
        }
        case SAI_State_ERROR:
        default:
            result = AGRBStatus_INVALID_STATE;
            break;
    }

    xSemaphoreGive(instance->mutex);
    return result;
}

AGRBStatusDef IOIF_SAI_PlayDirect(IOIF_SAIx_t id, const void* buffer, size_t length)
{
    if (id >= _sai_instance_count) return AGRBStatus_PARAM_ERROR;

    IOIF_SAI_Instance_t* instance = &_sai_instances[id];
    if (!instance->is_assigned) return AGRBStatus_NOT_INITIALIZED;
    if (buffer == NULL || length == 0) return AGRBStatus_PARAM_ERROR;

    TickType_t wait = (instance->config.timeout_ms == 0) ?
                      portMAX_DELAY : pdMS_TO_TICKS(instance->config.timeout_ms);

    if (xSemaphoreTake(instance->mutex, wait) != pdTRUE) return AGRBStatus_TIMEOUT;

    /* Circular DMA 실행 중이면 충돌 방지 */
    if (instance->state == SAI_State_BUSY) {
        xSemaphoreGive(instance->mutex);
        return AGRBStatus_INVALID_STATE;
    }
    if (instance->state == SAI_State_ERROR) {
        xSemaphoreGive(instance->mutex);
        return AGRBStatus_INVALID_STATE;
    }

    /* 외부 버퍼에서 직접 DMA 전송 (CM-H10 AudioPlayer 패턴)
     * 내부 DMA 버퍼 미사용, Circular 아닌 one-shot 전송
     * 완료 시 TxCplt 콜백 → COMPLETE 이벤트 발생 → 다음 블록 요청 */
    HAL_StatusTypeDef hal_result = HAL_SAI_Transmit_DMA(
        instance->config.hsai,
        (uint8_t*)buffer,
        length / sizeof(uint16_t));

    if (hal_result != HAL_OK) {
        xSemaphoreGive(instance->mutex);
        return IOIF_HAL_ERROR_TO_AGRB_STATUSDEF(hal_result);
    }

    instance->state = SAI_State_BUSY;
    xSemaphoreGive(instance->mutex);
    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_Stop(IOIF_SAIx_t id)
{
    if (id >= _sai_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_SAI_Instance_t* instance = &_sai_instances[id];
    if (!instance->is_assigned) return AGRBStatus_NOT_INITIALIZED;

    TickType_t wait = (instance->config.timeout_ms == 0) ?
                      portMAX_DELAY : pdMS_TO_TICKS(instance->config.timeout_ms);

    instance->log.event.requested.stop++;

    if (xSemaphoreTake(instance->mutex, wait) != pdTRUE) return AGRBStatus_TIMEOUT;

    HAL_StatusTypeDef hal_result = HAL_OK;

    if ( instance->state == SAI_State_BUSY )
    {
        hal_result = HAL_SAI_Abort( instance->config.hsai );
        instance->state = SAI_State_IDLE;
        instance->buffer.sequence = 0;
        instance->buffer.send_length = 0;
    }

    /* Stale DMA ISR notification 클리어 — Abort 직전 fire된 HALF/COMPLETE가
     * 남아있으면 다음 Play 시 EventTask가 stale 비트로 즉시 깨어남 → dual notify.
     * STATE + VALUE 모두 클리어 필수. */
    if (instance->event_task != NULL) {
        xTaskNotifyStateClear(instance->event_task);
        ulTaskNotifyValueClear(instance->event_task, SAI_NOTIFY_ALL);
    }

    xSemaphoreGive(instance->mutex);
    return IOIF_HAL_ERROR_TO_AGRB_STATUSDEF( hal_result );
}

AGRBStatusDef IOIF_SAI_Reset(IOIF_SAIx_t id)
{
    if (id >= _sai_instance_count) return AGRBStatus_PARAM_ERROR;
    IOIF_SAI_Instance_t* instance = &_sai_instances[id];
    if (!instance->is_assigned) return AGRBStatus_NOT_INITIALIZED;

    TickType_t wait = (instance->config.timeout_ms == 0) ?
                      portMAX_DELAY : pdMS_TO_TICKS(instance->config.timeout_ms);

    instance->log.event.requested.reset++;

    if (xSemaphoreTake(instance->mutex, wait) != pdTRUE) return AGRBStatus_TIMEOUT;

    AGRBStatusDef result = AGRBStatus_OK;

    /* Abort → DeInit → Init → IDLE */
    if ( instance->state == SAI_State_BUSY ) {
        HAL_SAI_Abort( instance->config.hsai );
    }

    HAL_StatusTypeDef hal = HAL_SAI_DeInit( instance->config.hsai );
    if ( hal != HAL_OK ) {
        result = IOIF_HAL_ERROR_TO_AGRB_STATUSDEF(hal);
    } else {
        hal = HAL_SAI_Init( instance->config.hsai );
        result = IOIF_HAL_ERROR_TO_AGRB_STATUSDEF(hal);
    }

    instance->state = SAI_State_IDLE;
    instance->buffer.sequence = 0;
    instance->buffer.send_length = 0;

    /* Clear pending notifications */
    if (instance->event_task != NULL) {
        xTaskNotifyStateClear(instance->event_task);
    }

    xSemaphoreGive(instance->mutex);
    return result;
}

size_t IOIF_SAI_GetBufferCapacity(IOIF_SAIx_t id)
{
    if (id >= _sai_instance_count) return 0;
    IOIF_SAI_Instance_t* instance = &_sai_instances[id];
    if (!instance->is_assigned || instance->buffer.dma == NULL) return 0;
    return instance->buffer.dma->size / 2;
}

/**********************************************************************/
/***************** PRIVATE FUNCTIONS **********************************/
/**********************************************************************/

static IOIF_SAI_Instance_t* _find_instance_by_hsai(SAI_HandleTypeDef* hsai)
{
    for (uint32_t i = 0; i < _sai_instance_count; i++) {
        if (_sai_instances[i].is_assigned && _sai_instances[i].config.hsai == hsai) {
            return &_sai_instances[i];
        }
    }
    return NULL;
}

static void _sai_event_task(void* arg)
{
    IOIF_SAIx_t id = (IOIF_SAIx_t)(uintptr_t)arg;
    IOIF_SAI_Instance_t* instance = &_sai_instances[id];

    uint32_t bits;
    for (;;) {
        /* ulBitsToClearOnExit = 0: 수동 클리어 패턴.
         * SAI_NOTIFY_ALL로 자동 클리어하면, bits 읽기~클리어 사이에 ISR이
         * 새 비트를 세팅해도 함께 클리어되어 이벤트 유실 발생.
         * → 처리한 비트만 수동으로 xTaskNotifyStateClear/ulTaskNotifyValueClear */
        if (xTaskNotifyWait(0, 0, &bits, portMAX_DELAY) != pdTRUE) continue;

        /* 처리할 비트만 클리어 (새로 도착한 비트는 보존) */
        ulTaskNotifyValueClear(NULL, bits);

        if (instance->callback.fn == NULL) continue;

        /* 진단: 두 이벤트 동시 도착 카운트 */
        if ((bits & SAI_NOTIFY_HALF_COMPLETE) && (bits & SAI_NOTIFY_COMPLETE)) {
            g_dbg_sai_dual_notify++;
        }

        if (bits & SAI_NOTIFY_ERROR) {
            instance->callback.fn(id, IOIF_SAI_EVENT_ERROR, instance->callback.ctx);
        }
        /* HALF_COMPLETE → COMPLETE 순서 필수 (DMA Circular 시간 순서 유지) */
        if (bits & SAI_NOTIFY_HALF_COMPLETE) {
            instance->buffer.last_event = 0;  /* HALF_COMPLETE: inactive = 1st half */
            instance->callback.fn(id, IOIF_SAI_EVENT_HALF_COMPLETE, instance->callback.ctx);
        }
        if (bits & SAI_NOTIFY_COMPLETE) {
            instance->buffer.last_event = 1;  /* COMPLETE: inactive = 2nd half */
            instance->callback.fn(id, IOIF_SAI_EVENT_COMPLETE, instance->callback.ctx);
        }
    }
}

/**********************************************************************/
/********************  HAL CALLBACK IMPLEMENTS ************************/
/**********************************************************************/

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    IOIF_SAI_Instance_t* instance = _find_instance_by_hsai(hsai);
    if ( instance == NULL ) return;

    instance->log.event.callback.complete++;

    if (instance->event_task != NULL) {
        BaseType_t xWoken = pdFALSE;
        xTaskNotifyFromISR(instance->event_task, SAI_NOTIFY_COMPLETE, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
    IOIF_SAI_Instance_t* instance = _find_instance_by_hsai(hsai);
    if ( instance == NULL ) return;

    instance->log.event.callback.half_complete++;

    if (instance->event_task != NULL) {
        BaseType_t xWoken = pdFALSE;
        xTaskNotifyFromISR(instance->event_task, SAI_NOTIFY_HALF_COMPLETE, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
    IOIF_SAI_Instance_t* instance = _find_instance_by_hsai(hsai);
    if ( instance == NULL ) return;

    instance->state = SAI_State_ERROR;

    uint32_t error = HAL_SAI_GetError(hsai);

    if ( error & HAL_SAI_ERROR_OVR ) {
        instance->log.event.error.overflow++;
    }
    if ( error & HAL_SAI_ERROR_UDR ) {
        instance->log.event.error.underflow++;
    }
    if ( error & HAL_SAI_ERROR_AFSDET ) {
        instance->log.event.error.afsdet++;
    }
    if ( error & HAL_SAI_ERROR_CNREADY ) {
        instance->log.event.error.cnready++;
    }
    if ( error & HAL_SAI_ERROR_LFSDET ) {
        instance->log.event.error.lfsdet++;
    }
    if ( error & HAL_SAI_ERROR_WCKCFG ) {
        instance->log.event.error.wckcfg++;
    }
    if ( error & HAL_SAI_ERROR_TIMEOUT ) {
        instance->log.event.error.timeout++;
    }
    if ( error & HAL_SAI_ERROR_DMA ) {
        instance->log.event.error.dma++;
    }

    if (instance->event_task != NULL) {
        BaseType_t xWoken = pdFALSE;
        xTaskNotifyFromISR(instance->event_task, SAI_NOTIFY_ERROR, eSetBits, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

#else /* Not (H7 + RTOS): Stub implementations */

AGRBStatusDef IOIF_SAI_AssignInstance(IOIF_SAIx_t* id, IOIF_SAI_Config_t* config)
{
    (void)id; (void)config;
    return AGRBStatus_NOT_SUPPORTED;
}

AGRBStatusDef IOIF_SAI_RegisterCallback(IOIF_SAIx_t id, IOIF_SAI_EventCallback_t cb, void* ctx)
{
    (void)id; (void)cb; (void)ctx;
    return AGRBStatus_NOT_SUPPORTED;
}

AGRBStatusDef IOIF_SAI_Play(IOIF_SAIx_t id, const void* buffer, size_t length)
{
    (void)id; (void)buffer; (void)length;
    return AGRBStatus_NOT_SUPPORTED;
}

AGRBStatusDef IOIF_SAI_PlayDirect(IOIF_SAIx_t id, const void* buffer, size_t length)
{
    (void)id; (void)buffer; (void)length;
    return AGRBStatus_NOT_SUPPORTED;
}

AGRBStatusDef IOIF_SAI_Stop(IOIF_SAIx_t id)
{
    (void)id;
    return AGRBStatus_NOT_SUPPORTED;
}

AGRBStatusDef IOIF_SAI_Reset(IOIF_SAIx_t id)
{
    (void)id;
    return AGRBStatus_NOT_SUPPORTED;
}

size_t IOIF_SAI_GetBufferCapacity(IOIF_SAIx_t id)
{
    (void)id;
    return 0;
}

#endif /* IOIF_MCU_SERIES_H7 && USE_FREERTOS */

#endif /* AGRB_IOIF_SAI_ENABLE */
