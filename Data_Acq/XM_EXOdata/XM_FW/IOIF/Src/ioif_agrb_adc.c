/**
 ******************************************************************************
 * @file    ioif_agrb_adc.c
 * @author  HyundoKim
 * @brief   [IOIF Layer] ADC 추상화 드라이버 구현부
 * @version 0.1
 * @date    Nov 12, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_adc.h"
#if defined(AGRB_IOIF_ADC_ENABLE)

#include <string.h>
#include <stdbool.h>

#if defined(USE_FREERTOS_DMA)
#include "cmsis_os.h" // <--- [수정] osDelay를 위해 이 헤더를 추가합니다.

/**
 * @brief DMA 전용 수신 버퍼 (Non-cacheable RAM에 위치)
 */
__attribute__((section(IOIF_DMA_SECTION)))
static uint16_t _adc_dma_buffer[IOIF_ADC_MAX_INSTANCES][IOIF_ADC_MAX_CHANNEL];
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#if defined(USE_FREERTOS_DMA)
/**
 * @brief ADC 인스턴스 Mutex 획득
 */
#define IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance)  do {                  \
    if (instance == NULL) return AGRBStatus_ERROR;                      \
    SemaphoreHandle_t __semaphore = (instance)->handle;                 \
    if (__semaphore != NULL) {                                          \
        if (xSemaphoreTake(__semaphore,                                 \
            pdMS_TO_TICKS(IOIF_ADC_CONVERSION_TIMEOUT)) != pdTRUE) {    \
            return AGRBStatus_TIMEOUT;                                  \
        }                                                               \
    } else {                                                            \
        return AGRBStatus_ERROR;                                        \
    }                                                                   \
} while(0)

/**
 * @brief ADC 인스턴스 Mutex 해제
 */
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance)  do {                  \
    if (instance == NULL) return AGRBStatus_ERROR;                      \
    SemaphoreHandle_t __semaphore = (instance)->handle;                 \
    if (__semaphore != NULL) {                                          \
        xSemaphoreGive(__semaphore);                                    \
    }                                                                   \
} while(0)

/**
 * @brief ADC 인스턴스 Mutex 해제 (ISR)
 */
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE_ISR(instance)  do {              \
    if (instance == NULL) return AGRBStatus_ERROR;                      \
    SemaphoreHandle_t __semaphore = (instance)->handle;                 \
    if (__semaphore != NULL) {                                          \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                  \
        xSemaphoreGiveFromISR(                                          \
            __semaphore,                                                \
            &xHigherPriorityTaskWoken);                                 \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   \
    }                                                                   \
} while(0)

/**
 * @brief DMA 완료 시그널 대기
 */
#define IOIF_ADC_WAIT_DMA_COMPLETE(instance)  do {                      \
    if (instance == NULL) return AGRBStatus_ERROR;                      \
    SemaphoreHandle_t __semaphore = (instance)->dma;                    \
    if (__semaphore != NULL) {                                          \
        if (xSemaphoreTake(__semaphore,                                 \
            pdMS_TO_TICKS(IOIF_ADC_CONVERSION_TIMEOUT)) != pdTRUE) {    \
            HAL_ADC_Stop_DMA(instance->hadc);                           \
            IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);                   \
            return AGRBStatus_TIMEOUT;                                  \
        }                                                               \
    } else {                                                            \
        IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);                       \
        return AGRBStatus_ERROR;                                        \
    }                                                                   \
} while(0)

/**
 * @brief [ISR] DMA 완료 시그널 전송
 */
#define IOIF_ADC_SIGNAL_DMA_COMPLETE(instance)  do {                    \
    if (instance == NULL) return; /* 핸들러 없음 */                     \
    SemaphoreHandle_t __semaphore = (instance)->dma;                    \
    if (__semaphore != NULL) {                                          \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                  \
        xSemaphoreGiveFromISR(                                          \
            __semaphore,                                                \
            &xHigherPriorityTaskWoken);                                 \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   \
    }                                                                   \
} while(0)

#else //No RTOS
/* ... (Bare-metal 매크로) ... */
#define IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance)
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance)
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE_ISR(instance)
#define IOIF_ADC_WAIT_DMA_COMPLETE(instance)
#define IOIF_ADC_SIGNAL_DMA_COMPLETE(instance)
#endif

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
 * @brief ADC 인스턴스 풀 (최대 2개)
 */
static IOIF_ADC_Instance_t _instances[IOIF_ADC_MAX_INSTANCES];
static uint32_t _instance_count = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static IOIF_ADC_Instance_t* _ioif_adc_find_handler(ADC_HandleTypeDef* hadc);
static inline uint32_t _ioif_adc_convert_resolution(uint32_t stm32_res_raw);
static inline ioif_agrb_adc_method_e _ioif_adc_convert_method(uint32_t stm32_method_raw);
static AGRBStatusDef _ioif_adc_get_value_dma_oneshot(IOIF_ADC_Instance_t* instance, IOIF_ADC_ReportData_t* report);
static AGRBStatusDef _ioif_adc_get_value_polling(IOIF_ADC_Instance_t* instance, IOIF_ADC_ReportData_t* report);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

AGRBStatusDef ioif_adc_assign_instance(IOIF_ADCx_t* id, ADC_HandleTypeDef* hadc)
{
    if (_instance_count >= IOIF_ADC_MAX_INSTANCES) return AGRBStatus_NO_RESOURCE;
    if (id == NULL || hadc == NULL) return AGRBStatus_PARAM_ERROR;

    // 이미 할당되었는지 검사
    for (uint32_t i = 0; i < _instance_count; i++) {
        if (_instances[i].hadc == hadc) {
            *id = i; // 기존 ID 반환
            return AGRBStatus_BUSY; // Already assigned
        }
    }

    // 새 인스턴스 할당
    memset(&_instances[_instance_count], 0, sizeof(IOIF_ADC_Instance_t));
    IOIF_ADC_Instance_t* instance = &_instances[_instance_count];
    instance->hadc = hadc;
    instance->channel = hadc->Init.NbrOfConversion;
    instance->method = _ioif_adc_convert_method(hadc->Init.ConversionDataManagement);
    instance->resolution = _ioif_adc_convert_resolution(hadc->Init.Resolution);

#if defined(USE_FREERTOS_DMA)
    memset(&_adc_dma_buffer[_instance_count], 0, sizeof(uint16_t) * IOIF_ADC_MAX_CHANNEL);

    instance->handle = xSemaphoreCreateMutex(); // Mutex로 변경
    instance->dma = xSemaphoreCreateBinary();
    
    if (instance->handle == NULL) return AGRBStatus_SEMAPHORE_ERROR;        
    if (instance->dma == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    
    // (Mutex는 Give 필요 없음)

    if (instance->method == IOIF_ADC_Method_DMAOneShot) {
        if (hadc->Init.ConversionDataManagement == ADC_CONVERSIONDATA_DMA_CIRCULAR) {
            return AGRBStatus_NOT_ALLOWED; // 순환 모드는 지원 안 함
        }
    }
#endif

    *id = _instance_count;
    _instance_count++;
    instance->assigned = true;

    return AGRBStatus_OK;
}

//런타임에는 쓰면 안되는 함수긴 함...
AGRBStatusDef ioif_adc_release_instance(IOIF_ADCx_t id)
{
    if (id >= _instance_count) return AGRBStatus_PARAM_ERROR;
    
    IOIF_ADC_Instance_t* instance = &_instances[id];
    if (!instance->assigned) return AGRBStatus_NOT_INITIALIZED;

#if defined(USE_FREERTOS_DMA)
    IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance); // 진행중인 작업이 끝날 때까지 대기

    if (instance->handle != NULL) {
        vSemaphoreDelete(instance->handle);
        instance->handle = NULL;
    }
    if (instance->dma != NULL) {
        vSemaphoreDelete(instance->dma);
        instance->dma = NULL;
    }
#endif

    memset(instance, 0, sizeof(IOIF_ADC_Instance_t));
    instance->assigned = false;

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_adc_get_value(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report)
{
    if (id >= _instance_count) return AGRBStatus_PARAM_ERROR;
    if (report == NULL) return AGRBStatus_PARAM_ERROR;

    IOIF_ADC_Instance_t* instance = &_instances[id];
    if (!instance->assigned) return AGRBStatus_NOT_INITIALIZED;
    if (instance->hadc == NULL) return AGRBStatus_NOT_INITIALIZED;
    if (instance->channel == 0) return AGRBStatus_PARAM_ERROR; // 채널 설정 없음
    if (instance->channel > IOIF_ADC_MAX_CHANNEL) return AGRBStatus_PARAM_ERROR;

    AGRBStatusDef result = AGRBStatus_OK; 
    
    // 인스턴스(ADC Peripheral) 보호 시작
    IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance);
    
    switch(instance->method) 
    {
        case IOIF_ADC_Method_DMAOneShot:
        {   
            result = _ioif_adc_get_value_dma_oneshot(instance, report);
        } break;
        case IOIF_ADC_Method_POLLING:
        {
            result = _ioif_adc_get_value_polling(instance, report);
        } break;
        default:
        {
            result = AGRBStatus_NOT_SUPPORTED;
        } break;
    }
    
    // 인스턴스(ADC Peripheral) 보호 해제
    IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);

    if (result == AGRBStatus_OK) {
        report->channel_count = instance->channel;
        report->resolution = instance->resolution;
    }
    
    return result;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief DMA One-Shot 방식으로 ADC 그룹 읽기 (내부 함수)
 * @note Mutex는 상위 함수(ioif_adc_get_value)에서 처리
 */
static AGRBStatusDef _ioif_adc_get_value_dma_oneshot(IOIF_ADC_Instance_t* instance, IOIF_ADC_ReportData_t* report)
{
    #if !defined(USE_FREERTOS_DMA)
    return AGRBStatus_NOT_SUPPORTED;
    #else
    
    // (id, report, instance NULL 체크는 상위 함수에서 완료)
    
    HAL_StatusTypeDef hal_status; 

    // 이전 변환이 멈췄는지 확인 (필요시)
    while (HAL_ADC_GetState(instance->hadc) & HAL_ADC_STATE_REG_BUSY) {
        osDelay(1);
    }

    uint16_t* dma_buffer = _adc_dma_buffer[instance - _instances];
    memset(dma_buffer, 0, sizeof(uint16_t) * IOIF_ADC_MAX_CHANNEL);

    // DMA 시작
    hal_status = HAL_ADC_Start_DMA(
        instance->hadc, 
        (uint32_t*)dma_buffer, 
        instance->channel 
    );

    if (hal_status != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    // DMA 완료 시그널 대기 (타임아웃 시 자동 Stop 및 에러 반환)
    IOIF_ADC_WAIT_DMA_COMPLETE(instance);
    
    // [버그 수정] HAL_ADC_Stop_DMA는 WAIT 매크로가 타임아웃 시 처리하므로
    // 성공 시에는 HAL_ADC_Stop()을 호출해야 합니다.
    hal_status = HAL_ADC_Stop(instance->hadc);
    if (hal_status != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    // 캐시된 DMA 버퍼를 무효화하고 CPU가 접근
    SCB_InvalidateDCache_by_Addr((uint32_t*)dma_buffer, sizeof(uint16_t) * instance->channel);
    for (uint32_t i = 0; i < instance->channel; i++) {
        report->value[i] = (uint32_t) dma_buffer[i];
    }

    return AGRBStatus_OK;
    #endif
}

/**
 * @brief Polling 방식으로 ADC 그룹 읽기 (내부 함수)
 * @note Mutex는 상위 함수(ioif_adc_get_value)에서 처리
 */
static AGRBStatusDef _ioif_adc_get_value_polling(IOIF_ADC_Instance_t* instance, IOIF_ADC_ReportData_t* report)
{
    // (id, report, instance NULL 체크는 상위 함수에서 완료)
    
    if (HAL_ADC_Start(instance->hadc) != HAL_OK) {
        HAL_ADC_Stop(instance->hadc);
        return AGRBStatus_ERROR;
    }

    for (uint32_t i = 0; i < instance->channel; i++) {
        // 변환 완료 대기
        if (HAL_ADC_PollForConversion(instance->hadc, IOIF_ADC_CONVERSION_TIMEOUT) != HAL_OK) {
            HAL_ADC_Stop(instance->hadc);
            return AGRBStatus_TIMEOUT;
        }

        // 값 읽기
        report->value[i] = HAL_ADC_GetValue(instance->hadc);
    }

    if (HAL_ADC_Stop(instance->hadc) != HAL_OK) {
        return AGRBStatus_ERROR;
    }
    
    return AGRBStatus_OK;
}

/**
 * @brief HAL의 ADC 해상도 정의를 12, 16 같은 정수로 변환
 */
static inline uint32_t _ioif_adc_convert_resolution(uint32_t stm32_res_raw)
{
    switch (stm32_res_raw) {
        case ADC_RESOLUTION_12B: return 12;
        case ADC_RESOLUTION_14B: return 14;
        case ADC_RESOLUTION_16B: return 16;
        #if defined (ADC_VER_V5_X)
        case ADC_RESOLUTION_14B_OPT: return 14;
        case ADC_RESOLUTION_12B_OPT: return 12;
        #endif
        default: return 12; // 기본 12비트
    }
}

/**
 * @brief HAL의 변환 모드를 IOIF enum으로 변환
 */
static inline ioif_agrb_adc_method_e _ioif_adc_convert_method(uint32_t stm32_method_raw)
{
    switch (stm32_method_raw) {
        case ADC_CONVERSIONDATA_DR: return IOIF_ADC_Method_POLLING;
        case ADC_CONVERSIONDATA_DMA_ONESHOT: return IOIF_ADC_Method_DMAOneShot;
        default: return IOIF_ADC_Method_POLLING; // 기본 Polling
    }
}

/**
 * @brief HAL 핸들 포인터로 IOIF 인스턴스를 찾음 (콜백용)
 */
static inline IOIF_ADC_Instance_t* _ioif_adc_find_handler(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL) return NULL;

    for (uint32_t i = 0; i < _instance_count; i++) {
        if (_instances[i].assigned && _instances[i].hadc == hadc) {
            return &_instances[i];
        }
    }
    return NULL; // 핸들러 없음
}

/**
 *------------------------------------------------------------
 * CALLBACK FUNCTIONS (HAL)
 *------------------------------------------------------------
 */

/**
 * @brief [HAL Callback] DMA 전송이 완료되면 호출됨
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{    
    IOIF_ADC_Instance_t* instance = _ioif_adc_find_handler(hadc);
    IOIF_ADC_SIGNAL_DMA_COMPLETE(instance); // 대기 중인 태스크(ioif_adc_get_value)를 깨움
}

/**
 * @brief [HAL Callback] (미사용)
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
}

/**
 * @brief [HAL Callback] (미사용)
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
}

/**
 * @brief [HAL Callback] ADC 에러 발생 시 호출됨
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    // 에러가 발생해도 DMA 시그널을 보내서,
    // 대기 중인 태스크(ioif_adc_get_value)가 Timeout에 빠지지 않고
    // 즉시 에러를 인지하고 리턴하도록 함
    IOIF_ADC_Instance_t* instance = _ioif_adc_find_handler(hadc);
    IOIF_ADC_SIGNAL_DMA_COMPLETE(instance);
}

#endif /* AGRB_IOIF_ADC_ENABLE */
