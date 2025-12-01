#include "ioif_agrb_adc.h"
#if !defined(AGRB_IOIF_ADC_DISABLE)

#include <string.h>
#include <stdbool.h>

#if defined(USE_FREERTOS_DMA)
__attribute__((section(IOIF_DMA_SECTION)))
static uint16_t _adc_dma_buffer[IOIF_ADC_MAX_INSTANCES][IOIF_ADC_MAX_CHANNEL];
#endif

#if defined(USE_FREERTOS_DMA)
#define IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance)  do {                  \
    if (instance == NULL) return; /* No handler found */                \
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

#define IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance)  do {                  \
    if (instance == NULL) return; /* No handler found */                \
    SemaphoreHandle_t __semaphore = (instance)->handle;                 \
    if (__semaphore != NULL) {                                          \
        xSemaphoreGive(__semaphore);                                    \
    }                                                                   \
} while(0)

#define IOIF_ADC_RELEASE_DEV_SEMAPHORE_ISR(instance)  do {              \
    if (instance == NULL) return; /* No handler found */                \
    SemaphoreHandle_t __semaphore = (instance)->handle;                 \
    if (__semaphore != NULL) {                                          \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                  \
        xSemaphoreGiveFromISR(                                          \
            __semaphore,                                                \
            &xHigherPriorityTaskWoken);                                 \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   \
    }                                                                   \
} while(0)

#define IOIF_ADC_WAIT_DMA_COMPLETE(instance)  do {                      \
    if (instance == NULL) return; /* No handler found */                \
    SemaphoreHandle_t __semaphore = (instance)->dma;                    \
    if (__semaphore != NULL) {                                          \
        if (xSemaphoreTake(__semaphore,                                 \
            pdMS_TO_TICKS(IOIF_ADC_CONVERSION_TIMEOUT)) != pdTRUE) {    \
            IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);                   \
            return AGRBStatus_TIMEOUT;                                  \
        }                                                               \
    } else {                                                            \
        IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);                       \
        return AGRBStatus_ERROR;                                        \
    }                                                                   \
} while(0)

#define IOIF_ADC_SIGNAL_DMA_COMPLETE(instance)  do {                    \
    if (instance == NULL) return; /* No handler found */                \
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

#define IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance)
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance)
#define IOIF_ADC_RELEASE_DEV_SEMAPHORE_ISR(instance)
#define IOIF_ADC_WAIT_DMA_COMPLETE(instance)
#define IOIF_ADC_SIGNAL_DMA_COMPLETE(instance)

#endif

static IOIF_ADC_Instance_t* _ioif_adc_find_handler(ADC_HandleTypeDef* hadc);
static uint32_t _ioif_adc_convert_resolution(uint32_t stm32_res_raw);
static ioif_agrb_adc_method_e _ioif_adc_convert_method(uint32_t stm32_method_raw);
static AGRBStatusDef _ioif_adc_get_value_dma_oneshot(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report);
static AGRBStatusDef _ioif_adc_get_value_polling(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report);

static IOIF_ADC_Instance_t _instances[IOIF_ADC_MAX_INSTANCES];
static uint32_t _instance_count = 0;

AGRBStatusDef ioif_adc_assign_instance(IOIF_ADCx_t* id, ADC_HandleTypeDef* hadc)
{
    if (_instance_count >= IOIF_ADC_MAX_INSTANCES) return AGRBStatus_NO_RESOURCE;
    if (id == NULL || hadc == NULL) return AGRBStatus_PARAM_ERROR;

    //Check if the hadc is already assigned
    for (uint32_t i = 0; i < _instance_count; i++) {
        if (_instances[i].hadc == hadc) {
            return AGRBStatus_BUSY; //Already assigned
        }
    }

    //Assign new instance
    memset(&_instances[_instance_count], 0, sizeof(IOIF_ADC_Instance_t));
    IOIF_ADC_Instance_t* instance = &_instances[_instance_count];
    instance->hadc = hadc;
    instance->channel = hadc->Init.NbrOfConversion; //Default to max channel count
    instance->method = _ioif_adc_convert_method(hadc->Init.ConversionDataManagement);
    instance->resolution = _ioif_adc_convert_resolution(hadc->Init.Resolution);

#if defined(USE_FREERTOS_DMA)
    //if 'USE FREERTOS_DMA' is defined, semaphore must be created for DMA method
    memset(&_adc_dma_buffer[_instance_count], 0, sizeof(uint16_t) * IOIF_ADC_MAX_CHANNEL);

    instance->handle = xSemaphoreCreateBinary();
    instance->dma = xSemaphoreCreateBinary();
    
    if (instance->handle == NULL) return AGRBStatus_SEMAPHORE_ERROR;        
    if (instance->dma == NULL) return AGRBStatus_SEMAPHORE_ERROR;
    
    //Release semaphore to indicate ready
    xSemaphoreGive(instance->handle);

    if (instance->method == IOIF_ADC_Method_DMAOneShot) {
        //If DMA Circular mode is used NO ALLOWED
        if (hadc->Init.ConversionDataManagement == ADC_CONVERSIONDATA_DMA_CIRCULAR) {
            return AGRBStatus_NOT_ALLOWED;
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

    //Release resources
#if defined(USE_FREERTOS_DMA)
    //ACQUIRE semaphore to ensure no ongoing operation
    IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance);

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

    AGRBStatusDef result = AGRBStatus_OK; 
    
    switch(instance->method) 
    {
        case IOIF_ADC_Method_DMAOneShot:
        {   
            result = _ioif_adc_get_value_dma_oneshot(id, report);
        } break;
        case IOIF_ADC_Method_POLLING:
        {
            result = _ioif_adc_get_value_polling(id, report);
        } break;
        default:
        {
            result = AGRBStatus_NOT_SUPPORTED;
        } break;
    }

    return result;
}


static AGRBStatusDef _ioif_adc_get_value_dma_oneshot(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report)
{
    #if !defined(USE_FREERTOS_DMA)
    return AGRBStatus_NOT_SUPPORTED;//"You must define USE_FREERTOS_DMA to use async ADC functions."
    #else
    if (id == IOIF_ADC_NOT_INITIALIZED) return AGRBStatus_NOT_INITIALIZED;
    if (report == NULL) return AGRBStatus_PARAM_ERROR;
    if ( id >= _instance_count) return AGRBStatus_PARAM_ERROR;

    IOIF_ADC_Instance_t* instance = &_instances[id];

    if (instance->hadc == NULL) return AGRBStatus_NOT_INITIALIZED;
    if (instance->channel == 0) return AGRBStatus_PARAM_ERROR; //No channel configured
    if (instance->channel > IOIF_ADC_MAX_CHANNEL) return AGRBStatus_PARAM_ERROR; //Invalid channel number

    report->channel_count = instance->channel;
    report->resolution = instance->resolution;

    HAL_StatusTypeDef hal_status; 

    IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance);
    {        
        while (HAL_ADC_GetState(instance->hadc) & HAL_ADC_STATE_REG_BUSY) {
            //Wait until previous conversion is finished
            //Should not take long
            osDelay(1);
        }

        memset(_adc_dma_buffer[id], 0, sizeof(uint16_t) * IOIF_ADC_MAX_CHANNEL);

        hal_status = HAL_ADC_Start_DMA(\
            instance->hadc, \
            (uint32_t*)_adc_dma_buffer[id], \
            instance->channel \
        );

        if (hal_status != HAL_OK) {
            return AGRBStatus_ERROR;
        }

        //Wait for DMA complete signal
        IOIF_ADC_WAIT_DMA_COMPLETE(instance);
        
        SCB_InvalidateDCache_by_Addr((uint32_t*)_adc_dma_buffer[id], sizeof(uint16_t) * IOIF_ADC_MAX_CHANNEL);
        for (uint32_t i = 0; i < instance->channel; i++) {
            report->value[i] = (uint32_t) _adc_dma_buffer[id][i];
        }

        hal_status = HAL_ADC_Stop(instance->hadc);
        if (hal_status != HAL_OK) {
            return AGRBStatus_ERROR;
        }
    }
    IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);

    return AGRBStatus_OK;

    #endif
}

static AGRBStatusDef _ioif_adc_get_value_polling(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report)
{
    if (id == IOIF_ADC_NOT_INITIALIZED) return AGRBStatus_NOT_INITIALIZED;
    if ( id >= _instance_count) return AGRBStatus_PARAM_ERROR;
    if (report == NULL) return AGRBStatus_PARAM_ERROR;

    IOIF_ADC_Instance_t* instance = &_instances[id];
 
    if (instance->hadc == NULL) return AGRBStatus_NOT_INITIALIZED;
    if (instance->channel == 0) return AGRBStatus_PARAM_ERROR; //No channel configured
    if (instance->channel > IOIF_ADC_MAX_CHANNEL) return AGRBStatus_PARAM_ERROR; //Invalid channel number

    report->channel_count = instance->channel;
    report->resolution = instance->resolution;

    IOIF_ADC_ACQUIRE_DEV_SEMAPHORE(instance);
    {
        if (HAL_ADC_Start(instance->hadc) != HAL_OK) {
            HAL_ADC_Stop(instance->hadc);
            IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);
            return AGRBStatus_ERROR;
        }

        for (uint32_t i = 0; i < instance->channel; i++) {
            //Wait for conversion complete
            if (HAL_ADC_PollForConversion(instance->hadc, IOIF_ADC_CONVERSION_TIMEOUT) != HAL_OK) {
                HAL_ADC_Stop(instance->hadc);
                IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);
                return AGRBStatus_TIMEOUT;
            }

            //Get value
            uint32_t adc_value = HAL_ADC_GetValue(instance->hadc);
            report->value[i] = adc_value;
        }

        if (HAL_ADC_Stop(instance->hadc) != HAL_OK) {
            IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);
            return AGRBStatus_ERROR;
        }
    }
    IOIF_ADC_RELEASE_DEV_SEMAPHORE(instance);
    
    return AGRBStatus_OK;
}
static inline ioif_agrb_adc_method_e _ioif_adc_convert_method(uint32_t stm32_method_raw)
{
    switch (stm32_method_raw) {
        case ADC_CONVERSIONDATA_DR: return IOIF_ADC_Method_POLLING;
        case ADC_CONVERSIONDATA_DMA_ONESHOT: return IOIF_ADC_Method_DMAOneShot;
        //#if defined (ADC_VER_V5_X)
        //case ADC_CONVERSIONDATA_DMA_CIRCULAR: return IOIF_ADC_Method_DMALoop;
        //#endif
        default: return IOIF_ADC_Method_POLLING; //Default to polling
    }
}

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
        default: return 0; //Unsupported resolution
    }
}

static inline IOIF_ADC_Instance_t* _ioif_adc_find_handler(ADC_HandleTypeDef* hadc)
{
    if (hadc == NULL) return AGRBStatus_PARAM_ERROR;

    for (uint32_t i = 0; i < _instance_count; i++) {
        if (_instances[i].assigned && _instances[i].hadc == hadc) {
            return &_instances[i];
        }
    }

    return NULL; //No handler found
}

/**********************************************/
/** Callback for DMA complete signal **********/
/**********************************************/

static uint32_t _debug_adc_dma_complete_count = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{    
    IOIF_ADC_Instance_t* instance = _ioif_adc_find_handler(hadc);

    IOIF_ADC_SIGNAL_DMA_COMPLETE(instance);                                   
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvHalfCpltCallback must be implemented in the user file.
  */
}

/**
  * @brief  Analog watchdog 1 callback in non-blocking mode.
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_LevelOutOfWindowCallback must be implemented in the user file.
  */
}

/**
  * @brief  ADC error callback in non-blocking mode
  *         (ADC conversion with interruption or transfer by DMA).
  * @note   In case of error due to overrun when using ADC with DMA transfer
  *         (HAL ADC handle parameter "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
  *         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
  *         - If needed, restart a new ADC conversion using function
  *           "HAL_ADC_Start_DMA()"
  *           (this function is also clearing overrun flag)
  * @param hadc ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    IOIF_ADC_Instance_t* instance = _ioif_adc_find_handler(hadc);

    IOIF_ADC_SIGNAL_DMA_COMPLETE(instance);

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ErrorCallback must be implemented in the user file.
  */
}


#endif // AGRB_IOIF_ADC_DISABLE
