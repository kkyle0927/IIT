#include "ioif_agrb_defs.h"
#if defined(AGRB_IOIF_ADC_ENABLE)

#ifndef _IOIF_AGRB_ADC_H_
#define _IOIF_AGRB_ADC_H_

#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_adc.h"

#include <stdint.h>
#include <stdbool.h>

#define IOIF_ADC_NOT_INITIALIZED    (0xFFFFFFFF)

#define IOIF_ADC_MAX_INSTANCES              (2)
#define IOIF_ADC_CHANNELS_PER_INSTANCE      (4) //Max 4 channels per instance
#define IOIF_ADC_MAX_CHANNEL                (8) //12, 14, 16 bits supported in STM32H7
#define IOIF_ADC_DEFAULT_TIMEOUT            (100U) //in ms
#define IOIF_ADC_CONVERSION_TIMEOUT         (100U) //in ms


#define IOIF_ADC_INITIALIZE(id, hadc)      								ioif_adc_assign_instance(&(id), (hadc))
#define IOIF_ADC_GET_VALUE(id, report)                                  ioif_adc_get_value((id), (report))
#define IOIF_ADC_RELEASE(id)                                            ioif_adc_release_instance((id))


typedef enum {
    IOIF_ADC_Method_POLLING = 0,
    IOIF_ADC_Method_DMAOneShot,
    //IOIF_ADC_Method_DMALoop,

} ioif_agrb_adc_method_e;

typedef ioif_agrb_adc_method_e IOIF_ADC_Method_e;
typedef uint32_t IOIF_ADCx_t;

typedef struct {
    bool assigned;
    
    ADC_HandleTypeDef* hadc;
    uint16_t channel; //ADC_CHANNEL_XXX, Ranked num
    ioif_agrb_adc_method_e method;
    uint32_t resolution; //in bits

#if defined(USE_FREERTOS_DMA)
    SemaphoreHandle_t handle;
    SemaphoreHandle_t dma; //for DMA completion
#endif

} IOIF_ADC_Instance_t;

typedef struct {
    uint32_t resolution;
    uint32_t channel_count;
    uint32_t value[IOIF_ADC_MAX_CHANNEL];

} IOIF_ADC_ReportData_t;


AGRBStatusDef ioif_adc_assign_instance(IOIF_ADCx_t* id, ADC_HandleTypeDef* hadc);
AGRBStatusDef ioif_adc_release_instance(IOIF_ADCx_t id);

AGRBStatusDef ioif_adc_get_value(IOIF_ADCx_t id, IOIF_ADC_ReportData_t* report); //value is in raw format (0 ~ 2^resolution - 1)

//AGRBStatusDef ioif_adc_configure_channel(IOIF_ADCx_t id, uint32_t channel, uint32_t rank);


#endif

#endif
