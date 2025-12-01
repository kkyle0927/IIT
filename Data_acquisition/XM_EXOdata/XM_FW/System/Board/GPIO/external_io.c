/**
 ******************************************************************************
 * @file    external_io.c
 * @author  HyundoKim
 * @brief   [System Layer] 외부 확장 IO 핀 관리 구현부
 * @version 0.1
 * @date    Nov 12, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "external_io.h"
#include <string.h> // for NULL

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

// system_startup에서 주입받을 IOIF 핸들
static IOIF_GPIOx_t s_dio_ids[EXT_DIO_COUNT];
static IOIF_ADCx_t  s_adc1_id = IOIF_ADC_NOT_INITIALIZED;
static IOIF_ADCx_t  s_adc2_id = IOIF_ADC_NOT_INITIALIZED;

// ADC 값을 한 번에 읽어오기 위한 공유 버퍼
static IOIF_ADC_ReportData_t s_adc1_report;
static IOIF_ADC_ReportData_t s_adc2_report;
// ADC 값의 유효 시간 (예: 2ms)을 위한 타임스탬프 (선택적 최적화)
// static uint32_t s_adc_last_read_time = 0;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

void ExternalIO_Init(IOIF_GPIOx_t dio_ids[EXT_DIO_COUNT], IOIF_ADCx_t adc1_id, IOIF_ADCx_t adc2_id)
{
    // [수정] adc1_id와 adc2_id를 모두 검사합니다.
    if (dio_ids == NULL || 
        adc1_id == IOIF_ADC_NOT_INITIALIZED || 
        adc2_id == IOIF_ADC_NOT_INITIALIZED) 
    {
        // 치명적 오류 처리
        return;
    }

    memcpy(s_dio_ids, dio_ids, sizeof(s_dio_ids));
    s_adc1_id = adc1_id;
    s_adc2_id = adc2_id;

    // 모든 DIO 핀을 기본 'Input Floating' 상태로 초기화
    // (system_startup에서 이미 수행했으므로 생략 가능, 또는 여기서 재확인)
    IOIF_GPIO_Initialize_t default_config = {
        .mode = IOIF_GPIO_Mode_Input,
        .pull = IOIF_GPIO_Floating,
        .init_state = false
    };
    
    for (int i = 0; i < EXT_DIO_COUNT; i++) {
        IOIF_GPIO_REINITIALIZE(s_dio_ids[i], &default_config);
    }
}

void ExternalIO_SetPinMode(ExternalDioPin_t pin, ExternalPinMode_t mode)
{
    if (pin >= EXT_DIO_COUNT) return;

    IOIF_GPIO_Initialize_t config;
    
    switch(mode)
    {
        case EXT_IO_MODE_INPUT:
            config.mode = IOIF_GPIO_Mode_Input;
            config.pull = IOIF_GPIO_Floating;
            break;
        case EXT_IO_MODE_INPUT_PULLUP:
            config.mode = IOIF_GPIO_Mode_Input;
            config.pull = IOIF_GPIO_PullUp;
            break;
        case EXT_IO_MODE_INPUT_PULLDOWN:
            config.mode = IOIF_GPIO_Mode_Input;
            config.pull = IOIF_GPIO_PullDown;
            break;
        case EXT_IO_MODE_OUTPUT:
            config.mode = IOIF_GPIO_Mode_Output;
            config.pull = IOIF_GPIO_Floating;
            config.init_state = false; // 기본 LOW 출력
            break;
        default:
            return;
    }

    IOIF_GPIO_REINITIALIZE(s_dio_ids[pin], &config);
}

void ExternalIO_WritePin(ExternalDioPin_t pin, bool state)
{
    if (pin >= EXT_DIO_COUNT) return;
    
    if (state) {
        IOIF_GPIO_SET(s_dio_ids[pin]);
    } else {
        IOIF_GPIO_RESET(s_dio_ids[pin]);
    }
}

bool ExternalIO_ReadPin(ExternalDioPin_t pin)
{
    if (pin >= EXT_DIO_COUNT) return false;
    
    bool state = false;
    IOIF_GPIO_GET_STATE(s_dio_ids[pin], &state);
    return state;
}

uint16_t ExternalIO_ReadAdc(ExternalAdcPin_t pin)
{
    if (pin >= EXT_ADC_COUNT) return 0;
    
    AGRBStatusDef status;

    if (pin == EXT_ADC_1 || pin == EXT_ADC_3)
    {
        // A0(PA0), A2(PA1)은 ADC1 그룹에 속함
        if (s_adc1_id == IOIF_ADC_NOT_INITIALIZED) return 0;
        
        status = IOIF_ADC_GET_VALUE(s_adc1_id, &s_adc1_report);
        if (status != AGRBStatus_OK) return 0;

        // CubeMX Rank 순서: CC1(idx 0), CC2(idx 1), A0(idx 2), A2(idx 3)
        return (pin == EXT_ADC_1) ? (uint16_t)s_adc1_report.value[2] : (uint16_t)s_adc1_report.value[3];
    }
    else // (pin == EXT_ADC_2 || pin == EXT_ADC_4)
    {
        // A1(PA0_C), A3(PA1_C)는 ADC2 그룹에 속함
        if (s_adc2_id == IOIF_ADC_NOT_INITIALIZED) return 0;
        
        status = IOIF_ADC_GET_VALUE(s_adc2_id, &s_adc2_report);
        if (status != AGRBStatus_OK) return 0;

        // CubeMX Rank 순서: A1(idx 0), A3(idx 1)
        return (pin == EXT_ADC_2) ? (uint16_t)s_adc2_report.value[0] : (uint16_t)s_adc2_report.value[1];
    }
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */
