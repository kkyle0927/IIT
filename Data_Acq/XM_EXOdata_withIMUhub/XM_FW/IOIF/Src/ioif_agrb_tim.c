/**
 ******************************************************************************
 * @file    ioif_agrb_tim.c
 * @author  HyundoKim
 * @brief   [IOIF Layer] TIMER 하드웨어 추상화 계층 구현
 * @details HAL TIM Base를 감싸서 인터럽트 기반의 주기적 콜백을 지원합니다.
 * @version 0.1
 * @date    Oct 30, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_tim.h"
#include <string.h>

#if defined(USE_FREERTOS_DMA)
#include "cmsis_os2.h" // RTOS
#endif

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

typedef struct {
    bool                is_assigned;
    TIM_HandleTypeDef*  htim;
    IOIF_TIM_Config_t   config;
} IOIF_TIM_Instance_t;

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

static IOIF_TIM_Instance_t s_tim_instances[IOIF_TIM_MAX_INSTANCES];
static uint32_t s_tim_instance_count = 0;

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
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/* No private functions (모든 로직이 HAL_TIM_PeriodElapsedCallback 내부에 인라인) */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

AGRBStatusDef IOIF_TIM_Assign_Instance(IOIF_TIMx_t* id, TIM_HandleTypeDef* htim, IOIF_TIM_Config_t* config)
{
    if (id == NULL || htim == NULL || config == NULL) {
        return AGRBStatus_PARAM_ERROR;
    }

    if (s_tim_instance_count >= IOIF_TIM_MAX_INSTANCES) {
        return AGRBStatus_NO_RESOURCE;
    }

    uint32_t new_id = s_tim_instance_count;
    IOIF_TIM_Instance_t* instance = &s_tim_instances[new_id];

    instance->is_assigned = true;
    instance->htim = htim;
    memcpy(&instance->config, config, sizeof(IOIF_TIM_Config_t));

    *id = new_id;
    s_tim_instance_count++;

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_TIM_Start_IT(IOIF_TIMx_t id)
{
    if (id >= s_tim_instance_count || !s_tim_instances[id].is_assigned) {
        return AGRBStatus_PARAM_ERROR;
    }

    if (HAL_TIM_Base_Start_IT(s_tim_instances[id].htim) != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_TIM_Stop_IT(IOIF_TIMx_t id)
{
    if (id >= s_tim_instance_count || !s_tim_instances[id].is_assigned) {
        return AGRBStatus_PARAM_ERROR;
    }

    if (HAL_TIM_Base_Stop_IT(s_tim_instances[id].htim) != HAL_OK) {
        return AGRBStatus_ERROR;
    }

    return AGRBStatus_OK;
}

uint32_t IOIF_TIM_GetTick(void)
{
    return HAL_GetTick();
}

void IOIF_TIM_Delay(uint32_t ms)
{
#if defined(USE_FREERTOS_DMA)
    osDelay(ms);
#else
    HAL_Delay(ms);
#endif
}

/**
 *------------------------------------------------------------
 * HAL CALLBACK FUNCTIONS
 *------------------------------------------------------------
 */
#if USE_BAREMETAL
/**
 * @brief  [HAL Callback] 타이머 인터럽트 발생 시 호출됨 (ISR Context)
 * @details
 * 1. 인터럽트를 발생시킨 타이머가 IOIF에 등록된 녀석인지 확인합니다.
 * 2. 등록된 타이머라면 -> 사용자가 등록한 함수(Core_RunLoop 등)를 실행합니다.
 * 3. 등록되지 않은 타이머라면 -> HAL의 Timebase(SysTick 대용)로 간주하고 Tick을 증가시킵니다.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    bool is_registered_instance = false;

    // 1. 등록된 인스턴스인지 검색
    for (uint32_t i = 0; i < s_tim_instance_count; i++) {
        if (s_tim_instances[i].is_assigned && s_tim_instances[i].htim == htim) {
            
            // 등록된 콜백 함수 실행 (여기가 Core_RunLoop가 실행될 위치입니다!)
            if (s_tim_instances[i].config.callback) {
                s_tim_instances[i].config.callback();
            }
            
            is_registered_instance = true;
            break; // 처리 완료
        }
    }

    // 2. IOIF에 등록되지 않은 타이머라면 HAL Timebase로 처리
    // (CubeMX에서 Timebase Source로 설정한 타이머가 여기 해당됩니다)
    if (is_registered_instance == false) {
        HAL_IncTick();
    }
}
#endif

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* 
 * _get_instance_by_handle 함수는 제거됨
 * 
 * 이유:
 * 1. 사용하는 곳이 HAL_TIM_PeriodElapsedCallback 한 곳뿐
 * 2. ISR에서 함수 호출 오버헤드 제거 (성능 최적화)
 * 3. 직접 for 루프가 더 효율적
 */
