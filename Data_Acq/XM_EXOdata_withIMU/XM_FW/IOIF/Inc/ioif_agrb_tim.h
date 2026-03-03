/**
 ******************************************************************************
 * @file    ioif_agrb_tim.h
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Oct 30, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IOIF_INC_IOIF_AGRB_TIM_H_
#define IOIF_INC_IOIF_AGRB_TIM_H_

#include "ioif_agrb_defs.h"

/* STM32 HAL Headers (MCU별 자동 선택) */
#if defined(IOIF_MCU_SERIES_H7)
    #include "stm32h7xx_hal.h"
    #include "stm32h7xx_hal_tim.h"
    #include "stm32h7xx_hal_tim_ex.h"
#elif defined(IOIF_MCU_SERIES_G4)
    #include "stm32g4xx_hal.h"
	#include "stm32g4xx_hal_tim.h"
	#include "stm32g4xx_hal_tim_ex.h"
#elif defined(IOIF_MCU_SERIES_F4)
    #include "stm32f4xx_hal.h"
	#include "stm32f4xx_hal_tim.h"
	#include "stm32f4xx_hal_tim_ex.h"
#else
    #error "Unsupported MCU series for IOIF TIM"
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define IOIF_TIM_MAX_INSTANCES      (2) // 1ms Main Timer + 여유분
#define IOIF_TIM_NOT_INITIALIZED    (0xFFFFFFFF)

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

typedef uint32_t IOIF_TIMx_t;

/**
 * @brief 타이머 인터럽트 발생 시 호출될 콜백 함수 타입
 */
typedef void (*IOIF_TIM_PeriodElapsedCallback_t)(void);

typedef struct {
    uint32_t                        period_ms;   // (참고용) 설정된 주기
    IOIF_TIM_PeriodElapsedCallback_t callback;   // 주기마다 호출될 함수
} IOIF_TIM_Config_t;

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief  타이머 인스턴스를 할당하고 초기화합니다.
 * @param[out] id      할당된 IOIF 핸들 ID
 * @param[in]  htim    STM32 HAL TIM 핸들
 * @param[in]  config  초기화 설정 (콜백 포함)
 * @return AGRBStatus_OK 성공 시
 */
AGRBStatusDef IOIF_TIM_Assign_Instance(IOIF_TIMx_t* id, TIM_HandleTypeDef* htim, IOIF_TIM_Config_t* config);

/**
 * @brief  타이머 인터럽트(Base_IT)를 시작합니다.
 */
AGRBStatusDef IOIF_TIM_Start_IT(IOIF_TIMx_t id);

/**
 * @brief  타이머 인터럽트를 정지합니다.
 */
AGRBStatusDef IOIF_TIM_Stop_IT(IOIF_TIMx_t id);

/**
 * @brief  시스템 틱(Tick)을 반환합니다. (HAL_GetTick 래퍼)
 */
uint32_t IOIF_TIM_GetTick(void);

/**
 * @brief  Delay 함수 (BareMetal/RTOS 분기 처리됨)
 */
void IOIF_TIM_Delay(uint32_t ms);

#endif /* IOIF_INC_IOIF_AGRB_TIM_H_ */
