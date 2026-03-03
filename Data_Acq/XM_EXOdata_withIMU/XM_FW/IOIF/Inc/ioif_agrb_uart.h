/**
 ******************************************************************************
 * @file    ioif_agrb_uart.h
 * @author  HyundoKim
 * @brief   [IOIF Layer] UART 하드웨어 추상화 계층 헤더
 * @details STM32 HAL UART를 감싸서 DMA 기반의 비동기 수신 및 동기 송신을 지원합니다.
 * @version 0.1
 * @date    Nov 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_defs.h"
#if defined(AGRB_IOIF_UART_ENABLE)

#pragma once

#ifndef IOIF_INC_IOIF_AGRB_UART_H_
#define IOIF_INC_IOIF_AGRB_UART_H_

#include <stdint.h>
#include <stdbool.h>

/* STM32 HAL Headers (MCU별 자동 선택) */
#if defined(IOIF_MCU_SERIES_H7)
    #include "stm32h743xx.h"
    #include "stm32h7xx_hal.h"  /* 전체 HAL (UART, DMA, 매크로 등 포함) */
    #include "stm32h7xx_hal_uart.h"
#elif defined(IOIF_MCU_SERIES_G4)
    #include "stm32g4xx.h"
    #include "stm32g4xx_hal.h"  /* 전체 HAL (UART, DMA, 매크로 등 포함) */
    #include "stm32g4xx_hal_uart.h"
#elif defined(IOIF_MCU_SERIES_F4)
    #include "stm32f4xx.h"
    #include "stm32f4xx_hal.h"  /* 전체 HAL (UART, DMA, 매크로 등 포함) */
    #include "stm32f4xx_hal_uart.h"
#else
    #error "Unsupported MCU series for IOIF UART"
#endif

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define IOIF_UART_MAX_INSTANCES      (6)
#define IOIF_UART_ID_NOT_ALLOCATED   (0xFFFFFFFF)

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

typedef uint32_t IOIF_UARTx_t;

typedef enum {
    IOIF_UART_Baudrate_9600,
    IOIF_UART_Baudrate_19200,
    IOIF_UART_Baudrate_38400,
    IOIF_UART_Baudrate_57600,
    IOIF_UART_Baudrate_115200,
    IOIF_UART_Baudrate_230400,
    IOIF_UART_Baudrate_460800,
    IOIF_UART_Baudrate_921600,
    // 필요시 추가
} IOIF_UART_Baudrate_e;

/**
 * @brief UART 수신 방식을 선택하는 열거형
 */
typedef enum {
    IOIF_UART_MODE_POLLING_TASK, // 1ms 태스크가 DMA 링버퍼를 폴링
    IOIF_UART_MODE_IDLE_EVENT,   // IDLE 라인 인터럽트 기반
} IOIF_UART_RxMode_e;

// [RX 콜백] DMA 수신 데이터가 처리된 후 System Layer로 전달되는 콜백
typedef void (*IOIF_UART_RxEventCallback_t)(uint8_t* rx_buf, uint32_t size, uint32_t id);

typedef struct {
    IOIF_UART_Baudrate_e baudrate;
    IOIF_UART_RxMode_e rxMode; // 수신 모드 선택
    uint32_t bounce_buffer_size; // (POLLING_TASK 모드 전용)
    IOIF_UART_RxEventCallback_t rx_event_callback;
} IOIF_UART_Config_t;

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES(extern)
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief UART 인스턴스를 할당하고 초기화합니다.
 * @param[out] id       할당된 IOIF 핸들 ID
 * @param[in]  huart    STM32 HAL UART 핸들
 * @param[in]  config   초기화 설정 구조체
 * @return AGRBStatusDef
 */
AGRBStatusDef IOIF_UART_AssignInstance(IOIF_UARTx_t* id, UART_HandleTypeDef* huart, IOIF_UART_Config_t* config);

/**
 * @brief UART 수신을 시작합니다. (DMA Circular Mode)
 */
AGRBStatusDef IOIF_UART_Start(IOIF_UARTx_t id);

/**
 * @brief RX IDLE Event 콜백 등록 및 수신 시작
 * @param id UART Instance ID
 * @param callback RX IDLE Event 발생 시 호출될 콜백 함수
 * @return AGRBStatus_OK=성공, AGRBStatus_ERROR=실패
 * 
 * @note 
 * - IDLE Event 모드 전용
 * - Circular DMA 모드로 수신 시작 (IDLE/HT/TC 이벤트 처리)
 * - RTOS/BareMetal 공통 사용 가능
 */
AGRBStatusDef IOIF_UART_SetRxIdleCallback(IOIF_UARTx_t id, IOIF_UART_RxEventCallback_t callback);

/**
 * @brief UART 데이터 전송 (Polling 모드, Blocking)
 * @param id UART Instance ID
 * @param tx_buf 전송할 데이터 버퍼
 * @param size 전송할 데이터 크기 (bytes)
 * @return AGRBStatus_OK=성공, AGRBStatus_TIMEOUT=타임아웃
 * 
 * @note 
 * - RTOS: Semaphore로 동기화
 * - BareMetal: Busy Check
 * - Polling 방식이므로 전송 완료까지 대기
 * - RTOS/BareMetal 공통 사용 가능
 */
AGRBStatusDef IOIF_UART_Write_Polling(IOIF_UARTx_t id, const uint8_t* tx_buf, uint32_t size);

/**
 * @brief UART 데이터 전송 (DMA 모드, Non-Blocking)
 * @param id UART Instance ID
 * @param tx_buf 전송할 데이터 버퍼
 * @param size 전송할 데이터 크기 (bytes, 최대 256)
 * @return AGRBStatus_OK=성공, AGRBStatus_TIMEOUT=타임아웃, AGRBStatus_BUSY=사용 중
 * 
 * @note 
 * - RTOS/BareMetal 공통 사용 가능 (확장성 고려)
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
AGRBStatusDef IOIF_UART_Write_DMA(IOIF_UARTx_t id, const uint8_t* tx_buf, uint32_t size);

/**
 * @brief UART 버퍼를 비웁니다.
 */
AGRBStatusDef IOIF_UART_Flush(IOIF_UARTx_t id);

/**
 * @brief [디버깅] UART 상태 정보 구조체
 */
typedef struct {
    uint32_t rx_state;          // UART RxState (0x20=READY, 0x22=BUSY_RX)
    uint32_t error_code;        // UART ErrorCode (0=정상)
    uint32_t isr;               // UART ISR 레지스터
    uint32_t dma_cndtr;         // DMA CNDTR (남은 데이터 개수)
    uint32_t dma_ccr;           // DMA CCR (DMA 설정)
    bool     idle_ie_enabled;   // IDLE 인터럽트 활성화 여부
    bool     dmar_enabled;      // DMA RX 활성화 여부
    uint32_t idle_callback_cnt; // IDLE 콜백 호출 횟수
    uint32_t error_callback_cnt;// 에러 콜백 호출 횟수
} IOIF_UART_DebugInfo_t;

/**
 * @brief [디버깅] UART 상태 정보 가져오기
 * @param id UART 인스턴스 ID
 * @param info 상태 정보를 담을 구조체 포인터
 * @return AGRBStatusDef
 */
AGRBStatusDef IOIF_UART_GetDebugInfo(IOIF_UARTx_t id, IOIF_UART_DebugInfo_t* info);

/**
 * @brief [범용] UART 상태를 완전히 리셋
 * @details 
 * 특정 UART의 상태를 완전히 클리어하고 재시작합니다.
 * 
 * [동작]
 * 1. HAL_UART_Abort() - 진행 중인 동작 중단
 * 2. UART 상태 강제 초기화 (RxState, ErrorCode)
 * 3. 에러 플래그 클리어 (ORE, NE, FE, PE)
 * 4. DMA on RX Error를 ENABLE로 설정 (에러 무시, 계속 동작)
 * 5. DMA 상태 리셋
 * 
 * [사용 시나리오]
 * - 센서 전원 리셋 후 UART 재초기화
 * - 통신 에러 복구
 * 
 * @param huart UART 핸들 포인터
 * @return AGRBStatusDef
 */
AGRBStatusDef IOIF_UART_ResetState(UART_HandleTypeDef* huart);

/* 
 * [제거됨] IOIF_UART_GetRecommendedDischargeTime_ms()
 * - System Layer가 module.h의 SENSOR_DISCHARGE_TIME_MS 사용
 * - IOIF는 제품별 설정을 반환해서는 안 됨 (아키텍처 위반)
 */

#endif // AGRB_IOIF_UART_ENABLE
#endif /* IOIF_INC_IOIF_AGRB_UART_H_ */
