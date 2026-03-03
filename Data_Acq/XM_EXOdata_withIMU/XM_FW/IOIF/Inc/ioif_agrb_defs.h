/**
 ******************************************************************************
 * @file    ioif_agrb_defs.h
 * @author  HyundoKim
 * @brief   [IOIF] AGR Common Definitions - IOIF Layer 전용
 * @version 3.0 (IOIF Layer 전용으로 간소화)
 * @date    Dec 5, 2025
 *
 * @details
 * IOIF (I/O Interface) Layer의 공통 정의를 관리합니다.
 * 
 * [적용 범위]
 * - IOIF Layer 전용 (ioif_agrb_*.c, ioif_agrb_*.h)
 * - IOIF 모듈 개발자를 위한 헤더
 * 
 * [System Layer는 사용 금지]
 * - System Layer는 module.h 사용
 * - IOIF는 System 설정에 의존하지 않음 (독립성)
 * 
 * [원칙]
 * - Hardware Abstraction에 필요한 최소한의 정의만 포함
 * - MCU 정보 자동 감지
 * - Status 코드 정의
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IOIF_AGRB_DEFS_H_
#define IOIF_AGRB_DEFS_H_

#include <stdint.h>
#include <stdbool.h>

/**
 *===========================================================================
 * EXECUTION MODE AUTO-DETECTION (SMART)
 *===========================================================================
 * 
 * IOIF는 제품 독립적이므로 module.h를 참조하지 않습니다.
 * 대신 FreeRTOS 헤더의 존재 여부로 환경을 자동 감지합니다.
 * 
 * [감지 전략]
 * 1. Compiler의 __has_include 기능으로 FreeRTOSConfig.h 존재 확인
 * 2. IOIF는 이를 감지하여 USE_FREERTOS_DMA 자동 정의
 * 3. 사용자 개입 불필요 (완전 자동 감지)
 * 
 * [Fallback]
 * - __has_include 미지원 컴파일러: FreeRTOS 매크로 감지
 */

/* --- FreeRTOS 존재 여부 확인 (Compiler Feature) --- */
#if defined(__has_include)
    #if __has_include("FreeRTOSConfig.h")
        #define IOIF_RTOS_DETECTED 1
    #else
        #define IOIF_RTOS_DETECTED 0
        // #warning "RTOS not detected. Defaulting to BareMetal mode."
    #endif
#else
    /* Fallback: FreeRTOS 특징적 매크로로 간접 확인 */
    #if defined(configUSE_PREEMPTION) || defined(INCLUDE_vTaskDelay)
        #define IOIF_RTOS_DETECTED 1
    #else
        #define IOIF_RTOS_DETECTED 0
    #endif
#endif

/* --- 환경별 매크로 정의 --- */
#if IOIF_RTOS_DETECTED
    #ifndef USE_FREERTOS_DMA
        #define USE_FREERTOS_DMA  /**< RTOS 환경 (자동 감지) */
    #endif
    
    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"
    
    /* STM32H7 Memory Section Attributes */
    #if defined(STM32H743xx) || defined(STM32H750xx)
        #define IOIF_FS_SECTION      ".RAM_D2_data"
        #define IOIF_DMA_SECTION     ".RAM_D3_data"
        #define IOIF_USB_CDC_SECTION ".RAM_D3_data"
    #else
        #define IOIF_DMA_SECTION
    #endif
#else
    #ifndef USE_BAREMETAL
        #define USE_BAREMETAL     /**< BareMetal 환경 (자동 감지) */
    #endif
    
    #define IOIF_DMA_SECTION      /**< STM32G4: No special section */
#endif

/**
 *===========================================================================
 * MCU INFORMATION (Auto-detect from Compiler Defines)
 *===========================================================================
 */

#if defined(STM32H743xx) || defined(STM32H750xx)
    #define IOIF_MCU_SERIES_H7          1
    #define IOIF_CPU_FREQ_HZ            480000000UL /**< 480 MHz (XM10, CM, MD) */
    #define IOIF_HAS_DCACHE             1           /**< D-Cache 존재 (MPU로 non-cacheable 설정 필수) */
    
#elif defined(STM32G474xx) || defined(STM32G431xx)
    #define IOIF_MCU_SERIES_G4          1
    #define IOIF_CPU_FREQ_HZ            160000000UL /**< 160 MHz (Sensor Modules) */
    #define IOIF_HAS_DCACHE             0           /**< D-Cache 없음 (캐시 문제 없음) */
    
#elif defined(STM32F407xx)
    #define IOIF_MCU_SERIES_F4          1
    #define IOIF_CPU_FREQ_HZ            168000000UL /**< 168 MHz (Legacy) */
    #define IOIF_HAS_DCACHE             0
    
#else
    #warning "Unknown MCU. Using default configuration."
    #define IOIF_CPU_FREQ_HZ            100000000UL
    #define IOIF_HAS_DCACHE             0
#endif

/**
 *===========================================================================
 * STM32 HAL HEADERS
 *===========================================================================
 * 
 * @note 각 IOIF 모듈(.c 파일)에서 필요한 HAL 헤더만 선택적으로 포함합니다.
 *       예: ioif_agrb_fdcan.c → stm32h7xx_hal_fdcan.h (H7) or stm32g4xx_hal_fdcan.h (G4)
 * 
 * @usage
 *   #if defined(IOIF_MCU_SERIES_H7)
 *       #include "stm32h7xx_hal_uart.h"
 *   #elif defined(IOIF_MCU_SERIES_G4)
 *       #include "stm32g4xx_hal_uart.h"
 *   #endif
 */

/**
 *===========================================================================
 * IOIF HARDWARE MODULES ENABLE/DISABLE
 *===========================================================================
 */

#define AGRB_IOIF_FDCAN_ENABLE          /**< FDCAN (CAN FD) */
#define AGRB_IOIF_UART_ENABLE           /**< UART (RS-232/485) */
#define AGRB_IOIF_GPIO_ENABLE           /**< GPIO (General Purpose I/O) */
#define AGRB_IOIF_TIM_ENABLE            /**< Timer (Hardware Timer) */
#define AGRB_IOIF_DWT_ENABLE            /**< DWT (Performance Profiling) */
#define AGRB_IOIF_USB_ENABLE            /**< USB (CDC, MSC) */
#define AGRB_IOIF_ADC_ENABLE            /**< ADC (Analog-to-Digital Converter) */
#define AGRB_IOIF_FILESYSTEM_ENABLE     /**< FatFs (USB MSC, SD Card) */

// #define AGRB_IOIF_SPI_ENABLE         /**< SPI (비활성화) */
// #define AGRB_IOIF_I2C_ENABLE         /**< I2C (비활성화) */

/**
 *===========================================================================
 * STATUS CODES
 *===========================================================================
 */

 typedef enum {
    /* --- Success --- */
    AGRBStatus_OK = 0,                  /**< 성공 */
    
    /* --- General Errors --- */
    AGRBStatus_ERROR,                   /**< 일반 오류 */
    AGRBStatus_FAILED,                  /**< 실패 */
    
    /* --- Resource Errors --- */
    AGRBStatus_NO_RESOURCE,             /**< 리소스 부족 (Instance Pool Full) */
    AGRBStatus_BUSY,                    /**< 장치 사용 중 */
    AGRBStatus_TIMEOUT,                 /**< 타임아웃 */
    
    /* --- Initialization Errors --- */
    AGRBStatus_NOT_INITIALIZED,         /**< 초기화 안 됨 */
    AGRBStatus_INITIAL_FAILED,          /**< 초기화 실패 */
    
    /* --- Parameter Errors --- */
    AGRBStatus_PARAM_ERROR,             /**< 파라미터 오류 (NULL, 범위 초과) */
    AGRBStatus_NOT_ALLOWED,             /**< 허용 안 됨 */
    AGRBStatus_NOT_SUPPORTED,           /**< 지원 안 됨 */
    
    /* --- Data Errors --- */
    AGRBStatus_BUFFER_OVERFLOW,         /**< 버퍼 오버플로 */
    AGRBStatus_NOT_FOUND,               /**< 찾을 수 없음 */
    
    /* --- Integrity & Security --- */
    AGRBStatus_INTEGRITY_ERROR,         /**< 데이터 무결성 오류 */
    AGRBStatus_SECURITY_ERROR,          /**< 보안 오류 */
    
    /* --- RTOS Specific --- */
    AGRBStatus_SEMAPHORE_ERROR,         /**< 세마포어 오류 (RTOS Only) */
    
} AGRBStatusDef;

/**
 *===========================================================================
 * HELPER MACROS
 *===========================================================================
 */

#define AGRB_IS_OK(status)              ((status) == AGRBStatus_OK)
#define AGRB_IS_ERROR(status)           ((status) != AGRBStatus_OK)

/**
 * @brief Status 코드를 문자열로 변환 (디버깅용)
 */
static inline const char* AGRBStatus_ToString(AGRBStatusDef status)
{
    switch (status) {
        case AGRBStatus_OK:                 return "OK";
        case AGRBStatus_ERROR:              return "ERROR";
        case AGRBStatus_FAILED:             return "FAILED";
        case AGRBStatus_NO_RESOURCE:        return "NO_RESOURCE";
        case AGRBStatus_BUSY:               return "BUSY";
        case AGRBStatus_TIMEOUT:            return "TIMEOUT";
        case AGRBStatus_NOT_INITIALIZED:    return "NOT_INITIALIZED";
        case AGRBStatus_INITIAL_FAILED:     return "INITIAL_FAILED";
        case AGRBStatus_PARAM_ERROR:        return "PARAM_ERROR";
        case AGRBStatus_NOT_ALLOWED:        return "NOT_ALLOWED";
        case AGRBStatus_NOT_SUPPORTED:      return "NOT_SUPPORTED";
        case AGRBStatus_BUFFER_OVERFLOW:    return "BUFFER_OVERFLOW";
        case AGRBStatus_NOT_FOUND:          return "NOT_FOUND";
        case AGRBStatus_INTEGRITY_ERROR:    return "INTEGRITY_ERROR";
        case AGRBStatus_SECURITY_ERROR:     return "SECURITY_ERROR";
        case AGRBStatus_SEMAPHORE_ERROR:    return "SEMAPHORE_ERROR";
        default:                            return "UNKNOWN";
    }
}

#endif /* IOIF_AGRB_DEFS_H_ */
