/**
 ******************************************************************************
 * @file    module.h
 * @author  HyundoKim
 * @brief   [System/Config] XM10 Platform System Layer Configuration
 * @version 3.0 (IMU Hub 스타일 리팩토링)
 * @date    Dec 5, 2025
 *
 * @details
 * System Layer 및 Application Layer에서 공유하는 설정값을 정의합니다.
 * 
 * [적용 범위]
 * - System/Core (core_process.c, system_startup.c)
 * - System/Links (cm_xm_link.c, imu_hub_xm_link.c, pnp_manager.c)
 * - System/Comm (uart_rx_handler.c, canfd_rx_handler.c)
 * - Application (main.c, XM_API)
 * 
 * [IOIF Layer는 사용 금지]
 * - IOIF Layer는 ioif_agrb_defs.h 사용
 * - IOIF는 System Layer 설정에 의존하지 않음 (독립성 유지)
 * 
 * [Device Layer는 사용 금지]
 * - Device Layer는 ioif_agrb_defs.h 사용
 * - Device는 제품 특화 설정에 의존하지 않음 (재사용성 확보)
 * 
 * [원칙]
 * - 실제로 여러 파일에서 공유되는 값만 정의
 * - 하드코딩된 매직 넘버를 여기로 이동
 * - 사용되지 않는 설정은 추가하지 않음
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_CONFIG_MODULE_H_
#define SYSTEM_CONFIG_MODULE_H_

/**
 *===========================================================================
 * PRODUCT IDENTIFICATION
 *===========================================================================
 */

#define XM10_FIRMWARE_VERSION        "3.0.0"
#define XM10_HARDWARE_REVISION       "Rev2.0"

/**
 *===========================================================================
 * EXECUTION ENVIRONMENT (CRITICAL)
 *===========================================================================
 */

/**
 * @brief XM10은 RTOS 전용 플랫폼입니다.
 * @note System Layer 전용 매크로입니다.
 *       Device Layer는 ioif_agrb_defs.h에서 자동 감지합니다.
 */
#define USE_FREERTOS_DMA

/**
 *===========================================================================
 * RTOS TASK CONFIGURATION (USE_FREERTOS_DMA defined)
 *===========================================================================
 */

#ifdef USE_FREERTOS_DMA

/* === 1. Real-Time Data Path (Highest Priority) === */
#define TASK_PRIO_SDO_ROUTER        osPriorityRealtime3  /**< SDO 라우터 */
#define TASK_STACK_SDO_ROUTER       (512)

#define TASK_PRIO_UART_RX           osPriorityRealtime2  /**< UART 데이터 수신 */
#define TASK_STACK_UART_RX          (512)

/* === 2. Non-Real-Time Services (Normal Priority) === */
#define TASK_PRIO_PNP_MANAGER       osPriorityNormal1    /**< 연결 관리 */
#define TASK_STACK_PNP_MANAGER      (512)
#define TASK_PERIOD_MS_PNP_MANAGER  100

#define TASK_PRIO_USB_CONTROL       osPriorityNormal     /**< USB 모드 전환 */
#define TASK_STACK_USB_CONTROL      (1024)
#define TASK_PERIOD_MS_USB_CONTROL  10

/* === 3. Low Priority I/O === */
#define TASK_PRIO_BTN_CONTROL       osPriorityBelowNormal7  /**< 버튼 입력 */
#define TASK_STACK_BTN_CONTROL      (512)

#define TASK_PRIO_USB_SAVE          osPriorityBelowNormal   /**< USB 데이터 저장 */
#define TASK_STACK_USB_SAVE         (4096 * 4)

#endif  /* USE_FREERTOS_DMA */

/**
 *===========================================================================
 * HARDWARE CONFIGURATION (실제 사용)
 *===========================================================================
 */

/* --- Connected Devices --- */
#define XM_MAX_SENSOR_MODULES       7           /**< 최대 센서 모듈 개수 (IMU Hub, EMG Hub, FES Hub, GRF Hub) */

/**
 *===========================================================================
 * TIMING CONFIGURATION (실제 사용)
 *===========================================================================
 */

/* --- Main Loop --- */
#define XM_MAIN_LOOP_FREQ_HZ        500         /**< Main Control Loop 주파수 (500Hz = 2ms) */

/* --- Communication Timeout --- */
#define XM_FDCAN_RX_TIMEOUT_MS      100         /**< FDCAN 수신 타임아웃 */
#define XM_HEARTBEAT_INTERVAL_MS    1000        /**< Heartbeat 전송 주기 */

/**
 *===========================================================================
 * LEGACY DEFINITIONS (TODO: 삭제 예정)
 *===========================================================================
 */

/* --- 하드웨어 종속적인 값 정의 --- */
// TODO: 추후 삭제 유무 결정
// #define BUS_VOLTAGE 24
// #define VBUS2DUTY_RATIO 200

/* --- Device Configuration --- */
// TODO: Device 계층 리팩토링에 따른 추후 삭제 유무 결정
// #define IOIF_LTC2944_ENABLED
// #define IOIF_BM1422AGMV_ENABLED
// #define IOIF_ICM20608G_ENABLED
// #define IOIF_NZRLED_ENABLED
// #define IOIF_RMB30SC_ENABLED
// #define IOIF_THINPOT_LS_ENABLED
// #define IOIF_TB67H450FNGEL_ENABLED
// #define IOIF_BUZZER_ENABLED
// #define IOIF_BATTERYLED_ENABLED
// #define IOIF_GRFSENSOR_ENABLED

#endif /* SYSTEM_CONFIG_MODULE_H_ */
