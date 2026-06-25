/**
 ******************************************************************************
 * @file    module.h
 * @author  HyundoKim
 * @brief   [System/Config] IMU Hub System Layer Configuration
 * @version 3.0 (실용적 재설계)
 * @date    Dec 3, 2025
 *
 * @details
 * System Layer 및 Application Layer에서 공유하는 설정값을 정의합니다.
 * 
 * [적용 범위]
 * - System/Core (system_startup.c)
 * - System/Links (xm_imu_hub_link.c, imu_sensors_link.c, pnp_manager.c)
 * - System/Comm (uart_rx_handler.c, canfd_rx_handler.c)
 * - Application (imu_hub_apps.c)
 * 
 * [IOIF Layer는 사용 금지]
 * - IOIF Layer는 ioif_agrb_defs.h 사용
 * - IOIF는 System Layer 설정에 의존하지 않음 (독립성 유지)
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

#define IMU_HUB_FIRMWARE_VERSION        "1.0.0"
#define IMU_HUB_HARDWARE_REVISION       "Rev1.0"

/**
 *===========================================================================
 * HARDWARE CONFIGURATION (실제 사용)
 *===========================================================================
 */

/* --- IMU Channels --- */
#define MAX_IMU_CHANNELS                6           /**< 최대 IMU 채널 수 (물리적 UART 포트 개수) */

/* --- Sensor Device Instances (Device Layer Multi-Instance 설정) ---
 * module.h에서 정의하면 Device 헤더의 #ifndef 기본값을 오버라이드합니다.
 * IMU Hub: 6개 채널에 EBIMU 또는 XSENS 혼합 연결 가능
 */
#define XSENS_MAX_INSTANCES             MAX_IMU_CHANNELS  /**< 최대 XSENS 인스턴스 (채널당 1개) */
#define EBIMU_MAX_INSTANCES             MAX_IMU_CHANNELS  /**< 최대 EBIMU 인스턴스 (채널당 1개) */

/**
 * @brief IMU 센서 타입 (Auto-Detect 지원)
 * @details 
 * Auto-Detect: 부팅 시 UNKNOWN으로 시작, ISR에서 Dual-Parser Probing으로 자동 감지
 * EBIMU + XSENS 혼합 구성을 지원합니다.
 */
typedef enum {
    IMU_SENSOR_UNKNOWN = 0,  /**< 미감지 (Dual-Parser Probing 진행 중) */
    IMU_SENSOR_EBIMU   = 1,  /**< E2BOX EBIMU-9DOFV6 (Binary Protocol, 1kHz) */
    IMU_SENSOR_XSENS   = 2,  /**< Xsens MTi-630 (Xbus Protocol, 1kHz) */
} ImuSensorType_t;

/**
 *===========================================================================
 * TIMING CONFIGURATION (실제 사용)
 *===========================================================================
 */

/* --- Main Loop --- */
#define MAIN_LOOP_PERIOD_MS             1           /**< Main Loop 주기 (imu_hub_apps.c) */

/* --- PnP Manager --- */
#define PNP_RUN_PERIOD_MS               500         /**< PnP Manager 실행 주기: 10ms → 500ms (CPU 부하 감소 및 Tx FIFO Full 방지) */

/* --- Sensor Reset Timing --- */
#define SENSOR_DISCHARGE_TIME_MS        1000        /**< 센서 방전 시간 (system_startup.c) */
#define SENSOR_STABILIZATION_TIME_MS    1000        /**< 센서 부팅 안정화 대기 시간 (system_startup.c) */

/**
 *===========================================================================
 * COMPILE-TIME CHECKS
 *===========================================================================
 */

#if MAX_IMU_CHANNELS > 6
    #error "MAX_IMU_CHANNELS cannot exceed 6 (Hardware limitation)"
#endif

#if MAIN_LOOP_PERIOD_MS != 1
    #warning "Main Loop period is not 1ms. Real-time performance may vary."
#endif

#endif /* SYSTEM_CONFIG_MODULE_H_ */
