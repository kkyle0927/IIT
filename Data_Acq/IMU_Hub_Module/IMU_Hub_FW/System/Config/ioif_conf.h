/**
 ******************************************************************************
 * @file    ioif_conf.h
 * @author  HyundoKim
 * @brief   [System/Config] IMU Hub IOIF Module Configuration
 * @version 1.0
 * @date    Feb 11, 2026
 *
 * @details
 * IOIF Submodule (AGR-EXO/IOIF)에서 사용할 모듈을 활성화합니다.
 * STM32 HAL의 stm32xx_hal_conf.h 패턴과 동일한 방식입니다.
 * 
 * ioif_agrb_defs.h에서 이 파일을 #include 합니다.
 * 프로젝트 include 경로(System/Config/)에 위치해야 합니다.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IOIF_CONF_H_
#define IOIF_CONF_H_

/**
 *===========================================================================
 * IOIF Module Enable/Disable (IMU Hub Module - STM32G474)
 *===========================================================================
 * 사용할 모듈만 #define으로 활성화합니다.
 * 비활성 모듈은 빈 번역 단위로 컴파일됩니다 (오버헤드 없음).
 */

/* ===== Production Modules (현재 사용 중) ===== */
#define AGRB_IOIF_FDCAN_ENABLE              /**< FDCAN (CAN FD) - DOP/PnP 통신 */
#define AGRB_IOIF_UART_ENABLE               /**< UART - IMU 센서 6채널 + 디버그 */
#define AGRB_IOIF_GPIO_ENABLE               /**< GPIO - IMU Power, LED */
#define AGRB_IOIF_TIM_ENABLE                /**< Timer - 시스템 타이머 */
#define AGRB_IOIF_DWT_ENABLE                /**< DWT - 고정밀 성능 측정 */

/* ===== Optional Modules (필요시 활성화) ===== */
// #define AGRB_IOIF_SPI_ENABLE             /**< SPI (미사용) */
// #define AGRB_IOIF_I2C_ENABLE             /**< I2C (미사용) */
// #define AGRB_IOIF_DMA_ENABLE             /**< DMA Pool Manager - SPI/I2C 사용 시 필요 */
// #define AGRB_IOIF_USB_ENABLE             /**< USB (IMU Hub에 없음) */
// #define AGRB_IOIF_USB_MODE_CDC_ONLY      /**< USB 활성화 시 모드 선택 — G474 는 CDC_ONLY 만 지원 */
// #define AGRB_IOIF_ADC_ENABLE             /**< ADC (미사용) */
// #define AGRB_IOIF_FILESYSTEM_ENABLE      /**< FatFs (미사용) */
/* PSRAM: SM-IMU HW 미탑재 + G4 D-Cache 없음 — CMakeLists 에서도 ioif_agrb_psram.c 제외 (Rule 12) */

/**
 *===========================================================================
 * Module-specific Overrides
 *===========================================================================
 */

/* FDCAN SW Tx Queue depth — IMU 는 TPDO1/TPDO2 + HB/SDO/EMCY 동시 적체 여유
 * 확보를 위해 기본 16 → 32 로 증량. RAM 비용 +1152B (G4 SRAM1 1.4%).
 * Rationale: SM CAN-FD FIFO Full 전략 (memory: project_sm_canfd_fifo_full). */
#define IOIF_FDCAN_SW_TX_QUEUE_SIZE         32

#endif /* IOIF_CONF_H_ */
