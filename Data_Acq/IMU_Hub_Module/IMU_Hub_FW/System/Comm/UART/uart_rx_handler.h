/**
 ******************************************************************************
 * @file    uart_rx_handler.h
 * @author  HyundoKim
 * @brief   [System/Comm] UART 다채널 수신 핸들러 (Auto-Detect + DataLake)
 * @version 5.0 (Multi-Instance Auto-Detect)
 * @date    Feb 11, 2026
 *
 * @details
 * [V5.0 변경사항 - Dual-Parser Auto-Detect]
 * - sensor_types[] 하드코딩 제거 → 모든 채널 UNKNOWN으로 시작
 * - ISR에서 Dual-Parser Probing: EBIMU + XSENS 파서 동시 시도
 * - 첫 유효 패킷 파싱 시 센서 타입 확정 (Lock)
 * - 확정 후 Device Layer DataLake API로 데이터 전달
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_COMM_UART_UART_RX_HANDLER_H_
#define SYSTEM_COMM_UART_UART_RX_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "module.h"  /* ImuSensorType_t, MAX_IMU_CHANNELS, *_MAX_INSTANCES (Device 헤더보다 먼저!) */
#include "ioif_agrb_uart.h"
#include "ebimu-9dofv6.h"
#include "mti-630.h"

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief UART Rx Handler 초기화 (Auto-Detect 모드)
 * @param uart_ids UART ID 배열 (MAX_IMU_CHANNELS개)
 * @param count 채널 수
 * @details 
 * 1. 모든 채널을 UNKNOWN으로 초기화
 * 2. 양쪽 파서(EBIMU + XSENS) 모두 초기화
 * 3. UART Rx Idle 콜백 등록
 * 4. Auto-Sense StateInit 호출 (Device Layer DataLake)
 * 
 * @note sensor_types[] 하드코딩 제거. ISR에서 자동 감지합니다.
 */
void UartRxHandler_Init(const IOIF_UARTx_t* uart_ids, uint8_t count);

/**
 * @brief 모든 IMU 데이터 가져오기 (Timer ISR에서 호출)
 * @param out_data IMU 데이터 배열 (MAX_IMU_CHANNELS개, EBIMU_Data_t 형식)
 * @details 
 * 채널별 감지된 센서 타입에 따라 Device DataLake에서 데이터를 읽습니다.
 * - EBIMU: EbimuV6_GetLatest(ch) → 직접 복사
 * - XSENS: XsensMTi_GetLatest(ch) → EBIMU_Data_t 변환 후 복사
 * - UNKNOWN: 0으로 채움
 *
 * ISR Priority 패턴으로 안전하게 데이터를 읽습니다.
 * Timer ISR(Prio 0) > UART ISR(Prio 1) → volatile 직접 읽기 안전
 */
void UartRxHandler_FetchAllImus(EBIMU_Data_t* out_data);

/**
 * @brief 특정 채널의 감지된 센서 타입 반환
 * @param ch 채널 인덱스 (0 ~ MAX_IMU_CHANNELS-1)
 * @return IMU_SENSOR_UNKNOWN / IMU_SENSOR_EBIMU / IMU_SENSOR_XSENS
 */
ImuSensorType_t UartRxHandler_GetSensorType(uint8_t ch);

/**
 * @brief 감지된 센서에 대해 ConfigureOutput 호출 (초기화 시 1회)
 * @details 
 * Auto-Detect 확정 후 또는 사전 설정이 필요한 경우 호출합니다.
 * 특정 타입으로 모든 채널을 일괄 설정합니다.
 * @param type 설정할 센서 타입 (IMU_SENSOR_EBIMU 또는 IMU_SENSOR_XSENS)
 */
void UartRxHandler_ConfigureAllAs(ImuSensorType_t type);

/**
 * @brief [R2] Hot-Swap 재감지 주기 처리
 * @details Lock 된 채널이 offline(Auto-Sense STOPPED) 전환되면 UNKNOWN 으로 리셋 +
 *   파서 reinit → 재연결/다른 센서 교체 시 ISR 이 재부팅 없이 재감지.
 * @note imu_sys_service 의 Auto-Sense 주기 블록에서 EbimuV6/XsensMTi_RunPeriodic 호출
 *   직후(IsOnline 최신화 후) 호출.
 */
void UartRxHandler_RunPeriodic(void);

#endif /* SYSTEM_COMM_UART_UART_RX_HANDLER_H_ */
