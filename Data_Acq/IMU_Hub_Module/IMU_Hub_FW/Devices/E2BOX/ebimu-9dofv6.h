/**
 ******************************************************************************
 * @file    ebimu-9dofv6.h
 * @author  HyundoKim
 * @brief   [Devices Layer] E2BOX EBIMU-9DOFV6 센서 드라이버 헤더
 * @details
 * - IOIF 의존 싱글톤 인터페이스 (AGR 아키텍처 표준)
 * - 파싱: Multi-instance (파서 객체 외부 관리, ISR-safe)
 * - 설정: ConfigureOutput, Cmd_* (IOIF UART 사용)
 * - IMU Hub Module / XM 공통 사용 가능 (Device + IOIF 복사 이식)
 * 
 * [의존성] Device → IOIF (00-core-architecture.mdc 준수)
 * 
 * @version 3.0 (Multi-Instance Auto-Sense + DataLake)
 * @date    Feb 11, 2026
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef DEVICES_E2BOX_EBIMU_9DOFV6_H_
#define DEVICES_E2BOX_EBIMU_9DOFV6_H_

#include <stdint.h>
#include <stdbool.h>
#include "ioif_agrb_uart.h"  /* IOIF_UARTx_t, IOIF_UART_Write_Polling */

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define EBIMU_MAX_PACKET_SIZE   (64)    // 패킷 최대 길이
#define EBIMU_HEADER_BYTE       (0x55)  // Binary Mode Header (S.O.F) — 프레임 = 0x55 0x55 + 20B payload

/**
 * @brief 최대 EBIMU 인스턴스 수 (module.h에서 오버라이드 가능)
 * @details XM: 1 (사용 시), IMU Hub: 6 — module.h에서 #define 후 이 헤더 include
 */
#ifndef EBIMU_MAX_INSTANCES
#define EBIMU_MAX_INSTANCES     (1)
#endif

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief EBIMU 파싱 데이터 구조체 (단일 센서용)
 */
typedef struct {
    uint32_t timestamp;  // 수신 시각 (ms) - caller가 설정
    uint8_t  imu_index;  // 센서 인덱스 (0 ~ 5) - caller가 설정

    // Orientation (Quaternion)
    float q_w, q_x, q_y, q_z;

    // Euler (Roll, Pitch, Yaw)
    float roll, pitch, yaw;

    // Acceleration (g -> m/s^2 변환 필요 시 상위에서 처리)
    float acc_x, acc_y, acc_z;

    // Gyroscope (deg/s)
    float gyr_x, gyr_y, gyr_z;
    
    // Magnetometer (uT)
    float mag_x, mag_y, mag_z;

} EBIMU_Data_t;

typedef enum {
    EBIMU_STATE_WAIT_HEADER_1, // First 0x55
    EBIMU_STATE_WAIT_HEADER_2, // Second 0x55
    EBIMU_STATE_READ_PAYLOAD
} EbimuParseState_t;

typedef struct {
    EbimuParseState_t state;
    uint8_t           buffer[64];
    uint8_t           buf_idx;
    uint8_t           payload_len;
} EbimuParser_t;

/**
 * @brief 데이터 수신 시 호출될 콜백 함수 타입
 * @param data 파싱 완료된 데이터 패킷 포인터
 */
typedef void (*EbimuDataCallback_t)(const EBIMU_Data_t* data);

/**
 * @brief EBIMU 드라이버 인터페이스 (싱글톤)
 * @details
 * [파싱] Multi-instance: InitParser, ParseByte, IsConnected
 *   - IMU Hub: 6개 파서 인스턴스 동시 관리
 *   - timestamp, imu_index는 caller(System Layer)가 설정
 * 
 * [설정] IOIF 의존: ConfigureOutput, Cmd_*
 *   - Binary 1kHz 모드 설정
 *   - 재시도 로직 내장 (5회)
 *   - 초기화 시 1회 호출
 */
typedef struct {
    /* ===== 파싱 (Multi-instance, ISR-safe) ===== */
    
    /**
     * @brief 파서 초기화
     */
    void (*InitParser)(EbimuParser_t* parser);

    /**
     * @brief 바이트 단위 파싱 (ISR에서 호출)
     * @param parser 파서 객체 포인터
     * @param byte   수신된 1바이트
     * @param output 파싱 완료 시 데이터를 담을 구조체
     * @return true: 패킷 완성됨
     * @note timestamp, imu_index는 caller가 설정해야 함
     */
    bool (*ParseByte)(EbimuParser_t* parser, uint8_t byte, EBIMU_Data_t* output);
    
    /**
     * @brief 연결 상태 확인 (타임아웃 기반)
     */
    bool (*IsConnected)(uint8_t imu_id, uint32_t last_rx_time, uint32_t current_time);

    /* ===== 센서 설정 (IOIF 의존, 초기화 시 1회 호출) ===== */
    
    /**
     * @brief 센서를 Binary 1kHz Quat+Gyro+Acc 모드로 설정
     * @param uart_id IOIF UART 인스턴스 ID
     */
    void (*ConfigureOutput)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 출력 포맷 설정 (Binary + Quat + Gyro + Acc)
     */
    void (*Cmd_SetOutputFormat)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 출력 주기 1kHz 설정
     */
    void (*Cmd_SetOutputRate_1kHz)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 자이로 캘리브레이션
     */
    void (*Cmd_CalibGyro)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 가속도계 간이 캘리브레이션
     */
    void (*Cmd_CalibAccelSimple)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 센서 리셋
     */
    void (*Cmd_Reset)(IOIF_UARTx_t uart_id);
    
    /**
     * @brief 공장 초기화
     */
    void (*Cmd_FactoryReset)(IOIF_UARTx_t uart_id);

} EBIMU_Driver_t;

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES(extern)
 *-----------------------------------------------------------
 */

// 싱글톤 인스턴스
extern EBIMU_Driver_t ebimu9dofv6;

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief [디버깅] 센서 설정 명령 전송 통계 가져오기
 * @param sent 성공 전송 횟수 출력
 * @param failed 실패 전송 횟수 출력
 */
void EBIMU_GetConfigStats(volatile uint32_t* sent, volatile uint32_t* failed);

/* ================================================================
 * Auto-Sense + DataLake API (Multi-Instance)
 * ================================================================
 * - Auto-Sense: 데이터 타임아웃 기반 연결 감지 (PnP Task에서 호출)
 * - DataLake: Task-Task 간 데이터 공유 (Mutex + Snapshot)
 * - Multi-Instance: ch 파라미터로 인스턴스 구분
 *   - XM: ch=0 고정 (EBIMU_MAX_INSTANCES=1, 사용 시)
 *   - IMU Hub: ch=0~5 (EBIMU_MAX_INSTANCES=6)
 *
 * RTOS: Mutex + Snapshot / BareMetal: volatile 직접 접근
 * ================================================================ */

/**
 * @brief Auto-Sense + DataLake 초기화
 * @param ch 채널 인덱스 (0 ~ EBIMU_MAX_INSTANCES-1)
 */
void EbimuV6_StateInit(uint8_t ch);

/**
 * @brief [Writer] DataLake에 최신 패킷 업데이트 (UartRxTask에서 호출)
 * @param ch 채널 인덱스
 * @param packet 파싱 완료된 IMU 데이터 패킷
 */
void EbimuV6_UpdateData(uint8_t ch, const EBIMU_Data_t* packet);

/**
 * @brief [Reader] DataLake에서 최신 스냅샷 가져오기 (Core Process에서 호출)
 * @param ch 채널 인덱스
 * @param out 데이터를 복사받을 구조체 포인터
 * @return true: 데이터 유효 (OPERATIONAL), false: 연결 끊김
 */
bool EbimuV6_GetLatest(uint8_t ch, EBIMU_Data_t* out);

/**
 * @brief Auto-Sense 연결 상태 확인
 * @param ch 채널 인덱스
 * @return true: OPERATIONAL (데이터 수신 중), false: STOPPED (타임아웃)
 */
bool EbimuV6_IsOnline(uint8_t ch);

/**
 * @brief Auto-Sense 주기적 실행 (PnP Task에서 호출, 100ms 주기)
 * @param ch 채널 인덱스
 * @details 데이터 타임아웃 체크. 500ms 동안 데이터 없으면 STOPPED 전환.
 */
void EbimuV6_RunPeriodic(uint8_t ch);

#endif /* DEVICES_E2BOX_EBIMU_9DOFV6_H_ */
