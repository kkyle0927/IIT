/**
 ******************************************************************************
 * @file    mti-630.h
 * @author  HyundoKim
 * @brief   [Devices Layer] Xsens MTi-630 IMU 드라이버 (ROBUST)
 * @details ioif_agrb_uart의 콜백을 받아 Xsens MTi 패킷을 파싱하고,
 * System Layer에 큐(Queue)로 전달합니다.
 * @version 1.0 (Robust Implementation - Ported from IMU_Hub_Module)
 * @date    Jan 26, 2026
 *
 * @changelog
 * v1.0 (Jan 26, 2026) - ROBUST Implementation:
 *                       - Added IsConnected() for timeout monitoring
 *                       - Added diagnostic statistics API
 *                       - Enhanced error handling
 *                       - IMU_Hub_Module proven pattern
 * v0.2 (Jan 22, 2026) - Added full boot sequence (GoToConfig, GoToMeasurement, Reset, SetBaudrate)
 * v0.1 (Nov 12, 2025) - Initial implementation
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef DEVICES_XSENS_MTI_630_H_
#define DEVICES_XSENS_MTI_630_H_

#include "ioif_agrb_uart.h"
#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/* ... (Xsens MTData2 정의) ... */
#define XSENS_PREAMBLE          (0xFA) // 패킷 시작
#define XSENS_BID               (0xFF) // 버스 ID (Master)
#define XSENS_MID_MTDATA2       (0x36) // 메시지 ID (MTData2)
#define XSENS_MID_SETOUTPUTCFG  (0xC0) // 출력 설정 명령 ID

// MTData2 Data IDs (XDI) - 1kHz 요청 데이터
#define XDI_QUATERNION       (0x2010) // 4x float (w,x,y,z) - 16 bytes
#define XDI_ACCELERATION     (0x4020) // 3x float (x,y,z) - 12 bytes
#define XDI_GYROSCOPE_DATA   (0x8020) // 3x float (x,y,z) - 12 bytes

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief [수정] Xsens MTData2 패킷 (1kHz 요청 데이터)
 */
typedef struct {
    uint32_t timestamp;   // 수신 시점의 시스템 틱 (ms)
    
    // 1. Quaternion (4-float)
    float q_w, q_x, q_y, q_z;
    
    // 2. Calibrated Accelerometer (3-float)
    float acc_x, acc_y, acc_z;
    
    // 3. Calibrated Gyroscope (3-float)
    float gyr_x, gyr_y, gyr_z;

} XsensMTi_packet_t;

/**
 * @brief Devices 계층이 System 계층으로 파싱된 패킷을 전달하는 콜백
 */
typedef void (*ImuPacketCallback_t)(const XsensMTi_packet_t* packet);

/**
 * @brief 진단 통계 구조체 (디버깅용)
 */
typedef struct {
    uint32_t packets_received;         // 성공적으로 파싱된 패킷 수
    uint32_t packets_failed_checksum;  // 체크섬 실패 패킷 수
    uint32_t packets_failed_overflow;  // 버퍼 오버플로우 패킷 수
    uint32_t config_cmd_sent;          // 전송 성공한 설정 명령 수
    uint32_t config_cmd_failed;        // 전송 실패한 설정 명령 수
} XsensMTi_Stats_t;

/**
 * @brief 드라이버 인터페이스 (싱글톤)
 */
typedef struct {
    /**
     * @brief 드라이버를 초기화합니다 (큐 생성, 콜백 등록).
     */
    bool (*init)(IOIF_UARTx_t id, ImuPacketCallback_t packet_cb);
    
    /**
     * @brief IMU에게 1kHz로 데이터 전송을 요청하는 설정 메시지를 보냅니다.
     * @details UartRxHandler가 init() 직후 호출해야 합니다.
     * @note 전체 부팅 시퀀스 포함 (GoToConfig → SetOutputConfig → GoToMeasurement)
     */
    bool (*ConfigureOutput)(void);
    
    /**
     * @brief 센서 연결 상태 확인 (타임아웃 기반)
     * @param last_rx_time 마지막 패킷 수신 시간 (ms)
     * @param current_time 현재 시간 (ms)
     * @return true: 연결됨, false: 타임아웃
     */
    bool (*IsConnected)(uint32_t last_rx_time, uint32_t current_time);
    
    /* --- Optional Commands (Individual) --- */
    void (*Cmd_Reset)(IOIF_UARTx_t uart_id);
    void (*Cmd_GoToConfig)(IOIF_UARTx_t uart_id);
    void (*Cmd_GoToMeasurement)(IOIF_UARTx_t uart_id);
    void (*Cmd_SetBaudrate)(IOIF_UARTx_t uart_id, uint32_t baudrate);
} XsensMTi630_t;

extern XsensMTi630_t xsensMTi630;

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

/**
 * @brief [디버깅] 진단 통계 가져오기
 * @param stats 통계 데이터를 담을 구조체 포인터
 */
void XsensMTi_GetStats(XsensMTi_Stats_t* stats);


#endif /* DEVICES_XSENS_MTI_630_H_ */
