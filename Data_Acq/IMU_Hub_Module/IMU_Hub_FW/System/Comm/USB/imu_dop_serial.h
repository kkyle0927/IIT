/**
 ******************************************************************************
 * @file    imu_dop_serial.h
 * @author  HyundoKim
 * @brief   [System/Comm/USB] IMU Hub DOP Serial Transport over USB-CDC
 * @version 1.0
 * @date    2026-03-26
 *
 * @details
 * USB-CDC를 통한 DOP/Serial Transport 브릿지.
 * Python GUI(sensor-studio)에서 USB-CDC로 접속하여
 * SDO Read/Write + TPDO 스트리밍을 수행합니다.
 *
 * [프레이밍]
 * COBS Frame: [COBS-encoded(Header + Payload + CRC16-LE)] [0x00]
 * Header: MsgType(1B) + NodeID(1B)
 *
 * [OD 구조]
 * - Communication Profile (0x1000~0x1FFF): Device Type, Error Reg, Heartbeat, Identity
 * - IMU Device Profile (0x6000~0x6FFF): 6ch Quaternion, Accel, Status, Config, Command
 * - Peripheral Test (0x7F00): Test Cmd, Result, Detail, Pass/Fail Bits
 *
 * [동작 모델 — BareMetal IPO]
 * - TX (TPDO): Main Task(TIM16, 1ms) Output 직후 → OD 갱신 → COBS encode → CDC Send
 *              Period divider로 N ms마다 전송 (센서 데이터 최신성 보장)
 * - RX (SDO):  System Task(TIM7, 1ms) → CDC ring drain → COBS decode → SDO routing
 * - TX (HB):   System Task(TIM7, 1ms) → 1초 주기 Heartbeat
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef SYSTEM_COMM_USB_IMU_DOP_SERIAL_H_
#define SYSTEM_COMM_USB_IMU_DOP_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS
 *-----------------------------------------------------------
 */

/** @brief TPDO 전송 주기 (ms) — 1ms = 1kHz (Control Task와 동기) */
#define IMU_DOP_TPDO_PERIOD_MS          1

/** @brief TPDO1 페이로드 크기 — 6ch × 40B + 6B status + 4B ctrl_tick = 250B */
#define IMU_DOP_TPDO1_PAYLOAD_SIZE      250

/** @brief Heartbeat 전송 주기 (ms) */
#define IMU_DOP_HEARTBEAT_PERIOD_MS     1000

/**
 *-----------------------------------------------------------
 * PUBLIC TYPES — 데이터 주입 (V2: System←App 역의존 제거)
 *-----------------------------------------------------------
 */

#define IMU_DOP_MAX_CHANNELS  6

/** @brief 단일 IMU 채널 데이터 (Application → System 주입) */
typedef struct {
    bool     is_connected;
    uint32_t last_update_time;
    float    q_w, q_x, q_y, q_z;
    float    acc_x, acc_y, acc_z;
    float    gyr_x, gyr_y, gyr_z;
    uint8_t  sensor_type;
} ImuDopSerial_ChannelData_t;

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/**
 * @brief DOP Serial Transport 초기화
 * @details OD 테이블 구성, DOP Context + Serial Context 초기화.
 *          system_startup.c → _InitSystemServices()에서 호출.
 */
void ImuDopSerial_Init(void);

/**
 * @brief CDC RX 데이터를 DOP Serial로 전달 (raw binary)
 * @param data 수신 데이터
 * @param len  수신 길이
 */
void ImuDopSerial_FeedRxData(const uint8_t *data, uint32_t len);

/**
 * @brief [Main Task] TPDO 전송 — IPO Output 직후 호출
 * @param now_ms 현재 시각 (ms)
 * @note  ImuHub_Task_RunLoop() (TIM16, 1ms)에서 Output 후 호출.
 *        Period divider 내장: IMU_DOP_TPDO_PERIOD_MS마다 실제 전송.
 */
void ImuDopSerial_SendTpdo(uint32_t now_ms);

/**
 * @brief [System Task] RX 처리 + Heartbeat 전송
 * @param now_ms 현재 시각 (ms)
 * @note  ImuSysService_Run() (단일 MainTask TIM16, 1ms)에서 호출.
 */
void ImuDopSerial_RunPeriodic(uint32_t now_ms);

/**
 * @brief DOP Serial 활성 여부 (CDC 연결 시에만 전송)
 * @return true=활성
 */
bool ImuDopSerial_IsActive(void);

/**
 * @brief IMU 데이터 주입 (Application → System, TPDO 전송 직전)
 * @param channels  6채널 IMU 데이터 배열
 * @param count     채널 수 (최대 IMU_DOP_MAX_CHANNELS)
 */
void ImuDopSerial_UpdateLiveData(const ImuDopSerial_ChannelData_t *channels, uint8_t count);

#endif /* SYSTEM_COMM_USB_IMU_DOP_SERIAL_H_ */
