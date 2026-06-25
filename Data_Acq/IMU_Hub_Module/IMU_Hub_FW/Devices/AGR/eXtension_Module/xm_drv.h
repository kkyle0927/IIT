/**
 ******************************************************************************
 * @file    xm_drv.h
 * @author  HyundoKim
 * @brief   [Device Layer] IMU Hub ↔ XM10 통신 드라이버
 * @version 5.0
 * @date    Feb 25, 2026
 *
 * @details
 * IMU Hub Module이 XM10(Master)과 FDCAN으로 통신하기 위한 드라이버입니다.
 * AGR-DOP V3 프로토콜 (CiA 301 기반, Core + Transport/CAN_FD 직접 사용)
 * 
 * [통신 구조]
 * - Node ID: 0x0D (IMU Hub A, v3.0)
 * - TPDO1 (0x18D): Group A 데이터 (Metadata + IMU 0, 1, 2)
 * - TPDO2 (0x28D): Group B 데이터 (Metadata + IMU 3, 4, 5)
 * - SDO Request  (0x60D): TPDO Mapping (0x1A00/0x1A01), Comm Profile 등
 * - SDO Response (0x58D): OD 기반 자동 응답
 * - Heartbeat (0x70D): PnP Slave 연결 상태
 * 
 * [OD Index 할당 (CiA 301 표준)]
 * - 0x1000~0x1FFF: Communication Profile (Device Type, Identity, TPDO Mapping)
 * - 0x2000~0x2FFF: 설정 파라미터 (IMU Connected Mask 등)
 * - 0x3000~0x3FFF: 진단/Metadata (PDO Metadata)
 * - 0x6000~0x6005: IMU 센서 데이터 (Device Profile)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef DEVICES_AGR_EXTENSION_MODULE_XM_DRV_H_
#define DEVICES_AGR_EXTENSION_MODULE_XM_DRV_H_

#include <stdint.h>
#include <stdbool.h>

#include "ioif_agrb_fdcan.h"

#include "agr_dop_types.h"
#include "Transport/CAN_FD/agr_dop_canfd.h"
#include "agr_nmt.h"
#include "agr_pnp_slave.h"

/**
 *-----------------------------------------------------------
 * PUBLIC DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/** @brief IMU 센서 개수 */
#define XM_DRV_IMU_COUNT            6

/** @brief Group A IMU 개수 (IMU 0, 1, 2) */
#define XM_DRV_GROUP_A_COUNT        3

/** @brief Group B IMU 개수 (IMU 3, 4, 5) */
#define XM_DRV_GROUP_B_COUNT        3

/**
 *-----------------------------------------------------------
 * SCALING FACTORS (float ↔ int16 변환)
 *-----------------------------------------------------------
 * @details EBIMU-9DOFV6 센서 데이터시트 기반 정확한 스케일링
 * 
 * [센서 → 드라이버 (EBIMU 내부)]
 * - EBIMU_SCALE_QUAT  = 0.0001  (int16 × 0.0001 → float)
 * - EBIMU_SCALE_GYRO  = 0.1     (int16 × 0.1 → float deg/s)
 * - EBIMU_SCALE_ACC   = 0.01    (int16 × 0.01 → float g)
 * 
 * [드라이버 → XM10 (역변환)]
 * - float → int16 변환 시 아래 스케일을 곱함
 */

/** @brief Quaternion: float (-1.0 ~ 1.0) → int16 (-10000 ~ 10000) */
#define XM_SCALE_QUAT_TO_INT16      10000

/** @brief Euler Angles: float (-180° ~ 180°) → int16 (-18000 ~ 18000), 0.01° precision */
#define XM_SCALE_EULER_TO_INT16     100

/** @brief Accelerometer: float (±16g) → int16 (±1600), 0.01g precision */
#define XM_SCALE_ACC_TO_INT16       100

/** @brief Gyroscope: float (±2000 deg/s) → int16 (±20000), 0.1 deg/s precision */
#define XM_SCALE_GYRO_TO_INT16      10

/** @brief Magnetometer: float (±4800 uT) → int16 (±4800), 1 uT precision */
#define XM_SCALE_MAG_TO_INT16       1

/**
 *-----------------------------------------------------------
 * OD INDEX DEFINITIONS (IMU Hub 전용)
 *-----------------------------------------------------------
 * @note Index는 노드 내에서만 고유하면 됩니다.
 */

/* ===== Communication Profile (CiA 301 Mandatory: 0x1000~0x1FFF) ===== */
#define XM_OD_IDX_DEVICE_TYPE       0x1000  /**< Device Type (UINT32, RO) */
#define XM_OD_IDX_ERROR_REGISTER    0x1001  /**< Error Register (UINT8, RO) */
#define XM_OD_IDX_HB_TIME          0x1017  /**< Producer Heartbeat Time (UINT16, RW, ms) */
#define XM_OD_IDX_IDENTITY         0x1018  /**< Identity Object (Vendor/Product/Rev/Serial) */

/* ===== TPDO Mapping Parameter (CiA 301 표준: 0x1A00~0x1BFF) ===== */
#define XM_OD_IDX_TPDO1_MAPPING     0x1A00  /**< TPDO1 Mapping (Group A: Metadata + IMU 0,1,2) */
#define XM_OD_IDX_TPDO2_MAPPING     0x1A01  /**< TPDO2 Mapping (Group B: Metadata + IMU 3,4,5) */

/* ===== 설정 파라미터 (Manufacturer-specific: 0x2000~) ===== */
#define XM_OD_IDX_IMU_CONN_MASK     0x2000  /**< IMU Connected Mask (RO, 1B) - bit0~5: IMU0~5 */

/* ===== PDO Metadata (0x3000~) ===== */
#define XM_OD_IDX_METADATA          0x3000  /**< PDO Metadata (4B): Timestamp(3B) + Valid Mask(1B) */

/* ===== IMU 센서 데이터 (0x6000~0x6005) ===== */
/**
 * @brief IMU OD Index 구조 (CANopen 표준 패턴)
 * 
 * Index: 0x6000~0x6005 (IMU 0~5)
 * 
 * SubIndex (개별 항목, 2 bytes):
 *   0x00~0x03: Quaternion (q_w, q_x, q_y, q_z)
 *   0x10~0x12: Euler Angles (roll, pitch, yaw)
 *   0x20~0x22: Accelerometer (acc_x, acc_y, acc_z)
 *   0x30~0x32: Gyroscope (gyr_x, gyr_y, gyr_z)
 *   0x40~0x42: Magnetometer (mag_x, mag_y, mag_z)
 * 
 * SubIndex (뭉탱이, BLOB):
 *   0x50: q[4] (8 bytes)
 *   0x51: rpy[3] (6 bytes)
 *   0x52: a[3] (6 bytes)
 *   0x53: g[3] (6 bytes)
 *   0x54: m[3] (6 bytes)
 * 
 * SubIndex (조합, BLOB):
 *   0x60: {q,a,g} (20 bytes) - 기본 세트
 *   0x61: {q,a,g,m} (26 bytes)
 *   0x62: {q,rpy,a,g,m} (32 bytes) - 전체
 */

/* 개별 항목 (예시, 실제 OD에서 정의) */
#define XM_OD_SUBIDX_QUAT_W         0x00
#define XM_OD_SUBIDX_QUAT_X         0x01
#define XM_OD_SUBIDX_QUAT_Y         0x02
#define XM_OD_SUBIDX_QUAT_Z         0x03

#define XM_OD_SUBIDX_EULER_ROLL     0x10
#define XM_OD_SUBIDX_EULER_PITCH    0x11
#define XM_OD_SUBIDX_EULER_YAW      0x12

#define XM_OD_SUBIDX_ACC_X          0x20
#define XM_OD_SUBIDX_ACC_Y          0x21
#define XM_OD_SUBIDX_ACC_Z          0x22

#define XM_OD_SUBIDX_GYR_X          0x30
#define XM_OD_SUBIDX_GYR_Y          0x31
#define XM_OD_SUBIDX_GYR_Z          0x32

#define XM_OD_SUBIDX_MAG_X          0x40
#define XM_OD_SUBIDX_MAG_Y          0x41
#define XM_OD_SUBIDX_MAG_Z          0x42

/* 뭉탱이 (배열) */
#define XM_OD_SUBIDX_QUAT_ARRAY     0x50  // q[4] (8B)
#define XM_OD_SUBIDX_EULER_ARRAY    0x51  // rpy[3] (6B)
#define XM_OD_SUBIDX_ACC_ARRAY      0x52  // a[3] (6B)
#define XM_OD_SUBIDX_GYR_ARRAY      0x53  // g[3] (6B)
#define XM_OD_SUBIDX_MAG_ARRAY      0x54  // m[3] (6B)

/* 조합 (전체) */
#define XM_OD_SUBIDX_QAG            0x60  // {q,a,g} (20B) - 기본
#define XM_OD_SUBIDX_QAGM           0x61  // {q,a,g,m} (26B)
#define XM_OD_SUBIDX_FULL           0x62  // {q,rpy,a,g,m} (32B) - 전체
/* [Snapshot/HD/IMU_Resolution_For_Swiss] float32 원본 해상도 (Swiss 학회 시연 전용) */
#define XM_OD_SUBIDX_QAG_F32        0x70  // {q,a,g} float32 (40B) - 다운스케일 없음

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES(extern)
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PUBLIC ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief PDO 메타데이터 구조체 (4 bytes, Packed)
 * @details
 * TPDO1/2의 앞 4 bytes에 포함되는 메타데이터입니다.
 * 
 * [구조]
 * - Byte 0-2: Timestamp (24-bit, 0~16,777,215 ms, 약 4.6시간 순환)
 * - Byte 3: Valid Mask (8-bit, bit0~5 = IMU 0~5)
 * 
 * [사용 예시]
 * ```c
 * XM_Drv_PdoMetadata_t meta;
 * meta.timestamp = GetSystemTimeMs() & 0x00FFFFFF;
 * meta.valid_mask = 0b00000111;  // IMU 0,1,2 연결됨
 * memcpy(pdo_buf, &meta, 4);
 * ```
 */
typedef struct __attribute__((packed)) {
    uint8_t  timestamp_low;     /**< Timestamp bits 0-7 */
    uint8_t  timestamp_mid;     /**< Timestamp bits 8-15 */
    uint8_t  timestamp_high;    /**< Timestamp bits 16-23 */
    uint8_t  valid_mask;        /**< IMU Valid Mask (bit0~5) */
} XM_Drv_PdoMetadata_t;  /* Total: 4 bytes */

/**
 * @brief IMU 센서 데이터 구조체 (int16 기반, 20 bytes per IMU)
 * @details
 * EBIMU-9DOFV6의 float 데이터를 int16로 변환하여 대역폭을 50% 절약합니다.
 * 
 * [PDO 전송 최적화]
 * - 순서: {q, a, g} → 20B (SubIndex 0x60에서 사용)
 * - rpy, m은 PDO에 포함하지 않음 (대역폭 절약)
 * 
 * [변환 규칙]
 * - Quaternion: float × 10000 → int16 (±10000 범위, 0.0001 정밀도)
 * - Accel: float × 100 → int16 (±1600 범위, 0.01g 정밀도)
 * - Gyro: float × 10 → int16 (±20000 범위, 0.1 deg/s 정밀도)
 * 
 * [복원 규칙 (XM10에서)]
 * - float quat_w = (float)q[0] / 10000.0f;
 * - float acc_x = (float)a[0] / 100.0f;
 * - float gyr_x = (float)g[0] / 10.0f;
 */
typedef struct __attribute__((packed)) {
    int16_t  q[4];       /**< 쿼터니언 (w, x, y, z) - 8 bytes */
    int16_t  a[3];       /**< 가속도 (x, y, z) - 6 bytes */
    int16_t  g[3];       /**< 자이로 (x, y, z) - 6 bytes */
    // int16_t  rpy[3];  /**< 오일러 각 (roll, pitch, yaw) - PDO 미포함 */
    // int16_t  m[3];    /**< 지자계 (x, y, z) - PDO 미포함 */
} XM_Drv_ImuData_t;      /* Total: 20 bytes */

/**
 * @brief [Snapshot/HD/IMU_Resolution_For_Swiss] IMU float32 원본 데이터 (40 bytes)
 * @details
 * Swiss 학회 시연 전용 — int16 다운스케일 없이 float32 원본 해상도를 그대로 전송한다.
 * OD SubIndex 0x70(XM_OD_SUBIDX_QAG_F32)에 매핑되며, TPDO1 = Metadata(4B) + 본 40B.
 * 단일 IMU(첫 연결 채널) 전용. int16 경로(0x60)와 가산 공존(별도 슬롯).
 *
 * [구성] {q,a,g} float32 — 양자화/클램프 없음
 * - Quaternion: float q_w/x/y/z 원본 (4 × 4 = 16B)
 * - Accel:      float acc_x/y/z 원본 (3 × 4 = 12B)
 * - Gyro:       float gyr_x/y/z 원본 (3 × 4 = 12B)
 */
typedef struct __attribute__((packed)) {
    float  q[4];       /**< 쿼터니언 (w, x, y, z) - 16 bytes */
    float  a[3];       /**< 가속도 (x, y, z) - 12 bytes */
    float  g[3];       /**< 자이로 (x, y, z) - 12 bytes */
} XM_Drv_ImuDataF_t;     /* Total: 40 bytes */

/**
 * @brief XM10으로부터 수신한 데이터 구조체
 * @details 
 * PDO/SDO로 받은 모든 설정값과 상태를 담습니다.
 * ISR에서 업데이트되며, Main Loop에서 XM_Drv_GetRxData()로 안전하게 읽습니다.
 */
 typedef struct {
    /* ===== 설정 파라미터 (SDO로 수신) ===== */
    uint8_t  imu_connected_mask; /**< XM10이 인식한 IMU 연결 마스크, OD 0x2000 */
    
    /* ===== 제어 명령 (PDO로 수신 - 향후 확장) ===== */
    // bool     enable_logging;     /**< 로깅 활성화 플래그 */
    // uint8_t  led_brightness;     /**< LED 밝기 설정 (0-100) */
    
    /* ===== 시스템 상태 ===== */
    uint32_t last_update_tick;   /**< 마지막 업데이트 시각 (ms) */
} XM_RxData_t;

/**
 * @brief IMU Hub → XM10 송신 데이터 구조체 (cm_drv.txData 패턴)
 * @details 
 * PDO와 SDO 전송용 데이터를 한 곳에서 관리합니다.
 * Main Loop에서 업데이트되며, PDO/SDO 전송 시 참조됩니다.
 */
typedef struct {
    /* ===== PDO 데이터 (실시간, 1kHz) ===== */
    XM_Drv_ImuData_t imu[XM_DRV_IMU_COUNT];  /**< IMU 센서 데이터 (6개 × 20B = 120B) */

    /* ===== 1ms 루프 카운터 (OD 0x6050, TPDO mapping 용) =====
     * Control Task 매 1ms tick 증가. XM 가 수신 측에서 gap 감지. */
    uint32_t ctrl_tick_ms;

    /* ===== SDO 데이터 (비정기, 이벤트 기반) ===== */
    uint8_t  calibration_status;  /**< 캘리브레이션 상태 (0=미완료, 1=완료), OD 0x3001 */
    uint16_t error_code;          /**< 에러 코드, OD 0x3002 */
    uint8_t  sensor_health[6];    /**< 센서 건강 상태 (6개), OD 0x3003 */
    // 향후 추가 SDO 변수들...
} XM_TxData_t;

/**
 *-----------------------------------------------------------
 * PUBLIC INLINE HELPER FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief float → int16 변환 (오버플로우 방지)
 * @param value float 값
 * @param scale 스케일 factor
 * @return int16 변환 값 (±32767 범위로 제한)
 */
static inline int16_t XM_Drv_FloatToInt16(float value, int scale)
{
    int32_t temp = (int32_t)(value * scale);
    if (temp > 32767) return 32767;
    if (temp < -32768) return -32768;
    return (int16_t)temp;
}

/**
 * @brief Quaternion float → int16 변환
 * @param quat_float float (-1.0 ~ 1.0)
 * @return int16 (±10000)
 */
static inline int16_t XM_Drv_QuatToInt16(float quat_float)
{
    return XM_Drv_FloatToInt16(quat_float, XM_SCALE_QUAT_TO_INT16);
}

/**
 * @brief Euler Angle float → int16 변환
 * @param angle_deg float (-180° ~ 180°)
 * @return int16 (±18000)
 */
static inline int16_t XM_Drv_EulerToInt16(float angle_deg)
{
    return XM_Drv_FloatToInt16(angle_deg, XM_SCALE_EULER_TO_INT16);
}

/**
 * @brief Accelerometer float → int16 변환
 * @param acc_g float (±16g)
 * @return int16 (±1600)
 */
static inline int16_t XM_Drv_AccelToInt16(float acc_g)
{
    return XM_Drv_FloatToInt16(acc_g, XM_SCALE_ACC_TO_INT16);
}

/**
 * @brief Gyroscope float → int16 변환
 * @param gyro_dps float (±2000 deg/s)
 * @return int16 (±20000)
 */
static inline int16_t XM_Drv_GyroToInt16(float gyro_dps)
{
    return XM_Drv_FloatToInt16(gyro_dps, XM_SCALE_GYRO_TO_INT16);
}

/**
 * @brief Magnetometer float → int16 변환
 * @param mag_ut float (±4800 uT)
 * @return int16 (±4800)
 */
static inline int16_t XM_Drv_MagToInt16(float mag_ut)
{
    return XM_Drv_FloatToInt16(mag_ut, XM_SCALE_MAG_TO_INT16);
}

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
 * @brief XM 드라이버 초기화 (V4.0: PnP Slave 주입)
 * @param fdcan_id  FDCAN Instance ID (Software Queue 사용)
 * @param slave_pnp System Layer에서 초기화된 PnP Slave 인스턴스 포인터
 * @return 0=성공, <0=에러
 *
 * @details
 * [V4.0 변경사항]
 * - 기존: 내부에서 AGR_PnP_Inst_t 생성 + 초기화
 * - 변경: System Layer에서 초기화된 AGR_PnP_Slave_t*를 주입받음
 * - 효과: PnP 라이프사이클을 System이 관리
 * @note Tx 는 내부 _Drv_TxFunc_Queue 로 일원화(BareMetal single-writer) → tx_func 주입 안 받음.
 */
int XM_Drv_Init(IOIF_FDCANx_t fdcan_id, AGR_PnP_Slave_t* slave_pnp);

/**
 * @brief CAN 메시지 수신 처리 (ISR Context에서 호출)
 * @param can_id CAN ID
 * @param data 데이터 버퍼
 * @param len 데이터 길이
 * @details 
 * FDCAN ISR의 Rx 콜백에서 직접 호출합니다.
 * SDO/PDO 처리 후 이중 버퍼의 Write Buffer에 데이터를 저장합니다.
 * 
 * @note ISR-Safe 함수입니다.
 */
void XM_Drv_ProcessCANMessage(uint32_t can_id, const uint8_t* data, uint8_t len);

/**
 * @brief Deferred SDO 처리 (main() while(1) 전용)
 * @details FDCAN Rx ISR 이 defer-ring 에 적재한 0x600 SDO 프레임을 main 컨텍스트에서
 *          decode + OD write + 응답 Tx 한다. 무거운 SDO 처리를 1kHz Timer ISR 밖으로
 *          빼 결정성을 보호하고, 응답 Tx 는 single-writer(Timer ISR) 로 일원화한다.
 * @note main() while(1) 에서 매 루프 호출. ISR 에서 호출 금지.
 */
void XM_Drv_ProcessDeferredSdo(void);

/**
 * @brief SDO defer ring full-drop 누적 횟수 (진단용)
 */
uint32_t XM_Drv_GetSdoDeferDropCount(void);

/**
 * @brief Tx defer ring drain — Timer ISR(단일 송신자) 전용
 * @details main() 이 적재한 비-PDO 프레임(SDO 응답)을 HW 직접 송신한다. PDO-first:
 *          TPDO direct 송신 후 호출. cap=2/tick, HW full 시 leave-in-place 재시도.
 * @note Timer ISR 에서만 호출 (HW Tx single-writer 일원화). main() 에서 호출 금지.
 */
void XM_Drv_DrainTxDeferRing(void);

/**
 * @brief master-online abort 처리 — Timer ISR 전용
 * @details _OnMasterOnline(RX ISR)이 set 한 flag 를 consume 하여 stuck TX 버퍼를
 *          AbortAllTx 한다. HW Tx 조작을 Timer ISR 로 일원화(single-writer).
 * @note RunLoop 맨 앞(TPDO 송신 前)에서 호출. main()/RX ISR 에서 호출 금지.
 */
void XM_Drv_ServiceMasterOnlineAbort(void);

/* [REMOVED V5.0] XM_Drv_CheckAndClearHeartbeat / XM_Drv_GetXmNmtState (deprecated, 호출처 0)
 * → 대체: XM_Drv_IsConnected() / XM_Drv_GetNmtState() (AGR_PnP_Slave 위임). */

/**
 * @brief IMU 데이터 업데이트 (Core Process에서 호출)
 * @param imu_index IMU 인덱스 (0~5)
 * @param data IMU 데이터
 */
void XM_Drv_UpdateImuData(uint8_t imu_index, const XM_Drv_ImuData_t* data);

/**
 * @brief [Snapshot/HD/IMU_Resolution_For_Swiss] IMU float32 원본 슬롯 갱신
 * @param data float32 IMU 데이터 (첫 연결 채널, NULL 무시)
 * @details OD 0x6000.0x70(40B float)에 매핑되는 단일 슬롯을 갱신한다.
 *          TPDO1 으로 int16 다운스케일 없이 전송한다 (Swiss 시연 전용).
 */
void XM_Drv_UpdateImuFloat(const XM_Drv_ImuDataF_t* data);

/**
 * @brief 프레임 타임스탬프 설정 (PDO 전송 전에 호출)
 * @param time_ms 현재 시간 (ms, 24-bit 범위: 0~16,777,215)
 * @note TPDO Payload의 timestamp_ms 필드에 저장됩니다 (하위 24-bit 사용).
 */
void XM_Drv_SetFrameTimestamp(uint32_t time_ms);

/**
 * @brief 유효 마스크 초기화 (새 프레임 시작 시 호출)
 */
void XM_Drv_ClearValidMask(void);

/**
 * @brief TPDO1 전송 (Group A: IMU 0, 1, 2)
 * @return 0=성공, -1=tx_func NULL, -2=Inhibit Time 제한
 * @details 
 * 1kHz 주기로 호출합니다.
 * PDO Inhibit Time이 설정된 경우, 최소 간격이 경과하지 않으면 -2를 반환합니다.
 */
int XM_Drv_SendTPDO1(void);

/**
 * @brief TPDO2 전송 (Group B: IMU 3, 4, 5)
 * @return 0=성공, -1=tx_func NULL, -2=Inhibit Time 제한
 * @details 
 * 1kHz 주기로 호출합니다.
 * PDO Inhibit Time이 설정된 경우, 최소 간격이 경과하지 않으면 -2를 반환합니다.
 */
int XM_Drv_SendTPDO2(void);

/**
 * @brief PDO Inhibit Time 설정 (버스 부하 제어)
 * @param pdo_num PDO 번호 (1 or 2)
 * @param inhibit_time_us 최소 전송 간격 (μs)
 * @details
 * PDO 전송 시 최소 간격을 강제하여 CAN 버스 부하를 제어합니다.
 * 
 * @example
 * ```c
 * // TPDO1을 최소 1ms 간격으로만 전송 (1000μs)
 * XM_Drv_SetPDOInhibitTime(1, 1000);
 * 
 * // TPDO2는 Inhibit Time 없이 즉시 전송
 * XM_Drv_DisablePDOInhibit(2);
 * ```
 */
void XM_Drv_SetPDOInhibitTime(uint8_t pdo_num, uint32_t inhibit_time_us);

/**
 * @brief PDO Inhibit Time 비활성화
 * @param pdo_num PDO 번호 (1 or 2)
 */
void XM_Drv_DisablePDOInhibit(uint8_t pdo_num);

/* [DEPRECATED V4.0] Boot-up/Heartbeat는 AGR_PnP_Slave가 자동 관리합니다.
 * - Boot-up: BOOT_UP 상태에서 1초마다 자동 재전송
 * - Heartbeat: PRE_OP/OPERATIONAL에서 1초마다 자동 전송
 * 수동 호출이 필요한 경우: AGR_PnP_Slave_SendBootupNow() / SendHeartbeatNow()
 */

/* [REMOVED V5.0] XM_Drv_HandleSDORequest (호출처 0) — SDO 는 FDCAN RX ISR defer-ring →
 * main() XM_Drv_ProcessDeferredSdo → AGR_SDO_ProcessRequest 로 자동 처리. 수동 진입점 폐기. */

/**
 * @brief 주기 실행 (V3.0: Public API)
 * @details PnP Heartbeat 관리 및 타임아웃 체크 (10ms 주기 권장)
 */
void XM_Drv_RunPeriodic(void);

/**
 * @brief XM10 연결 상태 확인
 * @return true=연결됨 (OPERATIONAL)
 */
bool XM_Drv_IsConnected(void);

/**
 * @brief 현재 NMT 상태 조회
 * @return NMT 상태
 */
AGR_NMT_State_t XM_Drv_GetNmtState(void);

/**
 * @brief 연결된 IMU 비트마스크 설정
 * @param mask 비트마스크 (bit0=IMU0, bit1=IMU1, ...)
 */
void XM_Drv_SetImuConnectedMask(uint8_t mask);

/**
 * @brief 연결된 IMU 비트마스크 조회
 * @return 비트마스크
 */
uint8_t XM_Drv_GetImuConnectedMask(void);

/**
 * @brief 현재 샘플링 주파수 조회
 * @return 샘플링 주파수 (Hz)
 * @details XM10이 SDO로 설정한 값 (기본값: 1000Hz)
 */
uint16_t XM_Drv_GetSampleRate(void);

/**
 * @brief FDCAN Rx 데이터 가져오기 (Main Loop용)
 * @param out_data [out] XM10 수신 데이터 구조체 포인터
 * @return true=성공, false=실패 (out_data가 NULL)
 * @details 
 * _FetchInputs()에서 호출하여 XM10으로부터 받은 모든 데이터를 가져옵니다.
 * ISR에서 업데이트된 내부 변수를 Critical Section으로 보호하며 복사합니다.
 * 
 * @code
 * XM_RxData_t xm_data;
 * if (XM_Drv_GetRxData(&xm_data)) {
 *     uint8_t mask = xm_data.imu_connected_mask;
 * }
 * @endcode
 */
bool XM_Drv_GetRxData(XM_RxData_t* out_data);

/**
 * @brief Tx 데이터 가져오기 (디버깅/모니터링용)
 * @param out_data [out] IMU Hub 송신 데이터 구조체 포인터
 * @return true=성공, false=실패
 * @details 
 * 현재 XM10으로 전송 중인 데이터를 확인할 때 사용합니다.
 */
bool XM_Drv_GetTxData(XM_TxData_t* out_data);

/**
 * @brief SYNC 수신 여부 확인 및 클리어 (Phase-Lock TPDO용)
 * @return true=SYNC 수신됨 (flag 자동 클리어), false=미수신
 */
bool XM_Drv_ConsumeSyncPending(void);

/**
 * @brief 총 SYNC 수신 횟수 (디버깅용)
 */
uint32_t XM_Drv_GetSyncCount(void);

/**
 * @brief Tx 경로 통계 조회 (backpressure 진단)
 * @param[out] hb_direct   HB가 HW FIFO 직접 전송된 횟수 (NULL 허용)
 * @param[out] hb_queued   HB가 SW Queue로 적체된 횟수 (NULL 허용)
 * @param[out] sdo_direct  SDO 응답 직접 전송 횟수 (NULL 허용)
 * @param[out] sdo_queued  SDO 응답 SW Queue 적체 횟수 (NULL 허용)
 * @param[out] emcy_direct EMCY 직접 전송 횟수 (NULL 허용)
 * @param[out] emcy_queued EMCY SW Queue 적체 횟수 (NULL 허용)
 */
void XM_Drv_GetTxPathStats(uint32_t* hb_direct, uint32_t* hb_queued,
                            uint32_t* sdo_direct, uint32_t* sdo_queued,
                            uint32_t* emcy_direct, uint32_t* emcy_queued);

/**
 * @brief 1ms 루프 카운터 갱신 (OD 0x6050, TPDO mapping)
 * @param tick_ms 현재 tick (ms 단위, 단조 증가)
 * @note Control Task 매 1ms 호출 — TPDO 인코딩 전에 갱신되어야 한다.
 */
void XM_Drv_UpdateCtrlTickMs(uint32_t tick_ms);

/**
 * @brief Emergency (EMCY) 메시지 전송 (CiA 301)
 * @param error_code     16-bit 에러 코드 (LE)
 * @param error_register 1-byte 에러 레지스터 (OD 0x1001 mirror)
 * @return 0 성공, <0 실패
 *
 * @details CAN-ID = 0x080 + Node ID (IMU=0x08D). HW Queue 모드에서
 *   가장 낮은 CAN-ID 로 자연 우선순위 확보. 현재는 MfgSpec 5B = 0x00 fixed.
 *
 * @note 트리거 조건은 TBD. 현재 호출처는 OD 0x7F02.01 SDO write 더미 주입 전용.
 */
int XM_Drv_SendEmcy(uint16_t error_code, uint8_t error_register);

#endif /* DEVICES_AGR_EXTENSION_MODULE_XM_DRV_H_ */
