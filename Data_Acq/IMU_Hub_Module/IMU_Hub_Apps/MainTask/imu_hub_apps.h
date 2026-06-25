/**
 ******************************************************************************
 * @file    imu_hub_apps.h
 * @author  HyundoKim
 * @brief   [Application Layer] IMU Hub Task - 시스템 데이터 & Public API
 * @version 4.0 (Task Layer IPO 패턴 적용)
 * @date    2026-03-19
 *
 * @details
 * [Task Layer IPO 패턴]
 * Timer ISR이 ImuHub_Task_RunLoop()를 직접 호출하여 전체 IPO 루프를 실행합니다.
 * Application Layer가 IPO를 직접 소유합니다.
 *
 * [IPO 실행 흐름 — 단일 Timer ISR (FDCAN Tx single-writer)]
 * TIM16 ISR (1ms, NVIC preempt 1) — MainTask (유일 FDCAN 송신자)
 *   └─ ImuHub_Task_RunLoop()  [Application Layer]
 *         ├─ XM_Drv_ServiceMasterOnlineAbort() → stuck-TX flush (RX ISR 지연분)
 *         ├─ _FetchInputs()   → UART IMU(seqlock) + FDCAN XM10 수집
 *         ├─ TaskMngr_Run()   → 사용자 알고리즘
 *         ├─ _FlushOutputs()  → TPDO1/2 direct (freshest-wins, full=drop)
 *         ├─ XM_Drv_DrainTxDeferRing() → SDO 응답 등 비-PDO 단일 송신
 *         └─ ImuSysService_Run() → LED, CDC, PnP HB, Debug (비RT, tick-gate)
 *
 * main() while(1) — background
 *   └─ XM_Drv_ProcessDeferredSdo() → SDO decode + OD write (무거운 작업, RX ISR 밖)
 *
 * FDCAN2 RX ISR (NVIC preempt 2): SDO→defer-ring 적재만 · SYNC/NMT/HB→flag/state inline
 *
 * [Live Expression 디버깅]
 * @code
 * g_sys.imus[0].is_connected      // IMU 0 연결 상태
 * g_sys.xm10.is_connected         // XM10 연결 상태
 * g_sys.xm10.nmt_state            // XM10 NMT 상태
 * g_uart_debug.uart[0].idle_callback_cnt  // UART 0 수신 확인
 * g_perf_stats.cpu_usage_percent  // CPU 사용률 (%)
 * g_perf_stats.loop_max_us        // 루프 최대 실행 시간 (µs)
 * @endcode
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#pragma once

#ifndef IMU_HUB_APPS_IMU_HUB_APPS_H_
#define IMU_HUB_APPS_IMU_HUB_APPS_H_

#include <stdint.h>
#include <stdbool.h>
#include "module.h"          /* MAX_IMU_CHANNELS */
#include "agr_nmt.h"         /* AGR_NMT_State_t */
#include "ebimu-9dofv6.h"    /* EBIMU_Data_t */

/**
 *-----------------------------------------------------------
 * PUBLIC TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief IMU 센서 데이터 (원시 데이터 + 연결 상태 통합)
 */
typedef struct {
    EBIMU_Data_t data;             /**< EBIMU 센서 데이터 (Quaternion, Gyro, Accel 등) */
    bool         is_connected;     /**< 연결 상태 (UART 타임아웃 기반) */
    uint32_t     last_update_time; /**< 마지막 수신 시간 (ms) */
} ImuData_t;

/**
 * @brief XM10 마스터 모듈 데이터
 */
typedef struct {
    bool            is_connected;       /**< XM10 연결 상태 (OPERATIONAL) */
    AGR_NMT_State_t nmt_state;          /**< NMT 상태 (디버깅용) */
    uint8_t         imu_connected_mask; /**< XM10에서 인식한 IMU 연결 마스크 (bit0~5) */
} Xm10Data_t;

/**
 * @brief IMU Hub 시스템 전역 데이터 구조체
 * @details Application Layer의 모든 Task에서 이 구조체를 통해 센서 데이터에 접근합니다.
 *
 * [사용 예시]
 * @code
 * if (g_sys.imus[0].is_connected) {
 *     float qw = g_sys.imus[0].data.q_w;
 * }
 * if (g_sys.xm10.is_connected) {
 *     uint8_t mask = g_sys.xm10.imu_connected_mask;
 * }
 * @endcode
 */
typedef struct {
    ImuData_t  imus[MAX_IMU_CHANNELS]; /**< IMU 센서 (데이터 + 연결 상태, 6채널) */
    Xm10Data_t xm10;                   /**< XM10 마스터 상태 및 설정값 */
    uint32_t   tick_cnt;               /**< 시스템 Tick 카운터 (1ms 단위) */
} ImuHubSystem_t;

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES (extern)
 *-----------------------------------------------------------
 */

/** @brief IMU Hub 시스템 전역 데이터 (User Algorithm에서 직접 접근) */
extern ImuHubSystem_t g_sys;

/** @brief [Live Expression] UART 디버깅 정보 */
typedef struct {
    struct {
        uint32_t rx_state;
        uint32_t error_code;
        uint32_t dma_cndtr;
        uint32_t rx_bytes;
        uint32_t idle_callback_cnt;
        uint32_t error_callback_cnt;
        bool     idle_ie_enabled;
        bool     dmar_enabled;
        bool     connected;
        uint8_t  pnp_state;
    } uart[MAX_IMU_CHANNELS];
    uint32_t sensor_config_sent;
    uint32_t sensor_config_failed;
    struct {
        bool    connected;
        uint8_t pnp_state;
    } xm10;
    uint32_t update_tick;
} UartDebugMonitor_t;

extern volatile UartDebugMonitor_t g_uart_debug;

/** @brief [Live Expression] 동작 중인 UART 확인용 */
typedef struct {
    bool     uart0_working;  /**< USART1 - IMU 1 */
    bool     uart1_working;  /**< USART2 - IMU 2 */
    bool     uart2_working;  /**< USART3 - IMU 3 */
    bool     uart3_working;  /**< UART4  - IMU 4 */
    bool     uart4_working;  /**< UART5  - IMU 5 */
    bool     uart5_working;  /**< LPUART1 - IMU 6 */
    uint32_t working_count;  /**< 동작 중인 센서 개수 */
} UartStatusMonitor_t;

extern volatile UartStatusMonitor_t g_uart_status;

/** @brief [Live Expression] 루프 성능 통계 + FDCAN Tx 진단 */
typedef struct {
    /* --- Loop Performance --- */
    uint32_t loop_max_us;           /**< 루프 최대 실행 시간 (µs) */
    uint32_t loop_min_us;           /**< 루프 최소 실행 시간 (µs) */
    uint32_t loop_avg_us;           /**< 루프 평균 실행 시간 (µs) — EMA(α=1/16) */
    uint32_t loop_jitter_us;        /**< 루프 지터 (max − min) (µs) */
    uint32_t loop_overrun_count;    /**< 1ms 초과 실행 횟수 */
    uint8_t  cpu_usage_percent;     /**< CPU 사용률 (%) */
    uint32_t uptime_sec;            /**< 가동 시간 (초) */

    /* --- TPDO Tx 통계 --- */
    uint32_t tpdo_tx_attempt_count; /**< TPDO 송신 시도 총합 (성공+실패) */
    uint32_t tpdo_tx_success_count; /**< TPDO 송신 성공 (HAL AddToTxFifoQ OK) */
    uint32_t tpdo_overrun_count;    /**< TPDO Drop (Tx FIFO Full 시) */
    uint32_t tpdo_full_streak;      /**< 현재 연속 Full 지속 (ms) */
    uint32_t tpdo_full_streak_max;  /**< 관찰된 최대 연속 Full (ms) */
    uint32_t abort_recovery_count;  /**< 연속 Full → AbortAllTx 발동 횟수 */

    /* --- HW Tx Queue 점유 --- */
    uint8_t  tx_in_flight_now;      /**< 현재 HW Queue 점유 (0~3) */
    uint8_t  tx_in_flight_max;      /**< 관찰된 최대 점유 (0~3) */

    /* --- HB / SDO / EMCY 경로 분리 통계 --- */
    uint32_t hb_direct_tx_count;    /**< HB 직접 HW 전송 횟수 (DOP ctx path, 현재 설계상 0) */
    uint32_t hb_queued_count;       /**< HB SW Queue 적체 횟수 (DOP ctx path, 현재 설계상 0) */
    uint32_t hb_pnp_direct_count;   /**< HB PnP path 직접 전송 (실제 HB 는 전부 여기) */
    uint32_t bootup_pnp_count;      /**< Bootup PnP path 송신 횟수 */
    uint32_t sdo_rsp_direct_count;  /**< SDO 응답 직접 전송 */
    uint32_t sdo_rsp_queued_count;  /**< SDO 응답 SW Queue 적체 */
    uint32_t emcy_direct_count;     /**< EMCY 직접 전송 (HW Queue natural prio) */
    uint32_t emcy_queued_count;     /**< EMCY SW Queue 적체 (HW full 시) */

    /* --- SW Tx Queue 사용 (IOIF GetQueueStats) --- */
    uint16_t sw_queue_depth_now;    /**< 현재 SW Queue 적체량 */
    uint16_t sw_queue_depth_peak;   /**< 관측 기간 최대 적체량 (high-water) */
    uint32_t sw_queue_drop_count;   /**< Queue full 로 drop 된 누적 메시지 */

    /* --- CAN Bus 상태 --- */
    uint32_t fdcan_bus_off_count;            /**< Bus-Off 진입 누적 (IOIF GetErrorStats) */
    uint32_t fdcan_err_passive_count;        /**< Error Passive 진입 누적 */
    uint8_t  fdcan_last_tec_at_err_passive;  /**< 가장 최근 Error Passive 진입 시 TEC snapshot */
    uint8_t  fdcan_tec;                      /**< Transmit Error Counter (HW ECR) */
    uint8_t  fdcan_rec;                      /**< Receive Error Counter (HW ECR) */

    /* --- 계산 결과 (비율) --- */
    uint32_t tpdo_drop_ppm;         /**< Drop 비율 PPM (overrun × 1,000,000 / attempt) */
} PerfStats_t;

extern volatile PerfStats_t g_perf_stats;

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/**
 * @brief IMU Hub Task 초기화 (1회 실행)
 * @details system_startup.c의 _InitSystemServices()에서 호출됩니다.
 *          g_sys 초기화 + User_Setup() (TSM 생성, 초기값 설정) 실행.
 */
void ImuHub_Task_Init(void);

/**
 * @brief IMU Hub Task 메인 루프 (1ms 주기)
 * @details TIM16 ISR에서 직접 호출됩니다.
 *          전체 IPO 루프 (Input → Process → Output)를 소유합니다.
 *
 * [실행 순서]
 * 0. master-online abort 처리 (RX ISR 지연분, TPDO 前)
 * 1. Input:   UART IMU 데이터 수집(seqlock) + FDCAN XM10 상태 수집
 * 2. Process: TaskMngr_Run() — TSM 기반 사용자 알고리즘
 * 3. Output:  TPDO1/2 direct + Tx-defer-ring drain (FDCAN 단일 송신자)
 * 4. Service: ImuSysService_Run() — IO/CDC/PnP HB/Debug (비RT, fold)
 * 5. Perf:    DWT 루프 시간 측정
 *
 * 무거운 SDO decode 는 main() while(1) XM_Drv_ProcessDeferredSdo() 에서 처리(결정성 보호).
 */
void ImuHub_Task_RunLoop(void);

#endif /* IMU_HUB_APPS_IMU_HUB_APPS_H_ */
