/**
 ******************************************************************************
 * @file    imu_hub_apps.c
 * @author  HyundoKim
 * @brief   [Application Layer] IMU Hub Task — Task Layer IPO 패턴
 * @version 4.0 (Task Layer IPO 적용)
 * @date    2026-03-19
 *
 * @details
 * [Task Layer IPO 패턴 (V4.0)]
 * Timer ISR은 ImuHub_Task_RunLoop()를 직접 호출하며, Application Layer가
 * 전체 IPO 루프를 소유합니다.
 *
 * [구조]
 * ┌─────────────────────────────────────────────┐
 * │  APPLICATION (IMU_Hub_Apps)                  │
 * │  ┌──────────────────────────────────────┐   │
 * │  │  ImuHub_Task_RunLoop() [IPO 소유]    │   │
 * │  │  ├─ _FetchInputs()                   │   │
 * │  │  ├─ _UserProcess()  ← TSM + 알고리즘 │   │
 * │  │  └─ _FlushOutputs()                  │   │
 * │  └──────────────────────────────────────┘   │
 * └──────────────┬──────────────────────────────┘
 *                ↓ (직접 호출, 수평 관계)
 * ┌─────────────────────────────────────────────┐
 * │  INTERFACE (Device / IOIF / System Comm)     │
 * │  uart_rx_handler, xm_drv, io_manager, ...   │
 * └─────────────────────────────────────────────┘
 *
 * [데이터 접근]
 * - g_sys.imus[i]: UART IMU 센서 데이터 (6채널)
 * - g_sys.xm10: XM10 마스터 상태
 * - g_sys.tick_cnt: 시스템 Tick (1ms)
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

/* ===== Application Layer Header ===== */
#include "imu_hub_apps.h"

/* ===== System Layer ===== */
#include "module.h"
#include "system_startup.h"
#include "io_manager.h"

/* ===== System Comm Layer ===== */
#include "uart_rx_handler.h"

/* ===== Device Layer ===== */
#include "xm_drv.h"
#include "ebimu-9dofv6.h"
#include "mti-630.h"

/* ===== System Layer ===== */
#include "system_startup.h"   /* System_GetFdcanHandle() */

/* ===== IOIF Layer ===== */
#include "ioif_agrb_tim.h"
#include "ioif_agrb_dwt.h"
#include "ioif_agrb_fdcan.h"
#include "imu_dop_serial.h"  /* ImuDopSerial_SendTpdo(), ImuDopSerial_UpdateLiveData() */

/* ===== System Service (단일 Timer ISR 로 fold — FDCAN Tx single-writer) ===== */
#include "imu_sys_service.h"

/* ===== Task State Machine ===== */
#include "task_mngr.h"

#include <math.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

/* Task State Machine State IDs */
#define STATE_IDLE          0  /**< 대기 상태 (XM10 연결 대기) */
#define STATE_RUNNING       2  /**< 실행 상태 (센서 데이터 처리) */

/**
 *-----------------------------------------------------------
 * PUBLIC VARIABLES (declared extern in imu_hub_apps.h)
 *-----------------------------------------------------------
 */

ImuHubSystem_t g_sys;

/* [Live Expression] UART 디버깅 모니터 */
volatile UartDebugMonitor_t  g_uart_debug  = {0};

/* [Live Expression] 동작 중인 UART 확인용 */
volatile UartStatusMonitor_t g_uart_status = {0};

/* [Live Expression] 성능 통계 */
volatile PerfStats_t         g_perf_stats  = {0};

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

/* IPO 주기 타이머 */

/* TPDO 교대 전송 토글 (false=TPDO1, true=TPDO2) — 500Hz 모드에서만 사용 */
#ifndef TPDO_DUAL_SEND_ENABLE
#define TPDO_DUAL_SEND_ENABLE   1
#endif
#if !TPDO_DUAL_SEND_ENABLE
static bool s_pdo_toggle = false;
#endif

/* Task State Machine (TaskMngr API — pool-backed handle) */
static TsmObject_t* s_tsm = NULL;

/* User Algorithm 변수 */
static float s_raw_roll[6]      = {0};   /**< Quaternion → Euler 원시 Roll */
static float s_raw_pitch[6]     = {0};   /**< Quaternion → Euler 원시 Pitch */
static float s_raw_yaw[6]       = {0};   /**< Quaternion → Euler 원시 Yaw */
static float s_filtered_roll[6] = {0};   /**< LPF 적용 Roll */
static float s_filtered_pitch[6] = {0};  /**< LPF 적용 Pitch */
static float s_filtered_yaw[6]  = {0};   /**< LPF 적용 Yaw */

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

/* IPO Steps */
static void _FetchInputs(void);
static void _FlushOutputs(void);
static void _InjectDopSerialData(void);

/* Task State Machine */
static void _State_Idle_Entry(void);
static void _State_Idle_Loop(void);
static void _State_Running_Entry(void);
static void _State_Running_Loop(void);
static void _State_Running_Exit(void);

/* User Algorithms */
static void _ComputeEulerAngles(void);
static void _ApplyLowPassFilter(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — Task API
 *-----------------------------------------------------------
 */

/**
 * @brief IMU Hub Task 초기화 (1회 실행)
 * @details system_startup.c의 _InitSystemServices()에서 호출됩니다.
 */
void ImuHub_Task_Init(void)
{
    /* 시스템 데이터 초기화 */
    memset(&g_sys, 0, sizeof(ImuHubSystem_t));
    g_sys.xm10.is_connected    = false;
    g_sys.xm10.nmt_state       = AGR_NMT_BOOT_UP;
    g_sys.xm10.imu_connected_mask = 0;

    /* ===== Task State Machine 초기화 ===== */
    s_tsm = TaskMngr_Create(STATE_IDLE);

    TaskMngr_AddState(s_tsm, STATE_IDLE,
                      _State_Idle_Entry,
                      _State_Idle_Loop,
                      NULL);
    TaskMngr_AddState(s_tsm, STATE_RUNNING,
                      _State_Running_Entry,
                      _State_Running_Loop,
                      _State_Running_Exit);

    /* ===== 사용자 변수 초기화 ===== */
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        s_raw_roll[i]      = 0.0f;
        s_raw_pitch[i]     = 0.0f;
        s_raw_yaw[i]       = 0.0f;
        s_filtered_roll[i] = 0.0f;
        s_filtered_pitch[i] = 0.0f;
        s_filtered_yaw[i]  = 0.0f;
    }

    /* ===== [User] 추가 초기화 영역 ===== */
    // ...
}

/**
 * @brief IMU Hub Task 메인 루프 (1ms 주기) — Task Layer IPO
 * @details TIM16 ISR에서 직접 호출됩니다.
 *          전체 IPO 루프를 이 함수에서 소유합니다.
 */
void ImuHub_Task_RunLoop(void)
{
    /* ===== [Perf 0] Loop 시작 시간 기록 ===== */
    uint32_t loop_start_cycles = IOIF_DWT_GetCycles();

    /* ===== [0a] master-online abort (RX ISR→Timer ISR 지연) — TPDO 송신 前 단일 수행 =====
     * _OnMasterOnline(RX ISR)이 요청한 stuck-TX flush 를 여기(단일 writer)서 수행. */
    XM_Drv_ServiceMasterOnlineAbort();

    /* ===== [0] 1ms tick counter → OD 0x6050 (TPDO mapping, XM 수신 gap 감지) ===== */
    XM_Drv_UpdateCtrlTickMs(IOIF_TIM_GetTick());

    /* ===== [1] Input: UART IMU + FDCAN XM10 수집 ===== */
    _FetchInputs();

    /* ===== [2] Process: TSM 기반 사용자 알고리즘 ===== */
    TaskMngr_Run(s_tsm);

    /* ===== [3-1] Output: TPDO 교대 전송 (500Hz/group) ===== */
    /*
     * [전송 전략] TPDO1 ↔ TPDO2 교대 전송 (Adaptive)
     * - 매 tick: TPDO1 또는 TPDO2 중 1개만 전송 (500Hz/group)
     * - Tx FIFO free_level 확인 후 전송 (보험)
     * - FIFO Full 시 Drop (stale data) + overrun 카운터 추적
     * - HW Tx FIFO 1 slot만 사용 → 2 slots 여유 (HB/SDO용)
     */
    _FlushOutputs();

    /* ===== [3-1b] FDCAN Tx defer ring drain (single-writer 단일 송신) =====
     * main() (XM_Drv_ProcessDeferredSdo)이 tx-defer-ring 에 적재한 SDO 응답 등
     * 비-PDO 프레임을 Timer ISR 가 단일 송신. TPDO direct(_FlushOutputs) 후 호출
     * = PDO-first. HW full 시 leave-in-place 재시도(비-PDO 무손실). */
    XM_Drv_DrainTxDeferRing();

    /* ===== [3-2] DOP Serial 데이터 주입 + TPDO 전송 ===== */
    _InjectDopSerialData();
    ImuDopSerial_SendTpdo(g_sys.tick_cnt);

    /* ===== [3-3] Software Tx Queue 처리 ===== */
    /* TPDO 활성(XM 연결) 시 HW FIFO 1 slot 예약, 미연결 시 전부 Queue 드레인 */
    uint8_t reserved = XM_Drv_IsConnected() ? 1 : 0;
    IOIF_FDCAN_ProcessQueue(System_GetFdcanHandle(), reserved);

    /* ===== [3-4] System 서비스(비RT) — (구) TIM7 ImuSystemTask 를 단일 Timer ISR 로 fold =====
     * FDCAN Tx single-writer 를 위해 PnP HB 송신을 TPDO 와 같은 컨텍스트(TIM16)로 통일.
     * 내부 tick-gate(LED/Debug/auto-sense 100ms, PnP 500ms, CDC drain 매 tick). */
    ImuSysService_Run();

    /* ===== [4] Perf: DWT Loop 실행 시간 측정 ===== */
    {
        uint32_t elapsed_cycles = IOIF_DWT_GetCycles() - loop_start_cycles;
        uint32_t elapsed_us = elapsed_cycles / (SystemCoreClock / 1000000U);

        if (elapsed_us > g_perf_stats.loop_max_us) { g_perf_stats.loop_max_us = elapsed_us; }
        if (g_perf_stats.loop_min_us == 0U || elapsed_us < g_perf_stats.loop_min_us) {
            g_perf_stats.loop_min_us = elapsed_us;
        }
        g_perf_stats.loop_avg_us    = (g_perf_stats.loop_avg_us * 15U + elapsed_us) / 16U;
        g_perf_stats.loop_jitter_us = g_perf_stats.loop_max_us - g_perf_stats.loop_min_us;
        if (elapsed_us > 1000U)      { g_perf_stats.loop_overrun_count++; }
        g_perf_stats.cpu_usage_percent = (uint8_t)(g_perf_stats.loop_avg_us / 10U);
        g_perf_stats.uptime_sec        = IOIF_TIM_GetTick() / 1000U;
    }

    /* ===== [Tick] 시스템 Tick 증가 ===== */
    g_sys.tick_cnt++;
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — IPO Steps
 *-----------------------------------------------------------
 */

/**
 * @brief [IPO Step 1] Input Gathering
 * @details UART 6채널 IMU + FDCAN XM10 데이터를 g_sys에 통합합니다.
 */
static void _FetchInputs(void)
{
    /* 1. UART IMU 센서 데이터 수집 (Auto-Detect: EBIMU / XSENS) */
    EBIMU_Data_t temp_imus[MAX_IMU_CHANNELS];
    UartRxHandler_FetchAllImus(temp_imus);

    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        g_sys.imus[i].data            = temp_imus[i];
        g_sys.imus[i].last_update_time = temp_imus[i].timestamp;
    }

    /* 2. IMU 연결 상태 (Device Auto-Sense 기반) */
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        ImuSensorType_t type = UartRxHandler_GetSensorType((uint8_t)i);
        if (type == IMU_SENSOR_EBIMU) {
            g_sys.imus[i].is_connected = EbimuV6_IsOnline((uint8_t)i);
        } else if (type == IMU_SENSOR_XSENS) {
            g_sys.imus[i].is_connected = XsensMTi_IsOnline((uint8_t)i);
        } else {
            g_sys.imus[i].is_connected = false;
        }
    }

    /* 3. FDCAN XM10 마스터 데이터 수집 */
    XM_RxData_t xm_rx;
    if (XM_Drv_GetRxData(&xm_rx)) {
        g_sys.xm10.imu_connected_mask = xm_rx.imu_connected_mask;
    }

    /* 4. XM10 연결 상태 */
    g_sys.xm10.is_connected = XM_Drv_IsConnected();
    g_sys.xm10.nmt_state    = XM_Drv_GetNmtState();
}

/**
 * @brief [IPO Step 3] Output Flushing — TPDO 교대 전송 (Adaptive)
 * @details
 * g_sys.imus[] 데이터를 float → int16 변환 후 TPDO1 ↔ TPDO2 교대 전송합니다.
 *
 * [전송 전략 — 양산 안전 설계]
 * - 기본: TPDO1/TPDO2 교대 전송 (500Hz/group, 1 slot/tick)
 * - 보험: GetTxFifoFreeLevel() 확인 후 전송
 * - 실패: Drop (stale data) + overrun 카운터 추적 (양산 진단)
 *
 * [왜 교대 전송?]
 * - HW Tx FIFO = 3 slots, CAN-FD 64B ≈ 84µs
 * - 동시 2개 전송 시 FIFO 2/3 점유 → HB/SDO 1 slot 경합 → FIFO Full 위험
 * - 교대 전송: 1 slot/tick → 2 slots 여유 → HB/SDO 안전 수용
 */
/* ===== Phase-Lock TPDO (SYNC 위상 정렬) ===== */
#define TPDO_FREE_RUN_INTERVAL_MS   1   /**< SYNC 없을 때 자율 전송 간격 (ms) */

/* ===== TPDO 송신 모드 =====
 * 0: 교대 모드 (매 tick TPDO1↔TPDO2 하나씩) — 각 그룹 500Hz, 1 slot/tick
 * 1: Dual-Send 모드 (매 tick TPDO1+TPDO2 동시) — 각 그룹 1kHz, 2 slot/tick
 *
 * 1kHz 모드 검증 시 #define TPDO_DUAL_SEND_ENABLE 1 로 전환.
 * 실기기에서 g_perf_stats.tpdo_overrun_count, tpdo_drop_ppm 등 모니터링 후 판단. */
#ifndef TPDO_DUAL_SEND_ENABLE
#define TPDO_DUAL_SEND_ENABLE       1   /**< 기본: 1kHz Dual-Send 모드 테스트 */
#endif

/* ===== 연속 Full 자가 치유 =====
 * FDCAN TX Queue가 연속 N ms 동안 Full이면 stuck TX 가정 (XM10 offline, 버스 에러 등).
 * AbortAllTx로 HW/SW Queue 전부 비우고 재시작. Bus-Off 진입(수초) 이전에 선제 회복.
 * 10ms = CiA 301 SYNC 주기의 여러 배수이면서 정상 arbitration 지연보다 충분히 긴 값. */
#define TPDO_FULL_STREAK_LIMIT_MS   10

static uint32_t s_last_can_tpdo_tick = 0;
static uint32_t s_can_tpdo_sync_count = 0;  /**< 디버깅: SYNC-triggered TPDO 횟수 */
static uint32_t s_can_tpdo_free_count = 0;  /**< 디버깅: Free-running TPDO 횟수 */
static uint32_t s_tpdo_full_streak_ms = 0;  /**< 연속 Full 지속 (ms) */

static void _FlushOutputs(void)
{
    if (!XM_Drv_IsConnected()) {
        return;
    }

    /* ===== Phase-Lock TPDO Gate ===== */
    bool should_send = false;
    uint32_t now_ms = g_sys.tick_cnt;

    if (XM_Drv_ConsumeSyncPending()) {
        should_send = true;
        s_can_tpdo_sync_count++;
    } else if (now_ms - s_last_can_tpdo_tick >= TPDO_FREE_RUN_INTERVAL_MS) {
        should_send = true;
        s_can_tpdo_free_count++;
    }

    if (!should_send) {
        return;
    }
    s_last_can_tpdo_tick = now_ms;

    /* 타임스탬프 및 Valid Mask 초기화 */
    XM_Drv_SetFrameTimestamp(g_sys.tick_cnt);
    XM_Drv_ClearValidMask();

    /* IMU 데이터 업데이트 (float → int16 변환) */
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        XM_Drv_ImuData_t drv_data;

        if (g_sys.imus[i].is_connected) {
            const EBIMU_Data_t* e = &g_sys.imus[i].data;

            drv_data.q[0] = XM_Drv_QuatToInt16(e->q_w);
            drv_data.q[1] = XM_Drv_QuatToInt16(e->q_x);
            drv_data.q[2] = XM_Drv_QuatToInt16(e->q_y);
            drv_data.q[3] = XM_Drv_QuatToInt16(e->q_z);

            drv_data.a[0] = XM_Drv_AccelToInt16(e->acc_x);
            drv_data.a[1] = XM_Drv_AccelToInt16(e->acc_y);
            drv_data.a[2] = XM_Drv_AccelToInt16(e->acc_z);

            drv_data.g[0] = XM_Drv_GyroToInt16(e->gyr_x);
            drv_data.g[1] = XM_Drv_GyroToInt16(e->gyr_y);
            drv_data.g[2] = XM_Drv_GyroToInt16(e->gyr_z);
        } else {
            memset(&drv_data, 0, sizeof(XM_Drv_ImuData_t));
        }

        XM_Drv_UpdateImuData((uint8_t)i, &drv_data);
    }

    /* ===== [Snapshot/HD/IMU_Resolution_For_Swiss] 첫 연결 채널 → float32 원본 슬롯 =====
     * Swiss 학회 시연 전용: IMU 1개를 int16 다운스케일 없이 float32 그대로 TPDO1 송신.
     * 물리 채널 위치 무관 — 첫 연결(online) 채널을 선택해 고정 슬롯(0x6000.0x70)에 적재.
     * XM 이 TPDO1 에 [Metadata 4B + 40B float] 만 매핑하므로 int16 경로(0x60)와 무충돌. */
    {
        XM_Drv_ImuDataF_t f32;
        int first_ch = -1;
        for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
            if (g_sys.imus[i].is_connected) { first_ch = i; break; }
        }
        if (first_ch >= 0) {
            const EBIMU_Data_t* e = &g_sys.imus[first_ch].data;
            f32.q[0] = e->q_w;   f32.q[1] = e->q_x;   f32.q[2] = e->q_y;   f32.q[3] = e->q_z;
            f32.a[0] = e->acc_x; f32.a[1] = e->acc_y; f32.a[2] = e->acc_z;
            f32.g[0] = e->gyr_x; f32.g[1] = e->gyr_y; f32.g[2] = e->gyr_z;
        } else {
            memset(&f32, 0, sizeof(f32));
        }
        XM_Drv_UpdateImuFloat(&f32);
    }

    /* IMU 연결 마스크 설정 */
    uint8_t connected_mask = 0;
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        if (g_sys.imus[i].is_connected) {
            connected_mask |= (uint8_t)(1U << i);
        }
    }
    XM_Drv_SetImuConnectedMask(connected_mask);

    /* ===== TPDO 송신 + 연속 Full 자가 치유 + 진단 =====
     * Queue 모드에서 IOIF_FDCAN_GetTxFifoFreeLevel 은 TFQF(full flag) 만
     * 노출하므로 0/1 만 반환한다. 실제 free slot 수는 TXBRP popcount 로
     * 3 - in_flight 로 계산해야 한다. */
    uint8_t in_flight = (uint8_t)IOIF_FDCAN_GetTxInFlightCount(System_GetFdcanHandle());
    g_perf_stats.tx_in_flight_now = in_flight;
    if (in_flight > g_perf_stats.tx_in_flight_max) {
        g_perf_stats.tx_in_flight_max = in_flight;
    }
    uint8_t hw_free = (in_flight < 3U) ? (uint8_t)(3U - in_flight) : 0U;

    g_perf_stats.tpdo_tx_attempt_count++;

#if TPDO_DUAL_SEND_ENABLE
    /* ===== 1kHz Dual-Send: TPDO1 + TPDO2 동시 전송 (2 slot/tick) =====
     * - hw_free ≥ 2: 둘 다 전송 (각 그룹 1kHz, HW Queue 1 slot 여유)
     * - hw_free == 1: Drop (TPDO 둘을 쪼개면 순서 역전 위험) + streak 증가
     * - hw_free == 0: Drop + streak 증가 */
    if (hw_free >= 2U) {
        XM_Drv_SendTPDO1();
        XM_Drv_SendTPDO2();
        g_perf_stats.tpdo_tx_success_count += 2U;
        s_tpdo_full_streak_ms = 0;
    } else {
        g_perf_stats.tpdo_overrun_count++;
        s_tpdo_full_streak_ms++;
    }
#else
    /* ===== 500Hz 교대 전송: 매 tick TPDO1↔TPDO2 하나씩 (1 slot/tick) ===== */
    if (hw_free >= 1U) {
        if (s_pdo_toggle) {
            XM_Drv_SendTPDO2();  /* Group B: IMU 3,4,5 */
        } else {
            XM_Drv_SendTPDO1();  /* Group A: IMU 0,1,2 */
        }
        s_pdo_toggle = !s_pdo_toggle;
        g_perf_stats.tpdo_tx_success_count++;
        s_tpdo_full_streak_ms = 0;
    } else {
        g_perf_stats.tpdo_overrun_count++;
        s_tpdo_full_streak_ms++;
    }
#endif

    /* 연속 Full 자가 치유 (양 모드 공통) */
    if (s_tpdo_full_streak_ms >= TPDO_FULL_STREAK_LIMIT_MS) {
        IOIF_FDCAN_AbortAllTx(System_GetFdcanHandle());
        g_perf_stats.abort_recovery_count++;
        s_tpdo_full_streak_ms = 0;
    }

    /* streak 기록 */
    g_perf_stats.tpdo_full_streak = s_tpdo_full_streak_ms;
    if (s_tpdo_full_streak_ms > g_perf_stats.tpdo_full_streak_max) {
        g_perf_stats.tpdo_full_streak_max = s_tpdo_full_streak_ms;
    }
}


/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — Task State Machine
 *-----------------------------------------------------------
 */

static void _State_Idle_Entry(void)
{
    /* [User] 대기 상태 진입 초기화 */
}

static void _State_Idle_Loop(void)
{
    /* XM10 연결 확인 → RUNNING 상태 전환 */
    if (g_sys.xm10.is_connected) {
        (void)TaskMngr_Transition(s_tsm, STATE_RUNNING);
        return;
    }

    IO_ButtonEvent_t btn1_evt = IO_GetButtonEvent(1);
    if (btn1_evt == IO_BTN_CLICK) {
        /* [User] Button 1 클릭 동작 정의 영역 */
    }
}

static void _State_Running_Entry(void)
{
    /* [User] 실행 상태 진입 초기화 */
}

static void _State_Running_Loop(void)
{
    /* XM10 연결 끊어짐 → IDLE 복귀 */
    if (!g_sys.xm10.is_connected) {
        (void)TaskMngr_Transition(s_tsm, STATE_IDLE);
        return;
    }

    /* ===== [User] 알고리즘 실행 ===== */
    _ComputeEulerAngles();
    _ApplyLowPassFilter();

    IO_ButtonEvent_t btn2_evt = IO_GetButtonEvent(2);
    if (btn2_evt == IO_BTN_CLICK) {
        /* [User] Button 2 클릭 동작 정의 영역 */
    }

    /* ===== [User] 추가 로직 작성 영역 ===== */
    // ...
}

static void _State_Running_Exit(void)
{
    /* [User] 실행 상태 종료 정리 */
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — User Algorithms
 *-----------------------------------------------------------
 */

/**
 * @brief Quaternion → Euler 각도 변환
 * @details 연결된 IMU 센서의 Quaternion 데이터를 Roll/Pitch/Yaw로 변환합니다.
 *          결과는 s_raw_*[]에 저장되며, _ApplyLowPassFilter()의 입력으로 사용됩니다.
 */
static void _ComputeEulerAngles(void)
{
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        if (!g_sys.imus[i].is_connected) { continue; }

        float qw = g_sys.imus[i].data.q_w;
        float qx = g_sys.imus[i].data.q_x;
        float qy = g_sys.imus[i].data.q_y;
        float qz = g_sys.imus[i].data.q_z;

        s_raw_roll[i]  = atan2f(2.0f * (qw * qx + qy * qz),
                                1.0f - 2.0f * (qx * qx + qy * qy));
        s_raw_pitch[i] = asinf(2.0f * (qw * qy - qz * qx));
        s_raw_yaw[i]   = atan2f(2.0f * (qw * qz + qx * qy),
                                1.0f - 2.0f * (qy * qy + qz * qz));
    }
}

/**
 * @brief Low-pass 필터 적용
 * @details filtered[k] = alpha * filtered[k-1] + (1 - alpha) * raw[k]
 *          _ComputeEulerAngles() 호출 후 s_raw_*[]에 원시값이 준비되어야 합니다.
 */
static void _ApplyLowPassFilter(void)
{
    const float alpha = 0.9f;

    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        if (!g_sys.imus[i].is_connected) { continue; }

        s_filtered_roll[i]  = alpha * s_filtered_roll[i]  + (1.0f - alpha) * s_raw_roll[i];
        s_filtered_pitch[i] = alpha * s_filtered_pitch[i] + (1.0f - alpha) * s_raw_pitch[i];
        s_filtered_yaw[i]   = alpha * s_filtered_yaw[i]   + (1.0f - alpha) * s_raw_yaw[i];

        /* ===== [User] 필터링된 각도 활용 영역 ===== */
        // float roll  = s_filtered_roll[i];
        // float pitch = s_filtered_pitch[i];
        // float yaw   = s_filtered_yaw[i];
    }
}

/**
 * @brief Application → System 데이터 주입 (TPDO 전송 직전)
 */
static void _InjectDopSerialData(void)
{
    ImuDopSerial_ChannelData_t ch_data[IMU_DOP_MAX_CHANNELS];
    for (uint8_t i = 0; i < MAX_IMU_CHANNELS && i < IMU_DOP_MAX_CHANNELS; i++) {
        ch_data[i] = (ImuDopSerial_ChannelData_t){
            .is_connected    = g_sys.imus[i].is_connected,
            .last_update_time = g_sys.imus[i].last_update_time,
            .q_w  = g_sys.imus[i].data.q_w,
            .q_x  = g_sys.imus[i].data.q_x,
            .q_y  = g_sys.imus[i].data.q_y,
            .q_z  = g_sys.imus[i].data.q_z,
            .acc_x = g_sys.imus[i].data.acc_x,
            .acc_y = g_sys.imus[i].data.acc_y,
            .acc_z = g_sys.imus[i].data.acc_z,
            .gyr_x = g_sys.imus[i].data.gyr_x,
            .gyr_y = g_sys.imus[i].data.gyr_y,
            .gyr_z = g_sys.imus[i].data.gyr_z,
            .sensor_type = (uint8_t)UartRxHandler_GetSensorType(i),
        };
    }
    ImuDopSerial_UpdateLiveData(ch_data, MAX_IMU_CHANNELS);
}
