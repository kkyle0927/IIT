/**
 ******************************************************************************
 * @file    imu_sys_service.c
 * @author  HyundoKim
 * @brief   [Application Layer] IMU Hub System Service - LED, PnP, DOP Serial Rx
 * @version 3.0 (단일 MainTask 로 fold — FDCAN Tx single-writer)
 * @date    2026-03-27
 *
 * @details
 * (구) 독립 TIM7 System Task. 이제 단일 MainTask(TIM16) ISR 의 ImuHub_Task_RunLoop()
 * 가 매 tick 호출하는 비실시간 시스템 서비스. tick 기반 주기 분기로 서브시스템 실행.
 *
 * [주기 분기]
 * - 매 tick:  DOP Serial Rx (CDC → COBS decode → SDO routing)
 * - 매 tick:  DOP Periodic (Heartbeat 1s)
 * - 100ms:   LED 상태 자동 매핑, Debug Monitor 업데이트
 * - 500ms:   PnP Periodic (NMT/Heartbeat)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_sys_service.h"
#include "imu_hub_apps.h"  /* g_sys, g_perf_stats, g_uart_debug, g_uart_status */
#include "imu_cdc.h"
#include "imu_dop_serial.h"
#include "module.h"
#include "system_startup.h"  /* System_GetFdcanHandle() */
#include "io_manager.h"
#include "xm_drv.h"
#include "uart_rx_handler.h"
#include "ebimu-9dofv6.h"
#include "mti-630.h"
#include "ioif_agrb_tim.h"
#include "ioif_agrb_fdcan.h"
#include "ioif_agrb_uart.h"

#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

#define LED_UPDATE_PERIOD_MS       100   /**< LED 상태 매핑 주기 */
#define DEBUG_UPDATE_PERIOD_MS     100   /**< Debug Monitor 업데이트 주기 */
#define AUTO_SENSE_PERIOD_MS       100   /**< IMU Auto-Sense Timeout 체크 주기 */

/**
 *-----------------------------------------------------------
 * EXTERN (system_startup.c에서 정의)
 *-----------------------------------------------------------
 */

extern IOIF_FDCANx_t g_fdcan2_id;

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static uint32_t s_last_pnp_run_ms = 0;
static uint32_t s_last_led_update_ms = 0;
static uint32_t s_last_debug_update_ms = 0;
static uint32_t s_last_auto_sense_ms = 0;

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void _UpdateLedState(void);
static void _UpdateDebugMonitors(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief System Service 초기화 (1회 실행)
 */
void ImuSysService_Init(void)
{
    s_last_pnp_run_ms = 0;
    s_last_led_update_ms = 0;
    s_last_debug_update_ms = 0;
    s_last_auto_sense_ms = 0;
}

/**
 * @brief System Service 실행 (비RT) — MainTask(TIM16) 루프에서 매 tick 호출
 */
void ImuSysService_Run(void)
{
    uint32_t now = IOIF_TIM_GetTick();

    /* ===== [매 tick] DOP Serial Rx (CDC → COBS → SDO routing) ===== */
    {
        uint8_t rx_buf[64];
        uint32_t rx_len;
        while ((rx_len = ImuCdc_DrainRxRaw(rx_buf, sizeof(rx_buf))) > 0) {
            ImuDopSerial_FeedRxData(rx_buf, rx_len);
        }
    }

    /* ===== [매 tick] DOP Periodic (Heartbeat) ===== */
    ImuDopSerial_RunPeriodic(now);

    /* ===== [100ms] LED 상태 자동 매핑 ===== */
    if (now - s_last_led_update_ms >= LED_UPDATE_PERIOD_MS) {
        _UpdateLedState();
        s_last_led_update_ms = now;
    }

    /* ===== [100ms] Debug Monitor 업데이트 ===== */
    if (now - s_last_debug_update_ms >= DEBUG_UPDATE_PERIOD_MS) {
        _UpdateDebugMonitors();
        s_last_debug_update_ms = now;
    }

    /* ===== [100ms] IMU Auto-Sense Timeout 체크 =====
     * 각 채널별 Auto-Sense 타이머를 틱하여 최근 수신이 없으면
     * OPERATIONAL → STOPPED 전환. IsConnected=false로 XM10에 통지. */
    if (now - s_last_auto_sense_ms >= AUTO_SENSE_PERIOD_MS) {
        for (uint8_t ch = 0; ch < MAX_IMU_CHANNELS; ch++) {
            EbimuV6_RunPeriodic(ch);
            XsensMTi_RunPeriodic(ch);
        }
        /* [R2] Hot-Swap: 위 timeout 체크로 갱신된 IsOnline 기반으로 Lock 해제/재감지 */
        UartRxHandler_RunPeriodic();
        s_last_auto_sense_ms = now;
    }

    /* ===== [500ms] PnP Periodic (NMT/PnP/Heartbeat) — PNP_RUN_PERIOD_MS from module.h ===== */
    if (now - s_last_pnp_run_ms >= PNP_RUN_PERIOD_MS) {
        XM_Drv_RunPeriodic();
        s_last_pnp_run_ms = now;
    }

    /* ===== [매 tick] I/O Manager (LED 패턴 + Button 디바운스) ===== */
	IO_Manager_Update(now);
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTIONS — LED State Mapping
 *-----------------------------------------------------------
 */

/**
 * @brief NMT 상태 → LED 상태 자동 매핑 (CiA 303-3)
 */
static void _UpdateLedState(void)
{
    if (g_sys.xm10.nmt_state == AGR_NMT_OPERATIONAL && !g_sys.xm10.is_connected) {
        /* Slave는 OPERATIONAL이나 Master HB 타임아웃 */
        IO_SetLinkState(IO_LINK_STATE_HEARTBEAT_LOST);
    } else if (g_sys.xm10.nmt_state == AGR_NMT_OPERATIONAL) {
        IO_SetLinkState(IO_LINK_STATE_OPERATIONAL);
    } else if (g_sys.xm10.nmt_state == AGR_NMT_PRE_OPERATIONAL) {
        IO_SetLinkState(IO_LINK_STATE_PRE_OPERATIONAL);
    } else {
        IO_SetLinkState(IO_LINK_STATE_DISCONNECTED);
    }
}

/**
 * @brief [Live Expression용] 디버깅 모니터 업데이트 (100ms 주기)
 */
static void _UpdateDebugMonitors(void)
{
    /* UART 상태 */
    for (int i = 0; i < MAX_IMU_CHANNELS; i++) {
        IOIF_UART_DebugInfo_t info;
        if (IOIF_UART_GetDebugInfo(i, &info) == AGRBStatus_OK) {
            g_uart_debug.uart[i].rx_state           = info.rx_state;
            g_uart_debug.uart[i].error_code         = info.error_code;
            g_uart_debug.uart[i].dma_cndtr          = info.dma_cndtr;
            g_uart_debug.uart[i].rx_bytes            = 2048 - info.dma_cndtr;
            g_uart_debug.uart[i].idle_callback_cnt  = info.idle_callback_cnt;
            g_uart_debug.uart[i].error_callback_cnt = info.error_callback_cnt;
            g_uart_debug.uart[i].idle_ie_enabled    = info.idle_ie_enabled;
            g_uart_debug.uart[i].dmar_enabled        = info.dmar_enabled;
            g_uart_debug.uart[i].connected           = g_sys.imus[i].is_connected;
        }
    }

    /* 센서 설정 통계 */
    {
        volatile uint32_t ebimu_sent = 0, ebimu_failed = 0;
        volatile uint32_t xsens_sent = 0, xsens_failed = 0;
        EBIMU_GetConfigStats(&ebimu_sent, &ebimu_failed);
        XsensMTi_GetConfigStats(&xsens_sent, &xsens_failed);
        g_uart_debug.sensor_config_sent   = ebimu_sent  + xsens_sent;
        g_uart_debug.sensor_config_failed = ebimu_failed + xsens_failed;
    }

    /* XM10 상태 */
    g_uart_debug.xm10.connected = g_sys.xm10.is_connected;
    g_uart_debug.xm10.pnp_state = (uint8_t)g_sys.xm10.nmt_state;
    g_uart_debug.update_tick     = g_sys.tick_cnt;

    /* 동작 중인 UART 확인 (idle_callback_cnt > 0) */
    g_uart_status.uart0_working = (g_uart_debug.uart[0].idle_callback_cnt > 0);
    g_uart_status.uart1_working = (g_uart_debug.uart[1].idle_callback_cnt > 0);
    g_uart_status.uart2_working = (g_uart_debug.uart[2].idle_callback_cnt > 0);
    g_uart_status.uart3_working = (g_uart_debug.uart[3].idle_callback_cnt > 0);
    g_uart_status.uart4_working = (g_uart_debug.uart[4].idle_callback_cnt > 0);
    g_uart_status.uart5_working = (g_uart_debug.uart[5].idle_callback_cnt > 0);
    g_uart_status.working_count =
        g_uart_status.uart0_working + g_uart_status.uart1_working +
        g_uart_status.uart2_working + g_uart_status.uart3_working +
        g_uart_status.uart4_working + g_uart_status.uart5_working;

    /* ===== FDCAN Tx 진단 (Live Expression용) ===== */
    IOIF_FDCANx_t fdcan_id = System_GetFdcanHandle();

    /* Bus 상태 카운터 (Bus-Off, Error Passive 누적 + last TEC snapshot) */
    {
        IOIF_FDCAN_ErrorStats_t es = {0};
        (void)IOIF_FDCAN_GetErrorStats(fdcan_id, &es);
        g_perf_stats.fdcan_bus_off_count          = es.bus_off_count;
        g_perf_stats.fdcan_err_passive_count      = es.error_passive_count;
        g_perf_stats.fdcan_last_tec_at_err_passive = es.last_tec_at_error_passive;
    }

    /* TEC/REC (HW 레지스터) */
    {
        uint8_t tec = 0, rec = 0;
        IOIF_FDCAN_GetErrorCounters(fdcan_id, &tec, &rec);
        g_perf_stats.fdcan_tec = tec;
        g_perf_stats.fdcan_rec = rec;
    }

    /* HB/SDO/EMCY 경로 통계 (Device 레이어에서 pull) */
    {
        uint32_t hb_d = 0, hb_q = 0, sdo_d = 0, sdo_q = 0, em_d = 0, em_q = 0;
        XM_Drv_GetTxPathStats(&hb_d, &hb_q, &sdo_d, &sdo_q, &em_d, &em_q);
        g_perf_stats.hb_direct_tx_count   = hb_d;
        g_perf_stats.hb_queued_count      = hb_q;
        g_perf_stats.sdo_rsp_direct_count = sdo_d;
        g_perf_stats.sdo_rsp_queued_count = sdo_q;
        g_perf_stats.emcy_direct_count    = em_d;
        g_perf_stats.emcy_queued_count    = em_q;
    }

    /* PnP path HB/Bootup 계측 (실제 HB 는 PnP Slave → System_Fdcan2_Transmit
     * 경로로만 흐르므로 DOP path 통계는 항상 0. 실제 전송 횟수는 여기). */
    {
        uint32_t pnp_hb = 0, pnp_boot = 0;
        System_GetFdcan2PnpTxStats(&pnp_hb, &pnp_boot);
        g_perf_stats.hb_pnp_direct_count = pnp_hb;
        g_perf_stats.bootup_pnp_count    = pnp_boot;
    }

    /* SW Tx Queue 통계 (IOIF 에서 pull — Live Expression 진단)
     * upstream IOIF_FDCAN_GetQueueStats 시그니처: uint8_t* now/peak, uint32_t* drop
     * (SW_TX_QUEUE_SIZE ≤ 255 전제, MAX override 여유 있음) */
    {
        uint8_t  depth_now = 0, depth_peak = 0;
        uint32_t drop = 0;
        (void)IOIF_FDCAN_GetQueueStats(fdcan_id, &depth_now, &depth_peak, &drop);
        g_perf_stats.sw_queue_depth_now  = depth_now;
        g_perf_stats.sw_queue_depth_peak = depth_peak;
        g_perf_stats.sw_queue_drop_count = drop;
    }

    /* Drop PPM 계산 (overrun / attempt × 1,000,000)
     * overflow 방지: attempt가 약 4200 넘으면 overrun × 1000000 overflow → 64bit */
    if (g_perf_stats.tpdo_tx_attempt_count > 0U) {
        uint64_t ppm = ((uint64_t)g_perf_stats.tpdo_overrun_count * 1000000ULL)
                       / g_perf_stats.tpdo_tx_attempt_count;
        g_perf_stats.tpdo_drop_ppm = (uint32_t)ppm;
    }
}
