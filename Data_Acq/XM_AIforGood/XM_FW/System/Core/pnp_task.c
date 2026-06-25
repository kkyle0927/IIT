/**
 ******************************************************************************
 * @file    pnp_task.c
 * @author  HyundoKim
 * @brief   PnP Task 구현 - DOP V2 Master PnP + Device 통합 관리
 * @version 2.1.0
 * @date    2026-03-02
 *
 * @details
 * [V2.1 변경사항 - Channel LED 통합]
 * - 각 디바이스 상태를 LedManager로 전달하여 PCA9957 채널 LED 자동 갱신
 *
 * [PnP Task 주기 작업] (100ms)
 * ┌─────────────────────────────────────────────────────────┐
 * │ 1. AGR_PnP_Master_RunPeriodic()                        │
 * │    - Master Heartbeat 전송 (1초마다 1회)                │
 * │    - 모든 Slave Heartbeat Timeout 체크                  │
 * │                                                         │
 * │ 2. Protocol Devices RunPeriodic()                       │
 * │    - CM_Drv_RunPeriodic: DOP V1 Heartbeat + Timeout    │
 * │    - ImuHub_Drv_RunPeriodic: DOP V2 Pre-Op SM          │
 * │                                                         │
 * │ 3. Sensor Devices RunPeriodic() (Auto-Sense)            │
 * │    - MarvelDex_RunPeriodic: UART 데이터 타임아웃 체크    │
 * │    - XsensMTi_RunPeriodic: UART 데이터 타임아웃 체크     │
 * │                                                         │
 * │ 4. Channel LED Update                                   │
 * │    - CAN-FD PnP → NMT State → Channel LED              │
 * │    - GRF UART → Auto-Sense → Channel LED               │
 * │    - USB → Host/Device Ready → Channel LED              │
 * └─────────────────────────────────────────────────────────┘
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "pnp_task.h"

/* System Config */
#include "module.h"

/* Node IDs */
#include "xm_node_id.h"

/* Protocol Device Drivers */
#include "cm_drv.h"
#include "imu_hub_drv.h"
#include "emg_hub_drv.h"
#include "fes_hub_drv.h"

/* Sensor Device Drivers (Auto-Sense) */
#include "mdaf-25-6850.h"
#include "mti-630.h"

/* LED Manager (Channel LED) */
#include "led_manager.h"

/* USB Status */
#include "usb_mode_handler.h"
#include "cdc_handler.h"        // CdcStream_IsStreamingActive()
#include "xm_api_usb.h"         // XM_GetUsbLogStatus()

/* FreeRTOS */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *-----------------------------------------------------------
 */

/* ===== DOP V2: Master PnP 인스턴스 (XM10 노드 단위 1개) ===== */
static AGR_PnP_Master_t s_master_pnp;
static bool s_master_pnp_initialized = false;

/* ===== Task ===== */
static osThreadId_t s_pnp_task_handle = NULL;

static const osThreadAttr_t s_pnp_task_attr = {
    .name = "PnP_Task",
    .stack_size = TASK_STACK_PNP_MANAGER,
    .priority = (osPriority_t)TASK_PRIO_PNP_MANAGER,
};

/* ===== Channel LED 이전 상태 (중복 호출 방지) ===== */
static ChannelDevState_t s_prev_imu_state = CH_DEV_NOT_CONNECTED;
static ChannelDevState_t s_prev_emg_state = CH_DEV_NOT_CONNECTED;
static ChannelDevState_t s_prev_fes_state = CH_DEV_NOT_CONNECTED;
static GrfSensorState_t  s_prev_grf_state[2] = { GRF_NOT_CONNECTED, GRF_NOT_CONNECTED };
static UsbLedState_t     s_prev_usb_state = USB_LED_NOT_CONNECTED;

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void _PnP_TaskEntry(void* argument);
static ChannelDevState_t _NmtToChannelDevState(AGR_NMT_State_t nmt, bool was_connected);
static void _UpdateCanDeviceLed(ChannelLedId_t ch, AGR_NMT_State_t nmt, ChannelDevState_t* prev);
static void _UpdateGrfSensorLed(ChannelLedId_t ch, uint8_t marveldex_ch, GrfSensorState_t* prev);
static void _UpdateUsbLed(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief PnP Task 초기화
 */
void PnP_Task_Init(AGR_PnP_TxFunc_t pnp_tx_func,
                    AGR_PnP_TxFunc_t cm_tx_func,
                    AGR_PnP_GetTickFunc_t get_tick)
{
    if (pnp_tx_func == NULL || cm_tx_func == NULL || get_tick == NULL) {
        return;
    }

    /* ===== 1. DOP V2: Master PnP 초기화 (XM10 노드) =====
     * [Rev2.0] PnP Master Heartbeat(0x702) → Ch2 (Sensor 버스)
     * Why: Sensor 모듈이 XM의 생존을 확인해야 하므로 Ch2로 전송 */
    if (!s_master_pnp_initialized) {
        int result = AGR_PnP_Master_Init(&s_master_pnp,
                                          AGR_NODE_ID_XM,
                                          pnp_tx_func,
                                          get_tick);
        if (result >= 0) {
            s_master_pnp_initialized = true;
        }
    }

    /* ===== 2. Protocol Device 초기화 =====
     * [Rev2.0] CM SDO/PDO → Ch1 (CM 전용 버스) */
    CM_Init(cm_tx_func, SYS_NODE_ID_XM, SYS_NODE_ID_CM);

    /* ===== 3. Sensor Device Auto-Sense 초기화 (Multi-Instance) ===== */
    for (uint8_t ch = 0; ch < MARVELDEX_MAX_INSTANCES; ch++) {
        MarvelDex_StateInit(ch);
    }
    XsensMTi_StateInit(0);  /* XM: XSENS ch=0 고정 */

    /* ===== 4. PnP Task 생성 (100ms 주기) ===== */
    s_pnp_task_handle = osThreadNew(_PnP_TaskEntry, NULL, &s_pnp_task_attr);
}

/**
 * @brief Master PnP 인스턴스 반환
 */
AGR_PnP_Master_t* PnP_Task_GetMaster(void)
{
    return s_master_pnp_initialized ? &s_master_pnp : NULL;
}

/**
 *-----------------------------------------------------------
 * STATIC (PRIVATE) FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief PnP Task Entry Point (100ms 주기)
 */
static void _PnP_TaskEntry(void* argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS_PNP_MANAGER);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* ===== 1. V2 Master PnP RunPeriodic ===== */
        if (s_master_pnp_initialized) {
            AGR_PnP_Master_RunPeriodic(&s_master_pnp);
        }

        /* ===== 2. Protocol Devices RunPeriodic ===== */
        CM_Drv_RunPeriodic();
        ImuHub_Drv_RunPeriodic();
        EmgHub_Drv_RunPeriodic();
        FesHub_Drv_RunPeriodic();

        /* ===== 3. Sensor Devices RunPeriodic (Auto-Sense, Multi-Instance) ===== */
        for (uint8_t ch = 0; ch < MARVELDEX_MAX_INSTANCES; ch++) {
            MarvelDex_RunPeriodic(ch);
        }
        XsensMTi_RunPeriodic(0);

        /* ===== 4. Channel LED Update ===== */
        if (ImuHub_Drv_IsIdentityMismatch()) {
            /* PnP IDENTIFY 불일치 — 잘못된 디바이스/계약버전 (Red Triple Flash).
             * NMT 상태 매핑보다 우선 — 침묵 실패 방지 (fail loud). EMG/FES 패턴 정합. */
            if (s_prev_imu_state != CH_DEV_WRONG_DEVICE) {
                s_prev_imu_state = CH_DEV_WRONG_DEVICE;
                LedManager_SetCanDeviceState(CH_LED_IMU, CH_DEV_WRONG_DEVICE);
            }
        } else {
            _UpdateCanDeviceLed(CH_LED_IMU, ImuHub_Drv_GetNmtState(), &s_prev_imu_state);
        }
        if (EmgHub_Drv_IsIdentityMismatch()) {
            /* PnP IDENTIFY 불일치 — 잘못된 디바이스/계약버전 (Red Triple Flash).
             * NMT 상태 매핑보다 우선 — 침묵 실패 방지 (fail loud). FES 패턴 정합. */
            if (s_prev_emg_state != CH_DEV_WRONG_DEVICE) {
                s_prev_emg_state = CH_DEV_WRONG_DEVICE;
                LedManager_SetCanDeviceState(CH_LED_EMG, CH_DEV_WRONG_DEVICE);
            }
        } else {
            _UpdateCanDeviceLed(CH_LED_EMG, EmgHub_Drv_GetNmtState(), &s_prev_emg_state);
        }
        if (FesHub_Drv_IsIdentityMismatch()) {
            /* PnP IDENTIFY 불일치 — 잘못된 디바이스/계약버전 (Red Triple Flash).
             * NMT 상태 매핑보다 우선 — 침묵 실패 방지 (fail loud). */
            if (s_prev_fes_state != CH_DEV_WRONG_DEVICE) {
                s_prev_fes_state = CH_DEV_WRONG_DEVICE;
                LedManager_SetCanDeviceState(CH_LED_FES, CH_DEV_WRONG_DEVICE);
            }
        } else {
            _UpdateCanDeviceLed(CH_LED_FES, FesHub_Drv_GetNmtState(), &s_prev_fes_state);
        }
        /* 향후 확장:
         * _UpdateCanDeviceLed(CH_LED_HMMG, HmmgHub_Drv_GetNmtState(), &s_prev_hmmg_state);
         */

        _UpdateGrfSensorLed(CH_LED_GRF_L, MARVELDEX_CH_LEFT, &s_prev_grf_state[0]);
        _UpdateGrfSensorLed(CH_LED_GRF_R, MARVELDEX_CH_RIGHT, &s_prev_grf_state[1]);

        CdcStream_CheckForDisconnect();  /* DTR=0 미지원 호스트 fallback */
        _UpdateUsbLed();
    }
}

/* ===== C1: CAN-FD PnP LED Helper ===== */

/**
 * @brief AGR_NMT_State_t → ChannelDevState_t 변환
 */
static ChannelDevState_t _NmtToChannelDevState(AGR_NMT_State_t nmt, bool was_connected)
{
    switch (nmt) {
        case AGR_NMT_BOOT_UP:
            return was_connected ? CH_DEV_DETECTING : CH_DEV_NOT_CONNECTED;

        case AGR_NMT_PRE_OPERATIONAL:
            return CH_DEV_PRE_OPERATIONAL;

        case AGR_NMT_OPERATIONAL:
            return CH_DEV_OPERATIONAL;

        case AGR_NMT_STOPPED:
            return was_connected ? CH_DEV_HEARTBEAT_LOST : CH_DEV_NOT_CONNECTED;

        default:
            return CH_DEV_NOT_CONNECTED;
    }
}

static void _UpdateCanDeviceLed(ChannelLedId_t ch, AGR_NMT_State_t nmt, ChannelDevState_t* prev)
{
    bool was_connected = (*prev >= CH_DEV_DETECTING && *prev <= CH_DEV_OPERATIONAL);
    ChannelDevState_t new_state = _NmtToChannelDevState(nmt, was_connected);

    if (new_state != *prev) {
        *prev = new_state;
        LedManager_SetCanDeviceState(ch, new_state);
    }
}

/* ===== C2: GRF UART Auto-Sense LED Helper ===== */

static void _UpdateGrfSensorLed(ChannelLedId_t ch, uint8_t marveldex_ch, GrfSensorState_t* prev)
{
    GrfSensorState_t new_state;

    if (MarvelDex_IsOnline(marveldex_ch)) {
        MarvelDex_packet_t pkt;
        if (MarvelDex_GetLatest(marveldex_ch, &pkt)) {
            if (pkt.batteryLevel < 5) {
                new_state = GRF_CRITICAL_BATTERY;
            } else if (pkt.batteryLevel < 20) {
                new_state = GRF_LOW_BATTERY;
            } else if (pkt.statusFlags != 0) {
                new_state = GRF_SENSOR_ERROR;
            } else {
                new_state = GRF_DATA_ACTIVE;
            }
        } else {
            new_state = GRF_DETECTING;
        }
    } else {
        new_state = GRF_NOT_CONNECTED;
    }

    if (new_state != *prev) {
        *prev = new_state;
        LedManager_SetGrfSensorState(ch, new_state);
    }
}

/* ===== C3: USB LED Helper ===== */

static void _UpdateUsbLed(void)
{
    UsbLedState_t new_state;

    if (g_isUsbHostReady) {
        /* MSC Path: 로깅 상태에 따라 세부 분기 */
        XmLogStatus_e log_st = XM_GetUsbLogStatus();
        if (log_st == XM_LOG_STATUS_ERROR_STOPPED) {
            new_state = USB_LED_ERROR;
        } else if (log_st == XM_LOG_STATUS_WARNING_QUEUE_FULL) {
            new_state = USB_LED_MSC_NEARLY_FULL;
        } else if (log_st == XM_LOG_STATUS_LOGGING) {
            new_state = USB_LED_MSC_LOGGING;
        } else {
            new_state = USB_LED_MSC_READY;
        }
    } else if (g_isUsbDeviceReady) {
        /* CDC Path: Streaming 활성 여부로 분기 */
        if (CdcStream_IsStreamingActive()) {
            new_state = USB_LED_CDC_DATA_ACTIVE;
        } else {
            new_state = USB_LED_CDC_CONNECTED;
        }
    } else {
        new_state = USB_LED_NOT_CONNECTED;
    }

    if (new_state != s_prev_usb_state) {
        s_prev_usb_state = new_state;
        LedManager_SetUsbLedState(new_state);
    }
}
