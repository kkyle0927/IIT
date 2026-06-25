/**
 ******************************************************************************
 * @file    xm_api_usb.c
 * @author  HyundoKim
 * @brief   USB 기능 통합 관리 구현부
 * @details 
 * 1. MSC (로깅): 사용자가 등록한 구조체를 Binary 형태로 파일 저장
 * 2. CDC (디버그): PC 터미널과의 양방향 통신 지원
 * 이 모든 기능은 core_process에 의해 2ms 주기로 자동 처리됩니다.
 * @version 0.1
 * @date    Nov 17, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api_usb.h"
#include "xm_api_data.h"  // Default Source (XM)
// System Layer 내부 모듈 포함
#include "data_logger.h"          // DataLogger_Start, DataLogger_Stop
#include "cdc_handler.h"          // CdcStream_Send, CdcStream_SetState
#include "cdc_dop_router.h"       // CDC↔DOP Serial 라우터 (sensor-studio GUI)
#include "xm_periph_stimulus.h"   // 페리페럴 stimulus 생성기 (회로 SI 측정용)
#include "phai_packet_builder.h"  // PhAI V2 프로토콜 패킷 빌더
#include "usb_mode_handler.h"
#include "xm_total_data.h"       // Total Data Packet (System-Managed)
#include "led_manager.h"         // PRODUCTION 모드 진입/이탈 시 user LED 패턴 갱신
#include "FreeRTOS.h"
#include "task.h"
#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */


/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// MSC (Logging) Source
static void* s_log_src_ptr = NULL;  // 기본값: NULL
static uint32_t s_log_src_size = 0;

// CDC (Streaming) Source
static void* s_stream_src_ptr = NULL;
static uint32_t s_stream_src_size = 0;

// PhAI V2: 스트리밍 Module ID (User가 변경 가능, 기본 COMBINED)
static uint8_t s_stream_module_id = PHAI_MODULE_COMBINED;

// User Custom Metadata (Module ID 0xEF, 연결 시 1회 전송)
static const char* s_custom_meta_json = NULL;
static uint8_t     s_custom_meta_target_id = 0;
static bool        s_meta_sent = false;

// 이전 연결 상태 추적 (재연결 감지 → Meta 재전송)
static bool s_prev_connected = false;

/* ──────────────────────────────────────────────────────────────────────────
 * USB-CDC 통신 모드 상태 (PhAI / PRODUCTION / TERMINAL)
 * ──────────────────────────────────────────────────────────────────────────
 *  - s_usb_mode        : 현재 모드 (atomic, ISR-safe read)
 *  - s_mode_locked     : 사용자가 명시 SetMode 호출 시 true — 이후 자동 전환
 *                        (cdc_dop_router 의 PRODUCTION latch) 비활성. 예제·
 *                        터미널 사용자가 명시한 모드를 보호.
 *  - s_mode_change_cb  : 모드 변경 알림 콜백. 향후 OD 모드 플래그(0x6000 /
 *                        0x7000) 와의 양방향 동기화 또는 외부 표시(LED 패턴
 *                        등) 트리거에 사용. 미등록 시 무동작.
 */
static volatile atomic_int  s_usb_mode      = (int)XM_USB_MODE_PHAI;
static volatile atomic_bool s_mode_locked   = false;
static XM_USB_ModeChangeCb_t s_mode_change_cb = NULL;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ==========================================================================
 * Registration Functions
 * ========================================================================== */

void XM_SetUsbLogSource(void* data_ptr, uint32_t size)
{
    if (data_ptr != NULL && size > 0) {
        s_log_src_ptr = data_ptr;
        s_log_src_size = size;
        DataLogger_SetPacketSize(size);
    }
}

void XM_SetUsbStreamSource(void* data_ptr, uint32_t size)
{
    if (data_ptr != NULL && size > 0) {
        s_stream_src_ptr = data_ptr;
        s_stream_src_size = size;
    }
}

/* ==========================================================================
 * Data Logger Implementation
 * ========================================================================== */

/**
 * @brief USB 로깅 준비 상태를 확인합니다.
 */
bool XM_IsUsbLogReady(void)
{
    // Facade 패턴: 실제 구현은 DataLogger 모듈에 위임
    return DataLogger_IsReady(); // (g_isUsbHostReady & fs_ready) 확인
}

/**
 * @brief 로깅 시작 명령을 System Layer(DataLogger)로 전달합니다.
 */
bool XM_StartUsbDataLog(const char* sessionName, const char* metadata)
{
    if (!XM_IsUsbLogReady()) return false;
    // Facade 패턴: 실제 구현은 DataLogger 모듈에 위임합니다.
    // 이 함수가 내부 상태를 LOG_STATUS_LOGGING로 변경함
    return DataLogger_Start(sessionName, metadata);
}

/**
 * @brief 로깅 중지 명령을 System Layer(DataLogger)로 전달합니다.
 */
void XM_StopUsbDataLog(void)
{
    if (XM_IsUsbLogReady()) DataLogger_Stop();
}

/**
 * @brief [Option A 2026-04-17] 현재/마지막 활성 USB 세션 이름을 조회합니다.
 * @details Emergency Stop 후 재시작 시 같은 폴더에 이어쓰기 용도.
 *          최초 진입 시 ""을 전달 → DataLogger가 boot_count 기반 이름 생성 → 이 API로 조회 →
 *          재진입 시 같은 이름 재전달 → FW의 session_index 자동 증분 (data_001_*, data_002_* ...)
 * @param[out] out_buf  세션 이름 복사 버퍼
 * @param[in]  buf_size 버퍼 크기
 */
void XM_GetActiveUsbSessionName(char* out_buf, uint32_t buf_size)
{
    DataLogger_GetActiveSessionName(out_buf, buf_size);
}

// /**
//  * @brief [실시간] 2ms 루프의 로그 데이터를 System Layer(DataLogger)로 전달합니다.
//  */
// bool LogUsbData(const void* logPacket, uint32_t packetSize)
// {
//     if (!IsUsbLogReady()) return false;
//     // Facade 패턴: 실제 구현은 DataLogger 모듈에 위임합니다.
//     // [핵심] 이 함수는 링 버퍼에 memcpy 후 즉시 반환됩니다. (Non-Blocking)
//     return DataLogger_Log(logPacket, packetSize);
// }

/**
 * @brief [실시간] 로거의 현재 상태를 가져옵니다.
 */
XmLogStatus_e XM_GetUsbLogStatus(void)
{
	if (!XM_IsUsbLogReady()) return XM_LOG_STATUS_IDLE;
    return (XmLogStatus_e)DataLogger_GetStatus();
}

void XM_SetUsbLogAutoTimestamp(bool enabled)
{
    DataLogger_SetAutoTimestamp(enabled);
}

void XM_SetUsbLogRollingSize(uint32_t size_mb)
{
    DataLogger_SetRollingSize(size_mb);
}

bool XM_GetUsbLogStats(XmLogStats_t* out_stats)
{
    if (out_stats == NULL) return false;

    DataLogger_Stats_t internal;
    DataLogger_GetStats(&internal);

    out_stats->total_bytes       = internal.total_bytes_written;
    out_stats->total_records     = internal.total_packets_logged;
    out_stats->dropped_records   = internal.dropped_packets;
    out_stats->write_errors      = internal.write_errors;
    out_stats->hot_buffer_percent  = internal.hot_buffer_peak_percent;
    out_stats->cold_buffer_percent = internal.cold_buffer_peak_percent;
    out_stats->disk_free_mb      = internal.disk_free_mb;
    out_stats->disk_total_mb     = internal.disk_total_mb;

    /* duration = end - start (로깅 중이면 현재 tick 사용) */
    uint32_t end = (internal.end_tick != 0) ? internal.end_tick : xTaskGetTickCount();
    uint32_t start = internal.start_tick;
    out_stats->duration_ms = (start != 0) ? (end - start) : 0;

    return true;
}

uint32_t XM_GetUsbDiskFreeMB(void)
{
    return DataLogger_GetDiskFreeMB();
}

uint32_t XM_GetUsbDiskTotalMB(void)
{
    return DataLogger_GetDiskTotalMB();
}

bool XM_InsertUsbLogMarker(XmLogMarkerType_e type, uint16_t data)
{
    return DataLogger_InsertMarker((uint8_t)type, data);
}

/* ============================================================================
 * USB 스트리밍 API (USB Streaming API) - Device/CDC Mode — PhAI V2
 * ==========================================================================*/

bool XM_IsUsbStreamConnected(void)
{
    return g_isUsbDeviceReady;
}

bool XM_IsUsbStreamingActive(void)
{
    return CdcStream_IsStreamingActive();
}

void XM_SetUsbAutoStream(bool enabled)
{
    CdcStream_SetAutoStreamEnabled(enabled);
}

/* ──────────────────────────────────────────────────────────────────────────
 * USB-CDC Mode API
 * ──────────────────────────────────────────────────────────────────────────
 * 설계 요지:
 *  - PHAI       : 기본값. DTR=1 시 PhAI Total Data auto-pump.
 *  - PRODUCTION : 첫 유효 DOP frame(SDO Request) 도달 시 자동 latch.
 *                 또는 사용자가 명시 호출 가능. PhAI pump 차단 + Heartbeat 송신.
 *  - TERMINAL   : 사용자(예제) 명시 호출 전용. PhAI/Heartbeat 모두 off.
 *                 명시 후 자동 전환 비활성 (s_mode_locked = true).
 *  - DTR off    : cdc_handler 가 latch reset 호출 → PHAI 복귀.
 */

/* PRODUCTION 모드 시각 표시 — User LED 0 (FUNC_LED_1) 사용.
 * DOUBLE_FLASH (CiA 303-3 경고 패턴) — 평시 사용자 LED 와 시각 구분.
 * 종료 시 OFF — 사용자가 Control_Loop 재개 후 setUserLedMode 호출하면 갱신됨.
 */
#define XM_PRODUCTION_LED_INDEX  0u

static void _ApplyModeTransition(XM_USB_Mode_e prev, XM_USB_Mode_e next)
{
    if (prev == next) return;

    /* PhAI pump 게이트는 XM_USB_ProcessPeriodic 안에서 mode 조회로 처리.
     * 여기서는 1) streaming 플래그 강제 차단, 2) 외부 알림 콜백 발사.
     */
    if (next != XM_USB_MODE_PHAI) {
        /* PHAI 가 아닌 모드로 진입 시: 진행 중인 auto-pump 즉시 중지.
         * 단순 SetAutoStreamEnabled(false) 만으로는 latch 안 풀림 → Force.
         */
        CdcStream_ForceStopStreaming();
        CdcStream_SetAutoStreamEnabled(false);
    } else {
        /* PHAI 로 복귀 (DTR re-toggle 경유). auto-pump 재개 허용. */
        CdcStream_SetAutoStreamEnabled(true);
    }

    /* User LED 패턴으로 PRODUCTION 모드 시각화 — 양산/SI 검증 운영자가
     * 보드 자체 LED 만으로 모드 상태를 한눈에 식별. Control_Loop 가 차단되어
     * 평시 사용자 LED 출력이 멈춘 상태와 명확히 구분된다.
     */
    if (next == XM_USB_MODE_PRODUCTION) {
        LedManager_SetUserLedMode(XM_PRODUCTION_LED_INDEX,
                                   LED_MODE_DOUBLE_FLASH, 0u);
    } else if (prev == XM_USB_MODE_PRODUCTION) {
        LedManager_SetUserLedMode(XM_PRODUCTION_LED_INDEX,
                                   LED_MODE_OFF, 0u);
    }

    if (s_mode_change_cb != NULL) {
        s_mode_change_cb(prev, next);
    }
}

XM_USB_Mode_e XM_USB_GetMode(void)
{
    return (XM_USB_Mode_e)atomic_load(&s_usb_mode);
}

void XM_USB_SetMode(XM_USB_Mode_e mode)
{
    XM_USB_Mode_e prev = (XM_USB_Mode_e)atomic_exchange(&s_usb_mode, (int)mode);
    /* 사용자 명시 호출 → lock. PRODUCTION 자동 latch 차단.
     * (PHAI 로 명시 복귀해도 lock 유지 — 사용자 의도 우선)
     */
    atomic_store(&s_mode_locked, true);
    _ApplyModeTransition(prev, mode);
}

void XM_USB_RegisterModeChangeCallback(XM_USB_ModeChangeCb_t cb)
{
    s_mode_change_cb = cb;
}

bool XM_USB_RequestProductionLatch(void)
{
    if (atomic_load(&s_mode_locked)) return false;  /* 사용자 명시 모드 보호 */
    XM_USB_Mode_e prev = (XM_USB_Mode_e)atomic_load(&s_usb_mode);
    if (prev == XM_USB_MODE_PRODUCTION) return false;

    atomic_store(&s_usb_mode, (int)XM_USB_MODE_PRODUCTION);
    _ApplyModeTransition(prev, XM_USB_MODE_PRODUCTION);
    return true;
}

void XM_USB_OnDtrLost(void)
{
    /* DTR=0 (USB 분리 / re-enumerate) 시 PHAI 로 복귀 + lock 해제.
     * cdc_handler 의 CdcStream_OnHostDtrChanged(false) 에서 호출 예정.
     */
    XM_USB_Mode_e prev = (XM_USB_Mode_e)atomic_exchange(&s_usb_mode, (int)XM_USB_MODE_PHAI);
    atomic_store(&s_mode_locked, false);
    _ApplyModeTransition(prev, XM_USB_MODE_PHAI);
}

void XM_SetUsbStreamModuleId(uint8_t module_id)
{
    s_stream_module_id = module_id;
}

bool XM_SendUsbData(const void* data, uint32_t len)
{
    if (!XM_IsUsbStreamConnected()) return false;
    return PhAI_PacketBuild(data, len, s_stream_module_id);
}

bool XM_SendUsbDebugMessage(const char* message)
{
    if (!XM_IsUsbStreamConnected()) return false;
    /* 디버그 메시지는 Raw 텍스트 전송 (PhAI 패킷 래핑 없이) */
    return CdcStream_Send(message, strlen(message));
}

uint32_t XM_GetUsbData(void* buffer, uint32_t max_len)
{
    if (!XM_IsUsbStreamConnected()) return 0;
    return CdcStream_Read(buffer, max_len);
}

/* ==========================================================================
 * System Interface (Called by Core Process)
 * ========================================================================== */

void XM_USB_ProcessPeriodic(void)
{
    /* 0. CDC Rx → AGR DOP Serial 라우터 (sensor-studio GUI)
     *    유효 DOP 프레임 수신 시 PhAI Auto-Stream 은 라우터가 자동 차단.
     *    GUI 미연결 시에는 단순히 RX 큐를 비우는 효과.
     */
    (void)CdcDopRouter_Process();

    /* 0.5 페리페럴 stimulus 는 별도 task (StimulusTask, 100ms 주기, BelowNormal)
     *     로 분리 — 여기서 호출하지 않음.
     *
     *     [2026-05-14 재활성] 보드 교체 후 무한리셋 원인이 HW (POR/BOR 매번
     *     발생, .noinit SRAM 매 reset 마다 random) 임이 확정됨. 본 함수 호출
     *     자체는 g_xm_status_connected[] 배열 mirror (atomic write 4건) 만
     *     수행하므로 reset 유발 가능성 없음. sensor-studio SI Toggle 패널의
     *     connected pill (USB CDC / FDCAN1 CM / FDCAN2 SM / UART GRF) 표시
     *     복구 목적으로 재활성. 관련: [[project_v211_cdc_reset_hang_diagnosis]].
     */
    XM_Stimulus_UpdateConnectedStatus();

    /* 1. MSC Data Logging */
    DataLogger_Status_e log_status = DataLogger_GetStatus();
    if (log_status == LOG_STATUS_LOGGING || log_status == LOG_STATUS_WARNING_QUEUE_FULL
        || log_status == LOG_STATUS_WARNING_DISK_LOW) {
        if (s_log_src_ptr != NULL && s_log_src_size > 0) {
            DataLogger_Log(s_log_src_ptr, s_log_src_size);
        }
    }

    /* 2. CDC Data Streaming (PhAI V2 프로토콜) */
    bool is_hw_ready = XM_IsUsbStreamConnected();
    bool is_logic_active = XM_IsUsbStreamingActive();

    /* 재연결 감지 → Meta 재전송 플래그 리셋 + StimulusTask resume.
     * sensor-studio 미연결 동안 stimulus task 는 자가 suspend 상태 (dorman).
     * connect rising edge 에서 vTaskResume 으로 깨우면 stimulus step 100ms 주기
     * 진입. disconnect 는 task 본문이 자가 polling 으로 감지 후 모든 stimulus
     * OFF + vTaskSuspend(NULL) — 외부 hook 별도 호출 불필요.
     */
    if (is_hw_ready && !s_prev_connected) {
        s_meta_sent = false;
        XM_Stimulus_OnUsbConnect();
    }
    s_prev_connected = is_hw_ready;

    /* PhAI Auto-Stream 은 PHAI 모드에서만 동작.
     * PRODUCTION : COBS+CRC wire 점유 (sensor-studio) → PhAI 바이너리 송출 금지.
     * TERMINAL   : 예제 사용자가 깨끗한 텍스트 IO 만 원함 → 자동 송출 금지.
     */
    XM_USB_Mode_e mode = XM_USB_GetMode();

    if (is_hw_ready && is_logic_active && mode == XM_USB_MODE_PHAI) {
        /* 2-1. User Custom Meta — 연결 후 1회 전송 (Module ID 0xEF) */
        if (!s_meta_sent && s_custom_meta_json != NULL) {
            /* Wire format: [target_module_id:1B] [json_bytes...] */
            uint32_t json_len = strlen(s_custom_meta_json);
            uint8_t meta_buf[513]; /* 1B target + 512B max JSON */
            if (json_len <= 512) {
                meta_buf[0] = s_custom_meta_target_id;
                memcpy(&meta_buf[1], s_custom_meta_json, json_len);
                if (PhAI_PacketBuild(meta_buf, 1 + json_len, PHAI_MODULE_USER_META)) {
                    s_meta_sent = true;
                }
            }
        }

        /* 2-2. System Total Data — 매 tick 자동 전송 (Module ID 0x20) */
        uint32_t total_size;
        const XM_TotalDataPacket_t* pkt = XM_TotalData_GetLatest(&total_size);
        PhAI_PacketBuild(pkt, total_size, PHAI_MODULE_TOTAL_DATA);

        /* 2-3. Legacy user stream source (과도기 호환, deprecated) */
        if (s_stream_src_ptr != NULL && s_stream_src_size > 0) {
            PhAI_PacketBuild(s_stream_src_ptr, s_stream_src_size, s_stream_module_id);
        }
    }
}

/* ==========================================================================
 * User Custom Data API (신규)
 * ========================================================================== */

void XM_SetUsbCustomMeta(uint8_t module_id, const char* json_str)
{
    s_custom_meta_target_id = module_id;
    s_custom_meta_json = json_str;
    s_meta_sent = false; /* 재등록 시 재전송 */
}

bool XM_SendUsbDataWithId(const void* data, uint32_t len, uint8_t module_id)
{
    if (!XM_IsUsbStreamConnected()) return false;
    return PhAI_PacketBuild(data, len, module_id);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */
