/**
 ******************************************************************************
 * @file    usb_mode_handler.c
 * @author  HyundoKim
 * @brief   [System Layer] USB 모드(Host/Device) 전환 및 정책 관리
 * @version 0.1
 * @date    Nov 5, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "usb_mode_handler.h"
#include "module.h" // 태스크 속성 매크로를 위해
#include "main.h"   // GPIO pin defines (USB_OTG_UFP_ID_Pin 등)

// [수정] 리팩토링된 IOIF 모듈 포함
#include "ioif_agrb_usb.h"
#include "ioif_agrb_fs.h"
#include "usbh_core.h"  // HOST_USER_* 이벤트 ID (IOIF 3-mode 리팩토링 후 transitive include 끊김)

#include "external_io.h"
#include "data_logger.h"
#include "cdc_handler.h"
#include "phai_packet_builder.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

#define _USB_STABLIZE_PERIOD_MS         (200) // 200ms

#define _CHATTERING_COUNT_THRESHOLD     (20) // 20회 연속 동일 상태 감지 시 확정

/* DRP (Dual Role Port) 토글 파라미터.
 * USB Type-C spec tCCDebounce(Source) = 100~200ms → PC가 XM의 Rd를 안정
 * 상태로 판단하기까지 필요한 시간. Rd phase가 이보다 짧으면 PC는 "CC 불안정"
 * 으로 보고 VBUS 공급을 보류함. 따라서 Rd phase는 스펙 최대값인 200ms 이상
 * 유지하고, 감지가 진행 중이면 toggle을 보류하여 attach 완료를 보장한다.
 *
 * - C-to-C CDC: Rd phase(200ms)에서 PC tCCDebounce 완료 → VBUS 공급 → 감지
 * - A-to-C CDC: VBUS 상시 공급이라 phase 무관하게 Device 감지
 * - USB 메모리 MSC: Rp phase에서 상대 Rd를 CC ADC로 감지 → Host 진입
 */
#define _DRP_SRC_PHASE_MS               (120) // Rp 활성 시간 (MSC 감지 + debounce 여유)
#define _DRP_SNK_PHASE_MS               (200) // Rd 활성 시간 (PC tCCDebounce 최대값)

/* Try.SNK 타임아웃 (USB Type-C spec tTryTimeout = 550ms).
 * Rp phase에서 host signal 감지 시 "진짜 USB 메모리"인지 "PC DRP의 Rd phase"인지
 * 판별하기 위해 Rd로 전환 후 VBUS를 기다리는 시간.
 *   - VBUS 수신 → PC였음 → Device 확정
 *   - 타임아웃 → USB 메모리였음 → Host 확정
 * PC tCCDebounce(100~200ms) + tVBUSOn(~275ms) 합계보다 길어야 함. */
#define _TRY_SNK_TIMEOUT_MS             (500)

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

// [신규] CDC 수신 큐에서 사용할 패킷 구조체
typedef struct {
    uint32_t length;
    uint8_t  data[64]; // (최대 64바이트 패킷 가정, 필요시 APP_RX_DATA_SIZE 사용)
} CDCRxPacket_t;

typedef enum {
    USB_ConnectedState_Idle = 0,
    USB_ConnectedState_Device,
    USB_ConnectedState_Host,   

    USB_ConnectedState_Unknown,

} USB_ConnectedState_e; // 현재 USB 연결 상태

typedef struct {
    USB_ConnectedState_e state;
    bool state_changed;
    TaskUSBControlTask_Init_t init;
    
    struct {
        bool vbus;
        bool enable;
        bool ufp;
    } pin_state;

    IOIF_ADC_ReportData_t latest_cc_value;

    struct { // Mode control state machine
        USB_ConnectedState_e checked;
        uint32_t debounce_count; // 현재 상태와 다른 경우 몇 번 연속 감지되었는지
    } mode_ctrl;

    struct { // DRP toggle state (Idle 상태에서만 유효)
        bool is_src_phase;            // true=Rp(DFP) 활성, false=Rd(UFP) 활성
        TickType_t phase_start_tick;  // 현재 phase 시작 시점

        /* Try.SNK 서브상태: Rp phase에서 host signal 감지 시 Rd로 강제 전환하고
         * VBUS 대기. DRP인 상대(PC)와 UFP-only 상대(USB 메모리)를 구분. */
        bool try_snk_active;
        TickType_t try_snk_start_tick;
    } drp;

    /* [신규] 원본 로직 복원을 위한 초기화 플래그 */
    bool host_initialized;
    bool device_initialized;
} USB_ControlTypeDef;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

// 태스크에 전달된 초기화 파라미터를 저장할 static 변수
static TaskUSBControlTask_Init_t s_usb_init_params;

static USB_ControlTypeDef _debug_usb_control;

// Facade Layer(xm_api.c)에서 참조할 변수들
volatile bool g_isUsbHostReady = false;   // USB 호스트(MSC) 준비 완료 플래그
volatile bool g_isUsbDeviceReady = false; // USB 디바이스(CDC) 준비 완료 플래그
 
// [수정] g_logCmdQueue의 중복 "정의"를 "선언"으로 변경
// (실제 변수는 data_logger.c에 있음)
extern QueueHandle_t g_logCmdQueue;   // USB MSC 쓰기 요청 큐

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// 태스크 속성 (main.c에서 이동)
static osThreadId_t s_usbContolTaskHandle;
static const osThreadAttr_t s_usbContolTask_attributes = {
  .name = "usbContolTask",
  .stack_size = TASK_STACK_USB_CONTROL, // module.h에서 정의
  .priority = (osPriority_t) TASK_PRIO_USB_CONTROL, // module.h에서 정의
};

// debug
static AGRBStatusDef _debug_debvice_init_status = AGRBStatus_NO_RESOURCE;
static AGRBStatusDef _debug_host_init_status = AGRBStatus_NO_RESOURCE;
// static const char* _DEBUG_STR_MESSAGE = "Hello, this is a debug message from IOIF USB Host MSC class.\r\n";

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static USB_ConnectedState_e _check_expected_state(USB_ControlTypeDef* usb);
static USB_ConnectedState_e _check_expected_state_drp(USB_ControlTypeDef* usb);
static void StartUSBControlTask(void *argument);

static void _set_usb_pin_idle_state(USB_ControlTypeDef* usb);
static void _set_usb_pin_host_state(USB_ControlTypeDef* usb);
static void _set_usb_pin_device_state(USB_ControlTypeDef* usb);
static void _restore_ufp_gpio_after_msp_deinit(void);

static void _mode_idle_process(USB_ControlTypeDef* usb);
static void _mode_device_process(USB_ControlTypeDef* usb);
static void _mode_host_process(USB_ControlTypeDef* usb);
static inline bool _is_valid_host_signal(uint32_t value);

// [신규] IOIF 계층에 등록할 콜백 함수들
static void _system_host_callback(uint8_t id);
static void _system_cdc_tx_callback(void);
static void _system_cdc_rx_callback(uint8_t* data, uint32_t length);
static void _system_cdc_dtr_callback(uint8_t dtr);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [신규] USB 제어 모듈 초기화 함수
 */
void USBControl_Init(TaskUSBControlTask_Init_t* init_params)
{
    if (init_params == NULL) {
        // TODO: Error handling
        return;
    }
    
    // 1. system_startup에서 전달받은 파라미터를 static 변수에 복사
    memcpy(&s_usb_init_params, init_params, sizeof(TaskUSBControlTask_Init_t));

    // 2. USB 제어 태스크를 스스로 생성
    s_usbContolTaskHandle = osThreadNew(StartUSBControlTask, NULL, &s_usbContolTask_attributes);
}

void EventUSBNotWorkDetected(void)
{
    UNUSED(0);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static void StartUSBControlTask(void *argument)
{
    TickType_t xLastWakeTime;
    TaskUSBControlTask_Init_t* init = &s_usb_init_params;
    USB_ControlTypeDef usb_control;

    memset(&usb_control, 0, sizeof(USB_ControlTypeDef));
    memcpy(&usb_control.init, init, sizeof(TaskUSBControlTask_Init_t));

    usb_control.state = USB_ConnectedState_Idle;
    usb_control.state_changed = true; // 최초 진입 시 모드 설정을 위해 true
    usb_control.host_initialized = false;   // [신규] 플래그 초기화
    usb_control.device_initialized = false; // [신규] 플래그 초기화

    xLastWakeTime = xTaskGetTickCount();
    for(;;) { 
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS_USB_CONTROL));

        memcpy(&_debug_usb_control, &usb_control, sizeof(USB_ControlTypeDef));

        switch( usb_control.state )
        {
            case USB_ConnectedState_Idle:   _mode_idle_process(&usb_control); break;
            case USB_ConnectedState_Device: _mode_device_process(&usb_control); break;
            case USB_ConnectedState_Host:   _mode_host_process(&usb_control); break;
            default: while(1) { vTaskDelay(1000); } /* Unknown state, hang here */
        }
    }  
}

/**
 * @brief Check the configuration channel state to determine USB connection status
 * 현재 설정과 실제 입력값에 의거하여 연결 상태를 판단한다.
 */
static USB_ConnectedState_e _check_expected_state(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return USB_ConnectedState_Unknown;    

    // CC와 VBUS, 출력 핀의 상태만으로 연결 상태를 판단한다.
    // 현재 상태와 UFP/DFP 핀 상태는 무시한다.
    // UFP/DFP 핀 상태는 실제로 동작 모드를 바꾸는 데 사용된다.

    // VBUS 핀 상태 읽기
    AGRBStatusDef status;
    bool pin_state;

    status = IOIF_GPIO_GET_STATE(usb->init.vbus_id, &pin_state);
    if (status != AGRBStatus_OK) return USB_ConnectedState_Unknown;
    usb->pin_state.vbus = pin_state;

    // ENABLE 핀 상태 읽기
    status = IOIF_GPIO_GET_STATE(usb->init.enable_id, &pin_state);
    if (status != AGRBStatus_OK) return USB_ConnectedState_Unknown;
    usb->pin_state.enable = pin_state;

    // UFP 핀 상태 읽기
    status = IOIF_GPIO_GET_STATE(usb->init.ufp_id, &pin_state);
    if (status != AGRBStatus_OK) return USB_ConnectedState_Unknown;
    usb->pin_state.ufp = pin_state;

    // CC 핀 상태 읽기
    uint16_t cc1 = 0, cc2 = 0;
    ExternalIO_GetUsbCcValues(&cc1, &cc2);
    
    usb->latest_cc_value.value[0] = cc1; // CC1 (PF13, ADC2)
    usb->latest_cc_value.value[1] = cc2; // CC2 (PF14, ADC2)
    // status는 OK로 가정 (ADC2는 Circular 모드로 항상 실행 중)

    //여기서부터 판단을 수행한다.
    //만일 VBUS가 High이지만 ENABLE가 Low라면, 외부로부터 전원이 공급되고 있는 상태이므로 Device 모드로 판단한다.
    if ( usb->pin_state.vbus && ( !usb->pin_state.enable ) ) return USB_ConnectedState_Device;

    /* Rev2.0: CC는 ADC2 (12-bit) */
    /* report.value[0] -> CC1 (PF13, ADC2_INP2) */
    /* report.value[1] -> CC2 (PF14, ADC2_INP6) */
    //만일 CC 채널중 하나 이상이 PULL-DOWN의 임계값 이하라면, 상대방이 PULL-DOWN을 걸고 있는 상태이므로 Host 모드로 판단한다.
    /* --- [수정] Host 모드 감지 (헬퍼 함수 사용) --- */
    if (_is_valid_host_signal(usb->latest_cc_value.value[0]) ||
        _is_valid_host_signal(usb->latest_cc_value.value[1]))
    {
        return USB_ConnectedState_Host;
    }

    //셋 다 아니라면, Idle 상태로 판단한다.
    return USB_ConnectedState_Idle;    
}

/**
 * @brief DRP 토글 중 phase 기반 연결 상태 감지
 * @details Idle 상태 전용. 현재 drp.is_src_phase에 따라 감지 조건 분기:
 *   - Rp phase: VBUS(A-to-C) 또는 CC ADC(USB 메모리 Rd) 감지
 *   - Rd phase: VBUS만 체크 (C-to-C: PC가 Rd 보고 VBUS 공급).
 *               CC ADC는 자기 Rd에 잡혀 무의미 → 건너뜀.
 */
static USB_ConnectedState_e _check_expected_state_drp(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return USB_ConnectedState_Unknown;

    AGRBStatusDef status;
    bool pin_state;

    status = IOIF_GPIO_GET_STATE(usb->init.vbus_id, &pin_state);
    if (status != AGRBStatus_OK) return USB_ConnectedState_Unknown;
    usb->pin_state.vbus = pin_state;

    status = IOIF_GPIO_GET_STATE(usb->init.enable_id, &pin_state);
    if (status != AGRBStatus_OK) return USB_ConnectedState_Unknown;
    usb->pin_state.enable = pin_state;

    uint16_t cc1 = 0, cc2 = 0;
    ExternalIO_GetUsbCcValues(&cc1, &cc2);
    usb->latest_cc_value.value[0] = cc1;
    usb->latest_cc_value.value[1] = cc2;

    /* VBUS 감지는 phase 무관 — A-to-C(Rp phase)도, C-to-C(Rd phase)도 모두 커버 */
    if (usb->pin_state.vbus && !usb->pin_state.enable) {
        return USB_ConnectedState_Device;
    }

    /* CC ADC 기반 Host 감지는 Rp phase에서만 유효.
     * Rd phase에서는 자기 5.1kΩ로 CC가 GND 근처로 당겨져 false positive 발생 우려. */
    if (usb->drp.is_src_phase) {
        if (_is_valid_host_signal(cc1) || _is_valid_host_signal(cc2)) {
            return USB_ConnectedState_Host;
        }
    }

    return USB_ConnectedState_Idle;
}

/* [수정] _mode_host_process 로직 전체를 원본 기준으로 복원 */
static void _mode_host_process(USB_ControlTypeDef* usb)
{
    static uint32_t _chattering_count = 0;
    if (usb == NULL) return;
    if (usb->state != USB_ConnectedState_Host) return;

    ioif_usb_device_deinit();
    _restore_ufp_gpio_after_msp_deinit(); // MspDeInit이 PA10 Analog로 리셋 → 복원
    if (g_isUsbDeviceReady) {
        CdcStream_OnConnectionChanged(false);
    }
    g_isUsbDeviceReady = false;
    usb->device_initialized = false;

    // 2. [상태 진입 시 1회 실행]
    if (usb->state_changed) {
        usb->state_changed = false;
        _set_usb_pin_host_state(usb); // 핀 설정
        usb->host_initialized = false;  // 초기화 플래그 리셋
        _chattering_count = 0;
        vTaskDelay(pdMS_TO_TICKS(_USB_STABLIZE_PERIOD_MS)); // 안정화 대기
        return;
    }

    // 3. [CC 기반 감시 복원]
    // _restore_ufp_gpio_after_msp_deinit()로 DFP/UFP 핀 복원 → CC 값 신뢰 가능.
    // CC debounce + USB 스택 이벤트 이중 안전장치로 Host 해제 판단.

    // 4. [초기화 완료 후 유지/감시]
    if ( usb->host_initialized )
    {
        // 4-1. CC 재검증: 케이블 분리 시 CC 값으로 즉시 감지
        USB_ConnectedState_e expected = _check_expected_state(usb);
        if (expected != USB_ConnectedState_Host) {
            _chattering_count++;
            if (_chattering_count >= _CHATTERING_COUNT_THRESHOLD) {
                usb->state = USB_ConnectedState_Idle;
                usb->state_changed = true;
                usb->host_initialized = false;
                _chattering_count = 0;
            }
            return;
        }
        _chattering_count = 0;

        // 4-2. USB 스택 이벤트 기반 disconnect 감지 (이중 안전장치)
        // g_isUsbHostReady: CLASS_ACTIVE → true, DISCONNECTION → false
        // 초기화 직후~CLASS_ACTIVE 전 구간: g_isUsbHostReady=false (enumeration 중)
        // → _host_was_active로 "한번이라도 연결 성공" 여부를 추적하여 false-positive 방지
        static bool _host_was_active = false;

        if ( g_isUsbHostReady ) {
            _host_was_active = true; // CLASS_ACTIVE 이후 최소 1회 true 확인
        }

        if ( _host_was_active && !g_isUsbHostReady ) {
            // CLASS_ACTIVE 이후 DISCONNECTION 발생 → Idle 전환
            _host_was_active = false;
            usb->state = USB_ConnectedState_Idle;
            usb->state_changed = true;
            usb->host_initialized = false;
            _chattering_count = 0;
        }
        return;
    }

    // 5. [신규] Host 모드가 비활성화되어 있으면 여기서 중단
    if (!usb->init.enable_host_mode) return;

    // [여기입니다] 안정화 후, 초기화가 안 되었다면 1회 호출
    if ( !usb->host_initialized )
    {
        // 레거시 for 루프 제거하고 단순하게 호출
        _debug_host_init_status = ioif_usb_host_init(_system_host_callback);
        if (_debug_host_init_status== AGRBStatus_OK) {
            usb->host_initialized = true;
        }
    }

    return;
}

/* [수정] _mode_device_process 로직 전체를 원본 기준으로 복원 */
static void _mode_device_process(USB_ControlTypeDef* usb)
{
    static uint32_t _chattering_count = 0;
    if (usb == NULL) return;
    if (usb->state != USB_ConnectedState_Device) return;

    // 1. 반대 모드(Host)가 실행 중이면 확실히 중지.
    //    Rev2.0: _restore_ufp_gpio_after_msp_deinit() 호출 금지 — PA15(DFP/UFP)는
    //    PA10(OTG ID)과 분리되어 MspDeInit 영향 없음. 이 함수가 PA15를 LOW(Rp)로
    //    강제하면 Device 모드(Rd 필요)가 파괴되어 PC가 연결 해제함.
    ioif_usb_host_deinit();
    g_isUsbHostReady = false;
    usb->host_initialized = false;

    // 2. [상태 진입 시 1회 실행]
    if (usb->state_changed) {
        usb->state_changed = false;
        _set_usb_pin_device_state(usb);
        usb->device_initialized = false;
        _chattering_count = 0;
        vTaskDelay(pdMS_TO_TICKS(_USB_STABLIZE_PERIOD_MS));
        return;
    }

    // 3. [핀 상태 감시]
    if ( _check_expected_state(usb) != USB_ConnectedState_Device ) {
        _chattering_count++;
        if ( _chattering_count < _CHATTERING_COUNT_THRESHOLD ) return;
        
        usb->state = USB_ConnectedState_Idle;
        usb->state_changed = true;
        _chattering_count = 0;
        return;
    } else {
        _chattering_count = 0;
    }

    // 4. [초기화 1회 실행]
    if ( usb->device_initialized ) return;
    if (!usb->init.enable_device_mode) return;

    // 5. [신규] 리팩토링된 IOIF 함수 호출
    _debug_debvice_init_status = ioif_usb_device_init(_system_cdc_tx_callback, _system_cdc_rx_callback);
    
    if (_debug_debvice_init_status == AGRBStatus_OK) {
        usb->device_initialized = true;
        g_isUsbDeviceReady = true;
        PhAI_PacketBuilder_Init();
        ioif_usb_device_register_dtr_callback(_system_cdc_dtr_callback);
        CdcStream_OnConnectionChanged(true);
    } else {
        g_isUsbDeviceReady = false;
        CdcStream_OnConnectionChanged(false);
    }
    
    return;
}

/* [수정] _mode_idle_process — DRP 토글 + phase-aware 감지 */
static void _mode_idle_process(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return;
    if (usb->state != USB_ConnectedState_Idle) return;

    // 1. [상태 진입 시 1회 실행]
    if (usb->state_changed) {
        ioif_usb_host_deinit();
        ioif_usb_device_deinit();
        _restore_ufp_gpio_after_msp_deinit(); // MspDeInit이 PA10 Analog로 리셋 → 복원

        g_isUsbHostReady = false;
        if (g_isUsbDeviceReady) {
            CdcStream_OnConnectionChanged(false);
        }
        g_isUsbDeviceReady = false;
        usb->host_initialized = false;
        usb->device_initialized = false;

        _set_usb_pin_idle_state(usb); // PA15=LOW(Rp)로 시작, DRP state 초기화

        usb->state_changed = false;
        vTaskDelay(pdMS_TO_TICKS(_USB_STABLIZE_PERIOD_MS));

        /* 안정화 지연 후 phase 기준점 재설정 — 이렇게 하지 않으면
         * 지연 동안 경과한 200ms로 인해 다음 iteration에서 즉시 toggle됨. */
        usb->drp.phase_start_tick = xTaskGetTickCount();
        return;
    }

    TickType_t now = xTaskGetTickCount();

    // 2. [Try.SNK 서브상태] — Rp phase에서 host signal 감지 시 진입.
    //    DRP 상대(PC)와 UFP-only 상대(USB 메모리) 판별.
    if (usb->drp.try_snk_active) {
        bool vbus_state = false;
        AGRBStatusDef st = IOIF_GPIO_GET_STATE(usb->init.vbus_id, &vbus_state);

        if (st == AGRBStatus_OK && vbus_state) {
            // PC가 VBUS 공급 → 상대는 DFP → XM은 Device
            usb->drp.try_snk_active = false;
            usb->state = USB_ConnectedState_Device;
            usb->state_changed = true;
            return;
        }

        if ((now - usb->drp.try_snk_start_tick) >= pdMS_TO_TICKS(_TRY_SNK_TIMEOUT_MS)) {
            // VBUS 없음 → 상대는 UFP-only (USB 메모리) → XM은 Host
            usb->drp.try_snk_active = false;
            usb->state = USB_ConnectedState_Host;
            usb->state_changed = true;
            return;
        }

        return; // Try.SNK 대기 계속
    }

    // 3. [DRP phase 토글] — Rp/Rd 교대. 감지 진행 중이면 toggle 보류.
    TickType_t elapsed = now - usb->drp.phase_start_tick;
    uint32_t phase_dur = usb->drp.is_src_phase ? _DRP_SRC_PHASE_MS : _DRP_SNK_PHASE_MS;

    /* 감지가 진행 중(Host/Device candidate 관찰 중)이면 toggle 보류.
     * 특히 Rd phase에서 PC가 tCCDebounce 후 VBUS를 늦게 공급하는 경우,
     * phase 경계를 넘어가서도 debounce를 완료해야 Device 전환이 확정됨. */
    bool detection_in_progress = (usb->mode_ctrl.checked != USB_ConnectedState_Idle);

    if (elapsed >= pdMS_TO_TICKS(phase_dur) && !detection_in_progress) {
        usb->drp.is_src_phase = !usb->drp.is_src_phase;
        if (usb->drp.is_src_phase) {
            IOIF_GPIO_RESET(usb->init.ufp_id);   // LOW → P-FET ON → Rp 활성
        } else {
            IOIF_GPIO_SET(usb->init.ufp_id);     // HIGH → N-FET ON → Rd 활성
        }
        usb->drp.phase_start_tick = now;
        /* MOSFET 스위칭 + CC 안정화 위해 이번 cycle은 감지 스킵.
         * debounce_count는 이미 0 (detection_in_progress=false 조건). */
        return;
    }

    // 4. [phase-aware 감지]
    USB_ConnectedState_e expected_state = _check_expected_state_drp(usb);

    if (expected_state == USB_ConnectedState_Unknown || expected_state == USB_ConnectedState_Idle) {
        usb->mode_ctrl.debounce_count = 0;
        usb->mode_ctrl.checked = USB_ConnectedState_Idle;
        return;
    }

    if (expected_state != usb->mode_ctrl.checked) {
        usb->mode_ctrl.checked = expected_state;
        usb->mode_ctrl.debounce_count = 0;
        return;
    }

    usb->mode_ctrl.debounce_count++;
    if (usb->mode_ctrl.debounce_count < USB_MODE_CHANGE_DEBOUNCE_COUNT) return;

    // 5. [debounce 완료 — 상태 확정 또는 Try.SNK 진입]
    if (expected_state == USB_ConnectedState_Host) {
        /* Host 후보는 바로 확정하지 않고 Try.SNK로 검증.
         * Rp phase에서 본 low CC는 USB 메모리(Rd) 또는 PC DRP의 Rd phase 둘 다 가능.
         * Rd로 전환하여 VBUS 유무로 최종 판별. */
        usb->drp.try_snk_active = true;
        usb->drp.try_snk_start_tick = now;
        IOIF_GPIO_SET(usb->init.ufp_id);  // Rd로 강제 전환
        usb->mode_ctrl.debounce_count = 0;
        usb->mode_ctrl.checked = USB_ConnectedState_Idle;
        return;
    }

    // Device는 VBUS 직접 근거라 검증 불필요 — 즉시 확정
    usb->state = expected_state;
    usb->state_changed = true;
}

static void _set_usb_pin_idle_state(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return;

    //ENABLE 핀 Low
    IOIF_GPIO_RESET(usb->init.enable_id);
    //UFP 핀 Low (→ Q1/Q2 P-FET ON → Rp 활성, Q3/Q4 N-FET OFF → Rd 비활성)
    IOIF_GPIO_RESET(usb->init.ufp_id);

    /* DRP 토글 state 초기화 — Rp phase로 시작 (기존 MSC 감지 로직과 호환).
     * phase_start_tick은 _mode_idle_process의 안정화 지연 이후 재설정됨. */
    usb->drp.is_src_phase = true;
    usb->drp.phase_start_tick = xTaskGetTickCount();
    usb->drp.try_snk_active = false;
    usb->drp.try_snk_start_tick = 0;

    return;
}

/**
 * @brief PA10(USB_DFP_UFP)을 GPIO Output LOW로 복원
 * @details HAL_PCD/HCD_MspDeInit이 PA10을 Analog(floating)으로 리셋하면
 *          R48(1MΩ) 풀업 → gate ≈ 3.3V → P-FET OFF, N-FET ON → CC ≈ GND
 *          → Host CC 감지 불가 (CDC→MSC 전환 실패 근본 원인).
 *          GPIO Output LOW로 복원하여 Rp 활성(Idle) 상태 유지.
 *          PA11/PA12는 다음 MspInit에서 AF10 재설정되므로 복원 불필요.
 */
static void _restore_ufp_gpio_after_msp_deinit(void)
{
    /* Rev2.0: DFP/UFP = PA15, OTG ID = PA10 (분리됨).
     * MspDeInit은 PA10만 리셋 → PA15(DFP/UFP)에 영향 없으므로
     * Rev1.1과 달리 GPIO 복원이 불필요할 수 있으나, 안전을 위해 유지. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = USB_DFP_UFP_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(USB_DFP_UFP_GPIO_Port, USB_DFP_UFP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(USB_DFP_UFP_GPIO_Port, &gpio);
}

static void _set_usb_pin_host_state(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return;

    //ENABLE 핀 High
    IOIF_GPIO_SET(usb->init.enable_id);
    //UFP 핀 Low
    IOIF_GPIO_RESET(usb->init.ufp_id);

    return;
}

static void _set_usb_pin_device_state(USB_ControlTypeDef* usb)
{
    if (usb == NULL) return;

    //ENABLE 핀 Low
    IOIF_GPIO_RESET(usb->init.enable_id);
    //UFP 핀 High
    IOIF_GPIO_SET(usb->init.ufp_id);

    return;
}

/**
 * @brief [신규] CC핀 값이 유효한 Host(Pull-down) 범위(5%~20%)에 있는지 확인
 * @details 0V 근처의 노이즈(0~5%)를 걸러내고, 유효한 Pull-down 신호만 감지
 * @param[in] value           ADC 측정값
 * @return 유효한 Host 신호이면 true
 */
static inline bool _is_valid_host_signal(uint32_t value)
{
    /* * ADC는 12비트(0-4095)로 가정합니다.
     *
     * 관찰된 실제 값:
     * - Host (연결됨): value[0] = ~570
     * - Idle (미연결): value[0] = 4095, value[1] = 2700~4095 (노이즈)
     *
     * Host 신호(570)와 Idle 노이즈(최소 2700)를 분리해야 합니다.
     */

    /* [수정] 상한선 (THRESHOLD_HIGH)
     * Host(570)보다는 높고, Idle 노이즈(2700)보다는 낮아야 함.
     * -> 1000으로 설정 (안전 마진)
     */
    const uint32_t THRESHOLD_HIGH = 1000;
    
    /* [유지] 하한선 (THRESHOLD_NOISE_FLOOR)
     * 0V 근처의 노이즈를 무시.
     * 5% (약 204)로 설정.
     */
    const uint32_t THRESHOLD_NOISE_FLOOR = (4095 / 20); // 약 204

    /*
     * [검증 로직]
     * Host (~570):  (570 > 204) && (570 < 1000)  -> TRUE (정상)
     * Idle (4095):  (4095 > 204) && (4095 < 1000) -> FALSE (정상)
     * Idle (2700):  (2700 > 204) && (2700 < 1000) -> FALSE (정상)
     */
    return (value > THRESHOLD_NOISE_FLOOR) && (value < THRESHOLD_HIGH);
}

/**
 * @brief [신규] IOIF_USB (Host) 계층으로부터 이벤트를 수신하는 콜백
 */
static void _system_host_callback(uint8_t id)
{
    // 이 함수는 IOIF 계층(ISR 또는 USBH 태스크)에서 호출됨
    switch(id)
    {
        case HOST_USER_SELECT_CONFIGURATION:
            // (필요시 로직 추가)
            break;

        case HOST_USER_DISCONNECTION:
            // [정책] USB 연결이 끊어지면 Host 준비 플래그를 false로 설정
            g_isUsbHostReady = false;
            // [정책] 파일 시스템 모듈에도 연결 끊김을 알림
            ioif_filesystem_HandleHostEvent(false);
            break;

        case HOST_USER_CLASS_ACTIVE:
            // [정책] MSC 클래스가 활성화되면 Host 준비 플래그를 true로 설정
            g_isUsbHostReady = true;
            // [정책] 파일 시스템 모듈에 준비 완료를 알림
            ioif_filesystem_HandleHostEvent(true);
            break;

        case HOST_USER_CONNECTION:
            // (필요시 로직 추가)
            break;

        default:
            break;
    }
}

/**
 * @brief IOIF_USB (Device) 계층으로부터 CDC 데이터를 송신하는 콜백
 */
static void _system_cdc_tx_callback(void)
{
    CdcStream_OnTxComplete();
}

/**
 * @brief IOIF_USB (Device) 계층으로부터 CDC 데이터를 수신하는 콜백
 */
static void _system_cdc_rx_callback(uint8_t* data, uint32_t length)
{
    CdcStream_OnRxReceived(data, length);
}

/**
 * @brief Host DTR 상태 변경 콜백 (ISR 컨텍스트에서 호출)
 * @details Host(PC)가 COM 포트를 열거나 닫을 때 CDC 상태를 리셋하여
 *          XM 리셋 없이 재연결이 가능하도록 합니다.
 */
static void _system_cdc_dtr_callback(uint8_t dtr)
{
    CdcStream_OnHostDtrChanged(dtr);
}
