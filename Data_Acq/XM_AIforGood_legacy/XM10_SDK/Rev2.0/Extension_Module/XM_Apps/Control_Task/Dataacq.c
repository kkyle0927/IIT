/**
 ******************************************************************************
 * @file    Dataacq.c
 * @brief   XM_AIforGood_legacy (Rev1.1) 의 데이터 수집 flow 를 Rev2.0 SDK 로 이식한 메인 TSM.
 *          - OFF / STANDBY / ACTIVE 3-state TSM
 *          - STANDBY 진입 시 XSENS 외부 IMU (MTi-630) 초기화 → TrunkIMU 활성화
 *          - BTN_2 (가운데) 클릭 또는 EXT_DIO_3 (sync) HIGH 로 Recording ON/OFF
 *          - BTN_1 (좌측) 클릭으로 SmartAssist 토글 (H10 동작 모드와 무관)
 *          - Recording ON 동안 SavingData_t 패킷 USB CDC 전송
 *
 *  Rev2.0 SDK 변경점:
 *   - 함수명: User_Setup/Loop → Control_Setup/Loop
 *   - 루프 주기: 2 ms (500 Hz) → 1 ms (1 kHz) — 송신 rate 2배
 *
 *  NOTE: 같은 폴더에 있는 control_task.c (또는 다른 Control_Setup/Loop 정의 파일)
 *        은 STM32CubeIDE 에서 Exclude from Build 처리해야 심볼 충돌이 없습니다.
 ******************************************************************************
 */

#include "xm_api.h"      // 통합 API 헤더
#include "mti-630.h"
#include "Basics.h"


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// --- Task State Machine Handle ---
static XmTsmHandle_t s_userHandle;

// Basics.h 에서 extern 선언된 공유 전역 변수들의 실제 정의
uint32_t      s_dataSaveLoopCnt = 0;
SavingData_t  SavingData;
XmLogicLevel_t sync_signal      = XM_LOW;
XmLogicLevel_t sync_signal_pre  = XM_LOW;

XmBtnEvent_t XM_button1;
XmBtnEvent_t XM_button2;
XmBtnEvent_t XM_button3;

RecordingState_t Recording = Record_OFF;
SmartAssist_t    SmartAssist = SmartAssist_OFF;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void Disconnected_Entry(void);
static void Disconnected_Loop(void);

static void Standby_Entry(void);
static void Standby_Loop(void);

static void Active_Entry(void);
static void Active_Loop(void);
static void Active_Exit(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* =================================================================
 * [필수 구현 1] 초기화 함수 (Control_Setup)
 * - 전원 인가 후 딱 한 번 실행됩니다.
 * - TSM 생성, 변수 초기화, EXT_DIO_3 (sync) 핀 설정 수행.
 * ================================================================= */
void Control_Setup(void)
{
    s_userHandle = XM_TSM_Create(XM_STATE_OFF);

    // OFF 상태 등록
    XmStateConfig_t off_conf = {
        .id = XM_STATE_OFF,
        .on_entry = Disconnected_Entry,
        .on_loop  = Disconnected_Loop
    };
    XM_TSM_AddState(s_userHandle, &off_conf);

    // STANDBY 상태 등록
    XmStateConfig_t sb_conf = {
        .id = XM_STATE_STANDBY,
        .on_entry = Standby_Entry,
        .on_loop  = Standby_Loop
    };
    XM_TSM_AddState(s_userHandle, &sb_conf);

    // ACTIVE 상태 등록
    XmStateConfig_t act_conf = {
        .id = XM_STATE_ACTIVE,
        .on_entry = Active_Entry,
        .on_loop  = Active_Loop,
        .on_exit  = Active_Exit
    };
    XM_TSM_AddState(s_userHandle, &act_conf);

    // 외부 sync 신호 (EXT_DIO_3) 입력 + pull-down
    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);

    // SYNC 초기값
    sync_signal = XM_LOW;
    sync_signal_pre = XM_LOW;

    // USB-CDC 를 TERMINAL 모드로 전환.
    // System 의 Total Data(0x20) auto-pump 를 꺼서 USB 대역폭을 확보하고,
    // FillAndSendSavingData() 의 XM_SendUsbData (Module 0x10) 만 라인에 흐르게 한다.
    // (XM_SendUsbData 는 여전히 PhAI V2.2(COBS) 로 래핑되며, GUI 가 이를 디코딩한다.)
    XM_USB_SetMode(XM_USB_MODE_TERMINAL);

    // XSENS MTi-630 외부 IMU 결합 (Rev2.0 API). 센서 미연결이어도 안전.
    // 1 kHz 루프 시작 전에 1회 호출 → 이후 XM.status.ext_imu.* 자동 채워짐.
    XM_AttachXsensMTi630();

    // 신품/공장 초기화 센서에만 필요한 Output 설정 송신 (블로킹 ~1초).
    // 이미 EEPROM 에 설정 보존된 센서면 이 줄은 주석 처리해도 됨.
    XM_ConfigureXsensMTi630();
}


/* =================================================================
 * [필수 구현 2] 반복 루프 함수 (Control_Loop)
 * - 1 ms (1 kHz) 주기로 호출됩니다.
 * - CM (Control Module) 연결 끊김을 최우선으로 감지하여 OFF 로 강제 전환.
 * ================================================================= */
void Control_Loop(void)
{
    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_OFF);
    }
    XM_TSM_Run(s_userHandle);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

static void Disconnected_Entry(void)
{
}

static void Disconnected_Loop(void)
{
    if (XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_STANDBY);
    }
}

static void Standby_Entry(void)
{
    // XSENS 는 Control_Setup 에서 이미 결합됨 (재시도/retry 불필요).
    XM_SetControlMode(XM_CTRL_MONITOR);
}

static void Standby_Loop(void)
{
    UpdateRecordingState();
    UpdateSmartAssistState();

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
    }
}

static void Active_Entry(void)
{
    XM_SetControlMode(XM_CTRL_TORQUE);
}

static void Active_Loop(void)
{
    UpdateRecordingState();
    UpdateSmartAssistState();
}

static void Active_Exit(void)
{

}
