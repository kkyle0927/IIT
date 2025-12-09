/**
 ******************************************************************************
 * @file    user_app.c
 * @author  HyundoKim
 * @brief   
 * @version 0.1
 * @date    Nov 18, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_api.h"      // 통합 API 헤더
#include "mti-630.h"
#include "Basics.h"


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
 * PUBLIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */


/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// --- Task State Machine Handle ---
static XmTsmHandle_t s_userHandle;

// 이 친구들은 이제 static 빼고 “정의”만 담당
uint32_t      s_dataSaveLoopCnt = 0;
SavingData_t  SavingData;
XmLogicLevel_t sync_signal      = XM_LOW;
XmLogicLevel_t sync_signal_pre  = XM_LOW;

XmBtnEvent_t XM_button1;
XmBtnEvent_t XM_button2;
XmBtnEvent_t XM_button3;

RecordingState_t Recording = Record_OFF;
SmartAssist_t    SmartAssist = SmartAssist_OFF;

bool xsensIMUenableRes = false;

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
 * [필수 구현 1] 초기화 함수 예시 (User_Setup)
 * - 전원 인가 후 딱 한 번 실행됩니다.
 * - TSM 생성, 변수 초기화, LED/Button 초기 설정 등을 수행합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Setup(void)
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

    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);

    // SYNC 초기값
    sync_signal = XM_LOW;
	sync_signal_pre = XM_LOW;
}


/* =================================================================
 * [필수 구현 2] 반복 루프 함수 예시 (User_Loop)
 * - 2ms(500Hz) 주기로 계속 호출됩니다.
 * - 제어 알고리즘, TSM 실행 로직을 여기에 작성합니다.
 * - 내부 IPO(Input-Process-Output)모델이 적용되어 있습니다.
 * - xm_api_data.h를 통해 RxData에 대해 확인할 수 있습니다.
 * - XM_SetControlMode함수를 Setup에서 실시간 제어 모드 / 모니터링 모드를 선택해야 합니다. (기본 모니터링 모드)
 * - 모니터링 모드에서는 H10으로 제어 명령을 전송하지 않습니다. 
 * - 실시간 제어 모드에서는 H10으로 Torque input을 2ms 주기로 전송합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Loop(void)
{
	// CM 연결 상태를 최우선으로 확인하여, 연결이 끊겼을 경우 OFF 상태로 강제 전환합니다.
    if (!XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_OFF);
    }
    // TSM 실행 (현재 상태에 맞는 Loop 함수가 실행됨)
    XM_TSM_Run(s_userHandle);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/* --- 상태별 동작 함수 정의 예시 --- */
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
	// XSENS IMU 부팅 시간 확보 (전원 인가 후 최소 150ms 필요)
    HAL_Delay(150);
    XM_SetControlMode(XM_CTRL_MONITOR);
    UpdateXsensImuEnable();
}

static void Standby_Loop(void)
{
	UpdateRecordingState();
	UpdateSmartAssistState();

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
    }

    UpdateXsensImuEnable();
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
