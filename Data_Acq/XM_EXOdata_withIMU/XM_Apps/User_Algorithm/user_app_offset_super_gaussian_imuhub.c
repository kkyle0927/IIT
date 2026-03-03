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

#include <string.h>
#include <math.h>       // expf, powf, fabsf, isfinite

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

typedef enum {
    Record_OFF,          // Recording ON
	Record_ON,         // Recording OFF
} RecordingState_t;

typedef enum {
    Enable_OFF,          // Assist possible
	Enable_ON,         // Assist impossible
} Enable_t;

typedef enum {
    SmartAssist_OFF,          // Default smart assist ON
	SmartAssist_ON,         // Default smart assist OFF
} SmartAssist_t;

typedef enum {
    Stop,				// Stop
	Rswing,           	// Single stance - R swing
    Lswing,           	// Single stance - L swing
	RLDoubleStance,   	// Double stance - Right side anterior
	LRDoubleStance,   	// Double stance - Left side anterior
} GaitState_t;

typedef enum {
	RInitial,
	RHeel,           	// Only right heel is in contact
    RToe,          		// Only right toe is in contact
	RHeelToe,   		// Right heel and toe are in contact
	ROff,   			// Right foot is not in contact
} RFootContact_t;

typedef enum {
	LInitial,
    LHeel,           	// Only light heel is in contact
    LToe,          		// Only light toe is in contact
	LHeelToe,   		// Light heel and toe are in contact
	LOff,   			// Light foot is not in contact
} LFootContact_t;

typedef enum {
    FVECPROF_LH  = 1,   // Left Hip actuator
    FVECPROF_RH  = 2,   // Right Hip actuator
    FVECPROF_NAN = 3    // 사용하지 않는 block (TRAJ_ID_NAN)
} FVecTrajId_t;


typedef struct {
    RecordingState_t Recording;			  // Recording 여부
    Enable_t Enable;			  		  // Assistance 가능 여부
    GaitState_t GaitState;				  // Gait 상태(R, L swing, Double stance)
    RFootContact_t RFootContactPhase;	  // RFootContactPhase
    LFootContact_t LFootContactPhase;	  // RFootContactPhase

    float         targetTorque_Nm;        // 목표 보조 토크
    float         currentTorque_Nm;       // 현재 보조 토크 (스무딩 적용)

} CurrentState_t;



/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

typedef struct __attribute__((packed)) {
    uint16_t sof;      // Start-of-frame (예: 0xAA55)
    uint16_t len;      // 이 패킷 전체 길이 (sizeof(SavingData_t))

    uint32_t loopCnt;
    uint8_t  h10Mode;
    uint8_t  h10AssistLevel;
    uint8_t  SmartAssist;
    uint8_t  imuHubConnected;  // IMU Hub 연결 상태 (0 or 1)

    float leftHipAngle,  rightHipAngle;
    float leftThighAngle, rightThighAngle;
//  float pelvicAngle, pelvicVelY;
//  float leftKneeAngle, rightKneeAngle;
//  bool  isLeftFootContact, isRightFootContact;
//  uint8_t gaitState;
//  uint8_t gaitCycle;
//  float forwardVelocity;

    float leftHipTorque,  rightHipTorque;
    float leftHipMotorAngle, rightHipMotorAngle;
//  float leftHipImuFrontalRoll;
//  float leftHipImuSagittalPitch;
//  float rightHipImuFrontalRoll;
//  float rightHipImuSagittalPitch;

    float leftHipImuGlobalAccX, leftHipImuGlobalAccY, leftHipImuGlobalAccZ;
    float leftHipImuGlobalGyrX, leftHipImuGlobalGyrY, leftHipImuGlobalGyrZ;
    float rightHipImuGlobalAccX, rightHipImuGlobalAccY, rightHipImuGlobalAccZ;
    float rightHipImuGlobalGyrX, rightHipImuGlobalGyrY, rightHipImuGlobalGyrZ;

    float TrunkIMU_LocalAccX, TrunkIMU_LocalAccY, TrunkIMU_LocalAccZ;
    float TrunkIMU_LocalGyrX, TrunkIMU_LocalGyrY, TrunkIMU_LocalGyrZ;
    float TrunkIMU_QuatW, TrunkIMU_QuatX, TrunkIMU_QuatY, TrunkIMU_QuatZ;

    float leftFSR1, leftFSR2, leftFSR3, leftFSR4, leftFSR5, leftFSR6;
    float leftFSR7, leftFSR8, leftFSR9, leftFSR10, leftFSR11, leftFSR12;
    float leftFSR13, leftFSR14;

    float rightFSR1, rightFSR2, rightFSR3, rightFSR4, rightFSR5, rightFSR6;
    float rightFSR7, rightFSR8, rightFSR9, rightFSR10, rightFSR11, rightFSR12;
    float rightFSR13, rightFSR14;

    // IMU Hub Module - 6 IMU sensors (q4 + acc3 + gyr3 = 10 floats each)
    struct {
        float q_w, q_x, q_y, q_z;
        float acc_x, acc_y, acc_z;
        float gyr_x, gyr_y, gyr_z;
    } imuHub[6];

    uint16_t crc;      // sof~(crc 바로 앞까지)에 대한 CRC16
} SavingData_t;



/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* --- 전역 변수 및 핸들 선언 --- */
// --- Task State Machine Handle ---
static XmTsmHandle_t s_userHandle;
static uint32_t s_dataSaveLoopCnt;

// --- Mode State Management ---
static uint32_t s_modeTransitionTimer = 0;
static RecordingState_t Recording = Record_OFF;
static Enable_t Enable = Enable_OFF;
static SmartAssist_t SmartAssist = SmartAssist_OFF;
static XmH10Mode_t s_previoush10Mode = XM_H10_MODE_STANDBY;

// --- Gait status ---
static GaitState_t GaitState = Stop;
static RFootContact_t RFootContact = RInitial;
static LFootContact_t LFootContact = LInitial;

// --- Button state ---
static XmBtnEvent_t XM_button1;
static XmBtnEvent_t XM_button2;
static XmBtnEvent_t XM_button3;

int click_num1 = 0;
int click_num2 = 0;
int click_num3 = 0;

// --- For CDC communication ---
#define Transmit_SOF       0xAA55
#define Receive_SOF       0xAA55u
#define Receive_RX_BUF_SZ 256u
static uint8_t  s_ReceiveRxBuf[Receive_RX_BUF_SZ];
static uint32_t s_ReceiveRxLen = 0;

// --- For Dat Save ---
static SavingData_t SavingData;
static XmLogicLevel_t sync_signal;
static XmLogicLevel_t sync_signal_pre;

// --- For Super Gaussian torque profile decoding ---
#define GAUSS_SLOT_MAX     10
#define GAUSS_MAX_TORQUE   20.0f   // 출력 토크 절대값 상한 (Nm)
#define GAUSS_MIN_SIGMA     5.0f   // sigma 최소값 (5 tick = 5ms), 0 나누기 방지
#define GAUSS_MIN_N         2.0f   // n 최소값 (표준 Gaussian)
#define GAUSS_MAX_N        10.0f   // n 최대값

typedef struct {
    float    tau_max;      // 피크 토크 (Nm)
    float    mu;           // 피크 시점 (tick, 1 tick = 1ms)
    float    sigma;        // 가우시안 폭 — 표준편차 (tick)
    float    n;            // Super Gaussian 차수 (≥2)
    float    offset_val;   // exp(-0.5 × 3^n) — 사전 계산
    float    norm;         // 1/(1-offset_val) — 사전 계산
    uint32_t t_end;        // 만료 시점 (tick) — 자동 계산: mu+3σ+1
    uint32_t time_stamp;   // 현재 시간 카운터
    float    tau;          // 현재 출력 토크
    uint8_t  is_full;      // 슬롯 활성 플래그
} GaussSlot_t;

// Actuator별 버퍼 (Left Hip / Right Hip)
static GaussSlot_t s_gauss_buf_LH[GAUSS_SLOT_MAX];
static GaussSlot_t s_gauss_buf_RH[GAUSS_SLOT_MAX];

// Actuator별 출력 토크
static float LH_Torque = 0.0f;
static float RH_Torque = 0.0f;

#define FVECPKT_MAX_BYTES 256u   // 패킷 최대 바이트 수 (필요시 여유있게 늘려도 됨)

static uint8_t  g_lastFvecPayload[FVECPKT_MAX_BYTES];
static uint16_t g_lastFvecLenBytes = 0;

// Enable edge 검출용
static Enable_t s_prevEnable = Enable_OFF;

static float assist_force_N = 0.0f;   // 필요하면 left/right 합산해서 쓰거나 별도 매핑
static int assist_activation_num = 0;

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

static uint16_t CalcCrc16(const uint8_t* data, uint32_t length);
static void FillAndSendSavingData(void);

static void  GaussDecoder_Init(void);
static void  GaussDecoder_InitSingle(GaussSlot_t buf[]);
static void  GaussDecoder_TriggerSingle(GaussSlot_t buf[], const float params[4]);
static float GaussDecoder_StepSingle(GaussSlot_t buf[], float *p_offset);

static void  FVecRx_Process(void);
static void  FVecRx_ParseFrames(void);
static void  ProcessFVecPacket(const uint8_t* payload, uint16_t payload_len);
static bool  GaussDecoder_IsAllIdle(void);
static bool  GaussDecoder_IsBufIdle(GaussSlot_t buf[]);
static void StopRecordingCleanup(void);
/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

bool xsensIMUenableRes = false;

/* =================================================================
 * [필수 구현 1] 초기화 함수 예시 (User_Setup)
 * - 전원 인가 후 딱 한 번 실행됩니다.
 * - TSM 생성, 변수 초기화, LED/Button 초기 설정 등을 수행합니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * - 함수 이름을 수정하거나 삭제하면 동작하지 않습니다.
 * ================================================================= */
void User_Setup(void)
{
    // 1. TSM 생성 (초기 상태: OFF)
    s_userHandle = XM_TSM_Create(XM_STATE_OFF);

    // 2. 상태 등록 [등록 1] OFF 상태 설정
    XmStateConfig_t off_conf = {
        .id = XM_STATE_OFF,
        .on_entry = Disconnected_Entry,
        .on_loop  = Disconnected_Loop
    };
    XM_TSM_AddState(s_userHandle, &off_conf);

    // [등록 2] STANDBY 상태 설정
    XmStateConfig_t sb_conf = {
        .id = XM_STATE_STANDBY,
        .on_entry = Standby_Entry,
        .on_loop  = Standby_Loop
    };
    XM_TSM_AddState(s_userHandle, &sb_conf);

    // [등록 3] STANDBY 상태 설정
	XmStateConfig_t act_conf = {
        .id = XM_STATE_ACTIVE,
        .on_entry = Active_Entry,
        .on_loop  = Active_Loop,
		.on_exit  = Active_Exit,
    };
    XM_TSM_AddState(s_userHandle, &act_conf);

//    // 외부 XSENS IMU 사용 설정
//    if (XM_EnableExternalImu()) {
//        // IMU 활성화 성공! (이제 UART4로 데이터가 들어옴)
//        xsensIMUenableRes = true;
//    } else {
//        // 실패 처리 (이미 켜져있거나 하드웨어 오류)
//    }

    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);


    sync_signal = XM_LOW;
	sync_signal_pre = XM_LOW;

	GaussDecoder_Init();   // Super Gaussian 토크 프로파일 버퍼 초기화
    // traj_id 개념을 구동기(LH/RH)로 통합하면서 별도 전역 선택값은 사용하지 않음

    // 외부 XSENS IMU 사용 설정
//    if (XM_EnableExternalImu()) {
//        // IMU 활성화 성공! (이제 UART4로 데이터가 들어옴)
//        xsensIMUenableRes = true;
//    } else {
//        // 실패 처리 (이미 켜져있거나 하드웨어 오류)
//    }

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
//    SetLedState(1, 0); // LED OFF
}

static void Disconnected_Loop(void)
{
    // 버튼 1을 누르면 STANDBY 상태로 전환
    if (XM_IsCmConnected()) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_STANDBY);
    }

}

static void Standby_Entry(void)
{

    // XSENS IMU 부팅 시간 확보 (전원 인가 후 최소 150ms 필요)
    HAL_Delay(150);

    XM_SetControlMode(XM_CTRL_MONITOR);

    // 아직 XSENS 초기화 안 된 경우에만 1회 수행
    if (xsensIMUenableRes == false) {
        if (XM_EnableExternalImu()) {

            // Enable 성공 시 안정화 시간
        	HAL_Delay(50);

            // XSENS output 설정
            xsensMTi630.ConfigureOutput();

            xsensIMUenableRes = true;   // 초기화 완료
        }
    }

}


static void Standby_Loop(void)
{
	sync_signal = XM_DigitalRead(XM_EXT_DIO_3);
	if (sync_signal == XM_LOW){
		if (XM_button2 == XM_BTN_CLICK && Recording == Record_OFF){
			Recording = Record_ON;
		}
		else if (Recording == Record_ON){
			if (XM_button2 == XM_BTN_CLICK){
                StopRecordingCleanup();
                Recording = Record_OFF;
                s_dataSaveLoopCnt = 0;
			}
		}
		else{
            StopRecordingCleanup();
            Recording = Record_OFF;
            s_dataSaveLoopCnt = 0;
		}
	}
	else if (sync_signal == XM_HIGH){
		Recording = Record_ON;
	}
    if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW){
        StopRecordingCleanup();
        Recording = Record_OFF;
        s_dataSaveLoopCnt = 0;
    }
	sync_signal_pre = sync_signal;

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
    }

    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    XM_button2 = XM_GetButtonEvent(XM_BTN_2);
    XM_button3 = XM_GetButtonEvent(XM_BTN_3);

    if (XM_button1 == XM_BTN_CLICK) {
    	if (SmartAssist == SmartAssist_ON){
    		SmartAssist = SmartAssist_OFF;
    		XM_SetH10AssistExistingMode(false);
    	}
		else if (SmartAssist == SmartAssist_OFF){
    		SmartAssist = SmartAssist_ON;
    		XM_SetH10AssistExistingMode(true);
    	}
        click_num1++;
    }

	if (Recording == Record_ON){
		FillAndSendSavingData();
		XM_SetLedEffect(3, XM_LED_HEARTBEAT, 1000);
	}
	else if (Recording == Record_OFF){
		XM_SetLedEffect(3, XM_LED_OFF, 1000);
	}

//    SetLedEffect(1, XM_LED_HEARTBEAT, 1000); // LED 심장박동



	if (xsensIMUenableRes == false) {
	    if (XM_EnableExternalImu()) {
	    	HAL_Delay(50);
	        xsensMTi630.ConfigureOutput();
	        xsensIMUenableRes = true;
	    }
	}
}

static void Active_Entry(void)
{
	XM_SetControlMode(XM_CTRL_TORQUE);
	if (SmartAssist == SmartAssist_ON){
		XM_SetH10AssistExistingMode(true);
	}
	else{
		XM_SetH10AssistExistingMode(false);
	}
}

float s_offset_LH = 0.0f;
float s_offset_RH = 0.0f;

static void Active_Loop(void)
{
	sync_signal = XM_DigitalRead(XM_EXT_DIO_3);
	FVecRx_Process();

	if (sync_signal == XM_LOW){
		if (XM_button2 == XM_BTN_CLICK && Recording == Record_OFF){
			Recording = Record_ON;
		}
		else if (Recording == Record_ON){
			if (XM_button2 == XM_BTN_CLICK){
                StopRecordingCleanup();
                Recording = Record_OFF;
                s_dataSaveLoopCnt = 0;
			}
		}
		else{
            StopRecordingCleanup();
            Recording = Record_OFF;
            s_dataSaveLoopCnt = 0;
		}
	}
	else if (sync_signal == XM_HIGH){
		Recording = Record_ON;
	}

    if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW){
            StopRecordingCleanup();
            Recording = Record_OFF;
            s_dataSaveLoopCnt = 0;
        }
	sync_signal_pre = sync_signal;

    if (XM.status.h10.h10Mode == XM_H10_MODE_STANDBY) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_STANDBY);
        Enable = Enable_OFF;
    }

    if (XM_button1 == XM_BTN_CLICK) {
    	if (SmartAssist == SmartAssist_ON){
    		SmartAssist = SmartAssist_OFF;
    		XM_SetH10AssistExistingMode(false);
    	}
		else if (SmartAssist == SmartAssist_OFF){
    		SmartAssist = SmartAssist_ON;
    		XM_SetH10AssistExistingMode(true);
    	}
        click_num1++;
    }

    XM_button1 = XM_GetButtonEvent(XM_BTN_1);
    XM_button2 = XM_GetButtonEvent(XM_BTN_2);
    XM_button3 = XM_GetButtonEvent(XM_BTN_3);

    if (XM_button3 == XM_BTN_CLICK) {
        click_num3++;
        if (click_num3 % 2 == 0){
        	Enable = Enable_OFF;
        }
        else if (click_num3 % 2 == 1){
        	Enable = Enable_ON;
        }
    }
    // 현재 Enable 상태를 지역 변수로 잡아둠
    Enable_t currEnable = Enable;

    // 1) falling edge: ON -> OFF (토크 0 + 디코더 리셋)
    if (s_prevEnable == Enable_ON && currEnable == Enable_OFF) {
        LH_Torque = 0.0f;
        RH_Torque = 0.0f;

        s_offset_LH = 0.0f;
		s_offset_RH = 0.0f;

        XM_SetAssistTorqueLH(0.0f);
        XM_SetAssistTorqueRH(0.0f);

        GaussDecoder_Init();   // 디코더 상태 전부 리셋
    }

    // 2) rising edge: OFF -> ON
    //    New behavior: trajectories are triggered immediately in ProcessFVecPacket
    //    (or queued to append). So we do not reset/replay here.

    // 3) Enable이 ON일 때만 디코더 진행 + 토크 출력
    if (currEnable == Enable_ON) {

        for (int n = 0; n < 2; ++n) {
            LH_Torque = GaussDecoder_StepSingle(s_gauss_buf_LH, &s_offset_LH);
            RH_Torque = GaussDecoder_StepSingle(s_gauss_buf_RH, &s_offset_RH);
        }

        if (GaussDecoder_IsBufIdle(s_gauss_buf_LH)) s_offset_LH = 0.0f;
		if (GaussDecoder_IsBufIdle(s_gauss_buf_RH)) s_offset_RH = 0.0f;

        XM_SetAssistTorqueLH(LH_Torque);
        XM_SetAssistTorqueRH(RH_Torque);

        assist_activation_num++;

        if (GaussDecoder_IsAllIdle()) {
            Enable = Enable_OFF;
        }

    } else {
        // Enable OFF 상태에서는 매 루프 0 토크만 보냄
        XM_SetAssistTorqueLH(0.0f);
        XM_SetAssistTorqueRH(0.0f);
    }

    // 4) 마지막에 이전 상태 업데이트
    s_prevEnable = currEnable;

    if (Recording == Record_ON){
    	XM_SetLedEffect(3, XM_LED_HEARTBEAT, 1000);
        FillAndSendSavingData();
    }
	else if (Recording == Record_OFF){
		XM_SetLedEffect(3, XM_LED_OFF, 1000);
	}
}

static void Active_Exit(void)
{

}


static void StopRecordingCleanup(void)
{
    LH_Torque = 0.0f;
    RH_Torque = 0.0f;

    s_offset_LH = 0.0f;
    s_offset_RH = 0.0f;

    XM_SetAssistTorqueLH(0.0f);
    XM_SetAssistTorqueRH(0.0f);
}


static void FillAndSendSavingData(void)
{
    // 1) loopCnt 채우고 증가
    SavingData.loopCnt = s_dataSaveLoopCnt++;

    // 2) payload 필드 채우기
    SavingData.h10Mode        = (uint8_t)XM.status.h10.h10Mode;
    SavingData.h10AssistLevel = XM.status.h10.h10AssistLevel;
    SavingData.SmartAssist    = SmartAssist;

    SavingData.leftHipAngle   = XM.status.h10.leftHipAngle;
    SavingData.rightHipAngle  = XM.status.h10.rightHipAngle;
    SavingData.leftThighAngle = XM.status.h10.leftThighAngle;
    SavingData.rightThighAngle= XM.status.h10.rightThighAngle;

    SavingData.leftHipTorque      = XM.status.h10.leftHipTorque;
    SavingData.rightHipTorque     = XM.status.h10.rightHipTorque;
    SavingData.leftHipMotorAngle  = XM.status.h10.leftHipMotorAngle;
    SavingData.rightHipMotorAngle = XM.status.h10.rightHipMotorAngle;

    SavingData.leftHipImuGlobalAccX = XM.status.h10.leftHipImuGlobalAccX;
    SavingData.leftHipImuGlobalAccY = XM.status.h10.leftHipImuGlobalAccY;
    SavingData.leftHipImuGlobalAccZ = XM.status.h10.leftHipImuGlobalAccZ;
    SavingData.leftHipImuGlobalGyrX = XM.status.h10.leftHipImuGlobalGyrX;
    SavingData.leftHipImuGlobalGyrY = XM.status.h10.leftHipImuGlobalGyrY;
    SavingData.leftHipImuGlobalGyrZ = XM.status.h10.leftHipImuGlobalGyrZ;

    SavingData.rightHipImuGlobalAccX = XM.status.h10.rightHipImuGlobalAccX;
    SavingData.rightHipImuGlobalAccY = XM.status.h10.rightHipImuGlobalAccY;
    SavingData.rightHipImuGlobalAccZ = XM.status.h10.rightHipImuGlobalAccZ;
    SavingData.rightHipImuGlobalGyrX = XM.status.h10.rightHipImuGlobalGyrX;
    SavingData.rightHipImuGlobalGyrY = XM.status.h10.rightHipImuGlobalGyrY;
    SavingData.rightHipImuGlobalGyrZ = XM.status.h10.rightHipImuGlobalGyrZ;

    SavingData.TrunkIMU_QuatW = XM.status.imu.q_w;
    SavingData.TrunkIMU_QuatX = XM.status.imu.q_x;
    SavingData.TrunkIMU_QuatY = XM.status.imu.q_y;
    SavingData.TrunkIMU_QuatZ = XM.status.imu.q_z;

    SavingData.TrunkIMU_LocalAccX = XM.status.imu.acc_x;
    SavingData.TrunkIMU_LocalAccY = XM.status.imu.acc_y;
    SavingData.TrunkIMU_LocalAccZ = XM.status.imu.acc_z;
    SavingData.TrunkIMU_LocalGyrX = XM.status.imu.gyr_x;
    SavingData.TrunkIMU_LocalGyrY = XM.status.imu.gyr_y;
    SavingData.TrunkIMU_LocalGyrZ = XM.status.imu.gyr_z;

    SavingData.leftFSR1  = XM.status.grf.leftSensorData[0];
    SavingData.leftFSR2  = XM.status.grf.leftSensorData[1];
    SavingData.leftFSR3  = XM.status.grf.leftSensorData[2];
    SavingData.leftFSR4  = XM.status.grf.leftSensorData[3];
    SavingData.leftFSR5  = XM.status.grf.leftSensorData[4];
    SavingData.leftFSR6  = XM.status.grf.leftSensorData[5];
    SavingData.leftFSR7  = XM.status.grf.leftSensorData[6];
    SavingData.leftFSR8  = XM.status.grf.leftSensorData[7];
    SavingData.leftFSR9  = XM.status.grf.leftSensorData[8];
    SavingData.leftFSR10 = XM.status.grf.leftSensorData[9];
    SavingData.leftFSR11 = XM.status.grf.leftSensorData[10];
    SavingData.leftFSR12 = XM.status.grf.leftSensorData[11];
    SavingData.leftFSR13 = XM.status.grf.leftSensorData[12];
    SavingData.leftFSR14 = XM.status.grf.leftSensorData[13];

    SavingData.rightFSR1  = XM.status.grf.rightSensorData[0];
    SavingData.rightFSR2  = XM.status.grf.rightSensorData[1];
    SavingData.rightFSR3  = XM.status.grf.rightSensorData[2];
    SavingData.rightFSR4  = XM.status.grf.rightSensorData[3];
    SavingData.rightFSR5  = XM.status.grf.rightSensorData[4];
    SavingData.rightFSR6  = XM.status.grf.rightSensorData[5];
    SavingData.rightFSR7  = XM.status.grf.rightSensorData[6];
    SavingData.rightFSR8  = XM.status.grf.rightSensorData[7];
    SavingData.rightFSR9  = XM.status.grf.rightSensorData[8];
    SavingData.rightFSR10 = XM.status.grf.rightSensorData[9];
    SavingData.rightFSR11 = XM.status.grf.rightSensorData[10];
    SavingData.rightFSR12 = XM.status.grf.rightSensorData[11];
    SavingData.rightFSR13 = XM.status.grf.rightSensorData[12];
    SavingData.rightFSR14 = XM.status.grf.rightSensorData[13];

    // IMU Hub data
    SavingData.imuHubConnected = XM.status.imu_hub.is_connected ? 1 : 0;
    for (int i = 0; i < 6; i++) {
        XmImuHubSensor_t* src = &XM.status.imu_hub.sensor[i];
        SavingData.imuHub[i].q_w   = src->q_w;
        SavingData.imuHub[i].q_x   = src->q_x;
        SavingData.imuHub[i].q_y   = src->q_y;
        SavingData.imuHub[i].q_z   = src->q_z;
        SavingData.imuHub[i].acc_x = src->acc_x;
        SavingData.imuHub[i].acc_y = src->acc_y;
        SavingData.imuHub[i].acc_z = src->acc_z;
        SavingData.imuHub[i].gyr_x = src->gyr_x;
        SavingData.imuHub[i].gyr_y = src->gyr_y;
        SavingData.imuHub[i].gyr_z = src->gyr_z;
    }

    // 3) 헤더/CRC 설정
    SavingData.sof = Transmit_SOF;
    SavingData.len = (uint16_t)sizeof(SavingData_t);

    // crc 자신을 제외한 전체에 대해 CRC 계산
    uint16_t crc = CalcCrc16((const uint8_t*)&SavingData,
                              sizeof(SavingData_t) - sizeof(SavingData.crc));
    SavingData.crc = crc;

    // 4) 전송 (리턴값은 필요하면 체크)
    (void)XM_SendUsbData(&SavingData, sizeof(SavingData_t));
}

static uint16_t CalcCrc16(const uint8_t* data, uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static void GaussDecoder_InitSingle(GaussSlot_t buf[])
{
    for (int i = 0; i < GAUSS_SLOT_MAX; i++) {
        memset(&buf[i], 0, sizeof(GaussSlot_t));
    }
}

static void GaussDecoder_Init(void)
{
    GaussDecoder_InitSingle(s_gauss_buf_LH);
    GaussDecoder_InitSingle(s_gauss_buf_RH);

    s_offset_LH = 0.0f;
    s_offset_RH = 0.0f;
}

static bool GaussDecoder_IsBufIdle(GaussSlot_t buf[])
{
    for (int i = 0; i < GAUSS_SLOT_MAX; i++) {
        if (buf[i].is_full) {
            return false;   // 아직 살아있는 slot 있음
        }
    }
    return true;            // 이 버퍼는 모두 비어 있음
}

static bool GaussDecoder_IsAllIdle(void)
{
    if (!GaussDecoder_IsBufIdle(s_gauss_buf_LH)) return false;
    if (!GaussDecoder_IsBufIdle(s_gauss_buf_RH)) return false;
    return true;            // 네 채널 전체가 다 비어 있음 → trajectory 종료
}

// n이 {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0} 중 하나일 때,
// 정수 곱셈 + sqrtf() (ARM HW 명령어)로 powf를 대체하여 ~20-400배 빠르게 계산
static float fast_powf_known_n(float x, float n)
{
    float x2 = x * x;

    if (n == 2.0f) return x2;
    if (n == 3.0f) return x2 * x;
    if (n == 4.0f) return x2 * x2;
    if (n == 5.0f) return x2 * x2 * x;

    // 반정수: sqrtf 사용 (Cortex-M4F VSQRT.F32 ~14 cycles)
    float sx = sqrtf(x);
    if (n == 2.5f) return x2 * sx;
    if (n == 3.5f) return x2 * x * sx;
    if (n == 4.5f) return x2 * x2 * sx;

    return powf(x, n);  // fallback (도달하지 않아야 함)
}

// params[4] = { tau_max(Nm), mu(tick), sigma(tick), n(order) }
static void GaussDecoder_TriggerSingle(GaussSlot_t buf[], const float params[4])
{
    for (int i = 0; i < GAUSS_SLOT_MAX; i++) {
        if (!buf[i].is_full) {
            GaussSlot_t *s = &buf[i];

            s->tau_max = params[0];
            s->sigma   = params[2];
            if (s->sigma < GAUSS_MIN_SIGMA) s->sigma = GAUSS_MIN_SIGMA;

            s->n = params[3];
            if (s->n < GAUSS_MIN_N) s->n = GAUSS_MIN_N;
            if (s->n > GAUSS_MAX_N) s->n = GAUSS_MAX_N;

            // mu >= 3*sigma 강제 (t=0에서 0 시작 보장)
            s->mu = params[1];
            if (s->mu < 3.0f * s->sigma) s->mu = 3.0f * s->sigma;

            // offset/norm 사전 계산 (3σ 지점에서 정확히 0이 되도록)
            s->offset_val = expf(-0.5f * fast_powf_known_n(3.0f, s->n));
            s->norm       = 1.0f / (1.0f - s->offset_val);

            // t_end 자동 계산: mu + 3σ + 1 (3σ 지점에서 0이므로)
            s->t_end = (uint32_t)(s->mu + 3.0f * s->sigma) + 1u;

            s->time_stamp = 0;
            s->tau        = 0.0f;
            s->is_full    = 1;
            break;
        }
    }
}

// Super Gaussian with y-offset:
// τ(t) = tau_max × max(0, (exp(-0.5 × |(t-μ)/σ|^n) - offset) × norm)
// 3σ 지점에서 정확히 0, 피크에서 정확히 tau_max
static float GaussDecoder_StepSingle(GaussSlot_t buf[], float *p_offset)
{
    float y_sum = 0.0f;

    for (int i = 0; i < GAUSS_SLOT_MAX; i++) {
        GaussSlot_t *s = &buf[i];
        if (!s->is_full) continue;

        float t     = (float)s->time_stamp;
        float diff  = fabsf(t - s->mu) / s->sigma;
        float raw   = expf(-0.5f * fast_powf_known_n(diff, s->n));
        float shape = (raw - s->offset_val) * s->norm;
        if (shape < 0.0f) shape = 0.0f;

        float tau = s->tau_max * shape;

        // NaN/Inf 안전 처리
        if (!isfinite(tau)) tau = 0.0f;

        // 토크 클램핑
        if (tau >  GAUSS_MAX_TORQUE) tau =  GAUSS_MAX_TORQUE;
        if (tau < -GAUSS_MAX_TORQUE) tau = -GAUSS_MAX_TORQUE;

        s->tau = tau;
        s->time_stamp++;

        // 슬롯 만료 처리 (3σ offset으로 이미 τ=0)
        if (s->time_stamp >= s->t_end) {
            *p_offset += s->tau;
            memset(s, 0, sizeof(GaussSlot_t));
            s->is_full = 0;
            continue;
        }

        y_sum += tau;
    }

    return y_sum + (*p_offset);
}

static void FVecRx_ParseFrames(void)
{
    uint32_t idx = 0;

    // s_ReceiveRxBuf[0 .. s_ReceiveRxLen-1] 안에 쌓인 데이터에서
    // [SOF, len, payload, crc] 프레임을 순차적으로 파싱
    while (s_ReceiveRxLen - idx >= 2u)   // SOF(2바이트)는 있어야 함
    {
        // 1) SOF 동기 맞추기 (0xAA55 찾기)
        uint16_t sof =
            (uint16_t)s_ReceiveRxBuf[idx] |
            ((uint16_t)s_ReceiveRxBuf[idx + 1u] << 8);

        if (sof != Receive_SOF)
        {
            // SOF 아니면 1바이트씩 버리면서 재동기화
            idx++;
            continue;
        }

        // 2) 헤더 길이 체크 (SOF + len 까지 최소 4바이트 필요)
        if (s_ReceiveRxLen - idx < 4u)
        {
            // len 필드까지 아직 다 안 들어왔으면 다음 루프에서 다시 시도
            break;
        }

        // payload_len 바이트 수
        uint16_t payload_len =
            (uint16_t)s_ReceiveRxBuf[idx + 2u] |
            ((uint16_t)s_ReceiveRxBuf[idx + 3u] << 8);

        // 너무 말도 안 되는 길이면(버퍼 초과) SOF 한 바이트만 넘기고 재동기화
        if (payload_len > (Receive_RX_BUF_SZ - 6u))  // SOF(2)+len(2)+CRC(2) 고려
        {
            idx++;
            continue;
        }

        // 이 프레임의 전체 바이트 길이 = SOF(2) + len(2) + payload + CRC(2)
        uint32_t frame_len = 2u + 2u + (uint32_t)payload_len + 2u;

        // 3) 프레임 전체가 아직 안 들어왔으면 다음 루프에서 다시 시도
        if (s_ReceiveRxLen - idx < frame_len)
        {
            break;
        }

        // 4) CRC 체크
        uint8_t* frame_ptr = &s_ReceiveRxBuf[idx];
        // 수신 CRC (payload 뒤에 있음)
        uint16_t crc_rx =
            (uint16_t)frame_ptr[4u + payload_len] |
            ((uint16_t)frame_ptr[4u + payload_len + 1u] << 8);

        // 계산 CRC : [len(2바이트) + payload(payload_len 바이트)]에 대해 계산
        uint16_t crc_calc = CalcCrc16(&frame_ptr[2u], 2u + (uint32_t)payload_len);

        if (crc_rx == crc_calc)
        {
            // CRC OK → payload를 그대로 ProcessFVecPacket에 넘김
            const uint8_t* payload = &frame_ptr[4u];
            ProcessFVecPacket(payload, payload_len);

            // 이 프레임 전체 소비
            idx += frame_len;
        }
        else
        {
            // CRC NG → SOF 한 바이트만 건너뛰고 재동기화 시도
            idx++;
        }
    }

    // 5) 사용한 부분(buf[0..idx-1]) 제거하고 나머지 앞으로 땡기기
    if (idx > 0u)
    {
        uint32_t remain = s_ReceiveRxLen - idx;
        if (remain > 0u)
        {
            memmove(s_ReceiveRxBuf, &s_ReceiveRxBuf[idx], remain);
        }
        s_ReceiveRxLen = remain;
    }
}



static void FVecRx_Process(void)
{
    uint8_t temp[64];
    uint32_t n = XM_GetUsbData(temp, sizeof(temp));
    if (n > 0)
    {
        if (s_ReceiveRxLen + n < Receive_RX_BUF_SZ)
        {
            memcpy(&s_ReceiveRxBuf[s_ReceiveRxLen], temp, n);
            s_ReceiveRxLen += n;
        }
        else
        {
            // Overflow → reset
            s_ReceiveRxLen = 0;
        }
    }

    FVecRx_ParseFrames();
}

static void ProcessFVecPacket(const uint8_t* payload, uint16_t payload_len)
{
    // 1) 마지막 수신 payload를 그대로 보관 (디버깅/확인용)
    if (payload_len > FVECPKT_MAX_BYTES) {
        payload_len = FVECPKT_MAX_BYTES;
    }

    if (payload_len == 0u) {
        return;
    }

    memcpy(g_lastFvecPayload, payload, payload_len);
    g_lastFvecLenBytes = payload_len;

    // 2) Overlay trigger:
    //    - 기존에 돌고 있는 슬롯들을 유지한 채(진행 중 trajectory),
    //      새 패킷에서 나온 이벤트를 추가로 trigger하여 출력이 '합(superposition)'
    //      되도록 한다.

        const uint16_t nf = payload_len / (uint16_t)sizeof(float);
        const float* lf32 = (const float*)payload;

        // Packet format: [traj_id][tau_max,mu,sigma,n]...[-1,0,0,0] repeat per traj block
        uint16_t j = 0u;
        bool any_triggered = false;

        while (j < nf) {
            const int traj_id = (int)lf32[j++];

            GaussSlot_t* target_buf = NULL;
            if (traj_id == (int)FVECPROF_LH) {
                target_buf = s_gauss_buf_LH;
            } else if (traj_id == (int)FVECPROF_RH) {
                target_buf = s_gauss_buf_RH;
            }

            while (j + 3u < nf) {
                if ((int)lf32[j] == -1) {
                    j += 4u;
                    break;
                }

                if (target_buf != NULL) {
                    GaussDecoder_TriggerSingle(target_buf, &lf32[j]);
                    any_triggered = true;
                }

                j += 4u;
            }
        }

        if (any_triggered) {
            Enable = Enable_ON;
        }
}
