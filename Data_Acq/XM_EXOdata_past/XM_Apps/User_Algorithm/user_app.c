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

#include <math.h>
#include <stdlib.h>

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

// --- For Dat Save ---
static bool CDCsave = false;
static SavingData_t SavingData;
static XmLogicLevel_t sync_signal;
static XmLogicLevel_t sync_signal_pre;

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

    // 외부 XSENS IMU 사용 설정
    if (XM_EnableExternalImu()) {
        // IMU 활성화 성공! (이제 UART4로 데이터가 들어옴)
        xsensIMUenableRes = true;
    } else {
        // 실패 처리 (이미 켜져있거나 하드웨어 오류)
    }

    XM_SetPinMode(XM_EXT_DIO_3, XM_EXT_DIO_MODE_INPUT_PULLDOWN);

    XM_SetControlMode(XM_CTRL_MONITOR);

    sync_signal == XM_LOW;
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
				Recording = Record_OFF;
				s_dataSaveLoopCnt = 0;
			}
		}
		else{
			Recording = Record_OFF;
			s_dataSaveLoopCnt = 0;
		}
	}
	else if (sync_signal == XM_HIGH){
		Recording = Record_ON;
	}
	if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW){
		Recording = Record_OFF;
		s_dataSaveLoopCnt = 0;
	}
	sync_signal_pre = sync_signal;

    if (XM.status.h10.h10Mode == XM_H10_MODE_ASSIST) {
        XM_TSM_TransitionTo(s_userHandle, XM_STATE_ACTIVE);
        Enable = Enable_ON;
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
	}


//    SetLedEffect(1, XM_LED_HEARTBEAT, 1000); // LED 심장박동
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

static void Active_Loop(void)
{
	sync_signal = XM_DigitalRead(XM_EXT_DIO_3);
	if (sync_signal == XM_LOW){
		if (XM_button2 == XM_BTN_CLICK && Recording == Record_OFF){
			Recording = Record_ON;
		}
		else if (Recording == Record_ON){
			if (XM_button2 == XM_BTN_CLICK){
				Recording = Record_OFF;
				s_dataSaveLoopCnt = 0;
			}
		}
		else{
			Recording = Record_OFF;
			s_dataSaveLoopCnt = 0;
		}
	}
	else if (sync_signal == XM_HIGH){
		Recording = Record_ON;
	}
	if (sync_signal_pre == XM_HIGH && sync_signal == XM_LOW){
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

//    if (XM_button2 == XM_BTN_CLICK) {
//        if (Recording == Record_OFF){
//        	click_num2++;
//            Recording = Record_ON;
//        }
//        else if (Recording == Record_ON){
//        	click_num2++;
//            Recording = Record_OFF;
//        }
//    }
    if (XM_button3 == XM_BTN_CLICK) {
        click_num3++;
    }

	if (Recording == Record_ON){
		FillAndSendSavingData();
	}

}

static void Active_Exit(void)
{

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

    // 3) 헤더/CRC 설정
    SavingData.sof = 0xAA55;
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
