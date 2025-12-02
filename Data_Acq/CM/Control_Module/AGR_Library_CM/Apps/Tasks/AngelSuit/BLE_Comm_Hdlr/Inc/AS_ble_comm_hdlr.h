

#ifndef BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_
#define BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_

#include "module.h"

#ifdef SUIT_MINICM_ENABLED

#include <stdint.h>
#include <time.h>

#include "robot_setting.h"
#include "robot_FSM.h"
#include "robot_DMS.h"

#include "data_object.h"
/*CDI*/
#include "data_object_common.h"
#include "data_object_dictionaries.h"
/*IOIF*/
#include "ioif_tim_common.h"
#include "ioif_esp32.h"

#include "AS_system_ctrl.h"
#include "AS_whole_body_ctrl.h"
#include "AS_gait_ctrl.h"
#include "AS_dev_mngr.h"
#include "aes.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define TORQUE_CONSTANT_SAM 0.085 // Nm * A
#define H10_SUIT_GEAR_RATIO 18.75
#define K10_SUIT_GEAR_RATIO 18.75

// METs 값 (일반 보행)
#define WALKING_MET         3.5     // 걷기 MET 값, 예시값
// 평균 체중 (kg)
#define AVERAGE_WEIGHT      70.0    // 체중 70kg 기준
#define TIME_INTERVAL_100MS 0.1     // 시간 간격, 100ms = 0.1초

#define SESSION_COUNT		10		// Session 사용 시간 저장 수


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _SUIT_BLE_FLASH_DATA {
    IDLE_FLASH_BLE_DATA,
    SAVE_FLASH_BLE_DATA,
    DOWNLOAD_FLASH_BLE_DATA,
    BLE_FLASH_FLAG_MAX_NUM
} SUITBLEFlashFlag;

typedef enum _SUIT_BLE_Module_Setting_t {
	SUIT_BLE_SETTING_SETDONE,
	SUIT_BLE_SETTING_DEFAULT,
	SUIT_BLE_SETTING_FLASH_DEFAULT = 255,
} SUIT_BLE_Module_Setting_t;

typedef enum _ASSIST_MODE {
    SMART_ASSIST_MODE,      // 스마트 보조 모드
    AQUA_MODE,              // 아쿠아 모드
    UNIVERSE_MODE,          // 우주 모드
    JOINT_LIMIT_MODE,       // 각도 & 각속도 제한 모드
    STS_MODE,               // 앉기 서기 보조 모드
    MANUAL_ASSIST_MODE,     // 동작 강제 제어 모드
    ASSIST_MODE_MAX_NUM
} ASSIST_MODE;


typedef bool (*AS_BLE_TxFncPtr) (uint8_t*, uint32_t);  // Data, Len
typedef struct _AS_BLEData_t {
//PDO
    uint8_t BatteryLevel;       // 배터리 정보 (0~100%)
    uint16_t FSM_StateId;       // FSM State ID
    uint8_t AssistMode;         // 보조 프로그램 종류 (0: 스마트 보조, 1: 아쿠아, 2: 우주)
    uint8_t StandbyMode;        // CM 대기 모드 (0: OFF, 1: ON)
    uint8_t AssistanceLevel;    // 보조력 단계 (0~10단계)
    uint8_t canOperate;         // 정상 여부
    uint32_t TrainingSessionID;
    uint32_t TrainingModeID;
    float LeftThighSagittalAngle;	// degree 단위
    float RightThighSagittalAngle;	// degree 단위
    float LeftShankSagittalAngle;	// degree 단위
    float RightShankSagittalAngle;	// degree 단위
    float LeftThighNeutralSagittalAngle;    // degree 단위
    float RightThighNeutralSagittalAngle;   // degree 단위
    float LeftShankNeutralSagittalAngle;    // degree 단위
    float RightShankNeutralSagittalAngle;   // degree 단위
    float LeftAmplitude;        // 왼다리 진폭
    float RightAmplitude;       // 오른다리 진폭
    int16_t LeftHipPositionAct;	// 왼쪽 Hip 엔코더 각도
    int16_t RightHipPositionAct;	// 오른쪽 Hip 엔코더 각도
    /* 이용시간 */
    uint16_t SessionDuration;     // 세션 훈련시간 (세션 동안 모든 사용기록 포함, sec)
    uint16_t ModeDuration;        // 모드 사용시간 (대기모드 포함, sec)
    uint16_t OperationDuration;	  // 로봇 운용시간 (대기모드 비포함, sec)

    /* 비대칭 지수 */
    int8_t AsymmetricLevel;     // 비대칭 지수 (-25 ~ 25)
    float AsymmetricLevelAvg;   // 비대칭 지수 평균값
    uint16_t WalkCount;	        // 보행수
    uint16_t LeftWalkCount;     // 왼쪽 다리 보행 수
    uint16_t RightWalkCount;    // 오른쪽 다리 보행 수
    float WalkDistance;	        // 보행 거리(m)
    float CalculateCalorie;	    // 소모 칼로리(kcal)
    float RightHipTorque;       // 오른쪽 엉덩이 토크
    float LeftHipTorque;        // 왼쪽 엉덩이 토크
    float RightKneeTorque;      // 오른쪽 무릎 토크
    float LeftKneeTorque;       // 왼쪽 무릎 토크
    float LeftThighVelocityRaw;
    float RightThighVelocityRaw;
    float LeftShankVelocityRaw;
	float RightShankVelocityRaw;
    float PelvicSagittalAngle;      // 골반 앞뒤 기울어짐 각도
    float PelvicFrontalAngle;		// 골반 좌우 기울어짐 각도
    float AsymmetricAngle;
    uint8_t   StepLengthAvg;
    uint8_t   STSPostureHold;
    uint8_t   STSPostureReady;
//    uint8_t   SittoStandDuration;
//    uint8_t   StandtoSitDuration;
    uint8_t	SittoStandAssistLevel;
    uint8_t StandtoSitAssistLevel;
    uint16_t  SittingDuration;
    uint16_t  StandingDuration;
    uint16_t  SittingDurationTotal;
    uint16_t  StandingDurationTotal;
    uint16_t  SittingCount;
    uint16_t  StandingCount;
    float LeftHipMaxAngle;
    float RightHipMaxAngle;
    float LeftHipMinAngle;
    float RightHipMinAngle;
    float LeftHipMaxVelocity;
    float RightHipMaxVelocity;
    float LeftHipMinVelocity;
    float RightHipMinVelocity;

	uint16_t MotionAnalysisDuration;
	uint32_t ReportId;
	uint32_t TrackUsage;
	uint32_t MotionAnalysisType;
	uint16_t MotionAnalysisWalkCountPerMin;
	uint16_t MotionAnalysisLeftWalkCount;
	uint16_t MotionAnalysisRightWalkCount;
	uint8_t MotionAnalysisLeftStepLength;
	uint8_t MotionAnalysisRightStepLength;
	int16_t MotionAnalysisFlexionDeg_LH;
	int16_t MotionAnalysisExtensionDeg_LH;
	int16_t MotionAnalysisFlexionDeg_RH;
	int16_t MotionAnalysisExtensionDeg_RH;
	int16_t MotionAnalysisVelocityMax_LH;
	int16_t MotionAnalysisVelocityMax_RH;
	uint8_t MotionAnalysisGroundContact;
	uint16_t MotionAnalysisWalkDistance;
	float MotionAnalysisWalkSpeed;

        uint16_t SPPBWalkSpeed;
    uint16_t SPPBWalkDistance;
    uint16_t SPPBWalkDuration;
    uint8_t SPPBAssistOnOff;

//SDO
    uint8_t functionNum;
	uint8_t msgMTC[10];
	uint8_t msgCTM[10];
    uint8_t msgCommand;         // 메세지 전송 관련 Command
    uint8_t msgSuccess;
    uint8_t ErrorMSGcode;
    char aes_encrypted_base_64_res[30];
    uint32_t initializeDate;
    uint32_t initializeTime;
    uint32_t UserID;            // 사용자 ID (없으면 default = 0)
    uint32_t ProUserID;         // PRO 사용자 ID (없으면 default = 0)
    uint32_t User_Age;
    uint32_t User_Gender;
    uint32_t User_Weight;
    uint32_t User_Height;
    uint32_t User_Right_Thigh_Length;
    uint32_t User_Left_Thigh_Length;
    uint32_t User_Right_Shin_Length;
    uint32_t User_Left_Shin_Length;
    uint32_t User_Right_Ankle_Length;
    uint32_t User_Left_Ankle_Length;

    uint8_t SoundLanguage;	    // 언어설정(0: 한국어,1: 영어)
    uint8_t SoundLevel;	        // 음량 레벨(0~100)
    uint8_t SoundGender;	    // 언어 성별(0: 남성, 1: 여성)
    uint8_t Soundplay;	        // 미리듣기 재생(0: off, 1: on)

    uint8_t NeutralPostureCalibration;

    float LeftThighNeutralPostureOffset;    // Neutral Posture Offset
    float RightThighNeutralPostureOffset;   // Neutral Posture Offset
    float LeftShankNeutralPostureOffset;    // Neutral Posture Offset
    float RightShankNeutralPostureOffset;   // Neutral Posture Offset

    uint8_t LeftHipExtensionTorque;   // Nm 단위
    uint8_t LeftHipFlexionTorque;	    // Nm 단위
    uint8_t RightHipExtensionTorque;	// Nm 단위
    uint8_t RightHipFlexionTorque;    // Nm 단위
    uint8_t LeftKneeExtensionTorque;	// Nm 단위
    uint8_t LeftKneeFlexionTorque;    // Nm 단위
    uint8_t RightKneeExtensionTorque;	// Nm 단위
    uint8_t RightKneeFlexionTorque;   // Nm 단위
    uint8_t LeftHipGravityCompensation;   // Nm 단위
    uint8_t RightHipGravityCompensation;  // Nm 단위
    uint8_t LeftHipVelocityCompensation;  // Nm 단위
    uint8_t RightHipVelocityCompensation; // Nm 단위
    uint8_t LeftKneeGravityCompensation;   // Nm 단위
    uint8_t RightKneeGravityCompensation;  // Nm 단위
    uint8_t LeftKneeVelocityCompensation;  // Nm 단위
    uint8_t RightKneeVelocityCompensation; // Nm 단위
    uint8_t DisableStairWalkingDetection;

    int8_t LeftHipFlexionLimit;
    int8_t LeftHipExtensionLimit;
    int8_t LeftHipFlexionVelocityLimit;
    int8_t LeftHipExtensionVelocityLimit;
    int8_t RightHipFlexionLimit;
    int8_t RightHipExtensionLimit;
    int8_t RightHipFlexionVelocityLimit;
    int8_t RightHipExtensionVelocityLimit;


    uint8_t LeftKneeFlexionLimit;
    uint8_t LeftKneeExtensionLimit;
    uint8_t LeftKneeVelocityLimit;
    uint8_t RightKneeFlexionLimit;
    uint8_t RightKneeExtensionLimit;
    uint8_t RightKneeVelocityLimit;

    uint32_t StartDate;           // 현재 날짜
    uint32_t StartTime;           // 현재 날짜
    uint32_t EndDate;           // 현재 날짜
    uint32_t EndTime;           // 현재 날짜

    uint8_t MobileUuidFront[10];
    uint8_t MobileUuidBack[10];
    uint8_t Decrypted_plain_txt[10];
    uint8_t Org_Code[10];
    uint32_t LoginData[4];


    uint32_t UseDate;           // 현재 날짜
    uint8_t Power;              // CM 전원 (0: OFF, 1: ON)
    uint8_t LeftAssistOnOff;    // 왼쪽 보조력 ON/OFF
    float StandHoldTorque;	        // Nm 단위
    float TrunkSagittalAngle;	    // degree 단위
    float TrunkFrontalAngle;	        // degree 단위
    float LeftThighFrontalAngle;	    // degree 단위
    float RightThighFrontalAngle;	// degree 단위
    float LeftShankFrontalAngle;	    // degree 단위
    float RightShankFrontalAngle;	// degree 단위
    float MD01_PositionOutput;	// degree 단위
    float MD02_PositionOutput;	// degree 단위
    float MD03_PositionOutput;	// degree 단위
    float MD04_PositionOutput;	// degree 단위
    float MD05_PositionOutput;	// degree 단위
    float MD06_PositionOutput;	// degree 단위
    float MD07_PositionOutput;	// degree 단위
    float MD08_PositionOutput;	// degree 단위
    float MD09_PositionOutput;	// degree 단위
    float MD10_PositionOutput;	// degree 단위
    float MD11_PositionOutput;	// degree 단위
    float MD12_PositionOutput;	// degree 단위
    float MD13_PositionOutput;	// degree 단위
    float MD14_PositionOutput;	// degree 단위
    float MD15_PositionOutput;	// degree 단위
    float TrunkSagittalTemperature;	        // 섭씨 온도
    float LeftThighSagittalTemperature;	    // 섭씨 온도
    float RightThighSagittalTemperature;	// 섭씨 온도
    float LeftShankSagittalTemperature;	    // 섭씨 온도
    float RightShankSagittalTemperature;	// 섭씨 온도
    float TrunkFrontalTemperature;	        // 섭씨 온도
    float LeftThighFrontalTemperature;	    // 섭씨 온도
    float RightThighFrontalTemperature;	    // 섭씨 온도
    float LeftShankFrontalTemperature;	    // 섭씨 온도
    float RightShankFrontalTemperature;	    // 섭씨 온도
    float TrunkNeutralSagittalAngle;        // degree 단위
    float TrunkNeutralFrontalAngle;	        // degree 단위
    float LeftThighNeutralFrontalAngle;     // degree 단위
    float RightThighNeutralFrontalAngle;    // degree 단위
    float LeftShankNeutralFrontalAngle;     // degree 단위
    float RightShankNeutralFrontalAngle;    // degree 단위
    float MD01_AngleBias;   // degree 단위
    float MD02_AngleBias;   // degree 단위
    float MD03_AngleBias;   // degree 단위
    float MD04_AngleBias;   // degree 단위
    float MD05_AngleBias;   // degree 단위
    float MD06_AngleBias;   // degree 단위
    float MD07_AngleBias;   // degree 단위
    float MD08_AngleBias;   // degree 단위
    float MD09_AngleBias;   // degree 단위
    float MD10_AngleBias;   // degree 단위
    float MD11_AngleBias;   // degree 단위
    float MD12_AngleBias;   // degree 단위
    float MD13_AngleBias;   // degree 단위
    float MD14_AngleBias;   // degree 단위
    float MD15_AngleBias;   // degree 단위

    uint16_t OperationFsmVersion;	
    uint16_t ServiceFsmVersion;
    uint16_t DataManagementSettingVersion;
    uint16_t MotionMapVersion;
    uint16_t RobotSettingVersion;

    uint8_t MotionAnalysisDataState;

    uint16_t MotionAnalysisReportId;
	uint16_t MotionAnalysisMeanWalkSpeed;
	uint16_t MotionAnalysisStdWalkSpeed;
	uint16_t MotionAnalysisMeanWalkCountPerMin;
	uint16_t MotionAnalysisStdWalkCountPerMin;
	uint16_t MotionAnalysisMeanStrideLength;
	uint16_t MotionAnalysisStdStrideLength;
	uint16_t MotionAnalysisBilateralMeanStepLength;
	uint16_t MotionAnalysisMeanLeftStepLength;
	uint16_t MotionAnalysisMeanRightStepLength;
	uint16_t MotionAnalysisAsymmetryStepLength;
	uint16_t MotionAnalysisMeanLeftSwingPhase;
	uint16_t MotionAnalysisMeanRightSwingPhase;
	uint16_t MotionAnalysisAsymmetrySwingPhase;
	uint16_t MotionAnalysisMeanLeftStancePhase;
	uint16_t MotionAnalysisMeanRightStancePhase;
	uint16_t MotionAnalysisAsymmetryStancePhase;
	uint16_t MotionAnalysisMeanLeftHipRoM;
	uint16_t MotionAnalysisMeanLeftHipMax;
	uint16_t MotionAnalysisMeanLeftHipMin;
	uint16_t MotionAnalysisMeanRightHipRoM;
	uint16_t MotionAnalysisMeanRightHipMax;
	uint16_t MotionAnalysisMeanRightHipMin;
	uint16_t MotionAnalysisAsymmetryRoM;
	uint16_t MotionAnalysisRMSLeftHipDiff;
	uint16_t MotionAnalysisRMSRightHipDiff;

    uint16_t ThighAngleLeftX[101];
    int16_t ThighAngleLeftYMean[101];
    int16_t ThighAngleLeftYStd[101];
    uint16_t ThighAngleRightX[101];
    int16_t ThighAngleRightYMean[101];
    int16_t ThighAngleRightYStd[101];
    uint16_t ThighTorqueLeftX[101];
    int16_t ThighTorqueLeftYMean[101];
    int16_t ThighTorqueLeftYStd[101];
    uint16_t ThighTorqueRightX[101];
    int16_t ThighTorqueRightYMean[101];
    int16_t ThighTorqueRightYStd[101];
	uint8_t MotionAnalysisFinishAble;

	uint8_t VersionData[19];
	uint8_t DataReceiveState;
	uint8_t CheckErrorMsgAck;

    uint8_t SPPB_Start_Flag;
    uint8_t SPPB_Stop_Flag;
    uint8_t SPPB_Standing_Count;
} AS_BLEData_t;

typedef struct _AS_ManuData_t {

    uint8_t test;

	uint8_t msgMTC[10];
	uint8_t msgCTM[10];

    uint8_t start_result;
    uint8_t sdcard_test_result;
    uint8_t SPI_test_result;
    uint8_t Wifi_test_result;
    uint8_t fdcan_test_result;
    uint8_t current_test_result;
    uint8_t button_test_result;
    uint8_t magnetic_test_result;
    uint8_t uuid_reset_result;
    uint8_t ESP32_command;

    uint8_t deviceLine[10];
    uint8_t deviceName[10];
    uint8_t deviceCode[10];

	uint8_t CommadData;
	uint8_t SuccessData;

    uint8_t Wifi_SSID_front[10];
    uint8_t Wifi_SSID_middle[10];
    uint8_t Wifi_SSID_back[10];
    uint8_t Wifi_PW_front[10];
    uint8_t Wifi_PW_middle[10];
    uint8_t Wifi_PW_back[10];


	uint8_t Robot_model[10];
	uint8_t Serial_number_front[10];
	uint8_t Serial_number_middle[10];
	uint8_t Serial_number_back[10];
	uint8_t RTC_year;
	uint8_t RTC_month;
	uint8_t RTC_day;
	uint8_t RTC_hour;
	uint8_t RTC_minute;
	uint8_t Inspector_name;
	uint8_t BT_MAC_add_front[10];
	uint8_t BT_MAC_add_back[10];

	uint8_t BT_name_front[10];
	uint8_t BT_name_back[10];
	uint32_t BT_baudrate;
	uint8_t BT_set_first;

	uint8_t CM_HW_ver;
	uint8_t RH_MD_HW_ver;
	uint8_t LH_MD_HW_ver;

    uint8_t CM_SW_ver_major;
    uint8_t CM_SW_ver_minor;
    uint8_t CM_SW_ver_patch;

	uint8_t RH_MD_SW_ver_major;
	uint8_t RH_MD_SW_ver_minor;
	uint8_t RH_MD_SW_ver_patch;
	
	uint8_t LH_MD_SW_ver_major;
	uint8_t LH_MD_SW_ver_minor;
	uint8_t LH_MD_SW_ver_patch;

    uint8_t ESP32_SW_ver_major;
	uint8_t ESP32_SW_ver_minor;
	uint8_t ESP32_SW_ver_patch;

    uint8_t CONTENTS_FILE_ver_major;
	uint8_t CONTENTS_FILE_ver_minor;
	uint8_t CONTENTS_FILE_ver_patch;

    uint16_t RH_MD_SW_ver_debug;
	uint16_t LH_MD_SW_ver_debug;
	
    uint8_t RH_MD_SW_ver;
	uint8_t LH_MD_SW_ver;

	float CM_Voltage_aver;
	float CM_Current_aver;
	float CM_Temp_aver;
	float CM_Percentage_aver;

	float RH_acc_mean_value;
	float RH_gyro_scailing;
	float RH_gyro_bias;
	float RH_mk_scaling;

	float LH_acc_mean_value;
	float LH_gyro_scaling;
	float LH_gyro_bias;
	float LH_mk_scaling;

	float RH_abs_1_value;
	float RH_abs_2_value;
	float RH_inc_value;

	float LH_abs_1_value;
	float LH_abs_2_value;
	float LH_inc_value;

	float RH_abs_value;
	float LH_abs_value;

	uint8_t INC_button_test;
	uint8_t DEC_button_test;
	uint8_t POWER_button_test;

	float CM_voltage;
	float CM_current;
	float CM_temp;

	float RH_voltage;
	float RH_current;
	float RH_temp;

	float LH_voltage;
	float LH_current;
	float LH_temp;

	uint8_t Func_SW;

	uint32_t RH_err_code;
	uint32_t LH_err_code;

	uint8_t RH_Serial_number_front[10];
	uint8_t RH_Serial_number_middle[10];
	uint8_t RH_Serial_number_back[10];

	uint8_t LH_Serial_number_front[10];
	uint8_t LH_Serial_number_middle[10];
	uint8_t LH_Serial_number_back[10];

	DOPI_DevObj_t devObj;
    uint8_t robowear_RH; 
    uint8_t robowear_LH; 

    uint8_t test1;
    uint16_t test2;
    uint32_t test3;
    float test4;

    uint8_t IsConnectSuccess;

    uint8_t RTC_data[7];

} AS_ManuData_t;

typedef struct _AS_BTModuleCMD_t {
	uint8_t IsReadyCheck;

}AS_BTModuleCMD_t;

typedef struct _AS_BLE_CtrlObj_t {
    // IO Properties
    uint16_t sdoTxID;
    uint16_t pdoTxID;
    AS_BLE_TxFncPtr txFnc;

    // Device Data
    AS_BLEData_t data;
    AS_ManuData_t data_manu;
    AS_BTModuleCMD_t data_cmd;

    // Device Object
	DOPI_DevObj_t devObj;
} AS_BLE_CtrlObj_t;

typedef struct _AS_BLECommState_t {
	IOIF_ESP32_BT_Comm_t current_state;
	IOIF_ESP32_BT_Comm_t previous_state;
} AS_BLECommState_t;

typedef struct _AS_TrainingInfo_t {
	bool isSessionActive;
	uint16_t session_duration;								// 세션 사용시간 : 세션 시작 ~ 종료
	uint16_t mode_duration[ASSIST_MODE_MAX_NUM];			// 모드별 사용시간 : 모드 시작 ~ 종료
	uint16_t operation_duration;							// 로봇 운용시간 : 로봇 동작 시간 ~ 종료 시간 (대기 모드는 제외)
	uint16_t countFor1sec_standbyState;
	uint16_t walk_count[ASSIST_MODE_MAX_NUM];
	float walk_distance[ASSIST_MODE_MAX_NUM];
	float asymmetric_level_avg[ASSIST_MODE_MAX_NUM];
	uint8_t step_length_avg[ASSIST_MODE_MAX_NUM];
} AS_TrainingInfo_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */
extern SUITBLEFlashFlag FLASHSaveFlag;

//extern FSMFileInfo 	FSM_File;

//extern BLEState isBLEConnect;
extern uint8_t LED_test;
extern uint8_t BTN_test[3];
extern uint8_t MAG_test[2];
extern uint8_t MAG_res[2];
extern DOPI_SDOMsg_t sdo_msg;
extern uint8_t powerOffCmdByBLE;
extern uint8_t ble_buff[30];
extern AS_BLECommState_t isBLEConnect;
extern float batPctg;
extern int16_t remaining_time;

extern int32_t alram_time;
//extern float prevRightThighNeutralPostureOffset;
//extern float prevLeftThighNeutralPostureOffset;

//float prevRightThighNeutralPostureOffset = 0.0f;
//float prevLeftThighNeutralPostureOffset = 0.0f;

extern AS_BLE_CtrlObj_t BLECtrlObj;

extern AS_TrainingInfo_t trainingInfo[SESSION_COUNT];
extern uint8_t current_session;

extern uint8_t prevAssistLvfromApp;

extern int16_t RightHipFlexionTorque;
extern int16_t RightHipExtensionTorque;
extern int16_t LeftHipFlexionTorque;
extern int16_t LeftHipExtensionTorque;
extern int16_t RightKneeExtensionTorque;
extern int16_t LeftKneeExtensionTorque;
extern int16_t RightKneeFlexionTorque;
extern int16_t LeftKneeFlexionTorque;

extern bool IsBLEconnectedtoApp;

extern uint8_t SPI_test_sw ;
extern uint8_t SPI_test_result;

extern uint8_t MotionAnalysisDataState;
extern uint32_t TrackUsage;
extern bool isMotionAnalysisCancel;
extern bool onetime_send_finish_able;
extern bool onetime_finish_set;

extern uint16_t GaitCycle[101];
extern int16_t ThighAngleLeftYMean[101];
extern int16_t ThighAngleLeftYStd[101];
extern int16_t ThighAngleRightYMean[101];
extern int16_t ThighAngleRightYStd[101];
extern int16_t ThighTorqueLeftYMean[101];
extern int16_t ThighTorqueLeftYStd[101];
extern int16_t ThighTorqueRightYMean[101];
extern int16_t ThighTorqueRightYStd[101];

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitBLEComm(void);
void RunBLEComm(void);

int16_t ScaleFloatToInt16(float value, float scaleFactor);

uint16_t Checksum_Calcul(uint8_t* t_test_packet, uint8_t t_length );
uint8_t Checksum_Check(uint8_t* t_test_packet, uint8_t t_length, uint16_t * t_checksum);

void FLASH_ALL_WRITE(void);
void FLASH_ALL_DOWNLOAD(void);

void SaveBLESetData_FLASH(void);
void DownloadBLESetData_FLASH(void);

/* ------------------- MANUFACTURE ------------------- */
int Check_FD_CAN_COMMU_TX(uint8_t MD_nodeID);
int Check_FD_CAN_COMMU_RX(uint8_t* t_byte_arr, uint8_t MD_nodeID);

/* ------------------- AES Encryption Functions ------------------- */
void base64_encode(const uint8_t *data, size_t input_length, size_t *output_length);

#endif /* SUIT_MINICM_ENABLED */

#endif /* BLE_COMM_HDLR_INC_AS_BLE_COMM_HDLR_H_ */
