/**
 * @file wholeBodyCtrlTask.c
 * @date Created on: Sep 21, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */
#include "AS_whole_body_ctrl.h"
#include "queue.h"
#include "AS_data_ctrl.h"
#include "ioif_flash_common.h"

#include "ext_link_mngr.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */
extern uint16_t LH_MD_sw_ver;
extern uint16_t RH_MD_sw_ver;
extern AS_BLECommState_t isBLEConnect;
extern uint8_t can_test[16];

uint8_t ey_debug = 0;

TaskObj_t wholeBodyCtrlTask;
extern osSemaphoreId_t sdio_sync_semaphore;			//sdio transmit sync semaphore
extern osMutexId_t mutex_sd_buffering;				//buffering mutex
extern SUIT_AudioState_t SuitAudioState;
extern uint32_t total_error_code;
extern uint32_t p_total_error_code;

MotionAnalysisConditions MotionFlags;

SAM_Motion_Struct RH_Sagittal;
SAM_Motion_Struct LH_Sagittal;
SAM_Motion_Struct RK_Sagittal;
SAM_Motion_Struct LK_Sagittal;

/////////////////// Contents File System //////////////////////////////////////////////////////
/* File-system */
IOIF_fCMD_res_t 	contentsfile_mountRes = FR_NOT_READY;

FATFS		contentsfsObj 	__attribute__((section(".FATFS_RAMD1_data")));
IOIF_FILE_t	contentsfileObj	__attribute__((section(".FATFS_RAMD1_data")));

/* Contents Files */
read_contents_status FSM_res = READ_CONTENTS_ERROR;
read_contents_status RS_res = READ_CONTENTS_ERROR;
read_contents_status MM_res = READ_CONTENTS_ERROR;
read_contents_status DMS_res = READ_CONTENTS_ERROR;

/* extern variables */
extern RobotSettingFileInfo RS_File;			// Info. Structure : Robot Setting File
extern FSMFileInfo 			FSM_File;			// Info. Structure : FSM File
extern DMSFileInfo 			DMS_File;			// Info. Structure : DMS File
extern MotionMapFileInfo 	MotionMap_File;		// Info. Structure : Motion Map File

extern osSemaphoreId_t BinSem_PlaySoundHandle;
extern audioType_t audioOutputMode;
extern MainSequence_Enum MD_state;

extern uint32_t mcu_temp;
extern bool isFaultOccurred;

extern SuitMode_t CMMode;

/* ---- Extension Module ---- */
// 통신 및 상태 관리용 객체 및 변수
extern ExtLinkMngrInst_t g_ext_link_mngr; // NMT 상태 관리 객체
extern ExtCommLinkObj_t g_ext_comm_obj; // 통신 채널 객체
/* ---- Extension Module ---- */

/**
 *------------------------------------------------------------
 *                            VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

int16_t remaining_time= -1;
int32_t alram_time= -2;

uint32_t assistModeLoopCnt;

uint32_t wholeBodyCtrlLoopCnt;
uint32_t pwrOffCnt;
uint16_t StandbyCnt;
uint8_t standby_sw;
static uint8_t lastStateCurr;

//float   SittoStandDuration = 5;
//float   StandtoSitDuration = 2.5;
//uint8_t  StandtoSitAssistLevel = 0;
//uint8_t  SittoStandAssistLevel = 0;

uint16_t SUIT_totalDuration = 0;
uint16_t SUIT_Duration = 0;
uint16_t countFor1sec = 0; // 1초 카운트를 위한 변수
uint16_t countFor1secSTS = 0;	// STS Mode 상태분석을 위한 1초 카운트 변수

uint16_t is_start;

/* For Data Save */
//////////////////////////////////////////////////////////////////////////////////////////////
//uint8_t sd_buf_1[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 1
//uint8_t sd_buf_2[SWAP_SD_BUF_SIZE] __attribute__((section(".FATFS_RAMD1_data")));	// switching sd buffer 2

uint_fast16_t* cur_buf_pos;
uint_fast16_t cur_buf_space;

uint_fast16_t sd_buf_1_pos = 0;     //sd buf 1 position(index)
uint_fast16_t sd_buf_2_pos = 0;     //sd buf 2 position(index)

uint8_t *sd_buf_cur = sd_buf_1;     //current buffer pointer : start with sd buf 1
uint8_t *sd_buf_next = sd_buf_2;    //next buffer pointer

uint8_t hexDataCursor = 0;
uint8_t sdSendDataToHex[HEX_BUF_SIZE]; // data buffer to hex

uint8_t test;
uint8_t pdo_test_0;
uint8_t pdo_test_1;
uint8_t pdo_test_2;
uint8_t pdo_test_3;

int convert_res = 0;
uint32_t dataSaveCnt = 0;

float inter_para[6] = {0, 0, 0, 5, 0, 1};

PIDObject MANU_RH = {0, };
PIDObject MANU_LH = {0, };

uint8_t oneTimeSave = 0;

uint8_t dataSaveFinished = 0;

//////////////////////////////////////////////////////////////////////////////////////////////
uint8_t isSUITReady = 0;
uint8_t neutralPosCalCMD = 0;
bool isNeutralCalCompleted = true;
bool isNeutralSaved = true;

uint8_t canOperate = 0;

float STS_torque = 0;
float f_STS_torque = 0;
uint16_t STS_cnt = 0;
uint8_t STS_odd = 0;

float RightHipSmartAssistGravityCompensation;
float LeftHipSmartAssistGravityCompensation;
float RightHipSmartAssistVelocityCompensation = 0.4;
float LeftHipSmartAssistVelocityCompensation = 0.4;

float RightKneeSmartAssistGravityCompensation;
float LeftKneeSmartAssistGravityCompensation;
float RightKneeSmartAssistVelocityCompensation;
float LeftKneeSmartAssistVelocityCompensation;

float RightHipAquaGravityCompensation;
float LeftHipAquaGravityCompensation;
float RightHipAquaVelocityCompensation;
float LeftHipAquaVelocityCompensation;

float RightKneeAquaGravityCompensation;
float LeftKneeAquaGravityCompensation;
float RightKneeAquaVelocityCompensation;
float LeftKneeAquaVelocityCompensation;

float RightHipUnivGravityCompensation;
float LeftHipUnivGravityCompensation;
float RightHipUnivVelocityCompensation;
float LeftHipUnivVelocityCompensation;

float RightKneeUnivGravityCompensation;
float LeftKneeUnivGravityCompensation;
float RightKneeUnivVelocityCompensation;
float LeftKneeUnivVelocityCompensation;

float RightHipFlexionLimit;
float RightHipExtensionLimit;
float RightHipFlexionVelocityLimit;
float RightHipExtensionVelocityLimit;

float LeftHipFlexionLimit;
float LeftHipExtensionLimit;
float LeftHipFlexionVelocityLimit;
float LeftHipExtensionVelocityLimit;

float RightKneeFlexionLimit;
float RightKneeExtensionLimit;
float RightKneeVelocityLimit;
float LeftKneeFlexionLimit;
float LeftKneeExtensionLimit;
float LeftKneeVelocityLimit;

float LeftHipMaxAngle;
float RightHipMaxAngle;
float LeftHipMinAngle;
float RightHipMinAngle;
float LeftHipMaxVelocity;
float RightHipMaxVelocity;
float LeftHipMinVelocity;
float RightHipMinVelocity;

uint8_t GaitStop = 0;
uint8_t FirstStep = 0;
uint8_t halfStep = 0;

uint32_t gaitCountLeft[2];
uint32_t gaitCountRight[2];
float gaitCount[2];
float gaitLapCount;
uint32_t LeftGaitCountOn = 0;
uint32_t RightGaitCountOn = 0;

float RHamplitude, RHamplitude_max[2], RHamplitude_min[2];
float LHamplitude, LHamplitude_max[2], LHamplitude_min[2];

float GaitRHThighSagittalAngle_max, GaitRHThighSagittalAngle_min;
float GaitLHThighSagittalAngle_max, GaitLHThighSagittalAngle_min;

float GaitRHThighSagittalVelocity_max, GaitRHThighSagittalVelocity_min;
float GaitLHThighSagittalVelocity_max, GaitLHThighSagittalVelocity_min;

uint8_t FSM_prev_state;

float Asymmetry[2];
float AsymmetryFilter;
float AsymmetryAvg = 0;
double AsymmetryAvgSum = 0;
uint32_t AsymmetryAvgCnt = 1;

float R_ThighSagittalAngle;
float L_ThighSagittalAngle;
float R_ThighSagittalVelocity;
float L_ThighSagittalVelocity;
float R_ThighSagittalVelocity_1;
float L_ThighSagittalVelocity_1;
float R_ThighSagittalVelocity_2;
float L_ThighSagittalVelocity_2;
float R_ThighSagittalVelocity_3;
float L_ThighSagittalVelocity_3;
float L_ShankSagittalAngle;
float R_ShankSagittalAngle;

float RH_PositionAct;
float LH_PositionAct;
float RH_EncoderVelocity;
float LH_EncoderVelocity;

float RH_IMU_TVCFDegree;
float LH_IMU_TVCFDegree;
float RH_IMU_BodyAngle_Sagittal, LH_IMU_BodyAngle_Sagittal;
float RH_IMU_BodyAngle_Frontal, LH_IMU_BodyAngle_Frontal;
float RK_PositionAct;
float LK_PositionAct;
float RK_EncoderVelocity;
float LK_EncoderVelocity;
float RK_IMU_BodyAngle_Sagittal, LK_IMU_BodyAngle_Sagittal;
float RK_IMU_BodyAngle_Frontal, LK_IMU_BodyAngle_Frontal;

float PelvicSagittalAngle, PelvicFrontalAngle;

float RH_ThighNeutralSagittalAngle;
float LH_ThighNeutralSagittalAngle;
float RK_ThighNeutralSagittalAngle;
float LK_ThighNeutralSagittalAngle;

float RH_accXCalib, RH_accYCalib, RH_accZCalib;
float LH_accXCalib, LH_accYCalib, LH_accZCalib;
float RH_gyroXCalib, RH_gyroYCalib, RH_gyroZCalib;
float LH_gyroXCalib, LH_gyroYCalib, LH_gyroZCalib;

float RK_accXCalib, RK_accYCalib, RK_accZCalib;
float LK_accXCalib, LK_accYCalib, LK_accZCalib;
float RK_gyroXCalib, RK_gyroYCalib, RK_gyroZCalib;
float LK_gyroXCalib, LK_gyroYCalib, LK_gyroZCalib;

float RH_accXGlobal[2], RH_accYGlobal[2], RH_accZGlobal[2];
float LH_accXGlobal[2], LH_accYGlobal[2], LH_accZGlobal[2];
float RH_gyroXGlobal[2], RH_gyroYGlobal[2], RH_gyroZGlobal[2];
float LH_gyroXGlobal[2], LH_gyroYGlobal[2], LH_gyroZGlobal[2];

float RK_accXGlobal[2], RK_accYGlobal[2], RK_accZGlobal[2];
float LK_accXGlobal[2], LK_accYGlobal[2], LK_accZGlobal[2];
float RK_gyroXGlobal[2], RK_gyroYGlobal[2], RK_gyroZGlobal[2];
float LK_gyroXGlobal[2], LK_gyroYGlobal[2], LK_gyroZGlobal[2];

// For Extension Board
float pelvic_Vel_Y; // Pevlc Yaw Motion

// 앉기 서기 서기 인식용 변수
float L_STS_stand_start_thigh_angle;
float R_STS_stand_start_thigh_angle;

/* 앉기 서기 보조 모드 테스트 */
uint8_t STSMode = 0;
uint8_t AssistMode = 0;

uint8_t PostureHold;
uint8_t SitReadyButton;
uint8_t StartSittingButton;
uint8_t StandReadyButton;
uint8_t StartStandingButton;

uint8_t STS_P_Vector_StartSittingDuration;
uint8_t STS_P_Vector_StartStandingDuration;

uint8_t STS_P_Vector_Duration_Completed_RH;
uint8_t STS_P_Vector_Duration_Completed_LH;
uint8_t STS_P_Vector_Duration_Completed_RK;
uint8_t STS_P_Vector_Duration_Completed_LK;

// STS Mode Pro App에 띄울 내용
uint16_t sitCount = 0;
uint16_t standCount = 0;
uint32_t sitTime =0;
uint32_t standTime=0;
uint32_t accumulateSitTime=0;
uint32_t accumulateStandTime=0;
uint8_t prev_state=0;
uint8_t prev_FSM_state=0;
float sumY_L;
float sumY_R;
float sumsumY_L;
float sumsumY_R;
uint8_t FSMStatePrev;

// Post Processing Save용
bool isFirstSaveData = true;
bool isSecondSaveData = false;
bool isThirdSaveData = false;

uint8_t secondSaveDataCnt;
uint8_t thirdSaveDataCnt;

uint32_t FirstDataSwapCnt;
uint32_t SecondDataSwapCnt;
uint32_t ThirdDataSwapCnt;

// Meta Data Logging
MetaItem_t meta;
extern osMessageQueueId_t sensorDataLogQueueHandle;
Sensor_LogPacket_V1_t sensordata;
extern AssistStage assistStage;

/* For air walking modes */
#ifdef AIR_WALKING_MODE_ENABLED
uint8_t Airwalking_pos = 0;
uint8_t Airwalking_Step = 0;
uint32_t aging_inter_cnt = 0;
uint8_t aging_speed_param = 0;
#endif
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);
static void StateOff_Ext(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static void CalculateDuration(AS_TrainingInfo_t* currentSession);
static void CheckSAMIMU(SAM_Motion_Struct* RH_MotionData, SAM_Motion_Struct* LH_MotionData);
static void CheckSAMFDCAN(void);
static void CheckStateNSetCompGain(void);
static void CheckMDJointLimitRoutine(void);
static void SetMaxMinJointAngleVelocity(void);
static void CheckCanOperate(void);
static void CheckNeutralPostureCMD(void);

static float ScaleInt16ToFloat(int16_t value, float scaleFactor);
static float ScaleUInt16ToFloat(uint16_t value, float scaleFactor);

/* Data Storage For Battery Usage */
//static void SaveBatUsage(void);

/* For Data Save */
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len);			// Data buffering Transmission functions

/* Data Save To Hex */
static char Num2Str(uint8_t num);
static void Append_SD_Data_Float16(uint8_t t_size, float t_data, float conversion_constant);
static void Append_SD_Data_Uint8(uint8_t t_size, uint8_t t_data);
static void Append_SD_Data_Uint16(uint8_t t_size, uint16_t t_data);
static void Append_SD_Data_Uint32(uint8_t t_size, uint32_t t_data);
static void Append_SD_Data_Uint64(uint8_t t_size, uint64_t t_data);

static void SaveDataMassStorage(void);
static void AppendSDData(SAM_Motion_Struct MotionData, int DevIdx);

static void InitSAMMotionData(SAM_Motion_Struct* SAM_MotionData);
static void UpdatePDOData(void);
static void UpdateMotionAnalysis_H10(void);
static void UpdateMotionAnalysis_K10(void);
static void GaitAnalysis(void);
static void STSAnalysis(void);

static void DecodingPDOData(SAM_Motion_Struct* MotionData, int DevIdx);
static void UpdateMotionData(void);

/* --------------------- SDO CALLBACK --------------------- */
static void Set_RH_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_LH_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_RK_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_LK_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_RH_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_LH_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_RK_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_LK_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void Set_Acc_Mean_Value(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

static void SetMetadata(void);
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(wholeBodyCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitWholeBodyCtrl(void)
{
	MANU_RH.Kp = 0.05;
	MANU_RH.Kd = 0.005;

	MANU_LH.Kp = 0.05;
	MANU_LH.Kd = 0.005;

	/* Init Task */
	InitTask(&wholeBodyCtrlTask);
	DOPC_AssignTaskID(&wholeBodyCtrlTask, TASK_IDX_WHOLE_BODY_CTRL);

	/* State Definition */
	TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_OFF,      StateOff_Ent,	    StateOff_Run,     StateOff_Ext,			true);
	TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_STANDBY,  NULL,			    StateStandby_Run, NULL,					false);
	TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run,  StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&wholeBodyCtrlTask,  TASK_STATE_ERROR,    NULL,			    StateError_Run,   NULL,					false);

	/* Routine Definition */

	// DOD
	DOP_CreateDOD(TASK_IDX_WHOLE_BODY_CTRL);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL)

	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_RH_NeutralPosture,	DOP_FLOAT32,	Set_RH_NeutralPosture);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_LH_NeutralPosture,	DOP_FLOAT32,	Set_LH_NeutralPosture);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_RK_NeutralPosture,	DOP_FLOAT32,	Set_RK_NeutralPosture);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_LK_NeutralPosture,	DOP_FLOAT32,	Set_LK_NeutralPosture);

	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_RH_P_VECTOR_DURATION_COMPLETED,	DOP_UINT8,	Set_RH_P_Vector_Duration_Completed);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_LH_P_VECTOR_DURATION_COMPLETED,	DOP_UINT8,	Set_LH_P_Vector_Duration_Completed);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_RK_P_VECTOR_DURATION_COMPLETED,	DOP_UINT8,	Set_RK_P_Vector_Duration_Completed);
	DOP_CreateSDO(TASK_ID_WHOLEBODY,	SDO_ID_WHOLEBODY_LK_P_VECTOR_DURATION_COMPLETED,	DOP_UINT8,	Set_LK_P_Vector_Duration_Completed);

	DOP_CreateSDO(TASK_ID_WHOLEBODY, 	SDO_ID_WHOLEBODY_ACC_MEAN_VALUE, 				DOP_FLOAT32, Set_Acc_Mean_Value);

	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_WHOLE_BODY_CTRL, wholeBodyCtrlTask);

	/***** (1) Download FSM *****/
	// Load FSM File on Flash to RAM
	// Download_FSM1();
	//Download_Test_FSM(FSM2_File);

	/***** (2) Download MotionMap *****/
	/* (Begin) For File Save Unit Test */
	//Make_MotionMap_Examples();
	//Save_MotionMap();
	// Download_MotionMap();
	// Get_Max_PFI_Vectors_Length();
	//MM_SAVE_GOOD = Check_MM_Save();
	//Get_Max_PFI_Vectors_Length();
	/* (End) For File Save Unit Test */

	/***** (3) Download DMS *****/
	/* (Begin) For File Save Unit Test */
	//Make_DMS_Examples();
	//Save_DMS();
	// Download_DMS();
	//DMS_SAVE_GOOD = Check_DMS_Save();
	/* (End) For File Save Unit Test */
	//Download_Test_DMS(DMSFile1);

	/***** (4) Download RS *****/
	// Load Robot Setting File on Flash to RAM
	// Download_RobotSetting();
	/* (End) For File Save Unit Test */
	//Download_Test_RobotSetting(RS_File1);
	//    Download_Test_RobotSetting(RS_File_AS_Test);

	/* Initiation */
	// All_MD_Init();
	//    InitFDCANDevMngr();
	//    Make_Overall_PDODataList();
}

void RunWholeBodyCtrl(void)
{
	/* Run Device */
	RunTask(&wholeBodyCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */
float traj_calcul(float traj_para[], int time, float time_amp, float mag_amp, int time_offset)
{
	float traj_time_to_0 = (traj_para[0]) * 1000 * time_amp;
	float traj_time_to_1 = (traj_para[0] + traj_para[1]) * 1000 * time_amp;
	float traj_time_to_2 = (traj_para[0] + traj_para[1] + traj_para[2]) * 1000 * time_amp;
	float traj_time_to_3 = (traj_para[0] + traj_para[1] + traj_para[2] + traj_para[3]) * 1000 * time_amp;
	float traj_time_to_4 = (traj_para[0] + traj_para[1] + traj_para[2] + traj_para[3] + traj_para[4]) * 1000 * time_amp;
	float res = 0;
	float y_0;
	float y_d;
	float s_0;
	float s_d;
	float a_0;
	float a_1;
	float a_2;
	float a_3;
	float a_4;
	float a_5;

	if (time - time_offset < traj_time_to_0)
	{
		res = 0;
	}
	else if (time - time_offset >= traj_time_to_0 && time - time_offset < traj_time_to_1)
	{
		y_0 = 0;
		y_d = traj_para[5] * mag_amp;
		s_0 = 0;
		s_d = 0;
		a_0 = y_0;
		a_2 = 0.5 * s_0 * (y_d - y_0);
		a_3 = (10 - 1.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		a_4 = (-15 + 1.5 * s_0 - s_d) * (y_d - y_0);
		a_5 = (6 - 0.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		res = a_0 + a_2 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_3 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_4 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) + a_5 * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp)) * ((double)(time - traj_time_to_0 - time_offset) / (traj_para[1] * 1000 * time_amp));
	}
	else if (time - time_offset >= traj_time_to_1 && time - time_offset < traj_time_to_2)
	{
		res = traj_para[5] * mag_amp;
	}
	else if (time - time_offset >= traj_time_to_2 && time - time_offset < traj_time_to_3)
	{
		y_0 = traj_para[5] * mag_amp;
		y_d = 0;
		s_0 = 0;
		s_d = 0;
		a_0 = y_0;
		a_2 = 0.5 * s_0 * (y_d - y_0);
		a_3 = (10 - 1.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		a_4 = (-15 + 1.5 * s_0 - s_d) * (y_d - y_0);
		a_5 = (6 - 0.5 * s_0 + 0.5 * s_d) * (y_d - y_0);
		res = a_0 + a_2 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_3 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_4 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) + a_5 * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp)) * ((double)(time - traj_time_to_2 - time_offset) / (traj_para[3] * 1000 * time_amp));
	}
	else if (time - time_offset >= traj_time_to_3 && time - time_offset < traj_time_to_4)
	{
		res = 0;
	}

	if (traj_para[5] > 0)
	{
		if (res > traj_para[5] * mag_amp || res < -traj_para[5] * mag_amp)
		{
			res = 0;
		}
	}
	else
	{
		if (res > -traj_para[5] * mag_amp || res < traj_para[5] * mag_amp)
		{
			res = 0;
		}
	}
	return res;
}

void Run_MANU_PID_Control(PIDObject *t_PID_obj, float t_ref, float t_actual, float t_period)
{
	float t_err;

	t_PID_obj->ref = t_ref;
	t_PID_obj->act = t_actual;

	t_err = t_ref - t_actual;

	if((t_actual >105) || (t_actual < -35)) t_err = t_PID_obj->err;

	t_PID_obj->err_sum += t_err * t_period;
	t_PID_obj->err_diff = (t_err - t_PID_obj->err) / t_period;
	t_PID_obj->err = t_err;

	t_PID_obj->control_input = t_PID_obj->Kp * t_PID_obj->err + t_PID_obj->Ki * t_PID_obj->err_sum + t_PID_obj->Kd * t_PID_obj->err_diff;
	if (t_PID_obj->control_input > 2)
	{
		t_PID_obj->control_input = 2;
	}
	else if (t_PID_obj->control_input < -2)
	{
		t_PID_obj->control_input = -2;
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void)
{
	wholeBodyCtrlLoopCnt = 0;

	contentsfile_mountRes = IOIF_FileMount(&contentsfsObj, (uint8_t*)"0:");	// Mount the file system

	FSM_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_FSM_TEST.csv", OPERATION_FSM);
	RS_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_RS.csv", ROBOT_SETTING);
	//DMS_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_DMS.csv", DMS); //DMS 사용 안하는 중
	MM_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_MM.csv", MOTION_MAP);

}

static void StateOff_Run(void)
{
	uint8_t t_riskMngState = DOPC_GetTaskState(TASK_IDX_RISKMNGT_HDLR);
	if (t_riskMngState == TASK_STATE_STANDBY
			&& FSM_res == READ_CONTENTS_OK && RS_res == READ_CONTENTS_OK
			/*&& DMS_res == READ_CONTENTS_OK*/ && MM_res == READ_CONTENTS_OK) {
		StateTransition(&wholeBodyCtrlTask.stateMachine, TASK_STATE_STANDBY);
	}

	if(FSM_res != READ_CONTENTS_OK)
		FSM_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_FSM_TEST.csv", OPERATION_FSM);
	else if(RS_res != READ_CONTENTS_OK)
		RS_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_RS.csv", ROBOT_SETTING);
    /*else if(DMS_res != READ_CONTENTS_OK) //DMS 사용 안하는 중
		DMS_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_DMS.csv", DMS);*/
	else if(MM_res != READ_CONTENTS_OK)
		MM_res = ReadContentFile(&contentsfileObj, (uint8_t*)"ContentsFiles/SUIT_MM.csv", MOTION_MAP);

// 해당 에러 적용하려면 전체 APP layer task enable 구조 변경해야함 -> 추후 적용
/*	if (wholeBodyCtrlLoopCnt > 100)
	{
		FaultArrange_t ID = {6,0};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
	}*/

	wholeBodyCtrlLoopCnt++;
}

static void StateOff_Ext(void)
{
	Download_RobotSetting();
	InitFDCANDevMngr();
	Make_Overall_PDODataList();

	/* Extension Board Link Manager Init */
	ExtLinkMngr_Init(); // 0xC is Extension Board Node ID

	wholeBodyCtrlLoopCnt = 0;
	is_start = 0;
}

static void StateStandby_Run(void)
{
	/* STEP 1) Activate MD & GRF CANFD */
	if (is_start == MSG_HDLR_ACTIVATE_CNT)	        { AllSetDevPDO(); }

	/* STEP 2) Homing Incremental Encoder Angles using Absoulte Encoder Angles */
	/* STEP 3) Initialize All MD C vector as 0 */
	/* STEP 4) Send MotionMap for Initial Motion */
	/* STEP 5) Set Device Routines & State Enable */
	if (is_start == ALL_DEV_ROUTINE_ACTIVATE_CNT)	{ AllDevSetRoutines(); }

	if (is_start == ALL_DEV_TASK_STATE_CNT)	        { AllDevEnableStates(); }

	if (is_start > WHOLE_BODY_LOOP_START_CNT) {
		uint8_t t_sysCtrlState = DOPC_GetTaskState(TASK_IDX_SYSTEM_CTRL);
		if (t_sysCtrlState == TASK_STATE_ENABLE /* &&  ey_debug == 1 */) {
			StateTransition(&wholeBodyCtrlTask.stateMachine, TASK_STATE_ENABLE);
		}
	}
	is_start++;
}

static void StateEnable_Ent(void)
{
	is_start = 0;
	Init_FSM();
	InitSAMMotionData(&RH_Sagittal);
	InitSAMMotionData(&LH_Sagittal);
	InitSAMMotionData(&RK_Sagittal);
	InitSAMMotionData(&LK_Sagittal);
	wholeBodyCtrlLoopCnt = 0;
	ReqAccMeanValue(RH_SAGITAL);
	ReqAccMeanValue(LH_SAGITAL);

	// 동작 분석 데이터 초기화
	InitGaitAnalysis();
}


static void StateEnable_Run(void)
{
	if(remaining_time > 0 && wholeBodyCtrlLoopCnt%1000 == 0){
		remaining_time--;
	}

	if(remaining_time == 0){

		if(alram_time == -1){
			alram_time = 30000;
		}

		if(alram_time>0){
			if(alram_time%1000==0)
			{
			if (SuitAudioState.audio_Isplay != true)
				{
					SuitAudioState.audio_id = AUDIO_ID_106;
					osSemaphoreRelease(BinSem_PlayBeepHandle);
				}
			}
			alram_time--;
			if(alram_time==0 )
			{
				alram_time = -2 ;
				remaining_time = -1;
			}
		}
	}

	/*RM*/
	if (md_error_flag == true) { //enable_ent시 초기화 됨 -> state 변경 x
		for (uint8_t i = 0; i < RM_MD_ERROR_ARRAY; i++) {
			FlagMDFault(i);
		}
		FaultTaskRun();

		memset(mdErrorBit, 0, sizeof(mdErrorBit));
		md_error_flag = false;
	}

	if(manu_mode==1 && prev_manu_mode == 0){
//		Send_I_Vector(2, 0, 250, 0, 0, 300);
//		Send_I_Vector(3, 0, 250, 0, 0, 300);

		DOPI_SDOUnit_t sdoUnit;
		DOPI_ClearSDO(&sdo_msg);

		userCtrlObj[RH_SAGITAL].data.joint_limit_sw = 1;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[RH_SAGITAL].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(2);

		DOPI_ClearSDO(&sdo_msg);
		userCtrlObj[LH_SAGITAL].data.joint_limit_sw = 1;
		sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[LH_SAGITAL].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
		DOPI_AppendSDO(&sdoUnit, &sdo_msg);

		TxSDO(3);
	}

	if (manu_mode == 1) {
		if(motion_test_mode == 0){
			if(inter_cnt++== 0){
				Send_I_Vector(2, 0, 0, 0, 0, 300);
				Send_I_Vector(3, 0, 0, 0, 0, 300);
			}
			//else if(inter_cnt < 1000000)	inter_cnt++;
		}
		else if(motion_test_mode == 1){

			if(inter_cnt==0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 0, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(1050), (uint16_t)(1900), 0, 0);
			}

			if(inter_cnt<2000){
				inter_cnt++;
			}
		}
		else if(motion_test_mode == 2){

			if(inter_cnt==0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 0, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(-200), (uint16_t)(1900), 0, 0);
			}

			if(inter_cnt<2000){
				inter_cnt++;
			}
		}

		if(motion_test_mode == 3){

			if(inter_cnt==0){
				Send_I_Vector(2, 0, 0, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(3, (int16_t)(1050), (uint16_t)(1900), 0, 0);
			}


			if(inter_cnt<2000){
				inter_cnt++;
			}
		}
		else if(motion_test_mode == 4){

			if(inter_cnt==1){
				Send_I_Vector(2, 0, 0, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(3, (int16_t)(-200), (uint16_t)(1900), 0, 0);
			}

			if(inter_cnt<2000){
				inter_cnt++;
			}
		}
		else if(motion_test_mode == 5){
			if(inter_cnt==0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(1050), (uint16_t)(1900), 30, 30);
				Send_P_Vector(3, (int16_t)(-200), (uint16_t)(1500), 30, 30);
			}

			if(inter_cnt<2000){
				inter_cnt++;
			}
			else if(aging_enable==1 && inter_cnt>=2000)
			{
				motion_test_mode = 6;
				inter_cnt = 0;

			}
		}
		else if(motion_test_mode == 6){
			if(inter_cnt==0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(-200), (uint16_t)(1500), 30, 30);
				Send_P_Vector(3, (int16_t)(1050), (uint16_t)(1900), 30, 30);
			}

			if(inter_cnt<2000){
				inter_cnt++;
			}
			else if(aging_enable==1 && inter_cnt>=2000)
			{
				motion_test_mode = 5;
				inter_cnt = 0;
			}
		}
	}

	if(standby_sw == 1)
	{
		StateTransition(&wholeBodyCtrlTask.stateMachine, TASK_STATE_STANDBY);
		standby_sw = 0;
	}

#ifdef AIR_WALKING_MODE_ENABLED
	if(Airwalking_Step >= AIRWALKING_MODE_STBY)
	{
		static uint16_t Upper_L = 1900;	//control speed parameter
		static uint16_t Lower_L = 1500; //control speed parameter
		static uint16_t inter_cnt_limit = 2000; //control speed parameter
		static uint8_t  update_speed_param = 0;

		static bool joint_lim_switch_disable = false;

		DOPI_SDOUnit_t sdoUnit;

		/* joint limit protection disabling*/
		if(joint_lim_switch_disable == false)
		{
			DOPI_ClearSDO(&sdo_msg);

			userCtrlObj[RH_SAGITAL].data.joint_limit_sw = 1;
			sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[RH_SAGITAL].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
			DOPI_AppendSDO(&sdoUnit, &sdo_msg);

			TxSDO(2);

			DOPI_ClearSDO(&sdo_msg);
			userCtrlObj[LH_SAGITAL].data.joint_limit_sw = 1;
			sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[LH_SAGITAL].devObj, TASK_ID_LOWLEVEL, SDO_ID_LOWLEVEL_DISABLE_JOINT_LIMIT, SDO_REQU, 1);
			DOPI_AppendSDO(&sdoUnit, &sdo_msg);

			TxSDO(3);

			joint_lim_switch_disable = true;
		}

		if(Airwalking_pos == AIRWALKING_OFF)
		{
			if(aging_inter_cnt++ == 0){
				Send_I_Vector(2, 0, 0, 0, 0, 300);
				Send_I_Vector(3, 0, 0, 0, 0, 300);
			}
		}

		else if(Airwalking_pos == AIRWALKING_NEUTRAL)
		{
			/* 1. Current Position Update */
			DOPI_ClearSDO(&sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[RH_SAGITAL].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
			DOPI_AppendSDO(&sdoUnit, &sdo_msg);
			TxSDO(2);

			DOPI_ClearSDO(&sdo_msg);
			sdoUnit = DOPI_CreateSDOUnit(&userCtrlObj[LH_SAGITAL].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_SET_YD_F, SDO_REQU, 1);
			DOPI_AppendSDO(&sdoUnit, &sdo_msg);
			TxSDO(3);

			/* 2. Operation start */
			Airwalking_pos = AIRWALKING_LEFTUP_RIGHTDOWN;
		}

		else if(Airwalking_pos == AIRWALKING_LEFTUP_RIGHTDOWN)
		{
			/* parameter update */
			update_speed_param = aging_speed_param;

			if(aging_inter_cnt == 0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(1050), Upper_L * update_speed_param, 30, 30);
				Send_P_Vector(3, (int16_t)(-200), Lower_L * update_speed_param, 30, 30);
			}

			if(aging_inter_cnt < (inter_cnt_limit * update_speed_param)){
				aging_inter_cnt++;
			}
			else if(aging_inter_cnt >= (inter_cnt_limit * update_speed_param) && Airwalking_Step > AIRWALKING_MODE_STBY)
			{
				Airwalking_pos = AIRWALKING_LEFTDOWN_RIGHTUP;
				aging_inter_cnt = 0;

			}
		}
		else if(Airwalking_pos == AIRWALKING_LEFTDOWN_RIGHTUP)
		{
			/* parameter update */
			update_speed_param = aging_speed_param;

			if(aging_inter_cnt == 0){
				Send_I_Vector(2, 0, 250, 0, 0, 300);
				Send_I_Vector(3, 0, 250, 0, 0, 300);
				Send_P_Vector(2, (int16_t)(-200), Lower_L * update_speed_param, 30, 30);
				Send_P_Vector(3, (int16_t)(1050), Upper_L * update_speed_param, 30, 30);
			}

			if(aging_inter_cnt < (inter_cnt_limit * update_speed_param)){
				aging_inter_cnt++;
			}
			else if(aging_inter_cnt >= (inter_cnt_limit * update_speed_param) && Airwalking_Step > AIRWALKING_MODE_STBY)
			{
				Airwalking_pos = AIRWALKING_LEFTUP_RIGHTDOWN;
				aging_inter_cnt = 0;
			}
		}
	}
#endif
	/* Update Current Data */
	UpdatePDOData();
	UpdateMotionData();
	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
		UpdateMotionAnalysis_H10();
	}
	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		UpdateMotionAnalysis_K10();
	}

	GaitAnalysis();
	GaitAnalysis_Improved();

	/* Check FDCAN(SAM10) Periodically */
	if (!(FSMMngrObj.state_curr >= 21 && FSMMngrObj.state_curr <= 23)
			&& audioOutputMode == AUDIO_INVALID) {
		CheckSAMFDCAN(); /* 오디오 겹침 방지 (RM 적용 전) */
	}
	/* For Neutral Posture */
	CheckNeutralPostureCMD();
	/* Check Can Operate */
	CheckCanOperate();
	/* Check I2C Error(IMU TVCF) */
	if (CMMode == SUIT_STANDBY_MODE) {
		CheckSAMIMU(&RH_Sagittal, &LH_Sagittal);
	} 

	/* FSM */
	Run_FSM();

	static bool oneTimeSetCompGain = false;
	if (!g_ext_link_mngr.is_ext_pack_connected) { // extension module과 연결되지 않을 시 기존 보조 알고리즘 수행
		CheckStateNSetCompGain();
		oneTimeSetCompGain = true;
	} else { // extension module과 연결시 extension module에서 설계한 보조 알고리즘 수행.
		if (g_ext_comm_obj.data.H10_Existing_Assist_Mode_Flag) { // XM10 연결시에도 H10 기존 보조 알고리즘 사용시
			CheckStateNSetCompGain();
			oneTimeSetCompGain = true;
		} else {
			if (oneTimeSetCompGain) {
				Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
				Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
				oneTimeSetCompGain = false;
			}
		}
	}
	
	/* MD 관절제한 루틴 On/Off */
//	CheckMDJointLimitRoutine();
	/* 관절제한모드에서 관절(엔코더) 각(속)도 최대최소 업데이트 */
	SetMaxMinJointAngleVelocity();

	if (STSMode == 1) {
		BFlag[70] = 1;
		STSMode = 0;
		AssistMode = STS_MODE;
	}

	/* Data Storage */
#ifdef SENSOR_DATA_LOGGING_ENABLED
	SaveDataMassStorage();
	#endif 

	/* Session Time Count */
	CalculateDuration(&trainingInfo[current_session]);

	if (AssistMode == STS_MODE){
		STSAnalysis();
	}
	if(FSMMngrObj.state_curr != prev_FSM_state){

		DOPI_SDOUnit_t sdo_unit;
		DOPI_ClearSDO(&sdo_msg);
		userCtrlObj[RH_SAGITAL].data.FSM_curr = FSMMngrObj.state_curr;
		sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[RH_SAGITAL].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FSM_CURR, SDO_REQU, 1);
		DOPI_AppendSDO(&sdo_unit, &sdo_msg);

		TxSDO(2);

		DOPI_ClearSDO(&sdo_msg);
		userCtrlObj[LH_SAGITAL].data.FSM_curr = FSMMngrObj.state_curr;
		sdo_unit = DOPI_CreateSDOUnit(&userCtrlObj[LH_SAGITAL].devObj, TASK_ID_MIDLEVEL, SDO_ID_MIDLEVEL_FSM_CURR, SDO_REQU, 1);
		DOPI_AppendSDO(&sdo_unit, &sdo_msg);

		TxSDO(3);
	}

	/* ---- Extension Module ---- */
	// 10ms 주기로 네트워크 관리(PnP)
    static uint32_t task_counter_10ms = 0;
	if (task_counter_10ms >= 10) {
		task_counter_10ms = 0;
		ExtLinkMngr_Execute(); // PnP 상태 관리 함수 호출
	}
    // PDO 데이터 전송 - 1ms 마다 조건부 실행
    // 조건: 
    //   - CM의 NMT 상태가 'OPERATIONAL'이고,
    //   - XM의 첫 'Heartbeat'을 수신하여 연결이 확인되었을 때
    if ((g_ext_link_mngr.cm_nmt_state == NMT_STATE_OPERATIONAL) && 
        (g_ext_link_mngr.is_ext_pack_connected == true)) {
        // PDO 전송 로직
        ScalingForPDOSendToExt(); // PDO Scaling
        _ExtLink_SendPDOs();      // PDO 전송 (1ms)
    }
	task_counter_10ms++;

	if (CMMode == SUIT_ASSIST_MODE) {
		assistModeLoopCnt++;	// 보조모드시 Count++
	} else { // in Standby State
		assistModeLoopCnt = 0;
	}
	/* ---- Extension Module ---- */

	wholeBodyCtrlLoopCnt++;
	prev_manu_mode= manu_mode;
	prev_FSM_state = FSMMngrObj.state_curr;
}

static void StateEnable_Ext(void)
{
	wholeBodyCtrlLoopCnt = 0;
	AllDevOffStates();
}

static void StateError_Run(void)
{

}

/* ----------------------- FUNCTION ----------------------- */
static void CalculateDuration(AS_TrainingInfo_t* currentSession)
{
//	static uint16_t countFor1sec = 0; // 1초 카운트를 위한 변수
//	static uint16_t countFor1sec_standbyState = 0; // 대기모드 1초 카운트를 위한 변수
 
//	countFor1sec++; // 1밀리초마다 증가
//	if (countFor1sec >= 1000) { // 1000밀리초가 되면 1초
//		countFor1sec = 0;       // countFor1sec 초기화
//		if (ble_init_status == true && IOIF_BLE_IsConnected() == true) {
//			SUIT_totalDuration++; // 앱 연결 시 총 사용시간 카운트
//			if (FSMMngrObj.state_curr == 0 || FSMMngrObj.state_curr == 1) {
//				countFor1sec_standbyState++;
//			}
//			else
//			{
//				SUIT_Duration = SUIT_totalDuration - countFor1sec_standbyState;
//			}
//		}
//	}

	if (currentSession->isSessionActive == true
			&& !((FSMMngrObj.state_curr >= 0 && FSMMngrObj.state_curr <= 2)
					|| (FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 51))) {
		isOperationDuring = true;
	} else {
		isOperationDuring = false;
	}

	/* 신규 계산 방식으로 변경 */
	countFor1sec++;
	if (countFor1sec >= 1000) { // 1000밀리초가 되면 1초
		countFor1sec = 0;       // countFor1sec 초기화

		/* 현재 세션 진행 중인 상태일 때만 duration 증가 */

		if(currentSession->isSessionActive == true)
		{
			/* 1. Session Duration Calculation */
			// 조건문 : 현재 세션이 진행 중이라면,
			// 세션 사용 시간 카운트 증가
			currentSession->session_duration++;

			/* 2. Mode Duration Calculation */
			// 조건문 : 현재 지금 모드가 해당 모드일 때만 카운트 증가
			if (BLECtrlObj.data.AssistMode < ASSIST_MODE_MAX_NUM)
			{
				currentSession->mode_duration[BLECtrlObj.data.AssistMode]++;
			}

			/* 3. Operation Duration Calculation */
			// 조건문 : 현재 지금 로봇이 대기 중인 상태라면, standby count 증가
			if ((FSMMngrObj.state_curr >= 0 && FSMMngrObj.state_curr <= 2)
					|| (FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 51)) {
				currentSession->countFor1sec_standbyState++;
			}
			else	// 로봇 운용시간은 총 동작 시간에서 대기 시간을 빼는 것으로 계산
			{
				currentSession->operation_duration = currentSession->mode_duration[BLECtrlObj.data.AssistMode] - currentSession->countFor1sec_standbyState;
			}
		}
		else		// session 진행 중인 상태가 아닐 경우,
		{
			currentSession->countFor1sec_standbyState = 0;			// 대기시간 count 초기화
			/* 모든 훈련 파라메터 초기화 */
			memset(&currentSession[current_session], 0, sizeof(currentSession[current_session]));
		}
	}
}

static void CheckStateNSetCompGain(void)
{
	static uint8_t H10_lastStateCurr = 0; // 0 means not set

	switch (BLECtrlObj.data.AssistMode) {
		case SMART_ASSIST_MODE : // 태블릿 앱 Nm 적용 필요
			if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
				if ((FSMMngrObj.state_curr <= 2 || FSMMngrObj.state_curr == 7
						|| (FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 32)
						|| (FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 47)) // 대기모드 or 안전대책 오디오출력 스테이트
					&& H10_lastStateCurr != 1) // 초기,대기,에러,오디오출력 스테이트 -> 보상 게인 리셋
				{
					Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0); // change code for add data of saturation and mode
					Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
					H10_lastStateCurr = 1;
					assistmode = SMART_ASSIST_MODE;
				} else if ((FSMMngrObj.state_curr == 5 || FSMMngrObj.state_curr == 6) &&  // 5,6 중력 보상 제외
						H10_lastStateCurr != 2)
				{
					Send_Comp_parameter(RH_SAGITAL, -RightHipSmartAssistGravityCompensation, RightHipSmartAssistVelocityCompensation, 1, 1);
					Send_Comp_parameter(LH_SAGITAL, -LeftHipSmartAssistGravityCompensation, LeftHipSmartAssistVelocityCompensation, 1, 1);
					H10_lastStateCurr = 2;
				} else if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr < 5) ||
						(FSMMngrObj.state_curr > 7 && FSMMngrObj.state_curr <= 17) ||
						(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
						FSMMngrObj.state_curr == 48) // 보행보조 오디오출력 스테이트
						&& H10_lastStateCurr != 3) {
					Send_Comp_parameter(RH_SAGITAL, RightHipSmartAssistGravityCompensation, RightHipSmartAssistVelocityCompensation, 1, 1);
					Send_Comp_parameter(LH_SAGITAL, LeftHipSmartAssistGravityCompensation, LeftHipSmartAssistVelocityCompensation, 1 , 1);
					H10_lastStateCurr = 3;
				}
			}
			break;

		case AQUA_MODE :
			if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
				if ((FSMMngrObj.state_curr <= 2 ||
						(FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 32) ||
						(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 47)) // 대기모드 or 안전대책 오디오출력 스테이트
					&& H10_lastStateCurr != 1) // 초기,대기,에러,오디오출력 스테이트 -> 보상 게인 리셋
				{
					Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
					Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
					H10_lastStateCurr = 1;
				} else if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
						(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
						FSMMngrObj.state_curr == 49) // 저항훈련 오디오출력 스테이트
						&& (H10_lastStateCurr != 2 || pre_assistStage != assistStage))
				{
					Send_Comp_parameter(RH_SAGITAL, assistStage * 0.1 * RightHipAquaGravityCompensation, assistStage * 0.1 * RightHipAquaVelocityCompensation, 5, 2);
					Send_Comp_parameter(LH_SAGITAL, assistStage * 0.1 * LeftHipAquaGravityCompensation, assistStage * 0.1 * LeftHipAquaVelocityCompensation, 5, 2);
					H10_lastStateCurr = 2;
					pre_assistStage = assistStage;
				}
			}
			break;

		case UNIVERSE_MODE :
			if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
				if ((FSMMngrObj.state_curr <= 2 ||
						(FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 32) ||
						(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 47)) // 대기모드 or 안전대책 오디오출력 스테이트
					&& H10_lastStateCurr != 1) // 초기,대기,에러,오디오출력 스테이트 -> 보상 게인 리셋
				{
					Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
					Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
					H10_lastStateCurr = 1;
					pre_assistStage = assistStage;
				} else if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
						(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
						FSMMngrObj.state_curr == 50) // 부하조절 오디오출력 스테이트
						&& (H10_lastStateCurr != 2 || pre_assistStage != assistStage))
				{
					Send_Comp_parameter(RH_SAGITAL, assistStage * 0.2 * RightHipUnivGravityCompensation, assistStage * 0.15 * RightHipUnivVelocityCompensation, 1.5, 3);
					Send_Comp_parameter(LH_SAGITAL, assistStage * 0.2 * LeftHipUnivGravityCompensation, assistStage * 0.15 * LeftHipUnivVelocityCompensation, 1.5, 3);
					H10_lastStateCurr = 2;
					pre_assistStage = assistStage;
				}
			}
			break;

		case JOINT_LIMIT_MODE :
			if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
				if ((FSMMngrObj.state_curr <= 2 ||
						(FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 32) ||
						(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 47)) // 대기모드 or 안전대책 오디오출력 스테이트
					&& H10_lastStateCurr != 1) // 초기,대기,에러,오디오출력 스테이트 -> 보상 게인 리셋
				{
					Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
					Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
					H10_lastStateCurr = 1;
				} else if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
						(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
						FSMMngrObj.state_curr == 51) // 관절운동제한 오디오출력 스테이트
						&& H10_lastStateCurr != 2)
				{
					Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
					Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
					H10_lastStateCurr = 2;

					Send_Limit_parameter(RH_SAGITAL, RightHipFlexionLimit, RightHipExtensionLimit, RightHipFlexionVelocityLimit, RightHipExtensionVelocityLimit, 4);
					Send_Limit_parameter(LH_SAGITAL, LeftHipFlexionLimit, LeftHipExtensionLimit, LeftHipFlexionVelocityLimit, LeftHipExtensionVelocityLimit, 4);
				}
			}
			break;

		default :
			if (H10_lastStateCurr != 4)
			{
				Send_Comp_parameter(RH_SAGITAL, 0, 0, 1, 0);
				Send_Comp_parameter(LH_SAGITAL, 0, 0, 1, 0);
				H10_lastStateCurr = 4;
			}
	}
}

static void CheckMDJointLimitRoutine(void)
{
	static uint8_t checkjointlimit = 0;

	if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
			(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
			FSMMngrObj.state_curr == 51)
		&& BLECtrlObj.data.AssistMode == JOINT_LIMIT_MODE &&
		checkjointlimit != 1)
	{
		AllDevSetJointLimitRoutine();
		checkjointlimit = 1;
	}
	else if (BLECtrlObj.data.AssistMode != JOINT_LIMIT_MODE && checkjointlimit != 2)
	{
		AllDevClearJointLimitRoutine();
		checkjointlimit = 2;
	}
	else if (((FSMMngrObj.state_curr >= 0 && FSMMngrObj.state_curr <= 2) ||
			(FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 32) ||
			(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 47))
			&& checkjointlimit != 2)
	{
		AllDevClearJointLimitRoutine();
		checkjointlimit = 2;
	}
}


static void SetMaxMinJointAngleVelocity(void)
{
	static bool setOneTime = false;

	if (trainingInfo[current_session].isSessionActive == true
			&& !(FSMMngrObj.state_curr >= 0 && FSMMngrObj.state_curr <= 2)
			&& BLECtrlObj.data.AssistMode == JOINT_LIMIT_MODE)
	{
		if (!setOneTime) {
			LeftHipMaxAngle = LH_PositionAct;
			LeftHipMinAngle = LH_PositionAct;
			LeftHipMaxVelocity = LH_EncoderVelocity;
			LeftHipMinVelocity = LH_EncoderVelocity;
			RightHipMaxAngle = RH_PositionAct;
			RightHipMinAngle = RH_PositionAct;
			RightHipMaxVelocity = RH_EncoderVelocity;
			RightHipMinVelocity = RH_EncoderVelocity;
			setOneTime = true;
		} else {
			LeftHipMaxAngle = fmax(LeftHipMaxAngle, LH_PositionAct);
			LeftHipMinAngle = fmin(LeftHipMinAngle, LH_PositionAct);
			LeftHipMaxVelocity = fmax(LeftHipMaxVelocity, LH_EncoderVelocity);
			LeftHipMinVelocity = fmin(LeftHipMinVelocity, LH_EncoderVelocity);
			RightHipMaxAngle = fmax(RightHipMaxAngle, RH_PositionAct);
			RightHipMinAngle = fmin(RightHipMinAngle, RH_PositionAct);
			RightHipMaxVelocity = fmax(RightHipMaxVelocity, RH_EncoderVelocity);
			RightHipMinVelocity = fmin(RightHipMinVelocity, RH_EncoderVelocity);
		}
	} else if (setOneTime) {
		setOneTime = false;
	}
}

static void CheckSAMIMU(SAM_Motion_Struct* RH_MotionData, SAM_Motion_Struct* LH_MotionData)
{
	// 상태 유지 변수들
	static int count = 0;
	static float RH_initialThighSagittalAngle, RH_initialGyroZGlobal;
	static float LH_initialThighSagittalAngle, LH_initialGyroZGlobal;
	static bool RH_angleChanged = false;
	static bool LH_angleChanged = false;
	static bool RH_gyroZChanged = false;
	static bool LH_gyroZChanged = false;

	if (count == 0) {
		// 초기값 설정
		RH_initialThighSagittalAngle = RH_MotionData->ThighSagittalAngle;
		RH_initialGyroZGlobal = RH_MotionData->gyroZGlobal;
		LH_initialThighSagittalAngle = LH_MotionData->ThighSagittalAngle;
		LH_initialGyroZGlobal = LH_MotionData->gyroZGlobal;

		RH_angleChanged = false;
		LH_angleChanged = false;
		RH_gyroZChanged = false;
		LH_gyroZChanged = false;
	}

	// 각도 변화 확인
	if (fabsf(RH_MotionData->ThighSagittalAngle - RH_initialThighSagittalAngle) > CHECK_IMU_THRESHOLD) {
		RH_angleChanged = true;
	}

	if (fabsf(LH_MotionData->ThighSagittalAngle - LH_initialThighSagittalAngle) > CHECK_IMU_THRESHOLD) {
		LH_angleChanged = true;
	}

	// 자이로 Z 변화 확인
	if (RH_MotionData->gyroZGlobal != RH_initialGyroZGlobal) {
		RH_gyroZChanged = true;
	}

	if (LH_MotionData->gyroZGlobal != LH_initialGyroZGlobal) {
		LH_gyroZChanged = true;
	}

	count++;
	if (count >= CHECK_IMU_DURATION_MS) {
		// 3초 후 최종 결과 확인
		if (RH_angleChanged && !RH_gyroZChanged && LH_angleChanged && !LH_gyroZChanged) {
			BFlag[29] = 1; // 양쪽 다 문제가 있는 경우
		} else if (RH_angleChanged && !RH_gyroZChanged) {
			BFlag[28] = 1; // 오른쪽만 문제가 있는 경우
		} else if (LH_angleChanged && !LH_gyroZChanged) {
			BFlag[27] = 1; // 왼쪽만 문제가 있는 경우
		}
		count = 0; // 3초 후 초기화
	}
}

static void CheckSAMFDCAN(void)
{
	static bool oneTimeSendCANCommuCheck = false;
	static uint16_t send_fdcan_cnt = 0;
	static uint16_t receive_fdcan_cnt = 0;
	static bool send_fdcan_flag = FALSE;

	if (!oneTimeSendCANCommuCheck && !send_fdcan_flag && send_fdcan_cnt == SEND_FDCAN_CNT) {
		can_test[NODE_ID_RH_SAG] = 0;
		can_test[NODE_ID_LH_SAG] = 0;

		Check_FD_CAN_COMMU_TX(NODE_ID_RH_SAG);
		Check_FD_CAN_COMMU_TX(NODE_ID_LH_SAG);
		receive_fdcan_cnt = 0;
		send_fdcan_flag = TRUE;
	}

	/* H10에서만 유효 */
	if (!oneTimeSendCANCommuCheck && send_fdcan_flag && receive_fdcan_cnt == RECEIVE_FDCAN_CNT) {
		if(can_test[NODE_ID_RH_SAG] == 1 && can_test[NODE_ID_LH_SAG] == 1) {
		} else if(can_test[NODE_ID_RH_SAG] == 1 && can_test[NODE_ID_LH_SAG] == 0){
			BFlag[24] = 1;
		} else if(can_test[NODE_ID_RH_SAG] == 0 && can_test[NODE_ID_LH_SAG] == 1){
			BFlag[25] = 1;
		} else if(can_test[NODE_ID_RH_SAG] == 0 && can_test[NODE_ID_LH_SAG] == 0){
			BFlag[26] = 1;
		}
		oneTimeSendCANCommuCheck = true;
		send_fdcan_cnt = 0;
		send_fdcan_flag = FALSE;
	}

	send_fdcan_cnt++;
	receive_fdcan_cnt++;
}

static void CheckCanOperate(void)
{
	static uint32_t power_cnt = 0;

	if(isFaultOccurred == true) {
		canOperate = 0;	// error state
	}
	else if (FSMMngrObj.state_curr >= 0 && FSMMngrObj.state_curr <= 17) { // 보행 가능 상태
		canOperate = 1; // 정상
	}

	if((MD_state != SAVE_PROPERTIES && MD_state != SAVE_PROPERTIES_NeutralPosture) && power_cnt > 500){
		if (FSMMngrObj.state_curr == 32) {
			IOIF_SYS_PwrOff(); //power off
		}
	}

	power_cnt++;
}

static void InitSAMMotionData(SAM_Motion_Struct* SAM_MotionData)
{
	for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
		uint8_t tUsage = RS_File.vec[DevIdx].usage;

		if (tUsage == 1) {
			memset(SAM_MotionData, 0, sizeof(SAM_Motion_Struct));
		}
	}
}

static void UpdatePDOData(void)
{
	for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
		uint8_t tUsage = RS_File.vec[DevIdx].usage;

		if (tUsage == 1) {
			switch (DevIdx) {
			case RH_SAGITAL:
				DecodingPDOData(&RH_Sagittal, DevIdx);
				break;
			case LH_SAGITAL:
				DecodingPDOData(&LH_Sagittal, DevIdx);
				break;
			case RK_SAGITAL:
				DecodingPDOData(&RK_Sagittal, DevIdx);
				break;
			case LK_SAGITAL:
				DecodingPDOData(&LK_Sagittal, DevIdx);
				break;
			default:
				break;
			}
		}
	}
}

static void UpdateMotionAnalysis_H10(void)
{
	memset(&MotionFlags, 0, sizeof(MotionAnalysisConditions));

	// STS Mode - 서기 시작 인식
	if ((L_ThighSagittalAngle - L_STS_stand_start_thigh_angle < -5.0f) && (R_ThighSagittalAngle - R_STS_stand_start_thigh_angle < -5.0f)){
		MotionFlags.ifStandingStart = true;
	}

	/* Left Hip */
	/* 각 도 */
	if (L_ThighSagittalAngle >= 40.0f) { // 왼쪽 허벅지 각도가 매우 크면 (40도 이상)
		MotionFlags.ifLHHyperFlexion2 = true;
	}

	if (L_ThighSagittalAngle >= 28.0f && L_ThighSagittalAngle < 40.0f) { // 왼쪽 허벅지 각도가 28~40도 사이라면
		MotionFlags.ifLHHyperFlexion = true;
	}

	if (L_ThighSagittalAngle >= 15.0f && L_ThighSagittalAngle < 28.0f) { // 왼쪽 허벅지 각도가 15~28도 사이라면
		MotionFlags.ifLHFlexion = true;
	}

	if (L_ThighSagittalAngle >= -7.0f && L_ThighSagittalAngle < 15.0f) { // 왼쪽 허벅지 각도가 -7~15도사이라면
		MotionFlags.ifLHNeutral = true;
	}

	if (L_ThighSagittalAngle >= -7.0f && L_ThighSagittalAngle < 5.0f) { // 왼쪽 허벅지 각도가 -7~5도사이라면
		MotionFlags.ifLHNeutral2 = true;
	}

	if (L_ThighSagittalAngle < -7.0f) { // 왼쪽 허벅지 각도가 -7도 보다 작으면
		MotionFlags.ifLHExtension = true;
	}
	
	/* 각 속 도 */
	// ThighSagittalVelocity --> incVelDiff (엔코더 산술미분 각속도)
	if (LH_EncoderVelocity > 50.0f) { // 왼쪽 고관절 각속도가 50 이상이면
		MotionFlags.ifLHFlexionIsFast = true;
	}

	if (LH_EncoderVelocity > 30.0f) { // 왼쪽 고관절 엔코더 각속도가 30 이상이면
		MotionFlags.ifLHFlexionIsOn = true;
	}

	if (LH_EncoderVelocity > 8.0f) { // 왼쪽 고관절 엔코더 각속도가 8 이상이면
		MotionFlags.ifLHFlexionIsOn2 = true;
	}

	if (fabs(LH_EncoderVelocity) < 30.0f) { // 왼쪽 고관절 엔코더 각속도 크기가 30 이하이면
		MotionFlags.ifLHIsStationary = true;
	}

	if (LH_EncoderVelocity < -5.0f) { // 왼쪽 고관절 엔코더 각속도가 -5 이하이면
		MotionFlags.ifLHExtensionIsOn = true;
	}

	if (LH_EncoderVelocity < -5.0f && LH_EncoderVelocity >= -30.0f) { // 왼쪽 고관절 각속도가 -30 이상 -5 이하이면
		MotionFlags.ifLHExtensionIsSlow = true;
	}

	if (LH_EncoderVelocity < -30.0f) { // 왼쪽 고관절 각속도가 -30 이하이면
		MotionFlags.ifLHExtensionIsFast = true;
	}

	/* Right Hip */
	/* 각 도 */
	if (R_ThighSagittalAngle >= 40.0f) { // 오른쪽 허벅지 각도가 매우 크면 (40도 이상)
		MotionFlags.ifRHHyperFlexion2 = true;
	}

	if (R_ThighSagittalAngle >= 28.0f && R_ThighSagittalAngle < 40.0f) { // 오른쪽 허벅지 각도가 15~40도 사이라면
		MotionFlags.ifRHHyperFlexion = true;
	}

	if (R_ThighSagittalAngle >= 15.0f && R_ThighSagittalAngle < 28.0f) { // 오른쪽 허벅지 각도가 15~28도 사이라면
		MotionFlags.ifRHFlexion = true;
	}

	if (R_ThighSagittalAngle >= -7.0f && R_ThighSagittalAngle < 15.0f) { // 오른쪽 허벅지 각도가 -7~15도사이라면
		MotionFlags.ifRHNeutral = true;
	}

	if (R_ThighSagittalAngle >= -7.0f && R_ThighSagittalAngle < 5.0f) { // 오른쪽 허벅지 각도가 -7~5도사이라면
		MotionFlags.ifRHNeutral2 = true;
	}

	if (R_ThighSagittalAngle < -7.0f) { // 오른쪽 허벅지 각도가 -7도보다 작으면
		MotionFlags.ifRHExtension = true;
	}

	/* 각 속 도 */
	if (fabs(RH_EncoderVelocity) < 30.0f) { // 오른쪽 고관절 엔코더 각속도가 30 이하이면
		MotionFlags.ifRHIsStationary = true;
	}

	if (RH_EncoderVelocity > 30.0f) { // 오른쪽 고관절 각속도가 30 이상이면
		MotionFlags.ifRHFlexionIsOn = true;
	}

	if (RH_EncoderVelocity > 8.0f) { // 오른쪽 고관절 각속도가 30 이상이면
		MotionFlags.ifRHFlexionIsOn2 = true;
	}

	if (RH_EncoderVelocity > 50.0f) { // 오른쪽 고관절 각속도가 50 이상이면
		MotionFlags.ifRHFlexionIsFast = true;
	}

	if (RH_EncoderVelocity < -5.0f) { // 오른쪽 고관절 각속도가 -5 이하이면
		MotionFlags.ifRHExtensionIsOn = true;
	}

	if (RH_EncoderVelocity < -5.0f && RH_EncoderVelocity >= -30.0f) { // 오른쪽 고관절 각속도가 -30 이상 -5 이하이면
		MotionFlags.ifRHExtensionIsSlow = true;
	}

	if (RH_EncoderVelocity < -30.0f) { // 오른쪽 고관절 각속도가 -30 이하이면
		MotionFlags.ifRHExtensionIsFast = true;
	}

	/* Right&Left Hip */
	/* 각 도 차 */
	if (fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) <= 5) { // 좌우 허벅지 각도차이 크기가 5 이하라면
		MotionFlags.ifAbsHipAngleGapIsXS = true;
	}

	if ((fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) > 5) && (fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) <= 15)) { // 좌우 허벅지 각도차이 크기가 5~15 라면
		MotionFlags.ifAbsHipAngleGapIsS = true;
	}

	if (L_ThighSagittalAngle - R_ThighSagittalAngle > 50) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 50도 이상 크면
		MotionFlags.ifLeftHipAngleGapIsXXXL = true;
	}

	if ((L_ThighSagittalAngle - R_ThighSagittalAngle > 35) && (L_ThighSagittalAngle - R_ThighSagittalAngle <= 50)) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 35~50도 크면
		MotionFlags.ifLeftHipAngleGapIsXXL = true;
	}

	if ((L_ThighSagittalAngle - R_ThighSagittalAngle > 20) && (L_ThighSagittalAngle - R_ThighSagittalAngle <= 35)) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 20~35도 크면
		MotionFlags.ifLeftHipAngleGapIsXL = true;
	}

	if ((L_ThighSagittalAngle - R_ThighSagittalAngle > 15) && (L_ThighSagittalAngle - R_ThighSagittalAngle <= 20)) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 15~20도 크면
		MotionFlags.ifLeftHipAngleGapIsL = true;
	}

	if ((L_ThighSagittalAngle - R_ThighSagittalAngle > 5) && (L_ThighSagittalAngle - R_ThighSagittalAngle <= 15)) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 5~15도 크면
		MotionFlags.ifLeftHipAngleGapIsS = true;
	}

	if (L_ThighSagittalAngle - R_ThighSagittalAngle > 0) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 0도 이상 크면
		MotionFlags.ifLeftHipAngleGapIsOn = true;
	}

	if (R_ThighSagittalAngle - L_ThighSagittalAngle > 50) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 50도 이상 크면
		MotionFlags.ifRightHipAngleGapIsXXXL = true;
	}

	if ((R_ThighSagittalAngle - L_ThighSagittalAngle > 35) && (R_ThighSagittalAngle - L_ThighSagittalAngle <= 50)) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 35~50도 크면
		MotionFlags.ifRightHipAngleGapIsXXL = true;
	}

	if ((R_ThighSagittalAngle - L_ThighSagittalAngle > 20) && (R_ThighSagittalAngle - L_ThighSagittalAngle <= 35)) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 20~35도 크면
		MotionFlags.ifRightHipAngleGapIsXL = true;
	}

	if ((R_ThighSagittalAngle - L_ThighSagittalAngle > 15) && (R_ThighSagittalAngle - L_ThighSagittalAngle <= 20)) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 15~20도 크면
		MotionFlags.ifRightHipAngleGapIsL = true;
	}

	if ((R_ThighSagittalAngle - L_ThighSagittalAngle > 5) && (R_ThighSagittalAngle - L_ThighSagittalAngle <= 15)) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 5~15도 크면
		MotionFlags.ifRightHipAngleGapIsS = true;
	}

	if (R_ThighSagittalAngle - L_ThighSagittalAngle > 0) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 5도 이상 크면
		MotionFlags.ifRightHipAngleGapIsOn = true;
	}
	/* 각 도 합 */
	if (L_ThighSagittalAngle + R_ThighSagittalAngle > 47) { // 왼쪽 허벅지 각도와 오른쪽 허벅지 각도의 합이 47 이상 크면
		MotionFlags.ifThighAngleSumIsXXL = true;
	}

	if ((L_ThighSagittalAngle + R_ThighSagittalAngle > 30) && (L_ThighSagittalAngle + R_ThighSagittalAngle <= 47)) { // 왼쪽 허벅지 각도와 오른쪽 허벅지 각도 합이 30~47도
		MotionFlags.ifThighAngleSumIsXL = true;
	}

	if ((L_ThighSagittalAngle + R_ThighSagittalAngle > 15) && (L_ThighSagittalAngle + R_ThighSagittalAngle <= 30)) { // 왼쪽 허벅지 각도와 오른쪽 허벅지 각도 합이 15~30도
		MotionFlags.ifThighAngleSumIsL = true;
	}

	/* MD global 가속도 */
	if ((RH_accXGlobal[0] < 1.0f) && (RH_accXGlobal[0] > -1.0f)){ // 오른쪽 x 가속도가 -1 ~ 1
		MotionFlags.ifRightAccXIsS = true;
	}

	if ((RH_accYGlobal[0] > 9.0f) && (RH_accYGlobal[0] < 10.5f)){ // 오른쪽 Y 가속도가 9 ~ 10.5
		MotionFlags.ifRightAccYIsS = true;
	}

	if (RH_accYGlobal[0] > 8.0f){ // 오른쪽 Y 가속도가 8 보다 클때
		MotionFlags.ifRightAccYIsXS = true;
	}

	if ((LH_accXGlobal[0] < 1.0f) && (LH_accXGlobal[0] > -1.0f)){ // 왼쪽 x 가속도가 -1 ~ 1
		MotionFlags.ifLeftAccXIsS = true;
	}

	if ((LH_accYGlobal[0] > 9.0f) && (LH_accYGlobal[0] < 10.5f)){ // 왼쪽 Y 가속도가 9 ~ 10.5
		MotionFlags.ifLeftAccYIsS = true;
	}

	if (LH_accYGlobal[0] > 8.0f){ // 왼쪽 Y 가속도가 8 보다 클때
		MotionFlags.ifLeftAccYIsXS = true;
	}

	/* 앉기 서기 보조 모드 */
	if (PostureHold == 1) { // 프로앱 자세유지버튼 on 상태
		MotionFlags.ifProApp_PostureHoldOn = true;
	}
	if (PostureHold == 0) { // 프로앱 자세유지버튼 off 상태(default)
		MotionFlags.ifProApp_PostureHoldOff = true;
	}
	if (SitReadyButton == 1) { // 프로앱 앉기 준비 버튼 on
		MotionFlags.ifProApp_SitReady = true;
		SitReadyButton = 0;
	}
	if (SitReadyButton == 2) { // 프로앱 앉기 준비 취소 버튼 on
		MotionFlags.ifProApp_SitReadyCancel = true;
		SitReadyButton = 0;
	}
	if (StartSittingButton == 1) { // 프로앱 앉기 시작 버튼 on
		MotionFlags.ifProApp_SitStart = true;
		StartSittingButton = 0;
	}
	if (StandReadyButton == 1) { // 프로앱 서기 준비 버튼 on
		MotionFlags.ifProApp_StandReady = true;
		StandReadyButton = 0;
	}
	if (StandReadyButton == 2) { // 프로앱 서기 준비 취소 버튼 on
		MotionFlags.ifProApp_StandReadyCancel = true;
		StandReadyButton = 0;
	}
	if (StartStandingButton == 1) { // 프로앱 서기 시작 버튼 on
		MotionFlags.ifProApp_StandStart = true;
		StartStandingButton = 0;
	}

	// p_Sitting(70), p_p_Standing_Start(73), p_Standing(74), p_Standing_Hold(75)
	if (FSMMngrObj.state_curr == 70 || FSMMngrObj.state_curr == 73 || FSMMngrObj.state_curr == 74 ||
		FSMMngrObj.state_curr == 75) {
		if (STS_P_Vector_Duration_Completed_RH == 1 && STS_P_Vector_Duration_Completed_LH == 1) {
			MotionFlags.ifVectorEnd = true; // 지정된 p-vector 길이만큼 시간 흘렀을 때
		}
	} else {
		STS_P_Vector_Duration_Completed_RH = 0;
		STS_P_Vector_Duration_Completed_LH = 0;
	}
	
	/* 기준값을 앱에서 조정해야 하는 경우 변수 어떻게? */
	if (L_ThighSagittalAngle >= 30.0f) { // 왼쪽 허벅지 각도 앉기 시각 각도
		MotionFlags.ifLHHyperFlexion3 = true;
	}
	if (R_ThighSagittalAngle >= 30.0f) { // 오른쪽 허벅지 각도 앉기 시각 각도
		MotionFlags.ifRHHyperFlexion3 = true;
	}
	if (L_ThighSagittalAngle >= 50.0f) { // 왼쪽 허벅지 각도 앉기 완료 각도1
		MotionFlags.ifLHHyperFlexion4 = true;
	}
	if (R_ThighSagittalAngle >= 50.0f) { // 오른쪽 허벅지 각도 앉기 완료 각도1
		MotionFlags.ifRHHyperFlexion4 = true;
	}
	if (L_ThighSagittalAngle >= 70.0f) { // 왼쪽 허벅지 각도 앉기 완료 각도2
		MotionFlags.ifLHHyperFlexion5 = true;
	}
	if (R_ThighSagittalAngle >= 70.0f) { // 오른쪽 허벅지 각도 앉기 완료 각도2
		MotionFlags.ifRHHyperFlexion5 = true;
	}

	/* 특정 state들 동안에 가속도값 누적합하기 */
	if ((FSMMngrObj.state_curr == 11 && FSMStatePrev == 11) || 
		(FSMMngrObj.state_curr == 9 && FSMStatePrev == 9)   ||
		(FSMMngrObj.state_curr == 13 && FSMStatePrev == 13) ||
		(FSMMngrObj.state_curr == 16 && FSMStatePrev == 16) ||
		(FSMMngrObj.state_curr == 33 && FSMStatePrev == 33) ||
		(FSMMngrObj.state_curr == 35 && FSMStatePrev == 35) ||
		(FSMMngrObj.state_curr == 37 && FSMStatePrev == 37) ||
		(FSMMngrObj.state_curr == 39 && FSMStatePrev == 39)) {
		
		sumY_L += LH_accYGlobal[0] - ACC_GRAVITY_CONSTANT;
		sumY_R += RH_accYGlobal[0] - ACC_GRAVITY_CONSTANT;
		sumsumY_L += sumY_L;
		sumsumY_R += sumY_R;
	} else {
        sumY_L = 0.0f;
        sumY_R = 0.0f;
        sumsumY_L = 0.0f;
        sumsumY_R = 0.0f;
    }
 
    /* Y가속도 누적합 ISI 신규 */
    if ((sumY_L > 300.0f) && (sumY_R > 300.0f)) {
        MotionFlags.ifsumYPlus = true;
    }
    if ((sumY_L < -120.0f) && (sumY_R < -120.0f)) {
        MotionFlags.ifsumYMinus = true;
    }
    if (sumsumY_L + sumsumY_R > 70000.0f) {
        MotionFlags.ifsumsumYPlus = true;
    }
    if ((sumsumY_L + sumsumY_R < -80000.0f)) {
        MotionFlags.ifsumsumYMinus = true;
    }

	FSMStatePrev = FSMMngrObj.state_curr;
}


static void UpdateMotionAnalysis_K10(void)
{
	memset(&MotionFlags, 0, sizeof(MotionAnalysisConditions));

	/* Left Hip */
	/* 각 도 */
	if (L_ThighSagittalAngle >= 40.0f) { // 왼쪽 허벅지 각도가 매우 크면 (40도 이상)
		MotionFlags.ifLHHyperFlexion2 = true;
	}

	if (L_ThighSagittalAngle < 40.0f && L_ThighSagittalAngle >= 20.0f) { // 왼쪽 허벅지 각도가 20~40도 사이라면
		MotionFlags.ifLHHyperFlexion = true;
	}

	if (L_ThighSagittalAngle < 20.0f && L_ThighSagittalAngle >= 10.0f) { // 왼쪽 허벅지 각도가 10~20도 사이라면
		MotionFlags.ifLHFlexion = true;
	}

	if (L_ThighSagittalAngle < 10.0f && L_ThighSagittalAngle >= 0.0f) { // 왼쪽 허벅지 각도가 0~10도사이라면
		MotionFlags.ifLHNeutral = true;
	}

	if (L_ThighSagittalAngle < 0.0f) { // 왼쪽 허벅지 각도가 0도 보다 작으면
		MotionFlags.ifLHExtension = true;
	}

	/* 각 속 도 */
	if (L_ThighSagittalVelocity > 50.0f) { // 왼쪽 고관절 각속도가 50 이상이면
		MotionFlags.ifLHFlexionIsFast = true;
	}

	if (L_ThighSagittalVelocity > 30.0f) { // 왼쪽 고관절 각속도가 30 이상이면
		MotionFlags.ifLHFlexionIsOn = true;
	}

	if (L_ThighSagittalVelocity > 8.0f) { // 왼쪽 고관절 각속도가 8 이상이면
		MotionFlags.ifLHFlexionIsOn2 = true;
	}

	if (fabs(L_ThighSagittalVelocity) < 30.0f) { // 왼쪽 고관절 엔코더 각속도 크기가 30 이하이면
		MotionFlags.ifLHIsStationary = true;
	}

	if (L_ThighSagittalVelocity < -5.0f) { // 왼쪽 고관절 각속도가 -5 이하이면
		MotionFlags.ifLHExtensionIsOn = true;
	}

	if (L_ThighSagittalVelocity < -30.0f) { // 왼쪽 고관절 각속도가 -30 이하이면
		MotionFlags.ifLHExtensionIsFast = true;
	}

	/* Right Hip */
	/* 각 도 */
	if (R_ThighSagittalAngle >= 40.0f) { // 오른쪽 허벅지 각도가 매우 크면 (40도 이상)
		MotionFlags.ifRHHyperFlexion2 = true;
	}

	if (R_ThighSagittalAngle < 40.0f && R_ThighSagittalAngle >= 20.0f) { // 오른쪽 허벅지 각도가 20~40도 사이라면
		MotionFlags.ifRHHyperFlexion = true;
	}

	if (R_ThighSagittalAngle < 20.0f && R_ThighSagittalAngle >= 10.0f) { // 오른쪽 허벅지 각도가 10~20도 사이라면
		MotionFlags.ifRHFlexion = true;
	}

	if (R_ThighSagittalAngle < 10.0f && R_ThighSagittalAngle >= 0.0f) { // 오른쪽 허벅지 각도가 0~15도사이라면
		MotionFlags.ifRHNeutral = true;
	}

	if (R_ThighSagittalAngle < 0.0f) { // 오른쪽 허벅지 각도가 0도보다 작으면
		MotionFlags.ifRHExtension = true;
	}

	/* 각 속 도 */
	if (R_ThighSagittalVelocity > 50.0f) { // 오른쪽 고관절 각속도가 50 이상이면
		MotionFlags.ifRHFlexionIsFast = true;
	}

	if (R_ThighSagittalVelocity > 30.0f) { // 오른쪽 고관절 각속도가 30 이상이면
		MotionFlags.ifRHFlexionIsOn = true;
	}

	if (R_ThighSagittalVelocity > 8.0f) { // 오른쪽 고관절 각속도가 30 이상이면
		MotionFlags.ifRHFlexionIsOn2 = true;
	}

	if (fabs(R_ThighSagittalVelocity) < 30.0f) { // 오른쪽 고관절 엔코더 각속도가 30 이하이면
		MotionFlags.ifRHIsStationary = true;
	}

	if (R_ThighSagittalVelocity < -5.0f) { // 오른쪽 고관절 각속도가 -5 이하이면
		MotionFlags.ifRHExtensionIsOn = true;
	}

	if (R_ThighSagittalVelocity < -30.0f) { // 오른쪽 고관절 각속도가 -30 이하이면
		MotionFlags.ifRHExtensionIsFast = true;
	}

	/* Right&Left Hip */
	/* 각 도 차 */
	if (fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) <= 5) { // 좌우 허벅지 각도차이 크기가 5 이하라면
		MotionFlags.ifAbsHipAngleGapIsXS = true;
	}
	
	if ((fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) <= 15) && (fabs(L_ThighSagittalAngle - R_ThighSagittalAngle) > 5)) { // 좌우 허벅지 각도차이 크기가 5~15 라면
		MotionFlags.ifAbsHipAngleGapIsS = true;
	}

	if (L_ThighSagittalAngle - R_ThighSagittalAngle > 15) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 15도 이상 크면
		MotionFlags.ifLeftHipAngleGapIsL = true;
	}

	if ((L_ThighSagittalAngle - R_ThighSagittalAngle > 5) && (L_ThighSagittalAngle - R_ThighSagittalAngle <= 15)) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 5~15도 크면
		MotionFlags.ifLeftHipAngleGapIsS = true;
	}

	if (L_ThighSagittalAngle - R_ThighSagittalAngle > 0) { // 왼쪽 허벅지 각도가 오른쪽 허벅지 각도보다 0도 이상 크면
		MotionFlags.ifLeftHipAngleGapIsOn = true;
	}

	if (R_ThighSagittalAngle - L_ThighSagittalAngle > 15) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 15도 이상 크면
		MotionFlags.ifRightHipAngleGapIsL = true;
	}

	if ((R_ThighSagittalAngle - L_ThighSagittalAngle > 5) && (R_ThighSagittalAngle - L_ThighSagittalAngle <= 15)) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 5~15도 크면
		MotionFlags.ifRightHipAngleGapIsS = true;
	}
	
	if (R_ThighSagittalAngle - L_ThighSagittalAngle > 0) { // 오른쪽 허벅지 각도가 왼쪽 허벅지 각도보다 5도 이상 크면
		MotionFlags.ifRightHipAngleGapIsOn = true;
	}

	/* 각 도 합 */
	if (L_ThighSagittalAngle + R_ThighSagittalAngle > 16) { // 왼쪽 허벅지 각도와 오른쪽 허벅지 각도의 합이 16도 이상 크면
		MotionFlags.ifThighAngleSumIsL = true;
	}

	/* Left Knee */
	/* 각 도 */
	// Shank(종아리) 절대각도가 아닌 무릎 엔코더 각도로 적용합니다.
	if (LK_PositionAct >= 40.0f) { // 왼쪽 무릎 엔코더 각도가 매우 크면 (40도 이상)
		MotionFlags.ifLKHyperFlexion2 = true;
	}

	if (LK_PositionAct < 40.0f && LK_PositionAct >= 20.0f) { // 왼쪽 무릎 엔코더 각도가 20~40도 사이라면
		MotionFlags.ifLKHyperFlexion = true;
	}

	if (LK_PositionAct < 20.0f && LK_PositionAct >= 5.0f) { // 왼쪽 무릎 엔코더 각도가 5~20도 사이라면
		MotionFlags.ifLKFlexion = true;
	}

	if (LK_PositionAct < 5.0f) { // 왼쪽 무릎 엔코더 각도가 -7~5도사이라면
		MotionFlags.ifLKNeutral = true;
	}

	/* 각 속 도 */
	// 무릎 엔코더 산술미분 각속도로 적용합니다.
	if (LK_EncoderVelocity > 20.0f) { // 왼쪽 무릎 엔코더 각속도가 20 이상이면
		MotionFlags.ifLKFlexionIsFast = true;
	}

	if (fabs(LK_EncoderVelocity) <= 20.0f) { // 왼쪽 무릎 엔코더 엔코더 각속도 크기가 20 이하이면
		MotionFlags.ifLKIsStationary = true;
	}

	if (LK_EncoderVelocity < -5.0f) { // 왼쪽 무릎 엔코더 각속도가 -5 이하이면
		MotionFlags.ifLKExtensionIsOn = true;
	}

	if (LK_EncoderVelocity < -20.0f) { // 왼쪽 무릎 엔코더 각속도가 -20 이하이면
		MotionFlags.ifLKExtensionIsFast = true;
	}

	/* Right Shank */
	/* 각 도 */
	// Shank(종아리) 절대각도가 아닌 무릎 엔코더 각도로 적용합니다.
	if (RK_PositionAct >= 40.0f) { // 오른쪽 허벅지 각도가 매우 크면 (40도 이상)
		MotionFlags.ifRKHyperFlexion2 = true;
	}

	if (RK_PositionAct < 40.0f && RK_PositionAct >= 20.0f) { // 오른쪽 허벅지 각도가 20~40도 사이라면
		MotionFlags.ifRKHyperFlexion = true;
	}

	if (RK_PositionAct < 20.0f && RK_PositionAct >= 5.0f) { // 오른쪽 허벅지 각도가 5~20도 사이라면
		MotionFlags.ifRKFlexion = true;
	}

	if (RK_PositionAct < 5.0f) { // 오른쪽 허벅지 각도가 -7~5도사이라면
		MotionFlags.ifRKNeutral = true;
	}

	/* 각 속 도 */
	if (RK_EncoderVelocity > 20.0f) { // 오른쪽 고관절 각속도가 50 이상이면
		MotionFlags.ifRKFlexionIsFast = true;
	}

	if (fabs(RK_EncoderVelocity) <= 20.0f) { // 오른쪽 고관절 엔코더 각속도가 30 이하이면
		MotionFlags.ifRKIsStationary = true;
	}

	if (RK_EncoderVelocity < -5.0f) { // 오른쪽 고관절 각속도가 -5 이하이면
		MotionFlags.ifRKExtensionIsOn = true;
	}

	if (RK_EncoderVelocity < -20.0f) { // 오른쪽 고관절 각속도가 -30 이하이면
		MotionFlags.ifRKExtensionIsFast = true;
	}

	/* IMU 가속도 */
	if (fabs(LK_accXGlobal[0]) < 2.0f) {
		MotionFlags.ifLeftAccXIsS = true;
	}

	if (fabs(RK_accXGlobal[0]) < 2.0f) {
		MotionFlags.ifRightAccXIsS = true;
	}

	if (fabs(LK_accYGlobal[0]) < 8.5f) {
		MotionFlags.ifLeftAccYIsS = true;
	}

	if (fabs(RK_accYGlobal[0]) < 8.5f) {
		MotionFlags.ifRightAccYIsS = true;
	}
}

static void GaitAnalysis(void)
{
	static float meanHipamplitude = 0;

	if (FSMMngrObj.state_curr == 10 || FSMMngrObj.state_curr == 15 ||
			FSMMngrObj.state_curr == 36 || FSMMngrObj.state_curr == 40 ||
			FSMMngrObj.state_curr == 45) {
		if (LeftGaitCountOn == 0) {
			gaitCount[0] = gaitCount[0] + 0.5;
			gaitLapCount = gaitLapCount + 0.5;
			gaitCountRight[0]++;
			LeftGaitCountOn = 1;
			if (halfStep == 0) {
				halfStep = 1;
			}
			if (FirstStep == 0 && GaitStop == 1) {
				FirstStep = 1;
			}
			if (GaitStop == 1) {
				GaitStop = 0;
			}
		}
	}

	if (FSMMngrObj.state_curr == 11 || FSMMngrObj.state_curr == 16 ||
			FSMMngrObj.state_curr == 35 || FSMMngrObj.state_curr == 39) {
		LeftGaitCountOn = 0;
	}

	if (FSMMngrObj.state_curr == 8 || FSMMngrObj.state_curr == 12 ||
			FSMMngrObj.state_curr == 34 || FSMMngrObj.state_curr == 38 ||
			FSMMngrObj.state_curr == 44) {
		if (RightGaitCountOn == 0) {
			gaitCountLeft[0]++;
			RightGaitCountOn = 1;
			if (halfStep == 1) {
				halfStep = 0;
				gaitCount[0] = gaitCount[0] + 0.5;
				gaitLapCount = gaitLapCount + 0.5;

				if (FirstStep == 1) {
					FirstStep = 0;
				}
			}
		}
	}

	if (FSMMngrObj.state_curr == 9 || FSMMngrObj.state_curr == 13 ||
			FSMMngrObj.state_curr == 33 || FSMMngrObj.state_curr == 37) {
		RightGaitCountOn = 0;
	}

	if (FSMMngrObj.state_curr == 4 || FSMMngrObj.state_curr == 7) {
		GaitStop = 1;
		gaitLapCount = 0;
		RHamplitude = 0;
		LHamplitude = 0;
	}

	if (gaitLapCount == 1.5) {
		RHamplitude = GaitRHThighSagittalAngle_max - GaitRHThighSagittalAngle_min;
		LHamplitude = GaitLHThighSagittalAngle_max - GaitLHThighSagittalAngle_min;
	}
	if (gaitLapCount > 1.5) {
		RHamplitude = 0.9 * RHamplitude + 0.1 * (GaitRHThighSagittalAngle_max - GaitRHThighSagittalAngle_min);
		LHamplitude = 0.9 * LHamplitude + 0.1 * (GaitLHThighSagittalAngle_max - GaitLHThighSagittalAngle_min);
	}

	meanHipamplitude = (RHamplitude + LHamplitude) / 2.0f;
	if (FSMMngrObj.state_curr != FSM_prev_state) {
		Asymmetry[0] = Asymmetry[1];
	} else if ((meanHipamplitude != 0.0f)) {
		Asymmetry[0] = 100 * (RHamplitude - LHamplitude) / meanHipamplitude;
	} else {
		Asymmetry[0] = 0;
	}
	Asymmetry[0] = fmax(-100, fmin(100, Asymmetry[0]));
	AsymmetryFilter = 0.005 * Asymmetry[0] + 0.995 * AsymmetryFilter;

	// 비대칭지수 평균값 계산
	if (FSMMngrObj.state_curr != 1) {
		AsymmetryAvgSum += Asymmetry[0]; // Asymmetry[0] 값을 누적
		AsymmetryAvgCnt++;
	}

	if (AsymmetryAvgCnt > 0) {
	    AsymmetryAvg = AsymmetryAvgSum / AsymmetryAvgCnt;
	} else {
		AsymmetryAvg = AsymmetryAvgSum;
	}
	AsymmetryAvg = fmax(-25.0, fmin(25.0, AsymmetryAvg));

	// 일정 주기마다 overflow 방지를 위해 초기화
	if (AsymmetryAvgCnt >= (3600000 * 3)) {  // 1ms * 3600000 = 1시간
	    AsymmetryAvgSum = 0;
	    AsymmetryAvgCnt = 1;
	}

	trainingInfo[current_session].asymmetric_level_avg[BLECtrlObj.data.AssistMode] = AsymmetryAvg;

	if (FSMMngrObj.state_curr == 8 || FSMMngrObj.state_curr == 9 || FSMMngrObj.state_curr == 12 ||
			FSMMngrObj.state_curr == 13 || FSMMngrObj.state_curr == 41 || FSMMngrObj.state_curr == 33 ||
			FSMMngrObj.state_curr == 34 || FSMMngrObj.state_curr == 37 || FSMMngrObj.state_curr == 38 ||
			FSMMngrObj.state_curr == 44 || FSMMngrObj.state_curr == 45) {
		RHamplitude_max[0] = RHamplitude_max[1];
		RHamplitude_min[0] = fmin(RHamplitude_min[1], R_ThighSagittalAngle);
		LHamplitude_max[0] = fmax(LHamplitude_max[1], L_ThighSagittalAngle);
		LHamplitude_min[0] = LHamplitude_min[1];
	}

	if (FSMMngrObj.state_curr == 10 || FSMMngrObj.state_curr == 11 || FSMMngrObj.state_curr == 15 ||
			FSMMngrObj.state_curr == 16 || FSMMngrObj.state_curr == 42 || FSMMngrObj.state_curr == 35 ||
			FSMMngrObj.state_curr == 36 || FSMMngrObj.state_curr == 39 || FSMMngrObj.state_curr == 40) {
		RHamplitude_max[0] = fmax(RHamplitude_max[1], R_ThighSagittalAngle);
		RHamplitude_min[0] = RHamplitude_min[1];
		LHamplitude_max[0] = LHamplitude_max[1];
		LHamplitude_min[0] = fmin(LHamplitude_min[1], L_ThighSagittalAngle);
	}

	if ((FSMMngrObj.state_curr == 8 || FSMMngrObj.state_curr == 9 || FSMMngrObj.state_curr == 12 ||
			FSMMngrObj.state_curr == 13 || FSMMngrObj.state_curr == 41 || FSMMngrObj.state_curr == 33 ||
			FSMMngrObj.state_curr == 34 || FSMMngrObj.state_curr == 37 || FSMMngrObj.state_curr == 38 ||
			FSMMngrObj.state_curr == 44 || FSMMngrObj.state_curr == 45)
			&& (FSM_prev_state == 10 || FSM_prev_state == 11 || FSM_prev_state == 15 ||
					FSM_prev_state == 16 || FSM_prev_state == 42 || FSM_prev_state == 35 ||
					FSM_prev_state == 36 || FSM_prev_state == 39 || FSM_prev_state == 40)) {
		GaitRHThighSagittalAngle_max = RHamplitude_max[0];
		GaitLHThighSagittalAngle_min = LHamplitude_min[0];
		RHamplitude_min[0] = 0;
		LHamplitude_max[0] = 0;
	}

	if ((FSMMngrObj.state_curr == 10 || FSMMngrObj.state_curr == 11 || FSMMngrObj.state_curr == 15 ||
			FSMMngrObj.state_curr == 16 || FSMMngrObj.state_curr == 42 || FSMMngrObj.state_curr == 35 ||
			FSMMngrObj.state_curr == 36 || FSMMngrObj.state_curr == 39 || FSMMngrObj.state_curr == 40)
		&& (FSM_prev_state == 8 || FSM_prev_state == 9 || FSM_prev_state == 12 ||
				FSM_prev_state == 13 || FSM_prev_state == 41 || FSM_prev_state == 33 ||
				FSM_prev_state == 34 || FSM_prev_state == 37 || FSM_prev_state == 38)) {
		GaitRHThighSagittalAngle_min = RHamplitude_min[0];
		GaitLHThighSagittalAngle_max = LHamplitude_max[0];
		RHamplitude_max[0] = 0;
		LHamplitude_min[0] = 0;
	}

	if (FSMMngrObj.state_curr == 4 || FSMMngrObj.state_curr == 7) {
		RHamplitude_max[0] = 0;
		RHamplitude_min[0] = 0;
		LHamplitude_max[0] = 0;
		LHamplitude_min[0] = 0;
	}

	gaitCount[1] = gaitCount[0];
	gaitCountRight[1] = gaitCountRight[0];
	gaitCountLeft[1] = gaitCountLeft[0];
	Asymmetry[1] = Asymmetry[0];
	RHamplitude_max[1] = RHamplitude_max[0];
	RHamplitude_min[1] = RHamplitude_min[0];
	LHamplitude_max[1] = LHamplitude_max[0];
	LHamplitude_min[1] = LHamplitude_min[0];
	FSM_prev_state = FSMMngrObj.state_curr;

}

static void STSAnalysis(void)
{

	countFor1secSTS++;
	// 보조 없이 Sit => Stand 갔을 때 Hold 풀어주기
	if (FSMMngrObj.state_curr == 66 && prev_state == 71)
	{
		BLECtrlObj.data.STSPostureHold = 0;
		PostureHold = 0;
	}
	//앉기 서기 count
	if (FSMMngrObj.state_curr == 71 && prev_state != 71 && prev_state != 65){sitCount++;}
	if ((((FSMMngrObj.state_curr == 66 ) || (FSMMngrObj.state_curr == 67)) &&( prev_state != 66 && prev_state != 67)) && prev_state != 65) {standCount++;}

	prev_state=FSMMngrObj.state_curr;

	if(countFor1secSTS >= 1000)
	{
		countFor1secSTS = 0;
		//Sit/Stand Time Count
		if (FSMMngrObj.state_curr >=66 && FSMMngrObj.state_curr <=68){
			standTime++;
			accumulateStandTime++;
		}
		else {
			standTime=0;
		}
		if (FSMMngrObj.state_curr >=71 && FSMMngrObj.state_curr <=72){
			sitTime++;
			accumulateSitTime++;
		}
		else {
			sitTime=0;
		}

	}

}

static void DecodingPDOData(SAM_Motion_Struct* MotionData, int DevIdx)
{
	/* Motion Data */
	MotionData->ThighSagittalAngle 				= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->thighAngleDeg), DEG_SCALING_FACTOR);
	MotionData->ThighSagittalVelocity 			= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->thighVelDeg), VELDEG_SCALING_FACTOR);

	MotionData->PositionAct 			= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->incActualPosDeg), DEG_SCALING_FACTOR);
	MotionData->EncoderVelocity 		= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->incActualVelDeg), VELDEG_SCALING_FACTOR);

	MotionData->IMU_BodyAngle_Sagittal 	= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->IMU_BodyAngle_Sagittal), DEG_SCALING_FACTOR);
	MotionData->IMU_BodyAngle_Frontal 	= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->IMU_BodyAngle_Frontal), DEG_SCALING_FACTOR);
	MotionData->IMU_BodyAngle_Transverse= ScaleInt16ToFloat((GetDevDataSet(DevIdx)->IMU_BodyAngle_Transverse), DEG_SCALING_FACTOR);
	
	/* Actuator Data */
	MotionData->MotorRefCurrent = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->currentRef), CURRENT_SCALING_FACTOR);
	MotionData->MotorActCurrent = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->currentAct), CURRENT_SCALING_FACTOR);
	MotionData->MotorActVoltage = ScaleUInt16ToFloat(GetDevDataSet(DevIdx)->volt_scaling, VOLT_SCALING_FACTOR);
	MotionData->MotorTemp 		= ScaleInt16ToFloat(GetDevDataSet(DevIdx)->motor_temp_scaling, TEMP_SCALING_FACTOR);

	MotionData->PvectorRef = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->p_vector_ref), RAD_SCALING_FACTOR);
	MotionData->PvectorRef = MotionData->PvectorRef * 180.0f / M_PI;
	MotionData->IvectorEpsilon = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->i_vector_epsilon), I_VECTOR_EPSILON_SCALING_FACTOR);
	MotionData->IvectorKp = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->i_vector_kP), I_VECTOR_KPKD_GAIN_SCALING_FACTOR);
	MotionData->IvectorLambda = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->i_vector_lambda), I_VECTOR_LAMBDA_SCALING_FACTOR);

	/* IMU Global Axis */
	MotionData->accXGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accXGlobal), ACC_SCALING_FACTOR);
	MotionData->accYGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accYGlobal), ACC_SCALING_FACTOR);
	MotionData->accZGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accZGlobal), ACC_SCALING_FACTOR);

	MotionData->gyroXGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrXGlobal), GYR_SCALING_FACTOR);
	MotionData->gyroYGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrYGlobal), GYR_SCALING_FACTOR);
	MotionData->gyroZGlobal = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrZGlobal), GYR_SCALING_FACTOR);

	/* IMU Raw */
	MotionData->accXRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accXRaw), ACC_SCALING_FACTOR);
	MotionData->accYRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accYRaw), ACC_SCALING_FACTOR);
	MotionData->accZRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accZRaw), ACC_SCALING_FACTOR);

	MotionData->gyroXRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrXRaw), GYR_SCALING_FACTOR);
	MotionData->gyroYRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrYRaw), GYR_SCALING_FACTOR);
	MotionData->gyroZRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrZRaw), GYR_SCALING_FACTOR);

	/* TVCF Check */
	// MotionData->wcTVCFCalib = ScaleUInt16ToFloat((GetDevDataSet(DevIdx)->wcTVCF_calib), WC_SCALING_FACTOR);
	// MotionData->accDegInit = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accDegInit), DEG_SCALING_FACTOR);
	// MotionData->accDegRaw = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accDegRaw), DEG_SCALING_FACTOR);
	// MotionData->accDeg = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->accDeg), DEG_SCALING_FACTOR);
	// MotionData->gyrDeg = ScaleInt16ToFloat((GetDevDataSet(DevIdx)->gyrDeg), DEG_SCALING_FACTOR);

	/* Test Data */
}

static void UpdateMotionData(void)
{
	// For Extension Board
	pelvic_Vel_Y = (RH_Sagittal.gyroYGlobal + LH_Sagittal.gyroYGlobal) / 2.0;

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
		R_ThighSagittalAngle = RH_Sagittal.ThighSagittalAngle;
		L_ThighSagittalAngle = LH_Sagittal.ThighSagittalAngle;
		R_ThighSagittalVelocity = RH_Sagittal.ThighSagittalVelocity;
		L_ThighSagittalVelocity = LH_Sagittal.ThighSagittalVelocity;

		if (FSMMngrObj.state_curr == 72){
			L_STS_stand_start_thigh_angle = L_ThighSagittalAngle;
			R_STS_stand_start_thigh_angle = R_ThighSagittalAngle;
		}

		RH_PositionAct = RH_Sagittal.PositionAct;
		LH_PositionAct = LH_Sagittal.PositionAct;
		RH_EncoderVelocity = RH_Sagittal.EncoderVelocity;
		LH_EncoderVelocity = LH_Sagittal.EncoderVelocity;

		RK_PositionAct = ThetaRK_Session;
		LK_PositionAct = ThetaLK_Session;
		RK_EncoderVelocity = (ThetaRK_Improved[0] - ThetaRK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD;
		LK_EncoderVelocity = (ThetaLK_Improved[0] - ThetaLK_Improved[1]) / WHOLEBODY_CONTROL_PERIOD;

		R_ShankSagittalAngle = R_ThighSagittalAngle - RK_PositionAct;
		L_ShankSagittalAngle = L_ThighSagittalAngle - LK_PositionAct;

		RH_IMU_BodyAngle_Sagittal = RH_Sagittal.IMU_BodyAngle_Sagittal;
		LH_IMU_BodyAngle_Sagittal = LH_Sagittal.IMU_BodyAngle_Sagittal;
		RH_IMU_BodyAngle_Frontal = RH_Sagittal.IMU_BodyAngle_Frontal;
		LH_IMU_BodyAngle_Frontal = LH_Sagittal.IMU_BodyAngle_Frontal;
		PelvicSagittalAngle = (-1.0f) * (RH_IMU_BodyAngle_Sagittal + LH_IMU_BodyAngle_Sagittal)/2 - TrunkNeutralPostureBias;  // MD Roll 방향과 반대
		PelvicFrontalAngle = (RH_IMU_BodyAngle_Frontal + LH_IMU_BodyAngle_Frontal)/2;

		RH_ThighNeutralSagittalAngle = RH_Sagittal.ThighNeutralSagittalAngle;
		LH_ThighNeutralSagittalAngle = LH_Sagittal.ThighNeutralSagittalAngle;
		RK_ThighNeutralSagittalAngle = 0;
		LK_ThighNeutralSagittalAngle = 0;

//		RH_accXCalib = RH_Sagittal.accXCalib;
//		RH_accYCalib = RH_Sagittal.accYCalib;
//		RH_gyroZCalib = RH_Sagittal.gyroZCalib;
//		LH_accXCalib = LH_Sagittal.accXCalib;
//		LH_accYCalib = LH_Sagittal.accYCalib;
//		LH_gyroZCalib = LH_Sagittal.gyroZCalib;

		RH_accXGlobal[0] = RH_Sagittal.accXGlobal;
		RH_accYGlobal[0] = RH_Sagittal.accYGlobal;
		RH_accZGlobal[0] = RH_Sagittal.accZGlobal;
		RH_gyroXGlobal[0] = RH_Sagittal.gyroXGlobal;
		RH_gyroYGlobal[0] = RH_Sagittal.gyroYGlobal;
		RH_gyroZGlobal[0] = RH_Sagittal.gyroZGlobal;

		LH_accXGlobal[0] = LH_Sagittal.accXGlobal;
		LH_accYGlobal[0] = LH_Sagittal.accYGlobal;
		LH_accZGlobal[0] = LH_Sagittal.accZGlobal;
		LH_gyroXGlobal[0] = LH_Sagittal.gyroXGlobal;
		LH_gyroYGlobal[0] = LH_Sagittal.gyroYGlobal;
		LH_gyroZGlobal[0] = LH_Sagittal.gyroZGlobal;
	}

	if (RS_File.vec[RH_SAGITAL].usage == 0 && RS_File.vec[LH_SAGITAL].usage == 0 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		R_ThighSagittalAngle = RK_Sagittal.ThighSagittalAngle;
		L_ThighSagittalAngle = LK_Sagittal.ThighSagittalAngle;
		R_ThighSagittalVelocity = RK_Sagittal.ThighSagittalVelocity;
		L_ThighSagittalVelocity = LK_Sagittal.ThighSagittalVelocity;

		RK_PositionAct = RK_Sagittal.PositionAct;
		LK_PositionAct = LK_Sagittal.PositionAct;
		RK_EncoderVelocity = RK_Sagittal.EncoderVelocity;
		LK_EncoderVelocity = LK_Sagittal.EncoderVelocity;

		R_ShankSagittalAngle = R_ThighSagittalAngle - RK_PositionAct;
		L_ShankSagittalAngle = L_ThighSagittalAngle - LK_PositionAct;

		RK_IMU_BodyAngle_Sagittal = RK_Sagittal.IMU_BodyAngle_Sagittal;
		LK_IMU_BodyAngle_Sagittal = LK_Sagittal.IMU_BodyAngle_Sagittal;
		RK_IMU_BodyAngle_Frontal = RK_Sagittal.IMU_BodyAngle_Frontal;
		LK_IMU_BodyAngle_Frontal = LK_Sagittal.IMU_BodyAngle_Frontal;
		PelvicSagittalAngle = (RK_IMU_BodyAngle_Sagittal + LK_IMU_BodyAngle_Sagittal)/2;
		PelvicFrontalAngle = (RK_IMU_BodyAngle_Frontal + LK_IMU_BodyAngle_Frontal)/2;

		RH_ThighNeutralSagittalAngle = RK_Sagittal.ThighNeutralSagittalAngle;
		LH_ThighNeutralSagittalAngle = LK_Sagittal.ThighNeutralSagittalAngle;
		RK_ThighNeutralSagittalAngle = RK_Sagittal.ShankNeutralSagittalAngle;
		LK_ThighNeutralSagittalAngle = LK_Sagittal.ShankNeutralSagittalAngle;

//		RK_accXCalib = RK_Sagittal.accXCalib;
//		RK_accYCalib = RK_Sagittal.accYCalib;
//		RK_gyroZCalib = RK_Sagittal.gyroZCalib;
//		LK_accXCalib = LK_Sagittal.accXCalib;
//		LK_accYCalib = LK_Sagittal.accYCalib;
//		LK_gyroZCalib = LK_Sagittal.gyroZCalib;

		RK_accXGlobal[0] = RK_Sagittal.accXGlobal;
		RK_accYGlobal[0] = RK_Sagittal.accYGlobal;
		RK_accZGlobal[0] = RK_Sagittal.accZGlobal;
		RK_gyroXGlobal[0] = RK_Sagittal.gyroYGlobal;
		RK_gyroYGlobal[0] = RK_Sagittal.gyroYGlobal;
		RK_gyroZGlobal[0] = RK_Sagittal.gyroZGlobal;

		LK_accXGlobal[0] = LK_Sagittal.accXGlobal;
		LK_accYGlobal[0] = LK_Sagittal.accYGlobal;
		LK_accZGlobal[0] = LK_Sagittal.accZGlobal;
		LK_gyroXGlobal[0] = LK_Sagittal.gyroXGlobal;
		LK_gyroYGlobal[0] = LK_Sagittal.gyroYGlobal;
		LK_gyroZGlobal[0] = LK_Sagittal.gyroZGlobal;
	}

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		R_ThighSagittalAngle = RH_Sagittal.ThighSagittalAngle;
		L_ThighSagittalAngle = LH_Sagittal.ThighSagittalAngle;

		R_ThighSagittalVelocity_1 = RH_Sagittal.ThighSagittalVelocity;
		L_ThighSagittalVelocity_1 = LH_Sagittal.ThighSagittalVelocity;
		R_ThighSagittalVelocity = (R_ThighSagittalVelocity_1 + R_ThighSagittalVelocity_2 + R_ThighSagittalVelocity_3)/3;
		L_ThighSagittalVelocity = (L_ThighSagittalVelocity_1 + L_ThighSagittalVelocity_2 + L_ThighSagittalVelocity_3)/3;

		R_ThighSagittalVelocity_3 = R_ThighSagittalVelocity_2;
		R_ThighSagittalVelocity_2 = R_ThighSagittalVelocity_1;
		L_ThighSagittalVelocity_3 = L_ThighSagittalVelocity_2;
		L_ThighSagittalVelocity_2 = L_ThighSagittalVelocity_1;

		RH_PositionAct = RH_Sagittal.PositionAct;
		LH_PositionAct = LH_Sagittal.PositionAct;
		RH_EncoderVelocity = RH_Sagittal.EncoderVelocity;
		LH_EncoderVelocity = LH_Sagittal.EncoderVelocity;

		RH_IMU_BodyAngle_Sagittal = RH_Sagittal.IMU_BodyAngle_Sagittal;
		LH_IMU_BodyAngle_Sagittal = LH_Sagittal.IMU_BodyAngle_Sagittal;
		RH_IMU_BodyAngle_Frontal = RH_Sagittal.IMU_BodyAngle_Frontal;
		LH_IMU_BodyAngle_Frontal = LH_Sagittal.IMU_BodyAngle_Frontal;
		PelvicSagittalAngle = (-1.0f) * (RH_IMU_BodyAngle_Sagittal + LH_IMU_BodyAngle_Sagittal)/2 - TrunkNeutralPostureBias;  // MD Roll 방향과 반대
		PelvicFrontalAngle = (RH_IMU_BodyAngle_Frontal + LH_IMU_BodyAngle_Frontal)/2;

		RH_ThighNeutralSagittalAngle = RH_Sagittal.ThighNeutralSagittalAngle;
		LH_ThighNeutralSagittalAngle = LH_Sagittal.ThighNeutralSagittalAngle;

//		RH_accXCalib = RH_Sagittal.accXCalib;
//		RH_accYCalib = RH_Sagittal.accYCalib;
//		RH_gyroZCalib = RH_Sagittal.gyroZCalib;
//		LH_accXCalib = LH_Sagittal.accXCalib;
//		LH_accYCalib = LH_Sagittal.accYCalib;
//		LH_gyroZCalib = LH_Sagittal.gyroZCalib;

		RH_accXGlobal[0] = RH_Sagittal.accXGlobal;
		RH_accYGlobal[0] = RH_Sagittal.accYGlobal;
		RH_accZGlobal[0] = RH_Sagittal.accZGlobal;
		RH_gyroXGlobal[0] = RH_Sagittal.gyroXGlobal;
		RH_gyroYGlobal[0] = RH_Sagittal.gyroYGlobal;
		RH_gyroZGlobal[0] = RH_Sagittal.gyroZGlobal;

		LH_accXGlobal[0] = LH_Sagittal.accXGlobal;
		LH_accYGlobal[0] = LH_Sagittal.accYGlobal;
		LH_accZGlobal[0] = LH_Sagittal.accZGlobal;
		LH_gyroXGlobal[0] = LH_Sagittal.gyroXGlobal;
		LH_gyroYGlobal[0] = LH_Sagittal.gyroYGlobal;
		LH_gyroZGlobal[0] = LH_Sagittal.gyroZGlobal;

//		R_ThighSagittalAngle = RK_Sagittal.ThighSagittalAngle;
//		L_ThighSagittalAngle = LK_Sagittal.ThighSagittalAngle;
//		R_ThighSagittalVelocity = RK_Sagittal.ThighSagittalVelocity;
//		L_ThighSagittalVelocity = LK_Sagittal.ThighSagittalVelocity;

		RK_PositionAct = RK_Sagittal.PositionAct;
		LK_PositionAct = LK_Sagittal.PositionAct;
		RK_EncoderVelocity = RK_Sagittal.EncoderVelocity;
		LK_EncoderVelocity = LK_Sagittal.EncoderVelocity;

		R_ShankSagittalAngle = R_ThighSagittalAngle - RK_PositionAct;
		L_ShankSagittalAngle = L_ThighSagittalAngle - LK_PositionAct;

		RK_IMU_BodyAngle_Sagittal = RK_Sagittal.IMU_BodyAngle_Sagittal;
		LK_IMU_BodyAngle_Sagittal = LK_Sagittal.IMU_BodyAngle_Sagittal;
		RK_IMU_BodyAngle_Frontal = RK_Sagittal.IMU_BodyAngle_Frontal;
		LK_IMU_BodyAngle_Frontal = LK_Sagittal.IMU_BodyAngle_Frontal;

//		RH_ThighNeutralSagittalAngle = RK_Sagittal.ThighNeutralSagittalAngle;
//		LH_ThighNeutralSagittalAngle = LK_Sagittal.ThighNeutralSagittalAngle;
		RK_ThighNeutralSagittalAngle = RK_Sagittal.ShankNeutralSagittalAngle;
		LK_ThighNeutralSagittalAngle = LK_Sagittal.ShankNeutralSagittalAngle;

//		RK_accXCalib = RK_Sagittal.accXCalib;
//		RK_accYCalib = RK_Sagittal.accYCalib;
//		RK_gyroZCalib = RK_Sagittal.gyroZCalib;
//		LK_accXCalib = LK_Sagittal.accXCalib;
//		LK_accYCalib = LK_Sagittal.accYCalib;
//		LK_gyroZCalib = LK_Sagittal.gyroZCalib;

		RK_accXGlobal[0] = RK_Sagittal.accXGlobal;
		RK_accYGlobal[0] = RK_Sagittal.accYGlobal;
		RK_accZGlobal[0] = RK_Sagittal.accZGlobal;
		RK_gyroXGlobal[0] = RK_Sagittal.gyroYGlobal;
		RK_gyroYGlobal[0] = RK_Sagittal.gyroYGlobal;
		RK_gyroZGlobal[0] = RK_Sagittal.gyroZGlobal;

		LK_accXGlobal[0] = LK_Sagittal.accXGlobal;
		LK_accYGlobal[0] = LK_Sagittal.accYGlobal;
		LK_accZGlobal[0] = LK_Sagittal.accZGlobal;
		LK_gyroXGlobal[0] = LK_Sagittal.gyroXGlobal;
		LK_gyroYGlobal[0] = LK_Sagittal.gyroYGlobal;
		LK_gyroZGlobal[0] = LK_Sagittal.gyroZGlobal;
	}
}

static void CheckNeutralPostureCMD(void)
{
    static bool CheckNeutralized = false;
    if (CheckNeutralized == false) {
        for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
            uint8_t tUsage = RS_File.vec[DevIdx].usage;
 
            if (tUsage == 1) {
                SendIsNeutralized(DevIdx, (uint8_t)1);
                SendIsNeutralInit(DevIdx, (uint8_t)1);
            }
        }
        CheckNeutralized = true;
        isNeutralSaved = false;
    }
 
    if (isNeutralSaved == false) {
        UpdateNotiSignal();
 
		if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
			if (RH_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED && LH_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED) {
				isNeutralSaved = true;
				RH_Sagittal.notiSignals = 0;
				LH_Sagittal.notiSignals = 0;
				for (int DevIdx = RH_SAGITAL; DevIdx < RK_SAGITAL; ++DevIdx) {
					userCtrlObj[DevIdx].devObj.noti = 0;
				}
			}
		}
		
		if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
			if (RK_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED && LK_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED) {
				isNeutralSaved = true;
				RK_Sagittal.notiSignals = 0;
				LK_Sagittal.notiSignals = 0;
				for (int DevIdx = RK_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
					userCtrlObj[DevIdx].devObj.noti = 0;
				}
			}
		}
    }

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
		if (RH_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED && LH_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED) {
        	BFlag[20] = 1; // 왼쪽 허벅지만 중립 각도가 설정되지 않은 경우
		} else if (RH_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED && LH_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED) {
			BFlag[21] = 1; // 오른쪽 허벅지만 중립각도가 설정되지 않은 경우
		} else if (RH_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED && LH_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED) {
			BFlag[22] = 1; // 양쪽 허벅지 중립각도가 설정되지 않은 경우
		}
	}

	if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
		if (RK_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED && LK_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED) {
			// BFlag[20] = 1; // 왼쪽 허벅지만 중립 각도가 설정되지 않은 경우
		} else if (RK_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED && LK_Sagittal.notiSignals == NEUTRAL_POSTURE_CALIBRATED) {
			// BFlag[21] = 1; // 오른쪽 허벅지만 중립각도가 설정되지 않은 경우
		} else if (RK_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED && LK_Sagittal.notiSignals == NEUTRAL_POSTURE_NOT_CALIBRATED) {
			// BFlag[22] = 1; // 양쪽 허벅지 중립각도가 설정되지 않은 경우
		}
	}

    if (neutralPosCalCMD == 1) {
        for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
            uint8_t tUsage = RS_File.vec[DevIdx].usage;
 
            if (tUsage == 1) {
                SendNeutralPostureCalCMD(DevIdx, (uint8_t)1);
            }
        }
        isNeutralCalCompleted = false;
        IOIF_LED24chAssist(ON, 0, 10, 0x20); // Assist LED 10ea ON
        neutralPosCalCMD = 0;
		calTrunkNeutralPostureCmd = 1;
		TrunkNeutralPostureBias = 0.0f;
    }

    if (isNeutralCalCompleted == false) {
        UpdateNotiSignal();

		if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 0 && RS_File.vec[LK_SAGITAL].usage == 0) {
			if (RH_Sagittal.notiSignals == 1 && LH_Sagittal.notiSignals == 1) {
				isNeutralCalCompleted = true;
				BFlag[23] = 1;
				assistStage = 0;
				IOIF_LED24chAssist(ON, 0, 10, 0x00);   // Assist LED 10ea OFF
				RH_Sagittal.notiSignals = 0;
				LH_Sagittal.notiSignals = 0;
				for (int DevIdx = RH_SAGITAL; DevIdx < RK_SAGITAL; ++DevIdx) {
					userCtrlObj[DevIdx].devObj.noti = 0;
				}
			}
		}

		if (RS_File.vec[RH_SAGITAL].usage == 1 && RS_File.vec[LH_SAGITAL].usage == 1 && RS_File.vec[RK_SAGITAL].usage == 1 && RS_File.vec[LK_SAGITAL].usage == 1) {
			if (RK_Sagittal.notiSignals == 1 && LK_Sagittal.notiSignals == 1) {
				isNeutralCalCompleted = true;
				BFlag[23] = 1;
				assistStage = 0;
				IOIF_LED24chAssist(ON, 0, 10, 0x00);   // Assist LED 10ea OFF
				RK_Sagittal.notiSignals = 0;
				LK_Sagittal.notiSignals = 0;
				for (int DevIdx = RK_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
					userCtrlObj[DevIdx].devObj.noti = 0;
				}
			}
		}
    }

    if (isNeutralCalCompleted == true && FSMMngrObj.state_curr == 27) {
    	BFlag[19] = 1;
    }
}

void UpdateNotiSignal(void)
{
	for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
		uint8_t tUsage = RS_File.vec[DevIdx].usage;

		if (tUsage == 1) {
			switch (DevIdx) {
			case RH_SAGITAL:
				RH_Sagittal.notiSignals = userCtrlObj[DevIdx].devObj.noti;
				break;
			case LH_SAGITAL:
				LH_Sagittal.notiSignals = userCtrlObj[DevIdx].devObj.noti;
				break;
			case RK_SAGITAL:
				RK_Sagittal.notiSignals = userCtrlObj[DevIdx].devObj.noti;
				break;
			case LK_SAGITAL:
				LK_Sagittal.notiSignals = userCtrlObj[DevIdx].devObj.noti;
				break;
			default:
				break;
			}
		}
	}
}

// Function to scale int16 to a float type
static float ScaleInt16ToFloat(int16_t value, float scaleFactor)
{
	// Scale the float value
	float scaledValue = (float)(value * scaleFactor / DATA_CONV_CONST_INT16);

	return scaledValue;
}

// Function to scale uint16 to a float type
static float ScaleUInt16ToFloat(uint16_t value, float scaleFactor)
{
	// Scale the float value
	float scaledValue = (float)(value * scaleFactor / DATA_CONV_CONST_UINT16);

	return scaledValue;
}

/* Data Storage Functions(to binary) */
static void SaveDataMassStorage(void)
{
	if ((FSMMngrObj.state_curr >= 4 && FSMMngrObj.state_curr <= 17) || (FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
			(FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75)){
		if (dataSaveCnt == 0){
			SetMetadata();
			memset(&sensordata, 0, sizeof(Sensor_LogPacket_V1_t));
			dataSaveMode = DATA_SAVE_PREPARING;
		}
		dataSaveCnt++;
		
	} else {
		dataSaveMode = DATA_SAVE_ABORT;
		dataSaveCnt=0;
	}

	uint8_t t_dataCtrlState = DOPC_GetTaskState(TASK_IDX_DATA_CTRL);

	if (t_dataCtrlState == TASK_STATE_ENABLE) {
		if (dataSaveMode == DATA_SAVE_ON) { // Data Ctrl Task State Standby -> Enable : DOPC_SetTaskState(TASK_IDX_DATA_CTRL, TASK_STATE_ENABLE)
			sensordata.seq++;
			sensordata.fsm_state = FSMMngrObj.state_curr;
			sensordata.assist_ratio = assistStage;

			sensordata.left_hip.encoder = LH_Sagittal.PositionAct;

			sensordata.left_hip.acc_global.x = LH_Sagittal.accXGlobal;
			sensordata.left_hip.acc_global.y = LH_Sagittal.accYGlobal;
			sensordata.left_hip.acc_global.z = LH_Sagittal.accZGlobal;
			sensordata.left_hip.gyro_global.x = LH_Sagittal.gyroXGlobal;
			sensordata.left_hip.gyro_global.y = LH_Sagittal.gyroYGlobal;
			sensordata.left_hip.gyro_global.z = LH_Sagittal.gyroZGlobal;

			sensordata.left_hip.euler.roll = LH_Sagittal.IMU_BodyAngle_Frontal;
			sensordata.left_hip.euler.pitch = LH_Sagittal.IMU_BodyAngle_Sagittal;

			sensordata.left_hip.act_current = LH_Sagittal.MotorActCurrent;

			sensordata.right_hip.encoder = RH_Sagittal.PositionAct;

			sensordata.right_hip.acc_global.x = RH_Sagittal.accXGlobal;
			sensordata.right_hip.acc_global.y = RH_Sagittal.accYGlobal;
			sensordata.right_hip.acc_global.z = RH_Sagittal.accZGlobal;
			sensordata.right_hip.gyro_global.x = RH_Sagittal.gyroXGlobal;
			sensordata.right_hip.gyro_global.y = RH_Sagittal.gyroYGlobal;
			sensordata.right_hip.gyro_global.z = RH_Sagittal.gyroZGlobal;

			sensordata.right_hip.euler.roll = RH_Sagittal.IMU_BodyAngle_Frontal;
			sensordata.right_hip.euler.pitch = RH_Sagittal.IMU_BodyAngle_Sagittal;

			sensordata.right_hip.act_current = RH_Sagittal.MotorActCurrent;


			osMessageQueuePut( sensorDataLogQueueHandle, &sensordata, 0, 0 );
//			hexDataCursor = 0;
//			memset(sdSendDataToHex, 0, sizeof(sdSendDataToHex));
//
//   /* CM Data */
//			Append_SD_Data_Uint32(CONV_UINT32, dataSaveCnt);
//			Append_SD_Data_Uint16(CONV_UINT16, FSMMngrObj.state_curr);
//     Append_SD_Data_Float16(CONV_FLOAT16, Theta_Trunk, DEG_ENC_CONSTANT);
//     Append_SD_Data_Float16(CONV_FLOAT16, R_ThighSagittalVelocity, VELDEG_ENC_CONSTANT);
  //   Append_SD_Data_Float16(CONV_FLOAT16, L_ThighSagittalVelocity, VELDEG_ENC_CONSTANT);
  //   Append_SD_Data_Float16(CONV_FLOAT16, dPLeftShank_X, 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, dPRightShank_X, 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, ThetaLShank[0], 200);
  //   Append_SD_Data_Float16(CONV_FLOAT16, ThetaRShank[0], 200);
  //   Append_SD_Data_Float16(CONV_FLOAT16, P2L_Y[0], 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, P2R_Y[0], 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, fLeftKnee_X, 1);
  //   Append_SD_Data_Float16(CONV_FLOAT16, fRightKnee_X, 1);
  //   Append_SD_Data_Float16(CONV_FLOAT16, velXEstimation, 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, velYEstimation, 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, velX[0], 2000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, velY[0], 2000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, posX[0], 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, posY[0], 100);
  //   Append_SD_Data_Uint8(CONV_UINT8, SingleStance);
  //   Append_SD_Data_Uint8(CONV_UINT8, LeftFootContact);
  //   Append_SD_Data_Uint8(CONV_UINT8, RightFootContact);
  //   Append_SD_Data_Uint8(CONV_UINT8, ISI_Stationary[0]);
  //   Append_SD_Data_Float16(CONV_FLOAT16, accSum, 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, MotionCheck, 1000);
  //   Append_SD_Data_Float16(CONV_FLOAT16, RH_accXGlobal[0], 1500);
  //   Append_SD_Data_Float16(CONV_FLOAT16, RH_accYGlobal[0], 1500);
  //   Append_SD_Data_Float16(CONV_FLOAT16, LH_accXGlobal[0], 1500);
  //   Append_SD_Data_Float16(CONV_FLOAT16, LH_accYGlobal[0], 1500);
  //   Append_SD_Data_Uint32(CONV_UINT32, gaitCount[0]);
  //   Append_SD_Data_Float16(CONV_FLOAT16, RHamplitude_max[0], 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, RHamplitude_min[0], 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, LHamplitude_max[0], 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, LHamplitude_min[0], 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, GaitPeriod, 100);
  //   Append_SD_Data_Float16(CONV_FLOAT16, Cadence, 100);
  //   Append_SD_Data_Uint64(CONV_UINT64, BFlagsSDData);   // B Flag
  //   Append_SD_Data_Uint64(CONV_UINT64, ISISDData);      // ISI
     // CM Gyro & Magnetometer
     // Append_SD_Data_Float16(CONV_FLOAT16, widmSensorDataObj_sagittal.accGlobalX, ACC_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, widmSensorDataObj_sagittal.accGlobalY, ACC_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, widmSensorDataObj_sagittal.gyrGlobalX, GYRO_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, widmSensorDataObj_sagittal.gyrGlobalY, GYRO_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, CM_imu6AxisDataObj.gyrY, GYRO_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, CM_imu6AxisDataObj.gyrZ, GYRO_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, CM_magDataObj.magX, MAG_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, CM_magDataObj.magY, MAG_CONSTANT);
     // Append_SD_Data_Float16(CONV_FLOAT16, CM_magDataObj.magZ, MAG_CONSTANT);

   /* SAM Data */
//   	   	   for (int DevIdx = RH_SAGITAL; DevIdx < SUIT_MODULE_NUM; ++DevIdx) {
//   	   		   uint8_t tUsage = RS_File.vec[DevIdx].usage;
//   	   		   if (tUsage == 1) {
//   	   			   switch (DevIdx) {
//   	   			   case RH_SAGITAL:
//   	   				   AppendSDData(RH_Sagittal, DevIdx);
//   	   				   break;
//   	   			   case LH_SAGITAL:
//   	   				   AppendSDData(LH_Sagittal, DevIdx);
//   	   				   break;
//   	   			   case RK_SAGITAL:
//   	   				   AppendSDData(RK_Sagittal, DevIdx);
//   	   				   break;
//   	   			   case LK_SAGITAL:
//   	   				   AppendSDData(LK_Sagittal, DevIdx);
//   	   				   break;
//   	   			   default:
//   	   				   break;
//   	   			   }
//   	   		   }
//   	   	   }

		   /* CRLF end line */
//		   sdSendDataToHex[hexDataCursor++] = '\r';
//		   sdSendDataToHex[hexDataCursor++] = '\n';
//
//		   _SD_DBuffered_Transmit(sdSendDataToHex, sizeof(sdSendDataToHex));
//
//		}
//		if (dataSaveMode == DATA_SAVE_ABORT) {
//	        /* 1. 남은 데이터 강제 저장 */
//	        if (*cur_buf_pos > 0) {  // 버퍼에 데이터가 남아있으면
//	            _SD_DBuffered_Transmit(sd_buf_cur, *cur_buf_pos);  // 강제 전송
//	        }
//
//	        /* 2. 세마포어 신호 (기존 버퍼 내용 처리 대기) */
//	        osSemaphoreRelease(sdio_sync_semaphore);
//
//			dataSaveCnt = 0;
		}
	}

}

static void AppendSDData(SAM_Motion_Struct MotionData, int DevIdx)
{
 /* Motion Data */
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.ThighSagittalAngle, DEG_ENC_CONSTANT);
 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.ThighSagittalVelocity, VELDEG_ENC_CONSTANT);

 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.PositionAct, DEG_ENC_CONSTANT);
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.EncoderVelocity, VELDEG_ENC_CONSTANT);

 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.IMU_BodyAngle_Sagittal, DEG_ENC_CONSTANT);
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.IMU_BodyAngle_Frontal, DEG_ENC_CONSTANT);
  // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.ThighSagittalNeutralPosture, DEG_ENC_CONSTANT);

  /* Actuator Data */
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.MotorRefCurrent, CURRENT_CONSTANT);
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.MotorActCurrent, CURRENT_CONSTANT);

  // Pvector Reference
  // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.PvectorRef, DEG_ENC_CONSTANT);

  // Ivector Parameters
  // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.IvectorEpsilon, I_VECTOR_EPSILON_CONSTANT);
  // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.IvectorKp, I_VECTOR_KPKD_CONSTANT);
  // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.IvectorLambda, I_VECTOR_LAMBDA_CONSTANT);

  /* IMU Auto Calibration Check */
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.accXGlobal, ACC_CONSTANT);
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.accYGlobal, ACC_CONSTANT);
 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.accZGlobal, ACC_CONSTANT);

 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.gyroXGlobal, GYRO_CONSTANT);
 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.gyroYGlobal, GYRO_CONSTANT);
 Append_SD_Data_Float16(CONV_FLOAT16, MotionData.gyroZGlobal, GYRO_CONSTANT);

  /* TVCF Check */
 //  Append_SD_Data_Float16(CONV_FLOAT16, MotionData.wcTVCFCalib, WC_CONSTANT);

  /* FSR Data */
  // Append_SD_Data_Uint16(CONV_UINT16, userCtrlObj[DevIdx].data.fsr_middleToe);
  // Append_SD_Data_Uint16(CONV_UINT16, userCtrlObj[DevIdx].data.fsr_bigToe);
  // Append_SD_Data_Uint16(CONV_UINT16, userCtrlObj[DevIdx].data.fsr_littleToe);
  // Append_SD_Data_Uint16(CONV_UINT16, userCtrlObj[DevIdx].data.fsr_heel);

 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.MotorTemp, TEMP_CONSTANT);
 // Append_SD_Data_Float16(CONV_FLOAT16, MotionData.MotorMCUTemp, TEMP_CONSTANT);

}

/* Key Function : SDIO Transmit with double buffer & swap(Switching) buffer algorithm */
static inline void _SD_DBuffered_Transmit(uint8_t *data, uint_fast16_t data_len) 
{
	cur_buf_pos = (sd_buf_cur == sd_buf_1) ? &sd_buf_1_pos : &sd_buf_2_pos;
	cur_buf_space = SWAP_SD_BUF_SIZE - *cur_buf_pos;

	// // 현재 버퍼의 남은 공간이 데이터 길이보다 작은 경우 바로 버퍼를 스왑
	// if (cur_buf_space < data_len) {
	//     // 새 버퍼로 스왑
	//     sd_buf_cur = (sd_buf_cur == sd_buf_1) ? sd_buf_2 : sd_buf_1;
	//     cur_buf_pos = (sd_buf_cur == sd_buf_1) ? &sd_buf_2_pos : &sd_buf_1_pos;
	//     *cur_buf_pos = 0; // 새 버퍼의 포지션 초기화

	//     // 새 버퍼에 전체 데이터 복사
	//     memcpy(sd_buf_cur, data, data_len);
	//     *cur_buf_pos = data_len; // 새 버퍼의 포지션 업데이트

	//     osSemaphoreRelease(sdio_sync_semaphore); // 세마포어 신호 발생 : 스왑 후 가득 찬 데이터 SDcard 전송을 위해
	// } else {
	//     // 현재 버퍼에 충분한 공간이 있을 경우
	//     memcpy(sd_buf_cur + *cur_buf_pos, data, data_len);
	//     *cur_buf_pos += data_len;
	// }

	if (cur_buf_space > data_len) {
		memcpy(sd_buf_cur + *cur_buf_pos, data, data_len);
		*cur_buf_pos += data_len;
	}else if(cur_buf_space == data_len){
		memcpy(sd_buf_cur + *cur_buf_pos, data, data_len);
		*cur_buf_pos += data_len;

		if (sd_buf_cur == sd_buf_1) {
			sd_buf_1_pos += cur_buf_space;
			sd_buf_cur = sd_buf_2;
			cur_buf_pos = &sd_buf_2_pos;
			sd_buf_2_pos = 0;
		} else {
			sd_buf_2_pos += cur_buf_space;
			sd_buf_cur = sd_buf_1;
			cur_buf_pos = &sd_buf_1_pos;
			sd_buf_1_pos = 0;
		}

		osSemaphoreRelease(sdio_sync_semaphore);							// release semaphore : start writing buffered data to SD card
	} else {
		memcpy(sd_buf_cur + *cur_buf_pos, data, cur_buf_space);
		uint_fast16_t remaining_data_len = data_len - cur_buf_space;

		if (sd_buf_cur == sd_buf_1) {
			sd_buf_1_pos += cur_buf_space;
			sd_buf_cur = sd_buf_2;
			cur_buf_pos = &sd_buf_2_pos;
			sd_buf_2_pos = 0;
		} else {
			sd_buf_2_pos += cur_buf_space;
			sd_buf_cur = sd_buf_1;
			cur_buf_pos = &sd_buf_1_pos;
			sd_buf_1_pos = 0;
		}

		memcpy(sd_buf_cur, data + cur_buf_space, remaining_data_len);
		*cur_buf_pos = remaining_data_len;

		osSemaphoreRelease(sdio_sync_semaphore);							// release semaphore : start writing buffered data to SD card
	}
}

static char Num2Str(uint8_t num)
{
	switch (num) {
	case 0:
		return '0';
	case 1:
		return '1';
	case 2:
		return '2';
	case 3:
		return '3';
	case 4:
		return '4';
	case 5:
		return '5';
	case 6:
		return '6';
	case 7:
		return '7';
	case 8:
		return '8';
	case 9:
		return '9';
	case 10:
		return 'A';
	case 11:
		return 'B';
	case 12:
		return 'C';
	case 13:
		return 'D';
	case 14:
		return 'E';
	case 15:
		return 'F';
	default:
		break;
	}
	return 0;
}

static void Set_RH_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&RH_Sagittal.ThighNeutralSagittalAngle, t_req->data, sizeof(RH_Sagittal.ThighNeutralSagittalAngle));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_LH_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&LH_Sagittal.ThighNeutralSagittalAngle, t_req->data, sizeof(LH_Sagittal.ThighNeutralSagittalAngle));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_RK_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&RK_Sagittal.ThighNeutralSagittalAngle, t_req->data, sizeof(RK_Sagittal.ThighNeutralSagittalAngle));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_LK_NeutralPosture(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&LK_Sagittal.ThighNeutralSagittalAngle, t_req->data, sizeof(LK_Sagittal.ThighNeutralSagittalAngle));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_RH_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&STS_P_Vector_Duration_Completed_RH, t_req->data, sizeof(STS_P_Vector_Duration_Completed_RH));

	SendPVectorDurationCompleted_RH(STS_P_Vector_Duration_Completed_RH);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_LH_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&STS_P_Vector_Duration_Completed_LH, t_req->data, sizeof(STS_P_Vector_Duration_Completed_LH));

	SendPVectorDurationCompleted_LH(STS_P_Vector_Duration_Completed_LH);
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_RK_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&STS_P_Vector_Duration_Completed_RK, t_req->data, sizeof(STS_P_Vector_Duration_Completed_RK));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_LK_P_Vector_Duration_Completed(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&STS_P_Vector_Duration_Completed_LK, t_req->data, sizeof(STS_P_Vector_Duration_Completed_LK));

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Acc_Mean_Value(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res){
	if(t_req->nodeID==6)
		{
			memcpy(&userCtrlObj[RH_SAGITAL].data.accMeanValue, t_req->data, sizeof(userCtrlObj[RH_SAGITAL].data.accMeanValue));
		}
	else if(t_req->nodeID==7)
	{
		memcpy(&userCtrlObj[LH_SAGITAL].data.accMeanValue, t_req->data, sizeof(userCtrlObj[LH_SAGITAL].data.accMeanValue));
	}
	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Append_SD_Data_Float16(uint8_t t_size, float t_data, float conversion_constant) // for float
{
	int16_t conv_data = 0;
	conv_data = (int16_t)(t_data * conversion_constant);

	if (hexDataCursor + t_size - 1 < HEX_BUF_SIZE) {
		sdSendDataToHex[hexDataCursor++]   = Num2Str((conv_data & 0xF000) >> 12);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((conv_data & 0x0F00) >> 8);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((conv_data & 0x00F0) >> 4);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((conv_data & 0x000F));
	}
}

static void Append_SD_Data_Uint8(uint8_t t_size, uint8_t t_data)
{
	if (hexDataCursor + t_size - 1 < HEX_BUF_SIZE) {
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0xF0) >> 4);
		sdSendDataToHex[hexDataCursor++]   = Num2Str(t_data & 0x0F);
	}
}

static void Append_SD_Data_Uint16(uint8_t t_size, uint16_t t_data) // for uint16
{
	if (hexDataCursor + t_size - 1 < HEX_BUF_SIZE) {
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0xF000) >> 12);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0F00) >> 8);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x00F0) >> 4);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x000F));
	}
}

static void Append_SD_Data_Uint32(uint8_t t_size, uint32_t t_data) // for uint32
{
	if (hexDataCursor + t_size - 1 < HEX_BUF_SIZE) {
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0xF0000000) >> 28);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0F000000) >> 24);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x00F00000) >> 20);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x000F0000) >> 16);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000F000) >> 12);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x00000F00) >> 8);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x000000F0) >> 4);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F));
	}
}

static void Append_SD_Data_Uint64(uint8_t t_size, uint64_t t_data) // for uint64
{
	if (hexDataCursor + t_size - 1 < HEX_BUF_SIZE) {
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0xF000000000000000) >> 60);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0F00000000000000) >> 56);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x00F0000000000000) >> 52);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x000F000000000000) >> 48);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000F00000000000) >> 44);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x00000F0000000000) >> 40);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x000000F000000000) >> 36);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 32);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 28);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 24);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 20);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 16);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 12);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 8);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000) >> 4);
		sdSendDataToHex[hexDataCursor++]   = Num2Str((t_data & 0x0000000F00000000));
	}
}


static void SetMetadata()
{
	memset(&meta, 0, sizeof(meta));

	static uint32_t seq = 0;
	IOIF_MCP79510_GetTime(&RTC_Time);
	uint8_t suit_hw_version = IOIF_SYS_HwRevGPIO();
	uint8_t serial_number_front[SERIAL_NUM_FRONT_LEN] = {0,};
	
	// 플래시에서 직접 Serial_number_front를 읽어옴
	uint32_t readAddr = IOIF_FLASH_SECTOR_6_BANK2_ADDR + IOIF_FLASH_READ_ADDR_SIZE_12B;
	IOIF_ReadFlash(readAddr, serial_number_front, sizeof(serial_number_front));

	meta.packet_ver = (uint8_t) 1;
	meta.period = (uint8_t) 1;
	
	meta.time = (uint32_t) IOIF_MCP79510_convert_to_linuxtime(&RTC_Time);
	strncpy(meta.h10_id, (char*)serial_number_front, H10_ID_STR_LEN);
	meta.h10_id[H10_ID_STR_LEN] = '\0';
	meta.cm_hw_ver.major = (uint8_t) 1;
	meta.cm_hw_ver.minor = (uint8_t)suit_hw_version;
	meta.cm_hw_ver.patch = (uint8_t) 0;
	
	meta.cm_sw_ver.major = FW_VER_MAJOR;
	meta.cm_sw_ver.minor = FW_VER_MINOR;
	meta.cm_sw_ver.patch = FW_VER_PATCH;

	meta.lh_md_sw_ver.major = BLECtrlObj.data_manu.LH_MD_SW_ver_major;
	meta.lh_md_sw_ver.minor = BLECtrlObj.data_manu.LH_MD_SW_ver_minor;
	meta.lh_md_sw_ver.patch = BLECtrlObj.data_manu.LH_MD_SW_ver_patch;

	meta.rh_md_sw_ver.major = BLECtrlObj.data_manu.RH_MD_SW_ver_major;
	meta.rh_md_sw_ver.minor = BLECtrlObj.data_manu.RH_MD_SW_ver_minor;
	meta.rh_md_sw_ver.patch = BLECtrlObj.data_manu.RH_MD_SW_ver_patch;

	meta.left_neutral_posture_angle = LH_Sagittal.ThighNeutralSagittalAngle;
	meta.right_neutral_posture_angle = RH_Sagittal.ThighNeutralSagittalAngle;
	meta.trunk_neutral_posture_angle = TrunkNeutralPostureBias;
	
	if (IOIF_ESP32_BT_IsConnected()){
//		snprintf(meta.tablet_ID, 	MAX_STR_LEN, BLECtrlObj.data.);
		meta.user_id = BLECtrlObj.data.UserID;
		meta.prouser_id = BLECtrlObj.data.ProUserID;
		meta.user_gender = (uint8_t) BLECtrlObj.data.User_Gender;
		meta.user_height = (uint16_t) BLECtrlObj.data.User_Height;
		meta.user_weight = (uint16_t) BLECtrlObj.data.User_Weight;
		// meta.user_footsize, BLECtrlObj.data.;
		meta.assist_mode = BLECtrlObj.data.AssistMode;
		
		// assist_param 초기화
		memset(meta.assist_param, 0, sizeof(meta.assist_param));
		
		// Assist mode별 parameter 설정
		switch (BLECtrlObj.data.AssistMode) {
			case SMART_ASSIST_MODE: // Mode 0
				meta.assist_param[0] = (float)BLECtrlObj.data.LeftHipExtensionTorque * 10.0f; // 0.01 N*m 단위
				meta.assist_param[1] = (float)BLECtrlObj.data.RightHipExtensionTorque * 10.0f;
				meta.assist_param[2] = (float)BLECtrlObj.data.LeftHipFlexionTorque * 10.0f;
				meta.assist_param[3] = (float)BLECtrlObj.data.RightHipFlexionTorque * 10.0f;
				meta.assist_param[4] = LeftHipSmartAssistGravityCompensation; // 계산된 값 사용
				meta.assist_param[5] = RightHipSmartAssistGravityCompensation;
				meta.assist_param[6] = (float)BLECtrlObj.data.DisableStairWalkingDetection;
				break;
				
			case AQUA_MODE: // Mode 1  
				meta.assist_param[0] = LeftHipAquaGravityCompensation; // 계산된 값 사용
				meta.assist_param[1] = RightHipAquaGravityCompensation;
				meta.assist_param[2] = LeftHipAquaVelocityCompensation;
				meta.assist_param[3] = RightHipAquaVelocityCompensation;
				break;
				
			case UNIVERSE_MODE: // Mode 2
				meta.assist_param[0] = (float)BLECtrlObj.data.LeftHipGravityCompensation;
				meta.assist_param[1] = (float)BLECtrlObj.data.RightHipGravityCompensation;
				break;
				
			case JOINT_LIMIT_MODE: // Mode 3
				meta.assist_param[0] = LeftHipFlexionLimit; // 계산된 값 사용 (값 - 15.0)
				meta.assist_param[1] = LeftHipExtensionLimit; // (값 + 15.0)
				meta.assist_param[2] = RightHipFlexionLimit;
				meta.assist_param[3] = RightHipExtensionLimit;
				meta.assist_param[4] = LeftHipFlexionVelocityLimit;
				meta.assist_param[5] = LeftHipExtensionVelocityLimit;
				meta.assist_param[6] = RightHipFlexionVelocityLimit;
				meta.assist_param[7] = RightHipExtensionVelocityLimit;
				break;
				
			default: // MANUAL_ASSIST_MODE 등 기타
				// assist_param은 이미 0으로 초기화됨
				break;
		}
	}

}

/* --------------------- SDO CALLBACK --------------------- */


#endif /* SUIT_MINICM_ENABLED */
