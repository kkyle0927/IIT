/*
 * gaitCtrlTask.c
 *
 *  Created on: Oct 11, 2023
 *      Author: INVINCIBLENESS
 */

#include "gait_ctrl.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskObj_t gaitCtrlTask;

#ifdef QUATERNION
// For Quaternion //
VQF_MagCalib_t vqfMagCalibObj;
VQF_t vqfObj;

float magValueCal[3];
VQF_Real_t vqfGyr[3];
VQF_Real_t vqfAcc[3];
VQF_Real_t vqfMag[3];
VQF_Real_t vqfQuat[4];
int16_t q_send[4];

float A_inv[3][3] = {  	{ 0.905e-03,  -0.0069e-03, -0.00083e-03},
						{-0.0069e-03,   0.971e-03,  -0.018e-03},
						{-0.00083e-03,   -0.0181e-03,  0.9629e-03}   };

float ironErr[3] = {0.635e+03, -0.703e+03, -1.42e+03};

float RealMagX;
float RealMagY;
float RealMagZ;

/* SPI communication(For KAIST) */
#define DATA_NUM 	6
float txFloatDataArray[DATA_NUM] = {0};
uint8_t txByteDataArray[DATA_NUM*4] = {0};

/////////////////////////////////////////////////////////////////////////////////
#endif /* QUATERNION */

// For Triggering //
WIDM_GaitData_t		widmGaitDataObj;
WIDM_AngleData_t 	widmAngleDataObj;
WIDM_AttachCase_t	widmAttachCaseObj;

#if defined(SUIT_MD_ENABLED) | defined(SUIT_MINICM_ENABLED)
// Prof - 11.02 version //
static uint32_t a = 0;
static uint32_t k = 0;
static uint16_t Period = 2000;
static uint32_t timeStampPrev = 0;
static double cutoffFreq = 6.0;
static double cutoffFreqSmooth = 6.0;
static float modeCheck = 0;
static double PeakAmp = 10;
static double PeakAmpSmooth = 10;
static double PeakWAmp = 70;
static double PeakWAmpSmooth = 70;
static double NeutralPosture = 0;
static uint8_t firstPeriodCheck = 1;
static double velLPF = 0;
static double degLPF0ABS = 0;

static uint8_t mode[2] = {0, 0};
static double velLPF2[2] = {0, 0};
static double velLPF0ABS = 0;
static double degLPF[2] = {0, 0};
static double gyroLPF[2] = {0, 0};
static double degBPF[3] = {0, 0, 0};		// center frequency BPF
static double degBPF0ABS = 0;
static double velBPF = 0;
static double velBPF0ABS = 0;
static double angleNorm = 0;
static double velocityNorm = 0;
static double degUnbiased[2] = {0, 0};
static double gaitPhase = 0;
static double gaitPhasePrev = 0;
uint32_t gaitCount = 0;

float prevIncDeg = 0.0;
float incDeg = 0.0;
float incDegTrig[3] = {0};
float prevIncVel = 0.0;
float incVel = 0.0;
float incVelTrig[3] = {0};

float filteredIncDeg = 0.0;
float filteredIncVel = 0.0;

uint8_t B1Flag = 0;	// walking
uint8_t B2Flag = 0; // 10%
uint8_t B3Flag = 0; // 20%
uint8_t B4Flag = 0; // 30%
uint8_t B5Flag = 0; // 40%
uint8_t B6Flag = 0; // 50%
uint8_t B7Flag = 0; // ~~ 0%  FB Transition
uint8_t B8Flag = 0; // ~~ 50% BF Transition
uint8_t B9Flag = 0; // ~~ 75% BF Moving
uint8_t B10Flag = 0; // ~~ 25% FB Moving
uint8_t B11Flag = 0; // 60%
uint8_t B12Flag = 0; // 70%
uint8_t B13Flag = 0; // 80%
uint8_t B14Flag = 0; // 90%
uint8_t B15Flag = 0; // B8 -> for extension incVel == 0
uint8_t B16Flag = 0;

uint8_t B17Flag = 0;
uint8_t B18Flag = 0;
uint8_t B19Flag = 0;
uint8_t B20Flag = 0;
uint8_t B21Flag = 0;


static uint8_t B7Finished = 0;
static uint8_t B8Finished = 0;

static uint8_t phaseThreshold1 = 10;
static uint8_t phaseThreshold2 = 20;
static uint8_t phaseThreshold3 = 30;
static uint8_t phaseThreshold4 = 40;
static uint8_t phaseThreshold5 = 50;
static uint8_t phaseThreshold6 = 60;
static uint8_t phaseThreshold7 = 88;
static uint8_t phaseThreshold8 = 80;
static uint8_t phaseThreshold9 = 90;

// H10 //
static double velLPF_H10[2] = {0, 0};


// For ISI check //
//static uint8_t B2_chk[ISIchkNum] = {0};
//static uint8_t B3_chk[ISIchkNum] = {0};
//static uint8_t B4_chk[ISIchkNum] = {0};
//static uint8_t B5_chk[ISIchkNum] = {0};
//static uint8_t B6_chk[ISIchkNum] = {0};
//static uint8_t B7_chk[ISIchkNum] = {0};
//static uint8_t B8_chk[ISIchkNum] = {0};
//static uint8_t B9_chk[ISIchkNum] = {0};
//static uint8_t B10_chk[ISIchkNum] = {0};

static uint8_t B2stack = 0;
static uint8_t B3stack = 0;
static uint8_t B4stack = 0;
static uint8_t B5stack = 0;
static uint8_t B6stack = 0;
static uint16_t B7stack = 0;
static uint16_t B8stack = 0;
static uint16_t B9stack = 0;
static uint16_t B10stack = 0;
static uint16_t B15stack = 0;

uint32_t RTBrokenFlag = 0;

static int chk1 = 0;
static int chk1Prev = 0;
static float incDegLPF[3] = {0};
static int B16_stack = 0;

#ifdef SUIT_MINICM_ENABLED
/* For SUIT series - MiniCM */
// RH //
static uint32_t a_RH = 0;
static uint32_t k_RH = 0;
static double velLPF_RH[2] = {0, 0};
static double velLPF0ABS_RH = 0;
static float modeCheck_RH = 0;
static double degLPF_RH[2] = {0, 0};
static uint8_t mode_RH[2] = {0, 0};
static double velLPF2_RH[2] = {0, 0};
static uint8_t firstPeriodCheck_RH = 1;
static uint16_t Period_RH = 1000;
static uint32_t timeStampPrev_RH = 0;
static double cutoffFreq_RH = 6.0;
static double cutoffFreqSmooth_RH = 6.0;
static double degBPF_RH[3] = {0, 0, 0};		// center frequency BPF
static double degBPF0ABS_RH = 0;
static double PeakAmp_RH = 10;
static double PeakAmpSmooth_RH = 10;
static double PeakWAmp_RH = 70;
static double PeakWAmpSmooth_RH = 70;
static double velBPF_RH = 0;
static double velBPF0ABS_RH = 0;
static double angleNorm_RH = 0;
static double velocityNorm_RH = 0;
static double gaitPhase_RH = 0;
static double gaitPhasePrev_RH = 0;
uint8_t B1Flag_RH = 0;	// walking
uint8_t B2Flag_RH = 0; // 10%
uint8_t B3Flag_RH = 0; // 20%
uint8_t B4Flag_RH = 0; // 30%
uint8_t B5Flag_RH = 0; // 40%
uint8_t B6Flag_RH = 0; // 50%
uint8_t B7Flag_RH = 0; // ~~ 0%  FB Transition
uint8_t B8Flag_RH = 0; // ~~ 50% BF Transition
uint8_t B9Flag_RH = 0; // ~~ 75% BF Moving
uint8_t B10Flag_RH = 0; // ~~ 25% FB Moving
uint8_t B11Flag_RH = 0; // 60%
uint8_t B12Flag_RH = 0; // 70%
uint8_t B13Flag_RH = 0; // 80%
uint8_t B14Flag_RH = 0; // 90%
static uint8_t phaseThreshold1_RH = 10;
static uint8_t phaseThreshold2_RH = 20;
static uint8_t phaseThreshold3_RH = 30;
static uint8_t phaseThreshold4_RH = 40;
static uint8_t phaseThreshold5_RH = 50;
static uint8_t phaseThreshold6_RH = 60;
static uint8_t phaseThreshold7_RH = 88;
static uint8_t phaseThreshold8_RH = 80;
static uint8_t phaseThreshold9_RH = 90;
static double degLPF0ABS_RH = 0;
static double NeutralPosture_RH = 0;
static double degUnbiased_RH[2] = {0, 0};
static uint8_t B7Finished_RH = 0;
static uint8_t B8Finished_RH = 0;
uint32_t gaitCount_RH = 0;


// LH //
static uint32_t a_LH = 0;
static uint32_t k_LH = 0;
static double velLPF_LH[2] = {0, 0};
static double velLPF0ABS_LH = 0;
static float modeCheck_LH = 0;
static double degLPF_LH[2] = {0, 0};
static uint8_t mode_LH[2] = {0, 0};
static double velLPF2_LH[2] = {0, 0};
static uint8_t firstPeriodCheck_LH = 1;
static uint16_t Period_LH = 1000;
static uint32_t timeStampPrev_LH = 0;
static double cutoffFreq_LH = 6.0;
static double cutoffFreqSmooth_LH = 6.0;
static double degBPF_LH[3] = {0, 0, 0};		// center frequency BPF
static double degBPF0ABS_LH = 0;
static double PeakAmp_LH = 10;
static double PeakAmpSmooth_LH = 10;
static double PeakWAmp_LH = 70;
static double PeakWAmpSmooth_LH = 70;
static double velBPF_LH = 0;
static double velBPF0ABS_LH = 0;
static double angleNorm_LH = 0;
static double velocityNorm_LH = 0;
static double gaitPhase_LH = 0;
static double gaitPhasePrev_LH = 0;
uint8_t B1Flag_LH = 0;	// walking
uint8_t B2Flag_LH = 0; // 10%
uint8_t B3Flag_LH = 0; // 20%
uint8_t B4Flag_LH = 0; // 30%
uint8_t B5Flag_LH = 0; // 40%
uint8_t B6Flag_LH = 0; // 50%
uint8_t B7Flag_LH = 0; // ~~ 0%  FB Transition
uint8_t B8Flag_LH = 0; // ~~ 50% BF Transition
uint8_t B9Flag_LH = 0; // ~~ 75% BF Moving
uint8_t B10Flag_LH = 0; // ~~ 25% FB Moving
uint8_t B11Flag_LH = 0; // 60%
uint8_t B12Flag_LH = 0; // 70%
uint8_t B13Flag_LH = 0; // 80%
uint8_t B14Flag_LH = 0; // 90%
static uint8_t phaseThreshold1_LH = 10;
static uint8_t phaseThreshold2_LH = 20;
static uint8_t phaseThreshold3_LH = 30;
static uint8_t phaseThreshold4_LH = 40;
static uint8_t phaseThreshold5_LH = 50;
static uint8_t phaseThreshold6_LH = 60;
static uint8_t phaseThreshold7_LH = 88;
static uint8_t phaseThreshold8_LH = 80;
static uint8_t phaseThreshold9_LH = 90;
static double degLPF0ABS_LH = 0;
static double NeutralPosture_LH = 0;
static double degUnbiased_LH[2] = {0, 0};
static uint8_t B7Finished_LH = 0;
static uint8_t B8Finished_LH = 0;
uint32_t gaitCount_LH = 0;
#endif

////////////////////////////////////////////////////////////////////////////
#endif

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static IOIF_6AxisData_t 	imu6AxisDataObj;
static IOIF_MagData_t 		magDataObj;

static WIDM_SensorData_t 	widmSensorDataObj;
static WIDM_FuzzyData_t		widmFuzzyDataObj;
static WIDM_NormData_t 		widmNormDataObj;
static WIDM_ThresData_t		widmThresDataObj;
static WIDM_Module_t		widmModuleObj;

// Loop Time Count //
static uint32_t gaitCtrlLoopCnt;
static float gaitCtrlTimeElap;

// For Debug //
static bool IsRecoverI2C1 = false;
static bool IsRecoverI2C2 = false;
static bool IsRecoverI2C3 = false;
static bool IsRecoverI2C4 = false;

static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;
static float wcDebug 			= 0.0;

static float deg_diff;
static float vel_diff;

//static uint8_t absOffsetCmd = 0;
uint8_t gaitCtrlState = 0;

static uint16_t stopCnt = 0;

// SELECTION !!!  For ALL Model //
static WIDM_AttachCase_t 	ATTACH_CASE_SEL;
static WIDM_Module_t	    MODULE_SEL;
static uint8_t 				WIDM_MODULE_NODE_ID;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- ROUTINE ------------------- */
static int RunGetIMUFunction(void);
static int RunTotalGaitFunction(void);
// Prof //
static int RunTotalGaitFunction_Prof(void);
static int RunTotalGaitFunction_1NE_0115(void);
static void CheckWalkingState_Prof(void);
static void GaitFunction_Prof_1029_K10(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData);
static void GaitFunction_Prof_1029_K10_Revised(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData);
static void GaitFunction_Prof_1029_H10(WIDM_AngleData_t* widmAngleData);
static void GaitFunction_Prof_1029_H10_revised(WIDM_AngleData_t* widmAngleData);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Functions for ALL cases(General) */
static int NodeIDCheck(uint8_t directionSet);
static void ModelSelection(uint8_t nodeID);
static void InitializeIMU(void);
static void ResetDataObj(void);
static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule);
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, float initialAngle);
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);
static void GetInitialAngle(WIDM_Module_t widmModule);
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase);
static void SetUsedDegVel(WIDM_AngleData_t* widmAngleData, WIDM_Module_t widmModule);
static void NoiseReduction(WIDM_AngleData_t* widmAngleData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule);
static void GetGaitPhase(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData);
static float GetPhaseRadius(float degDiff, float degTh, float velDiff, float velTh);
static void UpdateWalkingState(WIDM_GaitData_t* widmGaitData, float phaseRadiusStart, float phaseRadiusStop, int16_t sumIter);
static void CheckWalkingState(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_ThresData_t* widmThresData, WIDM_GaitData_t* widmGaitData);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* Functions for only SUIT MINICM */
#ifdef SUIT_MINICM_ENABLED
static void GaitFunction_Prof_1029_K10_MiniCM(float degFinal, float velFinal, float gyrZ, float degINC, float velINC);
static void GaitFunction_Prof_1029_H10_MiniCM(float degFinal, float velFinal);
static void GaitFunction_Prof_1029_RH10_MiniCM(float degFinal, float velFinal);
static void GaitFunction_Prof_1029_LH10_MiniCM(float degFinal, float velFinal);
static int RunTotalGaitFunction_SUIT_MiniCM(uint8_t RH_ON, uint8_t LH_ON, uint8_t RK_ON, uint8_t LK_ON);
#endif

/* Functions for only SUIT MD */
#ifdef SUIT_MD_ENABLED
static void Create_GaitDataPDO(uint8_t nodeID);
static void Create_BFlagPDO(uint8_t nodeID);
static int RunTotalGaitFunction_SUIT_MD(void);
#endif

/* Functions for only MD */
#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED) || defined(SUIT_MD_ENABLED)
static void GetABSLinkData(IOIF_AbsEnc_t* absEnc, WIDM_AngleData_t* widmAngleData);
static void GetINCLinkData(WIDM_AngleData_t* widmAngleData);

#ifdef IMUABS_MODE
static void CompensateIMUABS(WIDM_AngleData_t* widmAngleData);
static void GetInitialAngle_IMUABS(IOIF_6AxisData_t* imu6AxisData, IOIF_AbsEnc_t* absEnc, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);
#endif

#ifdef IMUINC_MODE
static void CompensateIMUINC(WIDM_AngleData_t* widmAngleData);
static void GetInitialAngle_IMUINC(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase);
#endif

#endif

/* Functions for only WIDM */
#if defined(WIDM_ENABLED)

#endif


/* Functions for [IMU + Absolute encdoer] mode */
#ifdef IMUABS_MODE

#endif /* SAM_IMUABS_MODE */


/* Functions for [IMU + Incremental encoder] mode */
#ifdef IMUINC_MODE

#endif /* SAM_IMUINC_MODE */



#ifdef QUATERNION
// Quaternion //
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void InitMagInfo(void);
static int EntGetQuaternion(void);
static int RunGetQuaternion(void);
static uint8_t TransmitData_SPI(void);
static void floatToBytes(float value, uint8_t* bytes);
static void floatArrayToByteArray(float* floatDataArray, uint8_t* byteDataArray, uint8_t dataNum);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif


/* ------------------- SDO CALLBACK ------------------- */
//static void SetAbsOffsetCmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);

DOP_COMMON_SDO_CB(gaitCtrlTask)

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void InitGaitCtrl(void)
{
    InitTask(&gaitCtrlTask);

    /* Checking Node ID */
    WIDM_MODULE_NODE_ID = NodeIDCheck(0);		// 0:LEFT, 1:RIGHT

#ifdef CM_MODULE

	/* Assign Device */
	COMMON_AssignDevice(&gaitCtrlTask, DEV_IDX_GAIT_CTRL);
	gaitCtrlTask.period = 1;

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, State_Off,      NULL,   			StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, State_Standby,  NULL,   			StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, State_Enable,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, State_Error,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_IDX_GAIT_CTRL);

   	// PDO
	DOP_COMMON_PDO_CREATE(TASK_IDX_GAIT_CTRL, gaitCtrlTask);
	// DOP_CreatePDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_IDX_GAIT_CTRL)
	// DOP_CreateSDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, SDOCallback);
#endif /* CM_MODULE */

#ifdef MD_MODULE
	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Off,      NULL,   				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Standby,  NULL,   				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Enable,   StateEnable_Ent,		StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Error,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_IMU_6AXIS_GETVALUE, 		NULL, IMU_6Axis_GetValue, 		NULL);

	/* DOD Definition */
	// DOD
	Create_DOD(TASK_ID_WIDM);

	// PDO
	// Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_X, 	 		 DOP_FLOAT32,   	1,    &widmAngleDataObj.degFinal);
	// Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_Y, 	 		 DOP_FLOAT32,   	1,    &widmAngleDataObj.velFinal);

#ifdef SUIT_MD_ENABLED
	/* For MD_MODULE PDO setting */
	// Create_GaitDataPDO(WIDM_MODULE_NODE_ID);
	 Create_BFlagPDO(WIDM_MODULE_NODE_ID);
#endif
	// Quaternion //
	// Create_PDO(TASK_ID_WIDM, PDO_ID_QUATERNION,          DOP_INT16,   4, &q_send);
	// Create_SDO(TASK_ID_WIDM, SDO_ID_IMU_MAG_INVA,        DOP_FLOAT32, SetMagInvAInfo);
	// Create_SDO(TASK_ID_WIDM, SDO_ID_IMU_MAG_IRON_ERROR,  DOP_FLOAT32, SetMagIronErrorInfo);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	// SDO
	MSG_COMMON_SDO_CREATE(TASK_ID_WIDM)

	// Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ABSOFFSET_CMD, 	DOP_UINT8, 	SetAbsOffsetCmd);
#endif /* MD_MODULE & WIDM_MODULE */

#ifdef WIDM_MODULE
	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Off,      NULL,   				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Standby,  NULL,   				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Enable,   StateEnable_Ent,		StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, e_State_Error,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_IMU_6AXIS_GETVALUE, 		NULL, IMU_6Axis_GetValue, 		NULL);

	/* DOD Definition */
	// DOD
	Create_DOD(TASK_ID_WIDM);

	// PDO
	// Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_X, 	 		 DOP_FLOAT32,   	1,    &widmAngleDataObj.degFinal);
	// Create_PDO(TASK_ID_WIDM,	 PDO_ID_WIDM_ACC_Y, 	 		 DOP_FLOAT32,   	1,    &widmAngleDataObj.velFinal);

	/* For MD_MODULE PDO setting */
	// Create_GaitDataPDO(WIDM_MODULE_NODE_ID);
	// Create_BFlagPDO(WIDM_MODULE_NODE_ID);

	// Quaternion //
	// Create_PDO(TASK_ID_WIDM, PDO_ID_QUATERNION,          DOP_INT16,   4, &q_send);
	// Create_SDO(TASK_ID_WIDM, SDO_ID_IMU_MAG_INVA,        DOP_FLOAT32, SetMagInvAInfo);
	// Create_SDO(TASK_ID_WIDM, SDO_ID_IMU_MAG_IRON_ERROR,  DOP_FLOAT32, SetMagIronErrorInfo);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////

	// SDO
	MSG_COMMON_SDO_CREATE(TASK_ID_WIDM)

	// Create_SDO(TASK_ID_WIDM, 	SDO_ID_WIDM_SET_ABSOFFSET_CMD, 	DOP_UINT8, 	SetAbsOffsetCmd);
#endif /* MD_MODULE & WIDM_MODULE */

	// !!! Select correct WIDM Module here //
	ModelSelection(WIDM_MODULE_NODE_ID);
	widmAttachCaseObj = ATTACH_CASE_SEL;
	widmModuleObj = MODULE_SEL;

	/* Init 6axis & 3axis IMU */
	InitializeIMU();

	/* Initial stage of get angle */
	ResetDataObj();
	InitValueSetting(&widmFuzzyDataObj, &widmNormDataObj, &widmGaitDataObj, &widmThresDataObj, widmModuleObj);
	GetInitialAngle(widmModuleObj);

#ifdef QUATERNION
	// Quaternion //
	InitMagInfo();
	/////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

#ifdef _USE_BAREMETAL
	/* Timer Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM2) > 0){
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM2, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunGaitCtrl, NULL);
#endif
}

#if defined(MD_MODULE) || defined(WIDM_MODULE)
void RunGaitCtrl(void* params)
#endif
#ifdef CM_MODULE
void RunGaitCtrl(void)
#endif
{
#ifdef _USE_OS_RTOS
	RunTask(&gaitCtrlTask);
#else /* _USE_BAREMETAL */

	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	Run_Task(&gaitCtrlTask);

	/* Elapsed Time Check */
#ifndef WIDM_ENABLED
	gaitCtrlTimeElap = DWT->CYCCNT / 480;	// in microsecond
#else
	gaitCtrlTimeElap = DWT->CYCCNT / 160;	// in microsecond
#endif

#endif /* _USE_OS_RTOS */
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	gaitCtrlState = 0;
#ifdef CM_MODULE
	Transition_State(&gaitCtrlTask.state_machine, State_Standby);
#else
	Transition_State(&gaitCtrlTask.state_machine, e_State_Standby);
#endif
}

static void StateStandby_Run(void)
{
	gaitCtrlState = 1;
#ifdef CM_MODULE
	Transition_State(&gaitCtrlTask.state_machine, State_Enable);
#else
	Transition_State(&gaitCtrlTask.state_machine, e_State_Enable);
#endif

}

static void StateEnable_Ent(void)
{
	gaitCtrlLoopCnt = 0;

#ifdef CM_MODULE

#endif

#ifdef SUIT_MD_ENABLED

#endif

#ifdef WIDM_MODULE
	EntGetQuaternion();
#endif

	Ent_Routines(&gaitCtrlTask.routine);
}
float accTot = 0;
static void StateEnable_Run(void)
{
	gaitCtrlState = 2;

	/* Select the function */
#ifdef CM_MODULE
//	RunTotalGaitFunction();
	RunGetIMUFunction();
//	RunTotalGaitFunction_SUIT_MiniCM(0, 1, 0, 0);		// For Suit H10 & K10 - MiniCM version	(RH, LH, RK, LK)
#endif

#ifdef SUIT_MD_ENABLED
//	RunTotalGaitFunction();
//	RunGetIMUFunction();
//	RunTotalGaitFunction_Prof();						// For Suit H10 & K10 - MD self version
//	RunTotalGaitFunction_SUIT_MD();						// For Suit H10 & K10 - MD just get Gait Data PDO version

	RunTotalGaitFunction_1NE_0115();
#endif

#ifdef WIDM_MODULE
//	RunTotalGaitFunction();
//	RunGetIMUFunction();
	RunGetQuaternion();
#endif

	/* For SPI Communication (For KAIST) */
//	TransmitData_SPI();


	Run_Routines(&gaitCtrlTask.routine);

//	TaskDelay(&gaitCtrlTask);
	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    gaitCtrlLoopCnt = 0;
    Ext_Routines(&gaitCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* ------------------- ROUTINE ------------------- */


/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	testImu3AxisRes = IOIF_Magneto_GetValue(&magDataObj);

	return 0;
}


/* Old version of WIDM algorithm */
static int RunTotalGaitFunction(void)
{
	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);

	uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
		return t6AxisRes;
	}

	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);

	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);

	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);

#ifdef IMUABS_MODE
	// For IMU + Encoder(ABS or INC) case //
	if (widmModuleObj == WIDM_IMUABS_SAM){
		// [Get Angle by using ABS] //
		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
		// [Combining the results of IMU & ABS] //
		CompensateIMUABS(&widmAngleDataObj);
	}
#endif /* IMUABS_MODE */

#ifdef IMUINC_MODE
	if (widmModuleObj == WIDM_IMUINC_SAM){
		// [Get Angle by using INC] //
		GetINCLinkData(&widmAngleDataObj);
		// [Combining the results of IMU & INC] //
		CompensateIMUINC(&widmAngleDataObj);
	}
#endif /* IMUINC_MODE */

	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);

	NoiseReduction(&widmAngleDataObj, &widmGaitDataObj, widmModuleObj);

	GetGaitPhase(&widmAngleDataObj, &widmNormDataObj, &widmGaitDataObj);

	CheckWalkingState(&widmAngleDataObj, &widmNormDataObj, &widmThresDataObj, &widmGaitDataObj);

	if (widmGaitDataObj.gaitPeriod > 3000){
		widmGaitDataObj.gaitPeriod = 3000;
	}

	return 0;
}


/* Prof version */
static int RunTotalGaitFunction_Prof(void)
{
	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);

	uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
		return t6AxisRes;
	}

	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);

	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);

	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);

#ifdef IMUABS_MODE
	// For IMU + Encoder(ABS or INC) case //
	if (widmModuleObj == WIDM_IMUABS_SAM){
		// [Get Angle by using ABS] //
		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
		// [Combining the results of IMU & ABS] //
		CompensateIMUABS(&widmAngleDataObj);
	}
#endif /* IMUABS_MODE */

#ifdef IMUINC_MODE
	if (widmModuleObj == WIDM_IMUINC_SAM){
		// [Get Angle by using INC] //
		GetINCLinkData(&widmAngleDataObj);
		// [Combining the results of IMU & INC] //
		CompensateIMUINC(&widmAngleDataObj);
	}
#endif /* IMUINC_MODE */

	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);

	if (WIDM_MODULE_NODE_ID == NODE_ID_LK || WIDM_MODULE_NODE_ID == NODE_ID_RK){
//		GaitFunction_Prof_1029_K10(&widmAngleDataObj, &widmSensorDataObj);
		GaitFunction_Prof_1029_K10_Revised(&widmAngleDataObj, &widmSensorDataObj);
	}
	else if (WIDM_MODULE_NODE_ID == NODE_ID_LH_SAG || WIDM_MODULE_NODE_ID == NODE_ID_RH_SAG){
//		GaitFunction_Prof_1029_H10(&widmAngleDataObj);
		GaitFunction_Prof_1029_H10_revised(&widmAngleDataObj);
	}


//	CheckWalkingState_Prof();

	// Convert Data format //
//	saveDegRaw = widmAngleDataObj.degFinal * 100;
//	saveVelRaw = widmAngleDataObj.velFinal * 10;
//	savePeriodCheck = cutoffFreqSmooth * 1000;
//	saveGyrZ = widmSensorDataObj.gyrZ[0] * 10;
//	saveDegNorm = angleNorm * 1000;

	return 0;
}

/* Prof version */
static int RunTotalGaitFunction_1NE_0115(void)
{
	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);

	uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
		return t6AxisRes;
	}

	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);

	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);

	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);

#ifdef IMUABS_MODE
	// For IMU + Encoder(ABS or INC) case //
	if (widmModuleObj == WIDM_IMUABS_SAM){
		// [Get Angle by using ABS] //
		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
		// [Combining the results of IMU & ABS] //
		CompensateIMUABS(&widmAngleDataObj);
	}
#endif /* IMUABS_MODE */

#ifdef IMUINC_MODE
	if (widmModuleObj == WIDM_IMUINC_SAM){
		// [Get Angle by using INC] //
		GetINCLinkData(&widmAngleDataObj);
		// [Combining the results of IMU & INC] //
		CompensateIMUINC(&widmAngleDataObj);
	}
#endif /* IMUINC_MODE */

	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);


	/* Old algorithm */
	NoiseReduction(&widmAngleDataObj, &widmGaitDataObj, widmModuleObj);

	GetGaitPhase(&widmAngleDataObj, &widmNormDataObj, &widmGaitDataObj);

	CheckWalkingState(&widmAngleDataObj, &widmNormDataObj, &widmThresDataObj, &widmGaitDataObj);

	if (widmGaitDataObj.gaitPeriod > 2000){
		widmGaitDataObj.gaitPeriod = 2000;
	}


	/* Prof algorithm */
	if (WIDM_MODULE_NODE_ID == NODE_ID_LK || WIDM_MODULE_NODE_ID == NODE_ID_RK){
		GaitFunction_Prof_1029_K10_Revised(&widmAngleDataObj, &widmSensorDataObj);
	}
	else if (WIDM_MODULE_NODE_ID == NODE_ID_LH_SAG || WIDM_MODULE_NODE_ID == NODE_ID_RH_SAG){
		GaitFunction_Prof_1029_H10_revised(&widmAngleDataObj);
	}


	return 0;
}

#ifdef SUIT_MINICM_ENABLED
static int RunTotalGaitFunction_SUIT_MiniCM(uint8_t RH_ON, uint8_t LH_ON, uint8_t RK_ON, uint8_t LK_ON)
{
	if (RH_ON == 1){
		GaitFunction_Prof_1029_RH10_MiniCM(widmDegFinal_RH, widmVelFinal_RH);
	}
	if (LH_ON == 1){
		GaitFunction_Prof_1029_LH10_MiniCM(widmDegFinal_LH, widmVelFinal_LH);
	}
	if (RK_ON == 1){
		GaitFunction_Prof_1029_K10_MiniCM(widmDegFinal_RK, widmVelFinal_RK, widmGyrZ_RK, widmDegINC_RK, widmVelINC_RK);
	}
	if (LK_ON == 1){
		GaitFunction_Prof_1029_K10_MiniCM(widmDegFinal_LK, widmVelFinal_LK, widmGyrZ_LK, widmDegINC_LK, widmVelINC_LK);
	}

	return 0;
}
#endif


#ifdef SUIT_MD_ENABLED
/* SUIT [MD] - Get Gait Data (PDO) */
static int RunTotalGaitFunction_SUIT_MD(void)
{
	WIDM_UpdateBuffer(&widmSensorDataObj, &widmAngleDataObj, &widmGaitDataObj, &widmNormDataObj);

	uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
		return t6AxisRes;
	}

	UpdateSensorRawData(&widmSensorDataObj, &imu6AxisDataObj, widmAttachCaseObj);

	WIDM_CalculateFuzzyInput(&widmSensorDataObj, &widmFuzzyDataObj);
	wcDebug = WIDM_CalculateFuzzyWc(&widmFuzzyDataObj);

	RunTvcfFilter(&widmSensorDataObj, &widmAngleDataObj, &widmFuzzyDataObj, WIDM_CONTROL_PERIOD, widmAttachCaseObj);

#ifdef IMUABS_MODE
	// For IMU + Encoder(ABS or INC) case //
	if (widmModuleObj == WIDM_IMUABS_SAM){
		// [Get Angle by using ABS] //
		GetABSLinkData(&AbsObj1, &widmAngleDataObj);
		// [Combining the results of IMU & ABS] //
		CompensateIMUABS(&widmAngleDataObj);
	}
#endif /* IMUABS_MODE */

#ifdef IMUINC_MODE
	if (widmModuleObj == WIDM_IMUINC_SAM){
		// [Get Angle by using INC] //
		GetINCLinkData(&widmAngleDataObj);
		// [Combining the results of IMU & INC] //
		CompensateIMUINC(&widmAngleDataObj);
	}
#endif /* IMUINC_MODE */

	SetUsedDegVel(&widmAngleDataObj, widmModuleObj);

	return 0;
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Functions for ALL cases(General) */

/* Module & Mode Selection Functions */
static int NodeIDCheck(uint8_t directionSet)
{
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED) || defined(SUIT_MINICM_ENABLED)
	int nodeID = 1;
	return nodeID;
#endif /* WALKON5_CM_ENABLED & L30_CM_ENABLED & SUIT_MINICM_ENABLED */

#if defined(WALKON5_MD_ENABLED) || defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* WALKON5_MD_ENABLED & L30_MD_REV06_ENABLED & L30_MD_REV07_ENABLED & L30_MD_REV08_ENABLED */

#if defined(SUIT_MD_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* SUIT_MD_ENABLED */

#ifdef WIDM_ENABLED
	int nodeID = 0;
	if (directionSet == 0){			// RIGHT WIDM
		nodeID = 14;
	}
	else if (directionSet == 1){	// LEFT WIDM
		nodeID = 15;
	}
	return nodeID;
#endif /* WIDM_ENABLED */
}

/* Select the Joint and Sensor method */
static void ModelSelection(uint8_t nodeID)
{
	switch (nodeID){
		case (NODE_ID_CM):
			ATTACH_CASE_SEL   = WIDM_CM_TEST;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
		case (NODE_ID_LH_SAG):	// LH
			ATTACH_CASE_SEL   = WIDM_H10_LEFT;
			MODULE_SEL 		  = WIDM_IMUINC_SAM;
			break;
		case (NODE_ID_RH_SAG):	// RH
			ATTACH_CASE_SEL   = WIDM_H10_RIGHT;
			MODULE_SEL 		  = WIDM_IMUINC_SAM;
			break;
		case (NODE_ID_LK):		// LK
			ATTACH_CASE_SEL   = WIDM_K10_LEFT;
			MODULE_SEL 		  = WIDM_IMU_SAM;
			break;
		case (NODE_ID_RK):		// RK
			ATTACH_CASE_SEL   = WIDM_K10_RIGHT;
			MODULE_SEL 		  = WIDM_IMU_SAM;
			break;
		case (NODE_ID_WIDM_R):
			ATTACH_CASE_SEL   = WIDM_RIGHT_U5;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
		case (NODE_ID_WIDM_L):
			ATTACH_CASE_SEL   = WIDM_LEFT_U5;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
		default:
			ATTACH_CASE_SEL   = WIDM_CM_TEST;
			MODULE_SEL 		  = WIDM_IMU_U5;
			break;
	}
}

/* Initialize 6Axis & 3Axis IMU */
static void InitializeIMU(void)
{

#ifdef SUIT_MINICM_ENABLED
	testImu6AxisRes = IOIF_AccGyro_Init(IOIF_I2C1);
	if (testImu6AxisRes != IOIF_ACCGYRO_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C1_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c1);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_MspDeInit(&hi2c1);

		// GPIOB, 6X_IMU_I2C_SDA(GPIO_PIN_7), 6X_IMU_I2C_SCL(GPIO_PIN_6)
		HAL_I2C_BusReset(&hi2c1, GPIOB, IOIF_GPIO_PIN_7, GPIOB, IOIF_GPIO_PIN_6);

		HAL_I2C_Init(&hi2c1);
		HAL_I2C_MspInit(&hi2c1);
		__HAL_I2C_ENABLE(&hi2c1);
		__HAL_RCC_I2C1_CLK_ENABLE();

		IsRecoverI2C1 = true;

		testImu6AxisRes = IOIF_AccGyro_Init(IOIF_I2C1);
	}

	if (testImu6AxisRes == IOIF_ACCGYRO_STATUS_OK) {

	}

	testImu3AxisRes = IOIF_Magneto_Init(IOIF_I2C2);
	if (testImu3AxisRes != IOIF_MAGNETO_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C2_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c2);
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_MspDeInit(&hi2c2);

		// GPIOB, 3X_IMU_SDA(GPIO_PIN_11), 3X_IMU_SCL(GPIO_PIN_10)
		HAL_I2C_BusReset(&hi2c2, GPIOB, IOIF_GPIO_PIN_11, GPIOB, IOIF_GPIO_PIN_10);

		HAL_I2C_Init(&hi2c2);
		HAL_I2C_MspInit(&hi2c2);
		__HAL_I2C_ENABLE(&hi2c2);
		__HAL_RCC_I2C2_CLK_ENABLE();

		IsRecoverI2C2 = true;

		testImu3AxisRes = IOIF_Magneto_Init(IOIF_I2C2);
	}

	if (testImu3AxisRes == IOIF_MAGNETO_STATUS_OK) {

	}
#endif /* SUIT_MINICM_ENABLED */


}


/* Setting for Initial values of "angle" variables after get initial thigh angle */
static void SetInitialAngle(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, float initialAngle)
{
	widmAngleData->degAccFiltered 	= initialAngle;
	widmAngleData->degGyrFiltered	= 0.0;

	widmAngleData->degLPF1st[0] 	= initialAngle;
	widmAngleData->degLPF1st[1] 	= initialAngle;
	widmAngleData->degLPF2nd[0] 	= initialAngle;
	widmAngleData->degLPF2nd[1] 	= initialAngle;
	widmNormData->degOri 	 		= initialAngle;
}

/*
 *Function to calculate the initial thigh angle - IMU case
*/
static void GetInitialAngle_IMU(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 	= 100;
	uint8_t tDataCheck_IMU	= 0;
	uint8_t tRealSamples	= 0;
	float tAccumulatedAngle = 0.0;
	float tInitThighAngle	= 0.0;
	float tImuAngle			= 0.0;

	for (uint8_t i = 1; i <= tTotalSamples; i++){
        for (uint16_t j = 0; j < 30000; j++){
        	// For delay of DMA reading
        }

        tDataCheck_IMU = IOIF_AccGyro_GetValue(imu6AxisData);

        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

            tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
            tAccumulatedAngle += tImuAngle;

            tRealSamples++;
        }
    }

	tInitThighAngle = tAccumulatedAngle / tRealSamples;
	widmAngleData->initAngle = tInitThighAngle;
    SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}

static void GetInitialAngle(WIDM_Module_t widmModule)
{
#ifdef IMU_MODE
	/* [IMU only] case */
	if (widmModule == WIDM_IMU_U5 || widmModule == WIDM_IMU_SAM){
		GetInitialAngle_IMU(&imu6AxisDataObj, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
#endif /* IMU_MODE */

	/* [IMU + ABS] case */
#ifdef IMUABS_MODE
	if (widmModule == WIDM_IMUABS_SAM){
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);		// 3times -> Prevent the case that it is not properly offset
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);
		IOIF_SetAbsOffset(IOIF_SPI1, &AbsObj1);
		GetInitialAngle_IMUABS(&imu6AxisDataObj, &AbsObj1, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
#endif /* IMUABS_MODE */

	/* [IMU + INC] case */
#ifdef IMUINC_MODE
	if (widmModule == WIDM_IMUINC_SAM){
		// Maybe Incremental encoder's initial offset is completed in "low_level_ctrl_task".
		GetInitialAngle_IMUINC(&imu6AxisDataObj, &widmSensorDataObj, &widmAngleDataObj, &widmNormDataObj, widmAttachCaseObj);
	}
#endif /* IMUINC_MODE */
}

/* Reset Value Zero */
static void ResetDataObj(void)
{
	widmSensorDataObj 		= 	(WIDM_SensorData_t){0};
	widmFuzzyDataObj 		= 	(WIDM_FuzzyData_t){0};
	widmAngleDataObj		=   (WIDM_AngleData_t){0};
	widmNormDataObj 		= 	(WIDM_NormData_t){0};
	widmGaitDataObj 		= 	(WIDM_GaitData_t){0};
	widmThresDataObj 		= 	(WIDM_ThresData_t){0};

	wcDebug					= 	0.0;
}

/* Setting for initial parameters */
static void InitValueSetting(WIDM_FuzzyData_t* widmFuzzyData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData, WIDM_ThresData_t* widmThresData, WIDM_Module_t widmModule)
{
	widmFuzzyData->wl 				= 0.5;
	widmFuzzyData->wh 				= 10.0;
	widmFuzzyData->var[0] 			= 8.0;
	widmFuzzyData->var[1] 			= 30.0;
	widmFuzzyData->var[2] 			= 5.8;
	widmFuzzyData->var[3] 			= 320.0;

	widmNormData->ampDeg 			= 30.0; 	//30
	widmNormData->ampDegLPF[0]		= 30.0;
	widmNormData->ampDegLPF[1]		= 30.0;

	widmNormData->ampVel 			= 400.0; 	//400
	widmNormData->ampVelLPF[0]		= 400.0;
	widmNormData->ampVelLPF[1]		= 400.0;


	widmGaitData->gaitPeriod 		= 1000;
	widmGaitData->gaitPhase 		= -100.0;
	widmGaitData->gaitPhasePre   	= -100.0;


	if (widmModule == WIDM_IMU_U5){
		widmThresData->degThStart	= 5.0;
		widmThresData->velThStart	= 20.0;
		widmThresData->degThStop 	= 5.0;
		widmThresData->velThStop 	= 3.0;
	}
	else if (widmModule == WIDM_IMU_SAM){
		widmThresData->degThStart	= 12.0;
		widmThresData->velThStart	= 20.0;
		widmThresData->degThStop 	= 3.0;
		widmThresData->velThStop 	= 12.0;
	}
	else if (widmModule == WIDM_IMUABS_SAM){
		widmThresData->degThStart	= 10.0;		//15.0
		widmThresData->velThStart	= 40.0;		//60.0
		widmThresData->degThStop 	= 8.0;		//5.0
		widmThresData->velThStop 	= 10.0;		//25.0
	}
	else if (widmModule == WIDM_IMUINC_SAM){
		widmThresData->degThStart	= 12.0;		//15.0
		widmThresData->velThStart	= 65.0;		//60.0
		widmThresData->degThStop 	= 8.0;		//5.0
		widmThresData->velThStop 	= 40.0;		//25.0
	}
}

/*
*The function UpdateSensorRawData updates the IMU raw values.
*/
static void UpdateSensorRawData(WIDM_SensorData_t* widmSensorData, IOIF_6AxisData_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	widmSensorData->accX[0] = imu6AxisData->accX;
	widmSensorData->accY[0] = imu6AxisData->accY;
	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

	if (widmAttachCase < 8){
		widmSensorData->gyrZ[0] = (-1) * (imu6AxisData->gyrZ); 	// For Negative Gyro case (Maybe LEFT case)
	}
	else if (widmAttachCase >= 8){
		widmSensorData->gyrZ[0] = imu6AxisData->gyrZ; 			// For Positive Gyro case (Maybe RIGHT case)
	}
}

#if defined(SUIT_MD_ENABLED)
/*
*The function GetABSLinkData considers the Absolute Encoder's raw values.
*/
static void GetABSLinkData(IOIF_AbsEnc_t* absEnc, WIDM_AngleData_t* widmAngleData)
{
	// Left Leg //
	if (widmAttachCaseObj < 8){
		widmAngleData->degABS 	= absEnc->posDegMultiTurn * 90 / 168.75;
		widmAngleData->velABS   = absEnc->velDeg * 90 / 168.75;
	}

	// Right Leg //
	else if (widmAttachCaseObj >= 8){
		widmAngleData->degABS 	= absEnc->posDegMultiTurn * 90 / 168.75 * (-1);
		widmAngleData->velABS   = absEnc->velDeg * 90 / 168.75 * (-1);
	}
}

#ifdef IMUABS_MODE
/*
 *Function to compensate IMU+ABS case
*/
static void CompensateIMUABS(WIDM_AngleData_t* widmAngleData)
{
	widmAngleData->degIMUABS = widmAngleData->degTvcf[0] + widmAngleData->degABS;
	widmAngleData->velIMUABS = widmAngleData->velRaw[0]  + widmAngleData->velABS;
}

/*
 *Function to calculate the initial thigh angle - IMU + ABS case
*/
static void GetInitialAngle_IMUABS(IOIF_6AxisData_t* imu6AxisData, IOIF_AbsEnc_t* absEnc, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 			= 100;
	uint8_t tDataCheck_IMU 			= 0;
	uint8_t tRealSamples			= 0;
	float tAccumulatedAngle_IMU 	= 0.0;
	float tAccumulatedAngle_ABS 	= 0.0;
	float tAccumulatedAngle_IMUABS 	= 0.0;

	float tInitAngle_IMU 			= 0.0;
	float tInitAngle_ABS 			= 0.0;
	float tInitAngle_IMUABS 		= 0.0;
	float tInitThighAngle			= 0.0;


	for (uint8_t i = 1; i <= tTotalSamples; i++){
		for (int j = 0; j < 100000; j++){
			// For Delay of DMA reading
		}
        tDataCheck_IMU = IOIF_AccGyro_GetValue(imu6AxisData);
        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

        	float tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
        	tAccumulatedAngle_IMU += tImuAngle;

        	GetABSLinkData(absEnc, widmAngleData);
        	tAccumulatedAngle_ABS += widmAngleData->degABS;
            tAccumulatedAngle_IMUABS += (tImuAngle + widmAngleData->degABS);

        	tRealSamples++;
        }
    }

	tInitAngle_IMU 		= tAccumulatedAngle_IMU / tRealSamples;
	tInitAngle_ABS 		= tAccumulatedAngle_ABS / tRealSamples;
	tInitAngle_IMUABS 	= tAccumulatedAngle_IMUABS / tRealSamples;

	widmAngleData->initAngle = tInitAngle_IMUABS;
	tInitThighAngle			 = tInitAngle_IMUABS;

	SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}
#endif /* IMUABS_MODE */

/*
*The function GetINCLinkData considers the Incremental Encoder's raw values.
*/
static void GetINCLinkData(WIDM_AngleData_t* widmAngleData)
{
//	// Left Leg //
//	if (widmAttachCaseObj < 8){
//		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0;		// If gear ratio is correctly selected (Suit = 18.75:1)
//		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
//	}
//
//	// Right Leg //
//	else if (widmAttachCaseObj >= 8){
//		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0 * (-1);
//		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
//	}

	// Debugging... //
	if (widmAttachCaseObj == WIDM_H10_RIGHT){
		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0;
		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
	}
	else if (widmAttachCaseObj == WIDM_H10_LEFT){
		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0;
		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
	}
	else if (widmAttachCaseObj == WIDM_K10_RIGHT){
		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0 * (-1);
		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
	}
	else if (widmAttachCaseObj == WIDM_K10_LEFT){
		widmAngleData->degINC 	= motor_out.position / WIDM_PI * 180.0 * (-1);
		widmAngleData->velINC   = (widmAngleData->degINC - widmAngleData->degINCPrev) / 0.001;
	}


	widmAngleData->degINCPrev = widmAngleData->degINC;		// 1ms Update Inc degree value
}

#ifdef IMUINC_MODE
/*
 *Function to compensate IMU + INC case
*/
static void CompensateIMUINC(WIDM_AngleData_t* widmAngleData)
{
//	widmAngleData->degIMUINC = widmAngleData->degTvcf[0] + widmAngleData->degINC;
	widmAngleData->degIMUINC = widmAngleData->degTvcf[0] + widmAngleData->degINC;
	widmAngleData->velIMUINC = widmAngleData->velRaw[0]  + widmAngleData->velINC;
}

/*
 *Function to calculate the initial thigh angle - IMU + INC case
*/
static void GetInitialAngle_IMUINC(IOIF_6AxisData_t* imu6AxisData, WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_AttachCase_t widmAttachCase)
{
	uint8_t tTotalSamples 			= 100;
	uint8_t tDataCheck_IMU 			= 0;
	uint8_t tRealSamples			= 0;
	float tAccumulatedAngle_IMU 	= 0.0;
	float tAccumulatedAngle_INC 	= 0.0;
	float tAccumulatedAngle_IMUINC 	= 0.0;

	float tInitAngle_IMU 			= 0.0;
	float tInitAngle_INC 			= 0.0;
	float tInitAngle_IMUINC 		= 0.0;
	float tInitThighAngle			= 0.0;


	for (uint8_t i = 1; i <= tTotalSamples; i++){
		for (int j = 0; j < 100000; j++){
			// For Delay of DMA reading
		}
        tDataCheck_IMU = IOIF_AccGyro_GetValue(imu6AxisData);
        if (tDataCheck_IMU == 0){
        	widmSensorData->accX[0] = imu6AxisData->accX;
        	widmSensorData->accY[0] = imu6AxisData->accY;
        	widmSensorData->gyrZ[0] = imu6AxisData->gyrZ;

        	float tImuAngle = WIDM_AttachCaseSetting(widmSensorData, widmAttachCase);
        	tAccumulatedAngle_IMU += tImuAngle;

        	GetINCLinkData(widmAngleData);
        	tAccumulatedAngle_INC += widmAngleData->degINC;
            tAccumulatedAngle_IMUINC += (tImuAngle + widmAngleData->degINC);

        	tRealSamples++;
        }
    }

	tInitAngle_IMU 		= tAccumulatedAngle_IMU / tRealSamples;
	tInitAngle_INC 		= tAccumulatedAngle_INC / tRealSamples;
	tInitAngle_IMUINC 	= tAccumulatedAngle_IMUINC / tRealSamples;

	widmAngleData->initAngle = tInitAngle_IMUINC;
	tInitThighAngle			 = tInitAngle_IMUINC;

	SetInitialAngle(widmAngleData, widmNormData, tInitThighAngle);
}
#endif /* IMUINC_MODE */

#endif

/*
 *Function to execute the time-varying complementary filter (with Fuzzy Logic - wc)
*/
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase)
{
	/* Apply time-varying complementary filter on the sensor data using fuzzy logic(wc) and update the thigh angle parameters */
	WIDM_RunTVCF(widmSensorData, widmAngleData, widmFuzzyData->wc, samplingPeriod, widmAttachCase);

	/* Update the unfiltered thigh angle to be the same as the filtered thigh angle */
	widmAngleData->degTvcf[0] = widmAngleData->degTvcfFiltered;
	widmAngleData->velRaw[0] = (widmAngleData->degTvcf[0] - widmAngleData->degTvcf[1]) / WIDM_CONTROL_PERIOD;
}


/*
 *Function to select finally used deg&vel value before filtering
*/
static void SetUsedDegVel(WIDM_AngleData_t* widmAngleData, WIDM_Module_t widmModule)
{
	if (widmModule == WIDM_IMU_U5 || widmModule == WIDM_IMU_SAM){
		widmAngleData->degFinal = widmAngleData->degTvcf[0];
		widmAngleData->velFinal = widmAngleData->velRaw[0];
	}
	else if (widmModule == WIDM_IMUABS_SAM){
		widmAngleData->degFinal = widmAngleData->degIMUABS;
		widmAngleData->velFinal = widmAngleData->velIMUABS;
	}
	else if (widmModule == WIDM_IMUINC_SAM){
		widmAngleData->degFinal = widmAngleData->degIMUINC;
		widmAngleData->velFinal = widmAngleData->velIMUINC;
	}

	widmAngleData->degFinal = widmAngleData->degFinal - widmAngleData->initAngle;
}

/*
 *Function to reduce noise in sensor data
*/
static void NoiseReduction(WIDM_AngleData_t* widmAngleData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule)
{
	float dt = 1000.0;
	float wTarget = WIDM_GetMaxValue(0.3, dt/widmGaitData->gaitPeriod);
	float freqLPF = 1.2 * wTarget * 2 * WIDM_PI;

	/* First LPF filter on angle data */
	widmAngleData->degLPF1st[0] = WIDM_LPF(
		widmAngleData->degFinal,
		widmAngleData->degLPF1st[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* Second LPF filter on angle data */
	widmAngleData->degLPF2nd[0] = WIDM_LPF(
		widmAngleData->degLPF1st[0],
		widmAngleData->degLPF2nd[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* First LPF filter on velocity data */
	widmAngleData->velLPF1st[0] = WIDM_LPF(
		widmAngleData->velFinal,
		widmAngleData->velLPF1st[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);

	/* Second LPF filter on velocity data */
	widmAngleData->velLPF2nd[0] = WIDM_LPF(
		widmAngleData->velLPF1st[0],
		widmAngleData->velLPF2nd[1],
		freqLPF,
		WIDM_CONTROL_PERIOD
	);
}

/*
 *Function to normalize sensor data and calculate the current phase of the gait
*/
static void GetGaitPhase(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_GaitData_t* widmGaitData)
{
	/* Prepare for circular normalization by finding the center point of the original ellipse */
	WIDM_Normalization(widmAngleData, widmNormData, widmGaitData);

	float dt = 0.001;
	widmNormData->degOriLPF[0] = WIDM_LPF(widmNormData->degOri, widmNormData->degOriLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->velOriLPF[0] = WIDM_LPF(widmNormData->velOri, widmNormData->velOriLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->ampDegLPF[0] = WIDM_LPF(widmNormData->ampDeg, widmNormData->ampDegLPF[1], NORM_CUTOFF_FREQ, dt);
	widmNormData->ampVelLPF[0] = WIDM_LPF(widmNormData->ampVel, widmNormData->ampVelLPF[1], NORM_CUTOFF_FREQ, dt);

	/* Normalize degree and velocity data based on calculated origin and amplitude */
	widmNormData->degNorm = (widmAngleData->degLPF2nd[0] - widmNormData->degOriLPF[0]) / widmNormData->ampDegLPF[0];
	widmNormData->velNorm = (widmAngleData->velLPF2nd[0] - widmNormData->velOriLPF[0]) / widmNormData->ampVelLPF[0];

	/* Calculate and update the current phase of the gait */
	widmGaitData->gaitPhase = WIDM_GetGaitPhase(widmNormData, widmGaitData); // Current phase (0 ~ 100%)
}

/*
 *This function calculates and returns the phase radius
*/
static float GetPhaseRadius(float degDiff, float degTh, float velDiff, float velTh)
{
    /* Calculate degree ratio */
    float degRatio = degDiff / degTh;

    /* Calculate velocity ratio */
    float velRatio = velDiff / velTh;

    /* Calculate and return the phase radius */
    return WIDM_SquareRootSum(degRatio, velRatio);
}

/*
 *This function updates the walking state based on the phase radii and sum_i
*/
static void UpdateWalkingState(WIDM_GaitData_t* widmGaitData, float phaseRadiusStart, float phaseRadiusStop, int16_t sumIter)
{
    /* The walking state is updated based on the current walking state, phase radii, and t_sum_i */
    switch (widmGaitData->walkingState)
    {
        case WIDM_STOP:
            /* If the start phase radius is greater than 1, set the walking state to 1 */
            if (phaseRadiusStart > 1){
            	widmGaitData->walkingState = WIDM_WALKING_START;
            }
            break;
        case WIDM_WALKING_START:
            /* If sum_i is greater than 1000, set the walking state to 2 */
            if (sumIter > 700){
            	widmGaitData->walkingState = WIDM_WALKING_HALF_CYCLE;
            }
            break;
        case WIDM_WALKING_HALF_CYCLE:
            /* If sum_i is 0, set the walking state to 3 */
            if (sumIter == 0){
            	widmGaitData->walkingState = WIDM_WALKING_ONE_CYCLE;
            }
            break;
        default:
            /* If the stop phase radius is less than 1, set the walking state to 0 */
//            if (phaseRadiusStop < 1){
//            	widmGaitData->walkingState = WIDM_STOP;
//            }
            if (phaseRadiusStop < 1){
//            	stopCnt++;
//            	if (stopCnt >= 250){
//            		widmGaitData->walkingState = WIDM_STOP;
//            		stopCnt = 0;
//            	}
            	widmGaitData->walkingState = WIDM_STOP;
            }
            else{
            	stopCnt = 0;
            }
            break;
    }
}

/*
*This function checks the walking state using the walking parameters and IMU system information
*/
static void CheckWalkingState(WIDM_AngleData_t* widmAngleData, WIDM_NormData_t* widmNormData, WIDM_ThresData_t* widmThresData, WIDM_GaitData_t* widmGaitData)
{
	float degDiff = 0.0;
	float velDiff = 0.0;
	float phaseRadiusStart = 0.0;
	float phaseRadiusStop = 0.0;

    /* Get the relevant values from the walking parameters and IMU system */
    degDiff = widmAngleData->degLPF2nd[0] - widmNormData->degOri;
    velDiff = widmAngleData->velLPF2nd[0] - widmNormData->velOri;

    // For Debugging Threshold values //
    deg_diff = degDiff;
    vel_diff = velDiff;

    /* Calculate the start and stop phase radii */
    phaseRadiusStart = GetPhaseRadius(degDiff, widmThresData->degThStart, velDiff, widmThresData->velThStart);
    phaseRadiusStop  = GetPhaseRadius(degDiff, widmThresData->degThStop, velDiff, widmThresData->velThStop);

    /* Update the walking state based on the phase radii and sum_i */
    UpdateWalkingState(widmGaitData, phaseRadiusStart, phaseRadiusStop, widmNormData->sumIter);

    /* If the walking state is 0 or 1, set the gait phase to -100 */
    if (widmGaitData->walkingState == WIDM_STOP || widmGaitData->walkingState == WIDM_WALKING_START){
        widmGaitData->gaitPhase = -100;
    }
}

///* ------------------- SDO CALLBACK ---------------------*/
//static void SetAbsOffsetCmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
//{
//	memcpy(&absOffsetCmd, req->data, 1);
//
//	res->size = 0;
//	res->status = DOP_SDO_SUCC;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SUIT_MD_ENABLED
// 10.29 version //
static void GaitFunction_Prof_1029_K10(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	velLPF0ABS = WIDM_Abs_double(velLPF);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);


	GetINCLinkData(&widmAngleDataObj);
	incDegTrig[0] = widmAngleDataObj.degINC;
	incVel = widmAngleDataObj.velINC;

	filteredIncDeg = WIDM_Abs_double(widmAngleDataObj.degINC);
	filteredIncDeg = WIDM_LPF_walking_Prof(widmAngleDataObj.degINC);
	incDegTrig[0] = filteredIncDeg;

	filteredIncVel = WIDM_Abs_double(incVel);
	filteredIncVel = WIDM_LPF_walking_Prof(incVel);
	incVelTrig[0] = filteredIncVel;

	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 4){									// mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		gyroLPF[0] = 0.98 * gyroLPF[1] + 0.02 * widmSensorData->gyrZ[0];
		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * gyroLPF[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%

	B15Flag = 0;		// after B8 Flag on, swing for extension


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && gyroLPF[1] > 0 && gyroLPF[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && gyroLPF[1] < 0 && gyroLPF[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
			B8stack++;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (gyroLPF[0] > 5){
            B9Flag = 1;
        }
        if (gyroLPF[0] < -5){
            B10Flag = 1;
        }

		if (B8Finished == 1) {
			if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2] && incDegTrig[0] > 40) {
			// if (incVelTrig[0] < incVelTrig[1] && incVelTrig[1] > incVelTrig[2]) {
			// if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2]) {
				// if (incVelTrig[0] < 0 && incVelTrig[1] > 0 && incVelTrig[1] > incVelTrig[2]) {
        			B15Flag = 1;
					B15stack++;
    			// }
				// B15Flag = 1;
				// B15stack++;
			}
		}
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];

	incDegTrig[2] = incDegTrig[1];
	incDegTrig[1] = incDegTrig[0];

	incVelTrig[2] = incVelTrig[1];
	incVelTrig[1] = incVelTrig[0];
}

static void GaitFunction_Prof_1029_K10_Revised(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	velLPF0ABS = WIDM_Abs_double(velLPF);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);


	GetINCLinkData(&widmAngleDataObj);

	if (k > 1){
		incDegLPF[0] = 0.98 * incDegLPF[1] + 0.02 * widmAngleDataObj.degINC;
	}

//	incDegTrig[0] = widmAngleDataObj.degINC;
//	incVel = widmAngleDataObj.velINC;
//
//	filteredIncDeg = WIDM_Abs_double(widmAngleDataObj.degINC);
//	filteredIncDeg = WIDM_LPF_walking_Prof(widmAngleDataObj.degINC);
//	incDegTrig[0] = filteredIncDeg;
//
//	filteredIncVel = WIDM_Abs_double(incVel);
//	filteredIncVel = WIDM_LPF_walking_Prof(incVel);
//	incVelTrig[0] = filteredIncVel;

	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 4){									// mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		gyroLPF[0] = 0.98 * gyroLPF[1] + 0.02 * widmSensorData->gyrZ[0];
//		gyroLPF[0] = 0.997 * gyroLPF[1] + 0.003 * widmSensorData->gyrZ[0];		// 1NE changed
		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * gyroLPF[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%

	B15Flag = 0;		// after B8 Flag on, swing for extension
	B16Flag = 0;		// 1NE's revised Extension timing


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && gyroLPF[1] > 0 && gyroLPF[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && gyroLPF[1] < 0 && gyroLPF[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
			B8stack++;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (gyroLPF[0] > 5){
            B9Flag = 1;
        }
        if (gyroLPF[0] < -5){
            B10Flag = 1;
        }

		if (B8Finished == 1) {
			if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2] && incDegTrig[0] > 40) {
			// if (incVelTrig[0] < incVelTrig[1] && incVelTrig[1] > incVelTrig[2]) {
			// if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2]) {
				// if (incVelTrig[0] < 0 && incVelTrig[1] > 0 && incVelTrig[1] > incVelTrig[2]) {
        			B15Flag = 1;
					B15stack++;
    			// }
				// B15Flag = 1;
				// B15stack++;
			}
		}

		// Added by 1NE //
		chk1Prev = chk1;
		if (k - chk1 > 0.6 * Period){
			if (incDegLPF[2] < incDegLPF[1] && incDegLPF[0]  < incDegLPF[1] && incDegLPF[1] > 10){
				B16Flag = 1;
				B16_stack++;
				chk1 = k;
			}
			else{
				chk1 = chk1Prev;
			}
		}

	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];

	incDegTrig[2] = incDegTrig[1];
	incDegTrig[1] = incDegTrig[0];

	incVelTrig[2] = incVelTrig[1];
	incVelTrig[1] = incVelTrig[0];

	incDegLPF[2] = incDegLPF[1];
	incDegLPF[1] = incDegLPF[0];
}

static void GaitFunction_Prof_1029_H10(WIDM_AngleData_t* widmAngleData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF_H10[0] = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k < 1000){
		velLPF_H10[0] = 0;
	}

	velLPF0ABS = WIDM_Abs_double(velLPF_H10[0]);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);
	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * velLPF_H10[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && velLPF_H10[1] > 0 && velLPF_H10[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && velLPF_H10[1] < 0 && velLPF_H10[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (velLPF_H10[0] > 8){
            B9Flag = 1;
        }
        if (velLPF_H10[0] < -8){
            B10Flag = 1;
        }
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	velLPF_H10[1] = velLPF_H10[0];
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];
}


float degUnbias = 0;
uint8_t update_num = 0;
double check1 = 0;
double check2 = 0;
double degBPFAmp = 100;
double degLPF1NE = 0;
double degBPF1NE[3] = {0, 0, 0};
//static void GaitFunction_Prof_1029_H10_revised(WIDM_AngleData_t* widmAngleData)
//{
//	k++; 			// Check this Loop Count
//	a = k/1000;		// For Debug, check each 1sec
//
//	velLPF_H10[0] = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
//	if (k < 1000){
//		velLPF_H10[0] = 0;
//	}
//
//	velLPF0ABS = WIDM_Abs_double(velLPF_H10[0]);
//	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);
//	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;
//
//	if (k > 1){
//		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;
//
//		mode[0] = mode[1];
//
//		if (k > 1000){
//			if (mode[1] == 0 && modeCheck > 10){									// mode = 1 : Walking, mode = 0 : Stop
//				mode[0] = 1;
//			}
//			else if (mode[1] == 1 && modeCheck < 1){
//				mode[0] = 0;
//			}
//		}
//
//		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * velLPF_H10[0];
//	}
//
//	if (k > 10){
//		if (mode[0] == 1){
//			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
//				if (firstPeriodCheck == 0){
//					Period = k - timeStampPrev;
//					if (Period > 2000){
//						Period = 2000;
//					}
//					else if (Period < 200){
//						Period = 200;
//					}
//				}
//				else{
//					firstPeriodCheck = 0;
//				}
//
//				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
//				timeStampPrev = k;
//			}
//		}
//		else if (mode[0] == 0){
//			firstPeriodCheck = 1;
//		}
//	}
//
//	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
//	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
//	degBPF0ABS = WIDM_Abs_double(degBPF[0]);
//
//	if (k > 3){
//		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
//			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
//		}
//
//		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
//		velBPF0ABS = WIDM_Abs_double(velBPF);
//
//		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
//			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
//		}
//	}
//
//	if (mode[0] == 1){
//		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
//		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
//	}
//
//	if (k > 1000){
//		angleNorm = degBPF[0] / PeakAmpSmooth;
//		velocityNorm = - velBPF / PeakWAmpSmooth;
//	}
//
//	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....
//
//	B2Flag = 0;			// 10%
//	B3Flag = 0;			// 20%
//	B4Flag = 0;			// 30%
//	B5Flag = 0;			// 40%
//	B6Flag = 0;			// 50%
//
//	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
//	B8Flag = 0;			// 50%	 Leg back->front
//	B9Flag = 0;			// 75%	 Leg middle->front
//	B10Flag = 0;		// 25%	 Leg middle->back
//
//	B11Flag = 0;		// 60%
//	B12Flag = 0;		// 70%
//	B13Flag = 0;		// 80%
//	B14Flag = 0;		// 90%
//
//	B17Flag = 0;		// 1.8 ~ 2 sec
//	B18Flag = 0;		// 1.5 ~ 1.8 sec
//	B19Flag = 0;		// 1.2 ~ 1.5 sec
//	B20Flag = 0;		// 0.8 ~ 1.2 sec
//	B21Flag = 0;		// 0 ~ 0.8 sec
//
//
//	if (k > 1000 && mode[0] == 1){
//		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
//			B2Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
//			B3Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
//			B4Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
//			B5Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
//			B6Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
//			B11Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
//			B12Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
//			B13Flag = 1;
//		}
//		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
//			B14Flag = 1;
//		}
//	}
//
//
//	degLPF0ABS = WIDM_Abs_double(degLPF[0]);
//
//	if (degLPF0ABS < 20 && mode[0] == 0){
//		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
//	}
//
//	degUnbiased[0] = degLPF[0] - NeutralPosture;
//
//
//
//	degUnbias = (float)degUnbiased[0];
//	// Plus Version
//
//
////	double w_1ne = 0.2*3.14*2;
////	degLPF1NE = WIDM_LPF_1NE(widmAngleDataObj.degFinal, w_1ne);
////	degBPF1NE[2] = degBPF1NE[1];
////	degBPF1NE[1] = degBPF1NE[0];
////	degBPF1NE[0] = WIDM_BPF_Peak_Prof_2_LH(degLPF1NE, 6);
////
////	if (k > 1000){
////		if (degBPF1NE[2] < degBPF1NE[1] && degBPF1NE[1] > degBPF1NE[0]){
////			check1 = degBPF1NE[1];
////			update_num++;
////		}
////		else if (degBPF1NE[2] > degBPF1NE[1] && degBPF1NE[1] < degBPF1NE[0]){
////			check2 = degBPF1NE[1];
////			update_num++;
////		}
////
////		if (update_num == 2){
////			degBPFAmp = WIDM_Abs_double(check1 - check2);
////			update_num = 0;
////		}
////
////
////		if (mode[0] == 1 && mode[1] == 0){
////			mode[0] = 1;
////		}
////		else if (mode[0] == 1){
////			if (degBPFAmp < 90){
////				mode[0] = 0;
////			}
////		}
////	}
//
//
//
//
//	if (k > 1000){
//		if (degUnbiased[0] > 10 && velLPF_H10[1] > 0 && velLPF_H10[0] < 0 && B7Finished == 0 && mode[0] == 1){
//			B7Flag = 1;
//			B7Finished = 1;
//			gaitCount++;
//		}
//		if (degUnbiased[0] < 3 && velLPF_H10[1] < 0 && velLPF_H10[0] > 0 && B8Finished == 0 && mode[0] == 1){
//			B8Flag = 1;
//			B8Finished = 1;
//		}
//		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
//			B7Finished = 0;
//		}
//		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
//			B8Finished = 0;
//		}
//        if (velLPF_H10[0] > 8){
//            B9Flag = 1;
//        }
//        if (velLPF_H10[0] < -8){
//            B10Flag = 1;
//        }
//	}
//
//
//
//	if (k > 1000){
////        if (Period > 1800 && Period <= 2000){
////        	B17Flag = 1;
////        }
////        else if (Period > 1500 && Period <= 1800){
////        	B18Flag = 1;
////        }
////        else if (Period > 1200 && Period <= 1500){
////        	B19Flag = 1;
////        }
////        else if (Period > 800 && Period <= 1200){
////        	B20Flag = 1;
////        }
////        else if (Period <= 800){
////        	B21Flag = 1;
////        }
//	}
//
//	// Update previous values//
//	mode[1] = mode[0];
//	B1Flag = mode[0];
//	gaitPhasePrev = gaitPhase;
//	velLPF_H10[1] = velLPF_H10[0];
//	degLPF[1] = degLPF[0];
//	gyroLPF[1] = gyroLPF[0];
//	velLPF2[1] = velLPF2[0];
//	degBPF[2] = degBPF[1];
//	degBPF[1] = degBPF[0];
//	degUnbiased[1] = degUnbiased[0];
//}

uint32_t s1 = 0;
uint32_t s3 = 0;
uint8_t s2 = 0;
float degBPF_LPF[2] = {0,0};
uint32_t walkCurr = 0;
uint32_t walkPrev = 0;
uint8_t first1NE = 1;
static void GaitFunction_Prof_1029_H10_revised(WIDM_AngleData_t* widmAngleData)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec
	static uint8_t firstStep = 0;
	static uint16_t stopCount = 0;

	velLPF_H10[0] = WIDM_LPF_walking_Prof((double)widmAngleData->velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k < 1000){
		velLPF_H10[0] = 0;
	}

	velLPF0ABS = WIDM_Abs_double(velLPF_H10[0]);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);
	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * widmAngleData->degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * velLPF_H10[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)widmAngleData->degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(widmAngleData->velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%

	B17Flag = 0;		// 1.8 ~ 2 sec
	B18Flag = 0;		// 1.5 ~ 1.8 sec
	B19Flag = 0;		// 1.2 ~ 1.5 sec
	B20Flag = 0;		// 0.8 ~ 1.2 sec
	B21Flag = 0;		// 0 ~ 0.8 sec


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.99 * NeutralPosture + 0.01 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;
	degUnbias = (float)degUnbiased[0];

	degBPF_LPF[0] = degBPF_LPF[1] * 0.99 + degBPF[0] * 0.01;
	degBPF_LPF[1] = degBPF_LPF[0];

	if ( mode[0] == 1 && (degBPF[0]*degBPF[0]*degBPF[0]*degBPF[0]) > 10000 ){
		if (first1NE == 1){
			walkCurr = 1;
			walkPrev = k;
			first1NE = 0;
		}
		else{
			walkCurr = 1;
			walkPrev = k;
		}
	}
	else{
		if ( mode[0] == 1 && (degBPF[0]*degBPF[0]*degBPF[0]*degBPF[0]) < 10000 && first1NE == 0 ){
			if (k - walkPrev < 1100){
				walkCurr = 1;
			}
			else{
				walkCurr = 0;
				first1NE = 1;
			}
		}
	}

	if (gaitPhase > 2 && gaitPhase < 98){
		if (gaitPhase > gaitPhasePrev){
			gaitPhase = gaitPhase;
		}
		else{
			gaitPhase = gaitPhasePrev;
		}
	}


	if (k > 1000){
		if (degUnbiased[0] > 10 && velLPF_H10[1] > 0 && velLPF_H10[0] < 0 && B7Finished == 0 && mode[0] == 1){
			if (walkCurr == 1){
				B7Flag = 1;
				B7Finished = 1;
				gaitCount++;
			}

			/* 1/15 Tuning */
//			if (abs(degBPF_LPF[0]) > 8){
//				B7Flag = 1;
//				B7Finished = 1;
//				gaitCount++;
//			}

			/* Original */
//			B7Flag = 1;
//			B7Finished = 1;
//			gaitCount++;
		}
		if (degUnbiased[0] < 3 && velLPF_H10[1] < 0 && velLPF_H10[0] > 0 && B8Finished == 0 && mode[0] == 1){
			if (walkCurr == 1){
				B8Flag = 1;
				B8Finished = 1;
			}

			/* 1/15 Tuning */
//			if (abs(degBPF_LPF[0]) > 8){
//				B8Flag = 1;
//				B8Finished = 1;
//			}

			/* Original */
//			B8Flag = 1;
//			B8Finished = 1;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}
        if (velLPF_H10[0] > 8){
            B9Flag = 1;
        }
        if (velLPF_H10[0] < -8){
            B10Flag = 1;
        }
	}



	if (k > 1000){
//        if (Period > 1800 && Period <= 2000){
//        	B17Flag = 1;
//        }
//        else if (Period > 1400 && Period <= 1800){
//        	B18Flag = 1;
//        }
//        else if (Period > 1100 && Period <= 1400){
//        	B19Flag = 1;
//        }
//        else if (Period > 700 && Period <= 1100){
//        	B20Flag = 1;
//        }
//        else if (Period <= 700){
//        	B21Flag = 1;
//        }
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	velLPF_H10[1] = velLPF_H10[0];
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];
}

/*
 *Function to reduce noise in sensor data : Professor's solution
*/
static void CheckWalkingState_Prof(void)
{
		if (B2Flag == 1){
//			B2_chk[B2stack] = (uint8_t)gaitPhase;		// 10%
			B2stack++;
		}
		// B3Flag //
		if (B3Flag == 1){
//			B3_chk[B3stack] = (uint8_t)gaitPhase;		// 20%
			B3stack++;
		}
		// B4Flag //
		if (B4Flag == 1){
//			B4_chk[B4stack] = (uint8_t)gaitPhase;		// 30%
			B4stack++;
		}
		// B5Flag//
		if (B5Flag == 1){
//			B5_chk[B5stack] = (uint8_t)gaitPhase;		// 40%
			B5stack++;
		}
		// B6Flag //
		if (B6Flag == 1){
//			B6_chk[B6stack] = (uint8_t)gaitPhase;		// 50%
			B6stack++;
		}
		// B7Flag //
		if (B7Flag == 1){
//			B7_chk[B7stack] = (uint8_t)gaitPhase;		// ~~ 0% FB transition
			B7stack++;
		}
		// B8Flag //
		if (B8Flag == 1){
//			B8_chk[B8stack] = (uint8_t)gaitPhase;		// ~~ 50% BF transition
			B8stack++;
		}
		// B9Flag //
		if (B9Flag == 1){
//			B9_chk[B9stack] = (uint8_t)gaitPhase;		// Backward -> Forward
			B9stack++;
		}
		// B10Flag //
		if (B10Flag == 1){
//			B10_chk[B10stack] = (uint8_t)gaitPhase;		// Forward -> Backward
			B10stack++;
		}
}
#endif /* SUIT_MD_ENABLED */



/* Gait Ctrl Algorithm for MiniCM */
#ifdef SUIT_MINICM_ENABLED
static void GaitFunction_Prof_1029_K10_MiniCM(float degFinal, float velFinal, float gyrZ, float degINC, float velINC)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF = WIDM_LPF_walking_Prof((double)velFinal);			// velLPF : velRaw -> Low-pass filtering
	velLPF0ABS = WIDM_Abs_double(velLPF);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);


//	GetINCLinkData(&widmAngleDataObj);
	incDegTrig[0] = degINC;
	incVel = velINC;

	filteredIncDeg = WIDM_Abs_double(degINC);
	filteredIncDeg = WIDM_LPF_walking_Prof(degINC);
	incDegTrig[0] = filteredIncDeg;

	filteredIncVel = WIDM_Abs_double(incVel);
	filteredIncVel = WIDM_LPF_walking_Prof(incVel);
	incVelTrig[0] = filteredIncVel;

	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 4){									// mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		gyroLPF[0] = 0.98 * gyroLPF[1] + 0.02 * gyrZ;
		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * gyroLPF[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%

	B15Flag = 0;		// after B8 Flag on, swing for extension


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && gyroLPF[1] > 0 && gyroLPF[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && gyroLPF[1] < 0 && gyroLPF[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
			B8stack++;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (gyroLPF[0] > 5){
            B9Flag = 1;
        }
        if (gyroLPF[0] < -5){
            B10Flag = 1;
        }

		if (B8Finished == 1) {
			if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2] && incDegTrig[0] > 40) {
			// if (incVelTrig[0] < incVelTrig[1] && incVelTrig[1] > incVelTrig[2]) {
			// if (incDegTrig[0] < incDegTrig[1] && incDegTrig[1] > incDegTrig[2]) {
				// if (incVelTrig[0] < 0 && incVelTrig[1] > 0 && incVelTrig[1] > incVelTrig[2]) {
        			B15Flag = 1;
					B15stack++;
    			// }
				// B15Flag = 1;
				// B15stack++;
			}
		}
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];

	incDegTrig[2] = incDegTrig[1];
	incDegTrig[1] = incDegTrig[0];

	incVelTrig[2] = incVelTrig[1];
	incVelTrig[1] = incVelTrig[0];
}


static void GaitFunction_Prof_1029_H10_MiniCM(float degFinal, float velFinal)
{
	k++; 			// Check this Loop Count
	a = k/1000;		// For Debug, check each 1sec

	velLPF_H10[0] = WIDM_LPF_walking_Prof((double)velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k < 1000){
		velLPF_H10[0] = 0;
	}

	velLPF0ABS = WIDM_Abs_double(velLPF_H10[0]);
	double velLPFAbs = WIDM_GetMaxValue_double(0, velLPF0ABS-15);
	modeCheck = 0.993 * modeCheck + 0.007 * velLPFAbs;

	if (k > 1){
		degLPF[0] = 0.98 * degLPF[1] + 0.02 * degFinal;

		mode[0] = mode[1];

		if (k > 1000){
			if (mode[1] == 0 && modeCheck > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode[0] = 1;
			}
			else if (mode[1] == 1 && modeCheck < 1){
				mode[0] = 0;
			}
		}

		velLPF2[0] = 0.9997 * velLPF2[1] + 0.0003 * velLPF_H10[0];
	}

	if (k > 10){
		if (mode[0] == 1){
			if (velLPF2[1] > 0 && velLPF2[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck == 0){
					Period = k - timeStampPrev;
					if (Period > 2000){
						Period = 2000;
					}
					else if (Period < 200){
						Period = 200;
					}
				}
				else{
					firstPeriodCheck = 0;
				}

				cutoffFreq = 2 * WIDM_PI / (Period * 0.001);
				timeStampPrev = k;
			}
		}
		else if (mode[0] == 0){
			firstPeriodCheck = 1;
		}
	}

	cutoffFreqSmooth = 0.992 * cutoffFreqSmooth + 0.008 * cutoffFreq;
	degBPF[0] = WIDM_BPF_Peak_Prof_1((double)degFinal, cutoffFreqSmooth);
	degBPF0ABS = WIDM_Abs_double(degBPF[0]);

	if (k > 3){
		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < degBPF[2] && degBPF[0] > degBPF[1]){
			PeakAmp = degBPF0ABS;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF = WIDM_BPF_Peak_Prof_2(velFinal, cutoffFreqSmooth);
		velBPF0ABS = WIDM_Abs_double(velBPF);

		if (mode[0] == 1 && firstPeriodCheck == 0 && degBPF[1] < 0 && degBPF[0] > 0){
			PeakWAmp = velBPF0ABS;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode[0] == 1){
		PeakAmpSmooth = 0.998 * PeakAmpSmooth + 0.002 * PeakAmp;
		PeakWAmpSmooth = 0.998 * PeakWAmpSmooth + 0.002 * PeakWAmp;
	}

	if (k > 1000){
		angleNorm = degBPF[0] / PeakAmpSmooth;
		velocityNorm = - velBPF / PeakWAmpSmooth;
	}

	gaitPhase = mode[0] * (atan2( (-1)*velocityNorm, (-1)*angleNorm)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag = 0;			// 10%
	B3Flag = 0;			// 20%
	B4Flag = 0;			// 30%
	B5Flag = 0;			// 40%
	B6Flag = 0;			// 50%

	B7Flag = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag = 0;			// 50%	 Leg back->front
	B9Flag = 0;			// 75%	 Leg middle->front
	B10Flag = 0;		// 25%	 Leg middle->back

	B11Flag = 0;		// 60%
	B12Flag = 0;		// 70%
	B13Flag = 0;		// 80%
	B14Flag = 0;		// 90%


	if (k > 1000 && mode[0] == 1){
		if (gaitPhase > phaseThreshold1 && gaitPhasePrev < phaseThreshold1){
			B2Flag = 1;
		}
		if (gaitPhase > phaseThreshold2 && gaitPhasePrev < phaseThreshold2){
			B3Flag = 1;
		}
		if (gaitPhase > phaseThreshold3 && gaitPhasePrev < phaseThreshold3){
			B4Flag = 1;
		}
		if (gaitPhase > phaseThreshold4 && gaitPhasePrev < phaseThreshold4){
			B5Flag = 1;
		}
		if (gaitPhase > phaseThreshold5 && gaitPhasePrev < phaseThreshold5){
			B6Flag = 1;
		}
		if (gaitPhase > phaseThreshold6 && gaitPhasePrev < phaseThreshold6){
			B11Flag = 1;
		}
		if (gaitPhase > phaseThreshold7 && gaitPhasePrev < phaseThreshold7){
			B12Flag = 1;
		}
		if (gaitPhase > phaseThreshold8 && gaitPhasePrev < phaseThreshold8){
			B13Flag = 1;
		}
		if (gaitPhase > phaseThreshold9 && gaitPhasePrev < phaseThreshold9){
			B14Flag = 1;
		}
	}


	degLPF0ABS = WIDM_Abs_double(degLPF[0]);

	if (degLPF0ABS < 20 && mode[0] == 0){
		NeutralPosture = 0.999 * NeutralPosture + 0.001 * degLPF[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased[0] = degLPF[0] - NeutralPosture;

	if (k > 1000){
		if (degUnbiased[0] > 10 && velLPF_H10[1] > 0 && velLPF_H10[0] < 0 && B7Finished == 0 && mode[0] == 1){
			B7Flag = 1;
			B7Finished = 1;
			gaitCount++;
		}
		if (degUnbiased[0] < 3 && velLPF_H10[1] < 0 && velLPF_H10[0] > 0 && B8Finished == 0 && mode[0] == 1){
			B8Flag = 1;
			B8Finished = 1;
		}
		if (degUnbiased[0] < 3 && degUnbiased[1] > 3){
			B7Finished = 0;
		}
		if (degUnbiased[0] > 5 && degUnbiased[1] < 5){
			B8Finished = 0;
		}

        if (velLPF_H10[0] > 8){
            B9Flag = 1;
        }
        if (velLPF_H10[0] < -8){
            B10Flag = 1;
        }
	}

	// Update previous values//
	mode[1] = mode[0];
	B1Flag = mode[0];
	gaitPhasePrev = gaitPhase;
	velLPF_H10[1] = velLPF_H10[0];
	degLPF[1] = degLPF[0];
	gyroLPF[1] = gyroLPF[0];
	velLPF2[1] = velLPF2[0];
	degBPF[2] = degBPF[1];
	degBPF[1] = degBPF[0];
	degUnbiased[1] = degUnbiased[0];
}


static void GaitFunction_Prof_1029_RH10_MiniCM(float degFinal, float velFinal)
{
	k_RH++; 			// Check this Loop Count
	a_RH = k_RH/1000;		// For Debug, check each 1sec

	velLPF_RH[0] = WIDM_LPF_walking_Prof_RH((double)velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k_RH < 1000){
		velLPF_RH[0] = 0;
	}

	velLPF0ABS_RH = WIDM_Abs_double(velLPF_RH[0]);
	double velLPFAbs_RH = WIDM_GetMaxValue_double(0, velLPF0ABS_RH-15);
	modeCheck_RH = 0.993 * modeCheck_RH + 0.007 * velLPFAbs_RH;

	if (k_RH > 1){
		degLPF_RH[0] = 0.98 * degLPF_RH[1] + 0.02 * degFinal;

		mode_RH[0] = mode_RH[1];

		if (k_RH > 1000){
			if (mode_RH[1] == 0 && modeCheck_RH > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode_RH[0] = 1;
			}
			else if (mode_RH[1] == 1 && modeCheck_RH < 1){
				mode_RH[0] = 0;
			}
		}

		velLPF2_RH[0] = 0.9997 * velLPF2_RH[1] + 0.0003 * velLPF_RH[0];
	}

	if (k_RH > 10){
		if (mode_RH[0] == 1){
			if (velLPF2_RH[1] > 0 && velLPF2_RH[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck_RH == 0){
					Period_RH = k_RH - timeStampPrev_RH;
					if (Period_RH > 2000){
						Period_RH = 2000;
					}
					else if (Period_RH < 200){
						Period_RH = 200;
					}
				}
				else{
					firstPeriodCheck_RH = 0;
				}

				cutoffFreq_RH = 2 * WIDM_PI / (Period_RH * 0.001);
				timeStampPrev_RH = k_RH;
			}
		}
		else if (mode_RH[0] == 0){
			firstPeriodCheck_RH = 1;
		}
	}

	cutoffFreqSmooth_RH = 0.992 * cutoffFreqSmooth_RH + 0.008 * cutoffFreq_RH;
	degBPF_RH[0] = WIDM_BPF_Peak_Prof_1_RH((double)degFinal, cutoffFreqSmooth_RH);
	degBPF0ABS_RH = WIDM_Abs_double(degBPF_RH[0]);

	if (k_RH > 3){
		if (mode_RH[0] == 1 && firstPeriodCheck_RH == 0 && degBPF_RH[1] < degBPF_RH[2] && degBPF_RH[0] > degBPF_RH[1]){
			PeakAmp_RH = degBPF0ABS_RH;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF_RH = WIDM_BPF_Peak_Prof_2_RH(velFinal, cutoffFreqSmooth_RH);
		velBPF0ABS_RH = WIDM_Abs_double(velBPF_RH);

		if (mode_RH[0] == 1 && firstPeriodCheck_RH == 0 && degBPF_RH[1] < 0 && degBPF_RH[0] > 0){
			PeakWAmp_RH = velBPF0ABS_RH;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode_RH[0] == 1){
		PeakAmpSmooth_RH = 0.998 * PeakAmpSmooth_RH + 0.002 * PeakAmp_RH;
		PeakWAmpSmooth_RH = 0.998 * PeakWAmpSmooth_RH + 0.002 * PeakWAmp_RH;
	}

	if (k_RH > 1000){
		angleNorm_RH = degBPF_RH[0] / PeakAmpSmooth_RH;
		velocityNorm_RH = - velBPF_RH / PeakWAmpSmooth_RH;
	}

	gaitPhase_RH = mode_RH[0] * (atan2( (-1)*velocityNorm_RH, (-1)*angleNorm_RH)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag_RH = 0;			// 10%
	B3Flag_RH = 0;			// 20%
	B4Flag_RH = 0;			// 30%
	B5Flag_RH = 0;			// 40%
	B6Flag_RH = 0;			// 50%

	B7Flag_RH = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag_RH = 0;			// 50%	 Leg back->front
	B9Flag_RH = 0;			// 75%	 Leg middle->front
	B10Flag_RH = 0;		// 25%	 Leg middle->back

	B11Flag_RH = 0;		// 60%
	B12Flag_RH = 0;		// 70%
	B13Flag_RH = 0;		// 80%
	B14Flag_RH = 0;		// 90%


	if (k_RH > 1000 && mode_RH[0] == 1){
		if (gaitPhase_RH > phaseThreshold1_RH && gaitPhasePrev_RH < phaseThreshold1_RH){
			B2Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold2_RH && gaitPhasePrev_RH < phaseThreshold2_RH){
			B3Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold3_RH && gaitPhasePrev_RH < phaseThreshold3_RH){
			B4Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold4_RH && gaitPhasePrev_RH < phaseThreshold4_RH){
			B5Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold5_RH && gaitPhasePrev_RH < phaseThreshold5_RH){
			B6Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold6_RH && gaitPhasePrev_RH < phaseThreshold6_RH){
			B11Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold7_RH && gaitPhasePrev_RH < phaseThreshold7_RH){
			B12Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold8_RH && gaitPhasePrev_RH < phaseThreshold8_RH){
			B13Flag_RH = 1;
		}
		if (gaitPhase_RH > phaseThreshold9_RH && gaitPhasePrev_RH < phaseThreshold9_RH){
			B14Flag_RH = 1;
		}
	}


	degLPF0ABS_RH = WIDM_Abs_double(degLPF_RH[0]);

	if (degLPF0ABS_RH < 20 && mode_RH[0] == 0){
		NeutralPosture_RH = 0.999 * NeutralPosture_RH + 0.001 * degLPF_RH[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased_RH[0] = degLPF_RH[0] - NeutralPosture_RH;

	if (k_RH > 1000){
		if (degUnbiased_RH[0] > 10 && velLPF_RH[1] > 0 && velLPF_RH[0] < 0 && B7Finished_RH == 0 && mode_RH[0] == 1){
			B7Flag_RH = 1;
			B7Finished_RH = 1;
			gaitCount_RH++;
		}
		if (degUnbiased_RH[0] < 3 && velLPF_RH[1] < 0 && velLPF_RH[0] > 0 && B8Finished_RH == 0 && mode_RH[0] == 1){
			B8Flag_RH = 1;
			B8Finished_RH = 1;
		}
		if (degUnbiased_RH[0] < 3 && degUnbiased_RH[1] > 3){
			B7Finished_RH = 0;
		}
		if (degUnbiased_RH[0] > 5 && degUnbiased_RH[1] < 5){
			B8Finished_RH = 0;
		}

        if (velLPF_RH[0] > 8){
            B9Flag_RH = 1;
        }
        if (velLPF_RH[0] < -8){
            B10Flag_RH = 1;
        }
	}

	// Update previous values//
	mode_RH[1] = mode_RH[0];
	B1Flag_RH = mode_RH[0];
	gaitPhasePrev_RH = gaitPhase_RH;
	velLPF_RH[1] = velLPF_RH[0];
	degLPF_RH[1] = degLPF_RH[0];
	velLPF2_RH[1] = velLPF2_RH[0];
	degBPF_RH[2] = degBPF_RH[1];
	degBPF_RH[1] = degBPF_RH[0];
	degUnbiased_RH[1] = degUnbiased_RH[0];
}


static void GaitFunction_Prof_1029_LH10_MiniCM(float degFinal, float velFinal)
{
	k_LH++; 			// Check this Loop Count
	a_LH = k_LH/1000;		// For Debug, check each 1sec

	velLPF_LH[0] = WIDM_LPF_walking_Prof_LH((double)velFinal);			// velLPF : velRaw -> Low-pass filtering
	if (k_LH < 1000){
		velLPF_LH[0] = 0;
	}

	velLPF0ABS_LH = WIDM_Abs_double(velLPF_LH[0]);
	double velLPFAbs_LH = WIDM_GetMaxValue_double(0, velLPF0ABS_LH-15);
	modeCheck_LH = 0.993 * modeCheck_LH + 0.007 * velLPFAbs_LH;

	if (k_LH > 1){
		degLPF_LH[0] = 0.98 * degLPF_LH[1] + 0.02 * degFinal;

		mode_LH[0] = mode_LH[1];

		if (k_LH > 1000){
			if (mode_LH[1] == 0 && modeCheck_LH > 10){									// mode = 1 : Walking, mode = 0 : Stop
				mode_LH[0] = 1;
			}
			else if (mode_LH[1] == 1 && modeCheck_LH < 1){
				mode_LH[0] = 0;
			}
		}

		velLPF2_LH[0] = 0.9997 * velLPF2_LH[1] + 0.0003 * velLPF_LH[0];
	}

	if (k_LH > 10){
		if (mode_LH[0] == 1){
			if (velLPF2_LH[1] > 0 && velLPF2_LH[0] < 0){								// Leg front->back moving point
				if (firstPeriodCheck_LH == 0){
					Period_LH = k_LH - timeStampPrev_LH;
					if (Period_LH > 2000){
						Period_LH = 2000;
					}
					else if (Period_LH < 200){
						Period_LH = 200;
					}
				}
				else{
					firstPeriodCheck_LH = 0;
				}

				cutoffFreq_LH = 2 * WIDM_PI / (Period_LH * 0.001);
				timeStampPrev_LH = k_LH;
			}
		}
		else if (mode_LH[0] == 0){
			firstPeriodCheck_LH = 1;
		}
	}

	cutoffFreqSmooth_LH = 0.992 * cutoffFreqSmooth_LH + 0.008 * cutoffFreq_LH;
	degBPF_LH[0] = WIDM_BPF_Peak_Prof_1_LH((double)degFinal, cutoffFreqSmooth_LH);
	degBPF0ABS_LH = WIDM_Abs_double(degBPF_LH[0]);

	if (k_LH > 3){
		if (mode_LH[0] == 1 && firstPeriodCheck_LH == 0 && degBPF_LH[1] < degBPF_LH[2] && degBPF_LH[0] > degBPF_LH[1]){
			PeakAmp_LH = degBPF0ABS_LH;			// At Leg Backward->Forward transition, get Amplitude of angle(deg)
		}

		velBPF_LH = WIDM_BPF_Peak_Prof_2_LH(velFinal, cutoffFreqSmooth_LH);
		velBPF0ABS_LH = WIDM_Abs_double(velBPF_LH);

		if (mode_LH[0] == 1 && firstPeriodCheck_LH == 0 && degBPF_LH[1] < 0 && degBPF_LH[0] > 0){
			PeakWAmp_LH = velBPF0ABS_LH;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
		}
	}

	if (mode_LH[0] == 1){
		PeakAmpSmooth_LH = 0.998 * PeakAmpSmooth_LH + 0.002 * PeakAmp_LH;
		PeakWAmpSmooth_LH = 0.998 * PeakWAmpSmooth_LH + 0.002 * PeakWAmp_LH;
	}

	if (k_LH > 1000){
		angleNorm_LH = degBPF_LH[0] / PeakAmpSmooth_LH;
		velocityNorm_LH = - velBPF_LH / PeakWAmpSmooth_LH;
	}

	gaitPhase_LH = mode_LH[0] * (atan2( (-1)*velocityNorm_LH, (-1)*angleNorm_LH)) / (2*WIDM_PI) * 100 + 50;  // 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....

	B2Flag_LH = 0;			// 10%
	B3Flag_LH = 0;			// 20%
	B4Flag_LH = 0;			// 30%
	B5Flag_LH = 0;			// 40%
	B6Flag_LH = 0;			// 50%

	B7Flag_LH = 0;			// 0%	 Leg front->back	&   Gait Count ++
	B8Flag_LH = 0;			// 50%	 Leg back->front
	B9Flag_LH = 0;			// 75%	 Leg middle->front
	B10Flag_LH = 0;		// 25%	 Leg middle->back

	B11Flag_LH = 0;		// 60%
	B12Flag_LH = 0;		// 70%
	B13Flag_LH = 0;		// 80%
	B14Flag_LH = 0;		// 90%


	if (k_LH > 1000 && mode_LH[0] == 1){
		if (gaitPhase_LH > phaseThreshold1_LH && gaitPhasePrev_LH < phaseThreshold1_LH){
			B2Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold2_LH && gaitPhasePrev_LH < phaseThreshold2_LH){
			B3Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold3_LH && gaitPhasePrev_LH < phaseThreshold3_LH){
			B4Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold4_LH && gaitPhasePrev_LH < phaseThreshold4_LH){
			B5Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold5_LH && gaitPhasePrev_LH < phaseThreshold5_LH){
			B6Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold6_LH && gaitPhasePrev_LH < phaseThreshold6_LH){
			B11Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold7_LH && gaitPhasePrev_LH < phaseThreshold7_LH){
			B12Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold8_LH && gaitPhasePrev_LH < phaseThreshold8_LH){
			B13Flag_LH = 1;
		}
		if (gaitPhase_LH > phaseThreshold9_LH && gaitPhasePrev_LH < phaseThreshold9_LH){
			B14Flag_LH = 1;
		}
	}


	degLPF0ABS_LH = WIDM_Abs_double(degLPF_LH[0]);

	if (degLPF0ABS_LH < 20 && mode_LH[0] == 0){
		NeutralPosture_LH = 0.999 * NeutralPosture_LH + 0.001 * degLPF_LH[0];				// Get (Stop & Standing) Posture's Angle
	}

	degUnbiased_LH[0] = degLPF_LH[0] - NeutralPosture_LH;

	if (k_LH > 1000){
		if (degUnbiased_LH[0] > 10 && velLPF_LH[1] > 0 && velLPF_LH[0] < 0 && B7Finished_LH == 0 && mode_LH[0] == 1){
			B7Flag_LH = 1;
			B7Finished_LH = 1;
			gaitCount_LH++;
		}
		if (degUnbiased_LH[0] < 3 && velLPF_LH[1] < 0 && velLPF_LH[0] > 0 && B8Finished_LH == 0 && mode_LH[0] == 1){
			B8Flag_LH = 1;
			B8Finished_LH = 1;
		}
		if (degUnbiased_LH[0] < 3 && degUnbiased_LH[1] > 3){
			B7Finished_LH = 0;
		}
		if (degUnbiased_LH[0] > 5 && degUnbiased_LH[1] < 5){
			B8Finished_LH = 0;
		}

        if (velLPF_LH[0] > 8){
            B9Flag_LH = 1;
        }
        if (velLPF_LH[0] < -8){
            B10Flag_LH = 1;
        }
	}

	// Update previous values//
	mode_LH[1] = mode_LH[0];
	B1Flag_LH = mode_LH[0];
	gaitPhasePrev_LH = gaitPhase_LH;
	velLPF_LH[1] = velLPF_LH[0];
	degLPF_LH[1] = degLPF_LH[0];
	velLPF2_LH[1] = velLPF2_LH[0];
	degBPF_LH[2] = degBPF_LH[1];
	degBPF_LH[1] = degBPF_LH[0];
	degUnbiased_LH[1] = degUnbiased_LH[0];
}

#endif






#ifdef SUIT_MD_ENABLED
/* [MD->MiniCM] Raw Data version */
static void Create_GaitDataPDO(uint8_t nodeID)
{
	if (nodeID == NODE_ID_RH_SAG){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_RH,		DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_RH,		DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
	}
	else if (nodeID == NODE_ID_LH_SAG){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_LH,		DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_LH,		DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
  	}
	else if (nodeID == NODE_ID_RK){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_RK,		DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_RK,		DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GYRZ_RK,		DOP_FLOAT32,	1,    &widmSensorDataObj.gyrZ[0]);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_INC_RK,	DOP_FLOAT32,	1,    &widmAngleDataObj.degINC);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_INC_RK,	DOP_FLOAT32,	1,    &widmAngleDataObj.velINC);
	}
	else if (nodeID == NODE_ID_LK){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_LK,		DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_LK,		DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GYRZ_LK,		DOP_FLOAT32,	1,    &widmSensorDataObj.gyrZ[0]);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_DEG_INC_LK,	DOP_FLOAT32,	1,    &widmAngleDataObj.degINC);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_VEL_INC_LK,	DOP_FLOAT32,	1,    &widmAngleDataObj.velINC);
	}
}

/* [MD->MiniCM] B_Flag version */
static void Create_BFlagPDO(uint8_t nodeID)
{
	if (nodeID == NODE_ID_RH_SAG){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B1FLAG_RH,		DOP_UINT8,	1,    &B1Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B7FLAG_RH,		DOP_UINT8,	1,    &B7Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B8FLAG_RH,		DOP_UINT8,	1,    &B8Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B9FLAG_RH,		DOP_UINT8,	1,    &B9Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B10FLAG_RH,	DOP_UINT8,	1,    &B10Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B17FLAG_RH,	DOP_UINT8,	1,    &B17Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B18FLAG_RH,	DOP_UINT8,	1,    &B18Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B19FLAG_RH,	DOP_UINT8,	1,    &B19Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B20FLAG_RH,	DOP_UINT8,	1,    &B20Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B21FLAG_RH,	DOP_UINT8,	1,    &B21Flag);

		 /* For TEST */
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_ACC_X,			DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_ACC_Y,			DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_ACC_Z,			DOP_FLOAT32,	1,    &degUnbias);
  	}
	else if (nodeID == NODE_ID_LH_SAG){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B1FLAG_LH,		DOP_UINT8,	1,    &B1Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B7FLAG_LH,		DOP_UINT8,	1,    &B7Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B8FLAG_LH,		DOP_UINT8,	1,    &B8Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B9FLAG_LH,		DOP_UINT8,	1,    &B9Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B10FLAG_LH,	DOP_UINT8,	1,    &B10Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B17FLAG_LH,	DOP_UINT8,	1,    &B17Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B18FLAG_LH,	DOP_UINT8,	1,    &B18Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B19FLAG_LH,	DOP_UINT8,	1,    &B19Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B20FLAG_LH,	DOP_UINT8,	1,    &B20Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B21FLAG_LH,	DOP_UINT8,	1,    &B21Flag);

		 /* For TEST */
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GYR_X,			DOP_FLOAT32,	1,    &widmAngleDataObj.degFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GYR_Y,			DOP_FLOAT32,	1,    &widmAngleDataObj.velFinal);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_GYR_Z,			DOP_FLOAT32,	1,    &widmGaitDataObj.gaitPhase);
  	}
	else if (nodeID == NODE_ID_RK){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B1FLAG_RK,		DOP_UINT8,	1,    &B1Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B8FLAG_RK,		DOP_UINT8,	1,    &B8Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B16FLAG_RK,	DOP_UINT8,	1,    &B16Flag);
  	}
	else if (nodeID == NODE_ID_LK){
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B1FLAG_LK,		DOP_UINT8,	1,    &B1Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B8FLAG_LK,		DOP_UINT8,	1,    &B8Flag);
		 Create_PDO(TASK_ID_WIDM, 	 PDO_ID_WIDM_B16FLAG_LK,	DOP_UINT8,	1,    &B16Flag);
  	}
}
#endif /* SUIT_MD_ENABLED */





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef QUATERNION
// Quaternion //
static void SetMagInvAInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&vqfMagCalibObj.a11, t_req->data, 9*4);

	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

   t_res->size = 0;
   t_res->status = DOP_SDO_SUCC;
}

static void SetMagIronErrorInfo(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&vqfMagCalibObj.b1, t_req->data, 3*4);

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;

   t_res->size = 0;
   t_res->status = DOP_SDO_SUCC;
}

static void InitMagInfo(void)
{
	vqfMagCalibObj.A_inv[0][0] = vqfMagCalibObj.a11;
	vqfMagCalibObj.A_inv[0][1] = vqfMagCalibObj.a12;
	vqfMagCalibObj.A_inv[0][2] = vqfMagCalibObj.a13;
	vqfMagCalibObj.A_inv[1][0] = vqfMagCalibObj.a21;
	vqfMagCalibObj.A_inv[1][1] = vqfMagCalibObj.a22;
	vqfMagCalibObj.A_inv[1][2] = vqfMagCalibObj.a23;
	vqfMagCalibObj.A_inv[2][0] = vqfMagCalibObj.a31;
	vqfMagCalibObj.A_inv[2][1] = vqfMagCalibObj.a32;
	vqfMagCalibObj.A_inv[2][2] = vqfMagCalibObj.a33;

	vqfMagCalibObj.ironErr[0] = vqfMagCalibObj.b1;
	vqfMagCalibObj.ironErr[1] = vqfMagCalibObj.b2;
	vqfMagCalibObj.ironErr[2] = vqfMagCalibObj.b3;
}

static int EntGetQuaternion(void)
{
	VQF_Params_t params = { 0 };
	vqfObj.params = params;

	VQF_Init(&vqfObj, 0.001, 0.001, 0.001);
	return 0;
}

static int RunGetQuaternion(void)
{
	//uint8_t t6AxisRes
	//uint8_t t3AxisRes

	IOIF_AccGyro_GetValue(&imu6AxisDataObj);
	IOIF_Magneto_GetValue(&magDataObj);

	//if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) { return t6AxisRes; }
	//if (t3AxisRes != IOIF_MAGNETO_STATUS_OK) { return t3AxisRes; }

	float accX = imu6AxisDataObj.accX;
	float accY = imu6AxisDataObj.accY;
	float accZ = imu6AxisDataObj.accZ;
	float gyrX = imu6AxisDataObj.gyrX * 0.017453f; //degree to radian
	float gyrY = imu6AxisDataObj.gyrY * 0.017453f; //degree to radian
	float gyrZ = imu6AxisDataObj.gyrZ * 0.017453f; //degree to radian

	float magX_r = -magDataObj.magY;
	float magY_r = magDataObj.magX;
	float magZ_r = magDataObj.magZ;

	RealMagX = magX_r;
	RealMagY = magY_r;
	RealMagZ = magZ_r;

	float magValueRaw[3] = {magX_r, magY_r, magZ_r};

	for (int i = 0; i < 3; i++) {
		magValueCal[i] = 0;
		for (int j = 0; j < 3; j++) {
			magValueCal[i] += A_inv[i][j] * (magValueRaw[j] - ironErr[j]);
//			magValueCal[i] += vqfMagCalibInfo.A_inv[i][j] * (magValueRaw[j] - mag_calib_info.iron_err[j]);
		}
	}
	float magnorm = sqrt(magValueCal[0]*magValueCal[0] + magValueCal[1]*magValueCal[1] + magValueCal[2]*magValueCal[2]);
	magValueCal[0] = magValueCal[0] / magnorm;
	magValueCal[1] = magValueCal[1] / magnorm;
	magValueCal[2] = magValueCal[2] / magnorm;

	float magX = magValueCal[0];
	float magY = magValueCal[1];
	float magZ = magValueCal[2];

    VQF_Real_t gyr[3] = {gyrX, gyrY, gyrZ};
    VQF_Real_t acc[3] = {accX, accY, accZ};
    VQF_Real_t mag[3] = {magX, magY, magZ};

	VQF_UpdateWithMag(&vqfObj, gyr, acc, mag);
	VQF_GetQuat9D(&vqfObj, vqfQuat);
	int i;
	for (i = 0; i < 4; i++) { q_send[i] = (int16_t)(vqfQuat[i] * 32768); }

	return 0;
}


/* SPI communication with Nucleo board (For KAIST) */
static uint8_t TransmitData_SPI(void)
{
//	txFloatDataArray[0] = imu6AxisDataObj.accX;
//	txFloatDataArray[1] = imu6AxisDataObj.accY;
//	txFloatDataArray[2] = imu6AxisDataObj.accZ;
//	txFloatDataArray[3] = imu6AxisDataObj.gyrX;
//	txFloatDataArray[4] = imu6AxisDataObj.gyrY;
//	txFloatDataArray[5] = imu6AxisDataObj.gyrZ;

	txFloatDataArray[0] = (float)vqfQuat[0];
	txFloatDataArray[1] = (float)vqfQuat[1];
	txFloatDataArray[2] = (float)vqfQuat[2];
	txFloatDataArray[3] = (float)vqfQuat[3];
	txFloatDataArray[4] = imu6AxisDataObj.gyrY;
	txFloatDataArray[5] = imu6AxisDataObj.gyrZ;

	floatArrayToByteArray(txFloatDataArray, txByteDataArray, DATA_NUM);

	uint8_t txDebug = HAL_SPI_Transmit(&hspi1, txByteDataArray, DATA_NUM * 4, 1);
	return txDebug;
}

static void floatToBytes(float value, uint8_t* bytes)
{
	memcpy(bytes, &value, sizeof(float));
}

static void floatArrayToByteArray(float* floatDataArray, uint8_t* byteDataArray, uint8_t dataNum)
{
	for (uint8_t i = 0; i < dataNum; i++){
		memcpy(&byteDataArray[i * sizeof(float)], &floatDataArray[i], sizeof(float));
	}
}


#endif /* Quaternion */

