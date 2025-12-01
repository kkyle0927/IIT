#include "AS_gait_ctrl.h"

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *      TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

 // Task Object //
TaskObj_t gaitCtrlTask;

// 9 Axis IMU Object //
IOIF_AccGyro_Data_t accgyroDataObj;
IOIF_Magneto_Data_t magDataObj;

uint8_t I2CfailedInitCnt;
uint16_t I2CfailedStateOffCnt;
uint16_t I2CfailedAutoCnt;
uint32_t I2CfailedStateEnCnt;

// Auto Calibration For TVCF //
uint8_t	moduleNodeID;
uint8_t autoCalibSensorCmd;
uint8_t CalNeutralPostureCmd;
uint8_t NeutralCompleted;

uint32_t neutralizedFlag;
uint32_t autoCalibratedFlag;

float tAccumulAngle_TVCF = 0.0;
float tAccumulateThighAngle = 0.0;
bool oneTimeInitAutoCal = true;

WIDM_GaitData_t		widmGaitDataObj_sagittal, widmGaitDataObj_frontal, widmGaitDataObj_transverse;
WIDM_AngleData_t 	widmAngleDataObj_sagittal, widmAngleDataObj_frontal, widmAngleDataObj_transverse;
WIDM_SensorData_t 	widmSensorDataObj_sagittal, widmSensorDataObj_frontal, widmSensorDataObj_transverse;
WIDM_FuzzyData_t	widmFuzzyDataObj_sagittal, widmFuzzyDataObj_frontal, widmFuzzyDataObj_transverse;
// WIDM_NormData_t		widmNormDataObj_sagittal, widmNormDataObj_frontal, widmNormDataObj_transverse;
// WIDM_ThresData_t	widmThresDataObj_sagittal, widmThresDataObj_frontal, widmThresDataObj_transverse;
WIDM_Module_t		widmModuleObj;

// SELECTION !!!  For ALL Model //
WIDM_AttachCase_t	widmAttachCaseObj;

/* USER CODE BEGIN PV */
/* For FLASH READ/WRITE Addr Debugging */
uint32_t writeAddr = 0;
uint32_t readAddr = 0;
/* USER CODE END PV */

GaitObj_t gaitObj_RH;
GaitObj_t gaitObj_LH;
GaitObj_t gaitObj_RK;
GaitObj_t gaitObj_LK;

float widm_accX_sagittal, widm_accY_sagittal, widm_gyrZ_sagittal;
float widm_degTvcf_sagittal, widm_velDegTvcf_sagittal, widm_degAcc_artan_sagittal;
float widm_fc_sagittal;
float widm_accX_frontal, widm_accY_frontal, widm_gyrZ_frontal;
float widm_degTvcf_frontal, widm_velDegTvcf_frontal, widm_degAcc_artan_frontal;
float widm_fc_frontal;

ScaledData FloatToInt16Data;

// For Debug
uint32_t I15Stack = 0;
uint32_t I16Stack = 0;

// Loop Time Count //
static uint32_t gaitCtrlLoopCnt;
uint32_t autoCalibCnt;
uint32_t calNeutralPostureCnt;
float gaitCtrlTimeElap;

uint8_t imuInitOffsetCmd;
uint8_t abs2InitOffsetCmd;

// For Detect Sensor Noise
uint8_t accXAlgorithmActivated;
uint8_t accYAlgorithmActivated;
uint8_t gyrZAlgorithmActivated;
uint32_t accXNoiseCnt;
uint32_t accYNoiseCnt;
uint32_t gyrZNoiseCnt;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- INITIALIZATION ------------------- */
static void InitValueSetting(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData, WIDM_FuzzyData_t* widmFuzzyData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule);
static void ModelSelection(uint8_t nodeID);

static void ResetDataObj(void);

/* ------------------- RUN REAL-TIME FUNCTIONS ------------------- */
static void RunRealTimeFuncs(void);
static void CalNeutralPosture(void);
static void initAutoCalParam(WIDM_SensorData_t* sensorData, IOIF_AccGyro_Data_t* imu6AxisData);
static void AutoCalibrationIMU(void);

/* ------------------- AUTO CALIBRATION SENSOR ------------------- */
static void UpdateSensorForAuto(WIDM_SensorData_t* widmSensorData, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);
static void AccumultateSqrtSum(WIDM_SensorData_t* sensorData);
static void GetAccMeanVal(WIDM_SensorData_t* sensorData, uint32_t sampleSize);
static void AccumulateGyroMatrix(WIDM_SensorData_t* sensorData, WIDM_AttachCase_t attachCase, uint32_t sampleCnt, float samplingPeriod);
static void GetGyroParam(WIDM_SensorData_t* sensorData);
static void CalculateInverse3x3(float matrix[3][3], float invMatrix[3][3], float det);

/* ------------------- GAIT TOTAL FUNCTION ------------------- */
/* Functions for Gait Parameters */
static float SafeAtan2f(float a, float b);
static void UpdateSensorRawData_sagittal(WIDM_SensorData_t* widmSensorData, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);
static void UpdateSensorRawData_frontal(WIDM_SensorData_t* widmSensorData, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase);
static void DetectSensorNoise(float* currentVal, float prevVal, float* lastValidVal, uint8_t* algorithmActivated, uint16_t threshold, uint32_t* errCode, uint32_t* noiseCounter);
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase);

static void CMLocalToGlobal(float local[3], float global[3]);
static int RunGaitFunc(void);

/* ------------------- STATIC FUNCTIONS ------------------- */
static uint16_t ScaleFloatToUInt16(float value, float scaleFactor);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(gaitCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitGaitCtrl(void)
{
	/* Init Task */
	InitTask(&gaitCtrlTask);
	DOPC_AssignTaskID(&gaitCtrlTask, TASK_IDX_GAIT_CTRL);

	/* Checking Node ID */
	moduleNodeID = NODE_ID_CM;

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_OFF,      NULL,   			StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_STANDBY,  NULL,   			StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	/* DOD Definition */
	// DOD
	// DOP_CreateDOD(TASK_IDX_GAIT_CTRL);

   	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_GAIT_CTRL, gaitCtrlTask);
	// DOP_CreatePDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_GAIT_CTRL)
	// DOP_CreateSDO(TASK_IDX_GAIT_CTRL, object_id, DOP_FLOAT32, SDOCallback);

	/* Select correct Module here */
	ModelSelection(moduleNodeID);

	// Quaternion //
	// InitMagInfo();

	/* Initial stage of get angle */
	InitValueSetting(&widmAngleDataObj_sagittal, &widmSensorDataObj_sagittal, &widmFuzzyDataObj_sagittal, &widmGaitDataObj_sagittal, widmModuleObj);
	InitValueSetting(&widmAngleDataObj_frontal, &widmSensorDataObj_frontal, &widmFuzzyDataObj_frontal, &widmGaitDataObj_frontal, widmModuleObj);
	InitValueSetting(&widmAngleDataObj_transverse, &widmSensorDataObj_transverse, &widmFuzzyDataObj_transverse, &widmGaitDataObj_transverse, widmModuleObj);

    readAddr = IOIF_FLASH_SENSOR_CAL_DATA_ADDR;
    /* Download Motor Setting */
    // IOIF_ReadFlash(readAddr, &widmSensorDataObj_sagittal.scaling_gyro,			IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    // IOIF_ReadFlash(readAddr, &widmSensorDataObj_sagittal.bias_gyro,				IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
    // IOIF_ReadFlash(readAddr, &widmSensorDataObj_sagittal.accMeanVal_calibrated,	IOIF_FLASH_READ_SIZE_4B); readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
}

void RunGaitCtrl(void)
{
	RunTask(&gaitCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */

static void StateOff_Run(void)
{
	static uint32_t calibCnt = 0;
	static uint32_t recoveryCnt = 0;
	static float tIMUAngleDeg = 0.0;
	static float tInitThighAngle_IMUDeg	 = 0.0;
	static float tAccumulatedAngle_IMUDeg = 0.0;

	StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_STANDBY);

	/* Thigh Angle Calibration & Set */
/*	if (I2CInitState == IOIF_I2C_STATUS_OK) {
		uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&accgyroDataObj);
		if (t6AxisRes == IOIF_I2C_STATUS_OK) {
			if (calibCnt < 1000) {	// 1000 Samples
				widmSensorDataObj_sagittal.accX[0] = -(accgyroDataObj.accX * ACC_GRAVITY_CONSTANT * cos(WIDM_PI/3) + accgyroDataObj.accZ * ACC_GRAVITY_CONSTANT * sin(WIDM_PI/3));
				widmSensorDataObj_sagittal.accY[0] = accgyroDataObj.accY * ACC_GRAVITY_CONSTANT;
				widmSensorDataObj_frontal.accX[0] = -(accgyroDataObj.accX * ACC_GRAVITY_CONSTANT * cos(WIDM_PI/3) + accgyroDataObj.accZ * ACC_GRAVITY_CONSTANT * sin(WIDM_PI/3));
				widmSensorDataObj_frontal.accY[0] = accgyroDataObj.accY * ACC_GRAVITY_CONSTANT;

				// 가속도 축 보정은 후 보정(최종 허벅지 각도 - 초기 IMU 기계적 각도 ?도)
				tIMUAngleDeg = SafeAtan2f((widmSensorDataObj_frontal.accX[0]), (widmSensorDataObj_frontal.accY[0])) * (180 / WIDM_PI);
				tAccumulatedAngle_IMUDeg += tIMUAngleDeg;
			} else if (calibCnt == 1000) {
				tInitThighAngle_IMUDeg = tAccumulatedAngle_IMUDeg / (float)calibCnt;
				widmAngleDataObj_frontal.degAccInitValue = tInitThighAngle_IMUDeg;
				
				 Set Init Value
				widmAngleDataObj_frontal.degAcc_arctan[1] = widmAngleDataObj_frontal.degAccInitValue; // For LPF init Value
				widmAngleDataObj_frontal.degAcc_1stLPF[1] = widmAngleDataObj_frontal.degAccInitValue; // For LPF init Value
				widmAngleDataObj_frontal.degAcc_calib_1stLPF[1] = widmAngleDataObj_frontal.degAccInitValue; // For LPF init Value
				widmAngleDataObj_frontal.degAcc_calib_1stLPF_LPF[1] = widmAngleDataObj_frontal.degAccInitValue; // For LPF init Value

				widmAngleDataObj_frontal.degGyr_Integral[1] = 0.0; // For Integral
				widmAngleDataObj_frontal.degGyr_Integral_calibrated[1] = 0.0; // For Integral
				widmAngleDataObj_frontal.degGyr_Integral_1stHPF[1] = 0.0; // For HPF init Value
				widmAngleDataObj_frontal.degGyr_calb_Integral_1stHPF[1] = 0.0; // For HPF init Value
				widmAngleDataObj_frontal.degGyr_calb_Integral_1stHPF_LPF[1] = 0.0; // For HPF init Value

				calibCnt = 0;
				StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_STANDBY);
			}

			recoveryCnt = 0;
		} else {	// IMU 6축 Data Read 실패했을 경우
			 Recovery 6axis IMU
			if (recoveryCnt > I2C_REVOVERY_TIME_INTERVAL) {
				// TODO : Error Handling
				t6AxisRes = IOIF_AccGyro_Init(IOIF_I2C2);
				if (t6AxisRes != IOIF_I2C_STATUS_OK) {
					I2CfailedStateOffCnt++;
				}
				recoveryCnt = 0;
			}

			calibCnt = 0;
			recoveryCnt++;
		}

		calibCnt++;
	} else {	// InitGaitCtrl에서 I2C 초기화 실패했을 경우
		if (recoveryCnt > I2C_REVOVERY_TIME_INTERVAL) {
			// TODO : Error Handling
			I2CInitState = IOIF_AccGyro_Init(IOIF_I2C2);
			if (I2CInitState != IOIF_I2C_STATUS_OK) {
				I2CfailedStateOffCnt++;
			}
			recoveryCnt = 0;
		}
		recoveryCnt++;
	}*/
}

static void StateStandby_Run(void)
{
    StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ENABLE);

    // TODO : Auto Calibration Must be in Slow Motion!
	if (autoCalibSensorCmd == 1) {
		IOIF_AccGyro_State_t t6AxisRes = IOIF_AccGyro_GetValue(&accgyroDataObj);
		if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
			gaitCtrlTask.errCode = ERROR_IMU_SENSOR_GET_VALUE;
			StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ERROR);
		}

		UpdateSensorForAuto(&widmSensorDataObj_sagittal, &accgyroDataObj, widmAttachCaseObj);

		AccumultateSqrtSum(&widmSensorDataObj_sagittal);
		AccumulateGyroMatrix(&widmSensorDataObj_sagittal, widmAttachCaseObj, autoCalibCnt, GAIT_CONTROL_PERIOD);

		autoCalibCnt++;
	} else if (autoCalibSensorCmd == 2) { // Auto-Calibration Sensor Parameters
		GetAccMeanVal(&widmSensorDataObj_sagittal, autoCalibCnt);
		GetGyroParam(&widmSensorDataObj_sagittal);

		IOIF_EraseFlash(IOIF_FLASH_SENSOR_CAL_DATA_ADDR, IOIF_ERASE_ONE_SECTOR);
		writeAddr = IOIF_FLASH_START_RS_ADDR;
		float memArr8[READ_FLASH_ARRAY_SIZE] = {0};
		memcpy(&memArr8[0], &widmSensorDataObj_sagittal.scaling_gyro,                       IOIF_FLASH_WRITE_SIZE_4B);
		memcpy(&memArr8[1], &widmSensorDataObj_sagittal.bias_gyro,                          IOIF_FLASH_WRITE_SIZE_4B);
		memcpy(&memArr8[2], &widmSensorDataObj_sagittal.accMeanVal_calibrated,              IOIF_FLASH_WRITE_SIZE_4B);
		IOIF_WriteFlash(writeAddr, memArr8);
		autoCalibSensorCmd = 0;
	}

	// PushRoutine(&gaitCtrlTask.routine, ROUTINE_ID_GAIT_TOTAL_FUNCTION);
	// StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ENABLE);
	// testTransitCmd = 0;
}

static void StateEnable_Ent(void)
{
	gaitCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{
//	RunGaitFunc();

	GaitAnalysis_PostProcessing();

	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
	gaitCtrlLoopCnt = 0;
}

static void StateError_Run(void)
{
	IOIF_Magneto_State_t tinitIMURes = IOIF_Magneto_Init();

	/* Initial stage of get angle */
	ResetDataObj();
	InitValueSetting(&widmAngleDataObj_sagittal, &widmSensorDataObj_sagittal, &widmFuzzyDataObj_sagittal, &widmGaitDataObj_sagittal, widmModuleObj);
	InitValueSetting(&widmAngleDataObj_frontal, &widmSensorDataObj_frontal, &widmFuzzyDataObj_frontal, &widmGaitDataObj_frontal, widmModuleObj);

	if (tinitIMURes == IOIF_MAGNETO_STATUS_OK) {
		gaitCtrlTask.errCode = 0;
		StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_OFF);
	}
}


/* ------------------- INITIALIZATION ------------------- */
/* Setting for initial parameters */
static void InitValueSetting(WIDM_AngleData_t* widmAngleData, WIDM_SensorData_t* widmSensorData, WIDM_FuzzyData_t* widmFuzzyData, WIDM_GaitData_t* widmGaitData, WIDM_Module_t widmModule)
{
	// widmAngleData->initOffsetIMUDeg	= 65.0f;

	widmFuzzyData->fc_low 			= 0.5;		// experimently decided
	widmFuzzyData->fc_high 			= 3;		// experimently decided
	widmFuzzyData->m0				= 8;		// Threshold Value (maybe middle value)
	widmFuzzyData->sensitivity		= 0.6;		// Sensor Sensitivity (natural logarithm)
	widmFuzzyData->scaling_mk		= 25;		// experimently decided	
	widmFuzzyData->fc 				= widmFuzzyData->fc_high;
	widmFuzzyData->fc_calib			= widmFuzzyData->fc_high;
	widmFuzzyData->fc_LPF[1]		= widmFuzzyData->fc_high;
	widmFuzzyData->fc_calib_LPF[1]	= widmFuzzyData->fc_high;
}

/* Select the Joint and Sensor method */
static void ModelSelection(uint8_t nodeID)
{
	switch (nodeID) {

		// H10
		case (NODE_ID_RH_SAG):	// RH
			widmAttachCaseObj 	= WIDM_H10_RIGHT_SAG;
			widmModuleObj 		= WIDM_H10_SAM;
			break;
		case (NODE_ID_LH_SAG):	// LH
			widmAttachCaseObj 	= WIDM_H10_LEFT_SAG;
			widmModuleObj 		= WIDM_H10_SAM;
			break;

		// K10
		case (NODE_ID_RK):		// RK
			widmAttachCaseObj 	= WIDM_K10_RIGHT_SAG;
			widmModuleObj 		= WIDM_K10_SAM;
			break;
		case (NODE_ID_LK):		// LK
			widmAttachCaseObj 	= WIDM_K10_LEFT_SAG;
			widmModuleObj 		= WIDM_K10_SAM;
			break;

		// CM
		case (NODE_ID_CM): 		// CM
			widmAttachCaseObj 	= WIDM_SUIT_CM_FRONTAL;
			widmModuleObj		= WIDM_SUIT_CM;
			
		default:
			break;
	}
}

/* Reset Value Zero */
static void ResetDataObj(void)
{
	widmSensorDataObj_sagittal 		= 	(WIDM_SensorData_t){0};
	widmFuzzyDataObj_sagittal 		= 	(WIDM_FuzzyData_t){0};
	widmAngleDataObj_sagittal		=   (WIDM_AngleData_t){0};
	// widmNormDataObj_sagittal 		= 	(WIDM_NormData_t){0};
	widmGaitDataObj_sagittal 		= 	(WIDM_GaitData_t){0};
	// widmThresDataObj_sagittal 		= 	(WIDM_ThresData_t){0};

	widmSensorDataObj_frontal		= 	(WIDM_SensorData_t){0};
	widmFuzzyDataObj_frontal 		= 	(WIDM_FuzzyData_t){0};
	widmAngleDataObj_frontal		=   (WIDM_AngleData_t){0};
	// widmNormDataObj_frontal 		= 	(WIDM_NormData_t){0};
	widmGaitDataObj_frontal 		= 	(WIDM_GaitData_t){0};
	// widmThresDataObj_frontal 		= 	(WIDM_ThresData_t){0};
}

/* ------------------- RUN REAL-TIME FUNCTIONS ------------------- */
static void RunRealTimeFuncs(void)
{
	CalNeutralPosture();
	AutoCalibrationIMU();
}

static void CalNeutralPosture(void)
{
	if (CalNeutralPostureCmd == 1) {
		if (calNeutralPostureCnt < 1000) {
			tAccumulateThighAngle += widmAngleDataObj_frontal.thighDegFinal_calib[0];
			calNeutralPostureCnt++;
		} else if (calNeutralPostureCnt == 1000) {
			widmAngleDataObj_frontal.NeutralPostureBias = tAccumulateThighAngle / (float)(calNeutralPostureCnt);
			calNeutralPostureCnt = 0;
			CalNeutralPostureCmd = 0;
			tAccumulateThighAngle = 0.0f;
			neutralizedFlag = NEUTRAL_POSTURE_CALIBRATED;
			// Send_NOTI((uint32_t)1);
		}
	}
}

static void initAutoCalParam(WIDM_SensorData_t* sensorData, IOIF_AccGyro_Data_t* imu6AxisData)
{
	memset(imu6AxisData, 0, sizeof(IOIF_AccGyro_Data_t));

	memset(&sensorData->accX[0], 0, sizeof(sensorData->accX));
	memset(&sensorData->accY[0], 0, sizeof(sensorData->accY));
	memset(&sensorData->accZ[0], 0, sizeof(sensorData->accZ));

	memset(&sensorData->gyrX[0], 0, sizeof(sensorData->gyrX));
	memset(&sensorData->gyrY[0], 0, sizeof(sensorData->gyrY));
	memset(&sensorData->gyrZ[0], 0, sizeof(sensorData->gyrZ));

	memset(&sensorData->accX_calibrated[0], 0, sizeof(sensorData->accX_calibrated));
	memset(&sensorData->accY_calibrated[0], 0, sizeof(sensorData->accY_calibrated));
	memset(&sensorData->accZ_calibrated[0], 0, sizeof(sensorData->accZ_calibrated));

	memset(&sensorData->gyrX_calibrated[0], 0, sizeof(sensorData->gyrX_calibrated));
	memset(&sensorData->gyrY_calibrated[0], 0, sizeof(sensorData->gyrY_calibrated));
	memset(&sensorData->gyrZ_calibrated[0], 0, sizeof(sensorData->gyrZ_calibrated));

	sensorData->sqrtAcc = 0;
	sensorData->m0Raw = 0;
	sensorData->m0RawSum = 0;

	sensorData->degAccArctan = 0;
	memset(&sensorData->degGyrIntegral[0], 0, sizeof(sensorData->degGyrIntegral));

	memset(&sensorData->phi[0], 0, sizeof(sensorData->phi));
	memset(&sensorData->PhiMatrixSum[0], 0, sizeof(sensorData->PhiMatrixSum));
	memset(&sensorData->ThetaVectorSum[0], 0, sizeof(sensorData->ThetaVectorSum));
	memset(&sensorData->invPhiMatrix[0][0], 0, sizeof(sensorData->invPhiMatrix));
}

static void AutoCalibrationIMU(void)
{
	static uint32_t revoceryCnt_AutoCal = 0;
	// TODO : Auto Calibration Must be in Slow Motion!
	if (autoCalibSensorCmd == 1) {
		if (oneTimeInitAutoCal == true) {
			initAutoCalParam(&widmSensorDataObj_frontal, &accgyroDataObj);
			oneTimeInitAutoCal = false;
		} else {
			uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&accgyroDataObj);
			if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
				I2CfailedAutoCnt++;
				/* Recovery 6axis IMU */
				if (revoceryCnt_AutoCal > IOIF_ACCGYRO_RECOVERY_TIME_INTERVAL) {
					// TODO : Error Handling

					t6AxisRes = IOIF_AccGyro_Init();
					gaitCtrlTask.errCode = ERROR_IMU_SENSOR_GET_VALUE;
					revoceryCnt_AutoCal = 0;
				}
				revoceryCnt_AutoCal++;
			} else {
				// TODO : Error Handling 필요
			}
			// UpdateSensorForAutoCal(&widmSensorDataObj_frontal, &accgyroDataObj, widmAttachCaseObj);

			AccumultateSqrtSum(&widmSensorDataObj_frontal);
			AccumulateGyroMatrix(&widmSensorDataObj_frontal, widmAttachCaseObj, autoCalibCnt, GAIT_CONTROL_PERIOD);

			autoCalibCnt++;
		}
	} else if (autoCalibSensorCmd == 2) { // Auto-Calibration Sensor Parameters
		if (autoCalibCnt > 0) {
			GetAccMeanVal(&widmSensorDataObj_frontal, autoCalibCnt);
			GetGyroParam(&widmSensorDataObj_frontal);
			autoCalibSensorCmd = 0;
			autoCalibCnt = 0;
			autoCalibratedFlag = 1;
			oneTimeInitAutoCal = true;

			IOIF_EraseFlash(IOIF_FLASH_SENSOR_CAL_DATA_ADDR, IOIF_ERASE_ONE_SECTOR);
			writeAddr = IOIF_FLASH_START_RS_ADDR;
			float memArr8[READ_FLASH_ARRAY_SIZE] = {0};
			memcpy(&memArr8[0], &widmSensorDataObj_sagittal.scaling_gyro,                       IOIF_FLASH_WRITE_SIZE_4B);
			memcpy(&memArr8[1], &widmSensorDataObj_sagittal.bias_gyro,                          IOIF_FLASH_WRITE_SIZE_4B);
			memcpy(&memArr8[2], &widmSensorDataObj_sagittal.accMeanVal_calibrated,              IOIF_FLASH_WRITE_SIZE_4B);
			IOIF_WriteFlash(writeAddr, memArr8);
		} else {
			// TODO : Error Handling 필요
			oneTimeInitAutoCal = true;
			autoCalibSensorCmd = 1;
		}
	} else {
		oneTimeInitAutoCal = true;
	}
}

/* ------------------- AUTO CALIBRATION SENSOR ------------------- */
static void UpdateSensorForAuto(WIDM_SensorData_t* sensorData, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	// TODO : Auto Calibration !!
	// Initialize filtered gyroZ
	sensorData->accX[0] = imu6AxisData->accX * ACC_GRAVITY_CONSTANT;
	sensorData->accY[0] = imu6AxisData->accY * ACC_GRAVITY_CONSTANT;
	sensorData->gyrZ[0] = imu6AxisData->gyrZ;

	// For Raw Sensor Data Error Check
	static uint8_t isInitialized_DSN = 1;
	if (isInitialized_DSN == 1) {
		sensorData->accX[1] = sensorData->accX[0];
		sensorData->accY[1] = sensorData->accY[0];
		sensorData->gyrZ[1] = sensorData->gyrZ[0];
		isInitialized_DSN = 0;
	}
	DetectSensorNoise(&sensorData->accX[0], sensorData->accX[1], &sensorData->accXLastValidData,
						&sensorData->accXAlgorithmActivated, ACC_X_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->accXNoiseCnt);
	DetectSensorNoise(&sensorData->accY[0], sensorData->accY[1], &sensorData->accYLastValidData,
						&sensorData->accYAlgorithmActivated, ACC_Y_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->accYNoiseCnt);
	DetectSensorNoise(&sensorData->gyrZ[0], sensorData->gyrZ[1], &sensorData->gyrZLastValidData,
						&sensorData->gyrZAlgorithmActivated, GYR_Z_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->gyrZNoiseCnt);

	// previous value save in WIDM_UpdateBuffer functions
}

// Function to calculate mean value for accelerometer with LPF applied
static void AccumultateSqrtSum(WIDM_SensorData_t* sensorData)
{
	static bool isInitialized_m0Rwa = 1;
	if (isInitialized_m0Rwa == 1) {
		sensorData->m0Raw = WIDM_SquareRootSum(sensorData->accX[0], sensorData->accY[0]);
		isInitialized_m0Rwa = 0;
	}

	sensorData->sqrtAcc = WIDM_SquareRootSum(sensorData->accX[0], sensorData->accY[0]);
	sensorData->m0Raw = 0.95 * sensorData->m0Raw + 0.05 * sensorData->sqrtAcc;
	sensorData->m0RawSum += sensorData->m0Raw;
}

static void GetAccMeanVal(WIDM_SensorData_t* sensorData, uint32_t sampleSize)
{
	sensorData->accMeanVal_calibrated = sensorData->m0RawSum / sampleSize;
}

/**
 * @brief Update degree values and accumulate matrices for gyro calibration
 *
 * @param sensorData Pointer to sensor data structure
 * @param attachCase Attachment case for specific settings
 * @param sampleCnt Current sample count
 * @param samplingPeriod Sampling period (time interval between samples)
 */
static void AccumulateGyroMatrix(WIDM_SensorData_t* sensorData, WIDM_AttachCase_t attachCase, uint32_t sampleCnt, float samplingPeriod)
{
    sensorData->degAccArctan =  SafeAtan2f((sensorData->accX[0]), (sensorData->accY[0])) * (180 / WIDM_PI);
    sensorData->degGyrIntegral[0] = sensorData->degGyrIntegral[1] + sensorData->gyrZ[0] * samplingPeriod;

    // Update PhiMatrixSum and ThetaVectorSum
    sensorData->phi[0] = sensorData->degGyrIntegral[0];
    sensorData->phi[1] = (-1.0f) * (float)sampleCnt * samplingPeriod;
    sensorData->phi[2] = 1;

    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            sensorData->PhiMatrixSum[i][j] += sensorData->phi[i] * sensorData->phi[j];
        }
        sensorData->ThetaVectorSum[i] += sensorData->phi[i] * sensorData->degAccArctan;
    }

    sensorData->degGyrIntegral[1] = sensorData->degGyrIntegral[0];
}

/**
 * @brief Auto-calibrate the gyroscope parameters
 *
 * @param sensorData Pointer to sensor data structure
 * @param sampleSize Total number of samples collected
 * @param mean_value Pointer to mean value of the accelerometer
 * @param samplingPeriod Sampling period (time interval between samples)
 * @param attachCase Attachment case for specific settings
 */
static void GetGyroParam(WIDM_SensorData_t* sensorData)
{
	// Calculate the determinant of PhiMatrixSum
	sensorData->matrixDeterminant = sensorData->PhiMatrixSum[0][0] *
							 (sensorData->PhiMatrixSum[1][1] * sensorData->PhiMatrixSum[2][2] - sensorData->PhiMatrixSum[1][2] * sensorData->PhiMatrixSum[2][1])
                             - sensorData->PhiMatrixSum[0][1] *
							 (sensorData->PhiMatrixSum[1][0] * sensorData->PhiMatrixSum[2][2] - sensorData->PhiMatrixSum[1][2] * sensorData->PhiMatrixSum[2][0])
                             + sensorData->PhiMatrixSum[0][2] *
							 (sensorData->PhiMatrixSum[1][0] * sensorData->PhiMatrixSum[2][1] - sensorData->PhiMatrixSum[1][1] * sensorData->PhiMatrixSum[2][0]);

    // Calculate the inverse of PhiMatrixSum
    CalculateInverse3x3(sensorData->PhiMatrixSum, sensorData->invPhiMatrix, sensorData->matrixDeterminant);

	// Calculate gyro calibration output
	memset(&sensorData->GyrCalOutput, 0, sizeof(sensorData->GyrCalOutput)); // 초기화

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            sensorData->GyrCalOutput[i] += sensorData->invPhiMatrix[i][j] * sensorData->ThetaVectorSum[j];
        }
    }

    sensorData->scaling_gyro = sensorData->GyrCalOutput[0];
    sensorData->bias_gyro = sensorData->GyrCalOutput[1] / sensorData->GyrCalOutput[0];
    sensorData->meanVal_gyro = sensorData->GyrCalOutput[2];
}

static void CalculateInverse3x3(float matrix[3][3], float invMatrix[3][3], float det)
{
    invMatrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / det;
    invMatrix[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) / det;
    invMatrix[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / det;
    invMatrix[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) / det;
    invMatrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / det;
    invMatrix[1][2] = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) / det;
    invMatrix[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / det;
    invMatrix[2][1] = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) / det;
    invMatrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / det;
}

/* ------------------- GAIT TOTAL FUNCTION ------------------- */
// 안전한 atan2f 함수
static float SafeAtan2f(float a, float b)
{
	const float EPSILON = 0.0001f; // 작은 임계값
	if (fabsf(a) < EPSILON) {
		a = EPSILON; // x가 너무 작은 경우, 작은 값으로 대체
	}
	if (fabsf(b) < EPSILON) {
		b = EPSILON; // y가 너무 작은 경우, 작은 값으로 대체
	}
	return atan2f(a, b);
}

/*
*The function UpdateSensorRawData_sagittal updates the IMU raw values.
*/
static void UpdateSensorRawData_sagittal(WIDM_SensorData_t* widmSensorData_sagittal, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	// Initialize filtered gyroZ
	widmSensorData_sagittal->accX[0] = -(imu6AxisData->accX * ACC_GRAVITY_CONSTANT * cos(WIDM_PI/3) + imu6AxisData->accZ * ACC_GRAVITY_CONSTANT * sin(WIDM_PI/3));
	widmSensorData_sagittal->accY[0] = imu6AxisData->accY * ACC_GRAVITY_CONSTANT;
	widmSensorData_sagittal->gyrZ[0] = -1.561*(imu6AxisData->gyrZ * cos(-WIDM_PI/3) + imu6AxisData->gyrX * sin(-WIDM_PI/6));

	// For Raw Sensor Data Error Check
	DetectSensorNoise(&widmSensorData_sagittal->accX[0], widmSensorData_sagittal->accX[1], &widmSensorData_sagittal->accXLastValidData, &accXAlgorithmActivated, ACC_X_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &accXNoiseCnt);
	DetectSensorNoise(&widmSensorData_sagittal->accY[0], widmSensorData_sagittal->accY[1], &widmSensorData_sagittal->accYLastValidData, &accYAlgorithmActivated, ACC_Y_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &accYNoiseCnt);
	DetectSensorNoise(&widmSensorData_sagittal->gyrZ[0], widmSensorData_sagittal->gyrZ[1], &widmSensorData_sagittal->gyrZLastValidData, &gyrZAlgorithmActivated, GYR_Z_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &gyrZNoiseCnt);
}

/*
*The function UpdateSensorRawData_frontal updates the IMU raw values.
*/
static void UpdateSensorRawData_frontal(WIDM_SensorData_t* widmSensorData_frontal, IOIF_AccGyro_Data_t* imu6AxisData, WIDM_AttachCase_t widmAttachCase)
{
	// Initialize filtered gyroZ
	widmSensorData_frontal->accX[0] = -(imu6AxisData->accX * ACC_GRAVITY_CONSTANT * cos(-WIDM_PI/6) + imu6AxisData->accZ * ACC_GRAVITY_CONSTANT * sin(-WIDM_PI/6));
	widmSensorData_frontal->accY[0] = imu6AxisData->accY * ACC_GRAVITY_CONSTANT;
	widmSensorData_frontal->gyrZ[0] = -1.561*(imu6AxisData->gyrZ * cos(WIDM_PI/6) + imu6AxisData->gyrX * sin(WIDM_PI/3));

	// For Raw Sensor Data Error Check
	DetectSensorNoise(&widmSensorData_frontal->accX[0], widmSensorData_frontal->accX[1], &widmSensorData_frontal->accXLastValidData, &accXAlgorithmActivated, ACC_X_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &accXNoiseCnt);
	DetectSensorNoise(&widmSensorData_frontal->accY[0], widmSensorData_frontal->accY[1], &widmSensorData_frontal->accYLastValidData, &accYAlgorithmActivated, ACC_Y_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &accYNoiseCnt);
	DetectSensorNoise(&widmSensorData_frontal->gyrZ[0], widmSensorData_frontal->gyrZ[1], &widmSensorData_frontal->gyrZLastValidData, &gyrZAlgorithmActivated, GYR_Z_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &gyrZNoiseCnt);
}

/*
 *Function to execute the time-varying complementary filter (with Fuzzy Logic - fc)
*/
static void RunTvcfFilter(WIDM_SensorData_t* widmSensorData, WIDM_AngleData_t* widmAngleData, WIDM_FuzzyData_t* widmFuzzyData, float samplingPeriod, WIDM_AttachCase_t widmAttachCase)
{
    /* Apply time-varying complementary filter on the sensor data using fuzzy logic(fc) and update the thigh angle parameters */
	// WIDM_RunTVCF(widmSensorData, widmAngleData, widmFuzzyData->fc_LPF[0], samplingPeriod, widmAttachCase);

    /* Update the unfiltered thigh angle to be the same as the filtered thigh angle */
	widmAngleData->degTvcf[0] = widmAngleData->angleTVCF_Deg;

    // Calculate the raw velocity
    widmAngleData->velDegTvcf[0] = (widmAngleData->degTvcf[0] - widmAngleData->degTvcf[1]) / samplingPeriod;
}

// threshold checking for all sensor values
static void DetectSensorNoise(float* currentVal, float prevVal, float* lastValidVal, uint8_t* algorithmActivated, uint16_t threshold, uint32_t* errCode, uint32_t* noiseCounter)
{
    // 현재 값과 이전 값을 비교하여 노이즈 감지
    if (abs(*currentVal - prevVal) >= threshold) {
        if (!(*algorithmActivated)) {
            // 첫 번째 노이즈 감지
            *lastValidVal = prevVal; // 마지막 유효한 값을 저장
            *currentVal = prevVal; // 현재 값을 이전 값으로 덮어씀
            *algorithmActivated = 1; // 알고리즘 활성화
            *errCode = ERROR_IMU_SENSOR_RAW_OVER_THRESHOLD; // 노이즈 감지 에러 코드 설정
			*noiseCounter = 1; // 노이즈 카운터 초기화 및 증가
        } else {
            // 연속된 노이즈 감지
			(*noiseCounter)++;
            *currentVal = *lastValidVal; // 마지막 유효한 값으로 덮어씀
            *errCode = ERROR_IMU_SENSOR_RAW_OVER_THRESHOLD; // 노이즈 감지 에러 코드 설정 유지
        }
    } else {
        // 노이즈가 감지되지 않으면 정상적인 신호로 간주
        *algorithmActivated = 0; // 알고리즘 비활성화
        *lastValidVal = *currentVal; // 마지막 유효한 값을 현재 값으로 업데이트
    }
}

static int RunGaitFunc(void)
{
	uint8_t t6AxisRes = IOIF_AccGyro_GetValue(&accgyroDataObj);
	if (t6AxisRes != IOIF_ACCGYRO_STATUS_OK) {
		StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ERROR);
		return t6AxisRes;
	}

	float local_acc[3] = {
		accgyroDataObj.accX,
        accgyroDataObj.accY,
        accgyroDataObj.accZ
		};
	float local_gyr[3] = {
		accgyroDataObj.gyrX,
        accgyroDataObj.gyrY,
        accgyroDataObj.gyrZ
	};
	float global_acc[3];
	float global_gyr[3];

	CMLocalToGlobal(local_acc, global_acc);
	CMLocalToGlobal(local_gyr, global_gyr);

	widmSensorDataObj_sagittal.accGlobalX = global_acc[0] * 9.81;
	widmSensorDataObj_sagittal.accGlobalY = global_acc[1] * 9.81;
	widmSensorDataObj_sagittal.accGlobalZ = global_acc[2] * 9.81;

	widmSensorDataObj_sagittal.gyrGlobalX = global_gyr[0];
	widmSensorDataObj_sagittal.gyrGlobalY = global_gyr[1];
	widmSensorDataObj_sagittal.gyrGlobalZ = global_gyr[2];

	UpdateSensorRawData_sagittal(&widmSensorDataObj_sagittal, &accgyroDataObj, widmAttachCaseObj);
	UpdateSensorRawData_frontal(&widmSensorDataObj_frontal, &accgyroDataObj, widmAttachCaseObj);

	RunTvcfFilter(&widmSensorDataObj_sagittal, &widmAngleDataObj_sagittal, &widmFuzzyDataObj_sagittal, WIDM_CONTROL_PERIOD, widmAttachCaseObj);
	RunTvcfFilter(&widmSensorDataObj_frontal, &widmAngleDataObj_frontal, &widmFuzzyDataObj_frontal, WIDM_CONTROL_PERIOD, widmAttachCaseObj);

	widm_accX_sagittal = widmSensorDataObj_sagittal.accX[0];
	widm_accY_sagittal = widmSensorDataObj_sagittal.accY[0];
	widm_gyrZ_sagittal = widmSensorDataObj_sagittal.gyrZ[0];
	widm_degTvcf_sagittal = widmAngleDataObj_sagittal.degTvcf[0];
	widm_velDegTvcf_sagittal = widmAngleDataObj_sagittal.velDegTvcf[0];
	widm_degAcc_artan_sagittal = widmAngleDataObj_sagittal.degAcc_arctan[0];
	widm_fc_sagittal = widmFuzzyDataObj_sagittal.fc_LPF[0];

	widm_accX_frontal = widmSensorDataObj_frontal.accX[0];
	widm_accY_frontal = widmSensorDataObj_frontal.accY[0];
	widm_gyrZ_frontal = widmSensorDataObj_frontal.gyrZ[0];
	widm_degTvcf_frontal = widmAngleDataObj_frontal.degTvcf[0];
	widm_velDegTvcf_frontal = widmAngleDataObj_frontal.velDegTvcf[0];
	widm_degAcc_artan_frontal = widmAngleDataObj_frontal.degAcc_arctan[0];
	widm_fc_frontal = widmFuzzyDataObj_frontal.fc_LPF[0];

	return 0;
}

static void CMLocalToGlobal(float local[3], float global[3]) 
{
	// 회전 행렬 정의
    float R[3][3] = {
        {0,  0,  1},  // x_global = z_local
        {0, -1,  0},  // y_global = -y_local
        {1,  0,  0}   // z_global = x_local
    };

    // 행렬 곱 수행
    for (int i = 0; i < 3; i++) {
        global[i] = 0.0;
        for (int j = 0; j < 3; j++) {
            global[i] += R[i][j] * local[j];
        }
    }
}

/* ----------------------- ROUTINE ------------------------ */
/* --------------------- SDO CALLBACK --------------------- */

/* ------------------- STATIC FUNCTIONS ------------------- */
// Function to scale a float to a 16-bit integer type(uint16)
static uint16_t ScaleFloatToUInt16(float value, float scaleFactor)
{
    // Scale the float value
    uint16_t scaledValue = (uint16_t)(value * DATA_CONV_CONST_UINT16 / scaleFactor);

    return scaledValue;
}

#endif /* SUIT_MINICM_ENABLED */
