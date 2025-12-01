#include "gait_ctrl.h"

/**
 *-----------------------------------------------------------
 *      TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

/* ------------------- Gait Ctrl Task ------------------- */
TaskObj_t gaitCtrlTask;
uint32_t gaitCtrlLoopCnt;
float gaitCtrlTimeElap;

/* ---------- IMU Sensor DAQ & I2C Recover Logic ---------- */
IOIF_6AxisData_t imuAccGyrRawDataObj;
IOIF_MagData_t imuMagRawDataObj;

static uint8_t testImu6AxisRes 	= IOIF_I2C_STATUS_OK;
// static uint8_t testImu3AxisRes 	= IOIF_I2C_STATUS_OK;

uint8_t I2CInitState;
uint8_t I2CfailedInitCnt;
uint16_t I2CfailedStateOffCnt;
uint16_t I2CfailedAutoCnt;
uint32_t I2CfailedStateEnCnt;
// static bool IsRecoverI2C1 = false;
static bool IsRecoverI2C2 = false;
#if defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
static bool IsRecoverI2C3 = false;
#endif

/* ------------------- IMU Mount Positions Based on Node ID ------------------- */
IMUAttachCase_t IMUAttachCase;
uint8_t	moduleNodeID;

/* ------------------- IMU Sensor Calibration ------------------- */
IMUProcessedData_t imuProcessedDataObj;
uint32_t autoCalibCnt;
uint8_t autoCalibSensorCmd;
uint32_t autoCalibratedFlag;

bool oneTimeInitAutoCal = true;

/* ------------------- Sensor Fusion Algorithm ------------------- */
VQF_IMUData_t imuVQFDataObj;
VQF_t _vqf_md;
FuzzyTVCFData_t fuzzyTVCFDataObj_Sagittal;
FilteredAngleData_t filteredAngleDataObj_Sagittal;

/* ------------------- Scaling Data For PDO ------------------- */
ScaledData floatTo2ByteData;

/* ------------------- Neutral Posture Calculation ------------------- */
uint32_t calNeutralPostureCnt;
uint8_t CalNeutralPostureCmd;
int16_t neutralOffset_int16;
int16_t UserNeutralPosture_int16;
uint32_t neutralizedFlag;

float tAccumulateThighAngle = 0.0;
float tAccumuulateKneeAngle = 0.0;


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

/* ------------------- IMU Mount Positions Based on Node ID ------------------- */
static int NodeIDCheck(void);
static void ModelSelection(uint8_t nodeID);

/* ------------------- Set & Reset Init Value ------------------- */
static void InitValueSetting(FilteredAngleData_t* angleData_sagittal, IMUProcessedData_t* sensorData,
		FuzzyTVCFData_t* fuzzyTVCFData_sagittal, VQF_IMUData_t* imuVQFData, IMUAttachCase_t attachCase);
static void ResetDataObj(void);

/* ------------------- ROUTINE ------------------- */
static int Ent_GaitTotalFunc(void);
static int Run_GaitTotalFunc(void);
static int Ext_GaitTotalFunc(void);

static int RunGetIMUFunction(void);

/* ------------------- IMU INITIALIZATION ------------------- */
static uint8_t InitializeIMU(void);

/* ------------------- Sensor Fusion Algorithm ------------------- */
static void UpdateIMUData(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData, IMUAttachCase_t attachCase);
static void CheckIMUNoise(float* currentVal, float prevVal, float* lastValidVal, uint8_t* algorithmActivated, uint16_t threshold, uint32_t* errCode, uint32_t* noiseCounter);
static void SavePrevBuff(IMUProcessedData_t* sensorData);

static float SafeAtan2f(float a, float b);

static void GetEncoderDegree(FilteredAngleData_t* filteredAngleData, IMUAttachCase_t attachCase);
static void CombineIMUnEncoderDegree(FilteredAngleData_t* filteredAngleData, VQF_IMUData_t* imuVQFData);
static void SetFinalThighSagittalDegVel(FilteredAngleData_t* filteredAngleData, VQF_IMUData_t* imuVQFData, IMUAttachCase_t attachCase);

/* ------------------- AUTO SENSOR CALIBRATION ------------------- */
static void AutoCalibrationIMU(void);

static void initAutoCalParam(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData);
static void UpdateSensorForAutoCal(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData, IMUAttachCase_t attachCase);
static void DetectIMUNoiseForAutoCal(IMUProcessedData_t* sensorData);

static void AccumultateSqrtSum(IMUProcessedData_t* sensorData);
static void AccumulateGyroMatrix(IMUProcessedData_t* sensorData, IMUAttachCase_t attachCase, uint32_t sampleCnt, float samplingPeriod);

static void GetAccMeanVal(IMUProcessedData_t* sensorData, uint32_t sampleSize);
static void GetGyroParam(IMUAttachCase_t attachCase,IMUProcessedData_t* sensorData);
static void CalInverse3x3(float matrix[3][3], float invMatrix[3][3], float det);

/* ------------------- Neutral Posture Calculation ------------------- */
static void CheckNeutralized(void);
static void CalNeutralPosture(void);

/* ------------------- Command Driven Functions ------------------- */
static void ExecuteCmdDrivenFunctions(void);

/* ------------------- Scaling Data For PDO ------------------- */
static void ScalingForPDO(ScaledData* Scaled2ByteData, IMUProcessedData_t* sensorData, FuzzyTVCFData_t* fuzzyTVCFData_sagittal,
		FilteredAngleData_t* angleData_sagittal, VQF_IMUData_t* imuVQFData);

/* ------------------- SDO CALLBACK ------------------- */
// Auto Calibration Command
static void SetAutoCalCmd(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
// Is it Neutral Posture Calculated initial state?
static void SetIsNeutralized(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
// Neutral Posture Calculation
static void SetNeutralPostureCalCmd(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
// Neutral Posture Offset
static void SetNeutralPostureOffset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
// User Info's Neutral Posture from Tablet
static void SetUserNeutralPostureAngle(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(gaitCtrlTask)
void InitGaitCtrl(void)
{
	InitTask(&gaitCtrlTask);

	/* Checking Node ID */
	moduleNodeID = NodeIDCheck();

	/* State Definition */
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&gaitCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_GAIT_TOTAL_FUNCTION, 		Ent_GaitTotalFunc, 		Run_GaitTotalFunc,		Ext_GaitTotalFunc);
	TASK_CREATE_ROUTINE(&gaitCtrlTask, ROUTINE_ID_GAIT_6AXIS_GETVALUE, 		NULL, 					RunGetIMUFunction,		NULL);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_GAIT);

	// PDO
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_THIGH_ANGLE_DEG,		DOP_INT16,	1,    &floatTo2ByteData.ThighDeg_sagittal);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_THIGH_VEL_DEG,			DOP_INT16,	1,    &floatTo2ByteData.ThighVelDeg_sagittal);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_INC_POS_DEG,			DOP_INT16,	1,    &floatTo2ByteData.IncDeg_sagittal);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_INC_VEL_DEG,			DOP_INT16,	1,    &floatTo2ByteData.IncVelDeg_sagittal);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_SAGITTAL,		DOP_INT16,	1,    &floatTo2ByteData.IMU_BodyAngle_Sagittal);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_FRONTAL,		DOP_INT16,	1,    &floatTo2ByteData.IMU_BodyAngle_Frontal);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_IMU_VQF_TRANSVERSE,		DOP_INT16,	1,    &floatTo2ByteData.IMU_BodyAngle_Transverse);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X,					DOP_INT16,	1,    &floatTo2ByteData.AccX);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y,					DOP_INT16,	1,    &floatTo2ByteData.AccY);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z,					DOP_INT16,	1,    &floatTo2ByteData.AccZ);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X,					DOP_INT16,	1,    &floatTo2ByteData.GyrX);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y,					DOP_INT16,	1,    &floatTo2ByteData.GyrY);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z,					DOP_INT16,	1,    &floatTo2ByteData.GyrZ);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.AccX_calib);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.AccY_calib);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.AccZ_calib);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.GyrX_calib);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.GyrY_calib);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z_CALIB,			DOP_INT16,	1,    &floatTo2ByteData.GyrZ_calib);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_X_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.AccX_global);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Y_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.AccY_global);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_Z_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.AccZ_global);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_X_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.GyrX_global);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Y_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.GyrY_global);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYR_Z_GLOBAL,			DOP_INT16,	1,    &floatTo2ByteData.GyrZ_global);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_ACC_MEAN_VALUE,			DOP_FLOAT32,	1,	&imuProcessedDataObj.accMeanVal_calibrated);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYRO_SCALING,			DOP_FLOAT32,	1,	&imuProcessedDataObj.scaling_gyro);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_GYRO_BIAS,				DOP_FLOAT32,	1,	&imuProcessedDataObj.bias_gyro);

	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_WC_TVCF,				DOP_UINT16,	1,    &floatTo2ByteData.fc_sagittal);
	DOP_CreatePDO(TASK_ID_GAIT,	PDO_ID_GAIT_WC_TVCF_CALIB,			DOP_UINT16,	1,    &floatTo2ByteData.fc_calib_sagittal);

	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_GAIT)
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_AUTO_CALIB_SENSOR, 				DOP_UINT8, 	SetAutoCalCmd);
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_IS_NEUTRALIZED, 				DOP_UINT8, 	SetIsNeutralized);
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_NEUTRAL_BIAS_CMD, 				DOP_UINT8, 	SetNeutralPostureCalCmd);
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_NEUTRAL_BIAS_OFFSET, 			DOP_INT16, 	SetNeutralPostureOffset);
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_NEUTRAL_INITIAL_DEG, 			DOP_INT16, 	Send_NeutralPostureDeg);
	DOP_CreateSDO(TASK_ID_GAIT, 	SDO_ID_GAIT_SET_USER_NEUTRAL_POSTURE_ANGLE, 	DOP_INT16, 	SetUserNeutralPostureAngle);

	/* Check Neutralized */
	CheckNeutralized();

	/* Select correct Module here */
	ModelSelection(moduleNodeID);

	/* Init 6axis & 3axis IMU */
	I2CInitState = InitializeIMU();

	/* Initial stage of get angle */
	InitValueSetting(&filteredAngleDataObj_Sagittal, &imuProcessedDataObj,
			&fuzzyTVCFDataObj_Sagittal, &imuVQFDataObj, IMUAttachCase);

	/* Timer 2 Callback Allocation (IMU) 1ms Period */
	if (IOIF_StartTimIT(IOIF_TIM2) > 0) {
		// TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM2, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunGaitCtrl, NULL);
}

void RunGaitCtrl(void* params)
{
	/* Loop Start Time Check */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* Run Device */
	RunTask(&gaitCtrlTask);
	ExecuteCmdDrivenFunctions();

	/* Elapsed Time Check */
	gaitCtrlTimeElap = DWT->CYCCNT / 480;	// in microsecond
}

/* ------------------- SCALING DATA FOR PDO COMMUNICATION ------------------- */
// Function to scale a float to a 16-bit integer type(int16)
int16_t ScaleFloatToInt16(float value, float scaleFactor)
{
	// Scale the float value
	int16_t scaledInt16Value = (int16_t)(value * DATA_CONV_CONST_INT16 / scaleFactor);

	return scaledInt16Value;
}

// Function to scale a float to a 16-bit integer type(uint16)
uint16_t ScaleFloatToUInt16(float value, float scaleFactor)
{
	// Scale the float value
	uint16_t scaledUint16Value = (uint16_t)(value * DATA_CONV_CONST_UINT16 / scaleFactor);

	return scaledUint16Value;
}

// Function to scale int16 to a float type
float ScaleInt16ToFloat(int16_t value, float scaleFactor)
{
    // Scale the float value
    float scaledValue = (float)(value * scaleFactor / DATA_CONV_CONST_INT16);

    return scaledValue;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	static uint32_t calibCnt = 0;
	static uint32_t recoveryCnt = 0;
	static float tIMUAngleDeg = 0.0;
	static float tIMUAngleDeg_calib = 0.0;
	static float tAccumulatedAngle_IMUDeg = 0.0;
	static float tAccumulatedAngle_IMUDeg_calib = 0.0;
	static float tInitThighAngle_IMUDeg	 = 0.0;
	static float tInitThighAngle_IMUDeg_calib	 = 0.0;

	VQF_Init(&_vqf_md, 0.001, 0.001, 0.001);

	/* Thigh Angle Calibration & Set */
	if (I2CInitState == IOIF_I2C_STATUS_OK) {
		uint8_t t6AxisRes = IOIF_Get6AxisValue(&imuAccGyrRawDataObj);
		if (t6AxisRes == IOIF_I2C_STATUS_OK) {
			if (calibCnt < 100) {	// 1000 Samples
				if (IMUAttachCase == IMU_H10_LEFT_SAG || IMUAttachCase == IMU_K10_LEFT_SAG) {
					/* Calibrated */
					imuProcessedDataObj.accX_calibrated[0] = imuAccGyrRawDataObj.accX * imuProcessedDataObj.accMeanVal_calibrated;
					imuProcessedDataObj.accY_calibrated[0] = (-1.0f) * imuAccGyrRawDataObj.accY * imuProcessedDataObj.accMeanVal_calibrated;

					/* Not Calibrated */
					imuProcessedDataObj.accX[0] = imuAccGyrRawDataObj.accX * ACC_GRAVITY_CONSTANT;
					imuProcessedDataObj.accY[0] = (-1.0f) * imuAccGyrRawDataObj.accY * ACC_GRAVITY_CONSTANT;
				} else if (IMUAttachCase == IMU_H10_RIGHT_SAG || IMUAttachCase == IMU_K10_RIGHT_SAG) {
					/* Calibrated */
					imuProcessedDataObj.accX_calibrated[0] = (-1.0f) * imuAccGyrRawDataObj.accX * imuProcessedDataObj.accMeanVal_calibrated;
					imuProcessedDataObj.accY_calibrated[0] = (-1.0f) * imuAccGyrRawDataObj.accY * imuProcessedDataObj.accMeanVal_calibrated;

					/* Not Calibrated */
					imuProcessedDataObj.accX[0] = (-1.0f) * imuAccGyrRawDataObj.accX * ACC_GRAVITY_CONSTANT;
					imuProcessedDataObj.accY[0] = (-1.0f) * imuAccGyrRawDataObj.accY * ACC_GRAVITY_CONSTANT;
				}

				// 가속도 축 보정은 후 보정(최종 허벅지 각도 - 초기 IMU 기계적 각도 65도 in SUIT H10)
				tIMUAngleDeg = SafeAtan2f(imuProcessedDataObj.accY[0], imuProcessedDataObj.accX[0]) * 180.0 / M_PI;
				tIMUAngleDeg_calib = SafeAtan2f(imuProcessedDataObj.accY_calibrated[0], imuProcessedDataObj.accX_calibrated[0]) * 180.0 / M_PI;
				tAccumulatedAngle_IMUDeg += tIMUAngleDeg;
				tAccumulatedAngle_IMUDeg_calib += tIMUAngleDeg_calib;
			} else if (calibCnt == 100) {
				tInitThighAngle_IMUDeg = tAccumulatedAngle_IMUDeg / (float)calibCnt;
				tInitThighAngle_IMUDeg_calib = tAccumulatedAngle_IMUDeg_calib / (float)calibCnt;
				filteredAngleDataObj_Sagittal.degAccInitValue = tInitThighAngle_IMUDeg;
				filteredAngleDataObj_Sagittal.degAccInitValue_calib = tInitThighAngle_IMUDeg_calib;

				/* Set Init Value */
				filteredAngleDataObj_Sagittal.degAcc_arctan[1] = filteredAngleDataObj_Sagittal.degAccInitValue; // For LPF init Value
				filteredAngleDataObj_Sagittal.degAcc_1stLPF[1] = filteredAngleDataObj_Sagittal.degAccInitValue; // For LPF init Value
				filteredAngleDataObj_Sagittal.degAcc_calib_1stLPF[1] = filteredAngleDataObj_Sagittal.degAccInitValue_calib; // For LPF init Value
				filteredAngleDataObj_Sagittal.degAcc_calib_1stLPF_LPF[1] = filteredAngleDataObj_Sagittal.degAccInitValue_calib; // For LPF init Value

				filteredAngleDataObj_Sagittal.degGyr_Integral[1] = 0; // For Integral
				filteredAngleDataObj_Sagittal.degGyr_Integral_calibrated[1] = 0; // For Integral
				filteredAngleDataObj_Sagittal.degGyr_Integral_1stHPF[1] = 0; // For HPF init Value
				filteredAngleDataObj_Sagittal.degGyr_calb_Integral_1stHPF[1] = 0; // For HPF init Value
				filteredAngleDataObj_Sagittal.degGyr_calb_Integral_1stHPF_LPF[1] = 0; // For HPF init Value

				calibCnt = 0;
				StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_STANDBY);
			}

			recoveryCnt = 0;
		} else {
			/* Recovery 6axis IMU */
			if (recoveryCnt > I2C_REVOVERY_TIME_INTERVAL) {
				// TODO : Error Handling
				__HAL_RCC_I2C2_CLK_DISABLE();
				__HAL_I2C_DISABLE(&hi2c2);
				HAL_I2C_DeInit(&hi2c2);
				HAL_I2C_MspDeInit(&hi2c2);

				// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
				HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

				HAL_I2C_Init(&hi2c2);
				HAL_I2C_MspInit(&hi2c2);
				__HAL_I2C_ENABLE(&hi2c2);
				__HAL_RCC_I2C2_CLK_ENABLE();

				IsRecoverI2C2 = true;

				t6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
				if (t6AxisRes != IOIF_I2C_STATUS_OK) {
					I2CfailedStateOffCnt++;
				}
				recoveryCnt = 0;
			}

			calibCnt = 0;
			recoveryCnt++;
		}

		calibCnt++;
	} else {
		if (recoveryCnt > I2C_REVOVERY_TIME_INTERVAL) {
			// TODO : Error Handling
			__HAL_RCC_I2C2_CLK_DISABLE();
			__HAL_I2C_DISABLE(&hi2c2);
			HAL_I2C_DeInit(&hi2c2);
			HAL_I2C_MspDeInit(&hi2c2);

			// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
			HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

			HAL_I2C_Init(&hi2c2);
			HAL_I2C_MspInit(&hi2c2);
			__HAL_I2C_ENABLE(&hi2c2);
			__HAL_RCC_I2C2_CLK_ENABLE();

			IsRecoverI2C2 = true;

			I2CInitState = IOIF_Init6Axis(IOIF_I2C2);
			if (I2CInitState != IOIF_I2C_STATUS_OK) {
				I2CfailedStateOffCnt++;
			}
			recoveryCnt = 0;
		}
		recoveryCnt++;
	}
}

// For Debugging
uint8_t testTransitCmd = 0;
static void StateStandby_Run(void)
{
	if (testTransitCmd == 1) {
		PushRoutine(&gaitCtrlTask.routine, ROUTINE_ID_GAIT_TOTAL_FUNCTION);
		StateTransition(&mid_level_ctrl_task.stateMachine, TASK_STATE_ENABLE);
		StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_ENABLE);
		testTransitCmd = 0;
	}

	gaitCtrlLoopCnt = 0;
}

static void StateEnable_Ent(void)
{
	autoCalibCnt = 0;
	EntRoutines(&gaitCtrlTask.routine);
}

static void StateEnable_Run(void)
{
	RunRoutines(&gaitCtrlTask.routine);
	gaitCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
	ExtRoutines(&gaitCtrlTask.routine);
}

static void StateError_Run(void)
{
	/* Init 6axis & 3axis IMU */
	uint8_t tinitIMURes = InitializeIMU();
	/* Initial stage of get angle */
	ResetDataObj();
	InitValueSetting(&filteredAngleDataObj_Sagittal, &imuProcessedDataObj,
				&fuzzyTVCFDataObj_Sagittal, &imuVQFDataObj, IMUAttachCase);

	if (tinitIMURes == IOIF_I2C_STATUS_OK) {
		gaitCtrlTask.errCode = 0;
		StateTransition(&gaitCtrlTask.stateMachine, TASK_STATE_OFF);
	}
}

/* ------------------- IMU Mount Positions Based on Node ID ------------------- */
static int NodeIDCheck(void)
{
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_8);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_9);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_10);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BSP_GPIO_PIN_11);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* L30_MD_REV06_ENABLED & L30_MD_REV07_ENABLED & L30_MD_REV08_ENABLED */

#if defined(SUIT_MD_ENABLED)
	int temp1, temp2, temp3, temp4;
	temp1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_2);
	temp2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_3);
	temp3 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_4);
	temp4 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_F, IOIF_GPIO_PIN_5);
	return ((temp1<<3)|(temp2<<2)|(temp3<<1)|(temp4));
#endif /* SUIT_MD_ENABLED */
}

/* Select the Joint and Sensor method */
static void ModelSelection(uint8_t nodeID)
{
	switch (nodeID) {

		// H10
		case (NODE_ID_RH_SAG):	// RH
			IMUAttachCase 	= IMU_H10_RIGHT_SAG;
			break;
		case (NODE_ID_LH_SAG):	// LH
			IMUAttachCase 	= IMU_H10_LEFT_SAG;
			break;

		// K10
		case (NODE_ID_RK):		// RK
			IMUAttachCase 	= IMU_K10_RIGHT_SAG;
			break;
		case (NODE_ID_LK):		// LK
			IMUAttachCase 	= IMU_K10_LEFT_SAG;
			break;
			
		default:
			break;
	}
}

/* ------------------- Set & Reset Init Value ------------------- */
/* Setting for initial parameters */
static void InitValueSetting(FilteredAngleData_t* angleData_sagittal, IMUProcessedData_t* sensorData,
		FuzzyTVCFData_t* fuzzyTVCFData_sagittal, VQF_IMUData_t* imuVQFData, IMUAttachCase_t attachCase)
{
	if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_H10_LEFT_SAG) {	// H10
		angleData_sagittal->initOffsetIMUDeg	= 65.0f;	// (deg) H10 IMU tilted degree
		imuVQFData->sensor_to_body_offset.sagittal		= 25.0f;
		imuVQFData->sensor_to_body_offset.frontal		= 0.0f;
		imuVQFData->sensor_to_body_offset.transverse	= 0.0f;
	} else if (attachCase == IMU_K10_RIGHT_SAG || attachCase == IMU_K10_LEFT_SAG) {
		angleData_sagittal->initOffsetIMUDeg	= 90.0f;	// (deg) K10 IMU tilted degree
		imuVQFData->sensor_to_body_offset.sagittal		= 0.0f;
		imuVQFData->sensor_to_body_offset.frontal		= 0.0f;
		imuVQFData->sensor_to_body_offset.transverse	= 0.0f;
	}

	fuzzyTVCFData_sagittal->fc_low 				= 0.5;		// experimently decided
	fuzzyTVCFData_sagittal->fc_high 			= 3;		// experimently decided
	fuzzyTVCFData_sagittal->m0					= 8;		// Threshold Value (maybe middle value)
	fuzzyTVCFData_sagittal->sensitivity			= 0.6;		// Sensor Sensitivity (natural logarithm)
	fuzzyTVCFData_sagittal->scaling_mk			= 25;		// experimently decided
	fuzzyTVCFData_sagittal->scaling_mk_calib	= 11;		// experimently decided
	fuzzyTVCFData_sagittal->fc 					= fuzzyTVCFData_sagittal->fc_high;
	fuzzyTVCFData_sagittal->fc_calib			= fuzzyTVCFData_sagittal->fc_high;
	fuzzyTVCFData_sagittal->fc_LPF[1]			= fuzzyTVCFData_sagittal->fc_high;
	fuzzyTVCFData_sagittal->fc_calib_LPF[1]		= fuzzyTVCFData_sagittal->fc_high;
}

/* Reset Value Zero */
static void ResetDataObj(void)
{
	imuProcessedDataObj 			= 	(IMUProcessedData_t){0};
	fuzzyTVCFDataObj_Sagittal 		= 	(FuzzyTVCFData_t){0};
	filteredAngleDataObj_Sagittal	=   (FilteredAngleData_t){0};
}

/* ------------------- ROUTINE ------------------- */
static int Ent_GaitTotalFunc(void)
{
	return 0;
}

/* SUIT [MD] - Get Gait Data (PDO) */
static int Run_GaitTotalFunc(void)
{
	static uint32_t revoceryCnt_GaitTotal = 0;
	uint8_t t6AxisRes = IOIF_Get6AxisValue(&imuAccGyrRawDataObj);

	if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
		I2CfailedStateEnCnt++;
		/* Recovery 6axis IMU */
		if (revoceryCnt_GaitTotal > I2C_REVOVERY_TIME_INTERVAL) {
			// TODO : Error Handling
			__HAL_RCC_I2C2_CLK_DISABLE();
			__HAL_I2C_DISABLE(&hi2c2);
			HAL_I2C_DeInit(&hi2c2);
			HAL_I2C_MspDeInit(&hi2c2);

			// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
			HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

			HAL_I2C_Init(&hi2c2);
			HAL_I2C_MspInit(&hi2c2);
			__HAL_I2C_ENABLE(&hi2c2);
			__HAL_RCC_I2C2_CLK_ENABLE();

			IsRecoverI2C2 = true;

			t6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
			revoceryCnt_GaitTotal = 0;
		}
		revoceryCnt_GaitTotal++;
	}

	/* Update Current Value */
	UpdateIMUData(&imuProcessedDataObj, &imuAccGyrRawDataObj, IMUAttachCase);

	imuVQFDataObj.accel_local.x = imuProcessedDataObj.accZ_calibrated[0];
	imuVQFDataObj.accel_local.y = imuProcessedDataObj.accX_calibrated[0];
	imuVQFDataObj.accel_local.z = imuProcessedDataObj.accY_calibrated[0];
	imuVQFDataObj.gyro_local.x = imuProcessedDataObj.gyrZ_calibrated[0];
	imuVQFDataObj.gyro_local.y = imuProcessedDataObj.gyrX_calibrated[0];
	imuVQFDataObj.gyro_local.z = imuProcessedDataObj.gyrY_calibrated[0];

	GetEulerAnglesWithObject((AS_IMU_Euler_t*)&imuVQFDataObj.euler, \
			(AS_IMU_Accel_t*)&imuVQFDataObj.accel_local, (AS_IMU_Accel_t*)&imuVQFDataObj.accel_bodyframe, \
			(AS_IMU_Gyro_t*)&imuVQFDataObj.gyro_local, (AS_IMU_Gyro_t*)&imuVQFDataObj.gyro_global, \
			&_vqf_md);

	imuVQFDataObj.angle_bodyframe.sagittal		= imuVQFDataObj.euler.roll - imuVQFDataObj.sensor_to_body_offset.sagittal;
	imuVQFDataObj.angle_bodyframe.frontal		= imuVQFDataObj.euler.pitch - imuVQFDataObj.sensor_to_body_offset.frontal;
	imuVQFDataObj.angle_bodyframe.transverse	= imuVQFDataObj.euler.yaw - imuVQFDataObj.sensor_to_body_offset.transverse;

	// [Get Angle by using Encoder] //
	GetEncoderDegree(&filteredAngleDataObj_Sagittal, IMUAttachCase);
	if (IMUAttachCase == IMU_H10_RIGHT_SAG || IMUAttachCase == IMU_H10_LEFT_SAG) { // H10
		// [Combining the results of IMU(TVCF) & Encoder] //
		CombineIMUnEncoderDegree(&filteredAngleDataObj_Sagittal, &imuVQFDataObj);	// H10 Thigh Angle
	}

	SetFinalThighSagittalDegVel(&filteredAngleDataObj_Sagittal, &imuVQFDataObj, IMUAttachCase);

	/* Scaling For PDO Data */
	ScalingForPDO(&floatTo2ByteData, &imuProcessedDataObj, &fuzzyTVCFDataObj_Sagittal,
			&filteredAngleDataObj_Sagittal, &imuVQFDataObj);

	/* Save Previous Value */
	SavePrevBuff(&imuProcessedDataObj);

	return 0;
}

static int Ext_GaitTotalFunc(void)
{
	return 0;
}

/* Just get 6Axis & 3Axis IMU values */
static int RunGetIMUFunction(void)
{
	testImu6AxisRes = IOIF_Get6AxisValue(&imuAccGyrRawDataObj);
	// testImu3AxisRes = IOIF_GetMagValue(&imuMagRawDataObj);

	static uint16_t t_identifier = 0;
	static uint32_t t_temp_arr[7] = {0};

	memcpy(&t_temp_arr[0], &gaitCtrlLoopCnt, 4);
	memcpy(&t_temp_arr[1], &imuAccGyrRawDataObj.accX, 4);
	memcpy(&t_temp_arr[2], &imuAccGyrRawDataObj.accY, 4);
	memcpy(&t_temp_arr[3], &imuAccGyrRawDataObj.accZ, 4);
	memcpy(&t_temp_arr[4], &imuAccGyrRawDataObj.gyrX, 4);
	memcpy(&t_temp_arr[5], &imuAccGyrRawDataObj.gyrY, 4);
	memcpy(&t_temp_arr[6], &imuAccGyrRawDataObj.gyrZ, 4);

	t_identifier = GUI_SYNC|GET_ACC_GYRO;
	Send_MSG(t_identifier, (uint8_t*)t_temp_arr, 28);

	return 0;
}

/* ------------------- INITIALIZATION ------------------- */
/* Initialize 6Axis & 3Axis IMU */
static uint8_t InitializeIMU(void)
{
#ifdef L30_MD_REV06_ENABLED
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C1);
	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { //i2c revocery
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

		testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C1);
	}

	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
		return IOIF_IMU6AXIS_STATUS_ERROR;
	}

	return testImu6AxisRes;
#endif /* L30_MD_REV06_ENABLED */

#if defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C3);
	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C3_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c3);
		HAL_I2C_DeInit(&hi2c3);
		HAL_I2C_MspDeInit(&hi2c3);

		// GPIOC, 6X_IMU_I2C_SDA(GPIO_PIN_9), GPIOA, 6X_IMU_I2C_SCL(GPIO_PIN_8)
		HAL_I2C_BusReset(&hi2c3, GPIOC, IOIF_GPIO_PIN_9, GPIOA, IOIF_GPIO_PIN_8);

		HAL_I2C_Init(&hi2c3);
		HAL_I2C_MspInit(&hi2c3);
		__HAL_I2C_ENABLE(&hi2c3);
		__HAL_RCC_I2C3_CLK_ENABLE();

		IsRecoverI2C3 = true;

		testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C3);
	}

	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
		return IOIF_IMU6AXIS_STATUS_ERROR;
	}

	testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { //i2c revocery
		// TODO : Error Handling
		__HAL_RCC_I2C1_CLK_DISABLE();
		__HAL_I2C_DISABLE(&hi2c1);
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_MspDeInit(&hi2c1);

		// GPIOB, 3X_IMU_I2C_SDA(GPIO_PIN_7), 3X_IMU_I2C_SCL(GPIO_PIN_6)
		HAL_I2C_BusReset(&hi2c1, GPIOB, IOIF_GPIO_PIN_7, GPIOB, IOIF_GPIO_PIN_6);

		HAL_I2C_Init(&hi2c1);
		HAL_I2C_MspInit(&hi2c1);
		__HAL_I2C_ENABLE(&hi2c1);
		__HAL_RCC_I2C1_CLK_ENABLE();

		IsRecoverI2C1 = true;

		testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	}

	if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) {
		return IOIF_IMU3AXIS_STATUS_ERROR;
	}

	return (testImu6AxisRes + testImu3AxisRes);
#endif /* L30_MD_REV07_ENABLED & L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED
	testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
	if (testImu6AxisRes != IOIF_IMU6AXIS_STATUS_OK) { //i2c revocery
		I2CfailedInitCnt++;
		for (int trial = 0; trial < I2C_REVOVERY_TRAILS; ++trial) {
			IOIF_msDelay(I2C_REVOVERY_TIME_INTERVAL);
			// TODO : Error Handling
			__HAL_RCC_I2C2_CLK_DISABLE();
			__HAL_I2C_DISABLE(&hi2c2);
			HAL_I2C_DeInit(&hi2c2);
			HAL_I2C_MspDeInit(&hi2c2);

			// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
			HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

			HAL_I2C_Init(&hi2c2);
			HAL_I2C_MspInit(&hi2c2);
			__HAL_I2C_ENABLE(&hi2c2);
			__HAL_RCC_I2C2_CLK_ENABLE();

			IsRecoverI2C2 = true;

			testImu6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
			if (testImu6AxisRes == IOIF_IMU6AXIS_STATUS_OK) {
				break;
			}
			I2CfailedInitCnt++;
		}
	}

	// testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	// if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) { //i2c revocery
	// 	// TODO : Error Handling
	// 	__HAL_RCC_I2C1_CLK_DISABLE();
	// 	__HAL_I2C_DISABLE(&hi2c1);
	// 	HAL_I2C_DeInit(&hi2c1);
	// 	HAL_I2C_MspDeInit(&hi2c1);

	// 	// GPIOB, 3X_IMU_I2C_SDA(GPIO_PIN_7), 3X_IMU_I2C_SCL(GPIO_PIN_6)
	// 	HAL_I2C_BusReset(&hi2c1, GPIOB, IOIF_GPIO_PIN_7, GPIOB, IOIF_GPIO_PIN_6);

	// 	HAL_I2C_Init(&hi2c1);
	// 	HAL_I2C_MspInit(&hi2c1);
	// 	__HAL_I2C_ENABLE(&hi2c1);
	// 	__HAL_RCC_I2C1_CLK_ENABLE();

	// 	IsRecoverI2C1 = true;

	// 	testImu3AxisRes = IOIF_InitMag(IOIF_I2C1);
	// }

	// if (testImu3AxisRes != IOIF_IMU3AXIS_STATUS_OK) {
	// 	return IOIF_IMU3AXIS_STATUS_ERROR;
	// }

	// return (testImu6AxisRes + testImu3AxisRes);

	return testImu6AxisRes;
#endif /* SUIT_MD_ENABLED */
}

/* ------------------- Sensor Fusion Algorithm ------------------- */
/*
 *The function UpdateIMUData updates the IMU raw values and apply calibration & orientation.
 */
// Global Coordinate System 기준
// acc x : 사람의 전방 (+방향)
// acc y : 사람의 머리 위 지면에 수직 (+방향)
// acc z : 사람의 오른쪽으로 뻗어나가는 방향 (+방향)
// gyro x (roll) : 사람의 전방축 기준 회전이 + 
// gyro y (yaw) : 사람의 머리 위 지면에 수직축 기준 회전이 + 
// gyro z (pitch) : 사람에서 오른쪽으로 뻗어나가는 방향 기준 + 방향
static void UpdateIMUData(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData, IMUAttachCase_t attachCase)
{
	if (attachCase == IMU_H10_LEFT_SAG || attachCase == IMU_K10_LEFT_SAG) { // Left Case
		/* Calibrated */
		sensorData->accX_calibrated[0] = imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);
		sensorData->accY_calibrated[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);
		sensorData->accZ_calibrated[0] = (-1.0f) * imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);

		sensorData->gyrX_calibrated[0] = imuAccGyrRawData->gyrX;
		sensorData->gyrY_calibrated[0] = (-1.0f) * imuAccGyrRawData->gyrY;
		sensorData->gyrZ_calibrated[0] = (-1.0f) * fabs(sensorData->scaling_gyro) * (imuAccGyrRawData->gyrZ - sensorData->bias_gyro);
	
		/* Not Calibrated */
		sensorData->accX[0] = imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT;
		sensorData->accY[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT;
		sensorData->accZ[0] = (-1.0f) * imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT;

		sensorData->gyrX[0] = imuAccGyrRawData->gyrX;
		sensorData->gyrY[0] = (-1.0f) * imuAccGyrRawData->gyrY;
		sensorData->gyrZ[0] = (-1.0f) * imuAccGyrRawData->gyrZ;

	} else if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_K10_RIGHT_SAG) { // Right Case
		/* Calibrated */
		sensorData->accX_calibrated[0] = (-1.0f) * imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);;
		sensorData->accY_calibrated[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);;
		sensorData->accZ_calibrated[0] = imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT * ACC_GRAVITY_CONSTANT / (sensorData->accMeanVal_calibrated);;

		sensorData->gyrX_calibrated[0] = (-1.0f) * imuAccGyrRawData->gyrX;
		sensorData->gyrY_calibrated[0] = (-1.0f) * imuAccGyrRawData->gyrY;
		sensorData->gyrZ_calibrated[0] = fabs(sensorData->scaling_gyro) * (imuAccGyrRawData->gyrZ - sensorData->bias_gyro);

		/* Not Calibrated */
		sensorData->accX[0] = (-1.0f) * imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT;
		sensorData->accY[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT;
		sensorData->accZ[0] = imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT;

		sensorData->gyrX[0] = (-1.0f) * imuAccGyrRawData->gyrX;
		sensorData->gyrY[0] = (-1.0f) * imuAccGyrRawData->gyrY;
		sensorData->gyrZ[0] = imuAccGyrRawData->gyrZ;
	}
}

// threshold checking for all sensor values
static void CheckIMUNoise(float* currentVal, float prevVal, float* lastValidVal, uint8_t* algorithmActivated, uint16_t threshold, uint32_t* errCode, uint32_t* noiseCounter)
{
	// 현재 값과 이전 값을 비교하여 노이즈 감지
	if (abs(*currentVal - prevVal) >= threshold) {
		if (!(*algorithmActivated)) {
			// 첫 번째 노이즈 감지
			*lastValidVal = prevVal; 	// 마지막 유효한 값을 저장
			*currentVal = prevVal; 		// 현재 값을 이전 값으로 덮어씀
			*algorithmActivated = 1; 	// 알고리즘 활성화
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

static void SavePrevBuff(IMUProcessedData_t* sensorData)
{
	sensorData->accX[1] = sensorData->accX[0];
	sensorData->accY[1] = sensorData->accY[0];
	sensorData->accZ[1] = sensorData->accZ[0];
	sensorData->gyrX[1] = sensorData->gyrX[0];
	sensorData->gyrY[1] = sensorData->gyrY[0];
	sensorData->gyrZ[1] = sensorData->gyrZ[0];

	sensorData->accX_calibrated[1] = sensorData->accX_calibrated[0];
	sensorData->accY_calibrated[1] = sensorData->accY_calibrated[0];
	sensorData->accZ_calibrated[1] = sensorData->accZ_calibrated[0];
	sensorData->gyrX_calibrated[1] = sensorData->gyrX_calibrated[0];
	sensorData->gyrY_calibrated[1] = sensorData->gyrY_calibrated[0];
	sensorData->gyrZ_calibrated[1] = sensorData->gyrZ_calibrated[0];

//	sensorData->accX_global[1] = sensorData->accX_global[0];
//	sensorData->accY_global[1] = sensorData->accY_global[0];
//	sensorData->accZ_global[1] = sensorData->accZ_global[0];
//	sensorData->gyrX_global[1] = sensorData->gyrX_global[0];
//	sensorData->gyrY_global[1] = sensorData->gyrY_global[0];
//	sensorData->gyrZ_global[1] = sensorData->gyrZ_global[0];
}

// 안전한 atan2f 함수
static float SafeAtan2f(float a, float b)
{
	float result = 0.0f;
    if (a == 0.0f && b == 0.0f) {
		result = M_PI;
        return result; // 정의되지 않은 경우, 특정 값 반환
    } else if (a == 0.0f && b < 0.0f) {
		result = M_PI;
        return result; // 정의되지 않은 경우, 특정 값 반환
	} else if (a < 0.0f && b < 0.0f) {
		result = atan2f(a, b) + 2 * M_PI; // 360
        return result; // 정의되지 않은 경우, 특정 값 반환
	}
    return atan2f(a, b); // 기본 계산 수행
}

/*
 *The function GetEncoderDegree considers the Incremental Encoder's raw values.
 */
static void GetEncoderDegree(FilteredAngleData_t* filteredAngleData, IMUAttachCase_t attachCase)
{
	/* We need Absolute Encoder at Actuator Out */
	if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_H10_LEFT_SAG)  {	// H10
		filteredAngleData->degINC = mid_level_state.position / M_PI * 180.0;
	} else if (attachCase == IMU_K10_RIGHT_SAG || attachCase == IMU_K10_LEFT_SAG) {	// K10
		filteredAngleData->degINC = mid_level_state.position / M_PI * 180.0 - filteredAngleDataObj_Sagittal.NeutralPostureBias_KneeEncoder;
	}
	filteredAngleData->velDegINC = mid_level_state.velocity_raw / M_PI * 180.0;
}

/*
 *Function to compensate IMU + INC case for Only SUIT H10
 */
static void CombineIMUnEncoderDegree(FilteredAngleData_t* filteredAngleData, VQF_IMUData_t* imuVQFData)
{
	filteredAngleData->degIMUINC = filteredAngleData->degINC + imuVQFData->angle_bodyframe.sagittal;
	filteredAngleData->degIMUINC_calib = filteredAngleData->degINC + imuVQFData->angle_bodyframe.sagittal;
}

/*
 *Function to select finally used deg&vel value before filtering
 */
static void SetFinalThighSagittalDegVel(FilteredAngleData_t* filteredAngleData, VQF_IMUData_t* imuVQFData, IMUAttachCase_t attachCase)
{
    static uint8_t isInitialized = 0;
    // 첫 실행시 초기화
    if (!isInitialized) {
        if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_H10_LEFT_SAG) {
            filteredAngleData->thighDegFinal_calib[1] = filteredAngleData->degIMUINC_calib - filteredAngleData->NeutralPostureBias;
        } else if (attachCase == IMU_K10_RIGHT_SAG || attachCase == IMU_K10_LEFT_SAG) {
            filteredAngleData->thighDegFinal_calib[1] = imuVQFData->angle_bodyframe.sagittal - filteredAngleData->NeutralPostureBias;
        }
        isInitialized = 1;
    }

    // 최종 허벅지 각도 & 각속도 계산
    if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_H10_LEFT_SAG) {
        filteredAngleData->thighDegFinal_calib[0] = filteredAngleData->degIMUINC_calib - filteredAngleData->NeutralPostureBias;
        filteredAngleData->thighVelDegFinal_calib[0] = (filteredAngleData->thighDegFinal_calib[0] - filteredAngleData->thighDegFinal_calib[1]) / GAIT_CONTROL_PERIOD;

    } else if (attachCase == IMU_K10_RIGHT_SAG || attachCase == IMU_K10_LEFT_SAG) {
        filteredAngleData->thighDegFinal_calib[0] = imuVQFData->angle_bodyframe.sagittal - filteredAngleData->NeutralPostureBias;
        filteredAngleData->thighVelDegFinal_calib[0] = (filteredAngleData->thighDegFinal_calib[0] - filteredAngleData->thighDegFinal_calib[1]) / GAIT_CONTROL_PERIOD;
    }

    // 현재 값을 이전 값으로 저장
    filteredAngleData->thighDegFinal[1] = filteredAngleData->thighDegFinal[0];
    filteredAngleData->thighDegFinal_calib[1] = filteredAngleData->thighDegFinal_calib[0];
}

/* ------------------- AUTO SENSOR CALIBRATION ------------------- */
static void AutoCalibrationIMU(void)
{
	static uint32_t revoceryCnt_AutoCal = 0;
	// TODO : Auto Calibration Must be in Slow Motion!
	if (autoCalibSensorCmd == 1) {
		if (oneTimeInitAutoCal == true) {
			initAutoCalParam(&imuProcessedDataObj, &imuAccGyrRawDataObj);
			oneTimeInitAutoCal = false;
		} else {
			uint8_t t6AxisRes = IOIF_Get6AxisValue(&imuAccGyrRawDataObj);
			if (t6AxisRes != IOIF_IMU6AXIS_STATUS_OK) {
				I2CfailedAutoCnt++;
				/* Recovery 6axis IMU */
				if (revoceryCnt_AutoCal > I2C_REVOVERY_TIME_INTERVAL) {
					// TODO : Error Handling
					__HAL_RCC_I2C2_CLK_DISABLE();
					__HAL_I2C_DISABLE(&hi2c2);
					HAL_I2C_DeInit(&hi2c2);
					HAL_I2C_MspDeInit(&hi2c2);

					// GPIOF, 6X_IMU_SDA(GPIO_PIN_0), 6X_IMU_SCL(GPIO_PIN_1)
					HAL_I2C_BusReset(&hi2c2, GPIOF, IOIF_GPIO_PIN_0, GPIOF, IOIF_GPIO_PIN_1);

					HAL_I2C_Init(&hi2c2);
					HAL_I2C_MspInit(&hi2c2);
					__HAL_I2C_ENABLE(&hi2c2);
					__HAL_RCC_I2C2_CLK_ENABLE();

					IsRecoverI2C2 = true;

					t6AxisRes = IOIF_Init6Axis(IOIF_I2C2);
					gaitCtrlTask.errCode = ERROR_IMU_SENSOR_GET_VALUE;
					revoceryCnt_AutoCal = 0;
				}
				revoceryCnt_AutoCal++;
			} else {
				// TODO : Error Handling 필요
			}
			UpdateSensorForAutoCal(&imuProcessedDataObj, &imuAccGyrRawDataObj, IMUAttachCase);

			AccumultateSqrtSum(&imuProcessedDataObj);
			AccumulateGyroMatrix(&imuProcessedDataObj, IMUAttachCase, autoCalibCnt, GAIT_CONTROL_PERIOD);

			autoCalibCnt++;
		}
	} else if (autoCalibSensorCmd == 2) { // Auto-Calibration Sensor Parameters
		if (autoCalibCnt > 0) {
			GetAccMeanVal(&imuProcessedDataObj, autoCalibCnt);
			GetGyroParam(IMUAttachCase, &imuProcessedDataObj);
			autoCalibSensorCmd = 0;
			autoCalibCnt = 0;
			autoCalibratedFlag = 1;
			oneTimeInitAutoCal = true;

			MS_enum = SAVE_PROPERTIES;
		} else {
			// TODO : Error Handling 필요
			oneTimeInitAutoCal = true;
			autoCalibSensorCmd = 1;
		}
	} else {
		oneTimeInitAutoCal = true;
	}
}

static void initAutoCalParam(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData)
{
	memset(imuAccGyrRawData, 0, sizeof(IOIF_6AxisData_t));

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

// 상대 좌표계 기준
// acc x : 사람의 전방 (+방향)
// acc y : 사람의 머리 위 지면에 수직 (+방향)
// acc z : 사람에서 뻗어나가는 방향 (+방향)
// gyro x (roll) : 사람의 전방축 기준 왼쪽 회전이 + 
// gyro y (yaw) : 사람의 머리 위 지면에 수직축 기준 왼쪽 회전이 + 
// gyro z (pitch) : 사람에서 뻗어나가는 방향 기준 지면에 수직인 상태에서 flexion이 + 방향
static void UpdateSensorForAutoCal(IMUProcessedData_t* sensorData, IOIF_6AxisData_t* imuAccGyrRawData, IMUAttachCase_t attachCase)
{

	sensorData->accX[0] = imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT;
	sensorData->accY[0] = imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT;
	sensorData->accZ[0] = imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT;
	sensorData->gyrX[0] = imuAccGyrRawData->gyrX;
	sensorData->gyrY[0] = imuAccGyrRawData->gyrY;
	sensorData->gyrZ[0] = imuAccGyrRawData->gyrZ;

//	if (attachCase == IMU_H10_LEFT_SAG || attachCase == IMU_K10_LEFT_SAG) {	// Left Case
//		/* Not Calibrated */
//		sensorData->accX[0] = imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT;
//		sensorData->accY[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT;
//		sensorData->accZ[0] = (-1.0f) * imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT;
//
//		sensorData->gyrX[0] = imuAccGyrRawData->gyrX;
//		sensorData->gyrY[0] = (-1.0f) * imuAccGyrRawData->gyrY;
//
//		// Scaling(sign), bias
//		sensorData->gyrZ[0] = imuAccGyrRawData->gyrZ;	// For Negative Gyro case (Maybe LEFT case)
//	} else if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_K10_RIGHT_SAG) { // Right Case
//		/* Not Calibrated */
//		sensorData->accX[0] = (-1.0f) * imuAccGyrRawData->accX * ACC_GRAVITY_CONSTANT;
//		sensorData->accY[0] = (-1.0f) * imuAccGyrRawData->accY * ACC_GRAVITY_CONSTANT;
//		sensorData->accZ[0] = imuAccGyrRawData->accZ * ACC_GRAVITY_CONSTANT;
//
//		sensorData->gyrX[0] = (-1.0f) * imuAccGyrRawData->gyrX;
//		sensorData->gyrY[0] = (-1.0f) * imuAccGyrRawData->gyrY;
//
//		// Scaling(sign), bias
//		sensorData->gyrZ[0] = imuAccGyrRawData->gyrZ;	// For Positive Gyro case (Maybe RIGHT case)
//	}

	DetectIMUNoiseForAutoCal(sensorData);
}

static void DetectIMUNoiseForAutoCal(IMUProcessedData_t* sensorData)
{
	// For Raw Sensor Data Error Check
	static uint8_t isInitialized_DSN = 1;
	if (isInitialized_DSN == 1) {
		sensorData->accX[1] = sensorData->accX[0];
		sensorData->accY[1] = sensorData->accY[0];
		sensorData->accZ[1] = sensorData->accZ[0];
		sensorData->gyrX[1] = sensorData->gyrX[0];
		sensorData->gyrY[1] = sensorData->gyrY[0];
		sensorData->gyrZ[1] = sensorData->gyrZ[0];
		isInitialized_DSN = 0;
	}
	CheckIMUNoise(&sensorData->accX[0], sensorData->accX[1], &sensorData->accXLastValidData, 
			&sensorData->accXAlgorithmActivated, ACC_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->accXNoiseCnt);
	CheckIMUNoise(&sensorData->accY[0], sensorData->accY[1], &sensorData->accYLastValidData, 
			&sensorData->accYAlgorithmActivated, ACC_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->accYNoiseCnt);
	CheckIMUNoise(&sensorData->accZ[0], sensorData->accZ[1], &sensorData->accZLastValidData, 
			&sensorData->accZAlgorithmActivated, ACC_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->accZNoiseCnt);
	CheckIMUNoise(&sensorData->gyrX[0], sensorData->gyrX[1], &sensorData->gyrXLastValidData, 
			&sensorData->gyrXAlgorithmActivated, GYR_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->gyrXNoiseCnt);
	CheckIMUNoise(&sensorData->gyrY[0], sensorData->gyrY[1], &sensorData->gyrYLastValidData, 
			&sensorData->gyrYAlgorithmActivated, GYR_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->gyrYNoiseCnt);
	CheckIMUNoise(&sensorData->gyrZ[0], sensorData->gyrZ[1], &sensorData->gyrZLastValidData, 
			&sensorData->gyrZAlgorithmActivated, GYR_THRESHOLD_CONSTANT, &gaitCtrlTask.errCode, &sensorData->gyrZNoiseCnt);
}

// Function to calculate mean value for accelerometer with LPF applied
static void AccumultateSqrtSum(IMUProcessedData_t* sensorData)
{
	static bool isInitialized_m0Raw = 1;
	if (isInitialized_m0Raw == 1) {
		sensorData->m0Raw = sqrt(pow(sensorData->accX[0], 2) + pow(sensorData->accY[0], 2) + pow(sensorData->accZ[0], 2));
		isInitialized_m0Raw = 0;
	}

	sensorData->sqrtAcc = sqrt(pow(sensorData->accX[0], 2) + pow(sensorData->accY[0], 2) + pow(sensorData->accZ[0], 2));
	sensorData->m0Raw = 0.95 * sensorData->m0Raw + 0.05 * sensorData->sqrtAcc;
	sensorData->m0RawSum += sensorData->m0Raw;
}

/**
 * @brief Update degree values and accumulate matrices for gyro calibration
 * 
 * @param sensorData Pointer to sensor data structure
 * @param attachCase Attachment case for specific settings
 * @param sampleCnt Current sample count
 * @param samplingPeriod Sampling period (time interval between samples)
 */
static void AccumulateGyroMatrix(IMUProcessedData_t* sensorData, IMUAttachCase_t attachCase, uint32_t sampleCnt, float samplingPeriod)
{
	// 가속도 축 보정은 후 보정(최종 허벅지 각도 - 초기 IMU 기계적 각도 25도)
	sensorData->degAccArctan = SafeAtan2f(sensorData->accY[0], sensorData->accX[0]) * 180.0f / M_PI;
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

static void GetAccMeanVal(IMUProcessedData_t* sensorData, uint32_t sampleSize)
{
	sensorData->accMeanVal_calibrated = sensorData->m0RawSum / (float)(sampleSize);
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
static void GetGyroParam(IMUAttachCase_t attachCase,IMUProcessedData_t* sensorData)
{
	// Calculate the determinant of PhiMatrixSum
	sensorData->matrixDeterminant = sensorData->PhiMatrixSum[0][0] * 
			(sensorData->PhiMatrixSum[1][1] * sensorData->PhiMatrixSum[2][2] - sensorData->PhiMatrixSum[1][2] * sensorData->PhiMatrixSum[2][1])
			- sensorData->PhiMatrixSum[0][1] *
			(sensorData->PhiMatrixSum[1][0] * sensorData->PhiMatrixSum[2][2] - sensorData->PhiMatrixSum[1][2] * sensorData->PhiMatrixSum[2][0])
			+ sensorData->PhiMatrixSum[0][2] *
			(sensorData->PhiMatrixSum[1][0] * sensorData->PhiMatrixSum[2][1] - sensorData->PhiMatrixSum[1][1] * sensorData->PhiMatrixSum[2][0]);

	// Calculate the inverse of PhiMatrixSum
	CalInverse3x3(sensorData->PhiMatrixSum, sensorData->invPhiMatrix, sensorData->matrixDeterminant);

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

// 3x3 행렬의 역행렬 계산 함수
static void CalInverse3x3(float matrix[3][3], float invMatrix[3][3], float det)
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

/* ------------------- Neutral Posture Calculation ------------------- */
static void CheckNeutralized(void)
{
	if (neutralizedFlag == NEUTRAL_POSTURE_NOT_CALIBRATED || neutralizedFlag == NEUTRAL_POSTURE_CALIBRATED) {
		// Do Nothing
	} else {
		neutralizedFlag = NEUTRAL_POSTURE_NOT_CALIBRATED;
	}
}

static void CalNeutralPosture(void)
{
	if (CalNeutralPostureCmd == 1) {
		if (calNeutralPostureCnt < 1000) {
			tAccumulateThighAngle += filteredAngleDataObj_Sagittal.thighDegFinal_calib[0];
			tAccumuulateKneeAngle += mid_level_state.position / M_PI * 180.0;
			calNeutralPostureCnt++;
		} else if (calNeutralPostureCnt == 1000) {
			filteredAngleDataObj_Sagittal.NeutralPostureBias = tAccumulateThighAngle / (float)(calNeutralPostureCnt);
			filteredAngleDataObj_Sagittal.NeutralPostureBias_KneeEncoder = tAccumuulateKneeAngle / (float)(calNeutralPostureCnt);
			calNeutralPostureCnt = 0;
			CalNeutralPostureCmd = 0;
			tAccumulateThighAngle = 0.0f;
			tAccumuulateKneeAngle = 0.0f;
			neutralizedFlag = NEUTRAL_POSTURE_CALIBRATED;
			MS_enum = SAVE_PROPERTIES_NeutralPosture;
			Send_NOTI((uint32_t)1);
			Send_NeutralPostureDeg();
		}
	}
}

/* ------------------- Command Driven Functions ------------------- */
static void ExecuteCmdDrivenFunctions(void)
{
	CalNeutralPosture();
	AutoCalibrationIMU();
}

/* ------------------- Scaling Data For PDO ------------------- */
static void ScalingForPDO(ScaledData* Scaled2ByteData, IMUProcessedData_t* sensorData, FuzzyTVCFData_t* fuzzyTVCFData_sagittal,
		FilteredAngleData_t* angleData_sagittal, VQF_IMUData_t* imuVQFData)
{
	// Scaling for PDO Data
	Scaled2ByteData->AccX = ScaleFloatToInt16(sensorData->accX[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->AccY = ScaleFloatToInt16(sensorData->accY[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->AccZ = ScaleFloatToInt16(sensorData->accZ[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->GyrX = ScaleFloatToInt16(sensorData->gyrX[0], GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrY = ScaleFloatToInt16(sensorData->gyrY[0], GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrZ = ScaleFloatToInt16(sensorData->gyrZ[0], GYR_SCALING_FACTOR);

	Scaled2ByteData->AccX_calib = ScaleFloatToInt16(sensorData->accX_calibrated[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->AccY_calib = ScaleFloatToInt16(sensorData->accY_calibrated[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->AccZ_calib = ScaleFloatToInt16(sensorData->accZ_calibrated[0], ACC_SCALING_FACTOR);
	Scaled2ByteData->GyrX_calib = ScaleFloatToInt16(sensorData->gyrX_calibrated[0], GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrY_calib = ScaleFloatToInt16(sensorData->gyrY_calibrated[0], GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrZ_calib = ScaleFloatToInt16(sensorData->gyrZ_calibrated[0], GYR_SCALING_FACTOR);

	Scaled2ByteData->AccX_global = ScaleFloatToInt16(imuVQFData->accel_bodyframe.x, ACC_SCALING_FACTOR);
	Scaled2ByteData->AccY_global = ScaleFloatToInt16(imuVQFData->accel_bodyframe.y, ACC_SCALING_FACTOR);
	Scaled2ByteData->AccZ_global = ScaleFloatToInt16(imuVQFData->accel_bodyframe.z, ACC_SCALING_FACTOR);
	Scaled2ByteData->GyrX_global = ScaleFloatToInt16(imuVQFData->gyro_global.x, GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrY_global = ScaleFloatToInt16(imuVQFData->gyro_global.y, GYR_SCALING_FACTOR);
	Scaled2ByteData->GyrZ_global = ScaleFloatToInt16(imuVQFData->gyro_global.z, GYR_SCALING_FACTOR);

	Scaled2ByteData->ThighDeg_sagittal 			= ScaleFloatToInt16(angleData_sagittal->thighDegFinal_calib[0], DEG_SCALING_FACTOR);
	Scaled2ByteData->ThighVelDeg_sagittal 		= ScaleFloatToInt16(angleData_sagittal->thighVelDegFinal_calib[0], VELDEG_SCALING_FACTOR);
	Scaled2ByteData->IncDeg_sagittal 			= ScaleFloatToInt16(angleData_sagittal->degINC, DEG_SCALING_FACTOR);
	Scaled2ByteData->IncVelDeg_sagittal 		= ScaleFloatToInt16(angleData_sagittal->velDegINC, VELDEG_SCALING_FACTOR);

	Scaled2ByteData->IMU_BodyAngle_Sagittal		= ScaleFloatToInt16(imuVQFData->angle_bodyframe.sagittal, DEG_SCALING_FACTOR);
	Scaled2ByteData->IMU_BodyAngle_Frontal		= ScaleFloatToInt16(imuVQFData->angle_bodyframe.frontal, DEG_SCALING_FACTOR);
	Scaled2ByteData->IMU_BodyAngle_Transverse	= ScaleFloatToInt16(imuVQFData->angle_bodyframe.transverse, DEG_SCALING_FACTOR);

	Scaled2ByteData->fc_sagittal 				= ScaleFloatToUInt16(fuzzyTVCFData_sagittal->fc, WC_SCALING_FACTOR); // uint16
	Scaled2ByteData->fc_calib_sagittal 			= ScaleFloatToUInt16(fuzzyTVCFData_sagittal->fc_calib, WC_SCALING_FACTOR); // uint16
}

/* ------------------- SDO CALLBACK ------------------- */
// Auto Calibration Command
static void SetAutoCalCmd(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&autoCalibSensorCmd, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

// Neutral Posture Calculation
static void SetIsNeutralized(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	Send_NOTI(neutralizedFlag);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

// Neutral Posture Calculation
static void SetNeutralPostureCalCmd(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&CalNeutralPostureCmd, t_req->data, 1);

	calNeutralPostureCnt = 0;
	tAccumulateThighAngle = 0.0f;
	tAccumuulateKneeAngle = 0.0f;
	filteredAngleDataObj_Sagittal.NeutralPostureBias = 0.0f;
	filteredAngleDataObj_Sagittal.NeutralPostureBias_KneeEncoder = 0.0f;

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

// Neutral Posture Offset
static void SetNeutralPostureOffset(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&neutralOffset_int16, t_req->data, sizeof(neutralOffset_int16));

	filteredAngleDataObj_Sagittal.NeutralPostureBiasOffset = ScaleInt16ToFloat(neutralOffset_int16, DEG_SCALING_FACTOR);
	filteredAngleDataObj_Sagittal.NeutralPostureBias += filteredAngleDataObj_Sagittal.NeutralPostureBiasOffset;
	MS_enum = SAVE_PROPERTIES_NeutralPosture;

	Send_NOTI((uint32_t)1);
	Send_NeutralPostureDeg();

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

// User Info's Neutral Posture from Tablet
static void SetUserNeutralPostureAngle(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&UserNeutralPosture_int16, t_req->data, sizeof(UserNeutralPosture_int16));

	filteredAngleDataObj_Sagittal.NeutralPostureBias = ScaleInt16ToFloat(UserNeutralPosture_int16, DEG_SCALING_FACTOR);
	Send_NeutralPostureDeg();

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* TVCF 알고리즘 */
/*
 * Calculate Fc(CutOff Frequency : Hz)
 */
//static void CalFcForTVCF_Sagittal(IMUProcessedData_t* sensorData, FuzzyTVCFData_t* fuzzyData)
//{
//	// For Gyration
//	sensorData->gyrationLPF[0] = ApplyLPF(&sensorData->isInitialized_gyrationLPF, &sensorData->gyrationLPF[1], sensorData->gyrY_global[0], GYRATION_LPF_ALPHA);
//	fuzzyData->gyroSuppress = GetMaxValue(18.0f, GetAbsoluteValue(sensorData->gyrationLPF[0])) - 18.0f;
//
//	// Square Root Sum
//	fuzzyData->SqrtSumData = SquareRootSum(sensorData->accX[0], sensorData->accY[0]);
//	fuzzyData->SqrtSumData_calib = SquareRootSum(sensorData->accX_calibrated[0], sensorData->accY_calibrated[0]);
//
//	// absolute sqrt ACC
//	fuzzyData->fuzzyInput[0] = fuzzyData->scaling_mk * GetAbsoluteValue(fuzzyData->SqrtSumData - FUZZY_LOGIC_GRAVITY_CONSTANT);
//	fuzzyData->fuzzyInput_calib[0] = fuzzyData->scaling_mk_calib * GetAbsoluteValue(fuzzyData->SqrtSumData_calib - sensorData->accMeanVal_calibrated);
//
//	// absolute Gyr(Angular Velocity)
//	fuzzyData->fuzzyInput[2] = GetAbsoluteValue(sensorData->gyrZ[0]);
//	fuzzyData->fuzzyInput_calib[2] = GetAbsoluteValue(sensorData->gyrZ_calibrated[0]);
//
//	/* fuzzy Input[0] + fuzzy Input[2] with LPF */
//	fuzzyData->mk = fuzzyData->fuzzyInput[0] + fuzzyData->fuzzyInput[2];
//	fuzzyData->mk_LPF[0] = ApplyLPF(&fuzzyData->isInitializedMk, &fuzzyData->mk_LPF[1], fuzzyData->mk, FUZZY_INPUT_LPF_ALPHA);
//
//	fuzzyData->mk_calib = fuzzyData->fuzzyInput_calib[0] + fuzzyData->fuzzyInput_calib[2];
//	fuzzyData->mk_calib_LPF[0] = ApplyLPF(&fuzzyData->isInitializedMkCalib, &fuzzyData->mk_calib_LPF[1], fuzzyData->mk_calib, FUZZY_INPUT_LPF_ALPHA);
//
//	/* Perform Fc calculations for each fuzzy input (Acc, Angular Velocity) */
//	fuzzyData->mu = tanh(fuzzyData->sensitivity * (fuzzyData->mk_LPF[0] - fuzzyData->m0));				// Update mu for TVCF cutoff frequency(wc)
//	fuzzyData->mu_calib = tanh(fuzzyData->sensitivity * (fuzzyData->mk_calib_LPF[0] - fuzzyData->m0));	// Update mu for TVCF cutoff frequency(wc)
//	fuzzyData->fc = (fuzzyData->fc_high - fuzzyData->fc_low) * 0.5 * (1 - fuzzyData->mu) + fuzzyData->fc_low;
//	fuzzyData->fc_calib = (fuzzyData->fc_high - fuzzyData->fc_low) * 0.5 * (1 - fuzzyData->mu_calib) + fuzzyData->fc_low;
//
//	// Apply low-pass filter (LPF) to wc
//	// fuzzyData->fc_LPF[0] = ApplyLPF(&fuzzyData->isInitialized_fc, &fuzzyData->fc_LPF[1], fuzzyData->fc, FC_LPF_ALPHA);
//	// fuzzyData->fc_calib_LPF[0] = ApplyLPF(&fuzzyData->isInitialized_calib_fc, &fuzzyData->fc_calib_LPF[1], fuzzyData->fc_calib, FC_LPF_ALPHA);
//
//	// Save Previous Values in SavePrevBuff Function
//	// fuzzyData->fc_LPF[1] = fuzzyData->fc_LPF[0];
//	// fuzzyData->fc_calib_LPF[1] = fuzzyData->fc_calib_LPF[0];
//	fuzzyData->mk_LPF[1] = fuzzyData->mk_LPF[0];
//	fuzzyData->mk_calib_LPF[1] = fuzzyData->mk_calib_LPF[0];
//	sensorData->gyrationLPF[1] = sensorData->gyrationLPF[0];
//}

///*
// *Function to execute the time-varying complementary filter (with Fuzzy Logic - fc)
// */
//static void GetTVCFSagittalDegree(IMUProcessedData_t* sensorData, FilteredAngleData_t* angleData, FuzzyTVCFData_t* widmFuzzyData, float samplingPeriod, IMUAttachCase_t attachCase)
//{
//	/* Apply time-varying complementary filter on the sensor data using fuzzy logic(fc) and update the thigh angle parameters */
//	CalTVCFSagittalDegree(sensorData, angleData, widmFuzzyData, samplingPeriod);
//
//	/* Update the unfiltered thigh angle to be the same as the filtered thigh angle */
//	if (attachCase == IMU_H10_RIGHT_SAG || attachCase == IMU_H10_LEFT_SAG) {	// H10
//		angleData->degTvcf[0] = angleData->angleTVCF_Deg - angleData->initOffsetIMUDeg;
//		angleData->degTvcf_calib[0] = angleData->angleTVCF_calib_Deg - angleData->initOffsetIMUDeg;
//	}
//
//	if (attachCase == IMU_K10_RIGHT_SAG || attachCase == IMU_K10_LEFT_SAG) {	// K10
//		angleData->degTvcf[0] = -(angleData->angleTVCF_Deg - angleData->initOffsetIMUDeg);
//		angleData->degTvcf_calib[0] = -(angleData->angleTVCF_calib_Deg - angleData->initOffsetIMUDeg);
//	}
//
//	// Calculate the raw velocity
//	angleData->degTvcf[1] = angleData->degTvcf[0];
//	angleData->degTvcf_calib[1] = angleData->degTvcf_calib[0];
//}

///*
// * Function to apply a Time Variant Complementary Filter (TVCF) to an angle
// */
//static void CalTVCFSagittalDegree(IMUProcessedData_t* sensorData, FilteredAngleData_t* angleData, FuzzyTVCFData_t* fuzzyData, float samplingPeriod)
//{
//	fuzzyData->fcHPF = fuzzyData->fc_calib + 0.2 * fuzzyData->gyroSuppress;
//	// fuzzyData->wcHPF_LPF = fuzzyData->fc_calib_LPF[0] + 0.2 * fuzzyData->gyroSuppress;
//
//	/* Calculate the angle using accelerometer measurements and convert it to degrees */
//	/* Find accelerometer-based angle according to the "WIDM Attach Case" */
//	// 가속도 축 보정은 후 보정(최종 허벅지 각도 - 초기 IMU 기계적 각도 65도)
//	angleData->degAcc_arctan[0] = SafeAtan2f(sensorData->accY_calibrated[0], sensorData->accX_calibrated[0]) * 180.0 / M_PI;
//	angleData->degGyr_Integral[0] = angleData->degGyr_Integral[1] + sensorData->gyrZ[0] * samplingPeriod;
//	angleData->degGyr_Integral_calibrated[0] = angleData->degGyr_Integral_calibrated[1] + sensorData->gyrZ_calibrated[0] * samplingPeriod;
//
//	/* Apply 1st order Low Pass Filter (LPF) on accelerometer angle */
//	angleData->degAcc_1stLPF[0] = Apply1stLPF_TVCF(angleData->degAcc_arctan[0], angleData->degAcc_arctan[1], angleData->degAcc_1stLPF[1], fuzzyData->fc, samplingPeriod);
//	angleData->degAcc_calib_1stLPF[0] = Apply1stLPF_TVCF(angleData->degAcc_arctan[0], angleData->degAcc_arctan[1], angleData->degAcc_calib_1stLPF[1], fuzzyData->fc_calib, samplingPeriod);
//	// angleData->degAcc_calib_1stLPF_LPF[0] = Apply1stLPF_TVCF(angleData->degAcc_arctan[0], angleData->degAcc_arctan[1], angleData->degAcc_calib_1stLPF_LPF[1], fuzzyData->fc_calib_LPF[0], samplingPeriod);
//
//	/* Apply 1st order High Pass Filter (HPF) with integration on gyroscope measurements */
//	angleData->degGyr_Integral_1stHPF[0] = Apply1stHPFwithIntegral_TVCF(sensorData->gyrZ[0], angleData->degGyr_Integral_1stHPF[1], fuzzyData->fc, samplingPeriod);
//	angleData->degGyr_calb_Integral_1stHPF[0] = Apply1stHPFwithIntegral_TVCF(sensorData->gyrZ_calibrated[0], angleData->degGyr_calb_Integral_1stHPF[1], fuzzyData->fcHPF, samplingPeriod);
//	// angleData->degGyr_calb_Integral_1stHPF_LPF[0] = Apply1stHPFwithIntegral_TVCF(sensorData->gyrZ_calibrated[0], angleData->degGyr_calb_Integral_1stHPF_LPF[1], fuzzyData->fcHPF_LPF, samplingPeriod);
//
//	/* Combine filtered accelerometer and gyroscope measurements to get the final angle */
//	angleData->angleTVCF_Deg = angleData->degAcc_1stLPF[0] + angleData->degGyr_Integral_1stHPF[0];
//	angleData->angleTVCF_calib_Deg = angleData->degAcc_calib_1stLPF[0] + angleData->degGyr_calb_Integral_1stHPF[0];
//	// angleData->angleTVCF_calib_Deg_LPF = angleData->degAcc_calib_1stLPF_LPF[0] + angleData->degGyr_calb_Integral_1stHPF_LPF[0];
//
//	/* Save current data as previous data for the next iteration */
//	angleData->degAcc_arctan[1] = angleData->degAcc_arctan[0];
//	angleData->degGyr_Integral[1] = angleData->degGyr_Integral[0];
//	angleData->degGyr_Integral_calibrated[1] = angleData->degGyr_Integral_calibrated[0];
//	angleData->degAcc_1stLPF[1] = angleData->degAcc_1stLPF[0];
//	angleData->degAcc_calib_1stLPF[1] = angleData->degAcc_calib_1stLPF[0];
//	angleData->degGyr_Integral_1stHPF[1] = angleData->degGyr_Integral_1stHPF[0];
//	angleData->degGyr_calb_Integral_1stHPF[1] = angleData->degGyr_calb_Integral_1stHPF[0];
//
//	// angleData->degAcc_calib_1stLPF_LPF[1] = angleData->degAcc_calib_1stLPF_LPF[0];
//	// angleData->degGyr_calb_Integral_1stHPF_LPF[1] = angleData->degGyr_calb_Integral_1stHPF_LPF[0];
//}

/*
 * 1st order Low Pass Filter (LPF) using bilinear transform
 *
 * Parameters:
 *   currAngle - Current accelerometer angle
 *   prevAngle - Previous accelerometer angle
 *   prevFilteredAngle - Previous filtered angle
 *   fc - Cutoff frequency for the filter
 *   samplingPeriod - Sampling period of the measurements
 *
 * Returns:
 *   Filtered angle after applying low pass filter
 */
//static float Apply1stLPF_TVCF(float currAngle, float prevAngle, float prevFilteredAngle, float cutoffFreq, float samplingPeriod)
//{
//	float c = M_PI * samplingPeriod * cutoffFreq;
//	float currFilteredAngle = ((1 - c) * prevFilteredAngle + c * prevAngle + c * currAngle) / (1 + c);
//	return currFilteredAngle;
//}

/*
 * 1st order High Pass Filter (HPF) with integration using bilinear transform
 *
 * Parameters:
 *   currValue - Current gyroscope measurement
 *   prevFilteredValue - Previous filtered gyroscope value
 *   fc - Cutoff frequency for the filter
 *   samplingPeriod - Sampling period of the measurements
 *
 * Returns:
 *   Filtered gyroscope value after applying high pass filter
 */
//static float Apply1stHPFwithIntegral_TVCF(float currValue, float prevFilteredValue, float cutoffFreq, float samplingPeriod)
//{
//	float c = M_PI * samplingPeriod * cutoffFreq;
//	float currFilteredValue = (samplingPeriod * currValue + (1 - c) * prevFilteredValue) / (1 + c);
//	return currFilteredValue;
//}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 예전 SBS D Forum에 사용되었던 Gait Phase 관련 함수 */
/* ------------------- Gait Data Estimation ------------------- */
// static void RunGaitFunc(GaitData_t* gaitData, FilteredAngleData_t* filteredAngleData, NormData_t* normData, IMUProcessedData_t* sensorData)
// {
// 	// Thigh Vel LPF (First)
// 	filteredAngleData->thighVelLPF[0] = ApplyLPF(&filteredAngleData->isInitialized_VelLPF, &filteredAngleData->thighVelLPF[1],
// 			filteredAngleData->thighVelDegFinal_calib[0], THIGH_VEL_FIRST_LPF_SMOOTHING_FACTOR);

// 	if (gaitData->gaitLoopCnt < 1000) { // Before 1sec
// 		filteredAngleData->thighVelLPF[0] = 0;
// 	}

// 	// Calculate the absolute value of thigh velocity after LPF
// 	filteredAngleData->thighVelLPFAbs = GetAbsoluteValue(filteredAngleData->thighVelLPF[0]);
// 	// Subtracting the threshold helps in filtering out minor fluctuations and focusing on significant changes.
// 	float tempThighVelLPFAbs = GetMaxValue(0, filteredAngleData->thighVelLPFAbs - THIGH_VEL_THRESHOLD);
// 	// Update modeCheck using an Exponential Moving Average (EMA) to smooth the signal for reliable gait mode detection.
// 	// Here, 0.993(1 - EMA_ALPHA) and 0.007(EMA_ALPHA) are the weights for the old and new values, respectively.
// 	// This method allows the latest data to be gradually reflected, smoothing out abrupt changes.
// 	gaitData->modeCheck = (1 - MODE_CHECK_EMA_ALPHA) * gaitData->modeCheck + MODE_CHECK_EMA_ALPHA * tempThighVelLPFAbs;

// 	// Thigh Degree LPF
// 	filteredAngleData->thighDegLPF[0] = ApplyLPF(&filteredAngleData->isInitialized_DegLPF, &filteredAngleData->thighDegLPF[1],
// 			filteredAngleData->thighDegFinal_calib[0], THIGH_DEG_FIRST_LPF_SMOOTHING_FACTOR);

// 	gaitData->gaitMode[0] = gaitData->gaitMode[1];
// 	if (gaitData->gaitLoopCnt > 1000) { // After 1sec
// 		// mode = GAIT_MODE_WALK : Walking, mode = GAIT_MODE_STOP : Stop
// 		if (gaitData->gaitMode[1] == GAIT_MODE_STOP && gaitData->modeCheck > MODE_CHECK_THRESHOLD_HIGH) {
// 			gaitData->gaitMode[0] = GAIT_MODE_WALK;
// 		} else if (gaitData->gaitMode[1] == GAIT_MODE_WALK && gaitData->modeCheck < MODE_CHECK_THRESHOLD_LOW) {
// 			gaitData->gaitMode[0] = GAIT_MODE_STOP;
// 		}
// 	}

// 	// Thigh Vel LPF (Second)
// 	filteredAngleData->thighVelLPF2[0] = (1 - THIGH_VELLPF2_SMOOTHING_FACTOR) * filteredAngleData->thighVelLPF2[1] +
// 			THIGH_VELLPF2_SMOOTHING_FACTOR * filteredAngleData->thighVelLPF[0];

// 	// Check if enough time has passed (10ms)
// 	if (gaitData->gaitLoopCnt > 10) {
// 		if (gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 			// Leg front->back moving point
// 			if (filteredAngleData->thighVelLPF2[1] > 0 && filteredAngleData->thighVelLPF2[0] < 0) {
// 				if (gaitData->firstPeriodCheck == 0) {
// 					gaitData->gaitPeriod = gaitData->gaitLoopCnt - gaitData->gaitLoopCntPrev;
// 					// Clamp gaitPeriod within 200(min) to 2000(max)
// 					if (gaitData->gaitPeriod > GAIT_PERIOD_MAX) {
// 						gaitData->gaitPeriod = GAIT_PERIOD_MAX;
// 					} else if (gaitData->gaitPeriod < GAIT_PERIOD_MIN) {
// 						gaitData->gaitPeriod = GAIT_PERIOD_MIN;
// 					}
// 				} else {
// 					gaitData->firstPeriodCheck = 0;
// 				}

// 				// Update cutoff frequency
// 				gaitData->gaitCutoffFreq = 2 * M_PI / (gaitData->gaitPeriod * GAIT_CONTROL_PERIOD);
// 				gaitData->gaitLoopCntPrev = gaitData->gaitLoopCnt;
// 			}
// 		} else if (gaitData->gaitMode[0] == GAIT_MODE_STOP) {
// 			gaitData->firstPeriodCheck = 1;
// 		}
// 	}

// 	// Smooth the cutoff frequency using Exponential Moving Average (EMA)
// 	// EMA is a type of moving average that places a greater weight and significance on the most recent data points.
// 	// It is commonly used for smoothing data, reducing noise, and identifying trends.
// 	// Formula: EMA_today = alpha * Value_today + (1 - alpha) * EMA_yesterday
// 	gaitData->gaitCutoffFreqLPFSmooth = (1 - GAIT_CUTOFF_EMA_ALPHA) * gaitData->gaitCutoffFreqLPFSmooth + 
// 			GAIT_CUTOFF_EMA_ALPHA * gaitData->gaitCutoffFreq;

// 	// Update band-pass filter for angle
// 	filteredAngleData->thighDegBPF[0] = UpdateBPF_ForPeakThighDeg(&filteredAngleData->isInitialized_DegBPF,
// 			filteredAngleData->thighDegBPFprevOutput, filteredAngleData->thighDegBPFprevInput,
// 			filteredAngleData->thighDegFinal_calib[0], gaitData->gaitCutoffFreqLPFSmooth);
// 	// UpdateBPF_PeakCutoff_ForDeg(BPFState_Deg, filteredAngleData->thighDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 	// filteredAngleData->thighDegBPF[0] = WIDM_BPF_Peak_Prof_1((double)filteredAngleData->thighDegFinal, gaitData->gaitCutoffFreqLPFSmooth);
// 	filteredAngleData->thighDegBPFAbs = GetAbsoluteValue(filteredAngleData->thighDegBPF[0]);

// 	// Update peak amplitude for angle
// 	if (gaitData->gaitMode[0] == GAIT_MODE_WALK && gaitData->firstPeriodCheck == 0 &&
// 			filteredAngleData->thighDegBPF[1] < filteredAngleData->thighDegBPF[2] &&
// 			filteredAngleData->thighDegBPF[0] > filteredAngleData->thighDegBPF[1]) {
// 		// At Leg Backward->Forward transition, get Amplitude of angle(deg)
// 		filteredAngleData->peakAmpThighDeg = filteredAngleData->thighDegBPFAbs;
// 	}

// 	// Update band-pass filter for velocity
// 	filteredAngleData->thighVelBPF[0] = UpdateBPF_ForPeakThighVel(&filteredAngleData->isInitialized_VelBPF,
// 			filteredAngleData->thighVelBPFprevOutput, filteredAngleData->thighVelBPFprevInput,
// 			filteredAngleData->thighVelDegFinal_calib[0], gaitData->gaitCutoffFreqLPFSmooth);
// 	// UpdateBPF_PeakCutoff_ForVel(BPFState_Vel, filteredAngleData->thighVelDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 	filteredAngleData->thighVelBPFAbs = GetAbsoluteValue(filteredAngleData->thighVelBPF[0]);

// 	// Update peak amplitude for velocity
// 	if (gaitData->gaitMode[0] == GAIT_MODE_WALK && gaitData->firstPeriodCheck == 0 &&
// 			filteredAngleData->thighDegBPF[1] < 0 && filteredAngleData->thighDegBPF[0] > 0) {
// 		// At maximum slope of filteredAngleData->thighDegBPF, get Amplitude of velocity(deg/s)
// 		filteredAngleData->peakAmpThighVel = filteredAngleData->thighVelBPFAbs;
// 	}

// 	// Smooth the peak amplitudes
// 	// Formula: EMA_today = alpha * Value_today + (1 - alpha) * EMA_yesterday
// 	if (gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 		filteredAngleData->peakAmpSmoothThighDeg = (1 - PEAK_AMP_DEG_EMA_ALPHA) * filteredAngleData->peakAmpSmoothThighDeg + 
// 				PEAK_AMP_DEG_EMA_ALPHA * filteredAngleData->peakAmpThighDeg;
// 		filteredAngleData->peakAmpSmoothThighVel = (1 - PEAK_AMP_VEL_EMA_ALPHA) * filteredAngleData->peakAmpSmoothThighVel +
// 				PEAK_AMP_VEL_EMA_ALPHA * filteredAngleData->peakAmpThighVel;
// 	}

// 	// Normalize angle and velocity if enough time has passed (1s)
// 	if (gaitData->gaitLoopCnt > 1000) {
// 		if (filteredAngleData->peakAmpSmoothThighDeg == 0 || filteredAngleData->peakAmpSmoothThighVel == 0) {
// 			filteredAngleData->thighDegNorm = 0;
// 			filteredAngleData->thighVelNorm = 0;
// 		} else {
// 			filteredAngleData->thighDegNorm = filteredAngleData->thighDegBPF[0] / filteredAngleData->peakAmpSmoothThighDeg;
// 			filteredAngleData->thighVelNorm = (-1.0f) * filteredAngleData->thighVelBPF[0] / filteredAngleData->peakAmpSmoothThighVel;
// 		}
// 	}

// 	// Calculate gait phase
// 	// 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....
// 	gaitData->gaitPhase = gaitData->gaitMode[0] * (atan2((-1)*filteredAngleData->thighVelNorm,
// 			(-1)*filteredAngleData->thighDegNorm)) / (2 * M_PI) * 100 + 50;

// 	BFlagArray[32] = 0;	// 10%
// 	BFlagArray[33] = 0;	// 20%
// 	BFlagArray[34] = 0;	// 30%
// 	BFlagArray[35] = 0;	// 40%
// 	BFlagArray[36] = 0;	// 50%
// 	BFlagArray[37] = 0;	// 60%
// 	BFlagArray[38] = 0;	// 70%
// 	BFlagArray[39] = 0;	// 80%
// 	BFlagArray[40] = 0;	// 90%

// 	BFlagArray[41] = 0;	// 1.8 ~ 2 sec
// 	BFlagArray[42] = 0;	// 1.5 ~ 1.8 sec
// 	BFlagArray[43] = 0;	// 1.2 ~ 1.5 sec
// 	BFlagArray[44] = 0;	// 0.8 ~ 1.2 sec
// 	BFlagArray[45] = 0;	// 0 ~ 0.8 sec

// 	BFlagArray[101] = 0; // 50%	 Leg back->front
// 	BFlagArray[102] = 0; // 0%	 Leg front->back & Gait Count++
// 	BFlagArray[103] = 0; // 75%	 Leg middle->front
// 	BFlagArray[104] = 0; // 25%	 Leg middle->back

// 	/* Update B[n] Array */
// 	if (gaitData->gaitLoopCnt > 1000 && gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 		if (gaitData->gaitPhase > GAIT_PHASE_10PER && gaitData->gaitPhasePrev < GAIT_PHASE_10PER) {
// 			BFlagArray[32] = 1;	// 10%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_20PER && gaitData->gaitPhasePrev < GAIT_PHASE_20PER) {
// 			BFlagArray[33] = 1;	// 20%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_30PER && gaitData->gaitPhasePrev < GAIT_PHASE_30PER) {
// 			BFlagArray[34] = 1;	// 30%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_40PER && gaitData->gaitPhasePrev < GAIT_PHASE_40PER) {
// 			BFlagArray[35] = 1;	// 40%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_50PER && gaitData->gaitPhasePrev < GAIT_PHASE_50PER) {
// 			BFlagArray[36] = 1;	// 50%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_60PER && gaitData->gaitPhasePrev < GAIT_PHASE_60PER) {
// 			BFlagArray[37] = 1;	// 60%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_70PER && gaitData->gaitPhasePrev < GAIT_PHASE_70PER) {
// 			BFlagArray[38] = 1;	// 70%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_80PER && gaitData->gaitPhasePrev < GAIT_PHASE_80PER) {
// 			BFlagArray[39] = 1;	// 80%
// 		}
// 		if (gaitData->gaitPhase > GAIT_PHASE_90PER && gaitData->gaitPhasePrev < GAIT_PHASE_90PER) {
// 			BFlagArray[40] = 1;	// 90%
// 		}
// 	}

// 	filteredAngleData->thighDegLPFAbs = GetAbsoluteValue(filteredAngleData->thighDegLPF[0]);

// 	// Smooth the NeutralPosture
// 	// Formula: EMA_today = alpha * Value_today + (1 - alpha) * EMA_yesterday
// 	// Get (Stop & Standing) Posture's Angle
// 	if (filteredAngleData->thighDegLPFAbs < STAND_STOP_DEG_THRESHOLD && gaitData->gaitMode[0] == GAIT_MODE_STOP) {
// 		filteredAngleData->NeutralPosture = (1 - NEUTRAL_POS_EMA_ALPHA) * filteredAngleData->NeutralPosture + 
// 				NEUTRAL_POS_EMA_ALPHA * filteredAngleData->thighDegLPF[0];
// 	}

// 	filteredAngleData->thighDegUnbiased[0] = filteredAngleData->thighDegLPF[0] - filteredAngleData->NeutralPosture;

// 	// If the thigh angle (raised to the fourth power) is greater than 10000, consider it as walking
// 	if (gaitData->gaitMode[0] == GAIT_MODE_WALK && pow(filteredAngleData->thighDegBPF[0], 4) > THIGH_ANGLE_WALK_THRESHOLD) {
// 		// 조건이 충족되면 보행 중으로 간주
// 		if (gaitData->firstStepDone == 1) {
// 			// 보행이 처음 시작될 때
// 			gaitData->isWalking  = 1; // 현재 보행 중
// 			gaitData->lastStepTime = gaitData->gaitLoopCnt; // 현재 시점 저장
// 			gaitData->firstStepDone = 0; // 보행이 시작되었음을 표시
// 		} else {
// 			// 보행 중일 때
// 			gaitData->isWalking = 1; // 현재 보행 중
// 			gaitData->lastStepTime = gaitData->gaitLoopCnt; // 현재 시점 저장
// 		}
// 	} else {
// 		// When walking stops and the thigh angle (raised to the fourth power) is less than 10000
// 		if (gaitData->gaitMode[0] == GAIT_MODE_WALK && pow(filteredAngleData->thighDegBPF[0], 4) < THIGH_ANGLE_WALK_THRESHOLD && gaitData->firstStepDone == 0) {
// 			// 보행이 중지되었을 때
// 			if (gaitData->gaitLoopCnt - gaitData->lastStepTime < MAX_STEP_DURATION) { // If the time elapsed since the last step is less than 1100 loops, continue walking
// 				gaitData->isWalking = 1; // 보행 지속
// 			} else {
// 				gaitData->isWalking = 0; // 보행 중지
// 				gaitData->firstStepDone = 1; // 보행이 종료되었음을 표시
// 			}
// 		}
// 	}

// 	// Check if gaitPhase is within a valid range
// 	if (gaitData->gaitPhase > GAIT_PHASE_MIN && gaitData->gaitPhase < GAIT_PHASE_MAX) {
// 		// Ensure gaitPhase does not decrease; update it only if it increases
// 		if (gaitData->gaitPhase > gaitData->gaitPhasePrev) {
// 			gaitData->gaitPhase = gaitData->gaitPhase;	// Keep current gaitPhase
// 		} else {
// 			gaitData->gaitPhase = gaitData->gaitPhasePrev;	// Revert to previous gaitPhase
// 		}
// 	}

// 	if (gaitData->gaitLoopCnt > 1000) {
// 		// B102: Set flag when specific conditions involving thigh angles and velocities are met
// 		if (filteredAngleData->thighDegUnbiased[0] > 10 && filteredAngleData->thighVelLPF[1] > 0 &&
// 				filteredAngleData->thighVelLPF[0] < 0 && BFlagArray[108] == 0 && gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 			if (gaitData->isWalking == 1) {
// 				BFlagArray[102] = 1;
// 				BFlagArray[108] = 1; // Change B108 to 1, acting as a token
// 				gaitData->gaitCount++;
// 			}
// 		}
// 		// B101: Set flag when another set of specific conditions involving thigh angles and velocities are met
// 		if (filteredAngleData->thighDegUnbiased[0] < 3 && filteredAngleData->thighVelLPF[1] < 0 &&
// 				filteredAngleData->thighVelLPF[0] > 0 && BFlagArray[107] == 0 && gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 			if (gaitData->isWalking == 1) {
// 				BFlagArray[101] = 1;
// 				BFlagArray[107] = 1; // Change B107 to 1, acting as a token
// 			}
// 		}
// 		// Reset B108 when a different condition involving thigh angles is met
// 		if (filteredAngleData->thighDegUnbiased[0] < 3 && filteredAngleData->thighDegUnbiased[1] > 3) {
// 			BFlagArray[108] = 0; // Reset B108 to 0, token concept
// 		}
// 		// Reset B107 when another condition involving thigh angles is met
// 		if (filteredAngleData->thighDegUnbiased[0] > 5 && filteredAngleData->thighDegUnbiased[1] < 5) {
// 			BFlagArray[107] = 0; // Reset B107 to 0, token concept
// 		}
// 		// B103: Set flag when thigh velocity exceeds a specific positive value
// 		if (filteredAngleData->thighVelLPF[0] > 8) {
// 			BFlagArray[103] = 1;
// 		}
// 		// B104: Set flag when thigh velocity exceeds a specific negative value
// 		if (filteredAngleData->thighVelLPF[0] < -8) {
// 			BFlagArray[104] = 1;
// 		}
// 	}

// 	// Perform additional actions based on gait period only after a certain number of gait loops
// 	if (gaitData->gaitLoopCnt > 1000) {
// 		if (gaitData->gaitPeriod > 1800 && gaitData->gaitPeriod <= 2000) {
// 			BFlagArray[41] = 1; // Set flag for gait period between 1.8sec and 2sec
// 		} else if (gaitData->gaitPeriod > 1400 && gaitData->gaitPeriod <= 1800) {
// 			BFlagArray[42] = 1; // Set flag for gait period between 1.4sec and 1.8sec
// 		} else if (gaitData->gaitPeriod > 1100 && gaitData->gaitPeriod <= 1400) {
// 			BFlagArray[43] = 1; // Set flag for gait period between 1.1sec and 1.4sec
// 		} else if (gaitData->gaitPeriod > 700 && gaitData->gaitPeriod <= 1100) {
// 			BFlagArray[44] = 1; // Set flag for gait period between 0.7sec and 1.1sec
// 		} else if (gaitData->gaitPeriod <= 700) {
// 			BFlagArray[45] = 1; // Set flag for gait period between 0sec and 0.7sec
// 		}
// 	}

// 	// Update previous values//
// 	BFlagArray[31] = gaitData->gaitMode[0];
// 	gaitData->gaitMode[1] = gaitData->gaitMode[0];
// 	gaitData->gaitPhasePrev = gaitData->gaitPhase;
// 	filteredAngleData->thighVelLPF[1] = filteredAngleData->thighVelLPF[0];
// 	filteredAngleData->thighDegLPF[1] = filteredAngleData->thighDegLPF[0];
// 	filteredAngleData->thighVelLPF2[1] = filteredAngleData->thighVelLPF2[0];
// 	filteredAngleData->thighDegBPF[2] = filteredAngleData->thighDegBPF[1];
// 	filteredAngleData->thighDegBPF[1] = filteredAngleData->thighDegBPF[0];
// 	filteredAngleData->thighVelBPF[2] = filteredAngleData->thighVelBPF[1];
// 	filteredAngleData->thighVelBPF[1] = filteredAngleData->thighVelBPF[0];
// 	filteredAngleData->thighDegUnbiased[1] = filteredAngleData->thighDegUnbiased[0];

// 	gaitData->gaitLoopCnt++;	// Check this Loop Count
// }

// static void RunGaitFunction_K10(GaitObj_t* gaitObj, ISI_Flag_t* ISIFlags, WIDM_BPFState* BPFState_Deg, WIDM_BPFState* BPFState_Vel, WIDM_LPFState* LPFState)
// {
// 	gaitObj->gaitFuncLoopCnt++; 			                			// Check this Loop Count(1ms)

// 		// Thigh Degree LPF
// 	filteredAngleData->thighDegLPF[0] = ApplyLPF(&filteredAngleData->isInitialized_DegLPF, &filteredAngleData->thighDegLPF[1],
// 											filteredAngleData->thighDegFinal, THIGH_DEG_FIRST_LPF_SMOOTHING_FACTOR);

// 	// Thigh Vel LPF (First)
// 	filteredAngleData->thighVelLPF[0] = ApplyLPF(&filteredAngleData->isInitialized_VelLPF, &filteredAngleData->thighVelLPF[1],
// 											filteredAngleData->thighVelDegFinal, THIGH_VEL_FIRST_LPF_SMOOTHING_FACTOR);

// 	gaitObj->VelData.velLPF_K10 = ApplyLPF_CustomAlpha(LPFState, filteredAngleData->thighVelDegFinal, LPF_SMOOTHING_FACTOR);    // velLPF : velRaw -> Low-pass filtering
// 	// gaitObj->VelData.velLPF_K10 = WIDM_LPF_walking_Prof(filteredAngleData->thighVelDegFinal);    // velLPF : velRaw -> Low-pass filtering
// 	filteredAngleData->thighVelLPFAbs = GetAbsoluteValue(gaitObj->VelData.velLPF_K10);
// 	double tempVelLPFAbs = GetMaxValue(0, filteredAngleData->thighVelLPFAbs - 15);

// 	// GetEncoderDegree(&widmAngleDataObj);

// 	if (gaitObj->gaitFuncLoopCnt > 1) {
// 		gaitObj->AngleData.incDegLPF[0] = 0.98 * gaitObj->AngleData.incDegLPF[1] + 0.02 * gaitObj->AngleData.incDegRaw;
// 	}

// //	gaitObj->AngleData.incDegTrig[0] = gaitObj->AngleData.incDegRaw;
// //	gaitObj->VelData.incVel = gaitObj->VelData.incVelRaw;
// //
// //	gaitObj->AngleData.filteredIncDeg = GetAbsoluteValue(gaitObj->AngleData.incDegRaw);
// //	gaitObj->AngleData.filteredIncDeg = WIDM_LPF_walking_Prof(gaitObj->AngleData.incDegRaw);
// //	gaitObj->AngleData.incDegTrig[0] = gaitObj->AngleData.filteredIncDeg;
// //
// //	gaitObj->VelData.filteredIncVel = GetAbsoluteValue(gaitObj->VelData.incVel);
// //	gaitObj->VelData.filteredIncVel = WIDM_LPF_walking_Prof(gaitObj->VelData.incVel);
// //	gaitObj->VelData.incVelTrig[0] = gaitObj->VelData.filteredIncVel;

// 	gaitData->modeCheck = 0.993 * gaitData->modeCheck + 0.007 * tempVelLPFAbs;

// 	if (gaitObj->gaitFuncLoopCnt > 1) {
// 		filteredAngleData->thighDegLPF[0] = 0.98 * filteredAngleData->thighDegLPF[1] + 0.02 * filteredAngleData->thighDegFinal;

// 		gaitData->gaitMode[0] = gaitData->gaitMode[1];

// 		if (gaitObj->gaitFuncLoopCnt > 1000) {
// 			if (gaitData->gaitMode[1] == 0 && gaitData->modeCheck > 4) {     // mode = 1 : Walking, mode = 0 : Stop  // modeCheck = 4
// 				gaitData->gaitMode[0] = GAIT_MODE_WALK;
// 			} else if (gaitData->gaitMode[1] == 1 && gaitData->modeCheck < 1) {
// 				gaitData->gaitMode[0] = GAIT_MODE_STOP;
// 			}
// 		}

// 		sensorData->gyroLPF[0] = 0.98 * sensorData->gyroLPF[1] + 0.02 * gaitObj->gyrZRaw[0];
// //		sensorData->gyroLPF[0] = 0.997 * sensorData->gyroLPF[1] + 0.003 * gaitObj->gyrZRaw[0];		// 1NE changed
// 		filteredAngleData->thighVelLPF2[0] = 0.9997 * filteredAngleData->thighVelLPF2[1] + 0.0003 * sensorData->gyroLPF[0];
// 	}

// 	if (gaitObj->gaitFuncLoopCnt > 10) {
// 		if (gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 			if (filteredAngleData->thighVelLPF2[1] > 0 && filteredAngleData->thighVelLPF2[0] < 0) {								// Leg front->back moving point
// 				if (gaitData->firstPeriodCheck == 0) {
// 					gaitData->gaitPeriod = gaitObj->gaitFuncLoopCnt - gaitObj->timeStampPrev;
// 					if (gaitData->gaitPeriod > GAIT_PERIOD_MAX) {
// 						gaitData->gaitPeriod = GAIT_PERIOD_MAX;
// 					} else if (gaitData->gaitPeriod < GAIT_PERIOD_MIN) {
// 						gaitData->gaitPeriod = GAIT_PERIOD_MIN;
// 					}
// 				} else {
// 					gaitData->firstPeriodCheck = 0;
// 				}

// 				gaitObj->GaitFilter.cutoffFreq = 2 * M_PI / (gaitData->gaitPeriod * 0.001);
// 				gaitObj->timeStampPrev = gaitObj->gaitFuncLoopCnt;
// 			}
// 		} else if (gaitData->gaitMode[0] == GAIT_MODE_STOP) {
// 			gaitData->firstPeriodCheck = 1;
// 		}
// 	}

// 	gaitObj->GaitFilter.cutoffFreqSmooth = 0.992 * gaitObj->GaitFilter.cutoffFreqSmooth + 0.008 * gaitObj->GaitFilter.cutoffFreq;

// 	filteredAngleData->thighDegBPF[0] = UpdateBPF_PeakCutoff_ForDeg(BPFState_Deg, filteredAngleData->thighDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 	// filteredAngleData->thighDegBPF[0] = WIDM_BPF_Peak_Prof_1(filteredAngleData->thighDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 	filteredAngleData->thighDegBPFAbs = GetAbsoluteValue(filteredAngleData->thighDegBPF[0]);

// 	if (gaitObj->gaitFuncLoopCnt > 3) {
// 		if (gaitData->gaitMode[0] == GAIT_MODE_WALK && gaitData->firstPeriodCheck == 0 && filteredAngleData->thighDegBPF[1] < filteredAngleData->thighDegBPF[2] && filteredAngleData->thighDegBPF[0] > filteredAngleData->thighDegBPF[1]) {
// 			filteredAngleData->peakAmpThighDeg = filteredAngleData->thighDegBPFAbs;	// At Leg Backward->Forward transition, get Amplitude of angle(deg)
// 		}

// 		filteredAngleData->thighVelBPF[0] = UpdateBPF_PeakCutoff_ForVel(BPFState_Vel, filteredAngleData->thighVelDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 		// filteredAngleData->thighVelBPF[0] = WIDM_BPF_Peak_Prof_2(filteredAngleData->thighVelDegFinal, gaitObj->GaitFilter.cutoffFreqSmooth);
// 		filteredAngleData->thighVelBPFAbs = GetAbsoluteValue(filteredAngleData->thighVelBPF[0]);

// 		if (gaitData->gaitMode[0] == GAIT_MODE_WALK && gaitData->firstPeriodCheck == 0 && filteredAngleData->thighDegBPF[1] < 0 && filteredAngleData->thighDegBPF[0] > 0) {
// 			filteredAngleData->peakAmpThighVel = filteredAngleData->thighVelBPFAbs;			// At maximum slope of degBPF, get Amplitude of velocity(deg/s)
// 		}
// 	}

// 	if (gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 		filteredAngleData->peakAmpSmoothThighDeg = 0.998 * filteredAngleData->peakAmpSmoothThighDeg + 0.002 * filteredAngleData->peakAmpThighDeg;
// 		filteredAngleData->peakAmpSmoothThighVel = 0.998 * filteredAngleData->peakAmpSmoothThighVel + 0.002 * filteredAngleData->peakAmpThighVel;
// 	}

// 	if (gaitObj->gaitFuncLoopCnt > 1000) {
// 		filteredAngleData->thighDegNorm = filteredAngleData->thighDegBPF[0] / filteredAngleData->peakAmpSmoothThighDeg;
// 		filteredAngleData->thighVelNorm = -filteredAngleData->thighVelBPF[0] / filteredAngleData->peakAmpSmoothThighVel;
// 	}

// 	// 0~100%		// "1" : 50~75%, "2" : 75~100%, "3" : 0~25%, "4" : 25~50% ....
// 	gaitData->gaitPhase = gaitData->gaitMode[0] * (atan2((-1)*filteredAngleData->thighVelNorm, (-1)*filteredAngleData->thighDegNorm)) / (2 * M_PI) * 100 + 50;

// 	BFlagArray[32] = 0;		// 10%
// 	BFlagArray[33] = 0;		// 20%
// 	BFlagArray[34] = 0;		// 30%
// 	BFlagArray[35] = 0;		// 40%
// 	BFlagArray[36] = 0;		// 50%

// 	BFlagArray[102] = 0;		// 0%	 Leg front->back	&   Gait Count ++
// 	BFlagArray[101] = 0;		// 50%	 Leg back->front
// 	BFlagArray[103] = 0;		// 75%	 Leg middle->front
// 	BFlagArray[104] = 0;		// 25%	 Leg middle->back

// 	BFlagArray[37] = 0;		// 60%
// 	BFlagArray[38] = 0;		// 70%
// 	BFlagArray[39] = 0;		// 80%
// 	BFlagArray[40] = 0;		// 90%

// 	ISIFlags->I15_Flag = 0;		// after B8 Flag on, swing for extension
// 	ISIFlags->I16_Flag = 0;		// 1NE's revised Extension timing


// 	if (gaitObj->gaitFuncLoopCnt > 1000 && gaitData->gaitMode[0] == GAIT_MODE_WALK) {
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold1 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold1){
// 			BFlagArray[32] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold2 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold2){
// 			BFlagArray[33] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold3 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold3){
// 			BFlagArray[34] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold4 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold4){
// 			BFlagArray[35] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold5 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold5){
// 			BFlagArray[36] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold6 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold6){
// 			BFlagArray[37] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold7 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold7){
// 			BFlagArray[38] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold8 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold8){
// 			BFlagArray[39] = 1;
// 		}
// 		if (gaitData->gaitPhase > gaitObj->phaseThreshold9 && gaitData->gaitPhasePrev < gaitObj->phaseThreshold9){
// 			BFlagArray[40] = 1;
// 		}
// 	}

// 	filteredAngleData->thighDegLPFAbs = GetAbsoluteValue(filteredAngleData->thighDegLPF[0]);

// 	if (filteredAngleData->thighDegLPFAbs < 20 && gaitData->gaitMode[0] == GAIT_MODE_STOP) {
// 		filteredAngleData->NeutralPosture = 0.999 * filteredAngleData->NeutralPosture + 0.001 * filteredAngleData->thighDegLPF[0];				// Get (Stop & Standing) Posture's Angle
// 	}

// 	filteredAngleData->thighDegUnbiased[0] = filteredAngleData->thighDegLPF[0] - filteredAngleData->NeutralPosture;

// 	if (gaitObj->gaitFuncLoopCnt > 1000) {
// 		if (filteredAngleData->thighDegUnbiased[0] > 10 && sensorData->gyroLPF[1] > 0 && 
// 			sensorData->gyroLPF[0] < 0 && BFlagArray[108] == 0 && gaitData->gaitMode[0] == GAIT_MODE_WALK){
// 			BFlagArray[102] = 1;
// 			BFlagArray[108] = 1;
// 			gaitObj->gaitCount++;
// 		}
// 		if (filteredAngleData->thighDegUnbiased[0] < 3 && sensorData->gyroLPF[1] < 0 && 
// 			sensorData->gyroLPF[0] > 0 && BFlagArray[107] == 0 && gaitData->gaitMode[0] == GAIT_MODE_WALK){
// 			BFlagArray[101] = 1;
// 			BFlagArray[107] = 1;
// 			// B8stack++;
// 		}
// 		if (filteredAngleData->thighDegUnbiased[0] < 3 && filteredAngleData->thighDegUnbiased[1] > 3) {
// 			BFlagArray[108] = 0;
// 		}
// 		if (filteredAngleData->thighDegUnbiased[0] > 5 && filteredAngleData->thighDegUnbiased[1] < 5) {
// 			BFlagArray[107] = 0;
// 		}

//         if (sensorData->gyroLPF[0] > 5) {
//             BFlagArray[103] = 1;
//         }
//         if (sensorData->gyroLPF[0] < -5) {
//             BFlagArray[104] = 1;
//         }

// 		if (BFlagArray[107] == 1) {
// 			if (gaitObj->AngleData.incDegTrig[0] < gaitObj->AngleData.incDegTrig[1] && gaitObj->AngleData.incDegTrig[1] > gaitObj->AngleData.incDegTrig[2] && gaitObj->AngleData.incDegTrig[0] > 40) {
// 			// if (gaitObj->VelData.incVelTrig[0] < gaitObj->VelData.incVelTrig[1] && gaitObj->VelData.incVelTrig[1] > gaitObj->VelData.incVelTrig[2]) {
// 			// if (gaitObj->AngleData.incDegTrig[0] < gaitObj->AngleData.incDegTrig[1] && gaitObj->AngleData.incDegTrig[1] > gaitObj->AngleData.incDegTrig[2]) {
// 				// if (gaitObj->VelData.incVelTrig[0] < 0 && gaitObj->VelData.incVelTrig[1] > 0 && gaitObj->VelData.incVelTrig[1] > gaitObj->VelData.incVelTrig[2]) {
//         			ISIFlags->I15_Flag = 1;
// 					I15Stack++;
//     			// }
// 				// ISIFlags->I15_Flag = 1;
// 				// B15stack++;
// 			}
// 		}

// 		// Added by 1NE //
// 		gaitObj->chk1Prev = gaitObj->chk1;
// 		if (gaitObj->gaitFuncLoopCnt - gaitObj->chk1 > 0.6 * gaitData->gaitPeriod) {
// 			if (gaitObj->AngleData.incDegLPF[2] < gaitObj->AngleData.incDegLPF[1] && gaitObj->AngleData.incDegLPF[0] < gaitObj->AngleData.incDegLPF[1] && gaitObj->AngleData.incDegLPF[1] > 10) {
// 				ISIFlags->I16_Flag = 1;
// 				I16Stack++;
// 				gaitObj->chk1 = gaitObj->gaitFuncLoopCnt;
// 			} else {
// 				gaitObj->chk1 = gaitObj->chk1Prev;
// 			}
// 		}
// 	}

// 	// Update previous values//
// 	gaitData->gaitMode[1] = gaitData->gaitMode[0];
// 	ISIFlags->I1_Flag = gaitData->gaitMode[0];
// 	gaitData->gaitPhasePrev = gaitData->gaitPhase;
// 	filteredAngleData->thighDegLPF[1] = filteredAngleData->thighDegLPF[0];
// 	sensorData->gyroLPF[1] = sensorData->gyroLPF[0];
// 	filteredAngleData->thighVelLPF2[1] = filteredAngleData->thighVelLPF2[0];
// 	filteredAngleData->thighDegBPF[2] = filteredAngleData->thighDegBPF[1];
// 	filteredAngleData->thighDegBPF[1] = filteredAngleData->thighDegBPF[0];
// 	filteredAngleData->thighDegUnbiased[1] = filteredAngleData->thighDegUnbiased[0];

// 	gaitObj->AngleData.incDegTrig[2] = gaitObj->AngleData.incDegTrig[1];
// 	gaitObj->AngleData.incDegTrig[1] = gaitObj->AngleData.incDegTrig[0];

// 	gaitObj->VelData.incVelTrig[2] = gaitObj->VelData.incVelTrig[1];
// 	gaitObj->VelData.incVelTrig[1] = gaitObj->VelData.incVelTrig[0];

// 	gaitObj->AngleData.incDegLPF[2] = gaitObj->AngleData.incDegLPF[1];
// 	gaitObj->AngleData.incDegLPF[1] = gaitObj->AngleData.incDegLPF[0];
// }
