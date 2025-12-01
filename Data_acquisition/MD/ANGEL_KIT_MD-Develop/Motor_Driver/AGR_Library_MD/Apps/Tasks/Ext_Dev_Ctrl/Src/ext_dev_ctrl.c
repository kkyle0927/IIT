

#include "ext_dev_ctrl.h"

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

TaskObj_t ext_dev_ctrl_task;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
IOIF_UPRIGHT_t uprightObj;
IOIF_LS_t lsObj;
IOIF_FSR_t fsrObj;
IOIF_LP_t lpObj;
static uint32_t rawAdc2;

IOIF_NzrLedObj_t nzrObj;
//static uint8_t testUPRes = IOIF_UPRIGHT_STATUS_OK;
//static uint8_t testLSRes = IOIF_LS_STATUS_OK;
//static uint8_t testFSRRes = IOIF_FSR_STATUS_OK;
//static uint8_t testLPRes = IOIF_LP_STATUS_OK;
static uint8_t testNzrRes = IOIF_NZRLED_STATUS_OK;
#endif

IOIF_NTC_t ntcObj;
int16_t motorTemp_scaled;

#ifdef SUIT_MD_ENABLED
IOIF_FSR_t middleToeObj;
IOIF_FSR_t bigToeObj;
IOIF_FSR_t littleToeObj;
IOIF_FSR_t heelObj;

RunStall_t StallDerateCtrl;
const float stall_coeff[3][4] = {
		{0.000058, -0.007352, 0.526381, -11.271653},
		{0.000035, -0.005504, 0.488643, -11.804374},
		{0.000019, -0.003354, 0.363545, -9.486789}
};
#endif

uint16_t* rawAdc3 = NULL;    // 0 = NTC, 1 = FSR, 2 = Linear Potentiometer

// for debug
static uint8_t testTimRes = IOIF_TIM_OK;
uint32_t MCUTemp=0;


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
static void StateError_Ent(void);
static void StateError_Run(void);
static void StateError_Ext(void);

#ifdef SUIT_MD_ENABLED
static void Clear_Fault_Motor_OTP(void);
static void Fault_Motor_OTP(void);
static int EntOtpDeratingCtrl(void);
static int RunOtpDeratingCtrl(void);
static void SetDerateState(OtpDerate_t* pDerate, RunStall_t* pStallDerate, float motor_temp);
static void SetDerateCurrLimit(OtpDerate_t* pDerate);

static int EntStallDeratingCtrl(void);
static int RunStallDeratingCtrl(void);

static void SetStallCtrl_Init(RunStall_t* pStallDerate);
static void SetStallCtrl_Run(float motor_temp, SetStall_t* pSetStall, RunStall_t* pStallDerate);
#endif

/* ------------------- SDO CALLBACK ------------------- */
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
static void Set_DcMotor_Length_Cmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_DcMotor_Direction_Cmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
#endif
/* ------------------- ROUTINE ------------------- */
static int GetMotorTemp(void);

#ifdef SUIT_MD_ENABLED
static int GetFsrVal(void);
#endif

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || \
    defined (L30_MD_REV08_ENABLED)
static int GetFootVal(void);
static int GetUprightLength(void);
static int RunUprightByLen(void);
static int RunUprightByBT(void);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(ext_dev_ctrl_task)

void InitExtDevCtrl(void)
{
	/* Init */
	InitTask(&ext_dev_ctrl_task);
	InitFaultInfo(TASK_ID_EXTDEV, &ext_dev_ctrl_task.fault);

	/* UPRIGHT UNIT TEST */
	//Init Leg Upright
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	IOIF_InitUpright(&uprightObj, 0, 1, 2, LM_DOWN_Pin, LM_UP_Pin, LM_IN1_Pin, LM_IN2_Pin);
	IOIF_InitLS(&lsObj, uprightObj.slope, uprightObj.offset);

    IOIF_InitNZRLED(IOIF_TIM4, IOIF_TIM_CHANNEL_1, &nzrObj, 1000);

    IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 0);
    IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 0);
    IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_WHITE, 0);
    IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_WHITE, 0);
#endif

    if (IOIF_StartADCDMA(IOIF_ADC3, &rawAdc3, IOIF_ADC3_BUFFER_LENGTH)) {
		//TODO: Error Process
	}

	/* State Definition */
	TASK_CREATE_STATE(&ext_dev_ctrl_task, TASK_STATE_OFF,      NULL,   				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&ext_dev_ctrl_task, TASK_STATE_STANDBY,  NULL,   				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&ext_dev_ctrl_task, TASK_STATE_ENABLE,   StateEnable_Ent,     StateEnable_Run, 	StateEnable_Ext,     false);
	TASK_CREATE_STATE(&ext_dev_ctrl_task, TASK_STATE_ERROR,    StateError_Ent,   	StateError_Run,     StateError_Ext,      false);

	/* Routine Definition */
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_FOOT,             NULL,   GetFootVal, 			NULL);
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE, NULL,   GetUprightLength,       NULL);
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_LENGTH_CMD, 	NULL,   RunUprightByLen, 		NULL);
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD, 	NULL,   RunUprightByBT, 		NULL);
#endif
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_NTC, 				NULL,   GetMotorTemp,           NULL);
#ifdef SUIT_MD_ENABLED
#ifdef USE_FSR_SENSOR
	TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_GET_FSR,          NULL,   GetFsrVal,              NULL);
#endif
    TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_OTP_DERATE_CTRL,	EntOtpDeratingCtrl,	RunOtpDeratingCtrl,	NULL);
    TASK_CREATE_ROUTINE(&ext_dev_ctrl_task, ROUTINE_ID_EXTDEV_STALL_DERATE_CTRL,	EntStallDeratingCtrl,	RunStallDeratingCtrl,	NULL);
#endif

#ifdef SUIT_MD_ENABLED
	CREATE_FAULT_RECOVERY(&ext_dev_ctrl_task.fault, OVER_TEMPERATURE_MOTOR, Clear_Fault_Motor_OTP);
#endif
    /* DOD Definition */
    // DOD
	DOP_CreateDOD(TASK_ID_EXTDEV);

    // PDO
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR, 					DOP_FLOAT32, 	1, &fsrObj.fsrVolt);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_LP,  					DOP_FLOAT32, 	1, &lpObj.lpVolt);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_REF,  		DOP_FLOAT32, 	1, &uprightObj.lengthRef);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_DIRECTION_CMD,  	    DOP_UINT8, 	    1, &uprightObj.DirCmd); // TODO : need to change uint16_t
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_LENGTH_ACT,  		DOP_FLOAT32, 	1, &uprightObj.lengthAct);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_DC_DIRECTION_ACT,  	    DOP_UINT8, 	    1, &uprightObj.DirAct);
#endif

    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_NTC_MOTOR_TEMP,         DOP_FLOAT32, 	1, &ntcObj.motorTemp);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_MCU_TEMP,         		DOP_FLOAT32, 	1, &MCUTemp);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_MOTOR_TEMP_SCALING,		DOP_INT16, 		1, &motorTemp_scaled);

#ifdef SUIT_MD_ENABLED
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR_MIDDLE_TOE,         DOP_UINT16, 	1, &middleToeObj.rawValue);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR_BIG_TOE,            DOP_UINT16, 	1, &bigToeObj.rawValue);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR_LITTLE_TOE,         DOP_UINT16, 	1, &littleToeObj.rawValue);
    DOP_CreatePDO(TASK_ID_EXTDEV, PDO_ID_EXTDEV_FSR_HEEL,               DOP_UINT16, 	1, &heelObj.rawValue);
#endif

	// SDO
    DOP_COMMON_SDO_CREATE(TASK_ID_EXTDEV)

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
    DOP_CreateSDO(TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_LENGTH,    DOP_FLOAT32,  Set_DcMotor_Length_Cmd);
    DOP_CreateSDO(TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_DIRECT,    DOP_UINT8,    Set_DcMotor_Direction_Cmd);
//    DOP_CreateSDO(TASK_ID_EXTDEV, SDO_ID_EXTDEV_DC_SET_BT,    	   DOP_UINT8,    Set_DCMotor_Button_Routine);
#endif

    /* Timer 16 Callback Allocation */
	if(IOIF_StartTimIT(IOIF_TIM16) > 0){
		//TODO: ERROR PROCESS
		testTimRes = IOIF_TIM_ERROR;
	}

	IOIF_SetTimCB(IOIF_TIM16, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunExtDevCtrl, NULL);
}

void RunExtDevCtrl(void* params)
{
	RunTask(&ext_dev_ctrl_task);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run()
{
    StateTransition(&ext_dev_ctrl_task.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run()
{
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE);
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);
#endif

#ifdef SUIT_MD_ENABLED
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_NTC);
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_GET_FSR);
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_OTP_DERATE_CTRL);
    PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_STALL_DERATE_CTRL);
#endif
    StateTransition(&ext_dev_ctrl_task.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent()
{
    EntRoutines(&ext_dev_ctrl_task.routine);
}

static void StateEnable_Run()
{
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

	MCUTemp=IOIF_ReadCPUTemp(rawAdc3);

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
    IOIF_RunNZRLED(IOIF_TIM_CHANNEL_1, &nzrObj);

    if(test_cmd == 1) {
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_RED, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_NONE, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_NONE, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_NONE, 10);
        test_cmd = 0;
    } else if (test_cmd == 2) {
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_NONE, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_RED, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_NONE, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_NONE, 10);
    } else if (test_cmd == 3) {
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_NONE, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_RED, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_RED, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_NONE, 10);
    } else if (test_cmd == 4) {
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_NONE, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_RED, 10);
        //IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_NONE, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_GREEN, 10);
    } else if (test_cmd == 5) {
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED1, IOIF_COLOR_WHITE, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED2, IOIF_COLOR_WHITE, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED3, IOIF_COLOR_WHITE, 10);
        IOIF_SetNZRLED(&nzrObj, IOIF_NZR_LED4, IOIF_COLOR_WHITE, 10);
    }
#endif

#ifdef SUIT_MD_ENABLED
    Fault_Motor_OTP();
#endif
	motorTemp_scaled = ScaleFloatToInt16(ntcObj.motorTemp, TEMP_SCALING_FACTOR);

    RunRoutines(&ext_dev_ctrl_task.routine);
}

static void StateEnable_Ext()
{
    ClearRoutines(&ext_dev_ctrl_task.routine);
}

static void StateError_Ent()
{
	FaultNumCntConsistencyCheck(&ext_dev_ctrl_task.fault);
	FaultPacketDef(TASK_ID_EXTDEV, &ext_dev_ctrl_task.fault);
	Send_EMCY(&ext_dev_ctrl_task.fault.packet);
}

static void StateError_Run()
{
	GetMotorTemp();
	FaultRecoveryFuncRun(&ext_dev_ctrl_task.fault);
	FaultPacketDef(TASK_ID_EXTDEV, &ext_dev_ctrl_task.fault);

	if(ext_dev_ctrl_task.fault.faultNumCnt == 0){
		ext_dev_ctrl_task.fault.faultBit = 0;
		StateTransition(&ext_dev_ctrl_task.stateMachine, ext_dev_ctrl_task.fault.restart_state);
	}
}

static void StateError_Ext()
{
	InitFaultInfo(TASK_ID_EXTDEV, &ext_dev_ctrl_task.fault);
	Send_EMCY(&ext_dev_ctrl_task.fault.packet);
}

/* ------------------- SDO CALLBACK ------------------- */

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
static void Set_DcMotor_Length_Cmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&uprightObj.lengthRef, (float*)req->data, sizeof(uprightObj.lengthRef));
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void Set_DcMotor_Direction_Cmd(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
    memcpy(&uprightObj.DirCmd, (float*)req->data, sizeof(uprightObj.DirCmd));
    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}
#endif

/* ------------------- ROUTINE ------------------- */
static int GetMotorTemp()
{
    uint8_t status = 0;
    status = IOIF_GetNTCTemp(&ntcObj, rawAdc3[0]);
    if (status != IOIF_NTC_STATUS_OK) {
        return status;
    }

    return status;
}

#ifdef SUIT_MD_ENABLED
// For SUIT FSR App Version
#ifdef USE_FSR_SENSOR
static int GetFsrVal()
{
    uint8_t status = 0;

    IOIF_CalFSRVolt(&middleToeObj,  rawAdc3[1]);
    IOIF_CalFSRVolt(&bigToeObj,     rawAdc3[2]);
    IOIF_CalFSRVolt(&littleToeObj,  rawAdc3[3]);
    IOIF_CalFSRVolt(&heelObj,       rawAdc3[4]);

    middleToeObj.rawValue = rawAdc3[1];
    bigToeObj.rawValue = rawAdc3[2];
    littleToeObj.rawValue = rawAdc3[3];
    heelObj.rawValue = rawAdc3[4];

    return status;
}
#endif
#endif

#ifdef SUIT_MD_ENABLED
static void Fault_Motor_OTP()
{
	static int16_t OtpSetCnt = 0;

	if (ntcObj.motorTemp >= FAULT_MOTOR_OTP_SET){
		OtpSetCnt++;
		if (OtpSetCnt >= FAULT_MOTOR_OTP_CNT) {
			OtpSetCnt = 0;
			SetFaultInfo(&ext_dev_ctrl_task.fault, TEMPORARY, OVER_TEMPERATURE_MOTOR, TASK_STATE_STANDBY, 0);
			StateTransition(&ext_dev_ctrl_task.stateMachine, TASK_STATE_ERROR);
			StateTransition(&low_level_ctrl_task.stateMachine, TASK_STATE_STANDBY);
		}
	}
	else {
		OtpSetCnt--;
		if (OtpSetCnt < 0) OtpSetCnt = 0;
	}
}

static void Clear_Fault_Motor_OTP()
{
	static int16_t OtpClrCnt = 0;

	if(ntcObj.motorTemp <= FAULT_MOTOR_OTP_CLEAR){
		OtpClrCnt++;
		if(OtpClrCnt >= FAULT_MOTOR_OTP_CNT){
			OtpClrCnt = 0;
			ClearFaultInfo(&ext_dev_ctrl_task.fault, OVER_TEMPERATURE_MOTOR);
		}
	}
	else{
		OtpClrCnt--;
		if (OtpClrCnt < 0) OtpClrCnt = 0;
	}
}

static int EntOtpDeratingCtrl()
{
	OtpDerateCtrl.setTime = 0;
	OtpDerateCtrl.clrTime = 0;
	OtpDerateCtrl.state = NORMAL_STATE;
	OtpDerateCtrl.rampCnt = 0;
	OtpDerateCtrl.DerateTarget = OTP_DE_RATE_NONE;
	OtpDerateCtrl.DerateRamp = OTP_DE_RATE_NONE;
	OtpDerateCtrl.OtplimitRate = (float)OtpDerateCtrl.DerateRamp/OTP_DE_RATE_NONE;
	return 0;
}

static int RunOtpDeratingCtrl()
{
	SetDerateState(&OtpDerateCtrl, &StallDerateCtrl, ntcObj.motorTemp);
	SetDerateCurrLimit(&OtpDerateCtrl);
	return 0;
}

static void SetDerateState(OtpDerate_t* pDerate, RunStall_t* pStallDerate, float motor_temp)
{
	uint16_t usState = 0;

	usState = pDerate->state;

	if(pStallDerate->enable){
		pDerate->DerateTarget = OTP_DE_RATE_LV5;
		pDerate->state = DERATE_LV5_STATE_FOR_STALL;
	}
	else if(motor_temp >= OVER_TEMP_DERATING_START_LV){
		pDerate->setTime++;
		pDerate->clrTime = 0;
		if((pDerate->setTime >= OVER_TEMP_DERATING_SET_TIME_LV1) && (pDerate->setTime < (OVER_TEMP_DERATING_SET_TIME_LV2 - OVER_TEMP_DERATING_SET_TIME_HYS))){
			pDerate->DerateTarget = OTP_DE_RATE_LV1;
			pDerate->state = DERATE_LV1_STATE;
		}
		else if(pDerate->setTime >= OVER_TEMP_DERATING_SET_TIME_LV2 && (pDerate->setTime < (OVER_TEMP_DERATING_SET_TIME_LV3 - OVER_TEMP_DERATING_SET_TIME_HYS))){
			pDerate->DerateTarget = OTP_DE_RATE_LV2;
			pDerate->state = DERATE_LV2_STATE;
		}
		else if(pDerate->setTime >= OVER_TEMP_DERATING_SET_TIME_LV3 && (pDerate->setTime < (OVER_TEMP_DERATING_SET_TIME_LV4 - OVER_TEMP_DERATING_SET_TIME_HYS))){
			pDerate->DerateTarget = OTP_DE_RATE_LV3;
			pDerate->state = DERATE_LV3_STATE;
		}
		else if(pDerate->setTime >= OVER_TEMP_DERATING_SET_TIME_LV4){
			pDerate->setTime = OVER_TEMP_DERATING_SET_TIME_LV4;
			pDerate->DerateTarget = OTP_DE_RATE_LV4;
			pDerate->state = DERATE_LV4_STATE;
		}
	}
	else{
		pDerate->setTime--;
		if(pDerate->setTime < 0) pDerate->setTime = 0;
	}

	if((pDerate->state) && (motor_temp < OVER_TEMP_DERATING_END_LV) && (!pStallDerate->enable)){
		pDerate->clrTime++;
		pDerate->setTime = 0;

		if(pDerate->state < DERATE_LV5_STATE_FOR_STALL){
			if(pDerate->clrTime >= OVER_TEMP_DERATING_CLEAR_TIME){
				pDerate->setTime = 0;
				pDerate->clrTime = 0;
				pDerate->DerateTarget = OTP_DE_RATE_NONE;
				pDerate->state = NORMAL_STATE;
			}
		}
		else if(pDerate->state == DERATE_LV5_STATE_FOR_STALL){
			if(pDerate->clrTime >= STALL_DERATING_CLEAR_TIME){
				pDerate->setTime = 0;
				pDerate->clrTime = 0;
				pDerate->DerateTarget = OTP_DE_RATE_NONE;
				pDerate->state = NORMAL_STATE;
			}
		}
	}

	if(usState != pDerate->state){
		if(pDerate->state == NORMAL_STATE){
			ClearWarningInfo(OTP_DERATING_CONTROL, &WarningStateObj);
			Send_EMCY(&WarningStateObj.packet);
		}
		else{
			SetWarningInfo(OTP_DERATING_CONTROL, &WarningStateObj);
			Send_EMCY(&WarningStateObj.packet);
		}
	}
}

static void SetDerateCurrLimit(OtpDerate_t* pDerate)
{
	if(pDerate->DerateRamp > pDerate->DerateTarget){
		pDerate->DerateRamp -= RAMP_TIME_DIV;
		if(pDerate->DerateRamp <= pDerate->DerateTarget){
			pDerate->DerateRamp = pDerate->DerateTarget;
		}
	}
	else if (pDerate->DerateRamp < pDerate->DerateTarget){
		pDerate->DerateRamp += RAMP_TIME_DIV;
		if(pDerate->DerateRamp >= pDerate->DerateTarget){
			pDerate->DerateRamp = pDerate->DerateTarget;
		}
	}
	else{
		;
	}

	pDerate->OtplimitRate = (float)pDerate->DerateRamp/OTP_DE_RATE_NONE;
}
#endif

static int EntStallDeratingCtrl(void)
{
	SetStallCtrl_Init(&StallDerateCtrl);

	return 0;
}

static int RunStallDeratingCtrl(void)
{
	SetStallCtrl_Run(ntcObj.motorTemp, &StallDetect, &StallDerateCtrl);

	return 0;
}


static void SetStallCtrl_Init(RunStall_t* pStallDerate)
{

	float temp0 = FAULT_MOTOR_OTP_SET * FAULT_MOTOR_OTP_SET;
	pStallDerate->curr_limit = motor_setting.contCurr_limit;
	pStallDerate->max_time_2 = (stall_coeff[2][0]*temp0 * FAULT_MOTOR_OTP_SET) + (stall_coeff[2][1]*temp0) + (stall_coeff[2][2]*FAULT_MOTOR_OTP_SET) + stall_coeff[2][3];
	pStallDerate->max_time_1 = (stall_coeff[1][0]*temp0 * FAULT_MOTOR_OTP_SET) + (stall_coeff[1][1]*temp0) + (stall_coeff[1][2]*FAULT_MOTOR_OTP_SET) + stall_coeff[1][3];
	pStallDerate->max_time_0 = (stall_coeff[0][0]*temp0 * FAULT_MOTOR_OTP_SET) + (stall_coeff[0][1]*temp0) + (stall_coeff[0][2]*FAULT_MOTOR_OTP_SET) + stall_coeff[0][3];
}

static void SetStallCtrl_Run(float motor_temp, SetStall_t* pSetStall, RunStall_t* pStallDerate)
{
	float temp0 =0.0, temp1 = 0.0, max_time = 0.0;
	uint8_t column = 0;

	switch(pStallDerate->step)
	{
		case 0:
			pStallDerate->enable = 0;
			if(pSetStall->activated) {pStallDerate->step = 1;}
			else					{pStallDerate->step = 0;}
			break;

		case 1:

			if(pSetStall->curr_tgt >= STALL_PROTECTION_CURR_2){
				column = STALL_PROTECTION_CURR_2 - STALL_PROTECTION_CURR_0;
				max_time = pStallDerate->max_time_2;
			}
			else if(pSetStall->curr_tgt >= STALL_PROTECTION_CURR_1){
				column = STALL_PROTECTION_CURR_1 - STALL_PROTECTION_CURR_0;
				max_time = pStallDerate->max_time_1;
			}
			else if(pSetStall->curr_tgt >= STALL_PROTECTION_CURR_0){
				column = STALL_PROTECTION_CURR_0 - STALL_PROTECTION_CURR_0;
				max_time = pStallDerate->max_time_0;
			}
			else{
				pStallDerate->step = 0;
				return;
			}

			if(motor_temp > 30) {temp0 = motor_temp;}
			else				{temp0 = 30.0;}

			temp1 = temp0 * temp0;
			pStallDerate->est_time = (((stall_coeff[column][0] * temp1) * temp0)					\
							+ (stall_coeff[column][1] * temp1) + (stall_coeff[column][2] * temp0) 	\
							+ stall_coeff[column][3]);

			if(pStallDerate->est_time < 0) {
				pStallDerate->est_time = 0;
			}
			else if(pStallDerate->est_time > max_time) {
				pStallDerate->est_time = max_time;
			}

			pStallDerate->est_time = max_time - pStallDerate->est_time;
			pStallDerate->est_cnt = pStallDerate->est_time * EXT_DEV_CONTROL_FREQUENCY;
			pStallDerate->half_cnt = pStallDerate->est_cnt>>1;
			if(pSetStall->activated) {pStallDerate->step = 2;}
			else					{pStallDerate->step = 0;}

			break;

		case 2:

			pStallDerate->tmr++;
			if(pStallDerate->tmr >= (pStallDerate->half_cnt)){
				pStallDerate->enable = 1;
				pStallDerate->tmr = pStallDerate->half_cnt;
			}

			if(!pSetStall->activated){
				pStallDerate->enable = 0;
				pStallDerate->tmr = 0;
				pStallDerate->step = 0;
			}

			break;
		default:
			break;
	}
}

#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (L30_MD_REV08_ENABLED)
static int GetFootVal()
{
    uint8_t status = 0;

    status = IOIF_CalFSRVolt(&fsrObj, rawAdc3[1]);
    if (status != IOIF_FSR_STATUS_OK) {
        return status;
    }

    status = IOIF_CalLPVolt(&lpObj, rawAdc3[2]);
    if (status != IOIF_LP_STATUS_OK) {
        return status;
    }

    return status;
}

static int GetUprightLength()
{
	IOIF_StartADCBlock(IOIF_ADC2);
	IOIF_RunADCConvBlock(IOIF_ADC2, POLL_CONV_TIMEOUT);
    IOIF_GetADCVal(IOIF_ADC2, &rawAdc2);
    IOIF_StopADCBlock(IOIF_ADC2);
    uprightObj.lengthAct=IOIF_GetLSmm(&lsObj, rawAdc2);
    return 0;
}

static int RunUprightByLen()
{
    IOIF_SetUprightLen(&uprightObj);
    IOIF_MoveUpright(&uprightObj);
    if(uprightObj.DirCmd == UPRIGHT_MOVE_STOP){
    	ClearRoutines(&ext_dev_ctrl_task.routine);
    	PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_DC_LENGTH_UPDATE);
    	PushRoutine(&ext_dev_ctrl_task.routine, ROUTINE_ID_EXTDEV_DC_BUTTON_CMD);
    }

	return 0;
}

static int RunUprightByBT()
{
    IOIF_SetUprightBT(&uprightObj);
    IOIF_MoveUpright(&uprightObj);

	return 0;
}
#endif

