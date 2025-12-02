/**
 * @file system_ctrl_task.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#include "AS_system_ctrl.h"

#include "ext_link_mngr.h"

extern osSemaphoreId_t BinSem_PlaySoundHandle;
extern osSemaphoreId_t BinSem_PlayBeepHandle;

#ifdef SUIT_MINICM_ENABLED

/**
 *-----------------------------------------------------------
 *       TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */
// Task
TaskObj_t sysCtrlTask;

uint32_t sysCtrlLoopCnt;
uint32_t standbyLoopCnt;
MainSequence_Enum MD_state;

// HW init.
volatile bool BatInit_Res = true;


// CM SW Version;
uint16_t CM_sw_ver = 0;


// Update Battery Level
static uint8_t bmRes = IOIF_BATMONITOR_STATUS_OK;

IOIF_BatMonitor_Data_t batData;
float voltSamples[BAT_VOLT_SAMPLE_SIZE];
int voltSampleIndex = 0;
static uint32_t lastBatLvSamplingTime = 0; // Last Sampling Time.

float medianVolt;
float totalVoltage;
float batPctg;

// Mode Change & Power Button
uint8_t modeBtState;
static uint32_t lastModeBtPressTime = 0;
static uint8_t lastModeBtState = IOIF_GPIO_PIN_SET;

uint8_t safety_sw;

// Assist Level Button, BLE
uint8_t assistIncBtState;
uint8_t assistDecBtState;

AssistStage assistStage = STAGE_0;
AssistStage pre_assistStage;
uint8_t assistFlag = 0;
float assistForcePctg = 0.0;

static uint32_t lastAssistBtPressTime = 0;
static uint32_t lastBothButtonsPressedTime = 0;
static uint8_t lastAssistIncBtState = IOIF_GPIO_PIN_SET;
static uint8_t lastAssistDecBtState = IOIF_GPIO_PIN_SET;

uint8_t currentAssistLevel = 0;
uint8_t currAssistLvfromBt = 0;
uint8_t prevAssistLvfromBt = 0;


uint8_t ledg0_error_flag[2];
uint8_t ledg1_error_flag[2];
uint8_t ledg2_error_flag[2];
uint8_t ledg3_error_flag[2];
uint8_t ledg4_error_flag[2];

uint8_t lastAssistLEDState = -1;	// -1 means not set
uint8_t lastBLELEDState = 0; 		// 0 means not set
uint8_t lastBatLEDState = 0;		// 0 means not set
uint8_t lastModeLEDState = 0; 		// 0 means not set

uint32_t Bat5pctCnt;
uint8_t Bat15pct_flag = 0;
uint8_t Bat5pct_flag = 0;


// BLE Test
uint8_t BTN_test[3] = {0, };
uint8_t LED_test = 0;
uint8_t prev_LED_test = 0;

//RM
static uint32_t mcu_temp = 0;

// Extern 
extern SUIT_AudioState_t SuitAudioState;	// For Audio State Check
extern bool IsOnceBLEconnectedtoApp;
extern bool init_error;

/* AIR_WALKING_MODE_ENABLED */
#ifdef AIR_WALKING_MODE_ENABLED

extern uint8_t Airwalking_pos;
extern uint8_t Airwalking_Step;
extern uint32_t aging_inter_cnt;
extern uint8_t aging_speed_param;

#endif

// For XM
extern ExtLinkMngrInst_t g_ext_link_mngr;
SuitMode_t CMMode = SUIT_STANDBY_MODE;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ----------------------- FUNCTION ----------------------- */
static void StartBoot(void);

// Update Battery Level
static void GetBatVal_Run(void);
static int QsortCompare_Float(const void* firstFloatPtr, const void* secondFloatPtr);
static float CalMedian(float* sampleData, int numElements);
static float GetBatState(void);

// Mode Change & Power Button
static void ModeChange(void);
static void CheckAssistMode(void);
static void SystemOff(void);
static void CheckSafetySW(void);

// Assist Level Button
static void GetAssistStagefromBt(void);
static void CalForcePctg(void);

// Update LED
static void UpdateBLELED(void);
static void UpdateErrorLED(void);
static void UpdateBatLED(void);
static void UpdateModeLED(void);
static void UpdateAuxLED(AssistStage stage);

//Risk Management
static void isBatteryFine(void);

/* ----------------------- ROUTINE ------------------------ */
/* --------------------- SDO CALLBACK --------------------- */
static void Set_MD_state(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);
static void Set_Safety_SW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */
/* --------------------- SDO CALLBACK --------------------- */
DOP_COMMON_SDO_CB(sysCtrlTask)

/* ------------------------- MAIN ------------------------- */
void InitSysCtrl(void)
{
	/* Init Task */
    InitTask(&sysCtrlTask);
	DOPC_AssignTaskID(&sysCtrlTask, TASK_IDX_SYSTEM_CTRL);

	/* Init Device */

	/* State Definition */
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_OFF,      StateOff_Ent,		StateOff_Run,       NULL,				true);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,       		false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,  	false);
	TASK_CREATE_STATE(&sysCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				false);

	DOP_CreateSDO(TASK_ID_SYSMNGT_CM,	SDO_ID_SYSMNGT_CM_FLASH_WRITE_NOTI,	DOP_UINT8,  Set_MD_state);
	DOP_CreateSDO(TASK_ID_MIDLEVEL,		SDO_ID_MIDLEVEL_SAFETY_SW,			DOP_UINT8,  Set_Safety_SW);

	/* Routine Definition */
	// TASK_CREATE_ROUTINE(&sysCtrlTask, ROUTINE_ID_SYSMNGT_GET_POWER_VALUE, 	NULL, 	GetBatVal_Run, 	NULL);

	/* DOD Definition */
	// DOD
	// DOP_CreateDOD(TASK_IDX_SYSTEM_CTRL);

   	// PDO
	// DOP_COMMON_PDO_CREATE(TASK_IDX_SYSTEM_CTRL, sysCtrlTask);
	// DOP_CreatePDO(TASK_IDX_SYSTEM_CTRL, object_id, DOP_FLOAT32, length, pDataAddr);

	// SDO
	// DOP_COMMON_SDO_CREATE(TASK_IDX_SYSTEM_CTRL)
	// DOP_CreateSDO(TASK_IDX_SYSTEM_CTRL, object_id, DOP_FLOAT32, SDOCallback);
}

void RunSysCtrl(void)
{
	RunTask(&sysCtrlTask);
}

/* ----------------------- FUNCTION ----------------------- */

void UpdateLED(void)
{
	UpdateBLELED();
	UpdateErrorLED();
	UpdateBatLED();
	UpdateModeLED();
//	if (BLECtrlObj.data.AssistMode == JOINT_LIMIT_MODE || BLECtrlObj.data.AssistMode == STS_MODE) {
//		assistStage = 10;
//	} else {
	GetAssistStagefromBt();
//	}
	UpdateAuxLED(assistStage);
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
	StartBoot();

	CM_sw_ver = FW_VER_MAJOR * 100 + FW_VER_MINOR * 10 + FW_VER_PATCH * 1;
}

static void StateOff_Run(void)
{
	sysCtrlLoopCnt++;

	uint8_t t_riskMngState = DOPC_GetTaskState(TASK_IDX_RISKMNGT_HDLR);
	if (t_riskMngState == TASK_STATE_STANDBY) {
		StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_STANDBY);
	}
}

static void StateStandby_Run(void)
{
	if (standbyLoopCnt < 100) {
		bmRes = IOIF_BatMonitor_GetValue(&batData);
		totalVoltage = totalVoltage + batData.batVolt;
	} else if (standbyLoopCnt == 100) {
		medianVolt = totalVoltage / 100;
	} else if (standbyLoopCnt == 101) {
		batPctg = GetBatState();
		if (batPctg >= 0 && batPctg <= 5) {
			IOIF_LED24chBattery(BLINK, 100, 1, RED, 0x20);
				if(SuitAudioState.audio_Isplay != true && audio_end == true) {
					if (Bat5pct_flag == 0) {
						audioOutputMode = AUDIO_FILE_PLAY;
						SuitAudioState.audio_id = AUDIO_ID_13;
						osSemaphoreRelease(BinSem_PlaySoundHandle);
						Bat5pct_flag = 1;
					}

					if (Bat5pctCnt > 199) {
						SuitAudioState.audio_id = AUDIO_ID_105;
						osSemaphoreRelease(BinSem_PlayBeepHandle);
						Bat5pctCnt = 0;
					}
					Bat5pctCnt++;
				}
			}
			else if (batPctg > 5 && batPctg <= 15) {
				IOIF_LED24chBattery(ON, LED_NA, 1, RED, 0x20);
				if(SuitAudioState.audio_Isplay != true && audio_end == true) {
					if (Bat15pct_flag == 0) {
						audioOutputMode = AUDIO_FILE_PLAY;
						SuitAudioState.audio_id = AUDIO_ID_17;
						osSemaphoreRelease(BinSem_PlaySoundHandle);
						Bat15pct_flag = 1;
					}
				}
			}
	} else if (standbyLoopCnt > 101) {
		uint8_t t_riskMngState = DOPC_GetTaskState(TASK_IDX_RISKMNGT_HDLR);
		if (t_riskMngState == TASK_STATE_ENABLE) {
			StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_ENABLE);
		}
	}
	standbyLoopCnt++;
}


static void StateEnable_Ent(void)
{
	sysCtrlLoopCnt = 0;

}

static void StateEnable_Run(void)
{

	if (FSMMngrObj.state_curr >= 1 || rm_power_off == true) {
		ModeChange();
		CheckAssistMode();
		SystemOff();
	}

	CalForcePctg();
	GetBatVal_Run();

	batPctg = GetBatState();
//	mcu_temp = IOIF_ReadCPUTemp();

	//RM
	if(BatInit_Res == true)	isBatteryFine();
//	else
//		StateTransition(&sysCtrlTask.stateMachine, TASK_STATE_ERROR);

	if(LED_test == 0 && prev_LED_test != 0)
	{
		IOIF_LED24chBluetooth	(OFF, LED_NA, LED_NA);
		IOIF_LED24chError		(OFF, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chBattery		(OFF, LED_NA, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chMode		(OFF, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chAssist		(OFF, LED_NA, LED_NA, LED_NA);
	}

	if(LED_test == 0)
	{
		UpdateLED();
		CheckSafetySW();
		safety_sw = 0;
	}
	else if (LED_test == 1 && prev_LED_test != 1)
	{
		IOIF_LED24chBluetooth	(OFF, LED_NA, LED_NA);
		IOIF_LED24chError		(ON,  LED_NA, RED, 0x50);
		IOIF_LED24chBattery		(ON,  LED_NA, 3, RED, 0x50);
		IOIF_LED24chMode		(ON,  LED_NA, RED, 0x50);
		IOIF_LED24chAssist		(OFF, LED_NA, LED_NA, LED_NA);
	}
	else if (LED_test == 2 )
	{
		IOIF_LED24chBluetooth	(OFF, LED_NA, LED_NA);
		IOIF_LED24chError		(ON,  LED_NA, GREEN, 0x80);
		IOIF_LED24chBattery		(ON,  LED_NA, 3, GREEN, 0x80);
		IOIF_LED24chMode		(ON,  LED_NA, GREEN, 0x10);
		IOIF_LED24chAssist		(OFF, LED_NA, LED_NA, LED_NA);
	}
	else if (LED_test == 3 )
	{
		IOIF_LED24chBluetooth	(ON,  LED_NA, 0x20);
		IOIF_LED24chError		(OFF, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chBattery		(OFF, LED_NA, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chMode		(ON,  LED_NA, BLUE, 0x10);
		IOIF_LED24chAssist		(OFF, LED_NA, LED_NA, LED_NA);
	}
	else if (LED_test == 4 )
	{
		IOIF_LED24chBluetooth	(OFF, LED_NA, LED_NA);
		IOIF_LED24chError		(OFF, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chBattery		(OFF, LED_NA, LED_NA, LED_NA, LED_NA);
		IOIF_LED24chMode		(ON,  LED_NA, WHITE, 0x10);
		IOIF_LED24chAssist		(ON,  LED_NA, 10, 0x20);
	}
	prev_LED_test = LED_test;

	/* Data Storage For Battery Usage */
	// if (medianVolt > 18) {
	// 	GetBatVal_Run();
	// } else {
	// 	dataSaveFinished = 1;
	// }

	/* LED Error Detect */
//	IOIF_LED24chErrorDetect();

	sysCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{
	// TODO : Error Handle
}

/* ----------------------- FUNCTION ----------------------- */

static void StartBoot(void)
{
	// Pwr ON
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_2, IOIF_GPIO_PIN_SET); // Port : MCU_24V_MOTOR_ON_GPIO_Port, Pin : MCU_24V_MOTOR_ON_Pin
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_3, IOIF_GPIO_PIN_SET); // Port : MC_5V_PWR_EN_GPIO_Port, Pin : MC_5V_PWR_EN_Pin
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, MC_24V_PWR_EN_Pin, IOIF_GPIO_PIN_SET); // Port : MC_24V_PWR_EN_GPIO_Port, Pin : MC_24V_PWR_EN_Pin
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_4, IOIF_GPIO_PIN_SET); // Port : WIDM_5V_PWR_EN_GPIO_Port Pin : WIDM_5V_PWR_EN_Pin
}

static void GetBatVal_Run(void)
{
	// Perform sampling only if the difference between the current time and the last time is greater than or equal to the sampling interval.
	if (sysCtrlLoopCnt - lastBatLvSamplingTime >= BAT_VOLT_SAMPLING_INTERVAL) { // Store sample data at intervals of every 60ms.
		bmRes = IOIF_BatMonitor_GetValue(&batData); // Read battery data

		// Store the battery voltage sample in the array.
		if (voltSampleIndex < BAT_VOLT_SAMPLE_SIZE) {
			voltSamples[voltSampleIndex++] = batData.batVolt;
		}

 		// Once the sample array is full, calculate the median voltage.
		if (voltSampleIndex == BAT_VOLT_SAMPLE_SIZE) {
			medianVolt = CalMedian(voltSamples, BAT_VOLT_SAMPLE_SIZE);
			voltSampleIndex = 0; // Reset sample index for the next round of sampling.
			oneTimeSave = 1;
		}




		// Update to the last sampling time
		lastBatLvSamplingTime = sysCtrlLoopCnt;
	}
}

/**
 * @brief Compare two floating-point numbers for qsort.
 * 
 * This function is utilized by the qsort function to compare two elements. It takes
 * two const void pointers to floating-point numbers, calculates the difference 
 * between them, and returns an integer representing their relative ordering.
 * 
 * @param firstFloatPtr Pointer to the first floating-point number.
 * @param secondFloatPtr Pointer to the second floating-point number.
 * @return int Returns positive if the first number is greater than the second, 
 * negative if the first is less than the second, and 0 if they are equal.
 */
static int QsortCompare_Float(const void* firstFloatPtr, const void* secondFloatPtr)
{
	// Convert void pointers to float pointers, then dereference to get the float values and calculate the difference
	float diff = *(float*)firstFloatPtr - *(float*)secondFloatPtr;

	// If diff is positive, return 1; if diff is negative, return -1; if equal, return 0.
	// This determines the order for the qsort function to sort the array in ascending order.
	return (diff > 0) - (diff < 0);
}

/**
 * @brief Calculate the median of a sorted array of floating-point numbers.
 * 
 * This function calculates the median value from a given array of floating-point 
 * numbers (data) by first sorting the array using qsort and then finding the 
 * middle value or the average of the two middle values depending on the total 
 * number of elements in the array.
 * 
 * @param sampleData Array of floating-point numbers to find the median of.
 * @param numElements The number of elements in the array.
 * @return float The median value of the array.
 */
static float CalMedian(float* sampleData, int numSampleData)
{
	// Use qsort to sort the samples array in ascending order.
	// The QsortCompare function is passsed as the comparator.
	qsort(sampleData, numSampleData, sizeof(float), QsortCompare_Float);

	// If the number of elements(n) is even, the median is the average of the two median numbers.
	if (numSampleData % 2 == 0) {
		return (sampleData[numSampleData/2 - 1] + sampleData[numSampleData/2]) / 2.0;
	} else { // If n is odd, the median is the middle number directly.
		return sampleData[numSampleData/2];
	}
}
float offset_volt_f;
float offset_volt_t;
static float GetBatState(void)
{
	static uint8_t filter_init;
	static float offset_volt;
	// Calculate the percentage of the battery voltage within the defined range
	// 6차 다항식 근사
	if(filter_init == 0 ){
		offset_volt_f = medianVolt - BATTERY_VOLT_MIN;
		offset_volt_t = medianVolt - BATTERY_VOLT_MIN;
		filter_init = 1;
	}


	offset_volt = medianVolt - BATTERY_VOLT_MIN;
	offset_volt_t = 0.99995 * offset_volt_f + 0.00005 * offset_volt;

	if(offset_volt_t<offset_volt_f){
		offset_volt_f = offset_volt_t;
	}

	float per = (((((0.000038 * offset_volt_f - 0.000881) * offset_volt_f + 0.006638) * offset_volt_f - 0.014935) * offset_volt_f + 0.002198) * offset_volt_f + 0.031441) * offset_volt_f - 0.002839;


	// 선형
//	float per = (medianVolt - BATTERY_VOLT_MIN) / (BATTERY_VOLT_MAX - BATTERY_VOLT_MIN);

    // Clamp the result to the range [0, 100]
    per = fmax(0.0, fmin(per, 1.0)) * 100;

	return per;
}

// static void ModeChange(void)
// {
// 	// uint8_t modeBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_6); // PB_SW_INT# Rev0.3
// 	uint8_t modeBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_8); // PB_SW_INT# Rev0.4
// 	uint32_t currentTime = IOIF_GetTick();

// 	if (modeBtState == IOIF_GPIO_PIN_RESET && lastModeBtState == 1 && (currentTime - lastModeBtPressTime > DEBOUNCE_TIME)) {
// 		if (SuitAudioState.audio_Isplay != true) {
// 			BFlag[18] = 1;
//         }
// 		BTN_test[0] = 1;
// 		lastModeBtPressTime = currentTime;
// 	}

// 	// 버튼의 이전 상태를 업데이트
// 	lastModeBtState = modeBtState;
// }

static void ModeChange(void)
{
	// modeBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_6); // PB_SW_INT# Rev0.3
	modeBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_8); // PB_SW_INT# Rev0.4
	uint8_t assistIncBt_NotUsed = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10);	// ASSIST_BTN_P
	uint8_t assistDecBt_NotUsed = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12); // ASSIST_BTN_N
	static uint32_t currentTime_ModeBt;
	static uint32_t lastClickTime_ModeBt = 0; 	// 마지막 클릭 시간을 추적
	static uint8_t clickCount_ModeBt = 0; 		// 클릭 수를 추적
	static uint32_t TimeLabs_ModeBt = 0;

	if (assistIncBt_NotUsed == IOIF_GPIO_PIN_RESET || assistDecBt_NotUsed == IOIF_GPIO_PIN_RESET) {
		// 다른 버튼이 눌린 상태에서는 모드 변경 동작을 하지 않음
		lastModeBtState = modeBtState;
		return;
	}
	if (modeBtState == IOIF_GPIO_PIN_RESET) {

		BTN_test[0] = 1;
	}
	if(IsOnceBLEconnectedtoApp == 0 || (FSMMngrObj.state_curr != 1 && IsBLEconnectedtoApp == 0)){
		if (modeBtState == IOIF_GPIO_PIN_RESET){
			currentTime_ModeBt = IOIF_GetTick(); // 현재 시간을 버튼이 눌렸을 때만 가져옴
			BTN_test[0] = 1;
			if (lastModeBtState != IOIF_GPIO_PIN_RESET && (currentTime_ModeBt - lastClickTime_ModeBt > DEBOUNCE_TIME)) {
				if (clickCount_ModeBt == 0) {
					lastClickTime_ModeBt = currentTime_ModeBt;
					clickCount_ModeBt++;
				} else {
					TimeLabs_ModeBt = currentTime_ModeBt - lastClickTime_ModeBt;
					if (TimeLabs_ModeBt >= DOUBLE_CLICK_TIME_MIN && TimeLabs_ModeBt <= DOUBLE_CLICK_TIME_MAX) {
						if (!SuitAudioState.audio_Isplay) {
#ifdef AIR_WALKING_MODE_ENABLED
							/* Aging Mode Change */
							if(Airwalking_Step > AIRWALKING_MODE_STBY)
							{
								Airwalking_Step++;

								if(Airwalking_Step == AIRWALKING_MODE_RUN_LOW)		aging_speed_param = 3;
								if(Airwalking_Step == AIRWALKING_MODE_RUN_MID)		aging_speed_param = 2;
								if(Airwalking_Step == AIRWALKING_MODE_RUN_HIGH)		aging_speed_param = 1;

								if(Airwalking_Step > AIRWALKING_MODE_RUN_HIGH)	// Enter to Neutral position, Standby
								{
									aging_speed_param = 3;
									Airwalking_Step = AIRWALKING_MODE_STBY;
									Airwalking_pos = AIRWALKING_OFF;
									aging_inter_cnt = 0;
								}
							}
							else		// Air-walking Start
							{
								aging_speed_param = 3;
								Airwalking_pos = AIRWALKING_NEUTRAL;
								Airwalking_Step = AIRWALKING_MODE_RUN_LOW;
								aging_inter_cnt = 0;
							}
#else
							BFlag[18] = 1; // 더블 클릭으로 모드 변환
							// <<< CMMode 변경 로직 추가
							CMMode = (CMMode + 1) % 2;
							if (g_ext_link_mngr.is_ext_pack_connected) {
								SendSUITMode((uint8_t)CMMode);
							}
#endif
						}
					}
					clickCount_ModeBt = 0; // 클릭 수 초기화
					lastClickTime_ModeBt = 0; // 마지막 클릭 시간 초기화
				}
			}
		}
	}

	// 클릭 시간 간격이 최대 더블 클릭 시간을 초과하면 클릭 수를 초기화
	if (clickCount_ModeBt == 1 && (IOIF_GetTick() - lastClickTime_ModeBt > DOUBLE_CLICK_TIME_MAX)) {
		clickCount_ModeBt = 0;
		lastClickTime_ModeBt = 0; // 마지막 클릭 시간을 초기화
	}

	// 버튼의 이전 상태를 업데이트
	lastModeBtState = modeBtState;
}

static void CheckAssistMode(void)
{
	if (FSMMngrObj.state_curr == 2){
		if(BLECtrlObj.data.AssistMode == SMART_ASSIST_MODE) {
			BFlag[56] = 1;
		} else if(BLECtrlObj.data.AssistMode == AQUA_MODE) {
			BFlag[57] = 1;
		} else if(BLECtrlObj.data.AssistMode == UNIVERSE_MODE) {
			BFlag[58] = 1;
		} else if(BLECtrlObj.data.AssistMode == JOINT_LIMIT_MODE) {
			BFlag[59] = 1;
		}
	}
}

static void CheckSafetySW(void)
{
	if(((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
		(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
		(FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75)) &&
		safety_sw == 1) {
			BFlag[54] = 1;
	}
}

static void SystemOff(void)
{
	static uint32_t power_cnt = 0;
	if((MD_state != SAVE_PROPERTIES && MD_state != SAVE_PROPERTIES_NeutralPosture) && power_cnt > 50){
		if(init_error == true && IOIF_SYS_IsPwrBtnPressed() == true) {
			IOIF_SYS_PwrOff();
		} else if (IOIF_SYS_IsPwrBtnPressed() == true) {
			BFlag[35] = 1;
		} else if (powerOffCmdByBLE == SUIT_PWR_OFF) {
			IOIF_SYS_PwrOff();
		} else if (rm_power_off == true && audioOutputMode == AUDIO_INVALID) {
			IOIF_SYS_PwrOff();
		}
	}
	power_cnt++;
}
	
static void GetAssistStagefromBt(void)
{
	assistIncBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_10);	// ASSIST_BTN_P
	assistDecBtState = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_12); 	// ASSIST_BTN_N
	uint32_t currentTime_assistBt = HAL_GetTick();
	if(IsOnceBLEconnectedtoApp == 0 || manu_mode){
		// 두 버튼이 동시에 눌렸을 경우 처리
		if (assistIncBtState == IOIF_GPIO_PIN_RESET && assistDecBtState == IOIF_GPIO_PIN_RESET) {
			if (lastBothButtonsPressedTime == 0) { // 두 버튼이 동시에 처음 눌린 시간 초기화
				lastBothButtonsPressedTime = currentTime_assistBt;
				BTN_test[1]=1;
				BTN_test[2]=1;
			} else if ((currentTime_assistBt - lastBothButtonsPressedTime) >= 3000 && // 두 버튼이 3초 이상 눌려있는지 확인
			(FSMMngrObj.state_curr == 1 || FSMMngrObj.state_curr == 18 || FSMMngrObj.state_curr == 19 || FSMMngrObj.state_curr == 20)) {
				neutralPosCalCMD = 1;
			}
			// 동시에 눌린 경우 개별 버튼 로직은 실행하지 않음
			return;
		} else {
			lastBothButtonsPressedTime = 0; // 버튼이 떼어지면 시간 초기화
		}

		// Assist Force Increment 버튼이 눌렸고, 이전 상태는 떼어져 있었던 경우
		if (assistIncBtState == IOIF_GPIO_PIN_RESET && lastAssistIncBtState == 1 && (currentTime_assistBt - lastAssistBtPressTime > DEBOUNCE_TIME)) {
			if (assistStage < STAGE_10) {
				pre_assistStage = assistStage; // detect assistForcePctg changing
				assistStage++;
				currAssistLvfromBt = assistStage;
				assistFlag = 1;
			}
			BTN_test[1]=1;
			UpdateAuxLED(assistStage);
			if (g_ext_link_mngr.is_ext_pack_connected) { // For XM
				SendSUITAssistLevel(assistStage);
			}
			// 이전 값과 현재 값을 비교하여 변경 사항이 있는 경우 플래그 설정
			if (currAssistLvfromBt != prevAssistLvfromBt) {
				prevAssistLvfromApp = currAssistLvfromBt;	// 이전 앱 보조력 단계에 현재 보조력 단계 적용
				BLECtrlObj.data.AssistanceLevel = currAssistLvfromBt;
			}

			lastAssistBtPressTime = currentTime_assistBt;
		} else if (assistDecBtState == IOIF_GPIO_PIN_RESET && lastAssistDecBtState == 1 && (currentTime_assistBt - lastAssistBtPressTime > DEBOUNCE_TIME)) {
			if (assistStage > STAGE_0) {
				pre_assistStage = assistStage; // detect assistForcePctg changing
				assistStage--;
				currAssistLvfromBt = assistStage;
				assistFlag = 2;
			}
			BTN_test[2]=1;
			UpdateAuxLED(assistStage);
			if (g_ext_link_mngr.is_ext_pack_connected) { // For XM
				SendSUITAssistLevel(assistStage);
			}
			// 이전 값과 현재 값을 비교하여 변경 사항이 있는 경우 플래그 설정
			if (currAssistLvfromBt != prevAssistLvfromBt) {
				prevAssistLvfromApp = currAssistLvfromBt;	// 이전 앱 보조력 단계에 현재 보조력 단계 적용
				BLECtrlObj.data.AssistanceLevel = currAssistLvfromBt;
			}
			lastAssistBtPressTime = currentTime_assistBt;
		}
	}
	// 버튼의 이전 상태를 업데이트
	lastAssistIncBtState = assistIncBtState;
	lastAssistDecBtState = assistDecBtState;
	prevAssistLvfromBt = currAssistLvfromBt;
}

static void CalForcePctg(void) 
{
		switch (assistStage) {
		case(STAGE_0) :
						assistForcePctg = 0.0;
		break;
		case(STAGE_1) :
						assistForcePctg = 0.1;
		break;
		case(STAGE_2) :
						assistForcePctg = 0.2;
		break;
		case(STAGE_3) :
						assistForcePctg = 0.3;
		break;
		case(STAGE_4) :
						assistForcePctg = 0.4;
		break;
		case(STAGE_5) :
						assistForcePctg = 0.5;
		break;
		case(STAGE_6) :
						assistForcePctg = 0.6;
		break;
		case(STAGE_7) :
						assistForcePctg = 0.7;
		break;
		case(STAGE_8) :
						assistForcePctg = 0.8;
		break;
		case(STAGE_9) :
						assistForcePctg = 0.9;
		break;
		case(STAGE_10) :
						assistForcePctg = 1.0;
		break;
		default :
			break;
		}
}


static void UpdateBLELED(void)
{
		if (isBLEConnect.current_state == IOIF_ESP32_BT_CONNECTED ) 	IOIF_LED24chBluetooth(ON, LED_NA, 0x50);
	else if (isBLEConnect.current_state == IOIF_ESP32_BT_ADVERTISING) 	IOIF_LED24chBluetooth(BLINK, 50, 0x50);
	else if (isBLEConnect.current_state == IOIF_ESP32_BT_IDLE) 		 	IOIF_LED24chBluetooth(OFF, LED_NA, LED_NA);
}

static void UpdateErrorLED(void)
{
	if (RM_error_led_ctrl == ON)
		IOIF_LED24chError(ON, LED_NA, RED, 0x50);
	else if (RM_error_led_ctrl == BLINK)
		IOIF_LED24chError(BLINK, 100, RED, 0x50);
	else if (FSMMngrObj.state_curr >= 18 && FSMMngrObj.state_curr <= 26)
		IOIF_LED24chError(BLINK, 100, RED, 0x20);
	else
		IOIF_LED24chError(OFF, LED_NA, LED_NA, LED_NA);
}

static void UpdateBatLED(void)
{
	if (batPctg >= 0 && batPctg <= 5) {
		IOIF_LED24chBattery(BLINK, 100, 1, RED, 0x60);	// 변경된 LED 의 최소 출력 : 0x60
		if(SuitAudioState.audio_Isplay != true && audio_end == true) {
			if (Bat5pct_flag == 0) {
				audioOutputMode = AUDIO_FILE_PLAY;
				SuitAudioState.audio_id = AUDIO_ID_13;
				osSemaphoreRelease(BinSem_PlaySoundHandle);
				Bat5pct_flag = 1;
			}

			if (Bat5pctCnt > 499) {		// 5sec 마다 알림
				SuitAudioState.audio_id = AUDIO_ID_105;
				osSemaphoreRelease(BinSem_PlayBeepHandle);
				Bat5pctCnt = 0;
			}
			Bat5pctCnt++;
		}
	}
	else if (batPctg > 5 && batPctg <= 15) {
		IOIF_LED24chBattery(ON, LED_NA, 1, RED, 0x60); // 변경된 LED 의 최소 출력 : 0x60
		if(SuitAudioState.audio_Isplay != true && audio_end == true) {
			if (Bat15pct_flag == 0) {
				audioOutputMode = AUDIO_FILE_PLAY;
				SuitAudioState.audio_id = AUDIO_ID_17;
				osSemaphoreRelease(BinSem_PlaySoundHandle);
				Bat15pct_flag = 1;
			}
		}
	}
	else if (batPctg > 15 && batPctg <= 30)  IOIF_LED24chBattery(ON, 	LED_NA, 1, GREEN, 0x80);
	else if (batPctg > 30 && batPctg <= 60)	 IOIF_LED24chBattery(ON, 	LED_NA, 2, GREEN, 0x80);
 	else if (batPctg > 60 && batPctg <= 100) IOIF_LED24chBattery(ON, 	LED_NA, 3, GREEN, 0x80);
}

static void UpdateModeLED(void)
{
	if (FSMMngrObj.state_curr == 0) 		IOIF_LED24chMode(OFF, LED_NA, LED_NA, LED_NA);
	else if (FSMMngrObj.state_curr == 1 || FSMMngrObj.state_curr == 2 ||
			(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 49))	\
											IOIF_LED24chMode(ON,  LED_NA, BLUE, 0x50);		// 대기 모드
	else if ((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
			(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45) ||
			(FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75))	\
											IOIF_LED24chMode(ON,  LED_NA, WHITE, 0x50);		// 보조 모드
}

static void UpdateAuxLED(AssistStage stage)
{
	if (FSMMngrObj.state_curr == 1 || FSMMngrObj.state_curr == 2 ||
			(FSMMngrObj.state_curr >= 46 && FSMMngrObj.state_curr <= 49)){
		if(neutralPosCalCMD != 1)
			IOIF_LED24chAssist(BLINK,  100, stage, 0x20);		// 대기 모드
	}
	else if (((FSMMngrObj.state_curr >= 3 && FSMMngrObj.state_curr <= 17) ||
			(FSMMngrObj.state_curr >= 33 && FSMMngrObj.state_curr <= 45)) &&
			(BLECtrlObj.data.AssistMode != JOINT_LIMIT_MODE))		\
											IOIF_LED24chAssist(ON,  LED_NA, stage, 0x20);		// 보조 모드
	else if ((BLECtrlObj.data.AssistMode == JOINT_LIMIT_MODE) || (FSMMngrObj.state_curr >= 65 && FSMMngrObj.state_curr <= 75))	\
											IOIF_LED24chAssist(OFF,  LED_NA, LED_NA, LED_NA);
}

static void isBatteryFine(void)
{

	static float f_batVolt, f_batCurr, f_brdTemp, f_mcuTemp  = 0;
	static bool brd_warning = false;
	static bool mcu_warning = false;

	f_batVolt = 0.9 * f_batVolt + 0.1 * batData.batVolt;
	f_batCurr = 0.9 * f_batCurr + 0.1 * batData.batCurr;
	f_brdTemp = 0.9 * f_brdTemp + 0.1 * batData.brdTemp;
	f_mcuTemp = 0.9 * f_mcuTemp + 0.1 * (float)mcu_temp;

	if (f_batVolt > RM_CM_VOLT_TRESHOLD)
	{
		FaultArrange_t ID = {1,0};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
	}

	if (f_batCurr > RM_CM_CURR_TRESHOLD)
	{
		FaultArrange_t ID = {1,1};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));
	}

	if (f_brdTemp > RM_CM_TEMP_TRESHOLD)
	{
		FaultArrange_t ID = {1,3};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));

	} else if (f_brdTemp > RM_CM_TEMP_WARNING && !brd_warning)
	{
		brd_warning = true;
		FaultArrange_t ID = {1,2};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_HIGH, FaultIDRegister, &ID, sizeof(ID));
	}

	if (f_mcuTemp > RM_CM_MCU_TEMP_TRESHOLD)
	{
		FaultArrange_t ID = {1,5};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_REALTIME, FaultIDRegister, &ID, sizeof(ID));

	} else if (f_mcuTemp > RM_CM_MCU_TEMP_WARNING && !mcu_warning)
	{
		mcu_warning = true;
		FaultArrange_t ID = {1,4};
		FaultTaskCreate(ID.category, ID.number, FAULT_PRIORITY_HIGH, FaultIDRegister, &ID, sizeof(ID));
	} else if (f_mcuTemp < RM_CM_MCU_TEMP_WARNING)
	{
		mcu_warning = false;
	}

}


/* ----------------------- ROUTINE ------------------------ */
/* --------------------- SDO CALLBACK --------------------- */

static void Set_MD_state(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&MD_state, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

static void Set_Safety_SW(DOP_SDOArgs_t* t_req, DOP_SDOArgs_t* t_res)
{
	memcpy(&safety_sw, t_req->data, 1);

	t_res->dataSize = 0;
	t_res->status = DOP_SDO_SUCC;
}

#endif /* SUIT_MINICM_ENABLED */
