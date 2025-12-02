/*
 * AS_RiskMngt_Hdlr.c
 *
 *  Created on: Aug 30, 2024
 *      Author: Angelrobotics
 */

#include "AS_RiskMngt_Hdlr.h"

#ifdef SUIT_MINICM_ENABLED

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

TaskObj_t RiskMngtTask;
uint32_t RiskMngtloopcnt;

FaultArrange_t fault_ID[FAULT_QUEUE_SIZE_MAX];
uint8_t fault_ID_index = 0;
IOIF_LED24chCtrl_t RM_error_led_ctrl = OFF;

bool audio_check = false;
bool MD_recovery_flag = false;
bool rm_power_off = false;
bool isFaultOccurred = false;
bool init_error = false;

bool init_fault[INIT_FAULT_COUNT] = {false};
/* SYSTEM LOG */
rtcTime_t CM_RTCC_Time;
extern 	rtcTime_t RTC_Time;
extern 	bool sdcard_ready_to_write;
uint8_t data_log[LOG_SIZE] __attribute__((section(".FATFS_RAMD1_data"))) = {0,};
uint8_t rtc_save_cnt = 0;
bool 	manual_set_rtcc = false;
bool 	log_test_on = false;

uint8_t error_log_buff[20] = {0,};



/* extern */
extern SUIT_AudioState_t SuitAudioState;
extern osSemaphoreId_t BinSem_PlaySoundHandle;
extern osSemaphoreId_t BinSem_PlayBeepHandle;
extern audioType_t	audioOutputMode;
extern uint8_t errorMSGcode;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */



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
static void StateError_Ent(void);
static void StateError_Run(void);
static void StateError_Ext(void);

/* ----------------------- FUNCTION ----------------------- */
static void HandleInitFault(void);
static FaultType Pair_FaultID_ISI(void);
static bool RunSenarioForRMISI(void);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------------- MAIN ------------------------- */
void InitRiskMngtHdlr(void)
{
	/* Init Task */
	InitTask(&RiskMngtTask);
	DOPC_AssignTaskID(&RiskMngtTask, TASK_IDX_RISKMNGT_HDLR);

	/* State Definition */
	TASK_CREATE_STATE(&RiskMngtTask, TASK_STATE_OFF,     NULL,     		   StateOff_Run,	NULL,               true);
	TASK_CREATE_STATE(&RiskMngtTask, TASK_STATE_STANDBY, NULL, 			   StateStandby_Run,NULL,               false);
	TASK_CREATE_STATE(&RiskMngtTask, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, StateEnable_Ext,    false);
	TASK_CREATE_STATE(&RiskMngtTask, TASK_STATE_ERROR,   StateError_Ent,   StateError_Run,  StateError_Ext,     false);

	/* Fault Task Queue Initialization */
	FaultTaskQueueInit();

#ifdef USB_LOG_ENABLED
	/* USB log init. */
	serial_log_init();
#endif

	/* log function Init. */
	log_init();
	memset(data_log, 0, LOG_SIZE);
}

void RunRiskMngtHdlr(void)
{
	RunTask(&RiskMngtTask);
}


/* ----------------------- FUNCTION ----------------------- */
void FaultIDRegister(void* param)
{
	FaultArrange_t* faultParam = (FaultArrange_t*)param;

	if(fault_ID_index < 64)
	{
		fault_ID[fault_ID_index].category = faultParam->category;
		fault_ID[fault_ID_index].number = faultParam->number;

		InitFaultSave(error_log_buff, fault_ID[fault_ID_index].category, fault_ID[fault_ID_index].number);

		fault_ID_index++;

		build_log_string(data_log, error_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		isFaultOccurred = true;
	} else
	{
		//Todo: queue 관리랑 연동,,,,
		fault_ID_index = 0;
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Run(void)
{
	FaultTaskRun(); //for init fault detect

	if (init_fault[0])
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_ERROR);
	else
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_STANDBY);

}

static void StateStandby_Run(void)
{
	uint8_t t_wholeBodyCtrlState = DOPC_GetTaskState(TASK_IDX_WHOLE_BODY_CTRL);

	if (t_wholeBodyCtrlState == TASK_STATE_STANDBY)
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	RiskMngtloopcnt = 0;
	log_test_on = true;
}


static void StateEnable_Run(void)
{

#ifdef USB_LOG_ENABLED
	/* log display via USB */
	print_log_to_USB();
#endif

	FaultTaskRun();

	if (FSMMngrObj.state_curr == 61 || FSMMngrObj.state_curr == 62 ||FSMMngrObj.state_curr == 63 )
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_ERROR);

	if (isFaultOccurred == true)
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_ERROR);

	/* Manual Set-up for RTCC *////
	if(manual_set_rtcc == true)
	{
		RTC_Time.year 	= 25;		//2025년
		RTC_Time.mon 	= 7;		//월
		RTC_Time.mday 	= 18;		//일
		RTC_Time.hour 	= 10;		//시 (24시간)
		RTC_Time.min 	= 51;		//분
		RTC_Time.sec 	= 00;		//00초

		IOIF_MCP79510_SetTime(&RTC_Time);

		manual_set_rtcc = false;
	}

	/* Log Save */
	if(rtc_save_cnt > 9)		// save period -> 10ms(RM task period) * 10 = 100ms
	{
		uint8_t data_log_buff[90] = {0,};			// 90byte payload
		/* 1. get real time */
		IOIF_MCP79510_GetTime(&RTC_Time);
		/* 2. Append System log Data */
		set_syslog_data(data_log_buff);
		/* 3. log build */
//		snprintf(data_log, sizeof(data_log), "Time: %02d:%02d:%02daabbccddeeffhhiijjkk\r\n", RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		/* 4. push to log buffer */
		if(sdcard_ready_to_write == true)		//sdcard 에 쓸 준비가 되면 그 때부터 logging 시작
		{
//			log_push_to_buffer(data_log);
			log_push_to_buffer(data_log, LOG_SIZE);
		}

		rtc_save_cnt = 0;
	}

	rtc_save_cnt++;
	RiskMngtloopcnt++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Ent(void)
{
	IOIF_LED24chAll(OFF, 0x00);
	audio_check = false;
}

static void StateError_Run(void)
{	
	Pair_FaultID_ISI();

	if (init_fault[0] == true) {
		HandleInitFault();
	}

	if (RunSenarioForRMISI() == true) {
		isFaultOccurred = false;
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_ENABLE);
	}

	if (IOIF_SYS_IsPwrBtnPressed() == true)
		IOIF_SYS_PwrOff();

}

static void StateError_Ext(void)
{
	UpdateLED();
//	errorMSGcode=0;
}


/* ----------------------- FUNCTION ----------------------- */

static void HandleInitFault(void)
{
	static bool first_run = true;
	static bool fatal_occur = false;

	//Todo : error 개수 11개 이상되면 indicating 알고리즘 수정 필요, 10개 단위로 error 5초간 보여주기
	if (first_run)
	{
		IOIF_LED24chError(ON, LED_NA, RED, 0x50);

		/* Fault indicate */
		for (int i = 1; i < INIT_FAULT_COUNT; i++ )
		{
			if (init_fault[i])
			{
				int led_index = (i % 10 == 0) ? 10 : (i % 10);
				IOIF_LEDforAssistOnebyOne(led_index, 0x50);
				if (i < INIT_FAULT_RTC)
					fatal_occur = true;
			}
		}
		first_run = false;
	}

	if (fatal_occur)
	{
		IOIF_SAI_PlayBeep(1000, 30, 200);
		IOIF_SAI_PlayBeep(1000, 0, 800);
	}
	else
	{
		IOIF_LED24chError(ON, LED_NA, RED, 0x50);
		/*Warning*/
		for (int i = INIT_FAULT_RTC; i < INIT_FAULT_COUNT; i++ )
		{
			if (init_fault[i])
			{
				//Todo : logging?
			}
		}

		for(int i = 4; i > 1; i--){
			IOIF_SAI_PlayBeep(1000, i * 13, 200);
			IOIF_SAI_PlayBeep(1000, 0, 800);
		}
		StateTransition(&RiskMngtTask.stateMachine, TASK_STATE_STANDBY);
	}

}

static FaultType Pair_FaultID_ISI(void)
{
	/*Risk Mng*/
//	uint8_t no_fault_cnt = 0;
	FaultType fault_type = FaultType_Default;

	for (int i = 0; i < FAULT_QUEUE_SIZE_MAX; i++) {

		/* CATEGORY : 1 */
		if (fault_ID[i].category == 1)
		{
			fault_type =  FaultType_Fatal;

			if (fault_ID[i].number == 0 || fault_ID[i].number == 1)
				rm_flag[RM1] = true;
			else if (fault_ID[i].number == 2 || fault_ID[i].number == 4)
				rm_flag[RM2] = true;
			else if (fault_ID[i].number == 3 || fault_ID[i].number == 5)
				rm_flag[RM3] = true;
		}

		/* CATEGORY : 2 (RH) */
		else if (fault_ID[i].category == 2)
		{
			fault_type =  FaultType_Warning;

			if (fault_ID[i].number == 6)
				rm_flag[RM5] = true;
			else if (fault_ID[i].number == 7)
				rm_flag[RM3] = true;
			else if (fault_ID[i].number == 8)
				rm_flag[RM2] = true;
			else
				rm_flag[RM4] = true;
		}

		/* CATEGORY : 3 (LH) */
		else if (fault_ID[i].category == 3)
		{
			fault_type =  FaultType_Warning;

			if (fault_ID[i].number == 6)
				rm_flag[RM5] = true;
			else if (fault_ID[i].number == 7)
				rm_flag[RM3] = true;
			else if (fault_ID[i].number == 8)
				rm_flag[RM2] = true;
			else
				rm_flag[RM4] = true;
		}

		/* CATEGORY : 6 (SD card) */
		else if (fault_ID[i].category == 6)
		{
			fault_type =  FaultType_Warning;

			if (fault_ID[i].number == 0)
				rm_flag[RM6] = true;
			else if (fault_ID[i].number == 1)
				rm_flag[RM7] = true;
		}
		


		if (fault_type ==  FaultType_Default)
		{
			rm_flag[RM0] = true;
		}

//		/* FAULT ZERO*/
//		else if(fault_ID[i].number == 0 && no_fault == true) //최적화 : flag 버퍼 전체 확인 방지
//		{
//			no_fault_cnt ++;
//			if (no_fault_cnt == 2) break;
//
//		}
	}

	memset(&fault_ID, 0, sizeof(fault_ID));

	return fault_type;
}

static bool RunSenarioForRMISI(void)
{

	uint8_t data_log_buff[90] = {0,};
	if (rm_flag[RM1] == true)
	{
		RM_error_led_ctrl = ON;

		errorMSGcode = 3;	// msg code
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_26);
		}

		if(SuitAudioState.latest_played_audio_id == AUDIO_ID_26) {

			for(int i = 6; i > 1; i--){
			IOIF_SAI_PlayBeep(1000, i * 13, 200);
			IOIF_SAI_PlayBeep(1000, 0, 800);
			}
			rm_power_off = true;
			audio_check = true;
			SuitAudioState.latest_played_audio_id = AUDIO_ID_INVALID;
		}

//		if(SuitAudioState.latest_played_audio_id == AUDIO_ID_26) rm_power_off = true;

	} else if (rm_flag[RM2] == true)
	{
		errorMSGcode = 5;	// msg code
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = BLINK;

		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_27);
		}

		audio_check = true;

	} else if (rm_flag[RM3] == true)
	{
		errorMSGcode = 4;
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = ON;
		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_28);
		}

		if(SuitAudioState.latest_played_audio_id == AUDIO_ID_28) {
			for(int i = 6; i > 1; i--){
			IOIF_SAI_PlayBeep(1000, i * 13, 200);
			IOIF_SAI_PlayBeep(1000, 0, 800);
			}

			rm_power_off = true;
			audio_check = true;
			SuitAudioState.latest_played_audio_id = AUDIO_ID_INVALID;
		}

	} else if (rm_flag[RM4] == 1)
	{

		errorMSGcode = 6;
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = ON;
		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_29);
		}

		if(SuitAudioState.latest_played_audio_id == AUDIO_ID_29) {
			for(int i = 6; i > 1; i--){
				IOIF_SAI_PlayBeep(1000, i * 13, 200);
				IOIF_SAI_PlayBeep(1000, 0, 800);
			}

			//osSemaphoreRelease(BinSem_PlayBeepHandle);

			rm_power_off = true;
			audio_check = true;
			SuitAudioState.latest_played_audio_id = AUDIO_ID_INVALID;
		}


	} else if (rm_flag[RM5] == true)
	{
		errorMSGcode = 7;
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = ON;
		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_30);
		}

		audio_check = true;

	} else if (rm_flag[RM6] == true)
	{
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = BLINK;

		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_31);
		}

		audio_check = true;
	} else if (rm_flag[RM7] == true)
	{
		set_syslog_data(data_log_buff);
		build_log_string(data_log, data_log_buff, RTC_Time.hour, RTC_Time.min, RTC_Time.sec);
		log_push_to_buffer(data_log, LOG_SIZE);

		RM_error_led_ctrl = BLINK;

		if(SuitAudioState.audio_Isplay != true)
		{
			Run_Audio_Action(AUDIO_ID_32);
		}

		audio_check = true;
	}

	if (audio_check)
	{
	 	for (int i = 0; i < RM_ID_NUM; i++) {
	 		rm_flag[i] = false;
	 	 	}
	}


	return audio_check;
}



#endif /* SUIT_MINICM_ENABLED */
