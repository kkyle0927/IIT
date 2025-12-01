#include "AS_Audio_Ctrl.h"


/**
 *-----------------------------------------------------------
 *      TYPE DEFINITIONS AND ENUMERATIONS AND VARIABLES
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

//#define CONSTANT_TO_STRING(constant) ConstantToString(constant)

TaskObj_t audioCtrlTask;
uint32_t audioCtrlLoopCnt;

extern SUIT_AudioState_t SuitAudioState;
extern const char* 		 SUITCM_AudioFiles[];
extern osSemaphoreId_t   BinSem_PlayBeepHandle;
extern audioType_t	     audioOutputMode;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes for this module.
 */

/* -------------------- STATE FUNCTION -------------------- */
static void StateOff_Ent(void);
static void StateStandby_Ent(void);
static void StateStandby_Run(void);
static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);
static void StateError_Run(void);

static const char* ConstantToString(int constant);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void InitAudioTask(void)
{
	/* Init Task */
	InitTask(&audioCtrlTask);
	DOPC_AssignTaskID(&audioCtrlTask, TASK_IDX_AUDIO_CTRL);

	/* State Definition */
	TASK_CREATE_STATE(&audioCtrlTask, TASK_STATE_OFF,     NULL,     		NULL,            NULL,               false);
	TASK_CREATE_STATE(&audioCtrlTask, TASK_STATE_STANDBY, NULL, 			StateStandby_Run,NULL,               true);
	TASK_CREATE_STATE(&audioCtrlTask, TASK_STATE_ENABLE,  StateEnable_Ent,  StateEnable_Run, StateEnable_Ext,    false);
	TASK_CREATE_STATE(&audioCtrlTask, TASK_STATE_ERROR,   NULL,             StateError_Run,  NULL,               false);

	//initAudioRes = WavAudio_FS_Init((uint8_t*) "1:", IOIF_WAVPLAY_SD);

}

void RunAudioTask(void)
{
	RunTask(&audioCtrlTask);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateStandby_Run(void)
{
	uint8_t t_wholeBodyCtrlState = DOPC_GetTaskState(TASK_IDX_WHOLE_BODY_CTRL);
	if (t_wholeBodyCtrlState == TASK_STATE_ENABLE) {
//		initAudioRes = WavAudio_FS_Init((uint8_t*) "1:", IOIF_WAVPLAY_SD);
		StateTransition(&audioCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}
}

static void StateEnable_Ent(void)
{

}

static void StateEnable_Run(void)
{

	osSemaphoreAcquire(BinSem_PlayBeepHandle, osWaitForever);

	if(SuitAudioState.audio_Isplay != true && SuitAudioState.audio_id != AUDIO_ID_INVALID && audioOutputMode != AUDIO_FILE_PLAY)
	{
		SuitAudioState.audio_Isplay = true;

//		if(SuitAudioState.audio_playback == true)
//		{
			//PlayWaveFile((uint8_t*)"1:", (uint8_t*)ConstantToString(SuitAudioState.audio_id));
			// if(SuitAudioState.audio_id == 100)
			// {
			// 	IOIF_SAI_StartBeep(50);
			// }
			// else if (SuitAudioState.audio_id == 101)
			// {
			// 	IOIF_SAI_OffBeep(50);
			// }
			if(SuitAudioState.audio_id == AUDIO_ID_100) {
				IOIF_SAI_StandbyBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_101) {
				IOIF_SAI_LeftThighLiftBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_102) {
				IOIF_SAI_RightThighLiftBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_103) {
				IOIF_SAI_LvWalkLeftSwingBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_104) {
				IOIF_SAI_LvWalkRightSwingBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_105) {
				IOIF_SAI_ErrorBeep(audioVolume);
			} else if (SuitAudioState.audio_id == AUDIO_ID_106) {
				IOIF_SAI_PlayBeep(1000, 30, 200);
				IOIF_SAI_PlayBeep(1000, 0, 700);
			}
//			SuitAudioState.audio_playback = false;
//		}

		SuitAudioState.audio_Isplay = false;
	}



	//	audioCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{

}

static void StateError_Run(void)
{

}


