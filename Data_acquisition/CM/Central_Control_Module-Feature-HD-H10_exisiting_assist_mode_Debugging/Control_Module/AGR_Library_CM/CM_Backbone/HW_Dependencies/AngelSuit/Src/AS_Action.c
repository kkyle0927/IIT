
#include "AS_Action.h"

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

extern osSemaphoreId_t BinSem_PlaySoundHandle;
extern osSemaphoreId_t BinSem_PlayBeepHandle;
extern SUIT_AudioState_t SuitAudioState;

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




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void Run_Audio_Action(uint16_t id)
{
	//Todo : Refactor
	/* Test Audio Action : 20240514 by TJ */
	if (id == 0) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_1;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 1) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_2;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 2) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_3;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 3) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_4;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 4) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_5;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 5) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_6;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 6) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_7;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 7) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_8;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}	
	else if (id == 8) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_9;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 9) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_10;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 10) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_11;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 11) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_12;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 12) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_13;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 13) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_14;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 14) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_15;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 15) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_16;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 16) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_17;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 17) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_18;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 18) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_19;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 19) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_20;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 20) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_21;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 21) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_22;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 22) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_23;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == 23) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_24;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}

	else if (id == AUDIO_ID_25) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_25;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_26) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_26;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_27) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_27;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_28) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_28;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_29) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_29;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_30) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_30;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_31) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_31;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_32) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_32;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_33) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_33;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	// else if (id == AUDIO_ID_34) {
	// 	if(SuitAudioState.audio_Isplay != true)	{
	// 		audioOutputMode = AUDIO_FILE_PLAY;
	// 		SuitAudioState.audio_id = AUDIO_ID_34;
	// 		osSemaphoreRelease(BinSem_PlaySoundHandle);
	// 	}
	// }
	else if (id == AUDIO_ID_34) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_35;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_35) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_36;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_36) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_37;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_37) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_38;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_38) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_39;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_39) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_40;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}
	else if (id == AUDIO_ID_40) {
		if(SuitAudioState.audio_Isplay != true)	{
			audioOutputMode = AUDIO_FILE_PLAY;
			SuitAudioState.audio_id = AUDIO_ID_41;
			osSemaphoreRelease(BinSem_PlaySoundHandle);
		}
	}

	/* Beep */
	else if (id == 999) {
		if(SuitAudioState.audio_Isplay != true)	{
			SuitAudioState.audio_id = AUDIO_ID_100;
			osSemaphoreRelease(BinSem_PlayBeepHandle);
		}
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* SUIT_MINICM_ENABLED */
