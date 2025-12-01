/*
 * AS_Audio_Ctrl.h
 *
 *  Created on: May 30, 2024
 *      Author: Angelrobotics
 */

#ifndef APPS_TASKS_ANGELSUIT_AUDIO_CTRL_INC_AS_AUDIO_CTRL_H_
#define APPS_TASKS_ANGELSUIT_AUDIO_CTRL_INC_AS_AUDIO_CTRL_H_

#include "main.h"
#include "module.h"

#ifdef SUIT_MINICM_ENABLED
/*CDI*/
#include "data_object_common.h"				//State Machine Definitions
/*IOIF*/
#include "ioif_sai_wavplay.h"
/*App*/
#include "AS_data_ctrl.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _SUIT_Audio_ID
{
	AUDIO_ID_INVALID,
	AUDIO_ID_1 = 1,
	AUDIO_ID_2,
	AUDIO_ID_3,
	AUDIO_ID_4,
	AUDIO_ID_5,
	AUDIO_ID_6,
	AUDIO_ID_7,
	AUDIO_ID_8,
	AUDIO_ID_9,
	AUDIO_ID_10,
	AUDIO_ID_11,
	AUDIO_ID_12,
	AUDIO_ID_13,
	AUDIO_ID_14,
	AUDIO_ID_15,
	AUDIO_ID_16,
	AUDIO_ID_17,
	AUDIO_ID_18,
	AUDIO_ID_19,
	AUDIO_ID_20,
	AUDIO_ID_21,
	AUDIO_ID_22,
	AUDIO_ID_23,
	AUDIO_ID_24,

	/* Risk Management */
	AUDIO_ID_25,
	AUDIO_ID_26,
	AUDIO_ID_27,
	AUDIO_ID_28,
	AUDIO_ID_29,
	AUDIO_ID_30,
	AUDIO_ID_31,
	AUDIO_ID_32,
	
	AUDIO_ID_33,
	AUDIO_ID_34,

	/* STS Mode */
	AUDIO_ID_35,
	AUDIO_ID_36,
	AUDIO_ID_37,
	AUDIO_ID_38,
	AUDIO_ID_39,
	AUDIO_ID_40,
	AUDIO_ID_41,

	AUDIO_ID_100 = 100,
	AUDIO_ID_101,
	AUDIO_ID_102,
	AUDIO_ID_103,
	AUDIO_ID_104,
	AUDIO_ID_105,
	AUDIO_ID_106, // Timer Beep Sound
	AUDIO_ID_MAX,
} SUIT_Audio_ID;

typedef enum _SUIT_Audio_Language {
	AUDIO_LANGUAGE_KOREAN,
	AUDIO_LANGUAGE_ENGLISH,

	AUDIO_LANGUAGE_MAX
} SUIT_Audio_Language;

typedef enum _SUIT_Audio_Gender {
	AUDIO_GENDER_MALE,
	AUDIO_GENDER_FEMALE
} SUIT_Audio_Gender;

typedef enum _SUIT_Audio_Preview {
	AUDIO_PREVIEW_OFF,
	AUDIO_PREVIEW_ON
} SUIT_Audio_Preview;


typedef struct _SUIT_AudioState_t{

	bool 		  		audio_Isplay;
	bool		  		audio_playback;
	SUIT_Audio_ID 		audio_id;
	SUIT_Audio_ID 		latest_played_audio_id;
	SUIT_Audio_Language audio_lang;

} SUIT_AudioState_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

/* SUIT CM Audio Files List */







/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitAudioTask(void);
void RunAudioTask(void);

#endif /* SUIT_MINICM_ENABLED */


#endif /* APPS_TASKS_ANGELSUIT_AUDIO_CTRL_INC_AS_AUDIO_CTRL_H_ */
