/**
 *-----------------------------------------------------------
 *                      WAV File Play
 *-----------------------------------------------------------
 * @file ioif_sai_wavplay.h
 * @date Created on: Sep 19, 2023
 * @author AngelRobotics FW Team
 * @brief Code for WAV file play with SAI interface.
 *
 * Todo: Add Annotation
 *
 * @ref SAI Datasheet
 */

#ifndef WAVPLAYER_SAI_INC_IOIF_AUDIO_WAVPLAY_SAI_H_
#define WAVPLAYER_SAI_INC_IOIF_AUDIO_WAVPLAY_SAI_H_

#include "module.h"

/** @defgroup AUDIO WAV SAI IOIF
  * @brief AUDIO WAV SAI driver
  * @{
  */

#ifdef IOIF_AUDIO_WAV_SAI_ENABLED

#include "fatfs.h"
#include "bsp_sdmmc.h"
#include "ioif_sai_common.h"
#include "ioif_usb_common.h"
/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#ifdef _USE_STM32H7_DCACHE
#define _CACHE_FORCEWT		(1<<2)
#define _CACHE_CACR			(*(volatile uint32_t*) (0xE000EF9C))
#endif

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_SAIWavState_t {
    IOIF_SAI_WAV_STATUS_OK = 0,
    IOIF_SAI_WAV_STATUS_ERROR,
} IOIF_SAIWavState_t;

typedef enum _IOIF_SAIWavPlayerType_t {
	IOIF_SAI_WAVPLAY_SD = 0,
	IOIF_SAI_WAVPLAY_USB = 1,
	IOIF_SAI_WAVPLAY_RAM = 2
} IOIF_SAIWavPlayerType_t;

typedef struct _IOIF_SAIWavFileHeader_t{
   /* RIFF */
   uint8_t ChunkID[4];       //4		"RIFF"
   int32_t ChunkSize;        //8		File Size = Chunk Size + 8byte (need to add Chunk ID 4byte, Chunk size 4byte)
   uint8_t Format[4];        //12		"WAVE"

   /* FMT Sub-Chunk */
   uint8_t Subchunk1ID[4];   //16      "FMT + 'space'"
   int32_t Subchunk1Size;    //20		FMT sub-chunk size
   int16_t AudioFormat;      //22   	1: PCM/Uncompressed, 2: MS ADPCM, ... , 80: MPEG, etc..
   int16_t NumChannels;      //24   	1: MONO, 2: STEREO, 3: left, center, right, 4:..., etc...
   int32_t SampleRate;       //28      Audio Sampling Rate
   int32_t ByteRate;         //32		Average Bytes Per Second
   int16_t BlockAlign;       //34  	Sample Frame size
   int16_t BitsPerSample;    //36		Bit Per Sample, 8bit/16bit/32bit...

   /* Data Sub-Chunk */
   uint8_t Subchunk2ID[4];	 //40		"data"
   int32_t Subchunk2Size;    //44 		Data Size
} IOIF_SAIWavFileHeader_t;	 // Total : 44 bytes

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern bool audio_end;



/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */


/* Player Init. & Deinit. */
IOIF_SAIWavState_t 	IOIF_SAI_WavAudioFSInit(uint8_t* DrivePath, IOIF_SAIWavPlayerType_t IoType);
IOIF_SAIWavState_t 	IOIF_SAI_WavAudioFSDeInit(uint8_t* DrivePath);

/* Get Wavefile Header Info. */
IOIF_SAIWavState_t 	IOIF_SAI_GetWavFileInfo(uint8_t* DrivePath, uint8_t* filename, IOIF_SAIWavFileHeader_t* wav_header);

/* Play Wavefile */
IOIF_SAIWavState_t IOIF_SAI_PlayWaveFile(uint8_t* DrivePath, uint8_t* filename, uint16_t volume);

/* D-Cache Write Through Forced Mode for Cache Coherence */

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunSAIWav(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /* MODULE : IOIF_AUDIO_WAV_SAI_ENABLED */

#endif /* WAVPLAYER_SAI_INC_IOIF_AUDIO_WAVPLAY_SAI_H_ */
