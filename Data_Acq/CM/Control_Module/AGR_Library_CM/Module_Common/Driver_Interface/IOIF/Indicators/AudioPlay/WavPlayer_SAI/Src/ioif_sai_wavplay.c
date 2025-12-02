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

#include "ioif_sai_wavplay.h"


#ifdef IOIF_AUDIO_WAV_SAI_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */
#define IOIF_SD_PORT	  BSP_SD1

#define IOIF_AUDIO_PLAY_MUTE_MS		 50	  // 최초 audio 재생 시, DAC 안정화를 위한 hw mute 적용
#define IOIF_AUDIO_PLAY_FADE_IN_MS     16   // 기존 값 그대로 사용(12~20ms 권장)

#define IOIF_AUDIO_START_FILTER_MS     25   // 초반 HPF/스무딩 적극 구간(20~40ms)
#define IOIF_AUDIO_DC_HPF_FC_HZ        5.0f// DC 블록 컷오프(5~20Hz)
#define IOIF_AUDIO_SMOOTH_BETA         0.5f// 1차 스무딩 계수(0.35~0.5)

#define IOIF_AUDIO_SLEW_MS             8    // 슬루 제한이 적용될 길이(ms)
#define IOIF_AUDIO_MAX_STEP_FULL       3000.0f // 슬루 최종 허용 스텝(2000~4500)
#define IOIF_AUDIO_GATE_THRESH_Q15     400  // 게이트 임계(200~600, 약 -40dB 부근)


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

FRESULT sc_mount_res, sc_read_res, sc_write_res;

/* Declaration : FATFS variable */
FATFS 	audio_fs   __attribute__((section(".FATFS_RAMD1_data")));		//FATFS data structure works in RAM_D1(AXIRAM) with D-cache
FIL	  	audio_file __attribute__((section(".FATFS_RAMD1_data")));		//FATFS data structure works in RAM_D1(AXIRAM) with D-cache

/* Extern : Link driver (USB,SD CARD) */
extern const Diskio_drvTypeDef SD_Driver;
extern const Diskio_drvTypeDef USBH_Driver;

/* Extern : Mutex */
extern osMutexId_t SoundMutexHandle;
//extern SUIT_AudioState_t SuitAudioState;

/* Declaration : WaveFile Header */
IOIF_SAIWavFileHeader_t w_header;

bool IsAudio_Start = false;
bool audio_end = false;
/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef _USE_STM32H7_DCACHE
static void DcacheForcedWT_Enable(void);
static void DcacheForcedWT_Disable(void);
#endif

static uint32_t audio_r_byte = 0;
//static uint32_t audio_w_byte = 0;


extern volatile uint32_t sai_sample_rate;
extern volatile uint32_t sai_qbuf_in;				// queue buffer index : head
extern volatile uint32_t sai_qbuf_out;				// queue buffer index : tail
extern volatile uint32_t sai_q_buf_len;			// sampling data queue buffer length calculation
extern volatile uint32_t sai_qbuf_len; 				// total size of queue buffer

extern int16_t sai_q_buf_zero[IOIF_SAI_BUF_LEN*2] __attribute__((section(".Sai1_RxBuff")));		// queue buffer for SAI
extern IOIF_SAIChannel_t sai_q_buf[IOIF_SAI_BUF_LEN]__attribute__((section(".Sai1_RxBuff")));		// 2-channel queue buffer for SAI

extern bool sai_stop;							// SAI DMA transmit start/stop (no use in DMA circular, only normal mode)
extern uint8_t sai_r_buf_dummy;

extern uint32_t g_post_silence_blocks;   // 끝나고 난 뒤 무음 블록 카운트


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


IOIF_SAIWavState_t IOIF_SAI_WavAudioFSInit(uint8_t* DrivePath, IOIF_SAIWavPlayerType_t IoType)
{
	IOIF_SAIWavState_t res = IOIF_SAI_WAV_STATUS_ERROR;
	BSP_SD_Status_t sd_status = BSP_MSD_ERROR;
	IOIF_USBState usbh_status = IOIF_USB_FAIL;

#ifdef _USE_STM32H7_DCACHE
	DcacheForcedWT_Enable();
#endif

#ifndef _USE_DEBUG_CLI
	res = (IOIF_SAIWavState_t)IOIF_SAI_PlaySoundInit();
#endif /*_USE_DEBUG_CLI*/

	if(IoType == IOIF_SAI_WAVPLAY_SD)
	{
		sd_status = BSP_InitSD(IOIF_SD_PORT); 				// SD driver Init.
		if(sd_status != BSP_MSD_OK)
			return res;
		/* Linked SD Driver */
		FATFS_LinkDriver(&SD_Driver, (char*)DrivePath);		// IO driver link
	}
	else if(IoType == IOIF_SAI_WAVPLAY_USB)
	{
		usbh_status = IOIF_InitUSB(IOIF_USBH_MSC, IOIF_USB_TIM_NULL);
		if(usbh_status != IOIF_USB_OK)
			return res;
		/* Linked USB Driver */
		FATFS_LinkDriver(&USBH_Driver, (char*)DrivePath);		// IO driver link
	}
	else if(IoType == IOIF_SAI_WAVPLAY_RAM)
	{
		//Todo : will be implemented
	}
	else
		return res;

	sc_mount_res = f_mount(&audio_fs, (const TCHAR*)DrivePath, 1);									// File-system Mount
	if(sc_mount_res != FR_OK)
		return res;


#ifdef _USE_STM32H7_DCACHE
	DcacheForcedWT_Disable();
#endif

	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);	//mute off

	return res;
}


IOIF_SAIWavState_t IOIF_SAI_WavAudioFSDeInit(uint8_t* DrivePath)
{
	IOIF_SAIWavState_t res = IOIF_SAI_WAV_STATUS_ERROR;
	//bool sd_status;
	BSP_SD_Status_t sd_status;

	//sd_status = BSP_SD_DeInit();										// SD driver Init.
	sd_status = BSP_DeInitSD(IOIF_SD_PORT);
	if(sd_status != BSP_MSD_OK)
		return res;

	sc_mount_res = f_mount(NULL, (const TCHAR*)DrivePath, 0);				// File-system Unmount
	if(sc_mount_res != FR_OK)
		return res;

	return res;
}


IOIF_SAIWavState_t IOIF_SAI_GetWavFileInfo(uint8_t* DrivePath, uint8_t* filename, IOIF_SAIWavFileHeader_t* wav_header)
{
	IOIF_SAIWavState_t res = IOIF_SAI_WAV_STATUS_ERROR;

	if(!filename ||!wav_header)			//if filename or received header structure is not assigned,
		return res;

	FRESULT wavfile_open_res, wavfile_read_res;

	wavfile_open_res = f_open(&audio_file, (const TCHAR*)filename, FA_READ);
	if(wavfile_open_res == FR_OK)
	{
		wavfile_read_res = f_read(&audio_file, wav_header, sizeof(IOIF_SAIWavFileHeader_t), (unsigned int*)&audio_r_byte);		// reading header file
		if(wavfile_read_res == FR_OK)
			return res = IOIF_SAI_WAV_STATUS_OK;
	}
	return res;
}



IOIF_SAIWavState_t IOIF_SAI_PlayWaveFile(uint8_t* DrivePath, uint8_t* filename, uint16_t volume)
{
    IOIF_SAIWavState_t res = IOIF_SAI_WAV_STATUS_ERROR;
    bool IsMute = false;
    float volume_out = 0;

    if(!filename) return res;

    /* Mute On (MAX98357 SDMODE Low) */
    IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);

    bool audio_first_transmit = true;
    IsAudio_Start = true;
    g_post_silence_blocks = 0;

    FRESULT wavfile_close_res, wavfile_open_res, wavfile_read_res, wavfile_seek_res;

    volatile uint32_t r_len = sai_q_buf_len;
    int16_t sound_buf_frame[sai_q_buf_len*2];

    /* volume control */
    volume = (volume < 0) ? 0 : ((volume > 100) ? 100 : volume);
    volume_out = volume * 0.01f;
    if(volume == 0) IsMute = true;

#ifdef _USE_STM32H7_DCACHE
    DcacheForcedWT_Enable();
#endif

    osMutexAcquire(SoundMutexHandle, 3000);

    wavfile_open_res = f_open(&audio_file, (const TCHAR*)filename, FA_READ);
    if(wavfile_open_res == FR_OK)
    {
        wavfile_read_res = f_read(&audio_file, &w_header, sizeof(IOIF_SAIWavFileHeader_t), (unsigned int*)&audio_r_byte);
        wavfile_seek_res = f_lseek(&audio_file, sizeof(IOIF_SAIWavFileHeader_t));
        if(wavfile_read_res != FR_OK || wavfile_seek_res != FR_OK) { goto PLAY_EXIT; }

        audio_end = false;

        /* Fade-in: 총 프레임 수 */
        uint32_t fade_total_frames = (w_header.SampleRate * IOIF_AUDIO_PLAY_FADE_IN_MS) / 1000;
        uint32_t fade_done_frames  = 0;

        /* 초반 전용 필터(DC 블록 + 스무딩) 파라미터/상태 */
        uint32_t filter_total_frames = (w_header.SampleRate * IOIF_AUDIO_START_FILTER_MS) / 1000;
        uint32_t filter_done_frames  = 0;

        // 1차 DC-blocker 계수: y[n] = x[n] - x[n-1] + a*y[n-1]
        float hpf_a = expf(-2.0f * M_PI * IOIF_AUDIO_DC_HPF_FC_HZ / (float)w_header.SampleRate);

        float hpf_prev_x_L = 0.0f, hpf_prev_y_L = 0.0f;
        float hpf_prev_x_R = 0.0f, hpf_prev_y_R = 0.0f;

        float smooth_prev_L = 0.0f, smooth_prev_R = 0.0f;

        // slew limiter
        uint32_t slew_total_frames = (w_header.SampleRate * IOIF_AUDIO_SLEW_MS) / 1000U;
        float prev_out_L = 0.0f, prev_out_R = 0.0f;
        uint8_t prev_valid = 0;

#ifdef _USE_DEBUG_CLI
        while(audio_r_byte && CLI_KeepLoop())
#else
        while(audio_r_byte)
#endif
        {
            uint32_t buf_len;
            int32_t len;
            uint32_t q_offset;

            buf_len = ((sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len);
            buf_len = (sai_qbuf_len - buf_len) - 1;

            if (buf_len > 0)
            {
                /* WAV 데이터 읽기 */
                wavfile_read_res = f_read(&audio_file, &sound_buf_frame,
                                          r_len * (2 * w_header.NumChannels),
                                          (unsigned int*)&audio_r_byte);

                len = audio_r_byte;
                if (len != (int32_t)(r_len * (2 * w_header.NumChannels))) { break; }

                q_offset = sai_qbuf_in * sai_q_buf_len;

                /* 볼륨 + (강화된) Fade-in + 초반 전용 필터 */
                for (uint32_t i = 0; i < r_len; i++)
                {
                    /* 기본 볼륨 적용 */
                    float L = 0.0f, R = 0.0f;
                    if (w_header.NumChannels == 2) {
                        L = (float)sound_buf_frame[i*2 + 0] * volume_out;
                        R = (float)sound_buf_frame[i*2 + 1] * volume_out;
                    } else {
                        L = (float)sound_buf_frame[i] * volume_out;
                        R = L;
                    }

                    /* 1. 초반 필터 램프인: HPF/스무딩을 점진적으로 섞기 */
                    if (filter_done_frames < filter_total_frames)
                    {
                        // 원신호→HPF→스무딩
                        float yL = (L - hpf_prev_x_L) + hpf_a * hpf_prev_y_L;
                        float yR = (R - hpf_prev_x_R) + hpf_a * hpf_prev_y_R;
                        hpf_prev_x_L = L;  hpf_prev_y_L = yL;
                        hpf_prev_x_R = R;  hpf_prev_y_R = yR;

                        // 1차 스무딩
                        yL = (1.0f - IOIF_AUDIO_SMOOTH_BETA) * yL + IOIF_AUDIO_SMOOTH_BETA * smooth_prev_L;
                        yR = (1.0f - IOIF_AUDIO_SMOOTH_BETA) * yR + IOIF_AUDIO_SMOOTH_BETA * smooth_prev_R;
                        smooth_prev_L = yL;  smooth_prev_R = yR;

                        // 여기서 바로 y를 쓰지 않고, (0→1)로 점진적으로 블렌드
                        float m = (float)filter_done_frames / (float)filter_total_frames; // 0..1
                        L = (1.0f - m) * L + m * yL;
                        R = (1.0f - m) * R + m * yR;

                        filter_done_frames++;
                    }

                    /* 2. 페이드-인 커브: smoothstep (자연스러운 S커브) */
                    if (!IsMute && fade_total_frames > 0 && fade_done_frames < fade_total_frames)
                    {
                        float t = (float)fade_done_frames / (float)fade_total_frames;   // 0..1
                        float fade_gain = t * t * (3.0f - 2.0f * t);                    // 0→1
                        fade_done_frames++;

                        L *= fade_gain;
                        R *= fade_gain;
                    }

                    /* 3. 초반 슬루 제한: 페이드 구간 초반 N ms 동안만 급격 점프 억제 */
                    if (slew_total_frames > 0 && fade_done_frames <= slew_total_frames)
                    {
                        float u = (float)fade_done_frames / (float)slew_total_frames;   // 0..1
                        float max_step = IOIF_AUDIO_MAX_STEP_FULL * u;                   // 허용치 선형 증가

                        if (prev_valid)
                        {
                            float dl = L - prev_out_L;
                            float dr = R - prev_out_R;
                            if (dl >  max_step) dl =  max_step;
                            if (dl < -max_step) dl = -max_step;
                            if (dr >  max_step) dr =  max_step;
                            if (dr < -max_step) dr = -max_step;
                            L = prev_out_L + dl;
                            R = prev_out_R + dr;
                        }
                        prev_out_L = L; prev_out_R = R; prev_valid = 1;
                    }
                    else
                    {
                        prev_out_L = L; prev_out_R = R; prev_valid = 1;
                    }

                    /* 4. 초저레벨 게이트: 페이드 전반부의 미세 잡음 소거 */
                    if (fade_done_frames < (fade_total_frames >> 1))
                    {
                        int32_t li = (int32_t)L; if (li < 0) li = -li;
                        int32_t ri = (int32_t)R; if (ri < 0) ri = -ri;
                        if (li < IOIF_AUDIO_GATE_THRESH_Q15 && ri < IOIF_AUDIO_GATE_THRESH_Q15)
                        {
                            L = 0.0f; R = 0.0f;
                            prev_out_L = 0.0f; prev_out_R = 0.0f;
                        }
                    }

                    /* 최종 저장 */
                    if (L >  32767.0f) L =  32767.0f;
                    if (L < -32768.0f) L = -32768.0f;
                    if (R >  32767.0f) R =  32767.0f;
                    if (R < -32768.0f) R = -32768.0f;

                    sai_q_buf[q_offset + i].left  = (int16_t)L;
                    sai_q_buf[q_offset + i].right = (int16_t)R;
                }

                /* 큐 인덱스 업데이트 */
                if (((sai_qbuf_in + 1) % sai_qbuf_len) != sai_qbuf_out) {
                    sai_qbuf_in = (sai_qbuf_in + 1) % sai_qbuf_len;
                }

                /* 첫 전송 전 일정 시간 mute 유지 (기존) */
                if (audio_first_transmit == true && IsMute == false)
                {
                    osDelay(IOIF_AUDIO_PLAY_MUTE_MS);   // 20~50ms 권장
                    IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);
                    audio_first_transmit = false;
                }
            }
        }
    }

PLAY_EXIT:
    wavfile_close_res = f_close(&audio_file);
    if(wavfile_close_res == FR_OK) res = IOIF_SAI_WAV_STATUS_OK;

#ifdef _USE_STM32H7_DCACHE
    DcacheForcedWT_Disable();
#endif

    osMutexRelease(SoundMutexHandle);

    audio_end = true;
    sai_stop  = true;
    return res;
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

#ifdef _USE_STM32H7_DCACHE
static void DcacheForcedWT_Enable(void)
{
	_CACHE_CACR |= _CACHE_FORCEWT;
}

static void DcacheForcedWT_Disable(void)
{
	_CACHE_CACR &= ~(_CACHE_FORCEWT);
}
#endif



#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunSAIWav(cli_args_t *args)
{
	bool ret = false;

	char* filenamepStr;
	uint16_t vol = 0;
	//IOIF_SAIWavFileHeader_t* cliWavefile = {0,};
	FRESULT cliaudiomount_res = f_mount(&audio_fs, (const TCHAR*) "0:", 1);

	/* Function : Play Beep [start/error/off] */
	if(args->argc == 2 && args->cmpStr(0, "getinfo") == true)
	{
		CLI_Delay(10); //debug task 주기 1ms라서 바로 실행 시 audio fail 발생
		/* Function : Play Beep Functions */
		//if(f_mount(&audio_fs, "0:", 1) == FR_OK)
		if(cliaudiomount_res == FR_OK)
		{

			filenamepStr = args->getStr(1); // filename parser

			if(IOIF_SAI_GetWavFileInfo((uint8_t *)"0:", (uint8_t*) filenamepStr, &w_header) == IOIF_SAI_WAV_STATUS_OK)
			{
				CLI_Printf("\r\n");
				CLI_Printf(" - File Type : %s \r\n", w_header.Format);
				CLI_Printf(" - Audio Format : %d (1: PCM, 2: MS ADPCM, ...)\r\n", w_header.AudioFormat);
				CLI_Printf(" - Audio Channels : %d (1: Mono, 2: Stereo, 3: left, center, right, ...) \r\n", w_header.NumChannels);
				CLI_Printf(" - Audio Sampling Rate : %d \r\n", w_header.SampleRate);
				CLI_Printf(" - Audio Sample Frame Size : %d \r\n", w_header.BlockAlign);
				CLI_Printf(" - Audio Bits Per Sample : %d bps \r\n", w_header.BitsPerSample);
				CLI_Printf("\r\n");
			}
			else
			{
				CLI_Printf(" * File Read Fail! *  \r\n");
			}
			ret = true;
		}
		else
		{
			CLI_Printf(" * File-system Mount Fail! *  \r\n");
			ret = true;
		}
	} else if(args->argc == 3 && args->cmpStr(0, "playfile") == true)
	{
		/* Function : Play Wavefile Info. */
		CLI_Delay(10); //debug task 주기 1ms라서 바로 실행 시 audio fail 발생

		if(cliaudiomount_res == FR_OK)
		{
			filenamepStr = args->getStr(1); // filename parser
			vol = args->getData(2);

			if(IOIF_SAI_PlayWaveFile((uint8_t *)"0:", (uint8_t*)filenamepStr, vol) == IOIF_SAI_WAV_STATUS_OK)
				CLI_Printf(" * Audio Play Done *  \r\n");
			else
				CLI_Printf(" * Audio Play Fails! *  \r\n");

			ret = true;
		}
		else
		{
			CLI_Printf(" * File-system Mount Fail! *  \r\n");
			ret = true;
		}
	}

	if(ret == false)		//help
	{
		CLI_Printf(" * Audio Play - FATFS* \r\n");
		CLI_Printf("   Playing WAV audio file or beep or note through SAI \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   sai_fatfs getinfo [filename] \r\n");
		CLI_Printf("   sai_fatfs playfile [filename] [volume(0-100)] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   sai_fatfs playfile AudioFiles|002_gait_start.wav 50 \r\n");
	}
}

#endif /*_USE_DEBUG_CLI*/
#endif /*IOIF_AUDIO_WAV_SAI_ENABLED*/
