/*
 * ioif_sai_common.c
 *
 *  Created on: May 2, 2024
 *      Author: Angelrobotics
 */

#include "ioif_sai_common.h"

/** @defgroup SAI SAI
 * @brief SAI BSP module driver
 * @{
 */
#ifdef BSP_SAI_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

#define IOIF_AUDIO_FADE_OUT_BLOCKS 		64		// fade out 실행할 block 갯수
#define IOIF_AUDIO_POST_SILENCE_BLOCKS 	3		// fade out 후 buffer 내 데이터를 0으로 (부드러운 끊김을 위한 silent block)


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

bool IsBeep_Start = false;
uint32_t g_post_silence_blocks = 0;   // 끝나고 난 뒤 무음 블록 카운트


extern bool audio_end;
extern bool IsAudio_Start;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

volatile uint32_t sai_sample_rate = IOIF_SAI_SAMPLE_FREQ;		// SAI sample rate (can be changed by audio sample rate)
volatile uint32_t sai_qbuf_in = 0;				// queue buffer index : head
volatile uint32_t sai_qbuf_out = 0;				// queue buffer index : tail
volatile uint32_t sai_q_buf_len = 0;				// sampling data queue buffer length calculation
volatile uint32_t sai_qbuf_len = 0; 				// total size of queue buffer


int16_t sai_q_buf_zero[IOIF_SAI_BUF_LEN*2] __attribute__((section(".Sai1_RxBuff")));		// queue buffer for SAI
IOIF_SAIChannel_t sai_q_buf[IOIF_SAI_BUF_LEN]__attribute__((section(".Sai1_RxBuff")));		// 2-channel queue buffer for SAI

bool sai_stop = false;							// SAI DMA transmit start/stop (no use in DMA circular, only normal mode)
uint8_t sai_r_buf_dummy = 0;						// SAI DMA dummy byte

#ifdef _USE_DEBUG_CLI
static char *NoteStr[] = {
		IOIF_SAI_MACROENUMTOSTR(NOTE_C),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Csharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_D),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Dsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_E),
		IOIF_SAI_MACROENUMTOSTR(NOTE_F),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Fsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_G),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Gsharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_A),
		IOIF_SAI_MACROENUMTOSTR(NOTE_Asharp),
		IOIF_SAI_MACROENUMTOSTR(NOTE_B),
};
#endif /*_USE_DEBUG_CLI*/

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
/* Make Beep using SAI */
static float SaiGetNoteHz(int8_t octave, int8_t note);
static float MakeSineWave(float x);
static float MakeChirp(float t, float f0, float f1, float t1);

/* SAI TX Complete Callback */
static void SaiAudioDMATxCB(void* param);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_SAIState_t IOIF_SAI_PlaySoundInit(void)
{
	IOIF_SAIState_t res = IOIF_SAI_STATUS_ERROR;

	memset(sai_q_buf_zero, 0, sizeof(sai_q_buf_zero));												//SAI buffer Init.
	memset(sai_q_buf, 0, sizeof(sai_q_buf));															//SAI buffer Init.

	sai_q_buf_len = (sai_sample_rate * 1) / (1000/IOIF_SAI_BUF_MS);      								//SAI buffer length calculation
	sai_qbuf_len = IOIF_SAI_BUF_LEN / sai_q_buf_len;														//SAI buffer length calculation

	BSP_SetSAICB(BSP_SAI1, BSP_SAI_TX_CPLT_CALLBACK, SaiAudioDMATxCB, NULL);						//SAI Transmit Complete Callback Register

	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*) sai_q_buf_zero, &sai_r_buf_dummy, sai_q_buf_len*2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		res = IOIF_SAI_STATUS_OK;

	return res;
}


IOIF_SAIState_t IOIF_SAI_PlayNote(int8_t octave, int8_t note, uint16_t volume, uint32_t time_ms)
{
	uint32_t pre_time;
	int32_t sample_rate = sai_sample_rate;
	int32_t num_samples = sai_q_buf_len;
	float sample_point;
	int16_t sample_index = 0;
	float div_freq;
	int32_t volume_out;
	uint32_t buf_len;

	volume = (volume < 0) ? 0 : ((volume > 100) ? 100 : volume);
	volume_out = (INT16_MAX / 40) * volume / 100;

	//IsPlaying_SAI_PlayBeep = true;

	sai_qbuf_in=0; sai_qbuf_out=0;				//sai queue buffer in/out init.

	/* SAI Transmit DMA Start */
	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		sai_stop = false;

	div_freq = (float)sample_rate/(float)SaiGetNoteHz(octave, note);
	pre_time = BSP_GetTick();

	/* Mute off */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);

	while(BSP_GetTick() - pre_time <= time_ms)
	{
		buf_len = ((sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len);
		buf_len = (sai_qbuf_len - buf_len) - 1;

		if (buf_len > 0)
		{
			for (int i=0; i<num_samples; i++)
			{
				sample_point = MakeSineWave(2 * M_PI * (float)(sample_index) / ((float)div_freq));
#ifdef _USE_SOUND_MONO
				sai_q_buf[sai_q_in * sai_q_buf_len + i] = (int16_t)(sample_point * volume_out);
#else
				sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].left  = (int16_t)(sample_point * volume_out);
				sai_q_buf[sai_qbuf_in*sai_q_buf_len + i].right = (int16_t)(sample_point * volume_out);
#endif
				sample_index = (sample_index + 1) % (int)div_freq;
			}

			if (((sai_qbuf_in + 1) % sai_qbuf_len) != sai_qbuf_out)
			{
				sai_qbuf_in = (sai_qbuf_in+1) % sai_qbuf_len;
			}
		}
	}
	/* Mute on */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
	//sai_stop = true;
	return true;
}

volatile int debug_idx = 0; //for debugging
IOIF_SAIState_t IOIF_SAI_PlayBeep(uint32_t freq_hz, uint16_t volume, uint32_t time_ms)
{
	bool ret = false;

	uint32_t pre_time = 0;
	int32_t sample_rate = sai_sample_rate;
	int32_t num_samples = IOIF_SAI_BUF_MS * sai_sample_rate / 1000;
	float sample_point = 0;
	int16_t sample_index = 0;
	int16_t div_freq = 0;
	int32_t volume_out = 0;
	uint32_t buf_len = 0;

	IsBeep_Start = true;

	// volume control within 0 - 100
	volume = (volume < 0) ? 0 : ((volume > 100) ? 100 : volume);
	volume_out = (INT16_MAX / 40) * volume / 100;

	sai_qbuf_in = 0;
	sai_qbuf_out = 0;

	/* SAI Transmit DMA Start */
	if(BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA) == BSP_OK)
		sai_stop = false;

	div_freq = (float)sample_rate / (float)freq_hz; // 주파수로 나누지 않고 주파수를 바로 사용

	pre_time = BSP_GetTick();

	/* Mute off */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);

	while (BSP_GetTick() - pre_time <= time_ms) {
		buf_len = ((sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len);
		buf_len = (sai_qbuf_len - buf_len) - 1;

		if (buf_len > 0) {
			for (int i = 0; i < num_samples; i++) {
				float t = (float)i / (float)sample_rate;
				// sample_point = MakeChirp(t, (float)freq_hz, (float)freq_hz * 2, (float)time_ms);
				sample_point = MakeSineWave(2 * M_PI * (float)(sample_index) / ((float)div_freq));

#ifdef _USE_SOUND_MONO
				sai_q_buf[sai_qbuf_in * sai_q_buf_len + i] = (int16_t)(sample_point * volume_out);
#else
				sai_q_buf[sai_qbuf_in * sai_q_buf_len + i].left  = (int16_t)(sample_point * volume_out);
				sai_q_buf[sai_qbuf_in * sai_q_buf_len + i].right = (int16_t)(sample_point * volume_out);
#endif
				sample_index = (sample_index + 1) % (int)div_freq;

				debug_idx++;
			}

			if (((sai_qbuf_in + 1) % sai_qbuf_len) != sai_qbuf_out) {
				sai_qbuf_in = (sai_qbuf_in + 1) % sai_qbuf_len;
			}
		}
	}

	/* Mute on */
	IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
	//sai_stop = true;
	IsBeep_Start = false;
	ret = true;

	return ret;
}

IOIF_SAIState_t IOIF_SAI_StartBeep(uint32_t volume)
{
	uint16_t beepsoundfreq = 300;			// frequency
	int note_durations[] = {10, 14, 14};	// durations

	for (uint8_t i = 0; i < 2; i++) {
		int note_duration = 1000 / note_durations[i];
		IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
		osDelay(note_duration * 0.3);
	}

	return true;
}


IOIF_SAIState_t IOIF_SAI_ErrorBeep(uint32_t volume)
{
	uint16_t beepsoundfreq = 1500;
	int note_durations = 10;

	int note_duration = 1000 / note_durations;
	IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
	osDelay(note_duration * 0.3);

	return true;
}


IOIF_SAIState_t IOIF_SAI_OffBeep(uint32_t volume)
{
	uint16_t beepsoundfreq = 300;				// frequency
	int note_durations[] = {10, 10, 14, 14};	// durations

	for(uint8_t i = 0; i < 3; i++) {
		int note_duration = 1000 / note_durations[i];
		IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
		osDelay(note_duration * 0.3);
	}

	return true;
}

IOIF_SAIState_t IOIF_SAI_StandbyBeep(uint32_t volume)
{
    uint16_t beepsoundfreq = 440;  	            // Standard A (A4) for a clear tone
    int note_durations[] = {15, 10, 10, 10};    // 500ms duration

	for (uint8_t i = 0; i < 4; i++) {
		int note_duration = 1000 / note_durations[i];
		IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
		osDelay(note_duration * 0.3);
	}

	return true;
}

IOIF_SAIState_t IOIF_SAI_LeftThighLiftBeep(uint32_t volume)
{
    uint16_t beepsoundfreq = 1047;       // C6 (Higher pitch for lifting)
    int note_durations[] = {15, 10};    // 250ms duration

    for (uint8_t i = 0; i < 2; i++) {
        int note_duration = 1000 / note_durations[i];
        IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
        osDelay(note_duration * 0.3);
    }

    return true;
}

IOIF_SAIState_t IOIF_SAI_RightThighLiftBeep(uint32_t volume)
{
    uint16_t beepsoundfreq = 1175;       // D6 (Higher pitch for lifting)
    int note_durations[] = {15, 10};    // 250ms duration

    for (uint8_t i = 0; i < 2; i++) {
        int note_duration = 1000 / note_durations[i];
        IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
        osDelay(note_duration * 0.3);
    }

    return true;
}

IOIF_SAIState_t IOIF_SAI_LvWalkLeftSwingBeep(uint32_t volume)
{
    uint16_t beepsoundfreq = 196;       // G3 (Lower pitch for swinging)
    int note_durations[] = {15, 10};    // 200ms duration

    for (uint8_t i = 0; i < 2; i++) {
        int note_duration = 1000 / note_durations[i];
        IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
        osDelay(note_duration * 0.3);
    }

    return true;
}

IOIF_SAIState_t IOIF_SAI_LvWalkRightSwingBeep(uint32_t volume)
{
    uint16_t beepsoundfreq = 220;       // A3 (Lower pitch for swinging)
    int note_durations[] = {15, 10};    // 200ms duration

    for (uint8_t i = 0; i < 2; i++) {
        int note_duration = 1000 / note_durations[i];
        IOIF_SAI_PlayBeep(beepsoundfreq, volume, note_duration);
        osDelay(note_duration * 0.3);
    }

    return true;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */


//static void SaiAudioDMATxCB(void* param)
//{
//	uint32_t len;
//
//	//IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
//
//	if (sai_stop == true) // if sai transmit is completed
//	{
//		sai_stop = false;
//		BSP_RunSAIDMA(BSP_SAI1, (uint8_t*) sai_q_buf_zero, &sai_r_buf_dummy, sai_q_buf_len*2, BSP_SAI_TRANSMIT_DMA);
//		return;
//	}
//
//	len = (sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len;											// sai queue buffer length check
//
//	if (len > 0)																								// if sound data exists in sai queue buffer,
//	{
//		if(audio_end != true)
//			IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_SET);
//		else if(audio_end == true && IsAudio_Start == false)
//			IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
//
//		//HAL_SAI_Transmit_DMA(hsai, (uint8_t*)&sai_q_buf[sai_qbuf_out*sai_q_buf_len], sai_q_buf_len * 2);		// sai DMA trasmit start from next frame
//		BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)&sai_q_buf[sai_qbuf_out*sai_q_buf_len], &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA);
//		if (sai_qbuf_out != sai_qbuf_in)
//			sai_qbuf_out = (sai_qbuf_out + 1) % sai_qbuf_len;													// queue buffer index increase
//	}
//	else
//	{
//		//HAL_SAI_Transmit_DMA(hsai, (uint8_t*)sai_q_buf_zero, sai_q_buf_len * 2);								// first transmit
//		IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
//		BSP_RunSAIDMA(BSP_SAI1, (uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,  sai_q_buf_len * 2, BSP_SAI_TRANSMIT_DMA);
//	}
//
//	if(audio_end == true)
//		IsAudio_Start = false;
//
//}

static void SaiAudioDMATxCB(void* param)
{
    uint32_t len;

    if (sai_stop == true) // if sai transmit is completed
    {
        sai_stop = false;
        BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)sai_q_buf_zero, &sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);
        return;
    }

    // 큐에 남아 있는 블록 개수
    len = (sai_qbuf_len + sai_qbuf_in - sai_qbuf_out) % sai_qbuf_len;

    if (len > 0)
    {
        // 오디오 데이터가 아직 남아 있으면,"끝난 뒤 무음 블록 카운트"는 리셋
        g_post_silence_blocks = 0;

        if (IsBeep_Start == false)
        {
            // Fade-out: audio_end == true 이고, 남은 블록 수(len)가 IOIF_FADE_OUT_BLOCKS 이하일 때
        	if (audio_end == true &&len <= IOIF_AUDIO_FADE_OUT_BLOCKS &&IOIF_AUDIO_FADE_OUT_BLOCKS > 0)
        	{
        	    uint32_t N = IOIF_AUDIO_FADE_OUT_BLOCKS;
        	    uint32_t k_block = N - len;   // 0 ~ N-1 : 현재 페이드 블록 인덱스

        	    if (k_block >= N)
        	        k_block = N - 1;

        	    // 전체 페이드 구간을 0.0 ~ 1.0 으로 보고,
        	    // 이 블록의 시작/끝 t 를 계산
        	    float t0 = (float)k_block     / (float)N;       // 이 블록의 시작 위치
        	    float t1 = (float)(k_block+1) / (float)N;       // 이 블록의 끝 위치
        	    if (t1 > 1.0f) t1 = 1.0f;

        	    // 부드러운 곡선: (1 - t)^2  (처음엔 천천히, 나중에 빨리 줄어듦)
        	    float g0 = 1.0f - t0;
        	    float g1 = 1.0f - t1;
        	    g0 = g0 * g0;      // (1 - t0)^2
        	    g1 = g1 * g1;      // (1 - t1)^2
        	    if (g0 < 0.0f) g0 = 0.0f;
        	    if (g1 < 0.0f) g1 = 0.0f;

        	    uint32_t base = sai_qbuf_out * sai_q_buf_len;

        	    // 블록 안에서 g0 → g1 로 선형 보간
        	    for (uint32_t i = 0; i < sai_q_buf_len; i++)
        	    {
        	        float alpha;
        	        if (sai_q_buf_len > 1)
        	            alpha = (float)i / (float)(sai_q_buf_len - 1);
        	        else
        	            alpha = 0.0f;

        	        float gain = g0 + (g1 - g0) * alpha;   // 이 샘플에 적용할 gain

        	        int32_t l = sai_q_buf[base + i].left;
        	        int32_t r = sai_q_buf[base + i].right;

        	        l = (int32_t)((float)l * gain);
        	        r = (int32_t)((float)r * gain);

        	        if (l >  32767) l =  32767;
        	        if (l < -32768) l = -32768;
        	        if (r >  32767) r =  32767;
        	        if (r < -32768) r = -32768;

        	        sai_q_buf[base + i].left  = (int16_t)l;
        	        sai_q_buf[base + i].right = (int16_t)r;
        	    }
        	}

            // 재생 중(큐에 데이터 있음)에는 SDMODE High 유지
            IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT,IOIF_MAX98357_SDMODE_GPIO_PIN,IOIF_GPIO_PIN_SET);
            // 이번 블록 DMA 전송
            BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)&sai_q_buf[sai_qbuf_out * sai_q_buf_len],&sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);

            if (sai_qbuf_out != sai_qbuf_in)
                sai_qbuf_out = (sai_qbuf_out + 1) % sai_qbuf_len;
        }
        else
        {
            // Beep용 경로는 그대로 유지
            BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)&sai_q_buf[sai_qbuf_out * sai_q_buf_len],&sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);
            if (sai_qbuf_out != sai_qbuf_in)
                sai_qbuf_out = (sai_qbuf_out + 1) % sai_qbuf_len;
        }
    }
    else
    {
        // len == 0: 더 이상 오디오 데이터가 큐에 없음
        if (audio_end == true && len != 0)
        {
            // 파일 끝까지 다 재생했고(audio_end == true),  큐도 비었을 때, 일정 블록 수만큼 dummy 0 를 재생한 후에 Mute (Soft mute)

            if (g_post_silence_blocks < IOIF_AUDIO_POST_SILENCE_BLOCKS)
            {
                // 아직 무음 블록을 충분히 재생하지 않았다면:
                // SDMODE는 High(켜진 상태) 유지 + 0데이터 재생
                IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT,IOIF_MAX98357_SDMODE_GPIO_PIN,IOIF_GPIO_PIN_SET);
                BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)sai_q_buf_zero,&sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);

                g_post_silence_blocks++;
            }
            else
            {
                // 충분히 무음 구간을 재생한 뒤에 SDMODE Low로 Mute
                IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT, IOIF_MAX98357_SDMODE_GPIO_PIN, IOIF_GPIO_PIN_RESET);
                BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)sai_q_buf_zero,&sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);

                IsAudio_Start = false;   // 재생 완전히 끝
            }
        }
        else
        {
            // 아직 audio_end == false 인데 len == 0 이라면,파일 읽기와 DMA 사이에 잠깐 비는 상황일 수 있으니
            // 그냥 0 데이터 보내면서 SDMODE는 High 유지
            IOIF_WriteGPIOPin(IOIF_MAX98357_SDMODE_GPIO_PORT,IOIF_MAX98357_SDMODE_GPIO_PIN,IOIF_GPIO_PIN_SET);
            BSP_RunSAIDMA(BSP_SAI1,(uint8_t*)sai_q_buf_zero,&sai_r_buf_dummy,sai_q_buf_len * 2,BSP_SAI_TRANSMIT_DMA);
        }
    }

    if (audio_end == true)
        IsAudio_Start = false;
}


static float SaiGetNoteHz(int8_t octave, int8_t note)
{
	float hz;
	float f_note;

	if (octave < 1) octave = 1;
	if (octave > 8) octave = 8;

	if (note <  1) note = 1;
	if (note > 12) note = 12;

	f_note = (float)(note-10)/12.0f;

	hz = pow(2, (octave-1)) * 55 * pow(2, f_note);

	return hz;
}

// to be changed sinf(x)
static float MakeSineWave(float x)
{
	const float B = 4 / M_PI;
	const float C = -4 / (M_PI * M_PI);

	return -(B * x + C * x * ((x < 0) ? -x : x));
}

static float MakeChirp(float t, float f0, float f1, float t1) 
{
    float k = (f1 - f0) / t1;
    return sinf(2 * M_PI * (f0 * t + 0.5 * k * t * t));
}

#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */
void CLI_RunSAICommon(cli_args_t *args)
{
	bool ret = false;

	uint32_t cliVolume = 0;
	int8_t cliOctave = 0; int8_t cliNote = 0; uint16_t cliNoteVol = 0; uint32_t cliNoteTime_ms = 0;

	/* Function : Play Beep [start/error/off] */
	if(args->argc == 3 && args->cmpStr(0, "playbeep") == true)
	{
		if(args->cmpStr(1, "start") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_StartBeep(cliVolume);
			CLI_Printf(" * Start Beep Done. *  \r\n");

			ret = true;
		}
		else if(args->cmpStr(1, "error") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_ErrorBeep(cliVolume);

			CLI_Printf(" * Error Beep Done. *  \r\n");
			ret = true;
		}
		else if(args->cmpStr(1, "off") == true)
		{
			cliVolume = args -> getData(2);
			IOIF_SAI_OffBeep(cliVolume);

			CLI_Printf(" * Off Beep Done. *  \r\n");
			ret = true;
		}
		else
			ret = false;

	} else if(args->argc == 5 && args->cmpStr(0, "playnote") == true)
	{	/* Function : Play Note Functions */
		cliOctave = args -> getData(1);
		cliOctave = (cliOctave < 1) ? 1 : ((cliOctave > 8) ? 8 : cliOctave);							// 1 to 8 limitation
		cliNote = args -> getData(2);
		cliNote = (cliNote < 1) ? 1 : ((cliNote > 12) ? 12 : cliNote);									// 1 to 12 limitation
		cliNoteVol = args -> getData(3);
		cliNoteVol = (cliNoteVol < 0) ? 0 : ((cliNoteVol > 100) ? 100 : cliNoteVol);					// 1 to 100 limitation
		cliNoteTime_ms = args -> getData(4);

		IOIF_SAI_PlayNote(cliOctave, cliNote, cliNoteVol, cliNoteTime_ms);
		CLI_Printf(" * Playing Note Done. \r\n");
		CLI_Printf(" * Octave : %d, Note : %s \r\n", cliOctave, NoteStr[cliNote - 1]);
		CLI_Printf(" * Volume : %d, Duration : %d \r\n", cliNoteVol, cliNoteTime_ms);

		ret = true;
	} else if(ret == false)		//help
	{
		CLI_Printf(" * Audio Play * \r\n");
		CLI_Printf("   Playing beep or note through SAI. \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   sai_basic playbeep [start/off/error] [volume(1~100)] \r\n");
		CLI_Printf("   sai_basic playnote [octave(1~8)] [note(1~12)] [volume(0~100)] [time(ms)] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   sai_basic playbeep start 50 \r\n");
	}
}
#endif /* _USE_DEBUG_CLI */

#endif /* BSP_SAI_MODULE_ENABLED */
