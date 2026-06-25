/**
 ******************************************************************************
 * @file    ioif_agrb_sai_audioplayer.c
 * @author  HyundoKim
 * @brief   [IOIF] SAI Audio Player — Ring Queue 기반 글리치-프리 오디오 재생
 * @version 1.0
 * @date    2026-04-13
 *
 * @details
 * [Ring Queue SPSC (Single-Producer Single-Consumer) 패턴]
 * - Producer: WAV Reader Task (SD → Queue)
 * - Consumer: SAI EventTask 콜백 (Queue → DMA 버퍼)
 * - volatile 인덱스로 lock-free 동기화 (Cortex-M7 단일코어)
 *
 * [DMA_CIRCULAR Double-Buffer (ST 표준 패턴)]
 * - DMA Circular로 시작 → 절대 멈추지 않음 → 블록 경계 gap 없음
 * - HALF_COMPLETE/COMPLETE 콜백: Queue → memcpy → inactive DMA half
 * - SD 읽기와 DMA 소비를 Task로 완전 분리
 * - Queue underrun 시 silence 전송 (글리치 방지)
 * - Queue + DMA buffer 모두 RAM_D2 Non-cacheable (.dma_buffer)
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "ioif_agrb_sai_audioplayer.h"

#if defined(AGRB_IOIF_SAI_ENABLE) && defined(AGRB_IOIF_FILESYSTEM_ENABLE)

#include <string.h>

#include "cmsis_os2.h"
#include "ioif_agrb_fs.h"

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

#define WAV_FMT_HEADER_SIZE     (36U)
#define WAV_MAX_HEADER_SEARCH   (512U)

#define Q_BLOCK_SAMPLES         IOIF_SAI_AUDIOPLAYER_BLOCK_SAMPLES
#define Q_BLOCK_BYTES           (Q_BLOCK_SAMPLES * sizeof(IOIF_SAI_AudioSample_t))
#define Q_DEPTH                 IOIF_SAI_AUDIOPLAYER_QUEUE_DEPTH

/** @brief EOF fade-out ramp 길이 (samples). ~1.5ms @44.1kHz, data→silence 팝 방지 */
#define FADE_OUT_SAMPLES        (64)

/** @brief Sample rate (Hz) — must match CubeMX SAI AudioFrequency */
#define WAVPLAYER_SAMPLE_RATE   (44100)

/** @brief 디버그: 1=SD 대신 sine wave 주입 (Queue/DMA 메커니즘 검증) */
volatile uint8_t g_dbg_audioplayer_sine_inject = 0;

/**
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static bool s_initialized = false;

/** @brief 교차 Task 접근 — volatile 필수
 *  Writer: Play(caller), Stop(caller), _SAI_Callback(EventTask), _ReaderTask
 *  Reader: _SAI_Callback guard(EventTask), IsPlaying(any task) */
static volatile bool s_playing = false;

/** @brief SAI instance ID (from Init config) */
static IOIF_SAIx_t s_sai_id = 0;

/** @brief Ring Queue — RAM_D2 (Non-cacheable, DMA 직접 접근 가능)
 *  PlayDirect 패턴: DMA가 Queue 블록에서 직접 전송하므로
 *  DMA 접근 가능 + Non-cacheable 필수 */
static IOIF_SAI_AudioSample_t s_q_buf[Q_BLOCK_SAMPLES * Q_DEPTH]
    __attribute__((section(".dma_buffer"), aligned(32)));

static volatile uint32_t s_q_wr = 0;   /**< Write head (block index, Reader Task only) */
static volatile uint32_t s_q_rd = 0;   /**< Read head  (block index, SAI Callback only) */

/** @brief SAI DMA 전달용 임시 버퍼 — RAM_D2 (IOIF_SAI_Play가 DMA 버퍼로 복사) */
static IOIF_SAI_AudioSample_t s_xfer_buf[Q_BLOCK_SAMPLES]
    __attribute__((section(".dma_buffer"), aligned(32)));

/** @brief WAV Reader Task */
static TaskHandle_t      s_reader_task = NULL;
static volatile bool     s_reader_stop = false;
static SemaphoreHandle_t s_q_space_sem = NULL; /**< Reader가 큐 full일 때 대기 */

/** @brief Done callback */
static IOIF_SAI_AudioPlayer_DoneCallback_t s_done_cb  = NULL;
static void*                             s_done_ctx = NULL;

/** @brief WAV file state */
static struct {
    IOIF_FILEx_t fid;
    uint32_t     data_size;
    uint32_t     bytes_read;
    uint16_t     num_channels;
    uint16_t     bits_per_sample;
    uint32_t     sample_rate;
    volatile bool eof_reached;  /**< Reader Task(W) ↔ SAI EventTask(R) 교차 접근 */
} s_wav = {0};

/* Debug */
volatile int32_t  g_dbg_audioplayer_ret      = 0x7F;
volatile uint32_t g_dbg_audioplayer_underruns = 0;
volatile uint8_t  g_dbg_audioplayer_q_level   = 0;
volatile int32_t  g_dbg_audioplayer_play_ret  = 0;    /**< IOIF_SAI_Play 리턴값 (콜백 내) */
volatile uint32_t g_dbg_audioplayer_cb_count  = 0;    /**< 콜백 호출 횟수 */
volatile uint32_t g_dbg_audioplayer_wav_sr    = 0;    /**< 파싱된 WAV sample rate (Hz) */
volatile uint16_t g_dbg_audioplayer_wav_ch    = 0;    /**< 파싱된 WAV 채널 수 (1=mono, 2=stereo) */
volatile uint16_t g_dbg_audioplayer_wav_bps   = 0;    /**< 파싱된 WAV bits per sample */

/* 블록 데이터 진단 — 읽기 직후 첫 8 stereo sample */
volatile int16_t  g_dbg_wav_block_L[8] = {0};   /**< Left channel first 8 samples */
volatile int16_t  g_dbg_wav_block_R[8] = {0};   /**< Right channel first 8 samples */
volatile uint32_t g_dbg_wav_bytes_read_total = 0; /**< 누적 읽은 바이트 */
volatile int32_t  g_dbg_wav_boundary_diff = 0;    /**< 현재 block_first - 이전 block_last */
volatile int32_t  g_dbg_wav_boundary_max  = 0;    /**< boundary diff 최대값 */
volatile int32_t  g_dbg_wav_boundary_min  = 0;    /**< boundary diff 최소값 */
volatile uint32_t g_dbg_wav_boundary_count = 0;   /**< 블록 경계 횟수 */
static int16_t    s_prev_block_last = 0;          /**< 이전 블록 마지막 sample */
volatile uint32_t g_dbg_audioplayer_wav_bytes = 0;    /**< WAV data chunk 크기 (바이트) */
volatile uint32_t g_dbg_audioplayer_prefill   = 0;    /**< Pre-fill 블록 수 */

/* 블록 단위 진단 */
volatile uint32_t g_dbg_audioplayer_short_reads = 0;  /**< bytes_got < bytes_to_read 횟수 (EOF 아닌) */
volatile uint32_t g_dbg_audioplayer_last_bytes_got = 0;  /**< 마지막 SD read 바이트 수 */
volatile int16_t  g_dbg_audioplayer_block_first = 0;  /**< 마지막 블록 첫 샘플 (연속성 확인) */
volatile int16_t  g_dbg_audioplayer_block_last  = 0;  /**< 마지막 블록 마지막 샘플 */

/**
 *-----------------------------------------------------------
 * QUEUE HELPERS (SPSC lock-free)
 *-----------------------------------------------------------
 */

static inline bool _q_full(void)
{
    return ((s_q_wr + 1) % Q_DEPTH) == s_q_rd;
}

static inline bool _q_empty(void)
{
    return s_q_wr == s_q_rd;
}

static inline uint32_t _q_count(void)
{
    return (s_q_wr + Q_DEPTH - s_q_rd) % Q_DEPTH;
}

static inline IOIF_SAI_AudioSample_t* _q_wr_block(void)
{
    return &s_q_buf[s_q_wr * Q_BLOCK_SAMPLES];
}

static inline IOIF_SAI_AudioSample_t* _q_rd_block(void)
{
    return &s_q_buf[s_q_rd * Q_BLOCK_SAMPLES];
}

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static int32_t _ParseWavHeader(void);
static bool    _FillQueueBlock(IOIF_SAI_AudioSample_t* block, uint32_t samples);
static void    _ApplyFadeOut(IOIF_SAI_AudioSample_t* block, uint32_t valid_samples);
static void    _SAI_Callback(IOIF_SAIx_t id, IOIF_SAI_Event_e event, void* ctx);
static void    _ReaderTask(void* arg);

/**
 *-----------------------------------------------------------
 * PUBLIC API
 *-----------------------------------------------------------
 */

AGRBStatusDef IOIF_SAI_AudioPlayer_Init(const IOIF_SAI_AudioPlayer_Config_t* config)
{
    if (config == NULL) return AGRBStatus_PARAM_ERROR;
    if (s_initialized)  return AGRBStatus_ALREADY_INITIALIZED;

    s_sai_id      = config->sai_id;
    s_playing     = false;
    s_q_wr        = 0;
    s_q_rd        = 0;
    s_reader_stop = false;

    /* Queue space semaphore (binary) */
    s_q_space_sem = xSemaphoreCreateBinary();
    if (s_q_space_sem == NULL) return AGRBStatus_SEMAPHORE_ERROR;

    /* WAV Reader Task — 생성 후 TaskNotify 대기 (suspend 상태) */
    BaseType_t ret = xTaskCreate(
        _ReaderTask,
        "WAV_Rd",
        IOIF_SAI_AUDIOPLAYER_TASK_STACK,
        NULL,
        IOIF_SAI_AUDIOPLAYER_TASK_PRIORITY,
        &s_reader_task);

    if (ret != pdPASS) {
        vSemaphoreDelete(s_q_space_sem);
        s_q_space_sem = NULL;
        return AGRBStatus_NOT_INITIALIZED;
    }

    /* Silence 버퍼 초기화 (EOF/Underrun 시 사용) */
    memset(s_xfer_buf, 0, Q_BLOCK_BYTES);

    s_initialized = true;
    g_dbg_audioplayer_ret = 0;
    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_AudioPlayer_RegisterDoneCallback(
    IOIF_SAI_AudioPlayer_DoneCallback_t cb, void* ctx)
{
    s_done_cb  = cb;
    s_done_ctx = ctx;
    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_AudioPlayer_Play(const char* filepath)
{
    if (!s_initialized) { g_dbg_audioplayer_ret = -1; return AGRBStatus_NOT_INITIALIZED; }
    if (filepath == NULL) { g_dbg_audioplayer_ret = -2; return AGRBStatus_PARAM_ERROR; }

    /* 기존 재생 정지 */
    if (s_playing) {
        IOIF_SAI_AudioPlayer_Stop();
    }

    /* SD카드 마운트 확인 */
    if (!AGRBFileSystem.is_ready()) {
        g_dbg_audioplayer_ret = -3;
        return AGRBStatus_NOT_INITIALIZED;
    }

    /* WAV 파일 열기 */
    FRESULT fr;
    AGRBFileSystemStatusDef fres = AGRBFileSystem.open(
        &s_wav.fid, filepath,
        IOIF_FileSystem_AccessMode_READONLY, &fr);
    if (fres != IOIF_FileSystem_OK) {
        g_dbg_audioplayer_ret = -4;
        return AGRBStatus_ERROR;
    }

    /* WAV 헤더 파싱 */
    int32_t parse_ret = _ParseWavHeader();
    if (parse_ret < 0) {
        AGRBFileSystem.close(s_wav.fid, &fr);
        g_dbg_audioplayer_ret = parse_ret;
        return AGRBStatus_PARAM_ERROR;
    }

    /* Queue 초기화 */
    s_q_wr = 0;
    s_q_rd = 0;
    s_reader_stop = false;
    g_dbg_audioplayer_underruns = 0;
    g_dbg_audioplayer_cb_count  = 0;

    /* WAV 포맷 디버그 기록 */
    g_dbg_audioplayer_wav_sr    = s_wav.sample_rate;
    g_dbg_audioplayer_wav_ch    = s_wav.num_channels;
    g_dbg_audioplayer_wav_bps   = s_wav.bits_per_sample;
    g_dbg_audioplayer_wav_bytes = s_wav.data_size;

    /* Pre-fill: Queue를 가능한 가득 채움 (DEPTH-1 = 3블록) */
    uint32_t prefill_count = 0;
    for (uint32_t i = 0; i < (Q_DEPTH - 1); i++) {
        _FillQueueBlock(_q_wr_block(), Q_BLOCK_SAMPLES);
        s_q_wr = (s_q_wr + 1) % Q_DEPTH;
        prefill_count++;
        if (s_wav.eof_reached) break;
    }
    g_dbg_audioplayer_prefill = prefill_count;

    /* SAI 콜백 등록 (AudioPlayer가 DMA 콜백 소유) */
    IOIF_SAI_RegisterCallback(s_sai_id, _SAI_Callback, NULL);

    /* 초기 DMA 전송: 양쪽 half 모두 pre-fill (artificial HALF_COMPLETE race 방지)
     * 1st block → DMA 1st half (IDLE path), 2nd block → DMA 2nd half (BUSY path) */
    AGRBStatusDef status = IOIF_SAI_Play(s_sai_id, _q_rd_block(), Q_BLOCK_BYTES);
    s_q_rd = (s_q_rd + 1) % Q_DEPTH;
    if (status != AGRBStatus_OK) {
        AGRBFileSystem.close(s_wav.fid, &fr);
        g_dbg_audioplayer_ret = -6;
        return status;
    }
    /* 2nd half pre-fill */
    if (!_q_empty()) {
        memcpy(s_xfer_buf, _q_rd_block(), Q_BLOCK_BYTES);
        IOIF_SAI_Play(s_sai_id, s_xfer_buf, Q_BLOCK_BYTES);
        s_q_rd = (s_q_rd + 1) % Q_DEPTH;
    }

    s_playing = true;

    /* Reader Task 깨우기 → 백그라운드에서 Queue 계속 채움 */
    xTaskNotifyGive(s_reader_task);

    g_dbg_audioplayer_ret = 0;
    return AGRBStatus_OK;
}

AGRBStatusDef IOIF_SAI_AudioPlayer_Stop(void)
{
    if (!s_initialized) return AGRBStatus_NOT_INITIALIZED;

    /* Reader Task 중단 신호 */
    s_reader_stop = true;
    if (s_q_space_sem) xSemaphoreGive(s_q_space_sem); /* full 대기 해제 */

    /* SAI DMA 정지 */
    IOIF_SAI_Stop(s_sai_id);

    /* WAV 파일 닫기 */
    if (s_wav.fid != IOIF_FILE_INVALID_ID) {
        FRESULT fr;
        AGRBFileSystem.close(s_wav.fid, &fr);
        s_wav.fid = IOIF_FILE_INVALID_ID;
    }

    s_playing = false;
    s_q_wr   = 0;
    s_q_rd   = 0;

    memset(&s_wav, 0, sizeof(s_wav));
    s_wav.fid = IOIF_FILE_INVALID_ID;  /* memset이 fid=0으로 초기화 → 이중 close 방지 */

    return AGRBStatus_OK;
}

bool IOIF_SAI_AudioPlayer_IsPlaying(void)
{
    return s_playing;
}

/**
 *-----------------------------------------------------------
 * SAI CALLBACK (Queue Consumer — DMA_CIRCULAR Double-Buffer, ST 표준)
 *-----------------------------------------------------------
 * DMA_CIRCULAR 모드: HALF_COMPLETE/COMPLETE 모두 처리.
 * - HALF_COMPLETE: DMA가 1st half 소비 → 1st half(inactive) 갱신
 * - COMPLETE: DMA가 2nd half 소비 → 2nd half(inactive) 갱신
 *
 * IOIF_SAI_Play(BUSY) 내부에서 sequence 카운터 기반으로
 * 올바른 half에 memcpy + 잔여 영역 zero-fill.
 *
 * DMA는 절대 멈추지 않으므로 블록 경계 gap/클릭 없음.
 * Queue 비면 silence(NULL) 전송 (글리치 대신 무음).
 */
static void _SAI_Callback(IOIF_SAIx_t id, IOIF_SAI_Event_e event, void* ctx)
{
    (void)id;
    (void)ctx;

    switch (event) {
        case IOIF_SAI_EVENT_HALF_COMPLETE:
        case IOIF_SAI_EVENT_COMPLETE:
        {
            /* Guard: Stop 후 stale notification 무시 */
            if (!s_playing) break;

            g_dbg_audioplayer_cb_count++;

            /* inject==2: Queue 완전 우회, 콜백에서 직접 440Hz sine 생성
             * Beep과 동일 패턴 (데이터 생성 → IOIF_SAI_Play) */
            if (g_dbg_audioplayer_sine_inject == 2) {
                static const int16_t lut[64] = {
                        0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
                     6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
                    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
                    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
                    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
                    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
                    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
                    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
                };
                static uint32_t cb_phase = 0;
                static uint32_t cb_samples_played = 0;
                const uint32_t inject2_total = WAVPLAYER_SAMPLE_RATE * 2; /* 2초 자동 정지 */
                const uint32_t inc = (uint32_t)((uint64_t)440 * 256 * 65536 / WAVPLAYER_SAMPLE_RATE);

                if (cb_samples_played >= inject2_total) {
                    /* 자동 정지 — silence 전송 후 Stop */
                    IOIF_SAI_Play(s_sai_id, NULL, 0);
                    cb_phase = 0;
                    cb_samples_played = 0;
                    IOIF_SAI_AudioPlayer_Stop();
                    if (s_done_cb) { s_done_cb(s_done_ctx); }
                    break;
                }

                for (uint32_t i = 0; i < Q_BLOCK_SAMPLES; i++) {
                    uint8_t idx = (uint8_t)((cb_phase >> 16) & 0xFF);
                    int16_t v;
                    if (idx < 64)       v = lut[idx];
                    else if (idx < 128) v = lut[127 - idx];
                    else if (idx < 192) v = (int16_t)(-lut[idx - 128]);
                    else                v = (int16_t)(-lut[255 - idx]);
                    v = (int16_t)(v / 4);
                    s_xfer_buf[i].left  = (uint16_t)v;
                    s_xfer_buf[i].right = (uint16_t)v;
                    cb_phase += inc;
                }
                cb_samples_played += Q_BLOCK_SAMPLES;
                IOIF_SAI_Play(s_sai_id, s_xfer_buf, Q_BLOCK_BYTES);
                break;
            }

            AGRBStatusDef ret;

            if (!_q_empty()) {
                memcpy(s_xfer_buf, _q_rd_block(), Q_BLOCK_BYTES);
                ret = IOIF_SAI_Play(s_sai_id, s_xfer_buf, Q_BLOCK_BYTES);
                s_q_rd = (s_q_rd + 1) % Q_DEPTH;

                if (s_q_space_sem) {
                    xSemaphoreGive(s_q_space_sem);
                }
            } else if (s_wav.eof_reached) {
                /* EOF + Queue drain 완료 → silence */
                ret = IOIF_SAI_Play(s_sai_id, NULL, 0);
                if (s_q_space_sem) {
                    xSemaphoreGive(s_q_space_sem);
                }
            } else {
                /* 실제 Underrun — silence */
                ret = IOIF_SAI_Play(s_sai_id, NULL, 0);
                g_dbg_audioplayer_underruns++;
            }

            g_dbg_audioplayer_play_ret = (int32_t)ret;
            g_dbg_audioplayer_q_level = (uint8_t)_q_count();
            break;
        }

        case IOIF_SAI_EVENT_ERROR:
            s_playing = false;
            break;

        default:
            break;
    }
}

/**
 *-----------------------------------------------------------
 * WAV READER TASK (Queue Producer)
 *-----------------------------------------------------------
 * SD카드에서 WAV 데이터를 읽어 Ring Queue에 채움.
 * Queue full이면 세마포어 대기 (SAI 콜백이 소비 후 해제).
 * EOF 도달 시 Queue 드레인 후 Done 콜백 호출.
 */
static void _ReaderTask(void* arg)
{
    (void)arg;

    while (1) {
        /* Play 호출까지 대기 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* SD → Queue 채움 루프 */
        while (!s_reader_stop && !s_wav.eof_reached) {
            /* Queue full이면 대기 */
            while (_q_full() && !s_reader_stop) {
                xSemaphoreTake(s_q_space_sem, pdMS_TO_TICKS(50));
            }
            if (s_reader_stop) break;

            /* 1블록 읽기: SD → mono→stereo 확장 → Queue */
            IOIF_SAI_AudioSample_t* block = _q_wr_block();
            if (!_FillQueueBlock(block, Q_BLOCK_SAMPLES)) {
                s_wav.eof_reached = true;
            }

            /* Write pointer 전진 (Consumer에게 공개) */
            s_q_wr = (s_q_wr + 1) % Q_DEPTH;
        }

        /* EOF: Queue 드레인 대기 — 콜백이 마지막 블록 소비 후 세마포어 줌 */
        if (!s_reader_stop) {
            while (!_q_empty() && !s_reader_stop) {
                xSemaphoreTake(s_q_space_sem, pdMS_TO_TICKS(100));
            }

            /* 자연 종료 — Stop + Done 콜백 */
            IOIF_SAI_AudioPlayer_Stop();

            if (s_done_cb) {
                s_done_cb(s_done_ctx);
            }
        }
        /* s_reader_stop=true이면 Stop()에서 이미 처리됨 */
    }
}

/**
 *-----------------------------------------------------------
 * WAV PARSER
 *-----------------------------------------------------------
 * RIFF/WAVE 헤더 파싱 + "data" 청크 탐색.
 * LIST/INFO 등 메타 청크가 있어도 스킵하여 "data" 위치를 찾음.
 */
static int32_t _ParseWavHeader(void)
{
    uint8_t hdr[WAV_FMT_HEADER_SIZE];
    uint32_t bytes_read = 0;
    FRESULT fr;

    /* 1. RIFF + fmt 청크 읽기 (처음 36바이트) */
    AGRBFileSystemStatusDef fres = AGRBFileSystem.read(
        s_wav.fid, hdr, WAV_FMT_HEADER_SIZE, &bytes_read, &fr);
    if (fres != IOIF_FileSystem_OK || bytes_read < WAV_FMT_HEADER_SIZE) {
        return -10;
    }

    /* Validate "RIFF" */
    if (hdr[0] != 'R' || hdr[1] != 'I' || hdr[2] != 'F' || hdr[3] != 'F') {
        return -11;
    }

    /* Validate "WAVE" */
    if (hdr[8] != 'W' || hdr[9] != 'A' || hdr[10] != 'V' || hdr[11] != 'E') {
        return -12;
    }

    /* AudioFormat must be 1 (PCM) */
    uint16_t audio_format = (uint16_t)(hdr[20] | (hdr[21] << 8));
    if (audio_format != 1) {
        return -13;
    }

    s_wav.num_channels    = (uint16_t)(hdr[22] | (hdr[23] << 8));
    s_wav.sample_rate     = (uint32_t)(hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24));
    s_wav.bits_per_sample = (uint16_t)(hdr[34] | (hdr[35] << 8));

    if (s_wav.bits_per_sample != 16) {
        return -14;
    }

    if (s_wav.num_channels != 1 && s_wav.num_channels != 2) {
        return -15;
    }

    /* 2. fmt subchunk 크기 파싱 → 정확한 다음 청크 위치 계산
     * 표준 PCM: fmtSize=16 → 다음 청크는 offset 36 (12+8+16)
     * 확장 PCM: fmtSize=18/40 → 일부 DAW가 생성 (cbSize 등 추가 필드)
     * hdr[16..19] = fmt subchunk size (little-endian) */
    uint32_t fmt_size = (uint32_t)(hdr[16] | (hdr[17] << 8) |
                                    (hdr[18] << 16) | (hdr[19] << 24));
    /* file_pos = RIFF(4) + size(4) + WAVE(4) + "fmt "(4) + fmtSize(4) + fmtData
     *          = 12 + 8 + fmt_size = 20 + fmt_size */
    uint32_t file_pos = 20 + fmt_size;

    /* fmt_size > 16인 경우 미읽은 바이트를 seek으로 건너뜀 */
    if (fmt_size > 16) {
        fres = AGRBFileSystem.seek(s_wav.fid, file_pos, &fr);
        if (fres != IOIF_FileSystem_OK) {
            return -19;
        }
    }

    /* 3. "data" 청크 탐색 — LIST/INFO 등 메타 청크 스킵 */
    uint8_t chunk_hdr[8];

    while (file_pos < WAV_MAX_HEADER_SEARCH) {
        bytes_read = 0;
        fres = AGRBFileSystem.read(s_wav.fid, chunk_hdr, 8, &bytes_read, &fr);
        if (fres != IOIF_FileSystem_OK || bytes_read < 8) {
            return -16;
        }
        file_pos += 8;

        uint32_t chunk_size = (uint32_t)(chunk_hdr[4] | (chunk_hdr[5] << 8) |
                                         (chunk_hdr[6] << 16) | (chunk_hdr[7] << 24));

        /* "data" 청크 발견 */
        if (chunk_hdr[0] == 'd' && chunk_hdr[1] == 'a' &&
            chunk_hdr[2] == 't' && chunk_hdr[3] == 'a') {
            s_wav.data_size   = chunk_size;
            s_wav.bytes_read  = 0;
            s_wav.eof_reached = false;
            return 0;
        }

        /* 다른 청크 스킵 (RIFF 청크는 2-byte aligned) */
        uint32_t skip = (chunk_size + 1) & ~1U;
        fres = AGRBFileSystem.seek(s_wav.fid, file_pos + skip, &fr);
        if (fres != IOIF_FileSystem_OK) {
            return -17;
        }
        file_pos += skip;
    }

    return -18;  /* "data" chunk not found */
}

/**
 * @brief EOF fade-out: 마지막 유효 샘플 구간에 linear ramp 적용 (data→zero 팝 방지)
 */
static void _ApplyFadeOut(IOIF_SAI_AudioSample_t* block, uint32_t valid_samples)
{
    if (valid_samples == 0) return;
    uint32_t fade_len = (valid_samples > FADE_OUT_SAMPLES) ? FADE_OUT_SAMPLES : valid_samples;
    uint32_t start = valid_samples - fade_len;
    for (uint32_t i = start; i < valid_samples; i++) {
        /* (valid_samples - 1 - i) → 마지막 샘플에서 정확히 0 도달
         * i=start: scale = (fade_len-1)/fade_len ≈ 98.4%
         * i=valid_samples-1: scale = 0/fade_len = 0 (완전 무음) */
        uint32_t remaining = valid_samples - 1 - i;
        block[i].left  = (uint16_t)(int16_t)((int32_t)(int16_t)block[i].left  * remaining / fade_len);
        block[i].right = (uint16_t)(int16_t)((int32_t)(int16_t)block[i].right * remaining / fade_len);
    }
}

/**
 *-----------------------------------------------------------
 * QUEUE BLOCK FILL (SD → Mono→Stereo → Queue Block)
 *-----------------------------------------------------------
 * @param block    대상 Queue 블록 포인터
 * @param samples  채울 스테레오 샘플 수
 * @return true: 데이터 있음, false: EOF
 */
static uint32_t s_sine_phase = 0;  /**< sine inject용 위상 누적 */

static bool _FillQueueBlock(IOIF_SAI_AudioSample_t* block, uint32_t samples)
{
    /* 디버그: sine wave 주입 모드 — SD 데이터 대신 440Hz sine 생성 (Queue 경유)
     * inject=1: Queue 경유 (Reader Task → Queue → Callback → DMA)
     * inject=2: Callback 직접 생성 (_SAI_Callback 내부, 별도 처리) */
    if (g_dbg_audioplayer_sine_inject == 1) {
        /* 64-entry sine LUT (Beep과 동일 품질) — peak ±8191 (25% of int16 range) */
        static const int16_t lut[64] = {
                0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
             6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
            12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
            18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
            23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
            27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
            30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
            32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
        };
        /* phase_inc: 440Hz × 256 / 44100 ≈ 2.56 → fixed-point 16.16 */
        const uint32_t phase_inc = (uint32_t)((uint64_t)440 * 256 * 65536 / WAVPLAYER_SAMPLE_RATE);
        for (uint32_t i = 0; i < samples; i++) {
            uint8_t idx = (uint8_t)((s_sine_phase >> 16) & 0xFF);
            int16_t val;
            if (idx < 64)       val = lut[idx];
            else if (idx < 128) val = lut[127 - idx];
            else if (idx < 192) val = (int16_t)(-lut[idx - 128]);
            else                val = (int16_t)(-lut[255 - idx]);
            val = (int16_t)(val / 4);  /* 25% amplitude */
            block[i].left  = (uint16_t)val;
            block[i].right = (uint16_t)val;
            s_sine_phase += phase_inc;
        }
        /* data_size를 크게 설정하여 계속 재생 */
        s_wav.bytes_read += samples * sizeof(IOIF_SAI_AudioSample_t);
        if (s_wav.bytes_read >= s_wav.data_size) {
            s_wav.eof_reached = true;
            return false;
        }
        return true;
    }

    if (s_wav.eof_reached) {
        memset(block, 0, samples * sizeof(IOIF_SAI_AudioSample_t));
        return false;
    }

    uint32_t remaining = s_wav.data_size - s_wav.bytes_read;
    if (remaining == 0) {
        s_wav.eof_reached = true;
        memset(block, 0, samples * sizeof(IOIF_SAI_AudioSample_t));
        return false;
    }

    FRESULT fr;

    g_dbg_wav_bytes_read_total = s_wav.bytes_read;

    if (s_wav.num_channels == 2) {
        /* Stereo: 직접 읽기 */
        uint32_t bytes_to_read = samples * sizeof(IOIF_SAI_AudioSample_t);
        if (bytes_to_read > remaining) bytes_to_read = remaining;

        uint32_t bytes_got = 0;
        AGRBFileSystem.read(s_wav.fid, (uint8_t*)block, bytes_to_read, &bytes_got, &fr);
        s_wav.bytes_read += bytes_got;

        uint32_t samples_got = bytes_got / sizeof(IOIF_SAI_AudioSample_t);
        if (samples_got < samples) {
            _ApplyFadeOut(block, samples_got);
            memset(&block[samples_got], 0,
                   (samples - samples_got) * sizeof(IOIF_SAI_AudioSample_t));
            s_wav.eof_reached = true;
        }
    } else {
        /* Mono → Stereo 확장 (backward copy로 in-place 변환) */
        uint32_t bytes_to_read = samples * sizeof(int16_t);
        if (bytes_to_read > remaining) bytes_to_read = remaining;

        int16_t* mono_buf = (int16_t*)block;
        uint32_t bytes_got = 0;
        AGRBFileSystem.read(s_wav.fid, (uint8_t*)mono_buf, bytes_to_read, &bytes_got, &fr);
        s_wav.bytes_read += bytes_got;
        g_dbg_audioplayer_last_bytes_got = bytes_got;

        uint32_t mono_samples = bytes_got / sizeof(int16_t);

        /* Short read 감지 (EOF 아닌 상황에서 부족한 읽기) */
        if (bytes_got < bytes_to_read && remaining > bytes_to_read) {
            g_dbg_audioplayer_short_reads++;
        }

        /* 변환 전 첫/마지막 mono 샘플 캡처 (연속성 진단) */
        if (mono_samples > 0) {
            g_dbg_audioplayer_block_first = mono_buf[0];
            g_dbg_audioplayer_block_last  = mono_buf[mono_samples - 1];
        }

        /* Backward expansion: mono → stereo (overwrite 방지) */
        for (int32_t i = (int32_t)mono_samples - 1; i >= 0; i--) {
            block[i].left  = (uint16_t)mono_buf[i];
            block[i].right = (uint16_t)mono_buf[i];
        }

        if (mono_samples < samples) {
            _ApplyFadeOut(block, mono_samples);
            memset(&block[mono_samples], 0,
                   (samples - mono_samples) * sizeof(IOIF_SAI_AudioSample_t));
            s_wav.eof_reached = true;
        }
    }

    /* 블록 데이터 캡처 + 경계 불연속 자동 계산 */
    for (int k = 0; k < 8 && k < (int)samples; k++) {
        g_dbg_wav_block_L[k] = (int16_t)block[k].left;
        g_dbg_wav_block_R[k] = (int16_t)block[k].right;
    }
    {
        int16_t cur_first = (int16_t)block[0].left;
        int16_t cur_last  = (samples > 0) ? (int16_t)block[samples - 1].left : 0;

        if (g_dbg_wav_boundary_count > 0) {
            int32_t diff = (int32_t)cur_first - (int32_t)s_prev_block_last;
            g_dbg_wav_boundary_diff = diff;
            if (diff > g_dbg_wav_boundary_max) g_dbg_wav_boundary_max = diff;
            if (diff < g_dbg_wav_boundary_min) g_dbg_wav_boundary_min = diff;
        }
        s_prev_block_last = cur_last;
        g_dbg_wav_boundary_count++;
    }

    /* inject=3: SD 읽기 후 데이터를 sine으로 교체
     * inject=4: SD 읽기 후 데이터의 L/R을 swap하여 재생 (데이터 무결성 확인) */
    if (g_dbg_audioplayer_sine_inject == 3) {
        static uint32_t inj3_phase = 0;
        const uint32_t inj3_inc = (uint32_t)((uint64_t)440 * 256 * 65536 / WAVPLAYER_SAMPLE_RATE);
        uint32_t count = s_wav.eof_reached ? 0 : samples;
        for (uint32_t i = 0; i < count; i++) {
            uint8_t idx = (uint8_t)((inj3_phase >> 16) & 0xFF);
            /* Full-scale sine: ±32000 (inject=1/2는 ±2048) */
            int16_t val = (int16_t)((idx < 128) ? (idx * 512 - 32000) : ((255 - idx) * 512 - 32000));
            block[i].left  = (uint16_t)val;
            block[i].right = (uint16_t)val;
            inj3_phase += inj3_inc;
        }
    } else if (g_dbg_audioplayer_sine_inject == 4) {
        /* inject=4: WAV 데이터 그대로 사용하되 매 sample에 0을 곱해 무음화
         * → 무음인데 파파팍 나면 데이터와 무관한 문제 */
    }

    return !s_wav.eof_reached;
}

#endif /* AGRB_IOIF_SAI_ENABLE && AGRB_IOIF_FILESYSTEM_ENABLE */
