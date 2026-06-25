/**
 ******************************************************************************
 * @file    diag_perf.c
 * @author  HyundoKim
 * @brief   Step 0 진단 계측 구현
 *
 * @details
 * Plan v1.5 REV2_USB_TightSpin_InvestigationPlan 참조.
 * 핵심: DWT 접근만 쓴다. FreeRTOS API 호출 금지 (ISR 안전성).
 *       버퍼는 DTCM 에 두어 측정 왜곡 방지.
 ******************************************************************************
 */

#include "diag_perf.h"

#if DIAG_PERF_ACTIVE

#include <string.h>
#include <stdio.h>
#include "xm_api_memory.h"    /* XM_DTCM_VAR */
#include "ioif_agrb_dwt.h"    /* IOIF_DWT_GetCycles / CyclesToUs — Step A-1 ISR timing */

/**
 *-----------------------------------------------------------
 * PRIVATE CONSTANTS
 *-----------------------------------------------------------
 */

#define P99_TOP_N            100U  /**< Top-N tracking for p99 approximation */

/**
 * @brief Bin 상한값 — jitter_us < k_bin_edges[i] 이면 bin[i] 에 속함.
 *        마지막 bin [20] 은 overflow (k_bin_edges[19] 이상).
 */
static const uint32_t k_bin_edges[DIAG_PERF_N_BINS - 1U] = {
    200U,  400U,  600U,  800U,  1000U,
    2000U, 3000U, 4000U, 5000U, 6000U, 7000U, 8000U, 9000U, 10000U,
    11000U, 12000U, 13000U, 14000U, 15000U, 16000U
};

/**
 *-----------------------------------------------------------
 * PRIVATE STATE (DTCM 배치)
 *-----------------------------------------------------------
 */

XM_DTCM_VAR static uint32_t s_hist[DIAG_PERF_N_BINS];
XM_DTCM_VAR static uint32_t s_top[P99_TOP_N];  /* ascending sorted top values */
XM_DTCM_VAR static uint32_t s_top_count;
XM_DTCM_VAR static uint32_t s_sample_count;    /* 전체 RecordJitter 호출 수 */
XM_DTCM_VAR static uint32_t s_window_count;    /* 측정 윈도우 내 기록 샘플 수 */
XM_DTCM_VAR static uint32_t s_jitter_max;
XM_DTCM_VAR static uint8_t  s_finalized;

/* ---------- Step A-1: ISR breakdown state ---------- */

/**
 * @brief DiagIsrId_t → DiagIsrCat_t compile-time lookup.
 * @note  OTG_FS → USB, FDCAN1/2/TIM7 → FDCAN, DMA2_S0~4 → DMA, QUADSPI → OTHER.
 */
static const uint8_t k_isr_cat[DIAG_ISR_ID_COUNT] = {
    [DIAG_ISR_OTG_FS]      = DIAG_ISR_CAT_USB,
    [DIAG_ISR_FDCAN1_IT0]  = DIAG_ISR_CAT_FDCAN,
    [DIAG_ISR_FDCAN2_IT0]  = DIAG_ISR_CAT_FDCAN,
    [DIAG_ISR_TIM7]        = DIAG_ISR_CAT_FDCAN,
    [DIAG_ISR_DMA2_S0]     = DIAG_ISR_CAT_DMA,
    [DIAG_ISR_DMA2_S1]     = DIAG_ISR_CAT_DMA,
    [DIAG_ISR_DMA2_S2]     = DIAG_ISR_CAT_DMA,
    [DIAG_ISR_DMA2_S3]     = DIAG_ISR_CAT_DMA,
    [DIAG_ISR_DMA2_S4]     = DIAG_ISR_CAT_DMA,
    [DIAG_ISR_QUADSPI]     = DIAG_ISR_CAT_OTHER,
};

static const char* const k_isr_name[DIAG_ISR_ID_COUNT] = {
    [DIAG_ISR_OTG_FS]      = "otg_fs",
    [DIAG_ISR_FDCAN1_IT0]  = "fdcan1_it0",
    [DIAG_ISR_FDCAN2_IT0]  = "fdcan2_it0",
    [DIAG_ISR_TIM7]        = "tim7",
    [DIAG_ISR_DMA2_S0]     = "dma2_s0",
    [DIAG_ISR_DMA2_S1]     = "dma2_s1",
    [DIAG_ISR_DMA2_S2]     = "dma2_s2",
    [DIAG_ISR_DMA2_S3]     = "dma2_s3",
    [DIAG_ISR_DMA2_S4]     = "dma2_s4",
    [DIAG_ISR_QUADSPI]     = "quadspi",
};

static const char* const k_cat_name[DIAG_ISR_CAT_COUNT] = {
    [DIAG_ISR_CAT_USB]   = "usb",
    [DIAG_ISR_CAT_FDCAN] = "fdcan",
    [DIAG_ISR_CAT_DMA]   = "dma",
    [DIAG_ISR_CAT_OTHER] = "other",
};

/* Per-ISR accumulators (nesting-aware exclusive cycles).
 * NOTE: regular BSS (RAM_D1) 배치 — DTCM 은 NOLOAD 이므로 zero-init 불가.
 *       ISR 훅은 부팅 직후부터 동작해야 하므로 반드시 BSS zero-init 필요. */
static uint64_t s_isr_cycles[DIAG_ISR_ID_COUNT];
static uint32_t s_isr_count[DIAG_ISR_ID_COUNT];
static uint32_t s_isr_max_cycles[DIAG_ISR_ID_COUNT];       /* 관찰된 최대 exclusive */
static uint32_t s_isr_outliers[DIAG_ISR_ID_COUNT];         /* clamp 초과 건수 */
static uint64_t s_isr_outlier_cycles[DIAG_ISR_ID_COUNT];   /* clamp 초과 누적 (분리 bucket) */
static uint32_t s_isr_drops;             /* nesting depth overflow */
static uint32_t s_isr_nest_depth_max;    /* 관찰된 최대 nesting depth */

static uint32_t s_stack_enter[DIAG_PERF_ISR_NEST_MAX];
static uint32_t s_stack_subtract[DIAG_PERF_ISR_NEST_MAX];
static uint8_t  s_stack_id[DIAG_PERF_ISR_NEST_MAX];
static volatile uint32_t s_stack_depth;

/* ---------- Phase 2A: f_write size histogram state (mutex-protected in IOIF) ---------- */
XM_DTCM_VAR static uint32_t s_write_hist[DIAG_PERF_WRITE_N_BINS];
XM_DTCM_VAR static uint32_t s_write_count;
XM_DTCM_VAR static uint64_t s_write_total_bytes;
XM_DTCM_VAR static uint32_t s_write_max_size;

/** Phase 2A: f_write 크기 bin 상한. < edge[i] → bin[i]. Last bin = overflow. */
static const uint32_t k_write_bin_edges[DIAG_PERF_WRITE_N_BINS - 1U] = {
    5U,        /* ≤4B      — CRC tail write */
    65U,       /* ≤64B     — metadata / footer tiny */
    257U,      /* ≤256B    — headers */
    1025U,     /* ≤1KB     — metadata / summary */
    4097U,     /* ≤4KB     — CRC data block */
    16385U,    /* ≤16KB    — partial drain */
    32769U,    /* ≤32KB    — MAX_WRITE_CHUNK_SIZE */
};

static const char* const k_write_bin_labels[DIAG_PERF_WRITE_N_BINS] = {
    "write_bin0_le_4B",
    "write_bin1_le_64B",
    "write_bin2_le_256B",
    "write_bin3_le_1KB",
    "write_bin4_le_4KB",
    "write_bin5_le_16KB",
    "write_bin6_le_32KB",
    "write_bin7_gt_32KB",
};

/* ---------- [2026-04-18] OffloadTask stall state (Task context, mutex-free) ---------- */
XM_DTCM_VAR static uint32_t s_offload_iter_count;
XM_DTCM_VAR static uint32_t s_offload_wake_gap_max_us;
XM_DTCM_VAR static uint32_t s_offload_iter_dur_max_us;
XM_DTCM_VAR static uint32_t s_offload_wake_gap_hist[DIAG_PERF_OFFLOAD_N_BINS];
XM_DTCM_VAR static uint32_t s_offload_iter_dur_hist[DIAG_PERF_OFFLOAD_N_BINS];

static const uint32_t k_offload_bin_edges[DIAG_PERF_OFFLOAD_N_BINS - 1U] = {
    25000U, 50000U, 100000U, 500000U,
};

static const char* const k_offload_bin_labels[DIAG_PERF_OFFLOAD_N_BINS] = {
    "le_25ms", "le_50ms", "le_100ms", "le_500ms", "gt_500ms",
};

/**
 *-----------------------------------------------------------
 * PRIVATE HELPERS
 *-----------------------------------------------------------
 */

/** jitter_us 에 해당하는 bin index 반환 */
static uint32_t _bin_index(uint32_t jitter_us)
{
    for (uint32_t i = 0; i < (DIAG_PERF_N_BINS - 1U); i++) {
        if (jitter_us < k_bin_edges[i]) {
            return i;
        }
    }
    return DIAG_PERF_N_BINS - 1U;  /* overflow */
}

/**
 * @brief s_top 에 새 값 삽입 (ascending sort 유지).
 * @details Top-N 이 가득 차면 최소값(s_top[0]) 과 비교하여 대체.
 */
static void _top_insert(uint32_t v)
{
    if (s_top_count < P99_TOP_N) {
        /* Insertion sort ascending — O(N), N<=100 이라 문제 없음 */
        int32_t i = (int32_t)s_top_count - 1;
        while (i >= 0 && s_top[i] > v) {
            s_top[i + 1] = s_top[i];
            i--;
        }
        s_top[i + 1] = v;
        s_top_count++;
    } else if (v > s_top[0]) {
        /* 최소값 교체 + 적절한 위치로 이동 */
        int32_t i = 0;
        while (i < (int32_t)(P99_TOP_N - 1U) && s_top[i + 1] < v) {
            s_top[i] = s_top[i + 1];
            i++;
        }
        s_top[i] = v;
    }
}

/**
 * @brief window 기반 p99 근사값 계산.
 * @details s_top 은 세션 전체 top-N 값. p99 = window_count * 0.01 번째 top.
 *          window_count >= 10000 이면 top-100 = top 1% → s_top[0] 이 p99.
 */
static uint32_t _compute_p99(void)
{
    if (s_top_count == 0U) return 0U;

    uint32_t n_top_1pct = s_window_count / 100U;
    if (n_top_1pct == 0U) {
        /* window_count < 100 → 정상 분석 불가. s_top[0] (가용 최저) 반환 */
        return s_top[0];
    }
    if (n_top_1pct >= s_top_count) {
        return s_top[0];
    }
    /* p99 = (window * 0.99) 번째 ascending = top (window * 0.01) 중 최저 */
    return s_top[s_top_count - n_top_1pct];
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

void DiagPerf_Reset(void)
{
    memset(s_hist, 0, sizeof(s_hist));
    memset(s_top, 0, sizeof(s_top));
    s_top_count = 0U;
    s_sample_count = 0U;
    s_window_count = 0U;
    s_jitter_max = 0U;
    s_finalized = 0U;

    /* Step A-1 ISR counters */
    memset(s_isr_cycles, 0, sizeof(s_isr_cycles));
    memset(s_isr_count, 0, sizeof(s_isr_count));
    memset(s_isr_max_cycles, 0, sizeof(s_isr_max_cycles));
    memset(s_isr_outliers, 0, sizeof(s_isr_outliers));
    memset(s_isr_outlier_cycles, 0, sizeof(s_isr_outlier_cycles));
    memset(s_stack_enter, 0, sizeof(s_stack_enter));
    memset(s_stack_subtract, 0, sizeof(s_stack_subtract));
    memset(s_stack_id, 0, sizeof(s_stack_id));
    s_stack_depth = 0U;
    s_isr_drops = 0U;
    s_isr_nest_depth_max = 0U;

    /* Phase 2A write histogram */
    memset(s_write_hist, 0, sizeof(s_write_hist));
    s_write_count = 0U;
    s_write_total_bytes = 0U;
    s_write_max_size = 0U;

    /* [2026-04-18] OffloadTask stall state */
    s_offload_iter_count = 0U;
    s_offload_wake_gap_max_us = 0U;
    s_offload_iter_dur_max_us = 0U;
    memset(s_offload_wake_gap_hist, 0, sizeof(s_offload_wake_gap_hist));
    memset(s_offload_iter_dur_hist, 0, sizeof(s_offload_iter_dur_hist));
}

/**
 * @brief Step A-1 — ISR 진입 훅.
 * @details ISR context 전용. Nesting stack 에 enter cycle / id 기록.
 *          Cortex-M preemption semantics 상 stack 조작은 race-free
 *          (lower-priority ISR 은 higher-priority ISR 을 preempt 할 수 없음).
 */
void DiagPerf_EnterISR(DiagIsrId_t id)
{
    uint32_t d = s_stack_depth;
    if (d >= DIAG_PERF_ISR_NEST_MAX) {
        s_isr_drops++;
        return;
    }
    s_stack_id[d] = (uint8_t)id;
    s_stack_enter[d] = IOIF_DWT_GetCycles();
    s_stack_subtract[d] = 0U;
    s_stack_depth = d + 1U;
    if (d + 1U > s_isr_nest_depth_max) s_isr_nest_depth_max = d + 1U;
}

/**
 * @brief Step A-1 — ISR 종료 훅.
 * @details Exclusive cycles = elapsed - subtract. Parent frame 의 subtract 에
 *          inclusive elapsed 를 더해 parent 의 exclusive 계산에서 제거.
 */
void DiagPerf_ExitISR(void)
{
    uint32_t d = s_stack_depth;
    if (d == 0U) return;  /* Enter 드롭되었던 경우 */
    d--;
    uint32_t elapsed = IOIF_DWT_GetCycles() - s_stack_enter[d];
    uint32_t exclusive = (elapsed > s_stack_subtract[d]) ? (elapsed - s_stack_subtract[d]) : 0U;
    uint8_t id = s_stack_id[d];
    if (id < DIAG_ISR_ID_COUNT) {
        if (exclusive <= DIAG_PERF_ISR_CLAMP_CYCLES) {
            s_isr_cycles[id] += (uint64_t)exclusive;
            s_isr_count[id]++;
            if (exclusive > s_isr_max_cycles[id]) s_isr_max_cycles[id] = exclusive;
        } else {
            /* [2026-04-18 fix] Outlier — 5ms 초과는 stack 데싱크로 인한 stale
             * s_stack_enter[d] 판정 (elapsed = CYCCNT_now − stale_enter → uint32 wrap
             * arithmetic → ~8.95s (DWT wrap 주기) 의 pseudo-value). count 만 기록,
             * cycles 누적 금지 — 누적 시 보고서 오염 (152M us / 17 = 8.94s 평균 관측).
             * 근본 원인 (Enter/Exit 쌍 불일치) 은 ExitISR 이 ID 미전달이라 검증 불가 →
             * 추후 ExitISR(id) 시그니처 확장 + mismatch counter 추가 과제. */
            s_isr_outliers[id]++;
        }
    }
    if (d > 0U) {
        s_stack_subtract[d - 1U] += elapsed;
    }
    s_stack_depth = d;
}

void DiagPerf_RecordJitter(uint32_t jitter_us)
{
    if (s_finalized) return;

    s_sample_count++;

    /* 측정 윈도우 밖 — 첫 500 샘플은 metadata write 오염 가능, 제외 */
    if (s_sample_count < DIAG_PERF_WINDOW_EXCLUDE_START) {
        return;
    }

    uint32_t idx = _bin_index(jitter_us);
    s_hist[idx]++;
    s_window_count++;
    if (jitter_us > s_jitter_max) s_jitter_max = jitter_us;
    _top_insert(jitter_us);
}

void DiagPerf_Finalize(void)
{
    s_finalized = 1U;
}

void DiagPerf_RecordWrite(uint32_t size)
{
    if (s_finalized) return;
    if (size == 0U) return;

    uint32_t idx = DIAG_PERF_WRITE_N_BINS - 1U;  /* overflow default */
    for (uint32_t i = 0; i < (DIAG_PERF_WRITE_N_BINS - 1U); i++) {
        if (size < k_write_bin_edges[i]) { idx = i; break; }
    }
    s_write_hist[idx]++;
    s_write_count++;
    s_write_total_bytes += (uint64_t)size;
    if (size > s_write_max_size) s_write_max_size = size;
}

/** [2026-04-18] µs 값을 OFFLOAD bin index 로 변환 */
static uint32_t _offload_bin_index(uint32_t us)
{
    for (uint32_t i = 0; i < (DIAG_PERF_OFFLOAD_N_BINS - 1U); i++) {
        if (us < k_offload_bin_edges[i]) return i;
    }
    return DIAG_PERF_OFFLOAD_N_BINS - 1U;  /* overflow */
}

void DiagPerf_RecordOffloadIter(uint32_t wake_gap_us, uint32_t iter_dur_us)
{
    if (s_finalized) return;

    s_offload_iter_count++;

    if (wake_gap_us > 0U) {  /* 첫 iteration 은 skip */
        if (wake_gap_us > s_offload_wake_gap_max_us) s_offload_wake_gap_max_us = wake_gap_us;
        s_offload_wake_gap_hist[_offload_bin_index(wake_gap_us)]++;
    }
    if (iter_dur_us > s_offload_iter_dur_max_us) s_offload_iter_dur_max_us = iter_dur_us;
    s_offload_iter_dur_hist[_offload_bin_index(iter_dur_us)]++;
}

/**
 * @brief diag_profile.txt 포맷으로 통계 출력.
 * @note  Plan v1.4 8.1(b-1) 포맷 스펙 준수.
 */
int DiagPerf_FormatReport(char* buf, int buf_size)
{
    if (buf == NULL || buf_size <= 0) return 0;

    const char* build_flags =
#if defined(DIAG_KILL_DRAIN_WRITE)
        "PROFILE=1,KILL_DRAIN=1"
#else
        "PROFILE=1,KILL_DRAIN=0"
#endif
        ;

    static const char* const k_labels[DIAG_PERF_N_BINS] = {
        "hist_bin00_0_200us",
        "hist_bin01_200_400us",
        "hist_bin02_400_600us",
        "hist_bin03_600_800us",
        "hist_bin04_800_1000us",
        "hist_bin05_1_2ms",
        "hist_bin06_2_3ms",
        "hist_bin07_3_4ms",
        "hist_bin08_4_5ms",
        "hist_bin09_5_6ms",
        "hist_bin10_6_7ms",
        "hist_bin11_7_8ms",
        "hist_bin12_8_9ms",
        "hist_bin13_9_10ms",
        "hist_bin14_10_11ms",
        "hist_bin15_11_12ms",
        "hist_bin16_12_13ms",
        "hist_bin17_13_14ms",
        "hist_bin18_14_15ms",
        "hist_bin19_15_16ms",
        "hist_bin20_overflow",
    };

    int len = 0;
    len += snprintf(buf + len, (size_t)(buf_size - len),
        "=== Diag Profile ===\n"
        "build_flags=%s\n"
        "usertask_total_samples=%lu\n"
        "usertask_window_samples=%lu\n"
        "usertask_window_excl_start=%u\n"
        "usertask_max_us=%lu\n"
        "usertask_p99_us=%lu\n",
        build_flags,
        (unsigned long)s_sample_count,
        (unsigned long)s_window_count,
        (unsigned)DIAG_PERF_WINDOW_EXCLUDE_START,
        (unsigned long)s_jitter_max,
        (unsigned long)_compute_p99());

    for (uint32_t i = 0; i < DIAG_PERF_N_BINS; i++) {
        if (len >= buf_size) break;
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "usertask_%s=%lu\n",
            k_labels[i], (unsigned long)s_hist[i]);
    }

    /* ---------- Step A-1: ISR breakdown ---------- */
    if (len < buf_size) {
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "--- ISR Breakdown (exclusive, nesting-aware, clamp=5ms) ---\n"
            "--- NOTE: outlier_us 필드는 2026-04-18 이후 항상 0 (stack-desync stale "
            "enter 으로 인한 wrap arithmetic 누적 방지). outlier_count 만 유의미. ---\n"
            "isr_nest_drops=%lu\n"
            "isr_nest_depth_max=%lu\n",
            (unsigned long)s_isr_drops,
            (unsigned long)s_isr_nest_depth_max);
    }

    /* Category 집계 */
    uint64_t cat_cycles[DIAG_ISR_CAT_COUNT] = {0};
    uint32_t cat_count[DIAG_ISR_CAT_COUNT] = {0};
    for (uint32_t id = 0; id < DIAG_ISR_ID_COUNT; id++) {
        uint8_t cat = k_isr_cat[id];
        cat_cycles[cat] += s_isr_cycles[id];
        cat_count[cat]  += s_isr_count[id];
    }

    for (uint32_t c = 0; c < DIAG_ISR_CAT_COUNT; c++) {
        if (len >= buf_size) break;
        /* 64-bit cycles → us: 480 cycles/us 로 나눔 (60s 누적 시 32-bit overflow 대비) */
        uint64_t us64 = cat_cycles[c] / (uint64_t)(IOIF_CPU_FREQ_HZ / 1000000UL);
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "isr_cat_%s_count=%lu\n"
            "isr_cat_%s_us=%lu\n",
            k_cat_name[c], (unsigned long)cat_count[c],
            k_cat_name[c], (unsigned long)us64);
    }

    /* Per-ISR raw — count / us(clamped sum) / max(single ISR) / outliers */
    for (uint32_t id = 0; id < DIAG_ISR_ID_COUNT; id++) {
        if (len >= buf_size) break;
        uint64_t us64 = s_isr_cycles[id] / (uint64_t)(IOIF_CPU_FREQ_HZ / 1000000UL);
        uint32_t max_us = IOIF_DWT_CyclesToUs(s_isr_max_cycles[id]);
        uint64_t outlier_us64 = s_isr_outlier_cycles[id] / (uint64_t)(IOIF_CPU_FREQ_HZ / 1000000UL);
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "isr_%s_count=%lu\n"
            "isr_%s_us=%lu\n"
            "isr_%s_max_us=%lu\n"
            "isr_%s_outlier_count=%lu\n"
            "isr_%s_outlier_us=%lu\n",
            k_isr_name[id], (unsigned long)s_isr_count[id],
            k_isr_name[id], (unsigned long)us64,
            k_isr_name[id], (unsigned long)max_us,
            k_isr_name[id], (unsigned long)s_isr_outliers[id],
            k_isr_name[id], (unsigned long)outlier_us64);
    }

    /* ---------- Phase 2A: f_write size distribution ---------- */
    if (len < buf_size) {
        uint32_t avg = (s_write_count > 0U)
            ? (uint32_t)(s_write_total_bytes / (uint64_t)s_write_count) : 0U;
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "--- Write Size Distribution (ioif_filesystem_write) ---\n"
            "write_count=%lu\n"
            "write_total_bytes=%lu\n"
            "write_avg_bytes=%lu\n"
            "write_max_bytes=%lu\n",
            (unsigned long)s_write_count,
            (unsigned long)s_write_total_bytes,
            (unsigned long)avg,
            (unsigned long)s_write_max_size);
    }
    for (uint32_t i = 0; i < DIAG_PERF_WRITE_N_BINS; i++) {
        if (len >= buf_size) break;
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "%s=%lu\n",
            k_write_bin_labels[i], (unsigned long)s_write_hist[i]);
    }

    /* ---------- [2026-04-18] OffloadTask stall distribution ---------- */
    if (len < buf_size) {
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "--- OffloadTask Stall (wake_gap 정상=20ms, iter_dur 정상=수ms) ---\n"
            "offload_iter_count=%lu\n"
            "offload_wake_gap_max_us=%lu\n"
            "offload_iter_dur_max_us=%lu\n",
            (unsigned long)s_offload_iter_count,
            (unsigned long)s_offload_wake_gap_max_us,
            (unsigned long)s_offload_iter_dur_max_us);
    }
    for (uint32_t i = 0; i < DIAG_PERF_OFFLOAD_N_BINS; i++) {
        if (len >= buf_size) break;
        len += snprintf(buf + len, (size_t)(buf_size - len),
            "offload_wake_gap_%s=%lu\n"
            "offload_iter_dur_%s=%lu\n",
            k_offload_bin_labels[i], (unsigned long)s_offload_wake_gap_hist[i],
            k_offload_bin_labels[i], (unsigned long)s_offload_iter_dur_hist[i]);
    }

    return len;
}

#endif /* DIAG_PERF_ACTIVE */
