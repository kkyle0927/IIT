/**
 ******************************************************************************
 * @file    test_od_index.c
 * @author  HyundoKim
 * @brief   Host-side unit tests for AGR_OD_BuildSortedIndex / binary lookup.
 * @date    2026-06-11
 *
 * @details
 * OD 정렬 인덱스(이진탐색) 회귀 테스트. 핵심 보장:
 *   1) 인덱스 빌드 후 FindEntryEx 결과 == 선형 탐색 결과 (전수 parity)
 *   2) 미스(없는 key)도 양쪽 동일하게 NULL
 *   3) 중복 (index,subindex) 는 -3 fail loud + 선형 fallback 유지
 *   4) capacity 부족 / NULL 인자 에러 코드
 *   5) sorted_idx == NULL (미구축 모듈) 은 기존 선형 경로 그대로
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "agr_od.h"
#include "agr_dop_types.h"

/*------------------------------------------------------------
 * Minimal assert harness (no external framework)
 *------------------------------------------------------------*/
static int g_total = 0;
static int g_fail  = 0;

#define CHECK(cond, msg) do {                                   \
    g_total++;                                                  \
    if (!(cond)) {                                              \
        g_fail++;                                               \
        printf("    [FAIL] %-58s (line %d)\n", (msg), __LINE__);\
    }                                                           \
} while (0)

static void banner(const char* name) { printf("--- %s\n", name); }

/*------------------------------------------------------------
 * Fixture — 의도적으로 비정렬 + 섹션 혼재 (실제 fes_od_table 모사)
 *------------------------------------------------------------*/
static uint32_t s_dummy[16];

#define E(idx, sub, slot) \
    { (idx), (sub), AGR_TYPE_UINT32, 4, AGR_ACCESS_RW, &s_dummy[(slot)], NULL, NULL, NULL }

static const AGR_OD_Entry_t s_entries[] = {
    E(0x6300, 0x00, 0),
    E(0x1000, 0x00, 1),
    E(0x1018, 0x03, 2),
    E(0x1018, 0x01, 3),
    E(0x7018, 0x1C, 4),
    E(0x6055, 0x01, 5),
    E(0x6055, 0x00, 6),
    E(0x1018, 0x02, 7),
    E(0x6310, 0x00, 8),
    E(0xFFFF, 0xFF, 9),   /* 경계: 최대 key */
    E(0x0000, 0x00, 10),  /* 경계: 최소 key */
    E(0x6300, 0x06, 11),
};
#define N_ENTRIES (sizeof(s_entries) / sizeof(s_entries[0]))

/* 선형 레퍼런스 (수정 전 구현과 동일 의미) */
static const AGR_OD_Entry_t* linear_ref(const AGR_OD_Table_t* od,
                                        uint16_t index, uint8_t sub)
{
    for (uint16_t i = 0; i < od->entry_count; i++) {
        if (od->entries[i].index == index && od->entries[i].subindex == sub) {
            return &od->entries[i];
        }
    }
    return NULL;
}

/*============================================================
 * T1: 빌드 성공 + 전수 parity (모든 entry 히트)
 *============================================================*/
static void t1_build_and_full_parity(void)
{
    banner("T1 build + full hit parity");

    AGR_OD_Table_t od = { s_entries, N_ENTRIES, NULL };
    uint16_t idx_buf[N_ENTRIES];

    CHECK(AGR_OD_BuildSortedIndex(&od, idx_buf, N_ENTRIES) == 0, "build returns 0");
    CHECK(od.sorted_idx == idx_buf, "sorted_idx attached");

    for (uint16_t i = 0; i < N_ENTRIES; i++) {
        const AGR_OD_Entry_t* bin =
            AGR_OD_FindEntryEx(&od, s_entries[i].index, s_entries[i].subindex);
        const AGR_OD_Entry_t* lin =
            linear_ref(&od, s_entries[i].index, s_entries[i].subindex);
        CHECK(bin == lin && bin == &s_entries[i], "hit parity (binary == linear)");
    }
}

/*============================================================
 * T2: 미스 parity — 존재하지 않는 key들 양쪽 NULL
 *============================================================*/
static void t2_miss_parity(void)
{
    banner("T2 miss parity");

    AGR_OD_Table_t od = { s_entries, N_ENTRIES, NULL };
    uint16_t idx_buf[N_ENTRIES];
    CHECK(AGR_OD_BuildSortedIndex(&od, idx_buf, N_ENTRIES) == 0, "build ok");

    /* 이웃 key (존재 entry +-1 sub), 중간 빈 index, 경계 밖 */
    CHECK(AGR_OD_FindEntryEx(&od, 0x6300, 0x01) == NULL, "miss 0x6300:01");
    CHECK(AGR_OD_FindEntryEx(&od, 0x1018, 0x00) == NULL, "miss 0x1018:00");
    CHECK(AGR_OD_FindEntryEx(&od, 0x1018, 0x04) == NULL, "miss 0x1018:04");
    CHECK(AGR_OD_FindEntryEx(&od, 0x5000, 0x00) == NULL, "miss 0x5000:00");
    CHECK(AGR_OD_FindEntryEx(&od, 0xFFFF, 0xFE) == NULL, "miss 0xFFFF:FE");
    CHECK(AGR_OD_FindEntryEx(&od, 0x0000, 0x01) == NULL, "miss 0x0000:01");
}

/*============================================================
 * T3: 중복 key — -3 + sorted_idx 미연결 (선형 fallback 유지)
 *============================================================*/
static void t3_duplicate_fail_loud(void)
{
    banner("T3 duplicate key fail loud");

    static const AGR_OD_Entry_t dup_entries[] = {
        E(0x6000, 0x00, 12),
        E(0x6001, 0x00, 13),
        E(0x6000, 0x00, 14),   /* 중복 */
    };
    AGR_OD_Table_t od = { dup_entries, 3, NULL };
    uint16_t idx_buf[3];

    CHECK(AGR_OD_BuildSortedIndex(&od, idx_buf, 3) == -3, "build returns -3");
    CHECK(od.sorted_idx == NULL, "sorted_idx stays NULL");
    /* 선형 fallback — 기존 의미(첫 매칭) 유지 */
    CHECK(AGR_OD_FindEntryEx(&od, 0x6000, 0x00) == &dup_entries[0],
          "linear fallback first-match");
}

/*============================================================
 * T4: 인자 에러 — NULL / capacity 부족
 *============================================================*/
static void t4_arg_errors(void)
{
    banner("T4 argument errors");

    AGR_OD_Table_t od = { s_entries, N_ENTRIES, NULL };
    uint16_t idx_buf[N_ENTRIES];

    CHECK(AGR_OD_BuildSortedIndex(NULL, idx_buf, N_ENTRIES) == -1, "NULL od");
    CHECK(AGR_OD_BuildSortedIndex(&od, NULL, N_ENTRIES) == -1, "NULL buf");
    CHECK(AGR_OD_BuildSortedIndex(&od, idx_buf, N_ENTRIES - 1) == -2,
          "capacity short");
    CHECK(od.sorted_idx == NULL, "sorted_idx untouched on error");
}

/*============================================================
 * T5: 미구축 모듈 — 선형 경로 회귀 (zero-init 호환)
 *============================================================*/
static void t5_linear_fallback_unbuilt(void)
{
    banner("T5 unbuilt table = legacy linear");

    AGR_OD_Table_t od = { s_entries, N_ENTRIES, NULL };
    for (uint16_t i = 0; i < N_ENTRIES; i++) {
        CHECK(AGR_OD_FindEntryEx(&od, s_entries[i].index, s_entries[i].subindex)
                  == &s_entries[i],
              "linear hit");
    }
    CHECK(AGR_OD_FindEntryEx(&od, 0x5000, 0x00) == NULL, "linear miss");
}

/*============================================================
 * T6: 단일 entry 테이블
 *============================================================*/
static void t6_single_entry(void)
{
    banner("T6 single entry table");

    static const AGR_OD_Entry_t one[] = { E(0x1000, 0x00, 15) };
    AGR_OD_Table_t od = { one, 1, NULL };
    uint16_t idx_buf[1];

    CHECK(AGR_OD_BuildSortedIndex(&od, idx_buf, 1) == 0, "build ok");
    CHECK(AGR_OD_FindEntryEx(&od, 0x1000, 0x00) == &one[0], "hit");
    CHECK(AGR_OD_FindEntryEx(&od, 0x1000, 0x01) == NULL, "miss above");
    CHECK(AGR_OD_FindEntryEx(&od, 0x0FFF, 0xFF) == NULL, "miss below");
}

/*============================================================
 * main
 *============================================================*/
int main(void)
{
    printf("=== AGR_MW OD sorted-index unit tests ===\n");

    t1_build_and_full_parity();
    t2_miss_parity();
    t3_duplicate_fail_loud();
    t4_arg_errors();
    t5_linear_fallback_unbuilt();
    t6_single_entry();

    printf("=== %d checks, %d failed ===\n", g_total, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
