/**
 ******************************************************************************
 * @file    test_pnp_identify.c
 * @brief   AGR PnP IDENTIFY 컴포넌트 + Pre-Op 직렬화 슬롯 호스트 유닛테스트
 * @date    2026-06-11
 *
 * @details
 * 빌드/실행: test/CMakeLists.txt (ctest --test-dir test/build)
 * 대상: PnP/agr_pnp_identify.c, PnP/agr_pnp_master.c (슬롯 API)
 *
 * 시나리오 T1~T14 — 설계: docs 및 fes_hub_drv IDENTIFY 통합 plan 참조.
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>

#include "agr_pnp_identify.h"
#include "agr_pnp_master.h"

/* ===== CHECK 매크로 (test_sdo_explicit_length.c 패턴) ===== */
static int g_total = 0;
static int g_fail  = 0;

#define CHECK(cond, msg) do {                                   \
    g_total++;                                                  \
    if (!(cond)) {                                              \
        g_fail++;                                               \
        printf("    [FAIL] %-58s (line %d)\n", (msg), __LINE__);\
    }                                                           \
} while (0)

/* ===== FES 기대 identity (실제 배포값과 동일) ===== */
static const AGR_Identify_Expected_t s_fes_expected = {
    .device_type  = 0x000B0000u,
    .vendor_id    = 0x414E474Cu,
    .product_code = 0x00040003u,
    .revision     = 0x00030000u,
};

/* ===== Fakes ===== */
static uint16_t g_last_read_idx = 0;
static uint8_t  g_last_read_sub = 0xFF;
static int      g_read_calls    = 0;

static int fake_send_read(void* user, uint16_t idx, uint8_t sub)
{
    (void)user;
    g_read_calls++;
    g_last_read_idx = idx;
    g_last_read_sub = sub;
    return 0;
}

static void fakes_reset(void)
{
    g_last_read_idx = 0;
    g_last_read_sub = 0xFF;
    g_read_calls    = 0;
}

static AGR_SDO_Msg_t mk_upload_rsp(uint16_t idx, uint8_t sub, uint32_t val)
{
    AGR_SDO_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.cs       = AGR_SDO_CS_UPLOAD_INIT_RSP_SIZED;  /* 0x41 */
    m.index    = idx;
    m.subindex = sub;
    m.data[0]  = (uint8_t)(val & 0xFFu);
    m.data[1]  = (uint8_t)((val >> 8) & 0xFFu);
    m.data[2]  = (uint8_t)((val >> 16) & 0xFFu);
    m.data[3]  = (uint8_t)((val >> 24) & 0xFFu);
    m.data_len = 4;
    return m;
}

static AGR_SDO_Msg_t mk_abort_rsp(uint16_t idx, uint8_t sub)
{
    AGR_SDO_Msg_t m;
    memset(&m, 0, sizeof(m));
    m.cs       = AGR_SDO_CS_ABORT;  /* 0x80 */
    m.index    = idx;
    m.subindex = sub;
    m.data_len = 4;
    return m;
}

/* 정상 4단계 응답 주입 헬퍼 (mismatch_at: -1=전부 일치, 0~3=해당 단계 오답) */
static AGR_Identify_Result_t run_sequence(AGR_Identify_Ctx_t* ctx, int mismatch_at)
{
    static const struct { uint16_t idx; uint8_t sub; } addr[4] = {
        { 0x1000, 0x00 }, { 0x1018, 0x01 }, { 0x1018, 0x02 }, { 0x1018, 0x03 },
    };
    const uint32_t good[4] = {
        s_fes_expected.device_type, s_fes_expected.vendor_id,
        s_fes_expected.product_code, s_fes_expected.revision,
    };

    AGR_Identify_Result_t r = AGR_Identify_Start(ctx, 1000);
    for (int s = 0; s < 4 && r == AGR_IDENTIFY_RESULT_IN_PROGRESS; s++) {
        uint32_t val = (s == mismatch_at) ? 0x00099999u : good[s];
        AGR_SDO_Msg_t rsp = mk_upload_rsp(addr[s].idx, addr[s].sub, val);
        r = AGR_Identify_OnSdoResponse(ctx, &rsp, 1000 + (uint32_t)s);
    }
    return r;
}

/* ===== PnP Master fakes (슬롯 테스트용) ===== */
static int fake_tx(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    (void)can_id; (void)data; (void)len;
    return 0;
}
static uint32_t fake_tick(void) { return 0; }

int main(void)
{
    AGR_Identify_Ctx_t ctx;

    /* ===== T1: 정상 PASS (4필드 모두 일치) ===== */
    printf("T1: all-match PASS\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    CHECK(AGR_Identify_GetResult(&ctx) == AGR_IDENTIFY_RESULT_IDLE, "init -> IDLE");
    AGR_Identify_Result_t r = run_sequence(&ctx, -1);
    CHECK(r == AGR_IDENTIFY_RESULT_PASS, "result PASS");
    CHECK(ctx.mismatch_count == 0, "no mismatch");
    CHECK(ctx.pass_count == 1, "pass_count 1");
    CHECK(g_read_calls == 4, "4 SDO reads sent");
    CHECK(ctx.actual_values[AGR_IDENTIFY_STEP_REVISION] == 0x00030000u, "revision stored");

    /* ===== T2~T5: 단계별 MISMATCH ===== */
    static const char* names[4] = { "device_type", "vendor_id", "product_code", "revision" };
    static const uint32_t exp_vals[4] = { 0x000B0000u, 0x414E474Cu, 0x00040003u, 0x00030000u };
    for (int s = 0; s < 4; s++) {
        printf("T%d: %s mismatch\n", 2 + s, names[s]);
        fakes_reset();
        AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                          AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
        r = run_sequence(&ctx, s);
        CHECK(r == AGR_IDENTIFY_RESULT_MISMATCH, "result MISMATCH");
        CHECK(ctx.last_mismatch.failed_step == (AGR_Identify_Step_t)s, "failed_step");
        CHECK(ctx.last_mismatch.expected == exp_vals[s], "expected recorded");
        CHECK(ctx.last_mismatch.actual == 0x00099999u, "actual recorded");
        CHECK(ctx.mismatch_count == 1, "mismatch_count 1");
    }

    /* ===== T6: 타임아웃 + 재시도 3회 소진 ===== */
    printf("T6: timeout after retries\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    r = AGR_Identify_Start(&ctx, 0);
    CHECK(r == AGR_IDENTIFY_RESULT_IN_PROGRESS, "in progress");
    CHECK(g_read_calls == 1, "first read sent");
    uint32_t t = 0;
    for (int retry = 1; retry <= 3; retry++) {
        t += 5001;
        r = AGR_Identify_RunPeriodic(&ctx, t);
        CHECK(r == AGR_IDENTIFY_RESULT_IN_PROGRESS, "retry keeps in-progress");
        CHECK(g_read_calls == 1 + retry, "resend on timeout");
    }
    t += 5001;
    r = AGR_Identify_RunPeriodic(&ctx, t);
    CHECK(r == AGR_IDENTIFY_RESULT_TIMEOUT, "retries exhausted -> TIMEOUT");
    CHECK(ctx.timeout_count == 1, "timeout_count 1");

    /* ===== T7: 2단계에서 SDO Abort 수신 ===== */
    printf("T7: abort at vendor step\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    (void)AGR_Identify_Start(&ctx, 0);
    AGR_SDO_Msg_t ok0 = mk_upload_rsp(0x1000, 0x00, s_fes_expected.device_type);
    (void)AGR_Identify_OnSdoResponse(&ctx, &ok0, 1);
    AGR_SDO_Msg_t ab = mk_abort_rsp(0x1018, 0x01);
    r = AGR_Identify_OnSdoResponse(&ctx, &ab, 2);
    CHECK(r == AGR_IDENTIFY_RESULT_ABORT, "abort -> ABORT");
    CHECK(ctx.abort_count == 1, "abort_count 1");

    /* ===== T8: WARN 모드 — revision 불일치여도 PASS ===== */
    printf("T8: WARN mode\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_WARN);
    r = run_sequence(&ctx, 3);
    CHECK(r == AGR_IDENTIFY_RESULT_PASS, "WARN -> PASS");
    CHECK(ctx.mismatch_count == 1, "mismatch recorded");
    CHECK(ctx.last_mismatch.failed_step == AGR_IDENTIFY_STEP_REVISION, "failed_step recorded");

    /* ===== T9: 부분 flags (vendor+product만) — device 오답이어도 미검사 ===== */
    printf("T9: partial flags\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_VENDOR_ID | AGR_IDENTIFY_FLAG_PRODUCT_CODE,
                      AGR_IDENTIFY_MODE_STRICT);
    r = AGR_Identify_Start(&ctx, 0);
    CHECK(g_last_read_idx == 0x1018 && g_last_read_sub == 0x01, "first read = vendor (device skipped)");
    AGR_SDO_Msg_t v = mk_upload_rsp(0x1018, 0x01, s_fes_expected.vendor_id);
    r = AGR_Identify_OnSdoResponse(&ctx, &v, 1);
    AGR_SDO_Msg_t p = mk_upload_rsp(0x1018, 0x02, s_fes_expected.product_code);
    r = AGR_Identify_OnSdoResponse(&ctx, &p, 2);
    CHECK(r == AGR_IDENTIFY_RESULT_PASS, "partial flags PASS");
    CHECK(g_read_calls == 2, "only 2 reads");

    /* ===== T10/T11: Pre-Op 직렬화 슬롯 ===== */
    printf("T10/T11: pre-op slot serialize\n");
    AGR_PnP_Master_t master;
    CHECK(AGR_PnP_Master_Init(&master, 0x02, fake_tx, fake_tick) == 0, "master init");
    CHECK(AGR_PnP_Master_GetPreOpSlotOwner(&master) == 0, "slot empty");
    CHECK(AGR_PnP_Master_TryAcquirePreOpSlot(&master, 0x0C) == true, "FES acquires");
    CHECK(AGR_PnP_Master_TryAcquirePreOpSlot(&master, 0x0D) == false, "IMU blocked");
    CHECK(AGR_PnP_Master_TryAcquirePreOpSlot(&master, 0x0C) == true, "re-acquire by owner ok");
    AGR_PnP_Master_ReleasePreOpSlot(&master, 0x0D);
    CHECK(AGR_PnP_Master_GetPreOpSlotOwner(&master) == 0x0C, "non-owner release ignored");
    AGR_PnP_Master_ReleasePreOpSlot(&master, 0x0C);
    CHECK(AGR_PnP_Master_GetPreOpSlotOwner(&master) == 0, "owner release clears");
    CHECK(AGR_PnP_Master_TryAcquirePreOpSlot(&master, 0x0D) == true, "next slave acquires");
    CHECK(AGR_PnP_Master_TryAcquirePreOpSlot(&master, 0x00) == false, "node 0 invalid");

    /* ===== T12: Reset 후 재시작 (Slave 재부팅 시나리오) ===== */
    printf("T12: reset & restart\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    (void)run_sequence(&ctx, -1);
    CHECK(ctx.result == AGR_IDENTIFY_RESULT_PASS, "first run PASS");
    AGR_Identify_Reset(&ctx);
    CHECK(AGR_Identify_GetResult(&ctx) == AGR_IDENTIFY_RESULT_IDLE, "reset -> IDLE");
    CHECK(ctx.actual_values[0] == 0, "actuals cleared");
    CHECK(ctx.pass_count == 1, "cumulative diag kept");
    r = run_sequence(&ctx, -1);
    CHECK(r == AGR_IDENTIFY_RESULT_PASS, "second run PASS");
    CHECK(ctx.pass_count == 2, "pass_count 2");

    /* ===== T13: 무관 SDO 응답 무시 (다른 트래픽과 공존) ===== */
    printf("T13: ignore unrelated response\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, &s_fes_expected,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    (void)AGR_Identify_Start(&ctx, 0);
    AGR_SDO_Msg_t other = mk_upload_rsp(0x7018, 0x00, 0xDEADBEEFu);
    r = AGR_Identify_OnSdoResponse(&ctx, &other, 1);
    CHECK(r == AGR_IDENTIFY_RESULT_IN_PROGRESS, "unrelated ignored");
    CHECK(ctx.current_step == AGR_IDENTIFY_STEP_DEVICE_TYPE, "step unchanged");
    AGR_SDO_Msg_t short_rsp = mk_upload_rsp(0x1000, 0x00, 0);
    short_rsp.data_len = 2;  /* malformed */
    r = AGR_Identify_OnSdoResponse(&ctx, &short_rsp, 2);
    CHECK(r == AGR_IDENTIFY_RESULT_IN_PROGRESS, "malformed ignored");

    /* ===== T14: expected=NULL — 검사 skip, 즉시 PASS (기존 모듈 무영향) ===== */
    printf("T14: NULL expected -> instant PASS\n");
    fakes_reset();
    AGR_Identify_Init(&ctx, fake_send_read, NULL, NULL,
                      AGR_IDENTIFY_FLAG_ALL, AGR_IDENTIFY_MODE_STRICT);
    r = AGR_Identify_Start(&ctx, 0);
    CHECK(r == AGR_IDENTIFY_RESULT_PASS, "instant PASS");
    CHECK(g_read_calls == 0, "no SDO sent");

    printf("\n%d checks, %d failed\n", g_total, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
