/**
 ******************************************************************************
 * @file    agr_pnp_identify.c
 * @author  HyundoKim
 * @brief   AGR PnP IDENTIFY — Slave identity 비교검증 상태머신 구현
 * @version 1.0.0
 * @date    2026-06-11
 *
 * @details
 * 순수 C / 비차단 / transport 무의존 (DOP V3 Core 철학). 설계 근거와
 * 구동 계약은 agr_pnp_identify.h 참조.
 *
 * > 본 설계는 DETERMINISTIC > DEFENSIVE 우선 — 검증은 Pre-Op 1회 수행,
 * > OPERATIONAL 중 재검증 없음 (제어 루프 jitter 회피).
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "agr_pnp_identify.h"
#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE — 단계 → OD 주소 / 기대값 / 플래그 매핑
 *-----------------------------------------------------------
 */

typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint8_t  flag;
} IdentifyStepDef_t;

static const IdentifyStepDef_t s_step_def[AGR_IDENTIFY_STEP_COUNT] = {
    [AGR_IDENTIFY_STEP_DEVICE_TYPE]  = { 0x1000, 0x00, AGR_IDENTIFY_FLAG_DEVICE_TYPE },
    [AGR_IDENTIFY_STEP_VENDOR_ID]    = { 0x1018, 0x01, AGR_IDENTIFY_FLAG_VENDOR_ID },
    [AGR_IDENTIFY_STEP_PRODUCT_CODE] = { 0x1018, 0x02, AGR_IDENTIFY_FLAG_PRODUCT_CODE },
    [AGR_IDENTIFY_STEP_REVISION]     = { 0x1018, 0x03, AGR_IDENTIFY_FLAG_REVISION },
};

static uint32_t _ExpectedOf(const AGR_Identify_Ctx_t* ctx, AGR_Identify_Step_t step)
{
    switch (step) {
        case AGR_IDENTIFY_STEP_DEVICE_TYPE:  return ctx->expected.device_type;
        case AGR_IDENTIFY_STEP_VENDOR_ID:    return ctx->expected.vendor_id;
        case AGR_IDENTIFY_STEP_PRODUCT_CODE: return ctx->expected.product_code;
        case AGR_IDENTIFY_STEP_REVISION:     return ctx->expected.revision;
        default:                             return 0;
    }
}

/** @brief 검사 플래그가 켜진 다음 단계로 전진 (없으면 STEP_COUNT 반환) */
static AGR_Identify_Step_t _NextCheckedStep(const AGR_Identify_Ctx_t* ctx,
                                            AGR_Identify_Step_t from)
{
    for (uint8_t s = (uint8_t)from; s < (uint8_t)AGR_IDENTIFY_STEP_COUNT; s++) {
        if ((ctx->check_flags & s_step_def[s].flag) != 0U) {
            return (AGR_Identify_Step_t)s;
        }
    }
    return AGR_IDENTIFY_STEP_COUNT;
}

/** @brief 현재 단계 SDO Read 송신 + 타이머 리셋 */
static void _SendCurrent(AGR_Identify_Ctx_t* ctx, uint32_t now_ms)
{
    const IdentifyStepDef_t* def = &s_step_def[ctx->current_step];
    ctx->step_start_ms = now_ms;
    /* 송신 실패는 무시 — RunPeriodic 타임아웃 경로가 재송신 (비차단) */
    (void)ctx->send_read(ctx->user, def->index, def->subindex);
}

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

void AGR_Identify_Init(AGR_Identify_Ctx_t* ctx,
                       AGR_Identify_SendReadFn_t send_read,
                       void* user,
                       const AGR_Identify_Expected_t* expected,
                       uint8_t check_flags,
                       AGR_Identify_Mode_t mode)
{
    if (ctx == NULL || send_read == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(AGR_Identify_Ctx_t));
    ctx->send_read   = send_read;
    ctx->user        = user;
    ctx->mode        = mode;
    ctx->timeout_ms  = AGR_IDENTIFY_TIMEOUT_MS_DEFAULT;
    ctx->max_retry   = AGR_IDENTIFY_MAX_RETRY_DEFAULT;
    ctx->result      = AGR_IDENTIFY_RESULT_IDLE;

    if (expected != NULL) {
        ctx->expected    = *expected;
        ctx->check_flags = (uint8_t)(check_flags & AGR_IDENTIFY_FLAG_ALL);
    } else {
        ctx->check_flags = 0U;  /* 기대값 없음 → 검사 skip (즉시 PASS) */
    }
}

AGR_Identify_Result_t AGR_Identify_Start(AGR_Identify_Ctx_t* ctx, uint32_t now_ms)
{
    if (ctx == NULL || ctx->send_read == NULL) {
        return AGR_IDENTIFY_RESULT_IDLE;
    }

    ctx->retry_count = 0;
    memset(ctx->actual_values, 0, sizeof(ctx->actual_values));

    AGR_Identify_Step_t first = _NextCheckedStep(ctx, AGR_IDENTIFY_STEP_DEVICE_TYPE);
    if (first >= AGR_IDENTIFY_STEP_COUNT) {
        /* 검사 항목 없음 (expected=NULL 또는 flags=0) — 기존 모듈 무영향 경로 */
        ctx->result = AGR_IDENTIFY_RESULT_PASS;
        ctx->pass_count++;
        return ctx->result;
    }

    ctx->current_step = first;
    ctx->result       = AGR_IDENTIFY_RESULT_IN_PROGRESS;
    _SendCurrent(ctx, now_ms);
    return ctx->result;
}

AGR_Identify_Result_t AGR_Identify_OnSdoResponse(AGR_Identify_Ctx_t* ctx,
                                                 const AGR_SDO_Msg_t* rsp,
                                                 uint32_t now_ms)
{
    if (ctx == NULL || rsp == NULL) {
        return AGR_IDENTIFY_RESULT_IDLE;
    }
    if (ctx->result != AGR_IDENTIFY_RESULT_IN_PROGRESS) {
        return ctx->result;
    }

    const IdentifyStepDef_t* def = &s_step_def[ctx->current_step];

    /* 현재 단계의 응답이 아니면 무시 (다른 SDO 트래픽과 공존) */
    if (rsp->index != def->index || rsp->subindex != def->subindex) {
        return ctx->result;
    }

    /* Slave가 명시 거부 (OD 미지원 등) — 재시도 무의미, 즉시 종료 */
    if (rsp->cs == AGR_SDO_CS_ABORT) {
        ctx->abort_count++;
        ctx->result = AGR_IDENTIFY_RESULT_ABORT;
        return ctx->result;
    }

    /* Upload Response (0x40/0x41 계열)만 처리 */
    if ((rsp->cs & 0xE0U) != 0x40U) {
        return ctx->result;
    }
    if (rsp->data_len < 4U) {
        return ctx->result;  /* malformed — 타임아웃 경로가 재시도 */
    }

    uint32_t actual = (uint32_t)rsp->data[0]
                    | ((uint32_t)rsp->data[1] << 8)
                    | ((uint32_t)rsp->data[2] << 16)
                    | ((uint32_t)rsp->data[3] << 24);
    ctx->actual_values[ctx->current_step] = actual;

    uint32_t expected = _ExpectedOf(ctx, ctx->current_step);
    if (actual != expected) {
        ctx->mismatch_count++;
        ctx->last_mismatch.failed_step = ctx->current_step;
        ctx->last_mismatch.expected    = expected;
        ctx->last_mismatch.actual      = actual;

        if (ctx->mode == AGR_IDENTIFY_MODE_STRICT) {
            ctx->result = AGR_IDENTIFY_RESULT_MISMATCH;
            return ctx->result;
        }
        /* WARN 모드: 기록만 하고 계속 진행 */
    }

    /* 다음 검사 단계로 전진 */
    AGR_Identify_Step_t next =
        _NextCheckedStep(ctx, (AGR_Identify_Step_t)((uint8_t)ctx->current_step + 1U));
    if (next >= AGR_IDENTIFY_STEP_COUNT) {
        ctx->result = AGR_IDENTIFY_RESULT_PASS;
        ctx->pass_count++;
        return ctx->result;
    }

    ctx->current_step = next;
    ctx->retry_count  = 0;
    _SendCurrent(ctx, now_ms);
    return ctx->result;
}

AGR_Identify_Result_t AGR_Identify_RunPeriodic(AGR_Identify_Ctx_t* ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return AGR_IDENTIFY_RESULT_IDLE;
    }
    if (ctx->result != AGR_IDENTIFY_RESULT_IN_PROGRESS) {
        return ctx->result;
    }

    if ((now_ms - ctx->step_start_ms) > ctx->timeout_ms) {
        if (ctx->retry_count < ctx->max_retry) {
            ctx->retry_count++;
            _SendCurrent(ctx, now_ms);  /* 동일 단계 재송신 */
        } else {
            ctx->timeout_count++;
            ctx->result = AGR_IDENTIFY_RESULT_TIMEOUT;
        }
    }
    return ctx->result;
}

AGR_Identify_Result_t AGR_Identify_GetResult(const AGR_Identify_Ctx_t* ctx)
{
    return (ctx != NULL) ? ctx->result : AGR_IDENTIFY_RESULT_IDLE;
}

void AGR_Identify_Reset(AGR_Identify_Ctx_t* ctx)
{
    if (ctx == NULL) {
        return;
    }
    ctx->result       = AGR_IDENTIFY_RESULT_IDLE;
    ctx->current_step = AGR_IDENTIFY_STEP_DEVICE_TYPE;
    ctx->retry_count  = 0;
    ctx->step_start_ms = 0;
    memset(ctx->actual_values, 0, sizeof(ctx->actual_values));
    /* 설정(expected/flags/mode/timeout)과 누적 진단 카운터는 유지 */
}
