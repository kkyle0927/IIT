/**
 ******************************************************************************
 * @file    hardfault_dump.c
 * @author  HyundoKim
 * @brief   HardFault dump — `.noinit` SRAM 영역에 fault context 캡처
 ******************************************************************************
 */

#include "hardfault_dump.h"
#include "stm32h7xx.h"  /* SCB, NVIC_SystemReset, SCB_CleanDCache_by_Addr */
#include <string.h>
#include <stdio.h>

/* FreeRTOS task name 접근용 (선택적) */
#if defined(__has_include)
#  if __has_include("FreeRTOS.h")
#    include "FreeRTOS.h"
#    include "task.h"
#    define HF_HAS_FREERTOS 1
#  endif
#endif
#ifndef HF_HAS_FREERTOS
#  define HF_HAS_FREERTOS 0
#endif

/* ─────────────────────────────────────────────────────────────
 * `.noinit` 영역 — warm reset 후에도 보존되는 덤프 저장소
 * ───────────────────────────────────────────────────────────── */
__attribute__((section(".noinit"), used))
static volatile HardFaultDump_t s_fault_dump;

/* 부팅 시 `.noinit` 에서 이관된 사본 — USB mount 이후 소비 */
static HardFaultDump_t s_boot_dump;
static bool            s_boot_dump_valid = false;


/* ─────────────────────────────────────────────────────────────
 * 내부 헬퍼
 * ───────────────────────────────────────────────────────────── */

static uint32_t _ComputeChecksum(const HardFaultDump_t *d)
{
    const uint32_t *p = (const uint32_t *)d;
    uint32_t acc = 0;
    /* magic_a, magic_b, checksum 자체를 제외하고 나머지 word XOR */
    const uint32_t n_words = sizeof(HardFaultDump_t) / 4U;
    for (uint32_t i = 3; i < n_words; ++i) {
        acc ^= p[i];
    }
    return acc ^ 0xA5A5A5A5U;  /* 전부-0 인 경우에도 checksum 은 non-zero */
}


/* ─────────────────────────────────────────────────────────────
 * Fault handler 경로 — naked dispatcher 가 호출
 *
 * 주의:
 *   - printf / HAL / FatFs / RTOS API 호출 금지 (재귀 fault)
 *   - 최소 stack frame 만 사용
 *   - 마지막에 SCB_CleanDCache + NVIC_SystemReset
 * ───────────────────────────────────────────────────────────── */
void HardFault_CaptureAndReset(uint32_t *stacked_sp,
                               uint32_t exc_return_val,
                               uint32_t exc_type)
{
    volatile HardFaultDump_t *d = &s_fault_dump;

    /* 전체 0 초기화 — 이전 덤프 잔재 제거 */
    for (uint32_t i = 0; i < sizeof(*d) / 4U; ++i) {
        ((volatile uint32_t *)d)[i] = 0U;
    }

    /* magic 을 가장 먼저 작성 + 즉시 cache flush — secondary fault 발생해도
     * "최소 fault 발생 사실 + exc_type" 은 다음 boot 가 인식 가능. fault-on-
     * fault 시 dump 가 random 값으로 남던 사례 (0xB08CB580 magic 검출 실패)
     * 회피. checksum 은 마지막에 재계산. */
    d->exc_type   = exc_type;
    d->magic_b    = HF_DUMP_MAGIC_B;
    __DMB();
    d->magic_a    = HF_DUMP_MAGIC_A;
    SCB_CleanDCache_by_Addr((uint32_t *)d, 16U);  /* magic_a/b + checksum(0) + exc_type */
    __DSB();

    d->exc_return = exc_return_val;
    d->msp        = __get_MSP();
    d->psp        = __get_PSP();
    d->ipsr       = __get_IPSR();

    /* 스택 프레임 8 word 복사 — stacked_sp 유효성 검증 강화.
     * Stack overflow 시 SP 가 가드 페이지 / unmapped 영역 가리킬 수 있음.
     * 그 영역 access 시 BusFault → fault-on-fault → CPU lockup → dump 미작성.
     * SRAM (D1: 0x24000000~0x2407FFFF, D2: 0x30000000~0x3004FFFF, D3: 0x38000000~
     * 0x38007FFF, DTCM: 0x20000000~0x2001FFFF) + 32B 정렬 + 8 word 안 끝나는지.
     */
    const uint32_t sp_addr = (uint32_t)stacked_sp;
    bool sp_valid =
        (stacked_sp != NULL) &&
        ((sp_addr & 0x3u) == 0u) &&  /* 4-byte align */
        (
            (sp_addr >= 0x20000000u && (sp_addr + 32u) <= 0x20020000u) ||  /* DTCM */
            (sp_addr >= 0x24000000u && (sp_addr + 32u) <= 0x24080000u) ||  /* RAM_D1 */
            (sp_addr >= 0x30000000u && (sp_addr + 32u) <= 0x30050000u) ||  /* RAM_D2 */
            (sp_addr >= 0x38000000u && (sp_addr + 32u) <= 0x38008000u)     /* RAM_D3 */
        );
    if (sp_valid) {
        d->r0   = stacked_sp[0];
        d->r1   = stacked_sp[1];
        d->r2   = stacked_sp[2];
        d->r3   = stacked_sp[3];
        d->r12  = stacked_sp[4];
        d->lr   = stacked_sp[5];
        d->pc   = stacked_sp[6];
        d->xpsr = stacked_sp[7];
    } else {
        /* SP invalid — stack overflow 또는 corruption 강한 의심. PC 자리에 marker.
         * sensor-studio / 외부 도구가 sp_addr 자체를 보고 fault 위치 추정 가능. */
        d->r0 = sp_addr;
        d->pc = 0xDEADBEEFu;
    }

    /* Fault status registers */
    d->cfsr  = SCB->CFSR;
    d->hfsr  = SCB->HFSR;
    d->mmfar = SCB->MMFAR;
    d->bfar  = SCB->BFAR;

    /* SysTick 의 uwTick 을 직접 읽음 — fault handler 에서 HAL wrapper 호출은 규칙 위반.
     * uwTick 은 uint32_t 단일 워드 읽기라 torn read 없음. */
    extern volatile uint32_t uwTick;
    d->uptime_ms = uwTick;

    /* Task name — thread mode 에서 PSP 사용 중이고 RTOS 가동 중일 때만 */
#if HF_HAS_FREERTOS
    {
        /* exc_return bit2 = 1 → PSP (thread), 0 → MSP (handler/boot) */
        const bool from_psp = (exc_return_val & 0x4U) != 0U;
        if (from_psp && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
            TaskHandle_t cur = xTaskGetCurrentTaskHandle();
            if (cur != NULL) {
                const char *name = pcTaskGetName(cur);
                if (name != NULL) {
                    uint32_t i = 0;
                    while (i < sizeof(d->task_name) - 1U && name[i] != '\0') {
                        d->task_name[i] = name[i];
                        ++i;
                    }
                    d->task_name[i] = '\0';
                }
            }
        }
    }
#endif

    /* Checksum 먼저 → magic 나중에 (tear 방지) */
    d->checksum = _ComputeChecksum((const HardFaultDump_t *)d);
    d->magic_b  = HF_DUMP_MAGIC_B;
    __DMB();
    d->magic_a  = HF_DUMP_MAGIC_A;

    /* ─── D-Cache writeback 필수 ───
     * RAM_D1 은 cacheable. Cache line 에만 있는 상태로 reset 되면
     * SRAM 실제 값은 이전 내용(또는 0) → 덤프 유실.
     * Clean = dirty line 을 SRAM 에 write-back (invalidate 는 미실시). */
    SCB_CleanDCache_by_Addr((uint32_t *)&s_fault_dump, sizeof(s_fault_dump));
    __DSB();
    __ISB();

    /* 재부팅 — NVIC_SystemReset 은 SRAM 보존 (POR/BOR 와 다름) */
    NVIC_SystemReset();

    /* 도달 불가 */
    for (;;) { __NOP(); }
}


/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

bool HardFault_LoadBootDump(void)
{
    HardFaultDump_t tmp;
    if (!HardFault_CheckPending(&tmp)) {
        s_boot_dump_valid = false;
        return false;
    }
    memcpy(&s_boot_dump, &tmp, sizeof(s_boot_dump));
    s_boot_dump_valid = true;
    return true;
}

bool HardFault_HasBootDump(void)
{
    return s_boot_dump_valid;
}

bool HardFault_TakeBootDump(HardFaultDump_t *out)
{
    if (out == NULL || !s_boot_dump_valid) {
        return false;
    }
    memcpy(out, &s_boot_dump, sizeof(*out));
    s_boot_dump_valid = false;
    /* 소비 완료 사본 0 초기화 — 민감 stack 내용 잔존 방지 */
    memset(&s_boot_dump, 0, sizeof(s_boot_dump));
    return true;
}


bool HardFault_CheckPending(HardFaultDump_t *out)
{
    if (out == NULL) {
        return false;
    }

    /* magic + checksum 이중 검증 */
    if (s_fault_dump.magic_a != HF_DUMP_MAGIC_A ||
        s_fault_dump.magic_b != HF_DUMP_MAGIC_B) {
        return false;
    }

    const uint32_t expect = _ComputeChecksum((const HardFaultDump_t *)&s_fault_dump);
    if (s_fault_dump.checksum != expect) {
        /* magic 은 맞지만 checksum 불일치 → 파손된 덤프, 무효화 */
        s_fault_dump.magic_a = 0;
        s_fault_dump.magic_b = 0;
        return false;
    }

    /* 유효 — 사본 생성 후 원본 magic clear (재소비 방지) */
    memcpy(out, (const void *)&s_fault_dump, sizeof(*out));
    s_fault_dump.magic_a = 0;
    s_fault_dump.magic_b = 0;
    return true;
}


uint32_t HardFault_FormatReport(const HardFaultDump_t *d,
                                char *buf, uint32_t bufsize)
{
    if (d == NULL || buf == NULL || bufsize == 0U) {
        return 0;
    }

    const char *exc_name;
    switch (d->exc_type) {
    case HF_EXC_HARDFAULT:  exc_name = "HardFault";   break;
    case HF_EXC_MEMMANAGE:  exc_name = "MemManage";   break;
    case HF_EXC_BUSFAULT:   exc_name = "BusFault";    break;
    case HF_EXC_USAGEFAULT: exc_name = "UsageFault";  break;
    case HF_EXC_NMI:        exc_name = "NMI";         break;
    default:                exc_name = "Unknown";     break;
    }

    const uint32_t cfsr  = d->cfsr;
    const uint32_t mmsr  = cfsr & 0xFFU;
    const uint32_t bfsr  = (cfsr >> 8) & 0xFFU;
    const uint32_t ufsr  = (cfsr >> 16) & 0xFFFFU;
    const bool mmar_valid = (mmsr & (1U << 7)) != 0U;
    const bool bfar_valid = (bfsr & (1U << 7)) != 0U;

    int n = snprintf(buf, bufsize,
        "== HardFault Dump ==\n"
        "Exception   : %s (type=%lu)\n"
        "Uptime (ms) : %lu\n"
        "Task        : %s\n"
        "\n"
        "-- Stacked registers --\n"
        "R0   = 0x%08lX   R1  = 0x%08lX\n"
        "R2   = 0x%08lX   R3  = 0x%08lX\n"
        "R12  = 0x%08lX   LR  = 0x%08lX\n"
        "PC   = 0x%08lX   xPSR= 0x%08lX\n"
        "\n"
        "-- Fault Status --\n"
        "CFSR = 0x%08lX   (MMSR=0x%02lX BFSR=0x%02lX UFSR=0x%04lX)\n"
        "HFSR = 0x%08lX\n"
        "MMFAR= 0x%08lX   %s\n"
        "BFAR = 0x%08lX   %s\n"
        "\n"
        "-- Context --\n"
        "EXC_RETURN = 0x%08lX  (%s mode)\n"
        "MSP  = 0x%08lX   PSP = 0x%08lX   IPSR=%lu\n"
        "\n"
        "-- Quick decode --\n"
        "%s%s%s%s%s%s%s%s%s%s",
        exc_name, (unsigned long)d->exc_type,
        (unsigned long)d->uptime_ms,
        d->task_name[0] ? d->task_name : "(none)",

        (unsigned long)d->r0,  (unsigned long)d->r1,
        (unsigned long)d->r2,  (unsigned long)d->r3,
        (unsigned long)d->r12, (unsigned long)d->lr,
        (unsigned long)d->pc,  (unsigned long)d->xpsr,

        (unsigned long)cfsr,
        (unsigned long)mmsr, (unsigned long)bfsr, (unsigned long)ufsr,
        (unsigned long)d->hfsr,
        (unsigned long)d->mmfar, mmar_valid ? "(valid)"   : "(INVALID)",
        (unsigned long)d->bfar,  bfar_valid ? "(valid)"   : "(INVALID)",

        (unsigned long)d->exc_return,
        (d->exc_return & 0x4U) ? "PSP/thread" : "MSP/handler",
        (unsigned long)d->msp, (unsigned long)d->psp, (unsigned long)d->ipsr,

        (ufsr & (1U << 0)) ? "  UFSR.UNDEFINSTR : undefined instruction\n" : "",
        (ufsr & (1U << 1)) ? "  UFSR.INVSTATE   : invalid EPSR.T/IT state (interworking?)\n" : "",
        (ufsr & (1U << 2)) ? "  UFSR.INVPC      : illegal EXC_RETURN\n" : "",
        (ufsr & (1U << 3)) ? "  UFSR.NOCP       : no coprocessor (FPU off?)\n" : "",
        (ufsr & (1U << 8)) ? "  UFSR.UNALIGNED  : unaligned access\n" : "",
        (ufsr & (1U << 9)) ? "  UFSR.DIVBYZERO  : divide by zero\n" : "",
        (bfsr & (1U << 0)) ? "  BFSR.IBUSERR    : instruction bus error (bad PC fetch)\n" : "",
        (bfsr & (1U << 2)) ? "  BFSR.IMPRECISERR: imprecise data bus error\n" : "",
        (bfsr & (1U << 1)) ? "  BFSR.PRECISERR  : precise data bus error @ BFAR\n" : "",
        (mmsr & (1U << 1)) ? "  MMSR.DACCVIOL   : data access violation @ MMFAR\n" : ""
    );

    if (n < 0) return 0;
    if ((uint32_t)n >= bufsize) return bufsize - 1U;
    return (uint32_t)n;
}
