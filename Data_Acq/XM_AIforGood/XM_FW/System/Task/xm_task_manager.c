/**
 ******************************************************************************
 * @file    xm_task_manager.c
 * @author  HyundoKim
 * @brief   XM10 사용자 Task 매니저 구현 (System Layer)
 * @details
 * API contract 가드 (단순 NULL/false 반환):
 *   - 가드 A: _Static_assert (이 파일)
 *   - 가드 B: instance / budget cap (런타임)
 *   - 가드 E: stack 파라미터 봉인 (default stack 자동 결정)
 *   - 가드 O: Handle magic (Use-After-Delete)
 *   - 가드 P: Task 이름 검증 (NULL / 빈)
 *   - 가드 U: Self-Delete 거부
 *   - 가드 K: Periodic period_ms 검증 (0 거부)
 *   - vPortFree 누수 해결 — XmTaskMgr_Delete 가 명시 해제
 *
 * Risk Management 가드 (F/G/H/I/J/L/N/Q/R)는 Phase 2 별도 도입 예정.
 *
 * @version 1.1
 * @date    2026-05-15
 ******************************************************************************
 */

#include "xm_task_manager.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS
 *-----------------------------------------------------------
 */

#define XM_TASK_MAGIC_ALIVE     0xA17EA70Cu
#define XM_TASK_MAGIC_DEAD      0xDEADBEEFu

#define SLOT_KIND_ONESHOT       0U
#define SLOT_KIND_PERIODIC      1U

/**
 *-----------------------------------------------------------
 * 가드 A — 컴파일 타임 정합성
 *-----------------------------------------------------------
 */

_Static_assert(XM_TASK_STACK_BACKGROUND_WORDS <= XM_TASK_STACK_MAX_WORDS,
               "BACKGROUND stack > MAX");
_Static_assert(XM_TASK_STACK_BELOW_WORDS      <= XM_TASK_STACK_MAX_WORDS,
               "BELOW stack > MAX");
_Static_assert(XM_TASK_STACK_ABOVE_WORDS      <= XM_TASK_STACK_MAX_WORDS,
               "ABOVE stack > MAX");
_Static_assert(XM_TASK_STACK_IDLE_WORDS       >= XM_TASK_STACK_MIN_WORDS,
               "IDLE stack < MIN");
_Static_assert(XM_TASK_MAX_INSTANCES * XM_TASK_STACK_MAX_WORDS * 4U
               <= XM_TASK_HEAP_BUDGET_BYTES * 2U,
               "instance x stack-max exceeds 2x budget");

/**
 *-----------------------------------------------------------
 * PRIVATE TYPES
 *-----------------------------------------------------------
 */

typedef struct XmTaskSlot {
    uint32_t                magic;
    uint8_t                 kind;
    char                    name[16];

    /* OneShot */
    XmTaskMgr_OneShotFn_t   one_func;
    void*                   one_arg;
    volatile bool           done;

    /* Periodic */
    XmTaskMgr_PeriodicFn_t  periodic_func;
    uint32_t                period_ms;

    osThreadId_t            thread_id;
    uint32_t                allocated_bytes;
} XmTaskSlot_t;

/**
 *-----------------------------------------------------------
 * STATIC STATE
 *-----------------------------------------------------------
 */

static uint32_t s_instance_count;
static uint32_t s_budget_used_bytes;

/**
 *-----------------------------------------------------------
 * STATIC FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static void     _OneShotWrapper (void* argument);
static void     _PeriodicWrapper(void* argument);
static uint32_t _GetDefaultStackWords(XmTaskMgr_Prio_t prio_hint);
static bool     _IsValidSlot    (XmTaskMgr_Handle_t handle);
static bool     _GuardBCheck    (uint32_t need_bytes);
static bool     _GuardPNameCheck(const char* name);

/**
 *-----------------------------------------------------------
 * STATIC HELPERS
 *-----------------------------------------------------------
 */

static uint32_t _GetDefaultStackWords(XmTaskMgr_Prio_t prio_hint)
{
    switch (prio_hint) {
        case XM_TASK_PRIO_IDLE:          return XM_TASK_STACK_IDLE_WORDS;
        case XM_TASK_PRIO_BACKGROUND:    return XM_TASK_STACK_BACKGROUND_WORDS;
        case XM_TASK_PRIO_BELOW_CONTROL: return XM_TASK_STACK_BELOW_WORDS;
        case XM_TASK_PRIO_ABOVE_CONTROL: return XM_TASK_STACK_ABOVE_WORDS;
        case XM_TASK_PRIO_NEAR_REALTIME: return XM_TASK_STACK_NRT_WORDS;
        default:                         return XM_TASK_STACK_BACKGROUND_WORDS;
    }
}

static bool _IsValidSlot(XmTaskMgr_Handle_t handle)
{
    if (handle == NULL) return false;
    XmTaskSlot_t* s = (XmTaskSlot_t*)handle;
    return s->magic == XM_TASK_MAGIC_ALIVE;
}

/* 가드 B — 런타임 sanity check: instance/budget 한도 */
static bool _GuardBCheck(uint32_t need_bytes)
{
    if (s_instance_count >= XM_TASK_MAX_INSTANCES) return false;
    if ((s_budget_used_bytes + need_bytes) > XM_TASK_HEAP_BUDGET_BYTES) return false;
    return true;
}

/* 가드 P — Task 이름 검증 (NULL / 빈 거부) */
static bool _GuardPNameCheck(const char* name)
{
    return (name != NULL) && (name[0] != '\0');
}

/**
 *-----------------------------------------------------------
 * PUBLIC API — OneShot / Periodic
 *-----------------------------------------------------------
 */

XmTaskMgr_Handle_t XmTaskMgr_CreateOneShot(const char*           name,
                                            XmTaskMgr_OneShotFn_t func,
                                            void*                 arg,
                                            XmTaskMgr_Prio_t      prio_hint)
{
    if (!_GuardPNameCheck(name)) return NULL;
    if (func == NULL) return NULL;

    uint32_t stack_words = _GetDefaultStackWords(prio_hint);
    uint32_t need_bytes  = stack_words * 4U + (uint32_t)sizeof(XmTaskSlot_t);

    if (!_GuardBCheck(need_bytes)) return NULL;

    XmTaskSlot_t* slot = (XmTaskSlot_t*)pvPortMalloc(sizeof(XmTaskSlot_t));
    if (slot == NULL) return NULL;

    memset(slot, 0, sizeof(*slot));
    slot->magic    = XM_TASK_MAGIC_ALIVE;
    slot->kind     = SLOT_KIND_ONESHOT;
    slot->one_func = func;
    slot->one_arg  = arg;
    slot->done     = false;
    strncpy(slot->name, name, sizeof(slot->name) - 1U);
    slot->allocated_bytes = need_bytes;

    osThreadAttr_t attr = {
        .name       = slot->name,
        .stack_size = stack_words * sizeof(uint32_t),
        .priority   = (osPriority_t)prio_hint,
    };
    slot->thread_id = osThreadNew(_OneShotWrapper, slot, &attr);
    if (slot->thread_id == NULL) {
        slot->magic = XM_TASK_MAGIC_DEAD;
        vPortFree(slot);
        return NULL;
    }

    s_instance_count++;
    s_budget_used_bytes += need_bytes;
    return (XmTaskMgr_Handle_t)slot;
}

XmTaskMgr_Handle_t XmTaskMgr_CreatePeriodic(const char*            name,
                                             XmTaskMgr_PeriodicFn_t func,
                                             uint32_t               period_ms,
                                             XmTaskMgr_Prio_t       prio_hint)
{
    if (!_GuardPNameCheck(name)) return NULL;
    if (func == NULL) return NULL;
    if (period_ms == 0U) return NULL;   /* 가드 K — period 0 거부 */

    uint32_t stack_words = _GetDefaultStackWords(prio_hint);
    uint32_t need_bytes  = stack_words * 4U + (uint32_t)sizeof(XmTaskSlot_t);

    if (!_GuardBCheck(need_bytes)) return NULL;

    XmTaskSlot_t* slot = (XmTaskSlot_t*)pvPortMalloc(sizeof(XmTaskSlot_t));
    if (slot == NULL) return NULL;

    memset(slot, 0, sizeof(*slot));
    slot->magic         = XM_TASK_MAGIC_ALIVE;
    slot->kind          = SLOT_KIND_PERIODIC;
    slot->periodic_func = func;
    slot->period_ms     = period_ms;
    strncpy(slot->name, name, sizeof(slot->name) - 1U);
    slot->allocated_bytes = need_bytes;

    osThreadAttr_t attr = {
        .name       = slot->name,
        .stack_size = stack_words * sizeof(uint32_t),
        .priority   = (osPriority_t)prio_hint,
    };
    slot->thread_id = osThreadNew(_PeriodicWrapper, slot, &attr);
    if (slot->thread_id == NULL) {
        slot->magic = XM_TASK_MAGIC_DEAD;
        vPortFree(slot);
        return NULL;
    }

    s_instance_count++;
    s_budget_used_bytes += need_bytes;
    return (XmTaskMgr_Handle_t)slot;
}

bool XmTaskMgr_IsComplete(XmTaskMgr_Handle_t handle)
{
    if (!_IsValidSlot(handle)) return false;   /* 가드 O — Use-After-Delete */
    XmTaskSlot_t* s = (XmTaskSlot_t*)handle;
    return s->done;
}

void XmTaskMgr_Suspend(XmTaskMgr_Handle_t handle)
{
    if (!_IsValidSlot(handle)) return;
    XmTaskSlot_t* s = (XmTaskSlot_t*)handle;
    if (s->thread_id) osThreadSuspend(s->thread_id);
}

void XmTaskMgr_Resume(XmTaskMgr_Handle_t handle)
{
    if (!_IsValidSlot(handle)) return;
    XmTaskSlot_t* s = (XmTaskSlot_t*)handle;
    if (s->thread_id) osThreadResume(s->thread_id);
}

void XmTaskMgr_Delete(XmTaskMgr_Handle_t handle)
{
    if (handle == NULL) return;
    XmTaskSlot_t* s = (XmTaskSlot_t*)handle;
    if (s->magic == XM_TASK_MAGIC_DEAD) return;  /* idempotent */
    if (s->magic != XM_TASK_MAGIC_ALIVE) return; /* 가드 O */

    /* 가드 U — self-Delete 거부 */
    if (s->thread_id != NULL && osThreadGetId() == s->thread_id) return;

    if (s->thread_id != NULL &&
        osThreadGetState(s->thread_id) != osThreadTerminated) {
        osThreadTerminate(s->thread_id);
    }

    if (s_instance_count > 0U) s_instance_count--;
    if (s_budget_used_bytes >= s->allocated_bytes) {
        s_budget_used_bytes -= s->allocated_bytes;
    }
    s->magic = XM_TASK_MAGIC_DEAD;
    vPortFree(s);
}

/**
 *-----------------------------------------------------------
 * PUBLIC API — Mutex (단순 wrapper)
 *-----------------------------------------------------------
 */

XmTaskMgr_MutexHandle_t XmTaskMgr_MutexCreate(void)
{
    return (XmTaskMgr_MutexHandle_t)osMutexNew(NULL);
}

bool XmTaskMgr_MutexLock(XmTaskMgr_MutexHandle_t m, uint32_t timeout_ms)
{
    if (m == NULL) return false;
    return osMutexAcquire((osMutexId_t)m,
                          (timeout_ms == 0U) ? 0U : pdMS_TO_TICKS(timeout_ms)) == osOK;
}

void XmTaskMgr_MutexUnlock(XmTaskMgr_MutexHandle_t m)
{
    if (m == NULL) return;
    osMutexRelease((osMutexId_t)m);
}

void XmTaskMgr_MutexDelete(XmTaskMgr_MutexHandle_t m)
{
    if (m == NULL) return;
    osMutexDelete((osMutexId_t)m);
}

/**
 *-----------------------------------------------------------
 * PUBLIC API — Budget / 진단 (단순 카운터 조회)
 *-----------------------------------------------------------
 */

uint32_t XmTaskMgr_GetHeapFreeBytes(void)
{
    return (uint32_t)xPortGetFreeHeapSize();
}

uint32_t XmTaskMgr_GetHeapMinEverBytes(void)
{
    return (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

uint32_t XmTaskMgr_GetInstanceCount(void)
{
    return s_instance_count;
}

uint32_t XmTaskMgr_GetBudgetRemainingBytes(void)
{
    if (s_budget_used_bytes >= XM_TASK_HEAP_BUDGET_BYTES) return 0U;
    return XM_TASK_HEAP_BUDGET_BYTES - s_budget_used_bytes;
}

/**
 *-----------------------------------------------------------
 * PUBLIC API — Task list 출력 (on-demand)
 *-----------------------------------------------------------
 * @note  현 단계 (Phase 1) 는 출력 채널 미연결 — 단순 stub.
 *        Phase 2 에서 PhAI Studio 진단 채널 연동 후 활성화.
 */
void XmTaskMgr_PrintTaskList(void)
{
    /* Phase 1: no-op. Phase 2 (PhAI Studio 통합) 에서 활성화 예정. */
}

/**
 *-----------------------------------------------------------
 * STATIC — Task wrapper
 *-----------------------------------------------------------
 */

static void _OneShotWrapper(void* argument)
{
    XmTaskSlot_t* slot = (XmTaskSlot_t*)argument;
    slot->one_func(slot->one_arg);
    slot->done = true;
    /* 자기 자신 Delete 안 함 — 외부 task 가 명시 호출 (가드 U 정책) */
    osThreadExit();
}

static void _PeriodicWrapper(void* argument)
{
    XmTaskSlot_t* slot = (XmTaskSlot_t*)argument;
    TickType_t       last   = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(slot->period_ms);

    for (;;) {
        vTaskDelayUntil(&last, period);
        if (slot->magic != XM_TASK_MAGIC_ALIVE) break;  /* Delete 시 즉시 종료 */
        slot->periodic_func();
    }
    osThreadExit();
}
