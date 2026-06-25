#include "agr_mw_conf.h"   /* user-supplied: per-module MAX_SYSTEM_TASKS override */
#include "task_mngr.h"
#include "internal/task_state_machine.h"

#include <stddef.h>

/* Per-module override via System/Config/agr_mw_conf.h:
 *   #define MAX_SYSTEM_TASKS 16    // e.g., D2P CM full spec
 * Default preserved at 10 for backward-compat with existing consumers
 * (XM10, IMU Hub) — pure additive change. */
#ifndef MAX_SYSTEM_TASKS
#define MAX_SYSTEM_TASKS    10
#endif

/* Concrete definition of the opaque handle. Engine state is the single
 * source of truth for current id/lifecycle; prev_* is tracked here. */
struct TsmObject_s {
    TaskStateMachine_t engine;
    uint8_t            prev_state_id;
    TsmLifecycle_e     prev_step;
};

static struct TsmObject_s s_taskPool[MAX_SYSTEM_TASKS];
static uint8_t            s_taskAllocIdx = 0;

TsmObject_t* TaskMngr_Create(uint8_t initial_state_id)
{
    if (s_taskAllocIdx >= MAX_SYSTEM_TASKS) {
        return NULL;
    }

    struct TsmObject_s* obj = &s_taskPool[s_taskAllocIdx++];

    TSM_Init(&obj->engine, initial_state_id);
    obj->prev_state_id = initial_state_id;
    obj->prev_step     = TSM_LIFECYCLE_ENTRY;

    return obj;
}

void TaskMngr_AddState(TsmObject_t* obj, uint8_t state_id,
                       EntryFunc_t entry, LoopFunc_t loop, ExitFunc_t exit)
{
    if (obj == NULL) return;
    TSM_AddState(&obj->engine, state_id, entry, loop, exit);
}

void TaskMngr_Run(TsmObject_t* obj)
{
    if (obj == NULL) return;

    uint8_t        prev_id = obj->engine.curr_state_id;
    TsmLifecycle_e prev_lc = obj->engine.lifecycle;

    TSM_Run(&obj->engine);

    if (obj->engine.curr_state_id != prev_id) {
        obj->prev_state_id = prev_id;
    }
    if (obj->engine.lifecycle != prev_lc) {
        obj->prev_step = prev_lc;
    }
}

int32_t TaskMngr_Transition(TsmObject_t* obj, uint8_t next_state_id)
{
    if (obj == NULL) return -1;
    return TSM_TransitionTo(&obj->engine, next_state_id);
}

void TaskMngr_ResetPool(void)
{
    s_taskAllocIdx = 0;
    TSM_ResetPool();
}

uint8_t TaskMngr_GetStateId(const TsmObject_t* obj)
{
    if (obj == NULL) return 0xFF;
    return obj->engine.curr_state_id;
}

TsmLifecycle_e TaskMngr_GetLifecycle(const TsmObject_t* obj)
{
    if (obj == NULL) return TSM_LIFECYCLE_ENTRY;
    return obj->engine.lifecycle;
}

uint8_t TaskMngr_GetPrevStateId(const TsmObject_t* obj)
{
    if (obj == NULL) return 0xFF;
    return obj->prev_state_id;
}

TsmLifecycle_e TaskMngr_GetPrevLifecycle(const TsmObject_t* obj)
{
    if (obj == NULL) return TSM_LIFECYCLE_ENTRY;
    return obj->prev_step;
}

/* ===== TsmDelay — one-shot timer ========================================
 *
 * Tick source is injected (TsmDelay_GetTickFunc_t) — TSM stays free of
 * direct cmsis_os2.h / HAL dependencies, matching the PnP "RTOS/BareMetal
 * 둘 다 지원" pattern (README §"Module Authoring Reference — PnP"). */

static TsmDelay_GetTickFunc_t s_tsm_get_tick = NULL;

void TsmDelay_SetTickProvider(TsmDelay_GetTickFunc_t fn)
{
    s_tsm_get_tick = fn;
}

void TsmDelay_Start(TsmDelay_t* d, uint32_t duration_ms)
{
    if (d == NULL) return;

    d->duration_ms = duration_ms;
    d->fired       = false;

    if (s_tsm_get_tick == NULL) {
        /* No tick source bound — leave inactive so Expired() returns false
         * instead of silently using a stale start_ms. */
        d->start_ms = 0;
        d->active   = false;
        return;
    }

    d->start_ms = s_tsm_get_tick();
    d->active   = true;
}

bool TsmDelay_Expired(TsmDelay_t* d)
{
    if (d == NULL || !d->active || d->fired || s_tsm_get_tick == NULL) {
        return false;
    }

    /* Wrap-around safe: subtraction in unsigned 32-bit, compared as signed.
     * As long as duration_ms <= INT32_MAX (~24.8 days @ 1 kHz tick), the
     * elapsed delta remains monotonically increasing across uint32_t wrap.
     * Standard idiom for tick-based timers on 32-bit MCUs. */
    int32_t elapsed = (int32_t)(s_tsm_get_tick() - d->start_ms);
    if (elapsed >= (int32_t)d->duration_ms) {
        d->fired  = true;
        d->active = false;
        return true;
    }
    return false;
}

void TsmDelay_Reset(TsmDelay_t* d)
{
    if (d == NULL) return;
    d->active = false;
    d->fired  = false;
}
