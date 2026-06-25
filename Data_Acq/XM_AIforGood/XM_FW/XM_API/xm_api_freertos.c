/**
 ******************************************************************************
 * @file    xm_api_freertos.c
 * @author  HyundoKim
 * @brief   XM10 사용자 Task API — Thin Facade
 * @details
 * 아키텍처 위치: Facade Layer (XM_FW/XM_API/)
 *   - Application 으로 노출되는 단일 진입점
 *   - 실제 구현은 System Layer (xm_task_manager / xm_runtime_diag) 가 담당
 *   - 본 파일은 단순 forward dispatch 만 수행 (아키텍처 룰 [00-core-architecture.md:112])
 *
 * @version 2.0
 * @date    2026-05-13
 ******************************************************************************
 */

#include "xm_api_freertos.h"

/* System Layer 호출 */
#include "xm_task_manager.h"

/* XmTaskMgr_Prio_t == XmTaskPrio_t (1:1 매핑 — System/API 사이 단순 cast) */
_Static_assert((int)XM_PRIO_IDLE          == (int)XM_TASK_PRIO_IDLE,          "prio map");
_Static_assert((int)XM_PRIO_BACKGROUND    == (int)XM_TASK_PRIO_BACKGROUND,    "prio map");
_Static_assert((int)XM_PRIO_BELOW_CONTROL == (int)XM_TASK_PRIO_BELOW_CONTROL, "prio map");
_Static_assert((int)XM_PRIO_ABOVE_CONTROL == (int)XM_TASK_PRIO_ABOVE_CONTROL, "prio map");
_Static_assert((int)XM_PRIO_NEAR_REALTIME == (int)XM_TASK_PRIO_NEAR_REALTIME, "prio map");

/**
 *-----------------------------------------------------------
 * OneShot / Periodic / Lifecycle
 *-----------------------------------------------------------
 */

XmTaskHandle_t XM_Task_CreateOneShot(const char*         name,
                                      XmTaskOneShotFunc_t func,
                                      void*               arg,
                                      XmTaskPrio_t        prio_hint)
{
    return (XmTaskHandle_t)XmTaskMgr_CreateOneShot(
        name, (XmTaskMgr_OneShotFn_t)func, arg, (XmTaskMgr_Prio_t)prio_hint);
}

XmTaskHandle_t XM_Task_CreatePeriodic(const char*          name,
                                       XmTaskPeriodicFunc_t func,
                                       uint32_t             period_ms,
                                       XmTaskPrio_t         prio_hint)
{
    return (XmTaskHandle_t)XmTaskMgr_CreatePeriodic(
        name, (XmTaskMgr_PeriodicFn_t)func, period_ms, (XmTaskMgr_Prio_t)prio_hint);
}

bool XM_Task_IsComplete(XmTaskHandle_t handle)
{
    return XmTaskMgr_IsComplete((XmTaskMgr_Handle_t)handle);
}

void XM_Task_Suspend(XmTaskHandle_t handle)
{
    XmTaskMgr_Suspend((XmTaskMgr_Handle_t)handle);
}

void XM_Task_Resume(XmTaskHandle_t handle)
{
    XmTaskMgr_Resume((XmTaskMgr_Handle_t)handle);
}

void XM_Task_Delete(XmTaskHandle_t handle)
{
    XmTaskMgr_Delete((XmTaskMgr_Handle_t)handle);
}

/**
 *-----------------------------------------------------------
 * Mutex
 *-----------------------------------------------------------
 */

XmMutexHandle_t XM_Mutex_Create(void)
{
    return (XmMutexHandle_t)XmTaskMgr_MutexCreate();
}

bool XM_Mutex_Lock(XmMutexHandle_t mutex, uint32_t timeout_ms)
{
    return XmTaskMgr_MutexLock((XmTaskMgr_MutexHandle_t)mutex, timeout_ms);
}

void XM_Mutex_Unlock(XmMutexHandle_t mutex)
{
    XmTaskMgr_MutexUnlock((XmTaskMgr_MutexHandle_t)mutex);
}

void XM_Mutex_Delete(XmMutexHandle_t mutex)
{
    XmTaskMgr_MutexDelete((XmTaskMgr_MutexHandle_t)mutex);
}

/**
 *-----------------------------------------------------------
 * Budget / 진단
 *-----------------------------------------------------------
 */

uint32_t XM_Task_GetHeapFreeBytes(void)        { return XmTaskMgr_GetHeapFreeBytes(); }
uint32_t XM_Task_GetHeapMinEverBytes(void)     { return XmTaskMgr_GetHeapMinEverBytes(); }
uint32_t XM_Task_GetInstanceCount(void)        { return XmTaskMgr_GetInstanceCount(); }
uint32_t XM_Task_GetBudgetRemainingBytes(void) { return XmTaskMgr_GetBudgetRemainingBytes(); }

void XM_RTOS_PrintTaskList(void) { XmTaskMgr_PrintTaskList(); }

/**
 *-----------------------------------------------------------
 * Deprecated — XM_BgTask_* thin wrapper
 *-----------------------------------------------------------
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

XmBgTaskHandle_t XM_BgTask_Create(const char* name, XmBgTaskFunc_t func,
                                   void* arg, uint32_t stack_words)
{
    (void)stack_words;
    return (XmBgTaskHandle_t)XmTaskMgr_CreateOneShot(
        (name != NULL) ? name : "BgTask",
        (XmTaskMgr_OneShotFn_t)func,
        arg,
        XM_TASK_PRIO_BACKGROUND);
}

bool XM_BgTask_IsDone(XmBgTaskHandle_t handle)
{
    return XmTaskMgr_IsComplete((XmTaskMgr_Handle_t)handle);
}

#pragma GCC diagnostic pop
