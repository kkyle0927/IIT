# AGR_MW/RiskMngr V2 — Implementation Plan (Production-Grade)

**Status**: Plan finalized, awaiting Phase 0 kickoff
**Author**: AGR HW Team (designed with Claude Opus 4.6)
**Date**: 2026-04-16
**Version**: 2.0 — Full production-quality revision

---

## 0. Revision History

| Ver | Date | Changes |
|---|---|---|
| 1.0 | 2026-04-16 | Initial plan after Q&A 13 decisions |
| 2.0 | 2026-04-16 | **Full re-review for production quality**: added Safety Requirements, Fault Lifecycle FSM, API Contracts (preconditions/WCET/thread-context), Framework Self-Protection, Fault Storm Handling, Observability, Configuration Validation, Memory Placement, ABI Versioning, Reserved Meta-Fault Codes, 5-level Test Strategy, Certification Artifacts |
| 2.1 | 2026-04-16 | **Usability & industry alignment** (6 fact-checked references): added §2a Industry Reference Alignment, §4a Quick Start Scenarios, §10.3b Polling-style API (`GetActiveFaults`), §10.5b Severity-filtered Subscribe, weak-linkage default for safe_state_cb (Zephyr pattern), deferred debouncing to v3.0 (AUTOSAR DEM PRE_* states) |
| 2.2 | 2026-04-16 | **3-Layer philosophy audit**: (1) §1.4 Architectural Principle 명시화, (2) Error dictionary 분리 (`agr_error_base.h` MW vs `<module>_error_codes.h` App), (3) §10 FATAL latency 상한 + HW-level cutoff 구분 명시, (4) Severity vs Priority 독립성 설명, (5) API Tier 0/1/2/3 표 (MVP=8개), (6) Integration guide L1/L2/L3 경계 예시 outline, (7) Decision #25 철학 확정 |

---

## 1. Executive Summary & Safety Posture

### 1.1 Goal
CM의 CDI `Risk_Mngr` 엔진(197 LOC, 기본 우선순위 큐)을 `AGR_MW/RiskMngr`로 승격하되,
**"Safety Element out of Context" (ISO 26262 Part 10)** 개념에 부합하는 재사용 가능한
fault-management framework으로 재설계한다.

### 1.2 Safety Posture Statement
본 framework는 **SIL-2 equivalent (IEC 61508)** / **ASIL-B candidate (ISO 26262)** 수준을
**목표**로 설계한다. 즉:
- Framework 자체가 fault의 원인이 되어서는 안 된다 (self-protection)
- Framework 장애가 silent이어서는 안 된다 (self-fault reporting)
- Deterministic timing (WCET bounded, no unbounded loops, no heap)
- 모든 state transition이 auditable (NVM trace)

> 실제 인증(certification)은 본 Plan 범위 밖. 그러나 **인증 진입이 가능한 품질 수준**으로
> 구현·문서화한다.

### 1.3 Non-Goals (명시적)
- Fault-tolerant computing (TMR, lockstep): out of scope
- Cryptographic integrity (Secure Boot 연계): out of scope (future)
- Multi-instance (domain separation): documented as future, 현재는 singleton

### 1.4 **3-Layer Architectural Principle** (최상위 설계 철학)

본 framework는 **Mechanism ↔ Policy 엄격 분리** 원칙을 따른다. 모든 설계 결정은 이 철학을 준수해야 한다.

```
┌──────────────────────────────────────────────────────────────────────────┐
│  Layer 1: DETECTION  (App — polling or ISR)                              │
│    "무엇을 감시할지" + "언제 임계치 넘는지"                                    │
│    예) sensor task가 ADC read, threshold check, ReportFault() 호출         │
│                                                                           │
│    🚫 MW는 절대 threshold를 모른다. MW는 센서를 읽지 않는다.                  │
└───────────────────────────────┬──────────────────────────────────────────┘
                                │ ReportFault(code, payload) = event
┌───────────────────────────────▼──────────────────────────────────────────┐
│  Layer 2: MECHANISM  (AGR_MW/RiskMngr — event-driven)                    │
│    "어떻게 전파·분류·큐잉·표준화·영구기록하는가"                                │
│    Queue, Priority sort, Severity routing, Retry FSM, CiA EMCY, NVM,     │
│    Dispatch to cb(s), Integrity (CRC/sentinel), Rate limit, Coalescing  │
│                                                                           │
│    🚫 MW는 절대 "어떤 action을 할지"를 결정하지 않는다.                         │
│    🚫 MW는 절대 App 비즈니스 로직을 포함하지 않는다.                            │
│    ✅ MW는 App이 Descriptor에서 선언한 `severity` / `policy`를 실행만 한다.    │
└───────────────────────────────┬──────────────────────────────────────────┘
                                │ subscriber cb / safe_state cb
┌───────────────────────────────▼──────────────────────────────────────────┐
│  Layer 3: POLICY  (App — action dispatcher)                              │
│    "어떤 fault에 어떤 동작을 할지" = 로봇 시스템 기획                            │
│    예) PlayAudio, SetLED, DegradedMode, NMT_Stop, PowerOff, SD_Log        │
│                                                                           │
│    🚫 App은 queue/dispatch mechanism을 직접 구현하지 않는다.                  │
│    ✅ App은 cb 안에서 자유롭게 제품 로직 작성.                                 │
└──────────────────────────────────────────────────────────────────────────┘
```

### 1.5 Layer 경계 판정 규칙 (설계/리뷰 시 적용)

새로운 기능/API 추가 시 다음 질문으로 레이어 판정:

| 질문 | Yes → | No → |
|---|---|---|
| "이것은 제품별로 달라지는가?" | L1 or L3 (App) | L2 후보 |
| "이것은 센서/HW 의존적인가?" | L1 | L2/L3 후보 |
| "이것은 비즈니스 결정(어떤 행동)을 담는가?" | L3 | L2 후보 |
| "이것은 재사용 가능한 데이터 변환/큐잉/표준인가?" | L2 | 재검토 |
| "Descriptor에 들어가는 parameterization인가?" | L3 config consumed by L2 | — |

**예시 판정**:
- Battery threshold 26V → L1 (App sensor task에 상수)
- Fault queue priority 정렬 → L2 (MW mechanism)
- "FATAL 시 power-off" → L3 (App safe_state_cb)
- Retry 횟수 = 3회 → L3 config, L2가 실행
- EMCY frame 인코딩 → L2 (CiA 표준 mechanism)
- SD 카드 경로 → L1/L3 (App)

---

## 2. Standards & Compliance Reference

| Standard | Clause | 적용 방식 |
|---|---|---|
| **CiA 301 v4.2.0** | §7.2.7 (EMCY), §7.5.2 (OD mandatory objects) | Full compliance on 0x1001/0x1003/0x1014/0x1015 + EMCY 8-byte frame |
| **ISO 26262-6:2018** | §5.4.3 (SW safety requirements), §7.4.4 (SEooC) | Design intent 수준 준수 |
| **IEC 61508-3:2010** | §7.4 (SW architecture), Annex B (tables) | Techniques: static analysis, bounded data, defensive programming |
| **MISRA-C:2012** | Mandatory + Required rules | Baseline (deviation은 명시적 주석) |
| **CERT-C:2016** | MEM, INT, EXP, FIO sections | Secondary reference |
| **ISO 13482:2014** | §5.10 (protective stop) | Safe-state 진입 API 설계 근거 |

**MISRA 준수 수준**: Mandatory 100%, Required 95%+ (justified deviation만 허용).
Advisory는 case-by-case.

---

## 2a. Industry Reference Alignment (fact-checked 2026-04-16)

본 framework는 6개 권위 있는 업계 표준의 **공통 패턴**을 따른다:

| 표준 | 공통 패턴 | 본 MW 대응 |
|---|---|---|
| **AUTOSAR DEM** R24-11 | `Dem_SetEventStatus(EventId, Status)` 이벤트 API + 내부 status bit processing + debouncing | `ReportFault()` + dispatcher + rate-limit (debouncing은 v3.0 deferred) |
| **NASA cFS EVS** | `CFE_EVS_SendEvent` + Software Bus pub/sub + 앱 사전 등록 필수 | `ReportFault()` + `Subscribe()` + descriptor 사전 등록 |
| **CANopen CiA 301 EMCY** | "transmitted only once per error event" + inhibit time 0x1015 | EMCY producer + coalescing (occurrence_count) + 0x1015 구현 |
| **IEC 60601-1-8** (Medical) | 3-tier 우선순위, high priority dominates, 물리/기술 알람 분리 | 4-tier severity (Info/Warn/Error/Fatal) + priority queue |
| **Zephyr RTOS** | Weak-linkage `k_sys_fatal_error_handler()` + `SYS_FATAL_ERROR_DEFINE` iterable section | Weak default safe_state_cb (halt+wdg) + multi-subscriber array |
| **Safety-Critical 원칙** | Top-Half (ISR capture) + Bottom-Half (Task process) | `ReportFaultFromISR()` + `RunPeriodic()` |

### 설계 결정: **Event-driven API + Deferred Task Processing (Hybrid)**

**근거** (검증된 인용):
- Polling은 "non-deterministic under load" (Safety-Critical Embedded Systems 원칙)
- EMCY는 "interrupt type error alerts"에 적합 (CiA 301)
- DEM은 "SWC가 호출 시 status bit processing 트리거" (AUTOSAR R24-11)

### 폴링은 어디서 쓰이나?
업계 공통: **감지 (detection) 단계에서만 사용**.
- 센서 threshold check: 각 App task가 polling (10ms ADC read)
- Threshold 위반 시 → `ReportFault()` (event)
- MW 내부 housekeeping (retry tick, NVM flush, CRC verify): `RunPeriodic()` (deferred polling of state)

**본 framework 자체는 pure event-driven**. Polling은 App 영역.

### 폴링 스타일 소비자 지원
일부 App은 "callback 대신 주기적으로 active fault 목록을 pull"을 선호할 수 있다 (더 예측 가능). 이를 위해 `GetActiveFaults()` API 제공 (§10.3b). AUTOSAR DEM의 `Dem_GetEventStatus` 대응.

---

## 3. Decisions Log (누적)

| # | 주제 | 결정 | 근거 |
|---|---|---|---|
| 1 | MD V1 format 유지 | v2.1 default OFF, v3.0 제거 | Deprecation discipline |
| 2 | NVM 매체 | MW 인터페이스만; 매체는 각 모듈 | Portability |
| 3 | Heartbeat 0x1017 | NMT/PnP 소관 | 중복 방지 |
| 4 | Subscriber 개수 | 모듈별 override (default 0) | Per-module 결정 |
| 5 | MAX_ACTIVE | 모듈별 override (default 16) | Per-module 결정 |
| 6 | Host test 프레임워크 | 별도 Plan | 현 scope 보호 |
| 7 | CiA 301 수준 | Full compliance | Long-term interop |
| 8 | CM OOB 버그 | H10 세션에서 | V1 production 보호 |
| 9 | OD 0x1000 | System Layer 소관 | Identity ≠ fault |
| 10 | NVM trigger | Fatal 즉시 + 주기 batch (A+C) | Flash wear vs coverage |
| 11 | EMCY tx 시그니처 | DOP 기존 타입 재사용 | DRY |
| 12 | Test hook 플래그 | 전용 매크로 | Least privilege |
| 13 | Compat layer | Hybrid (conf.h + auto-include) | Single-knob 통제 |
| **14** | Framework 자체 fault | **Reserved 예약 범위 + 자체 보고** | Meta-safety |
| **15** | WCET 명시 | **모든 public API에 budget 선언** | Determinism |
| **16** | Descriptor integrity | **Init 시 CRC32 계산, 주기 검증** | Self-protection |
| **17** | Fault storm 방어 | **Token-bucket rate limit + coalescing** | Backpressure |
| **18** | Timestamp source | **모노토닉 (HAL_GetTick 권장)** — wall-clock은 NVM record에만 | Overflow determinism |
| **19** | Subscriber blocking | **Non-blocking 강제 + watchdog timer** | RunPeriodic 지연 차단 |
| **20** | 재부팅 경로 보존 | **Backup SRAM 우선 지원 + Flash fallback** | Post-mortem 분석 |
| **21** | Safe-state cb 미설정 시 동작 | **Zephyr-style weak default: `while(1) IWDG_Refresh()`** | Fail-safe 기본값 |
| **22** | 폴링 스타일 소비자 지원 | **`GetActiveFaults(buf, max)` accessor 제공** | AUTOSAR DEM 대응 |
| **23** | Severity-filtered subscribe | **`SubscribeFiltered(cb, min_sev)` 제공** | Medical 3-tier 대응 |
| **24** | AUTOSAR DEM debouncing (PRE_PASSED/PRE_FAILED) | **v3.0 deferred** — 현재 즉시 Active 진입 | 복잡도 vs 가치 |
| **25** | **3-Layer 엄격 분리 철학** | **Mechanism(MW) ↔ Policy(App) 경계 확정** — 새 API/기능 추가 시 §1.5 판정 규칙 적용 | 최상위 설계 원칙 |
| **26** | Error code 파일 분리 | **`agr_error_base.h` (MW: 범위+meta) + `<module>_error_codes.h` (App: 제품 코드)** | L2/L3 경계 준수 |
| **27** | FATAL 대응 latency 상한 | **MW safe_state_cb worst-case = RunPeriodic 주기**. µs급 HW 차단은 App ISR 직접 수행 | 시스템 vs HW 레벨 구분 |

---

## 4. Scope

### ✅ In Scope (MW 모듈 자체)
- Fault engine: descriptor registry + active table + SPSC queue + priority dispatch
- **Fault lifecycle FSM (formal)**
- 4-level Severity × 4-level Policy matrix
- Full CiA 301: OD 0x1001/0x1003/0x1014/0x1015 + EMCY 8-byte producer/consumer
- ISR-safe report path (lock-free MPSC + task drain)
- Backend abstraction: FreeRTOS / BareMetal
- NVM persistence interface + Internal Flash reference adapter
- Safe-state callback injection
- Error dictionary v2 (네임스페이스화)
- V1 compat layer with deprecation schedule
- **Framework self-protection (descriptor CRC, canaries, POST, runtime self-test)**
- **Fault storm handling (rate limit, coalescing, overflow meta-fault)**
- **Observability (stats, trace hooks, health report)**
- **Configuration validation at Init**
- **Reserved meta-fault code range (0xF000–0xFFFF)**
- Test hook seam (`AGR_RISK_MNGR_TEST_HOOK`)

### ❌ Out of Scope
- Heartbeat 0x1017 (NMT/PnP)
- Device Type 0x1000 (System Layer)
- Node ID 관리 (System Layer)
- Multi-instance (singleton only; future work)
- Secure boot / tamper detection 통합
- 각 모듈 마이그레이션 (per-module Plan)
- Host test 프레임워크 선정 (별도 Plan)
- **Fault debouncing (AUTOSAR DEM PRE_PASSED/PRE_FAILED)** — v3.0 deferred

---

## 4a. Quick Start Scenarios

이 섹션은 실제 소비자(App 개발자)가 **무엇을 작성하는지** 보여준다. 업계 표준 공통: framework 문서는 반드시 concrete scenario를 제공해야 한다.

### Scenario 1: Basic CM (단일 Subscriber + SafeState) — 권장 기본 패턴

```c
/* 1. Fault 카탈로그 — 컴파일 타임 정적 */
static const AGR_RiskMngr_FaultDescriptor_t cm_faults[] = {
    { AGR_ERR_BATT_OVER_VOLT,  SEV_FATAL,   POLICY_SHUTDOWN,       ERR_REG_VOLTAGE, 0, 0 },
    { AGR_ERR_BATT_OCP,        SEV_FATAL,   POLICY_SHUTDOWN,       ERR_REG_CURRENT, 0, 0 },
    { AGR_ERR_BOARD_OT,        SEV_FATAL,   POLICY_SHUTDOWN,       ERR_REG_TEMP,    0, 0 },
    { AGR_ERR_BOARD_OT_WARN,   SEV_WARNING, POLICY_AUTO_RETRY,     ERR_REG_TEMP,    10, 1000 },
    { AGR_ERR_SD_FULL,         SEV_WARNING, POLICY_LATCH,          ERR_REG_MFR,     10, 0 },
    { AGR_ERR_MD_CH1_OCP,      SEV_ERROR,   POLICY_BOUNDED_RETRY,  ERR_REG_CURRENT, 2, 100 },
};

/* 2. Top-level Risk Mgmt Task */
void RiskMngt_Task(void *arg) {
    AGR_RiskMngr_InitConfig_t cfg = { .node_id = CM_NODE_ID, .emcy_tx = DOP_CanTx };
    AGR_RiskMngr_Init(&cfg);
    AGR_RiskMngr_RegisterDescriptors(cm_faults, ARRAY_SIZE(cm_faults));
    AGR_RiskMngr_Subscribe(CM_OnFault);
    AGR_RiskMngr_SetSafeStateCallback(CM_EnterSafeState, NULL);
    AGR_RiskMngr_SetNvmBackend(&flash_nvm);
    AGR_RiskMngr_LoadBlackBoxOnBoot();

    for (;;) {
        AGR_RiskMngr_RunPeriodic(HAL_GetTick());
        osDelay(10);
    }
}

/* 3. 통합 Action Dispatcher (사용자가 작성하는 핵심 부분) */
static void CM_OnFault(const AGR_RiskMngr_FaultEvent_t *ev)
{
    SD_LogFault(ev);  /* 항상 로그 */

    switch (ev->severity) {
    case SEV_FATAL:   PlayAudio(AUDIO_FATAL); SetLED(LED_FATAL); break;
    case SEV_ERROR:   PlayAudio(AUDIO_ERROR); SetLED(LED_ERROR);
                      DegradedMode_Enter(ev->code); break;
    case SEV_WARNING: SetLED(LED_WARN); break;
    case SEV_INFO:    break;
    }
}

/* 4. SafeState — FATAL 1회 호출 보장 (MW가 re-entrancy 차단) */
static void CM_EnterSafeState(AGR_RiskMngr_Severity_t sev, uint16_t code, void *arg) {
    AGR_PnP_Master_NmtBroadcastStop();
    osDelay(500);           /* 오디오 flush */
    GPIO_PowerOff();
    while (1) { HAL_IWDG_Refresh(&hiwdg); }
}

/* 5. Fault 감지 (각 task 또는 ISR) */
void System_Task(void *arg) {
    for (;;) {
        float v = ADC_GetBattV();
        if (v > 26.0f) AGR_RiskMngr_ReportFault(AGR_ERR_BATT_OVER_VOLT, &v, sizeof(v));
        osDelay(10);
    }
}

void TIM6_IRQHandler(void) {
    if (OCP_HW_TRIGGERED) AGR_RiskMngr_ReportFaultFromISR(AGR_ERR_PHASE_OCP_HW);
}
```

**사용자가 작성하는 총 코드 규모**: Descriptor 테이블 (~20줄) + RiskMngt_Task (~15줄) + OnFault dispatcher (~10줄) + SafeState (~5줄) + 각 task의 `ReportFault` 호출 (~1줄 each) = **~60 LOC 핵심 로직**.

---

### Scenario 2: Polling-style Consumer (callback 회피)

일부 팀은 callback 모델보다 "주기적으로 state를 pull"을 선호한다 (더 예측 가능).
```c
void RiskMngt_Task(void *arg) {
    AGR_RiskMngr_FaultEvent_t active[AGR_RISK_MNGR_MAX_ACTIVE];
    AGR_RiskMngr_Init(&cfg);
    AGR_RiskMngr_RegisterDescriptors(faults, N);
    AGR_RiskMngr_SetSafeStateCallback(EnterSafeState, NULL); /* FATAL은 여전히 즉시 */

    for (;;) {
        AGR_RiskMngr_RunPeriodic(HAL_GetTick());

        /* Pull 모델 */
        uint8_t n = AGR_RiskMngr_GetActiveFaults(active, ARRAY_SIZE(active));
        for (uint8_t i = 0; i < n; i++) {
            DispatchAction(&active[i]);   /* 사용자의 dispatcher */
        }

        osDelay(10);
    }
}
```

**주의**: Pull 모델은 callback 모델보다 latency 1 cycle 높음. FATAL은 여전히 callback 권장 (즉각 대응).

---

### Scenario 3: 여러 Subscriber + Severity 필터 (Medical/Advanced)

```c
/* 각 concern별 독립 subscriber */
static void Logger_OnAny(const AGR_RiskMngr_FaultEvent_t *ev) { SD_LogFault(ev); }
static void UI_OnHigh(const AGR_RiskMngr_FaultEvent_t *ev)    { ShowAlert(ev); }
static void Audio_OnCritical(const AGR_RiskMngr_FaultEvent_t *ev) { BeepUrgent(ev); }

AGR_RiskMngr_Subscribe(Logger_OnAny);                             /* 모든 이벤트 */
AGR_RiskMngr_SubscribeFiltered(UI_OnHigh,       SEV_WARNING);     /* WARN 이상 */
AGR_RiskMngr_SubscribeFiltered(Audio_OnCritical, SEV_ERROR);      /* ERROR 이상 */
```

IEC 60601-1-8 의료 알람의 3-tier 우선순위 구조에 대응.

---

### Scenario 4: ISR-driven Slave (FES Hub)

```c
/* 1ms TIM6 ISR에서 감지 (G4 BareMetal, FreeRTOS 없음) */
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
    if (FES_OverCurrent_Detected()) {
        /* WCET ≤ 500 ns 보장, 저장만 하고 task에서 처리 */
        AGR_RiskMngr_ReportFaultFromISR(AGR_ERR_FES_CH1_OCP);
    }
}

/* Main loop */
int main(void) {
    /* ... HW init ... */
    AGR_RiskMngr_Init(&cfg);
    AGR_RiskMngr_RegisterDescriptors(fes_faults, ARRAY_SIZE(fes_faults));
    AGR_RiskMngr_Subscribe(FES_OnFault);
    AGR_RiskMngr_SetSafeStateCallback(FES_DisableAllChannels, NULL);

    for (;;) {
        AGR_RiskMngr_RunPeriodic(HAL_GetTick());
        /* 기타 메인 루프 작업 */
    }
}
```

FreeRTOS 없이도 동일 API. Main loop = bottom-half.

---

## 5. Functional Requirements (FR)

번호는 traceability용. 모든 test case가 이 FR을 참조한다.

| ID | Requirement | Verify by |
|---|---|---|
| FR-01 | Descriptor 테이블을 정적 등록할 수 있어야 한다 | Unit |
| FR-02 | Fault를 code + payload로 보고할 수 있어야 한다 | Unit |
| FR-03 | ISR context에서 code만으로 보고할 수 있어야 한다 | Unit + timing |
| FR-04 | 동일 fault code 연속 보고는 `occurrence_count` 증분으로 coalescing되어야 한다 | Unit |
| FR-05 | Fault dispatch는 priority descending order로 수행되어야 한다 | Unit |
| FR-06 | Severity FATAL 진입 시 safe-state callback이 1회만 호출되어야 한다 | Unit |
| FR-07 | Bounded retry 소진 시 latch 상태로 전이 | Unit + FSM |
| FR-08 | Latched fault는 `AGR_RiskMngr_ClearFault()`로만 해제 | Unit + FSM |
| FR-09 | Error Register (0x1001)는 active descriptor의 err_reg_bits OR | Unit |
| FR-10 | History (0x1003)는 최근 N개 code ring buffer | Unit |
| FR-11 | EMCY frame은 CiA 301 §7.2.7.1 layout 준수 | Wire test |
| FR-12 | Inhibit time (0x1015) 이내 동일 code 재송신 차단 | Timing test |
| FR-13 | 모든 active fault clear 시 error-free EMCY 1회 송신 | Wire test |
| FR-14 | Remote EMCY inject (Master→Slave fault 수집) | Integration |
| FR-15 | NVM backend 미주입 시 black-box silent disable | Unit |
| FR-16 | Fatal fault 발생 시 NVM 즉시 flush 시도 | Integration |
| FR-17 | 주기 (AGR_RISK_MNGR_NVM_PERIODIC_MS)마다 NVM batch flush | Integration |
| FR-18 | Boot 시 이전 세션 last N record 복원 | Power-loss test |
| FR-19 | Subscriber 등록/해제 가능 (up to MAX_SUBSCRIBERS) | Unit |
| FR-20 | Descriptor 등록 시 code 중복 검출 → Init 실패 | Unit |
| FR-21 | Queue overflow 시 oldest drop + meta-fault 자체 보고 | Storm test |
| FR-22 | Framework self-test API가 known fault injection으로 dispatch 검증 | Self-test |
| FR-23 | `GetActiveFaults(buf, max)` polling accessor 제공 | Unit |
| FR-24 | `SubscribeFiltered(cb, min_severity)` 필터 subscriber | Unit |
| FR-25 | safe_state_cb 미설정 시 weak default 동작 (halt + watchdog refresh) | Unit |

---

## 6. Non-Functional Requirements (NFR)

### 6.1 Timing (WCET targets — Cortex-M7 @ 400 MHz 기준)

| API | WCET target | 근거 |
|---|---|---|
| `AGR_RiskMngr_ReportFaultFromISR()` | **≤ 500 ns (200 cycles)** | FES 1ms TIM6 ISR 예산 1µs |
| `AGR_RiskMngr_ReportFault()` | **≤ 5 µs** | Task context, jitter 허용 |
| `AGR_RiskMngr_RunPeriodic()` (empty queue) | **≤ 2 µs** | 10ms 주기 대비 0.02% |
| `AGR_RiskMngr_RunPeriodic()` (16 items drain) | **≤ 200 µs** | Worst case per period |
| `AGR_RiskMngr_OdRead_0x1001()` | **≤ 1 µs** | SDO 처리 중 호출 |
| `AGR_RiskMngr_GetErrorRegister()` | **≤ 500 ns** | 자주 호출될 수 있음 |
| `_PersistBlackBox()` (Fatal flush) | **≤ 50 ms** | Flash write 시간 포함 |

**측정 의무**: Phase 7 이전에 모든 API 실측. 타겟 초과 시 설계 재검토.

### 6.2 Memory Budget

| 영역 | Target | 내역 |
|---|---|---|
| ROM (Core + CiA + EMCY) | **≤ 12 KB** | Sans NVM ref impl |
| ROM (NVM ref impl) | ≤ 4 KB | Internal Flash adapter |
| ROM (V1 compat) | ≤ 2 KB | Deprecated, 단종 예정 |
| RAM (context singleton) | **≤ 2 KB** | `AGR_RISK_MNGR_MAX_ACTIVE=16` 기준 |
| Stack (deepest call) | **≤ 256 B** | ISR path 포함 |

### 6.3 Integrity
- 모든 packed struct: `_Static_assert`로 size 고정
- Descriptor 테이블: Init 시 CRC32 계산, RunPeriodic 마다 1/N 빈도 재검증
- Active 테이블: magic word sentinel (start/end), 주기 점검
- NVM record: CRC16-CCITT 필수

### 6.4 Determinism
- 동적 할당 금지 (malloc/free 금지)
- 무한 루프 금지 — 모든 반복은 compile-time bounded
- Recursive 함수 금지
- Variable-length array 금지
- Function pointer 호출은 **단일 subscriber/safe-state/emcy_tx/nvm** hook 만 (table lookup 아님)

---

## 7. Safety Requirements (SafR)

Framework 자체의 Safety — "Who watches the watcher?"

| ID | Requirement | Realization |
|---|---|---|
| SafR-01 | Framework 자체 fault를 silent하게 삼키지 않는다 | Reserved meta-fault codes (0xF000–0xFFFF) + 자가 보고 |
| SafR-02 | Descriptor table corruption을 검출한다 | Init CRC32 + 주기 검증 (dispatch 10회당 1회) |
| SafR-03 | Active table corruption을 검출한다 | Magic sentinel (start `0xA6A6A6A6`, end `0x6A6A6A6A`) |
| SafR-04 | Subscriber가 blocking되어 RunPeriodic을 지연시키지 않는다 | Subscriber 호출 전 timer start, 초과 시 suspension + meta-fault |
| SafR-05 | Safe-state callback이 RiskMngr API를 재귀 호출해도 deadlock 없다 | `in_dispatch` flag + fault queue defer |
| SafR-06 | Fatal fault 보고 후 safe-state 진입 전에 강제 리셋이 발생해도 NVM에 기록됨 | Synchronous flush 보장 (A trigger) |
| SafR-07 | Fault storm 시에도 framework가 unresponsive 되지 않는다 | Rate limit (token bucket) + overflow meta-fault |
| SafR-08 | POST (Power-On Self Test) 실패 시 initialization이 실패하고 명확한 error code 반환 | `Init` returns negative + stored `last_init_error` |
| SafR-09 | Self-test API가 known fault inject→dispatch 경로를 검증 | `AGR_RiskMngr_SelfTest()` |
| SafR-10 | Watchdog refresh 지점에서 RiskMngr 내부 상태 health 확인 | `AGR_RiskMngr_IsHealthy()` returns bool |

### 7.1 FMEA Summary (Framework 자체)

| Mode | Effect | Detection | Mitigation |
|---|---|---|---|
| Descriptor 테이블 flash bit-flip | 잘못된 severity dispatch | CRC32 (SafR-02) | Meta-fault + safe-state |
| Active 테이블 stack overrun 파괴 | 잘못된 fault state | Magic sentinel (SafR-03) | Meta-fault + reinit attempt |
| Queue producer/consumer race | Fault 손실 | Sequence no + drop count | Atomic operations + `__DMB` |
| Subscriber deadlock | RunPeriodic hang | Timer watchdog (SafR-04) | Subscriber 일시 suspend |
| NVM write 실패 | Black-box 누락 | backend write 리턴 코드 | Meta-fault `NVM_FAILURE` |
| 부팅 직후 stale NVM record | 잘못된 history | CRC + sequence no | Discard invalid, start fresh |

---

## 8. Architecture

```
                ┌────────────── 소비자 (각 모듈의 App/System) ─────────────┐
                │ Descriptor 테이블 등록 / Fault 감지 / Safe-state cb       │
                │ Subscriber 등록 / NVM backend 주입 / EMCY tx 주입          │
                └─────────────────────────┬────────────────────────────────┘
                                          │ Public API
                                          │
    ┌─────────────────────────────────────▼──── AGR_MW/RiskMngr Core ──────────────────────────┐
    │                                                                                           │
    │  ┌─── Engine ────────┐ ┌─── Lifecycle FSM ─┐ ┌─── Self-Protection ────┐                 │
    │  │ Descriptor Reg     │ │ Inactive          │ │ POST + Runtime self-test│                │
    │  │  + CRC32 integrity │ │ Pending           │ │ Descriptor CRC verify   │                │
    │  │ Active Table       │ │ Active            │ │ Active table sentinels  │                │
    │  │  + magic sentinels │ │ Retrying          │ │ Subscriber timer wdg    │                │
    │  │ MPSC Lock-free Q   │ │ Latched           │ │ Meta-fault dispatcher   │                │
    │  │ Priority sort      │ │ Acknowledged      │ └─────────────────────────┘                │
    │  │ Rate limiter       │ │ Cleared           │                                            │
    │  │  (token bucket)    │ └───────────────────┘ ┌─── Observability ──────┐                 │
    │  │ Coalescer          │                       │ Stats (counters, peak) │                 │
    │  │ Dispatcher         │ ┌─── CiA 301 OD ────┐ │ Trace ring (DEBUG)     │                 │
    │  │ Retry FSM          │ │ 0x1001 Err Reg    │ │ Health report          │                 │
    │  │ Latch mgmt         │ │ 0x1003 History    │ └────────────────────────┘                 │
    │  │ Safe-state trigger │ │ 0x1014 COB-ID     │                                            │
    │  │ Subscriber fanout  │ │ 0x1015 Inhibit    │ ┌─── NVM ────────────────┐                 │
    │  └────────────────────┘ └───────────────────┘ │ Record + CRC16         │                 │
    │                                               │ A/B ring + seq no      │                 │
    │  ┌─── EMCY Codec ─────┐  ┌─── V1 Compat ──┐   │ Fatal immediate flush  │                 │
    │  │ Encoder 8B std     │  │ Macro alias    │   │ Periodic batch flush   │                 │
    │  │ Decoder (inject)   │  │ MD wire adapt  │   │ Boot restore           │                 │
    │  │ Inhibit time       │  │ Deprecation    │   └────────────────────────┘                 │
    │  │ Error-free tx      │  │ markers        │                                              │
    │  └────────────────────┘  └────────────────┘   ┌─── Test Hook ──────────┐                 │
    │                                               │ Snapshot / Inject      │                 │
    │                                               │ Mock tick (opt-in)     │                 │
    │                                               └────────────────────────┘                 │
    └─────────────────────┬─────────────────────────────────┬──────────────────────────────────┘
                          │ Backend (compile-time select)   │ External hooks
          ┌───────────────▼─────────────┐      ┌────────────▼──────────────┐
          │ USE_FREERTOS                 │      │ DOP CAN Tx func (inject)  │
          │  StaticQueue + FromISR       │      │ IOIF Flash driver         │
          │ BareMetal                    │      │ HAL_GetTick (inject)      │
          │  Lock-free MPSC + __DMB+DSB  │      └───────────────────────────┘
          └──────────────────────────────┘
```

---

## 9. Fault Lifecycle State Machine (Formal)

모든 fault는 다음 FSM을 따른다:

```
     [INACTIVE] ──────── ReportFault(code) ───────▶ [PENDING] (queued)
          ▲                                              │
          │                                              │ RunPeriodic drain
          │                                              ▼
          │                                          [ACTIVE]
          │                                              │
          │             ┌────────────────────────────────┼──────────────────┐
          │             │                                │                  │
          │          SEV==FATAL?                    policy check        INFO/WARN?
          │             │ yes                            │                  │ no retry
          │             ▼                                ▼                  ▼
          │      [SAFE_STATE_PENDING]          ┌────────────────┐     remain
          │             │                      │    Policy      │    [ACTIVE]
          │    SafeState cb fires once         │                │          │
          │             ▼                      ▼                ▼          │
          │      [SAFE_STATE_LATCHED]    AUTO_RETRY    BOUNDED_RETRY       │
          │             │                      │      (retry_count++)      │
          │             │              re-dispatch     │                   │
          │             │                              ▼                   │
          │             │                        retry ≤ max?              │
          │             │                           yes│ no                │
          │             │                              │   ▼               │
          │             │                              │ [LATCHED]         │
          │             │                              │   │               │
          │             │                              │   │ ClearFault()  │
          │             │                              │   ▼               │
          │             │                              │ [ACKNOWLEDGED]    │
          │             │                              │   │               │
          │             │                              │   │ 후속 clear    │
          │             │                              │   ▼               │
          └─────────────┴──────────── ClearAllRecoverable() ────────────────┘
                                     (FATAL은 Latched만, safe-state 유지)
```

### 전이 규칙 (Transition Table)

| From | Event | Condition | To | Side-effect |
|---|---|---|---|---|
| INACTIVE | ReportFault | — | PENDING | Queue enqueue, occurrence=1 |
| INACTIVE | ReportFault (dup code during pending) | 같은 code PENDING 존재 | PENDING | occurrence++ (coalesce), no re-queue |
| PENDING | RunPeriodic dispatch | — | ACTIVE | Active table 등록, OD update, EMCY tx |
| ACTIVE | ReportFault (same code) | — | ACTIVE | occurrence++, last_ts 갱신, EMCY inhibit |
| ACTIVE | — | SEV==FATAL | SAFE_STATE_PENDING | SafeState callback queued |
| ACTIVE | retry tick | policy==AUTO_RETRY | ACTIVE | re-dispatch 시도 (주기 backoff) |
| ACTIVE | retry tick | policy==BOUNDED_RETRY, retry_count < max | ACTIVE | retry_count++ |
| ACTIVE | retry tick | policy==BOUNDED_RETRY, retry_count == max | LATCHED | Error-free EMCY 보류 |
| ACTIVE | retry tick | policy==LATCH | LATCHED | 즉시 |
| SAFE_STATE_PENDING | SafeState cb return | — | SAFE_STATE_LATCHED | 재진입 guard |
| LATCHED | ClearFault(code) | — | ACKNOWLEDGED | 기록만, active 유지 |
| ACKNOWLEDGED | ClearFault(code) 2회 | — | INACTIVE | Active table에서 제거, OD 재계산 |
| LATCHED | ClearAllRecoverable() | SEV != FATAL | INACTIVE | 일괄 해제 |
| * | Code 조건 미충족 | — | (no-op) | Meta-fault 보고 |

### 불변식 (Invariants)
- **I-01**: SafeState cb는 lifetime 당 **정확히 1회** 호출됨 (다수 FATAL 동시 발생해도)
- **I-02**: LATCHED는 **명시적 Clear 없이 INACTIVE로 돌아가지 않음**
- **I-03**: 0x1001 err_reg는 ACTIVE | SAFE_STATE_* | LATCHED | ACKNOWLEDGED 상태 entry의 OR
- **I-04**: Error-free EMCY는 모든 ACTIVE entry가 해제되었을 때 **정확히 1회** 송신

---

## 10. API Specification (with Formal Contracts)

### 10.0 API Tier 구분 (진입 장벽 완화)

약 40개 API가 있으나 사용자는 **Tier 0 (8개)만 알면 MVP 구현 가능**하다.

| Tier | 용도 | API 수 | 항목 |
|---|---|---|---|
| **0 — MVP** | 최소 동작 | **8** | `Init`, `RegisterDescriptors`, `SetSafeStateCallback`, `Subscribe`, `ReportFault`, `ReportFaultFromISR`, `ClearFault`, `RunPeriodic` |
| **1 — CiA 301** | 표준 호환 슬레이브 | +9 | OdRead/Write_0x1001/0x1003/0x1014/0x1015 (4쌍 + 1 set), `InjectRemoteEMCY` |
| **2 — Ergonomic** | 편의 query/filter | +8 | `GetActiveFaults`, `GetLatestFault`, `SubscribeFiltered`, `IsFaultActive`, `IsFaultLatched`, `GetErrorRegister`, `GetAggregateSeverity`, `ClearAllRecoverable` |
| **3 — Advanced** | 자가 보호 / 관측성 / NVM | +15 | `SelfTest`, `IsHealthy`, `GetStats`, `GetHealth`, `SetNvmBackend`, `PersistBlackBox`, `LoadBlackBoxOnBoot`, `AcknowledgeFault`, `SuppressFault`, `ResetStats`, `GetLastInitError`, Test hooks (4), V1 compat (1) |

**권장 학습 순서**: Tier 0으로 프로토타입 → Tier 1로 CiA 호환 → Tier 2로 UX 개선 → Tier 3으로 production-hardening.

---

각 public API는 다음 annotation을 가진다:

```c
/**
 * @brief       간결한 목적
 * @context     [ISR | Task | Main | Any]       — 호출 가능 context
 * @preempt     [Safe | Unsafe]                  — preemption 허용 여부
 * @reentrant   [Yes | No]                       — re-entrancy 허용
 * @blocking    [No | May-block-up-to-N-ms]      — 블로킹 특성
 * @WCET        ≤ N cycles (@ 400 MHz)           — 측정 대상
 * @pre         호출 전 만족되어야 할 조건
 * @post        호출 후 보장되는 상태
 * @param       ... (in/out 명시)
 * @retval      0       성공
 * @retval      -1      EINVAL (정의된 각 음수 값)
 * @retval      -2      ENOSPC
 * @safety      FR-XX, SafR-YY
 */
```

### 10.0a **FATAL 대응 Latency 상한 — 중요**

본 framework의 SafeState cb는 **"시스템 레벨" 응답**이다. **"HW 레벨 즉시 차단"이 아니다**.

| 레벨 | 담당 | Latency | 예시 |
|---|---|---|---|
| **HW-level (µs)** | **App ISR이 직접 수행** | < 10 µs | PWM disable, Gate driver shutdown, 48V relay cut |
| **System-level (ms)** | **MW SafeState cb** | worst-case = RunPeriodic 주기 (보통 10 ms) | NMT Stop broadcast, Audio, Power latch release |

**권장 ISR 패턴**:
```c
void TIM6_IRQHandler(void) {
    if (OCP_HW_TRIGGERED) {
        HAL_TIM_PWMStop(&htim1, TIM_CHANNEL_ALL);  /* 1. HW 먼저 차단 (µs) */
        GPIO_WritePin(GATE_DRIVER_EN, RESET);       /*    — App 책임 */
        AGR_RiskMngr_ReportFaultFromISR(OCP_HW);    /* 2. 그 다음 MW 통지 */
    }
}
```

**MW가 해주는 것**: 통지 받은 후 최대 10ms 내 SafeState cb 호출 → 시스템 전체 차단 조치.
**MW가 해주지 않는 것**: PWM µs 단위 차단. 이건 App의 ISR-level 책임.

---

### 10.1 Lifecycle API

```c
/**
 * @context   Main (pre-RTOS 권장)
 * @preempt   Unsafe
 * @reentrant No
 * @blocking  May-block-up-to-10-ms (NVM boot restore 포함)
 * @WCET      ≤ 50 ms (NVM 포함), ≤ 500 µs (NVM 없이)
 * @pre       cfg != NULL; descriptors 미등록 상태
 * @post      성공 시 POST 통과; 실패 시 s_last_init_error 설정
 * @safety    SafR-08
 */
int  AGR_RiskMngr_Init(const AGR_RiskMngr_InitConfig_t* cfg);

int  AGR_RiskMngr_RegisterDescriptors(
        const AGR_RiskMngr_FaultDescriptor_t* table, uint16_t count);

/**
 * @note    미설정 시 weak default: 무한 루프 + IWDG refresh (Zephyr 패턴)
 *          override로 production 동작 정의. weak이므로 linker가 자동 선택.
 */
int  AGR_RiskMngr_SetSafeStateCallback(
        AGR_RiskMngr_SafeStateCb_t cb, void* arg);

int  AGR_RiskMngr_Subscribe(AGR_RiskMngr_OnFaultCb_t cb);

/**
 * @brief   Severity 필터링된 subscriber (IEC 60601-1-8 3-tier 대응)
 * @param   min_severity  이 값 이상만 콜백. 예: SEV_WARNING 설정 시 INFO 제외
 */
int  AGR_RiskMngr_SubscribeFiltered(
        AGR_RiskMngr_OnFaultCb_t cb,
        AGR_RiskMngr_Severity_t  min_severity);

/**
 * @context   Task
 * @preempt   Safe
 * @reentrant No
 * @blocking  No (subscriber callback 내부에 의존 — SafR-04 timer로 보호)
 * @WCET      ≤ 200 µs (16 items drain)
 */
void AGR_RiskMngr_RunPeriodic(uint32_t now_ms);
```

### 10.2 Report API

```c
/**
 * @context   Task
 * @preempt   Safe
 * @reentrant Yes (서로 다른 code)
 * @blocking  No
 * @WCET      ≤ 5 µs
 * @pre       Init 완료; code가 descriptor에 등록됨
 * @retval    0   enqueued
 * @retval    -1  EINVAL (unknown code)
 * @retval    -2  ENOSPC (queue full → meta-fault 자체 보고)
 * @retval    -3  EAGAIN (rate limit)
 * @safety    FR-02, FR-21
 */
int  AGR_RiskMngr_ReportFault(uint16_t code, const void* payload, uint8_t len);

/**
 * @context   ISR
 * @preempt   Safe
 * @reentrant Yes
 * @blocking  No
 * @WCET      ≤ 500 ns (200 cycles)
 * @pre       Init 완료
 * @note      payload 없음. Task context에서 detail 재보고 가능
 * @safety    FR-03
 */
int  AGR_RiskMngr_ReportFaultFromISR(uint16_t code);
```

### 10.3 Recovery & Query API
```c
int  AGR_RiskMngr_ClearFault(uint16_t code);         /* LATCHED/ACKNOWLEDGED → INACTIVE */
int  AGR_RiskMngr_ClearAllRecoverable(void);          /* SEV!=FATAL만 */

bool AGR_RiskMngr_IsFaultActive(uint16_t code);
bool AGR_RiskMngr_IsFaultLatched(uint16_t code);
uint8_t AGR_RiskMngr_GetErrorRegister(void);          /* 0x1001 */
uint8_t AGR_RiskMngr_GetActiveCount(void);
AGR_RiskMngr_Severity_t AGR_RiskMngr_GetAggregateSeverity(void);
```

### 10.3b Polling-style API (NEW, AUTOSAR DEM 대응)
```c
/**
 * @brief   현재 active fault 목록을 snapshot으로 pull
 * @context Task
 * @WCET    ≤ 50 µs (MAX_ACTIVE=16 기준)
 * @param   buf   App이 제공하는 출력 버퍼
 * @param   max   버퍼 최대 엔트리 수
 * @return  실제 복사된 엔트리 수 (0..max)
 * @note    Mutex 내부에서 memcpy snapshot — race 없음
 */
uint8_t AGR_RiskMngr_GetActiveFaults(
            AGR_RiskMngr_FaultEvent_t* buf,
            uint8_t max);

/**
 * @brief   가장 최근 이벤트 조회 (non-destructive peek)
 */
int AGR_RiskMngr_GetLatestFault(AGR_RiskMngr_FaultEvent_t* out);
```

### 10.4 CiA 301 OD Access
```c
/* DOP 레이어가 SDO read/write 처리 중 호출. OD Abort code 리턴 규약 준수 */
int AGR_RiskMngr_OdRead_0x1001(uint8_t* out);
int AGR_RiskMngr_OdRead_0x1003(uint8_t subidx, uint32_t* out);
int AGR_RiskMngr_OdWrite_0x1003(uint8_t subidx, uint32_t val);   /* subidx 0, val 0 → clear */
int AGR_RiskMngr_OdRead_0x1014(uint32_t* out);
int AGR_RiskMngr_OdWrite_0x1014(uint32_t val);
int AGR_RiskMngr_OdRead_0x1015(uint16_t* out);
int AGR_RiskMngr_OdWrite_0x1015(uint16_t val);
```

### 10.5 EMCY Consumer
```c
int  AGR_RiskMngr_InjectRemoteEMCY(uint8_t node_id, const uint8_t frame[8]);
```

### 10.6 NVM
```c
int AGR_RiskMngr_SetNvmBackend(const AGR_RiskMngr_NvmBackend_t* be);
int AGR_RiskMngr_PersistBlackBox(void);         /* 동기 flush */
int AGR_RiskMngr_LoadBlackBoxOnBoot(void);
```

### 10.7 Self-Protection API (NEW)
```c
/**
 * @brief     Framework 내부 상태 integrity + dispatch 경로 검증
 * @context   Task
 * @blocking  No
 * @WCET      ≤ 20 µs
 * @safety    SafR-09
 */
int  AGR_RiskMngr_SelfTest(void);

/**
 * @brief     Watchdog refresh 지점에서 호출하여 health 확인
 * @context   Any
 * @WCET      ≤ 200 ns
 * @safety    SafR-10
 */
bool AGR_RiskMngr_IsHealthy(void);

/**
 * @brief     Init 실패 시 마지막 error code 조회
 */
int  AGR_RiskMngr_GetLastInitError(void);
```

### 10.8 Observability API (NEW)
```c
typedef struct {
    uint32_t total_reports;
    uint32_t total_dispatched;
    uint32_t total_dropped_rate_limit;
    uint32_t total_dropped_queue_full;
    uint32_t total_inhibit_skipped;
    uint32_t total_coalesced;
    uint16_t peak_queue_depth;
    uint16_t peak_active_count;
    uint32_t subscriber_timeouts;
    uint32_t self_test_failures;
    uint32_t descriptor_crc_verifications;
    uint32_t descriptor_crc_mismatches;    /* 0이 아니면 즉시 fault */
} AGR_RiskMngr_Stats_t;

int AGR_RiskMngr_GetStats(AGR_RiskMngr_Stats_t* out);
int AGR_RiskMngr_ResetStats(void);       /* 누적 카운터만 */
```

### 10.9 Ack / Suppress API (NEW)
```c
/**
 * @brief  LATCHED fault을 "사용자가 확인함" 표시. 해제는 아님.
 * @note   UI가 오류 메시지 표시 후 "확인" 버튼 시 호출. FSM: LATCHED → ACKNOWLEDGED
 */
int AGR_RiskMngr_AcknowledgeFault(uint16_t code);

/**
 * @brief  특정 code를 일정 시간 suppress (maintenance mode)
 * @note   duration_ms 동안 Report 무시. Meta-stat만 기록.
 */
int AGR_RiskMngr_SuppressFault(uint16_t code, uint32_t duration_ms);
```

### 10.10 V1 Compat
```c
#if defined(AGR_RISK_MNGR_V1_COMPAT)
int AGR_RiskMngr_V1_SendEMCY_MD(uint32_t* err_packet)
    AGR_RISK_V1_DEPRECATED("migrate to AGR_RiskMngr_ReportFault");
#endif
```

### 10.11 Test Hook
```c
#if defined(AGR_RISK_MNGR_TEST_HOOK)
void AGR_RiskMngr_Test_Reset(void);
void AGR_RiskMngr_Test_GetSnapshot(AGR_RiskMngr_TestSnapshot_t* out);
void AGR_RiskMngr_Test_SetMockTick(uint32_t ms);
int  AGR_RiskMngr_Test_InjectActive(const AGR_RiskMngr_FaultEntry_t* e);
void AGR_RiskMngr_Test_ForceDescriptorCorruption(void);   /* SafR-02 테스트 */
#endif
```

---

## 11. Data Structures

### 11.0 Severity vs Priority — 독립된 두 축

혼동 방지: `severity`와 `priority`는 **서로 다른 관심사**이며 독립적으로 선택된다.

| 필드 | 의미 | 영향 | 예시 |
|---|---|---|---|
| `severity` | **얼마나 심각한가** — App 비즈니스 의미 | SafeState 트리거 여부, err_reg_bits 합성, Subscriber 필터 | OverVolt = FATAL (시스템 정지 필요) |
| `priority` | **얼마나 빨리 dispatch되어야 하는가** — MW queue 순서 | Queue sort order (0=먼저) | SD_FULL은 SEV_WARNING이지만 priority=10 (느긋), OCP는 SEV_ERROR + priority=0 (급함) |

**둘은 독립**: WARNING이지만 priority=0인 경우 가능 (UI 지연 민감), FATAL이지만 priority=10도 가능 (이미 SafeState 진입이 보장되므로 dispatch 순서는 덜 중요).

### 11.1 Fault Descriptor
```c
typedef struct {
    uint16_t                    code;           /* unique per registry */
    AGR_RiskMngr_Severity_t     severity;
    AGR_RiskMngr_Policy_t       policy;
    uint8_t                     err_reg_bits;   /* 0x1001 contribution */
    uint8_t                     max_retries;    /* for BOUNDED_RETRY */
    uint16_t                    backoff_ms;
    uint16_t                    rate_limit_per_sec;  /* 0 = unlimited */
    uint8_t                     priority;       /* 0 = highest */
    uint8_t                     flags;          /* e.g., NVM_IMMEDIATE */
    const char*                 name;           /* DEBUG only; release may strip */
} AGR_RiskMngr_FaultDescriptor_t;

_Static_assert(sizeof(AGR_RiskMngr_FaultDescriptor_t) <= 24,
               "Descriptor size budget");
```

### 11.2 Active Fault Entry
```c
typedef enum {
    AGR_RM_STATE_INACTIVE         = 0,
    AGR_RM_STATE_PENDING          = 1,
    AGR_RM_STATE_ACTIVE           = 2,
    AGR_RM_STATE_RETRYING         = 3,
    AGR_RM_STATE_SAFE_STATE_LATCH = 4,
    AGR_RM_STATE_LATCHED          = 5,
    AGR_RM_STATE_ACKNOWLEDGED     = 6,
} AGR_RiskMngr_FaultState_t;

typedef struct {
    uint16_t                       code;
    uint16_t                       occurrence_count;  /* saturating */
    uint32_t                       first_ts_ms;
    uint32_t                       last_ts_ms;
    uint32_t                       suppress_until_ms;
    uint16_t                       descriptor_idx;
    uint8_t                        retry_count;
    uint8_t                        state;             /* AGR_RiskMngr_FaultState_t */
    uint8_t                        emcy_sent;         /* avoid duplicate EMCY */
    uint8_t                        payload_len;
    uint8_t                        payload[AGR_RISK_MNGR_PAYLOAD_BYTES];
} AGR_RiskMngr_FaultEntry_t;

_Static_assert(sizeof(AGR_RiskMngr_FaultEntry_t) <= 32,
               "FaultEntry size budget (32-byte aligned)");
```

### 11.3 Context (with Self-Protection)
```c
#define AGR_RM_SENTINEL_HEAD  0xA6A6A6A6U
#define AGR_RM_SENTINEL_TAIL  0x6A6A6A6AU

typedef struct {
    uint32_t                       sentinel_head;      /* SafR-03 */

    const AGR_RiskMngr_FaultDescriptor_t* registry;
    uint16_t                       registry_count;
    uint32_t                       registry_crc32;     /* SafR-02 */

    AGR_RiskMngr_FaultEntry_t      active[AGR_RISK_MNGR_MAX_ACTIVE];
    uint16_t                       active_count;

    uint16_t                       history[AGR_RISK_MNGR_HISTORY_SIZE];
    uint8_t                        history_head;
    uint8_t                        history_count;

    uint8_t                        err_reg;
    uint32_t                       emcy_cob_id;
    uint16_t                       emcy_inhibit_100us;

    bool                           safe_state_entered;
    bool                           in_dispatch;

    AGR_RiskMngr_OnFaultCb_t       subscribers[AGR_RISK_MNGR_MAX_SUBSCRIBERS];
    uint8_t                        subscriber_count;

    AGR_RiskMngr_SafeStateCb_t     safe_state_cb;
    void*                          safe_state_arg;

    AGR_MW_CanTxFunc_t             emcy_tx;
    uint8_t                        node_id;

    const AGR_RiskMngr_NvmBackend_t* nvm;
    uint32_t                       last_periodic_flush_ms;
    uint32_t                       last_descriptor_verify_idx;

    AGR_RiskMngr_Stats_t           stats;
    int                            last_init_error;
    bool                           is_healthy;

    uint32_t                       sentinel_tail;      /* SafR-03 */
} AGR_RiskMngr_Ctx_t;
```

### 11.4 NVM Record (wire-stable)
```c
typedef struct __attribute__((packed)) {
    uint32_t magic;              /* 'RMNV' = 0x564E4D52 little-endian */
    uint8_t  format_version;     /* ABI versioning — 현재 1 */
    uint8_t  severity;
    uint16_t code;
    uint32_t timestamp_ms;
    uint8_t  retry_count;
    uint8_t  _reserved;
    uint8_t  payload[4];         /* NVM record는 4 byte only (EMCY 5 byte 중 앞 4) */
    uint32_t seq_no;
    uint16_t crc16_ccitt;
} AGR_RiskMngr_BlackBoxRecord_t;

_Static_assert(sizeof(AGR_RiskMngr_BlackBoxRecord_t) == 24,
               "BlackBox record wire size — ABI stable");
```

---

## 12. Configuration (with Validation)

### 12.1 Tunables (agr_mw_conf.h overridable)
```c
/* 최소 기본값 — 모듈이 반드시 override 검토 권장 */
#ifndef AGR_RISK_MNGR_MAX_ACTIVE
#define AGR_RISK_MNGR_MAX_ACTIVE        (16U)
#endif
#ifndef AGR_RISK_MNGR_MAX_DESCRIPTORS
#define AGR_RISK_MNGR_MAX_DESCRIPTORS   (64U)
#endif
#ifndef AGR_RISK_MNGR_HISTORY_SIZE
#define AGR_RISK_MNGR_HISTORY_SIZE      (8U)     /* CiA 301 권장 */
#endif
#ifndef AGR_RISK_MNGR_MAX_SUBSCRIBERS
#define AGR_RISK_MNGR_MAX_SUBSCRIBERS   (0U)
#endif
#ifndef AGR_RISK_MNGR_QUEUE_DEPTH
#define AGR_RISK_MNGR_QUEUE_DEPTH       (32U)    /* 16 → 32 상향 (storm 대비) */
#endif
#ifndef AGR_RISK_MNGR_PAYLOAD_BYTES
#define AGR_RISK_MNGR_PAYLOAD_BYTES     (5U)
#endif
#ifndef AGR_RISK_MNGR_NVM_PERIODIC_MS
#define AGR_RISK_MNGR_NVM_PERIODIC_MS   (1000U)
#endif
#ifndef AGR_RISK_MNGR_SUBSCRIBER_TIMEOUT_US
#define AGR_RISK_MNGR_SUBSCRIBER_TIMEOUT_US  (100U)  /* SafR-04 */
#endif
#ifndef AGR_RISK_MNGR_DESCRIPTOR_VERIFY_FREQ
#define AGR_RISK_MNGR_DESCRIPTOR_VERIFY_FREQ  (10U)  /* RunPeriodic 10회당 1회 */
#endif
```

### 12.2 Compile-time Validation
```c
_Static_assert(AGR_RISK_MNGR_MAX_ACTIVE > 0
             && AGR_RISK_MNGR_MAX_ACTIVE <= 256, "Range");
_Static_assert(AGR_RISK_MNGR_QUEUE_DEPTH >= 8, "Queue too small");
_Static_assert((AGR_RISK_MNGR_QUEUE_DEPTH & (AGR_RISK_MNGR_QUEUE_DEPTH - 1)) == 0,
               "Queue depth must be power of 2 (ring mask)");
_Static_assert(AGR_RISK_MNGR_HISTORY_SIZE >= 1
             && AGR_RISK_MNGR_HISTORY_SIZE <= 254, "CiA 301 0x1003 sub-idx range");
```

### 12.3 Runtime Validation (Init 시)
```c
static int _ValidateDescriptors(const AGR_RiskMngr_FaultDescriptor_t* t, uint16_t n) {
    if (t == NULL || n == 0) return -AGR_RM_ERR_EINVAL;
    if (n > AGR_RISK_MNGR_MAX_DESCRIPTORS) return -AGR_RM_ERR_ENOSPC;

    for (uint16_t i = 0; i < n; ++i) {
        /* 1. code 범위 (reserved range 침범 금지) */
        if (t[i].code >= AGR_ERR_BASE_META) return -AGR_RM_ERR_RESERVED_RANGE;
        /* 2. severity 범위 */
        if (t[i].severity >= AGR_RISK_MNGR_SEV_COUNT) return -AGR_RM_ERR_EINVAL;
        /* 3. policy 범위 */
        if (t[i].policy >= AGR_RISK_MNGR_POLICY_COUNT) return -AGR_RM_ERR_EINVAL;
        /* 4. BOUNDED_RETRY인데 max_retries==0은 모순 */
        if (t[i].policy == AGR_RISK_MNGR_POLICY_BOUNDED_RETRY && t[i].max_retries == 0)
            return -AGR_RM_ERR_INCONSISTENT;
        /* 5. non-INFO인데 err_reg_bits==0은 모순 (CiA 301 필수) */
        if (t[i].severity > AGR_RISK_MNGR_SEV_INFO && t[i].err_reg_bits == 0)
            return -AGR_RM_ERR_INCONSISTENT;
        /* 6. code 중복 검사 (O(n²)이지만 Init 1회) */
        for (uint16_t j = i + 1; j < n; ++j) {
            if (t[i].code == t[j].code) return -AGR_RM_ERR_DUPLICATE_CODE;
        }
    }
    return 0;
}
```

---

## 13. Thread Safety & Concurrency Model

### 13.1 Access Classes
| Data | Readers | Writers | Protection |
|---|---|---|---|
| Descriptor registry | All threads, ISRs | Init only | Immutable after Init (CRC verified) |
| Queue (ring buffer) | Drain task | Multiple producers (Task + ISR) | MPSC lock-free w/ DMB+DSB |
| Active table | RunPeriodic + Query | RunPeriodic only | Writer exclusive; snapshot for query |
| err_reg | Query | RunPeriodic | volatile uint8, atomic read/write |
| Stats | Query + RunPeriodic | RunPeriodic + atomics on ISR-updated fields | Atomic increments |

### 13.2 Producer-Side (ReportFaultFromISR)
```c
/* MPSC lock-free enqueue (BareMetal): */
int _EnqueueFromISR(uint16_t code) {
    uint32_t tail = __LDREXW(&s_ring.tail);
    uint32_t next = (tail + 1) & (DEPTH - 1);
    if (next == __LDREXW(&s_ring.head)) {
        s_ctx.stats.total_dropped_queue_full++;
        _ReportMetaFault(AGR_ERR_RM_QUEUE_OVERFLOW);   /* 자체 */
        return -AGR_RM_ERR_ENOSPC;
    }
    s_ring.buf[tail].code = code;
    s_ring.buf[tail].ts   = HAL_GetTick();
    __DMB();  /* buf write must precede tail publish */
    if (__STREXW(next, &s_ring.tail) != 0) {
        /* contention — retry once, then defer */
        return _EnqueueFromISR_Retry(code);
    }
    __DSB();
    return 0;
}
```

### 13.3 Subscriber Timeout (SafR-04)
```c
static void _InvokeSubscriber(uint8_t i, const AGR_RiskMngr_FaultEvent_t* ev) {
    uint32_t start = _GetTickUs();
    s_ctx.subscribers[i](ev);
    uint32_t elapsed = _GetTickUs() - start;
    if (elapsed > AGR_RISK_MNGR_SUBSCRIBER_TIMEOUT_US) {
        s_ctx.stats.subscriber_timeouts++;
        _ReportMetaFault(AGR_ERR_RM_SUBSCRIBER_TIMEOUT);
        /* Persistent violator: 3회 초과 시 suspend */
        if (++s_subscriber_violation[i] >= 3) {
            s_ctx.subscribers[i] = NULL;  /* suspend */
        }
    }
}
```

### 13.4 Re-entrancy Guard (SafR-05)
- Subscriber나 SafeState cb가 `ReportFault`를 호출하면 `in_dispatch==true` → queue로 defer
- Same-code repeat → coalesce (occurrence++)

### 13.5 Priority Ceiling (FreeRTOS)
- RunPeriodic 태스크 priority는 **모든 reporter보다 높거나 같게** 권장
- 낮으면 priority inversion (reporter가 drain을 기다림)
- 문서화: integration_guide.md

---

## 14. CiA 301 Compliance Matrix

(§8 참조 + 다음 테스트 요구)

| OD Index | Test Case ID | Verification |
|---|---|---|
| 0x1001 | CIA-01 | 각 err_reg bit에 맵핑된 fault inject → read 값 확인 |
| 0x1001 | CIA-02 | 모든 fault clear → 0x00 반환 |
| 0x1003 sub 0 | CIA-03 | Read = 현재 history count |
| 0x1003 sub 1 | CIA-04 | Read = 가장 최근 code + mfr |
| 0x1003 sub 0 | CIA-05 | Write 0 → history clear |
| 0x1003 sub 1+ | CIA-06 | Write → Abort 0x06010002 (RO) |
| 0x1014 | CIA-07 | Default = 0x80 + NodeID |
| 0x1014 | CIA-08 | Write bit[31]=1 → EMCY 비활성 |
| 0x1015 | CIA-09 | Write N → 실제 inhibit N*100µs 적용 |
| EMCY | CIA-10 | Frame layout byte-exact (§7.2.7.1) |
| EMCY | CIA-11 | Error-free (모든 clear 시) exactly-once |
| EMCY | CIA-12 | Inhibit 내 중복 송신 drop |

---

## 15. EMCY Tx — DOP 타입 재사용

```c
/* DOP 또는 공통 MW 타입 헤더에서 */
typedef int (*AGR_MW_CanTxFunc_t)(uint32_t can_id,
                                   const uint8_t* data,
                                   uint8_t len);

/* RiskMngr Init config */
typedef struct {
    uint8_t             node_id;
    AGR_MW_CanTxFunc_t  emcy_tx;      /* NULL이면 EMCY 비활성 (OD만 업데이트) */
    /* ... */
} AGR_RiskMngr_InitConfig_t;
```

---

## 16. NVM Design — Wear, Corruption, ABI

### 16.1 Backend Contract
```c
typedef struct {
    int (*write)(uint32_t offset, const void* data, uint32_t len);
    int (*read)(uint32_t offset, void* data, uint32_t len);
    int (*erase_sector)(uint32_t sector_idx);
    uint32_t sector_size;
    uint32_t sector_count;     /* 최소 2 — A/B ring */
    uint32_t write_unit;       /* flash min-write (e.g., 32 B on H7) */
} AGR_RiskMngr_NvmBackend_t;
```

### 16.2 Ring Layout
```
Sector A: [hdr][rec0][rec1]...[rec N-1]
Sector B: [hdr][rec0]...
Header: { magic, format_version, active_flag, seq_start, sector_size, reserved }

Write sequence:
  1. 현재 active sector에 append (cursor 추적)
  2. Sector가 거의 가득차면 (reserved N records 남음) → B 준비
  3. A의 remaining write → B로 migrate (안 함: 단순히 B start seq = A.end+1)
  4. B에 기록 시작 → A erase
```

### 16.3 Flash Wear Analysis
- Typical H7 internal flash: 10K erase cycles per sector
- Periodic flush 1 Hz × record 24 B = 24 B/s
- Sector size 128 KB → erase frequency = 128 KB / 24 B/s ≈ **60분/sector**
- Lifetime: 10000 × 2 sectors × 60 min ≈ **1.4 year 연속 운전** (최악)
- **완화**: (1) Periodic flush는 pending 있을 때만, (2) Fatal 제외 batch, (3) Rate limit

### 16.4 ABI Versioning
- Record에 `format_version` 필드
- Boot 시 version mismatch → graceful degrade (old records 읽되 새로 기록은 신 포맷)
- format_version 변경은 **Major version bump** 필요 (documented)

### 16.5 Corruption Recovery
- Each record CRC16 검증
- Fail → skip record (not bail out)
- Sector header CRC 별도
- 양 sector 모두 invalid → fresh start (factory state)

### 16.6 Backup SRAM 대안 (권장: Primary for fast path)
- H7 backup SRAM: 4 KB, retain across warm reset (VBAT 필요 X — standby power)
- 용도: 직전 N=64 record (1.5 KB) → **rapid post-mortem without flash wear**
- Flash는 **long-term persistence용** (cold boot 복원)
- 양쪽 동시 지원 가능: 각각 별도 backend 주입

---

## 17. Framework Self-Protection

### 17.1 Descriptor Table Integrity
- Init 시 CRC32 계산, `s_ctx.registry_crc32` 저장
- RunPeriodic마다 `s_ctx.last_descriptor_verify_idx` 진행 → 전체 테이블을 주기적으로 분산 검증
- 10회 RunPeriodic당 1 pass 완료
- 불일치 검출 시:
  - `stats.descriptor_crc_mismatches++`
  - `_ReportMetaFault(AGR_ERR_RM_DESCRIPTOR_CORRUPTED)`
  - `safe_state_cb` 강제 호출

### 17.2 Active Table Sentinel (Stack Overrun 검출)
- Context struct 처음/끝에 magic word
- RunPeriodic 시 검증
- 위반 시 meta-fault `AGR_ERR_RM_CONTEXT_CORRUPTED` + safe-state

### 17.3 POST (Power-On Self Test)
Init 시점에 실행:
1. Context sentinel 초기화 및 검증
2. Descriptor validation (§12.3)
3. Descriptor CRC32 계산 및 baseline 저장
4. Queue backend init 및 ping/pong test
5. Known fault code inject → dispatch → subscriber notify → clear 순환 검증 (`SelfTest`)
6. NVM backend (있으면) header read + CRC check
7. Overall result → `s_ctx.last_init_error` + return

### 17.4 Runtime Self-Test
`AGR_RiskMngr_SelfTest()`:
- 예약된 code `AGR_ERR_RM_SELF_TEST_MARKER = 0xFEED` 사용
- Inject → RunPeriodic 1회 → 해당 code가 ACTIVE 상태인지 확인 → Clear
- 테스트 중 실제 subscriber 호출되므로 보호 플래그 (`in_self_test`)로 UI 등 격리
- WCET ≤ 20 µs 보장

### 17.5 Meta-Fault Reserved Range
```c
#define AGR_ERR_BASE_META                  (0xF000U)
#define AGR_ERR_RM_QUEUE_OVERFLOW          (0xF001U)
#define AGR_ERR_RM_DESCRIPTOR_CORRUPTED    (0xF002U)
#define AGR_ERR_RM_CONTEXT_CORRUPTED       (0xF003U)
#define AGR_ERR_RM_SUBSCRIBER_TIMEOUT      (0xF004U)
#define AGR_ERR_RM_NVM_WRITE_FAILURE       (0xF005U)
#define AGR_ERR_RM_NVM_CORRUPTION          (0xF006U)
#define AGR_ERR_RM_SELF_TEST_FAIL          (0xF007U)
#define AGR_ERR_RM_RATE_LIMIT_EXCEEDED     (0xF008U)
#define AGR_ERR_RM_DESCRIPTOR_MISSING      (0xF009U)
#define AGR_ERR_RM_SELF_TEST_MARKER        (0xFEEDU)  /* internal only */
```

Meta-fault의 descriptor는 MW가 **자체 포함** (App이 등록하지 않음).
Severity=ERROR, Policy=LATCH로 설정 — 무시할 수 없도록.

### 17.6 Error Code Codification Layering (Decision #26, L2/L3 분리)

MW는 **분류 체계와 예약 범위**만 정의. 제품 코드는 각 모듈이 소유한다.

```
AGR_MW/RiskMngr/Inc/
├── agr_error_base.h          ← L2: MW 소관
│   ├── CiA 호환 범위:
│   │   #define AGR_ERR_BASE_GENERIC      0x1000
│   │   #define AGR_ERR_BASE_CURRENT      0x2000
│   │   #define AGR_ERR_BASE_VOLTAGE      0x3000
│   │   #define AGR_ERR_BASE_TEMPERATURE  0x4000
│   │   #define AGR_ERR_BASE_COMM         0x8100
│   ├── AGR 제조사 영역:
│   │   #define AGR_ERR_BASE_AGR_SENSOR   0x5100
│   │   #define AGR_ERR_BASE_AGR_CONTROL  0x5200
│   │   … (범위만)
│   ├── 예약/Meta:
│   │   #define AGR_ERR_BASE_META         0xF000   (MW 자체 fault)
│   │   #define AGR_ERR_RM_QUEUE_OVERFLOW 0xF001
│   │   …
│   └── ❌ 제품 코드 포함 금지 (AGR_ERR_BATT_OVER_VOLT 등)
│
└── agr_risk_mngr_compat.h    ← L2 (V1 alias만)

각 모듈의 System Layer:         ← L3: App 소관
├── CM/System/Config/cm_error_codes.h
│   #include "agr_error_base.h"
│   #define AGR_ERR_BATT_OVER_VOLT      (AGR_ERR_BASE_VOLTAGE | 0x010)
│   #define AGR_ERR_BOARD_OT            (AGR_ERR_BASE_TEMPERATURE | 0x010)
│   … (CM 전용 코드들)
│
├── XM10/System/Config/xm_error_codes.h
├── EMG_Hub/System/Config/emg_error_codes.h
├── FES_Hub/System/Config/fes_error_codes.h
└── … (각 모듈별 독립 파일)
```

**판정 규칙 (Node ID 정책과 동일)**:
- MW가 제공하는 것: 분류 체계 (어느 범위가 전류/전압/온도/통신), 예약 영역, 중복 방지 가이드
- App이 소유하는 것: 제품별 코드 할당, 의미 부여, Descriptor 등록

**충돌 방지**: 범위별 base를 MW가 고정하므로 모듈 간 코드 충돌 구조적 불가.

---

## 18. Fault Storm Handling

### 18.1 Token-Bucket Rate Limiter (per code)
- Descriptor에 `rate_limit_per_sec` (0=unlimited)
- 각 descriptor마다 `tokens_x1000[i]` 카운터 (×1000 for fractional)
- RunPeriodic 시 충전: `tokens += rate_limit * 1000 * dt_ms / 1000`
- Report 시 소비: `tokens -= 1000`
- Tokens < 0 → drop + `stats.total_dropped_rate_limit++`

### 18.2 Coalescing
- Same code already ACTIVE: `occurrence_count++` (saturating), `last_ts_ms` 갱신
- Queue에 re-enqueue 없음
- `stats.total_coalesced++`

### 18.3 Overflow Policy
- Queue full → **oldest-drop** (새 보고 우선) + meta-fault
- Active table full → **lowest-priority-drop** + meta-fault

### 18.4 Storm Test Case
- 10,000 reports in 100 ms (100k/s rate)
- 기대: 모든 active code는 최소 1회 dispatch, rate limit 동작, meta-fault 단 1회 (first overflow)

---

## 19. Observability & Diagnostics

### 19.1 Stats (§10.8 참조)
- `GetStats` — 24 B struct
- SDO access를 위해 OD 0x2xxx (manufacturer-specific) range 매핑 권장 (consumer가 wiring)

### 19.2 Trace Ring (DEBUG only)
- `AGR_RISK_MNGR_ENABLE_TRACE` 정의 시 ring buffer (128 events × 8 B)
- 각 event: `{ts, code, state_from, state_to}`
- ITM / SWO 또는 SEGGER RTT 출력 hook
- Release 빌드에서는 컴파일 제외

### 19.3 Health Report API (NEW)
```c
typedef struct {
    bool     healthy;
    uint8_t  active_count;
    uint8_t  latched_count;
    uint8_t  err_reg;
    uint32_t uptime_ms;
    uint32_t time_since_last_fault_ms;
} AGR_RiskMngr_HealthReport_t;

int AGR_RiskMngr_GetHealth(AGR_RiskMngr_HealthReport_t* out);
```

---

## 20. V1 Compatibility & Deprecation

| 릴리즈 | V1_COMPAT default | MD adapter | 매크로 alias | 릴리즈 조건 |
|---|---|---|---|---|
| v2.0.0 | ON | Active (양방향) | Active | 본 Plan 완료 |
| v2.1.0 | OFF | Active (opt-in) | Active (opt-in) | 모든 V1 모듈 신 API 채택 완료 |
| v2.5.0 | — | Deprecation warning 강제 | Warning | Soft removal |
| v3.0.0 | — | **제거** | **제거** | Hard removal |

Deprecation warning: `__attribute__((deprecated))` + 컴파일 시 메시지.

---

## 21. Implementation Phases (재편성, 13 작업일)

### Phase 0 — Scaffold & Requirements (1일)
- [ ] 디렉터리 생성
- [ ] 빈 헤더/소스 (guard + doxygen 블록)
- [ ] `agr_mw_conf_template.h` 섹션 추가
- [ ] **SRS 문서 초안** (`docs/srs.md` — §5 FR + §6 NFR + §7 SafR 옮김)
- [ ] **Traceability matrix 초안** (`docs/traceability.csv`)
- [ ] 본 plan commit

### Phase 1 — Types, Dictionary, Compat (1.5일)
- [ ] `agr_risk_mngr_types.h` + 모든 `_Static_assert`
- [ ] `agr_error_dictionary.h` v2 (네임스페이스 + reserved range)
- [ ] `agr_risk_mngr_compat.h` (deprecation markers)
- [ ] `docs/cia301_mapping.md`
- [ ] `docs/deprecation_schedule.md`
- **Gate**: 타입 layout `_Static_assert` 통과 (3 configs)

### Phase 2 — Core Engine + Self-Protection (4일)
- [ ] Backend abstraction (FreeRTOS + BareMetal)
- [ ] MPSC lock-free queue (w/ CAS retry)
- [ ] Descriptor registry + CRC32
- [ ] Active table w/ sentinels
- [ ] Fault Lifecycle FSM (§9 전이 테이블 구현)
- [ ] Rate limiter + coalescer
- [ ] Dispatcher + subscriber fanout + timeout
- [ ] Safe-state trigger + re-entrancy guard
- [ ] Retry FSM (Auto/Bounded/Latch/Shutdown)
- [ ] `Init` + POST
- [ ] `SelfTest` (known fault inject)
- [ ] Stats counters
- **Gate**: FSM invariant tests (host mock) 통과

### Phase 3 — CiA 301 Full (2일)
- [ ] `agr_risk_mngr_cia.c`: 0x1001/0x1003 RW/0x1014 RW/0x1015 RW
- [ ] `agr_risk_mngr_emcy.c`: frame codec + inhibit + error-free
- [ ] `InjectRemoteEMCY` (Master 측)
- [ ] Abort code 규약 준수
- **Gate**: CIA-01~12 테스트 케이스 통과

### Phase 4 — V1 Compat & MD Adapter (1일)
- [ ] `agr_risk_mngr_compat.c`: V1→v2 wire 변환
- [ ] 매크로 alias 전부
- [ ] Deprecation warning 실측
- **Gate**: V1 compat 빌드 경고만 나고 기능 동일 확인

### Phase 5 — NVM Interface + Reference (2일)
- [ ] Backend 계약 헤더
- [ ] Core의 A/B ring writer
- [ ] `agr_risk_mngr_nvm_internal_flash.c`
- [ ] Backup SRAM alternative 어댑터 (H7)
- [ ] `LoadBlackBoxOnBoot` + corruption recovery
- [ ] Fatal immediate flush (A trigger)
- [ ] Periodic batch (C trigger)
- **Gate**: Power-loss simulation 테스트 통과

### Phase 6 — Observability (0.5일)
- [ ] `GetStats`, `ResetStats`, `GetHealth`, `IsHealthy`
- [ ] Trace ring (AGR_RISK_MNGR_ENABLE_TRACE)
- **Gate**: Stats 카운터 정합성 확인

### Phase 7 — Test Hook & Docs (1일)
- [ ] `agr_risk_mngr_test_hook.h`
- [ ] `docs/integration_guide.md`
- [ ] `docs/api_contracts.md` (§10 확장판)
- [ ] `docs/fmea.md`
- [ ] `docs/wcet_report.md` (실측 결과 포함)
- [ ] `README.md` 완성
- [ ] `docs/traceability.csv` 완성

### Phase 8 — Verification Gate (1일)
- [ ] `/fw-review` 통과
- [ ] `/fw-verify` 통과
- [ ] 3 타겟 smoke build
- [ ] WCET 측정 (Cortex-M7 @ 400 MHz, Cortex-M4 @ 170 MHz)
- [ ] Memory footprint 측정
- [ ] MISRA-C 검사 (가능한 도구로)
- [ ] 기존 빈 스켈레톤 파일 정리
- **Final Gate**: Acceptance criteria (§24) 100% 통과

**총 공수: ~13 작업일**

---

## 22. Testing Strategy (5 Levels)

### Level 1 — Compile (CI)
- 3 configs smoke build (H7+RTOS, H7+BM, G4+BM)
- Static analysis: cppcheck + clang-tidy + (가능 시) Coverity
- MISRA-C 검사

### Level 2 — Unit (Host-side, test hook 사용)
- FSM transition table 100% coverage
- Config validation (유효/무효 각 케이스)
- Queue overflow / rate limit / coalescing
- CiA OD read/write + abort codes
- EMCY frame byte-exact
- Self-test 경로
- Integrity (descriptor CRC mismatch 시뮬)

### Level 3 — Integration (HIL)
- 실제 FDCAN 루프백으로 EMCY tx/rx
- NVM write/read full cycle
- Boot restore (power cycle)
- DOP 레이어와 OD access 연동

### Level 4 — Timing (Cycle-accurate)
- Logic analyzer + ITM으로 WCET 실측
- ReportFromISR < 500 ns 보장
- RunPeriodic < 200 µs (16 items) 보장

### Level 5 — Soak & Stress
- 10M report iterations 연속 운전
- Fault storm (100k/s)
- Subscriber deliberately slow (SafR-04 검증)
- Brown-out 주기적 발생 (NVM 복원 검증)
- Priority inversion 재현 시도

---

## 23. Documentation Deliverables

| 산출물 | 내용 | Phase |
|---|---|---|
| `plan_risk_mngr_v2.md` | 본 문서 | 0 |
| `srs.md` | Software Requirements Spec (FR + NFR + SafR) | 0 |
| `traceability.csv` | Req ↔ Design ↔ Code ↔ Test 매핑 | 0→7 |
| `integration_guide.md` | 소비자 통합 단계별 가이드 — **L1/L2/L3 경계 예시 필수**, "이건 MW에 넣지 마세요 / 이건 App이 책임" 섹션 | 7 |
| `api_contracts.md` | 모든 API 상세 계약 (pre/post/WCET) | 7 |
| `cia301_mapping.md` | OD 인덱스 매핑 표 | 1 |
| `deprecation_schedule.md` | V1 단종 일정 | 1 |
| `fmea.md` | Framework 자체 FMEA | 7 |
| `wcet_report.md` | 실측 결과 + 타겟 대비 | 8 |
| `README.md` | 프로젝트 개요, 빌드, 의존성 | 7 |

---

## 24. Acceptance Criteria (Final Gate)

Phase 8 완료 판정 체크리스트:

### Functional
- [ ] FR-01 ~ FR-22 모든 테스트 통과
- [ ] SafR-01 ~ SafR-10 모든 테스트 통과
- [ ] CIA-01 ~ CIA-12 모든 테스트 통과

### Non-Functional
- [ ] 모든 API WCET 타겟 이내 (실측 증빙)
- [ ] Memory footprint: ROM ≤ 12 KB, RAM ≤ 2 KB (기본 config)
- [ ] 3 configs 빌드 성공
- [ ] MISRA Mandatory 100%, Required ≥ 95%

### Quality
- [ ] `/fw-review` clean
- [ ] `/fw-verify` clean
- [ ] 문서 10종 완성
- [ ] Traceability matrix 완성 (100% Req 커버)

### Regression Safety
- [ ] V1 compat ON 빌드에서 기존 매크로 모두 resolve됨
- [ ] V1 compat OFF 빌드에서 신 API만으로 동작
- [ ] MD V1 wire format byte-exact (compat adapter)

---

## 25. Risk Register

| # | Risk | Prob | Impact | Mitigation | Owner |
|---|---|---|---|---|---|
| R-01 | WCET 타겟 미달 (ISR path) | Low | High | Early prototyping Phase 2 (ReportFromISR 먼저) | 개발자 |
| R-02 | Queue MPSC lock-free race — 드문 손실 | Low | High | Phase 5에서 soak 10⁷회 | 개발자 |
| R-03 | NVM flash wear 조기 | Low | Med | Rate limit + batch | 개발자 |
| R-04 | Subscriber callback가 평균 100µs 초과 | Med | Med | SafR-04 timer + suspend (이미 설계됨) | 소비자 |
| R-05 | V1 compat alias와 v2 매크로 충돌 | Very Low | Low | Alias는 `#define` — ODR 영향 無 | 개발자 |
| R-06 | Backup SRAM 미사용 타겟 (G4 일부) | Med | Low | Internal flash fallback | 개발자 |
| R-07 | Descriptor 테이블 대형화 시 CRC verify overhead | Low | Low | Distributed verify (1/N per tick) | 개발자 |
| R-08 | CiA 301 해석 차이 (§7.2.7.3 error-free frame) | Low | Med | CAN dump compare with reference tool (CANopenNode) | 개발자 |
| R-09 | FreeRTOS StaticQueue size mismatch | Low | Low | `_Static_assert`로 컴파일 타임 확인 | 개발자 |
| R-10 | 소비자가 `RunPeriodic` 호출 주기 불규칙 | Med | Med | `now_ms` 파라미터 기반 — 주기 의존 최소화 | 소비자 |

---

## 26. Out of Scope / Future Work

1. **Multi-instance** (domain separation) — singleton의 한계 명확할 때 도입
2. **Cryptographic integrity** — Secure boot 도입 시 NVM record 서명
3. **Host test framework 선정** (CMock / Unity / Catch2) — 별도 Plan
4. **각 모듈 마이그레이션** — 모듈별 Plan
5. **CM OOB 버그 fix** — H10 세션
6. **Safety certification artifact 완비** (ISO 26262 WP full set) — 인증 진입 시
7. **Heartbeat-Error Register 통합** — NMT 모듈 협업
8. **OD 0x2xxx manufacturer range에 Stats 노출** — 소비자 결정
9. **Formal verification** (e.g., TLA+ for FSM) — quality uplift 시

---

## 27. Approval & Kickoff

본 Plan v2.0 승인 후:
1. Phase 0 착수 — 디렉터리 + SRS 초안 + conf 템플릿 + traceability 스켈레톤
2. 각 Phase 완료 시 **Gate 리뷰** (단순 checklist 통과, 회의 불필요)
3. Phase 8 완료 후 **Acceptance Review** (§24)
4. Release v2.0.0 tag → 각 모듈 마이그레이션 Plan 착수

---

**End of Plan v2.0**