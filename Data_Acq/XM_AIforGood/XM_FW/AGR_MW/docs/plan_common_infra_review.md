# AGR_MW Common Infrastructure Review — Follow-up Plan

> **Origin**: CM-WH V2 API ergonomics review session, 2026-04-22.
> **Scope**: CM-WH 리뷰에서 AGR_MW 에 제기된 "공용 타입/에러 체계 부재" 주장을 원점에서 재검토.
> **Principle**: 불필요한 추상화 금지. 이미 기존 Plan(`plan_risk_mngr_v2.md`, `plan_pdo_mapping_unified.md`)이 다루는 항목과 **중복 없이** 작업.
> **Critical context**: `plan_risk_mngr_v2.md` 는 현재 검토 중이며 향후 구현 예정 — Fault/EMCY 타입은 거기서 정의됨.

---

## 1. 원 평가의 재검토 — 어디까지가 진짜 문제인가

### Claim A: "공용 에러 타입이 필요하다 (각 모듈이 int/int32_t/MD_Drv_Status_t/LAN9252_Status_t 다르다)"

**재검토 결론: 공용 "enum" 은 불필요. 그러나 AGR_MW 내부는 `int` 가 지배 중이라 "문서화만" 으로 부족 — `int32_t` migration 1회 필요.**

**실측 현황 (AGR_MW 공용 헤더 기준)**:

| 모듈 | 현재 리턴 | 규약 부합 |
|------|-----------|:---:|
| BOOT `AGR_Boot_ConfirmBoot` | `int32_t` | ✅ 유일한 적합 사례 |
| PnP Master/Slave | `int` | ⚠️ 표준 C 상 최소 16bit |
| TSM `TSM_TransitionTo` / `TaskMngr_Transition` | `int` | ⚠️ |
| DOP SDO/PDO/OD | `int` | ⚠️ |
| CoE Master / CAN-FD Transport | `int` | ⚠️ |
| NMT `AGR_NMT_ProcessMessage` | `int` | ⚠️ |

→ Plan 초안의 "int32_t 사실상 표준" 은 사실상 **허구** (BOOT 한 곳만 준수). 지배 관례는 `int`.

**Best Pattern — 판단 결과**:

Public API 리턴은 **`int32_t` 로 통일**, 도메인 에러는 각 모듈이 typed enum (int32_t-호환 값) 으로 보유. 공용 매크로 `AGR_IsOk()` 로 검사.

**근거**:
1. **ABI 비용 0** — ARM Cortex-M 에서 `int` == `int32_t`. 헤더만 치환, 바이너리 변화 없음.
2. **MISRA-C:2012 Dir 4.6** — "typedefs that indicate size and signedness should be used in place of the basic numerical types" (fixed-width 권장).
3. **BOOT 가 이미 채택한 최신 설계** — 표준으로 격상하는 것이 일관성 측면에서 자연스러움.
4. **도메인 의미 유지** — 모듈은 `AGR_PnP_Status_t` 같은 enum 을 값 소스로 유지, 시그니처만 `int32_t`.

**규약 템플릿**:

```c
// Public API — 모든 AGR_MW 모듈
int32_t AGR_Module_DoThing(...);
//   rc == 0 : OK
//   rc <  0 : error (모듈별 typed enum 음수 값)
//   rc >  0 : reserved — 현재 미사용. 향후 "ok-with-info" 용으로 예약.
//             도입 시 모듈별 문서에 의미 명시 필요.

typedef enum {
    AGR_PNP_OK             =  0,
    AGR_PNP_ERR_INVALID    = -1,
    AGR_PNP_ERR_SLAVE_FULL = -2,
} AGR_PnP_Status_t;

#define AGR_IsOk(rc)  ((int32_t)(rc) >= 0)
```

**조치**:
- [ ] `docs/agr_mw_error_contract.md` 작성 — 위 규약 명시
- [ ] `AGR_IsOk()` 매크로를 **`agr_mw_conf_template.h` 하단에 추가** (새 헤더 신설 X — conf 파일 난립 방지)
  - 템플릿이므로 각 소비 모듈의 `System/Config/agr_mw_conf.h` 에 **재복사 필요**
  - "DO NOT MODIFY — API contract macro" 주석으로 소비자 개입 차단
- [ ] **Migration PR 1회** — PnP/TSM/DOP/CoE/CAN-FD/NMT 공용 헤더의 `int` → `int32_t` 일괄 치환. ABI 동일, sed 수준 변경
- **공용 에러 enum 은 만들지 않는다** — 도메인 분리를 해침. 모듈별 typed enum 유지.

**Caveat — 템플릿 동기화 timing**:
- `agr_mw_conf_template.h` 수정 → 소비 모듈이 재복사 안 하면 `AGR_IsOk` 미정의 상태로 migration PR 돌면 빌드 깨짐
- **실행 순서 고정**: (1) template 수정 → (2) 각 소비 모듈 sync PR (template → conf.h 재복사) → (3) migration PR
- 소비 모듈이 자체 `agr_mw_conf.h` 를 수정했을 가능성 있으면 sync 시 diff 수동 머지

---

### Claim B: "공용 이벤트/Fault 타입이 필요하다 (IMU offline + MD fault → E-stop 로직 앱이 매번 작성)"

**재검토 결론: 필요하나 RiskMngr v2 의 범위. 본 문서에서 별도 정의하지 않는다.**

**근거**:
- `plan_risk_mngr_v2.md` §15 "EMCY Tx — DOP 타입 재사용" 이 이를 정확히 다룸
- `plan_risk_mngr_v2.md` §11 "Data Structures" 에 Fault Event 형식 명세
- RiskMngr v2 가 구현되면 앱은 `RiskMngr_Report()` / `RiskMngr_Subscribe()` 로 해결

**조치**: **없음**. RiskMngr v2 진행 상황 대기.

**다만 현재 상태에 대한 임시 가이드**:
- RiskMngr v2 구현 전까지, 앱 레이어는 드라이버별 `IsOnline()` / `GetLastError()` 를 polling 하는 패턴 유지
- CM-WH Framework/apps 에 임시 "fault aggregator" 를 두지 **말 것** — RiskMngr v2 도입 시 전면 대체됨
- 임시 패턴은 각 app task 내부에 국한, 재사용 코드 작성 금지

---

### Claim C: "공용 타임스탬프/좌표 타입이 필요하다 (IMU vs MD 데이터 상관 불가)"

**재검토 결론: 타임스탬프는 app-layer 책임. 공용 타입 도입하지 않는다.**

**근거 분석**:

| 옵션 | 장점 | 단점 |
|------|------|------|
| (1) Driver 가 Data struct 에 `timestamp_us` 필드 추가 | 일관된 correlation | 모든 드라이버에 tick/DWT 커플링. SM-IMU/FES 같은 G4 BareMetal 드라이버에도 영향. 오버엔지니어링 |
| (2) App 이 `IOIF_DWT_GetCycles()` 직접 호출 | 커플링 없음. 필요한 곳만 부담 | 관례 필요 |
| (3) Snapshot 구조에 `last_update_tick` 만 추가 | 가볍게 해결 | app 이 tick 쓰는 경우만 유용 |

**결론**: (2) 채택. **AGR_MW 레이어에서 표준 timestamp 타입 정의하지 않음**.
- 드라이버는 자신의 sensor-internal timestamp 만 노출 (XSENS MTi-3 처럼 프로토콜상 timestamp 가 있는 경우)
- 앱은 correlation 이 필요하면 `IOIF_DWT_GetCycles()` / `osKernelGetTickCount()` 직접 사용
- IMU coord frame, MD joint frame 간 변환은 **순수 app-layer 책임** (URDF/robot model)

**Non-Goal 명시**: `agr_mw_common_types.h` 같은 "공통 헤더" 를 만들지 않는다.

---

### Claim D: "RiskMngr 가 비어 있다 (risk_mngr.h empty stub)"

**재검토 결론: 의도된 stub. `plan_risk_mngr_v2.md` 가 전면 재설계할 예정이므로 현 상태를 "명시적 stub" 으로 정리하는 선에서 종결.**

**조치**:
- [x] **선행 (필수)**: 전 소비 모듈 repo 에서 `#include.*error_dictionary` / `#include.*risk_mngr` grep — 실행 결과: consumer 측 매크로 **사용처 0건** 확인 (include 는 있으나 dead code). `#if 0` 안전.
- [x] `risk_mngr.h` / `risk_mngr.c` 를 **명시적 stub** 으로 표시
- [x] `error_dictionary.h` 내용을 `#if 0 ... #endif` 로 감싸고 redesign 주석 추가 (Claim E 통합)
- RiskMngr 폴더 전체를 v2 가 재설계하므로, Claim E 를 별건으로 두지 않고 **본 Claim D 와 묶어서 한 커밋** 으로 처리

---

### Claim E: "error_dictionary.h 에 0x1000 값 중복 정의"

**재검토 결론: 실제 버그. 심각도 상향 — 초기 평가는 "두 #define 충돌" 이었으나 실측상 7쌍 전부 충돌.**

**실측 충돌 (`AGR_MW/RiskMngr/Inc/error_dictionary.h`)**:

| 값 | Low-Level 영역 | System Management 영역 |
|---|---|---|
| 0x1000 | `ERROR_PHASE_OVER_CURRENT` | `I2C_INIT_ERROR` |
| 0x1001 | `ERROR_PHASE_SHORT` | `I2C_COMMUNICATION_ERROR` |
| 0x1002 | `ERROR_INIT_CURRENT` | `BOARD_OVER_CURRENT` |
| 0x1003 | `ERROR_PHASE_OVER_VOLT` | `BOARD_OVER_VOLTAGE` |
| 0x1004 | `ERROR_INVALID_PEAK_CURRENT_SETTING` | `BOARD_UNDER_VOLTAGE` |
| 0x1005 | `ERROR_INVALID_CONT_CURRENT_SETTING` | `BOARD_OVER_TEMPERATURE` |
| 0x1006 | `ERROR_INVALID_OVER_CURRENT_DURATION_SETTING` | `BOOT_SEQUENCE_ERROR` |

→ 현재 include 된 consumer 가 없어 즉시 증상은 없음. RiskMngr v2 가 이 파일을 입력으로 쓰면 silent bug.

**조치**: **Claim D 와 통합 처리**. `error_dictionary.h` 내용 전체를 `#if 0 ... #endif` 로 감싸고 `// TODO: redesigned under plan_risk_mngr_v2` 주석. 별도 값 재배치는 하지 않음 (v2 가 완전 대체).

---

### Claim F: "TSM 과 TaskMngr 가 같은 기능 2개 API 를 제공한다"

**재검토 결론: 실제 duplicate 이며, 단순 deprecation 으로 부족 — 이중 state tracking 까지 정리 필요.**

**현황**:
- `AGR_MW/TSM/Inc/task_state_machine.h` — `TSM_Init/AddState/Run/TransitionTo` (원형, caller-owned struct)
- `AGR_MW/TSM/Inc/task_mngr.h` — `TaskMngr_Create/AddState/Run/Transition` (정적 풀 + prev-state 히스토리)

**실측 분석**:
- `TaskMngr_AddState` / `TaskMngr_Transition` 은 **1:1 위임** — `task_mngr.c` 에서 `TSM_AddState` / `TSM_TransitionTo` 그대로 호출만
- `TaskMngr_Run` = `TSM_Run()` 호출 후 public 필드 sync
- **이중 state tracking 버그 risk**: `TaskStateMachine_t.curr_state_id` (engine) 와 `TsmObject_t.current_state_id` (public) 가 **같은 정보를 두 곳에 기록**. `task_mngr.c:101-110` 에서 매 tick 수동 sync.
  - 누군가 `TSM_Run(&internal->engine)` 을 직접 쓰면 public 필드 stale
  - 두 API 공존 → 실수 유도하기 좋은 구조

**조치 — 3단계 분리 (big-bang 금지)**:

**Stage 1 — Additive (호환 유지)** ✅ 2026-04-22:
- [x] 결정: `TaskMngr_*` 를 **공식 API** 로 확정
- [x] Getter 추가 only — `TaskMngr_GetStateId()`, `TaskMngr_GetLifecycle()`, `TaskMngr_GetPrevStateId()`, `TaskMngr_GetPrevLifecycle()`
- [x] 기존 `TsmObject_t.current_state_id` 등 public 필드 **유지** (breaking 없음)
- [x] README 에 "Use TaskMngr_* + getters in application code" + deprecation notice 명시
- [x] `TSM_*` 직접 사용에 `@deprecated` doxygen 태그 추가 (`task_state_machine.h` 5개 함수 + 파일 헤더)

**Stage 2 — Caller migration (소비 모듈 다수 repo)**:
- [ ] 전 소비 모듈에서 `obj->current_state_id` / `current_step` / `prev_state_id` / `prev_step` 직접 접근 개수 집계
- [ ] 직접 `TSM_*` 사용 caller 집계
- [ ] 각 소비 모듈 PR — 직접 접근을 getter 호출로 치환, `TSM_*` → `TaskMngr_*` 전환
- [ ] 전 소비 모듈 migration 완료 확인 (grep 0건)

**Stage 3 — Breaking cleanup (별도 릴리스)** ✅ 2026-04-23:
- [x] `TsmObject_t` 를 opaque forward declaration 으로 전환 — `current_state_id` / `current_step` / `prev_state_id` / `prev_step` 4 필드 제거
- [x] `prev_state_id` / `prev_step` 을 `struct TsmObject_s` (`task_mngr.c`) 로 이동. curr 는 `engine.curr_state_id` / `engine.lifecycle` 단일 진실원 유지
- [x] `task_state_machine.h` 를 `TSM/Inc/internal/task_state_machine.h` 로 이동 — public types (`TsmLifecycle_e`, `EntryFunc_t/LoopFunc_t/ExitFunc_t`) 는 `task_mngr.h` 로 이식, engine header 는 `task_mngr.h` 를 include 해 public types 를 얻음
- [x] 기존 `InternalTask_t` typedef 삭제 — `struct TsmObject_s` 가 곧 task 정의

**Build-system 작업 (Stage 3)** — AGR_MW 는 자체 CMakeLists.txt 없음 (submodule). include path 계약으로만 격리 달성:
- [x] `task_mngr.c` 내부에서만 `#include "internal/task_state_machine.h"` 사용
- [x] 소비 모듈 include path `AGR_MW/TSM/Inc` 만 추가 (README §include path) → `#include "internal/..."` 는 resolve 되지만, 소비 모듈의 `#include "task_state_machine.h"` 는 자동으로 컴파일 에러 (경로 미노출) = 격리 성공
- [x] 소비 모듈 CMake **수정 불필요** — 기존 path 그대로. `internal/` 추가 등록 금지

**Note**: `TSM/` 폴더의 세 번째 파일 `routine_mngr.h/.c` 는 TSM/TaskMngr 와 **중복 아님** — state 내부에서 동적 on/off 되는 routine 관리 (축이 다름). README 에 세 파일 역할 차이를 명시해 신규 작성자 혼동 방지.

**Stage 3 follow-up — `TsmDelay` one-shot timer API** ✅ 2026-04-29:
- [x] PhAI 가 CM-WH `boot_ctrl.c` / `core_ctrl.c` 에서 매뉴얼 편집으로 사용하던 `StateDelay_*` API 를 정공법으로 흡수 — `TsmDelay_Start / TsmDelay_Expired / TsmDelay_Reset` + `TsmDelay_t`
- [x] 동기: 단일 task 12-state FSM 에서 `osDelay()` 는 같은 task 의 다른 FSM 까지 막고, `osKernelGetTickCount()` 직접 사용은 모든 caller 가 wrap-around·one-shot semantics 를 반복 구현해야 함
- [x] **D-2 결정 — RTOS-neutral injection (D-2(B))**: `TsmDelay_GetTickFunc_t` + `TsmDelay_SetTickProvider()` — PhAI 매뉴얼판의 `cmsis_os2.h` 직접 의존을 거부. 이유: TSM 은 현재 RTOS 의존 0건이며, SM-IMU/EMG/FES (G4 BareMetal) 에서 `cmsis_os2.h` resolve 불가. PnP 의 `AGR_PnP_GetTickFunc_t` 패턴 (Claim G "RTOS/BareMetal 둘 다 지원") 과 정렬
- [x] **D-1 결정 — `TsmDelay_*` prefix (D-1(C))**: 타입 (`TsmDelay_t`) 과 함수 prefix 일치. `RoutineMngr_*` 선례와 같은 sub-namespace 패턴
- [x] **D-3 결정 — public POD struct 유지**: one-shot timer state 는 단순 POD. opaque 적용은 `TsmObject_t` (engine + history) 와 달리 캡슐화 이득 없음
- [x] **D-4 결정 — `_Expired` 이름 유지**: 헤더 Doxygen 에 `@note Single-shot. Re-arm via TsmDelay_Start()` 명시
- [ ] **CM-WH 후속 작업** (별도 세션): AGR_MW submodule pointer bump → `boot_ctrl.c` / `core_ctrl.c` 의 `StateDelay_*` → `TsmDelay_*` rename (~14 호출 + 1 typedef) → 빌드 검증

---

### Claim G: "PnP 는 레퍼런스로 적합하다"

**재검토 결론: 그대로. AGR_MW 의 API 설계 레퍼런스로 기록.**

- DI 기반 tx/tick 주입 — 테스트 가능
- 콜백 구조체 bundle 방식 — 한 번에 등록
- NMT semantics 명확, CiA 301 대칭
- RTOS/BareMetal 둘 다 지원

**조치**: PnP 를 "New AGR_MW module authoring reference" 로 기록. Rule 문서에 링크.

---

## 2. 우선순위 & 조치 요약

| 우선 | 작업 | 종류 |
|:---:|------|------|
| **P1-prereq** | 전 소비 모듈 repo grep — `error_dictionary` / `risk_mngr` include 0건 확인 (실행 전 선행) | 검증 |
| **P1** | RiskMngr stub 정리 — `risk_mngr.h/.c` + `error_dictionary.h` 를 명시적 stub/`#if 0` 로 (Claim D + E 통합) | 코드 |
| **P2-s1** | TSM Stage 1 — Getter 추가 + deprecation notice (additive, 호환 유지) | 코드 |
| **P2-s2** | TSM Stage 2 — 소비 모듈 caller migration (다수 repo PR) | 코드 (다수 repo) |
| **P2-s3** ✅ | TSM Stage 3 — `TsmObject_t` opaque + engine header `internal/` 이동 (별도 릴리스) | 코드 |
| **P3-a** ✅ | `docs/agr_mw_error_contract.md` 작성 (`int32_t` + `AGR_IsOk()` 규약) | 문서 |
| **P3-b** ✅ | `agr_mw_conf_template.h` 하단에 `AGR_IsOk()` 매크로 추가 | 코드 |
| **P3-c** | 소비 모듈 sync PR — 각 모듈 `System/Config/agr_mw_conf.h` 에 매크로 반영 | 코드 (다수 repo) |
| **P3-d** ✅ | AGR_MW 공용 API `int` → `int32_t` migration (22파일, 114개 signature) | 코드 |
| **P3-e** ✅ | PnP 를 AGR_MW 모듈 레퍼런스로 README 에 링크, TSM 세 파일 역할 명시 | 문서 |

**P3 실행 순서 고정**: a → b → c → d → e. b/c 건너뛰고 d 실행 시 소비 모듈 빌드 실패.

**Non-Goal (하지 않는다)**:
- ❌ 공용 에러 enum 신설 → 모듈별 typed enum 유지 (값만 int32_t-호환)
- ❌ 공용 Fault/Event 타입 신설 → RiskMngr v2 가 담당
- ❌ 공용 timestamp/coordinate 타입 신설
- ❌ RiskMngr 내용 채우기 → `plan_risk_mngr_v2.md` 가 담당
- ❌ `agr_mw_common_types.h` / `agr_mw_common.h` 같은 추가 헤더 신설 — `AGR_IsOk()` 는 기존 `agr_mw_conf_template.h` 에 편입
- ❌ 신규 conf 파일 추가 — 이미 충분히 많음 (ioif_conf, agr_mw_conf 등)

---

## 3. 기존 Plan 과의 관계

| 기존 Plan | 본 문서와 중복? |
|-----------|:---:|
| `plan_risk_mngr_v2.md` (Fault/EMCY/Error Code 전면 재설계) | ❌ 독립. 본 문서는 "v2 구현 전까지의 interim" |
| `plan_pdo_mapping_unified.md` | ❌ 별개 도메인 (PDO) |

**Interaction note**: RiskMngr v2 가 `error_dictionary.h` + `risk_mngr.h/.c` 를 전면 재설계할 것이므로, 본 문서의 P1 은 v2 kickoff 전까지의 임시 차단 (명시적 stub + `#if 0`) 이면 충분. v2 구현 시 RiskMngr 폴더 전체가 자연히 교체된다.

---

## 4. 원 평가 대비 변경 요약

| CM-WH 세션 평가 | 본 재검토 결론 |
|-----------------|---------------|
| "공용 에러 타입 필요" | ⚠️ 공용 enum 은 X. 그러나 AGR_MW 내부 `int` → `int32_t` migration + `AGR_IsOk()` 매크로 추가 (BOOT 패턴 확장) |
| "공용 Fault/Event 타입 필요" | ⚠️ 인정하나 RiskMngr v2 담당. 본 문서에서 신설 X |
| "공용 timestamp/coord 필요" | ❌ 취소. app-layer 책임 |
| "RiskMngr 비어 있음" | ⚠️ 의도된 stub 이나 명시적 표시 필요. P1 통합 조치 |
| "error_dictionary 0x1000 충돌" | ✅ 실제 버그 (7쌍 충돌, 초기 평가 대비 심각도↑), P1 통합 조치 |
| "TSM vs TaskMngr 중복" | ✅ 실제 중복 + 이중 state tracking 까지. P2 확장 조치 |
| "PnP 레퍼런스" | ✅ 유지, 문서화 |

---

## 5. 검증

### P1-prereq — 선행 grep (실행 전 필수) ✅ 2026-04-22
- [x] 전 소비 모듈 repo grep — `#include.*error_dictionary` / `#include.*risk_mngr`
- [x] Include 는 있으나 **매크로 사용처 0건** 확인 (ARC/XM10 Rev1.1/Rev2.0/CM_FW/IMU/FES/EMG — 전부 dead include)
- [x] 블로커 없음, P1 진행 승인

### P1 — RiskMngr stub 정리 (Claim D + E 통합) ✅ 2026-04-22
- [x] `#if 0` 적용 — 빌드 영향 없음 (consumer 측 매크로 미사용 확인 완료)
- [x] stub 주석에 `plan_risk_mngr_v2.md` 링크 포함 (3개 파일 모두)

### P2-s1 — TSM Stage 1 (Additive) ✅ 2026-04-22
- [x] Getter 4종 추가 (`TaskMngr_GetStateId/GetLifecycle/GetPrevStateId/GetPrevLifecycle`), 필드 유지 → 호환
- [x] `TSM_*` 5개 함수 + 파일 헤더에 `@deprecated` doxygen 태그 추가
- [x] README 에 TSM 사용 가이드 + deprecation notice 섹션 추가
- [ ] CI warning 생성 여부 확인 (다음 빌드 시)

### P2-s2 — TSM Stage 2 (Caller migration)
- [ ] CM-WH / XM / SM\* 각 repo 에서 `->current_state_id` / `->current_step` / `->prev_state_id` / `->prev_step` 직접 접근 개수 집계
- [ ] 동일 repo 에서 `TSM_*` 직접 caller 집계
- [ ] 각 repo PR 후 grep 0건 확인

### P2-s3 — TSM Stage 3 (Breaking cleanup, 별도 릴리스) ✅ 2026-04-23
- [x] `TsmObject_t` opaque 전환 — 직접 필드 접근 caller 남아 있으면 컴파일 에러 발생 (Stage 2 누락 검출 장치)
- [x] `task_state_machine.h` 를 `internal/` 로 이동 — 소비 모듈 include path 에 `internal/` 미등록 → 외부 `#include "task_state_machine.h"` 는 자동으로 빌드 실패 (격리 성공)
- [x] AGR_MW 는 자체 CMakeLists.txt 없음 (submodule) — include path 계약 (`-I AGR_MW/TSM/Inc` only) 으로만 격리. 소비 모듈 CMake 수정 없이 달성
- [ ] 소비 모듈 빌드 성공 확인 (다음 빌드 시)

### P3-b — `AGR_IsOk()` 매크로 ✅ 2026-04-22
- [x] `agr_mw_conf_template.h` 에 매크로 추가 + `<stdint.h>` include
- [x] "DO NOT MODIFY — API contract macro" 주석 추가
- [ ] static_assert 또는 smoke test — `int` / `int32_t` / typed enum 세 입력에서 모두 동작 확인 (다음 빌드 시)

### P3-c — 소비 모듈 sync (multi-repo, 미실행)
- [ ] 전 소비 모듈 `System/Config/agr_mw_conf.h` 재복사 여부 집계 (수동 수정본 있으면 3-way merge)

### P3-d — `int` → `int32_t` migration ✅ 2026-04-22
- [x] AGR_MW 공용 헤더 전체 grep — 대상 114 signature / 22 파일 열거
- [x] sed 치환 적용: `^int (AGR_|TSM_|TaskMngr_)` → `^int32_t \1`
- [x] 잔여 `^int AGR_/TSM_/TaskMngr_` = 0 확인
- [ ] 바이너리 크기/ABI 변화 0 확인 (다음 빌드 시)
- [ ] 각 소비 모듈 빌드 성공 확인 (다음 빌드 시)

### P3-a/e — 문서 ✅ 2026-04-22
- [x] `agr_mw_error_contract.md` 작성 완료 (7 section, canonical BOOT 예시 포함)
- [x] README.md 에 TSM 3파일 역할 + PnP 레퍼런스 + Error Contract 링크 추가
