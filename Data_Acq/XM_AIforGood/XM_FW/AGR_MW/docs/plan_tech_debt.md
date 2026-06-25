# AGR_MW Tech Debt Cleanup Plan

> **Origin**: 2026-04-22 — `plan_common_infra_review.md` 완료 후 잔여 기술부채 감사
> **Scope**: 기존 plan 이 다루지 않는 AGR_MW-internal 기술부채
> **Principle**: AGR_MW-internal 항목 우선 처리, multi-repo 범위는 plan 항목으로 분리

---

## 1. 감사 결과 요약

### 1.1 AGR_MW-내부 처리 가능

| # | 항목 | 위치 | 분류 | 우선순위 |
|---|------|------|------|:---:|
| **T1** | `routine_mngr.c` 주석처리된 legacy dead code | `TSM/Src/routine_mngr.c:109-209` | 정리 | **P1** |
| **T2** | `AGR_NMT_Init` deprecation (internal-only 사용) | `DOP/agr_nmt.h:127`, `agr_nmt.c:56` | 정리 | **P2** |

### 1.2 Design 필요 — 별도 처리

| # | 항목 | 위치 | 분류 | 우선순위 |
|---|------|------|------|:---:|
| **T3** | Async SDO Master response handling | `DOP/Transport/Serial/agr_dop_serial.c:296` (+ CAN_FD/UDP 공통 gap) | 설계 필요 | **P3** (deferred) |

### 1.3 기존 Plan 이 이미 다루는 항목 (본 plan 제외)

- RiskMngr 전면 재설계 → `plan_risk_mngr_v2.md`
- TSM Stage 2/3 migration → `plan_common_infra_review.md` §Claim F
- Consumer sync / int32_t multi-repo → `plan_common_infra_review.md` §P2-s2, §P3-c
- PDO Mapping 통합 → `plan_pdo_mapping_unified.md`

---

## 2. 세부 항목

### T1 — routine_mngr.c legacy dead code 제거

**현황**:
- `TSM/Src/routine_mngr.c:1-107` — 활성 구현 (`RoutineMngr_Init/Add/SetEnable/RunAll`)
- `TSM/Src/routine_mngr.c:109-209` — **주석 처리된 레거시 구현** (`CreateRoutineEntity`, `InitRoutine`, `EntRoutines`, `RunRoutines`, `ExtRoutines`, `ClearRoutines`, `PushRoutine`)
- 활성 구현으로 이미 대체됨 → legacy 보존 가치 없음
- TODO 3건 (`//TODO: routine func exception handling`) 모두 이 dead code 내부

**조치**:
- [ ] `routine_mngr.c` lines 109-209 삭제
- [ ] 최종 파일 ~110 줄로 축소
- Git history 로 레거시 구현 복구 가능하므로 안전

**위험**: 없음 (코드 제거 아님, 주석 제거)

---

### T2 — `AGR_NMT_Init` deprecation

**현황**:
- `AGR_NMT_Init(inst, timeout_ms)` — basic 초기화 (timeout 만)
- `AGR_NMT_InitEx(inst, timeout_ms, node_id, on_state_changed, on_timeout, user_ctx)` — 완전 초기화
- `_InitEx` 내부에서 `_Init` 호출 (`agr_nmt.c:83`) — `_Init` 은 internal helper 성격
- **External caller 조사** (2026-04-22 grep):
  - AGR_MW 내부: agr_nmt.c:83 (InitEx→Init) 만
  - Consumer repo: 호출 0건 (문서 예시 / rule 문서 스니펫만)
- → `_Init` 는 사실상 **private helper** 상태

**조치 (3단계 safe progression, Claim F 패턴)**:
- Stage 1 (additive, 호환 유지):
  - [ ] `AGR_NMT_Init` 에 `@deprecated Use AGR_NMT_InitEx() with node_id + callbacks. Basic Init will be made internal.` 태그
  - [ ] README / agr_nmt.h docstring 예시도 `_InitEx` 로 교체
- Stage 2 (consumer grep, 외부 caller 없음 재확인):
  - [ ] 최종 grep 검증 (multi-repo 포함)
- Stage 3 (breaking, 별도 릴리스):
  - [ ] `AGR_NMT_Init` 를 `static` 으로 이동 (agr_nmt.c 내 private)
  - [ ] 헤더 선언 제거

**위험**: Stage 1 은 0 (deprecated 태그만). Stage 3 은 외부 caller 가 나타나면 break.

---

### T3 — SDO Master Response Handling → **`plan_sdo_master.md` 로 승격**

현황 및 설계 상세는 별도 plan 으로 분리 (2026-04-22):
- 📄 **[`plan_sdo_master.md`](plan_sdo_master.md)** — CiA 301 기반 4 transport (CAN-FD / CoE / UDP / Serial) 공통 SDO Master 구현 plan
- 9개 설계 결정 (D-1 ~ D-9) 포함, user 선택 대기
- P1~P5 Phase 계획 (~2주), P6 은 use case 발생 시

본 T3 항목은 **링크 포인터로만 유지**. 실제 작업은 `plan_sdo_master.md` 에서.

---

## 3. 실행 우선순위

| 우선 | 항목 | AGR_MW-internal? | 범위 |
|:---:|------|:---:|------|
| **P1** | T1 — routine_mngr.c dead code 제거 | ✅ | 1 파일 |
| **P2** | T2 Stage 1 — `AGR_NMT_Init` `@deprecated` + 예시 교체 | ✅ | 2 파일 (h, docs) |
| **P3** (deferred) | T3 — Async SDO Master (별도 plan) | 설계 | 다수 |

**AGR_MW 에서 오늘 실행 가능**: P1 + P2 Stage 1.

**Multi-repo 넘어가야 할 것**: T2 Stage 2/3 (외부 caller 확인 + breaking removal).

---

## 4. Non-Goal

- ❌ Legacy git history 보존을 위한 별도 아카이브 — git log 가 이미 커버
- ❌ `_Init` 을 `_InitBasic` 으로 rename — 호환 깨짐, 가치 낮음
- ❌ Async SDO 구현 — 본 plan 은 목록화만, 실제 구현은 별도

---

## 5. 검증

### T1
- [ ] `routine_mngr.c` 줄 수 ~110 (기존 220)
- [ ] `RoutineMngr_*` 4개 함수만 정의
- [ ] Build 영향 0 (주석이었으므로)

### T2 Stage 1
- [ ] `AGR_NMT_Init` doxygen 에 `@deprecated` 존재
- [ ] `agr_nmt.h` docstring 예시 3건 (line 23, 40, 41) 가 `_InitEx` 사용
- [ ] CI @deprecated warning 확인 (다음 빌드 시)
