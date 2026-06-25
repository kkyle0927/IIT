# AGR_MW Error Return Contract

> **Status**: Active
> **Scope**: All public functions in `AGR_MW/` submodule
> **Reference plans**: `plan_common_infra_review.md` §Claim A

---

## 1. 규약 요약

모든 AGR_MW public 함수의 에러 리턴은 다음 규약을 따른다.

| 항목 | 규정 |
|------|------|
| 리턴 타입 | **`int32_t`** (stdint.h). `int` 사용 금지. |
| `rc == 0` | **OK** — 정상 종료 |
| `rc <  0` | **Error** — 모듈별 typed enum 의 음수 값 |
| `rc >  0` | **Reserved** — 현재 미사용. 향후 "ok-with-info" 용으로 예약. 도입 시 해당 모듈 헤더에 의미 명시. |
| 검사 매크로 | `AGR_IsOk(rc)` 사용 — `agr_mw_conf.h` 에 정의 |

---

## 2. 근거

### 2.1 왜 `int` 대신 `int32_t`

1. **C 표준**: `int` 는 최소 16bit 만 보장. 32bit 플랫폼에 암묵적 의존하는 것은 이식성 저해.
2. **MISRA-C:2012 Dir 4.6**: "typedefs that indicate size and signedness should be used in place of the basic numerical types." → fixed-width 권장.
3. **ABI 비용 0**: ARM Cortex-M (H7/G4/F4) 에서 `int` == `int32_t`. 시그니처 치환만으로 바이너리 변화 없음.
4. **기존 최신 설계 정합**: BOOT 모듈 (`agr_boot_core.h`) 이 이미 `int32_t` 채택. 표준으로 격상.

### 2.2 왜 공용 에러 enum 은 안 만드는가

- 도메인별 에러 의미가 다름 (PnP 의 `SLAVE_FULL` 과 DOP 의 `SDO_ABORT` 를 같은 enum 에 넣으면 네임스페이스 오염)
- 각 모듈이 자체 typed enum 을 유지하되, **값만 `int32_t` 호환** (<0) 이면 공용 매크로 (`AGR_IsOk`) 로 검사 가능
- 모듈별 enum 은 모듈 자체 문서화 책임

---

## 3. 작성 규약 (신규 모듈용)

### 3.1 Public 함수 시그니처

```c
/* ✅ 올바른 예 */
int32_t AGR_MyModule_DoThing(AGR_MyModule_Ctx_t* ctx, uint32_t arg);

/* ❌ 금지 */
int MyModule_DoThing(...);         /* int 사용 금지 */
MyModule_Status_t MyModule_DoThing(...); /* 시그니처는 int32_t, enum 은 "값 소스" */
```

### 3.2 모듈별 Typed Enum (값 정의용)

```c
/* agr_mymodule.h */
typedef enum {
    AGR_MYMOD_OK              =  0,
    AGR_MYMOD_ERR_INVALID     = -1,
    AGR_MYMOD_ERR_FULL        = -2,
    AGR_MYMOD_ERR_TIMEOUT     = -3,
    /* ... */
} AGR_MyMod_Status_t;

int32_t AGR_MyModule_DoThing(...);
/* 구현에서: return (int32_t)AGR_MYMOD_ERR_FULL; */
```

### 3.3 호출자 패턴

```c
int32_t rc = AGR_MyModule_DoThing(&ctx, 42);
if (!AGR_IsOk(rc)) {
    /* error handling — rc 는 AGR_MyMod_Status_t 의 음수 값 */
    log_error("MyModule failed: %d", (int)rc);
    return rc;  /* 에러 그대로 전파 */
}
```

### 3.4 `AGR_IsOk` 매크로

```c
/* agr_mw_conf_template.h 에 정의됨 — 수정 금지 */
#define AGR_IsOk(rc)  ((int32_t)(rc) >= 0)
```

- `rc` 가 typed enum 이어도 int32_t 로 캐스트되어 일관 평가
- 양수 (reserved) 도 OK 로 분류 — "에러 아님" 의미 유지

---

## 4. Canonical Example — BOOT

`AGR_MW/BOOT/Inc/agr_boot_core.h:52` — 본 규약의 레퍼런스 구현:

```c
int32_t AGR_Boot_ConfirmBoot(void);  /* 0 on success, <0 on flash error */
```

---

## 5. Migration 상태 (2026-04-22 기준)

| 모듈 | 리턴 타입 | 규약 부합 |
|------|-----------|:---:|
| BOOT | `int32_t` | ✅ (기존 선행) |
| PnP Master/Slave | `int32_t` | ✅ |
| TSM / TaskMngr | `int32_t` | ✅ |
| DOP SDO/PDO/OD | `int32_t` | ✅ |
| CoE Master/Slave | `int32_t` | ✅ |
| CAN-FD / UDP / Serial Transport | `int32_t` | ✅ |
| NMT | `int32_t` | ✅ |

AGR_MW 공용 헤더 전체 migration 완료 (22 파일 / 114 signature).

**Consumer migration status**:
- Cortex-M 에서 `int` ≡ `int32_t` 이므로 consumer caller 수정 불필요 (ABI 호환)
- `AGR_IsOk()` 매크로 채택은 consumer 별로 `agr_mw_conf.h` 재동기화 시 활성화 — `plan_common_infra_review.md` §P3-c 참조

---

## 6. 예외 — 본 규약 미적용 케이스

다음 경우 `int32_t` 대신 다른 타입 사용 허용:

| 상황 | 권장 타입 | 예시 |
|------|-----------|------|
| 순수 getter (에러 불가능) | 반환 값 타입 직접 | `uint8_t TaskMngr_GetStateId(...)` |
| Boolean predicate | `bool` | `bool AGR_PnP_Master_IsSlaveOnline(...)` |
| 에러 없음이 자명한 setter/reset | `void` | `void TaskMngr_ResetPool(void)` |
| Enum state query | enum 타입 | `AGR_NMT_State_t AGR_NMT_GetState(...)` |

**판단 기준**: 함수가 실패 경로를 가지면 `int32_t`. 순수 query/setter 로 실패 불가능하면 자연 타입.

---

## 7. 위반 감지

- CI 또는 harness 의 정적 검사에서 AGR_MW 공용 헤더의 `^int\s+(AGR_|TSM_|TaskMngr_)` 패턴 검출 시 경고
- 본 규약은 AGR_MW 공용 API 에만 적용 — 내부 `static` 함수는 자유
