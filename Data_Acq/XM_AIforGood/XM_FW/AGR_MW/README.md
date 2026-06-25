# AGR_MW (AGR Middleware)

AGR 모듈 공용 미들웨어 서브모듈. IOIF(HW 추상화)와 Application 사이의 프로토콜/서비스 레이어.

## 구조

```
AGR_MW/
  DOP/          — Data Object Protocol V3 (CiA 301, Core + CAN-FD/CoE/UDP/Serial Transport)
  PnP/          — Plug-and-Play (Master/Slave, CANopen 기반)
  BOOT/         — App-side Boot Core (ConfirmBoot, RequestUpdate)
  TSM/          — Task State Machine (TaskMngr / RoutineMngr)
  RiskMngr/     — Risk Manager (현재 stub, plan_risk_mngr_v2 기반 재설계 예정)
```

## 사용법

### 1. Git Submodule 추가

```bash
git submodule add -b Develop https://github.com/AGR-EXO/AGR_MW.git {Module}_FW/AGR_MW
```

### 2. agr_mw_conf.h 생성

`agr_mw_conf_template.h`를 각 모듈의 `System/Config/agr_mw_conf.h`로 복사 후 수정.

> **`agr_mw_conf_template.h`는 참고용 템플릿입니다.**
> AGR_MW 서브모듈 내부의 이 파일은 빌드에 포함되지 않습니다.
> 실제로 사용되는 conf는 각 모듈 레포의 `System/Config/agr_mw_conf.h`입니다.
> (`ioif_conf.h`와 동일한 패턴)

```c
// System/Config/agr_mw_conf.h 예시
#define AGR_MW_MCU_SERIES_H7  1   // 자동 감지 (CubeMX define 기반)
#define AGR_MW_BOOT_ENABLE        // H7 + 부트로더 사용 모듈만
```

### 3. Include Path 추가 (.cproject)

```
AGR_MW/BOOT/Inc
AGR_MW/DOP
AGR_MW/DOP/Core
AGR_MW/DOP/Transport/CAN_FD
AGR_MW/DOP/Transport/CoE
AGR_MW/DOP/Transport/UDP
AGR_MW/DOP/Transport/Serial
AGR_MW/PnP
AGR_MW/TSM/Inc
AGR_MW/RiskMngr/Inc
```

## MCU 지원

| MCU Series | 매크로 | BOOT | DOP/PnP/TSM/etc |
|-----------|--------|------|-----------------|
| STM32H7 (H743, H750) | `AGR_MW_MCU_SERIES_H7` | O | O |
| STM32G4 (G474, G431) | `AGR_MW_MCU_SERIES_G4` | X (disable) | O |

## 적용 모듈

| 모듈 | MCU | BOOT |
|------|-----|------|
| XM (Extension Module) | STM32H743XI | Enable |
| CM-WH (WalkON H CM) | STM32H743ZI | Disable (향후) |
| SM-IMU (IMU Hub) | STM32G474RE | Disable |
| SM-EMG (EMG Hub) | STM32G474RE | Disable |
| SM-FES (FES Hub) | STM32G474RE | Disable |

## TSM (Task State Machine)

TSM 은 2 개의 public 헤더를 제공합니다. 내부 FSM 엔진은 `TSM/Inc/internal/` 로 격리되어 외부에서 직접 include 불가.

| 파일 | 역할 | 사용처 |
|------|------|--------|
| `task_mngr.h` (`TaskMngr_*`) | FSM 공식 API + 모니터링 | **Application 은 이것만 사용** |
| `routine_mngr.h` (`RoutineMngr_*`) | State Loop 내부의 동적 기능 on/off | 상태 내에서 기능 단위 enable/disable |
| `internal/task_state_machine.h` (`TSM_*`) | 내부 엔진 — consumer 측 include 금지 (path 미노출) | TSM 모듈 내부 구현 전용 |

### 권장 사용 패턴

```c
// FSM 정의 — TaskMngr (task_mngr.h)
TsmObject_t* task = TaskMngr_Create(INITIAL_STATE);
TaskMngr_AddState(task, STATE_RUN, on_entry, on_loop, on_exit);
TaskMngr_Run(task);
TaskMngr_Transition(task, STATE_STOP);

// 상태 조회 — getter 전용 (TsmObject_t 는 opaque handle)
uint8_t id = TaskMngr_GetStateId(task);
TsmLifecycle_e s = TaskMngr_GetLifecycle(task);
uint8_t prev_id = TaskMngr_GetPrevStateId(task);

// 상태 내부의 동적 기능 관리 — RoutineMngr (routine_mngr.h)
// 예: STATE_RUN 의 on_loop 에서 여러 제어 루틴을 조건부 실행
RoutineMngr_SetEnable(&mngr, ROUTINE_POS_CTRL, true);
RoutineMngr_RunAll(&mngr);
```

> **Breaking (2026-04-23)**: `TsmObject_t` 는 opaque 로 전환되었습니다. `obj->current_state_id` / `current_step` / `prev_state_id` / `prev_step` 직접 접근은 컴파일 에러입니다 — `TaskMngr_Get*()` 로 전환하세요. 자세한 내용은 `docs/plan_common_infra_review.md` §Claim F Stage 3.

### TsmDelay — One-shot Timer for FSM Ent/Run

상태 단위 시간 대기를 표현하기 위한 보조 API. `osDelay()` 가 같은 task 의 다른 FSM 까지 막는 문제를 회피하면서, `osKernelGetTickCount()` 직접 사용 시 모든 caller 가 반복하던 wrap-around 처리·one-shot 의미 구현 보일러플레이트를 제거합니다.

**RTOS-neutral 주입**: PnP 의 `get_tick` 패턴과 동일. 시작 시 1회 binding.

```c
#include "task_mngr.h"
#include "cmsis_os2.h"   // 또는 stm32xxxx_hal.h (HAL_GetTick)

void System_Init(void) {
    TsmDelay_SetTickProvider(osKernelGetTickCount);   // FreeRTOS
    // TsmDelay_SetTickProvider(HAL_GetTick);  // BareMetal — STM32 HAL tick
}
```

**FSM Ent/Run 사용 패턴**:

```c
static TsmDelay_t s_boot_delay;

static void Init_Ent(void) {
    TsmDelay_Start(&s_boot_delay, 50);   // 50 ms 대기 시작
}

static void Init_Run(void) {
    if (TsmDelay_Expired(&s_boot_delay)) {
        TaskMngr_Transition(boot_task, BOOT_STATE_BAT_VOLT_CHECK);
    }
}
```

**의미**:
- One-shot — `TsmDelay_Expired()` 는 만료 직후 1회만 `true` 반환. 재무장은 `TsmDelay_Start()`.
- Wrap-around safe — duration ≤ INT32_MAX (~24.8일 @ 1 kHz tick).
- Tick provider 미설정 시 `Expired()` 는 항상 `false` (timer never fires).

## Module Authoring Reference — PnP

신규 AGR_MW 모듈을 설계할 때 **PnP (`AGR_MW/PnP/`) 를 레퍼런스**로 참고하세요.

채택할 만한 패턴:
- **Dependency Injection** — `tx_func`, `get_tick` 을 Init 시 주입 → RTOS/BareMetal 양쪽 호환, 테스트 용이
- **Callback struct bundle** — 여러 콜백을 struct 로 묶어 한 번에 등록 (`AGR_PnP_Master_SlaveCallbacks_t`)
- **Config struct** — 등록 파라미터를 struct 로 전달 (`AGR_PnP_SlaveConfig_t`)
- **CiA 301 대칭** — NMT state machine 이 표준 의미 그대로 반영
- **정적 할당** — `malloc` 없이 fixed-size pool (MAX_SLAVES 상한)

자세한 구조는 [`PnP/agr_pnp_master.h`](PnP/agr_pnp_master.h) 참고.

## Error Return Contract

모든 AGR_MW public 함수는 `int32_t` 리턴 + `AGR_IsOk()` 검사 규약을 따릅니다. 자세한 내용은 [`docs/agr_mw_error_contract.md`](docs/agr_mw_error_contract.md).

## BOOT Core + Trigger 분리

AGR_MW/BOOT/에는 **공통 Core만** 포함:
- `agr_boot_types.h` — 타입, 플래시 레이아웃, enum (BL과 바이너리 호환)
- `agr_boot_core.h/c` — ConfirmBoot(), RequestUpdate()

모듈별 **Trigger** (부트 진입 방식)는 각 모듈의 `System/Boot/`에 위치:
- XM: `boot_ftp_trigger.c` (USB CDC FTP)
- CM: (향후) UART FTP
- SM: (향후) CAN-FD 기반 부트