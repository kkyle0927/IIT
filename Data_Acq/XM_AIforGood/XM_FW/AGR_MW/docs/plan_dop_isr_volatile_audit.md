# Plan — DOP Context ISR↔main 공유 필드 volatile 누락 (CMW-08)

> **Origin**: SM-EMG DOP V3 CAN-FD 코어 정렬 세션(2026-06-18) sync-audit 발견.
> AGR_MW 가 SSOT 이므로 본 레포에서 작업.
> **상태**: ✅ **수정 완료 (2026-06-19)** — 3필드 모두 `volatile uint16_t` 전환 + ISR↔main
> 계약 주석 (agr_dop_types.h:399~413). thread-safety-audit PASS. 6개 소비모듈 전수조사 완료.

## 문제 (verified candidate)

`AGR_CANFD_ProcessRxMessage()` (CAN-FD 소비 모듈에서 **FDCAN Rx ISR context** 호출)
가 `AGR_DOP_SdoDiag_t` 카운터를 증가시킨다 — `DOP/Transport/CAN_FD/agr_dop_canfd.c`:

```c
280:  ctx->sdo_diag.sdo_decode_err_count++;
282:  ctx->sdo_diag.sdo_oversize_count++;
285:  ctx->sdo_diag.last_bad_index = req.index;
```

구조체 `DOP/agr_dop_types.h:406~410`:

```c
typedef struct {
    uint16_t sdo_oversize_count;     /* declared > MAX_DATA_SIZE 거부 횟수 */
    uint16_t sdo_decode_err_count;   /* 디코드 실패 횟수 */
    uint16_t last_bad_index;         /* 마지막 거부된 OD index */
} AGR_DOP_SdoDiag_t;
```

→ ISR(transport decode)에서 쓰고 **Risk Manager 가 main loop 에서 폴링**(주석 line 403:
"Risk Manager 가 폴링 → EMCY 발행")하는데 **plain `uint16_t` (volatile 아님)**.
컴파일러가 main 측 폴링 읽기를 캐시/생략할 수 있어 카운터 변화가 누락될 위험.

## §불일치 — reconcile 해결 (2026-06-19)

**해결**: transport 별로 건드리는 필드 수가 다른 게 "2 vs 3" 의 원인이었음 (전수 grep 확정):
- **CoE** (agr_dop_coe_slave.c:121~123): **2필드** (`*_count` 2종, `last_bad_index` 없음)
- **CAN-FD** (agr_dop_canfd.c:280~285) / **Serial** (agr_dop_serial.c:268~273): **3필드 전부**

→ audit 의 "2필드" = CoE 또는 두 카운터 지칭. `last_bad_index` 는 진단 전용(stale 허용)이나,
**균일성·단순성 위해 3필드 모두 volatile 전환** (비용 무시 수준, 2vs3 모호성 영구 제거).

## 이미 OK (이 fix 대상 아님)

- `AGR_NMT_Inst_t.state` / `.last_activity_ms` (`DOP/agr_nmt.h:105~106`) 는 **이미
  `volatile`** (주석 line 91: "ISR과 메인 루프에서 공유될 수 있음"). 건드리지 말 것.
- `tx_pdo_map[4]` / `rx_pdo_map[4]` (agr_dop_types.h:450~451): SDO(PDO 매핑)로 변경.
  SM-EMG 는 R13 으로 SDO 를 main loop 에서 처리(defer) → EMG path 는 main-only. 단
  **lib 은 소비자별 defer 를 가정 못 함** (IMU/FES 는 ISR 처리 가능) → ISR/main 계약을
  주석으로 명문화할지 별도 검토.

## 설계 고려

- `volatile` 은 **가시성**만 보장, uint16 증분의 **원자성**은 아님. 단 여기선 **ISR 만
  write(증분), main 은 read(폴링)** 이므로 M4/M7 에서 `volatile` 만으로 read freshness 충분.
- 대안: main 에서 snapshot 복사 후 사용. lib 단순성 위해 `volatile` 권장.
- **금지**: 본 fix 로 `cmsis_os2.h`/HAL/뮤텍스 introduce 금지 — AGR_MW 는 순수 C,
  G4 BareMetal 깸 방지 ([[feedback-agr-mw-inject-dont-include]]).

## Cross-module 영향

AGR_MW 는 **공유 lib** (XM/CM/SM-EMG/SM-IMU/SM-FES). 변경 → submodule bump → 전 소비
모듈 재빌드/재검증. AGR_MW HEAD = `fb47584` (현재 전 모듈 정합).

## 소비모듈 전수조사 (2026-06-19, 12-agent 워크플로우 + 적대 verify)

| 모듈 | RTOS/BM | ProcessRxMessage 컨텍스트 | sdo_diag read | ISR-write? |
|---|---|---|---|---|
| XM | RTOS | **호출 안 함** (자체 AGR_SDO_Decode) | 없음 | n/a |
| CM-WH | RTOS | RTOS task (RxTask prio52) | 없음 | no |
| SAM3x | BareMetal | deferred→main | 없음 | no |
| **IMU** | BareMetal | **ISR 직접** | 없음(risk_mngr stub) | **yes (유일)** |
| EMG | BareMetal | PDO=ISR(sdo_diag 미도달)/SDO=defer→main | 없음(stub) | no |
| FES | BareMetal | deferred→main | 없음 | no |

**판정**: 라이브 버그 아님 / **선제 수정**. ① `sdo_diag` 를 읽는 코드(Risk Manager)가 현재
전 모듈 stub — 읽는 쪽 없음. ② ISR-write 는 IMU 단 하나. → Risk Manager v2 도입 전 선제
volatile 로 IMU ISR-write + (미래)main-read 대비. 단일 ISR-writer 라 RMW 원자성 이슈 없음
(증분은 한 컨텍스트만). EMG R13 SDO defer = 확인됨.

## 작업 체크리스트

- [x] 정확한 대상 필드 확정 (§불일치 reconcile) — 3필드 모두 volatile (2vs3 = transport별 차이)
- [x] `AGR_DOP_SdoDiag_t` 카운터 `volatile uint16_t` 전환 (agr_dop_types.h:406~410)
- [x] ISR-write / main-read 계약 주석 보강 (agr_dop_types.h:403~413)
- [x] `thread-safety-audit` Skill — PASS
- [ ] **남음** 호스트 유닛테스트(test/) 회귀 + 전 소비 모듈 빌드
- [ ] **남음** submodule bump 조율

## 참조

- Rule `critical/13-comm-core-patterns.md`, `domain/14-dop-messages.md`,
  `critical/02-safety-checklist.md` (BareMetal volatile+Flag)
- [[feedback-agr-mw-inject-dont-include]], [[project-baremetal-isr-priority-per-module]]
- SM-EMG R13 deferred-SDO 결정 (ISR=적재만, main=처리) — 같은 ISR-최소화 축
