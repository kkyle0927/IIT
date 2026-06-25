# AGR_MW — SDO Master (Client) Implementation Plan

> **Status**: Design Decisions Confirmed (2026-04-22, revised — USDO 폐기)
> **Owner**: HyundoKim
> **Scope**: AGR_MW DOP Core + 4 Transport — CAN-FD / CoE / UDP / Serial, 전 transport **CiA 301 baseline 통일**
> **Origin**: `plan_tech_debt.md §T3` 에서 승격. Master 측 SDO Response 처리 부재 해결.

---

## 1. 배경 — DOP 본질과 표준 선택

### DOP 본질 (설계 북극성)
1. **Transport-agnostic Object Dictionary access** — 기기가 표준 OD (index, subindex, 타입, 크기) 로 자기 자신을 노출. 어떤 매체든 동일 OD 접근 의미.
2. **SDO / PDO 역할 분리** — SDO = 간헐적 config/diagnostic, PDO = 주기 실시간. Transport 불문 동일.
3. **CANopen 표준 의미론 준수** — CS byte, index/subindex, abort code. CiA 301 이 공통 토대.
4. **Core + Transport 분리** — Core = OD + SDO 의미 처리, Transport = wire framing.

### 왜 CiA 301 baseline 통일인가 (USDO 폐기 배경)

CiA 1301 USDO 는 CiA 301 을 확장한 신 프로토콜이지만, AGR 에서는 **이득이 없고 비용이 큼**:

**USDO 이득 — AGR 에서 발현 X**:
- Transaction ID (multi-in-flight): AGR 는 1 Master + N Slaves 패턴, 한 slave 당 동시 요청 use case **0건**
- Source/Dest 헤더 필드: CAN-ID (0x580/0x600 + node) 가 이미 sender/receiver 식별 → 중복
- Multi-hop routing (gateway): AGR 에코시스템 단일 segment, 불필요
- ~58B single-frame: AGR 의 현 "CiA 301 CS + 60B extended payload" 가 **기능 동등**

**USDO 비용 — 크고 실질적**:
- Wire format 비호환 → 신 Master ↔ 구 Slave 완전 절단
- Consumer repo 6개 (ARC / XM10 Rev1.1/Rev2.0 / CM-WH / IMU Hub / EMG Hub / FES Hub) **lock-step 업그레이드 필수**
- Device Driver Pre-Op state machine 전면 수정
- Slave OD handler 전면 수정 (USDO 헤더 파싱)
- 총 공수 ~2개월, field-deployed 장비 동시 펌웨어 업데이트 요구 (실질 불가)

**DOP 본질 관점**:
- CAN-FD 만 USDO 로 바꾸면 **"transport 간 의미론 통일성"** 이 깨짐 — DOP 핵심 가치 훼손
- Engineer 가 transport 넘나들 때 mental model 2개 유지 필요

→ **결론: 전 transport CiA 301 baseline 통일**. USDO 는 실질 이득 없음 + 비용 큼 + DOP 본질 훼손.

### 표준 맵

| Transport | Baseline | Wire format | 재설계 |
|-----------|----------|-------------|:---:|
| CAN-FD | **CiA 301 + 60B extended payload** (AGR 현 방식) | 유지 | ❌ |
| CoE | CiA 301 CoE (SOEM) | 유지 | ❌ |
| CANopen over UDP | AGR 커스텀 (CiA 301 semantics) | 유지 | ❌ |
| CANopen over Serial | AGR 커스텀 (CiA 301 semantics) | 유지 | ❌ |

**어느 transport 도 wire format 변경 없음**. 신규는 Master response 처리 pipeline 만 추가.

---

## 2. 현재 상태 — Master 측 gap 실측

| Transport | Master 송신 | Master 응답 수신 | 비고 |
|---|:---:|---|---|
| CAN-FD | ✅ `AGR_CANFD_SendSDO()` | ❌ `case SDO_TX: return 1;` | 폐기 |
| Serial | ✅ `AGR_Serial_SendSDO()` | ❌ `case SDO_RSP: break;` (TODO) | 폐기 |
| UDP | ✅ `AGR_UDP_SendSDO()` | ⚠️ `s_sdo_cb` 로 raw passthrough | Master/Slave 분기 없음 |
| CoE | ✅ `AGR_COE_Master_SDORead/Write` (SOEM blocking sync) | ⚠️ `wkc > 0` 만, abort code 노출 X | |

### Core 재사용 자산
- ✅ `AGR_SDO_CreateReadReq/CreateWriteReq/CreateAbortResponse`
- ✅ `AGR_SDO_AbortCode_t` (CiA 301 전체)
- ✅ `AGR_SDO_Encode/Decode`
- ❌ Master-side response processor 없음 (Slave 용 `AGR_SDO_ProcessRequest` 만 존재)
- ❌ In-flight transaction 관리 없음
- ❌ Timeout tracking 없음

---

## 3. 설계 원칙

1. **전 transport CiA 301 baseline 통일** — Wire format 변경 0, consumer 수정 최소. DOP 의 transport-agnostic 본질 유지.
2. **Single-in-flight per (master, slave)** — CiA 301 자연 모델. `(slave_id, index, subindex)` 매칭.
3. **Slave pipeline 과 Transport layer 에서 대칭** — `AGR_SDO_ProcessRequest` (slave) ↔ `AGR_SDO_Master_ProcessResponse` (master). User-facing API 는 비대칭 (Master 는 per-request callback).
4. **Callback-only API** — `on_done` 필수. Polling mode / blocking mode 빌트인 X. Caller 가 callback 에서 자체 flag/state 관리.
5. **AGR_MW 는 RTOS 독립** — OS primitive 의존 없음. Blocking semantic 필요 시 caller 가 semaphore 로 wrap.
6. **기존 fire-and-forget caller 보호** — `AGR_CANFD_SendSDO()` 등 기존 API 유지, 신규 Master API 는 additive.

---

## 4. 설계 결정 — **최종 확정** (2026-04-22)

### ✅ D-1: Protocol Baseline
**결정**: 전 transport **CiA 301 baseline 통일**. CiA 1301 USDO 폐기.

- AGR 에서 USDO 이득 없음 + consumer migration 비용 과다 + DOP 의미론 통일성 훼손
- 단, **미래 확장 여지는 남김** — §13 "Future Extension" 참고

### ✅ D-2: Transaction 매칭 모델
**결정**: **Single-in-flight per (master, slave)** 전 transport 공통. `(slave_id, index, subindex)` 로 매칭.

- CAN-FD: CAN-ID 가 (direction, slave) 구분, payload 의 index/subindex 로 transaction 식별
- CoE: SOEM 이 inherently sync → 자동 single-in-flight
- UDP / Serial: 포맷상 단일 payload per message → 자연 single-in-flight
- 동일 slave 에 in-flight 있을 때 추가 요청 → `AGR_SDO_Master_AllocTx` NULL 반환 (caller 가 재시도 or 에러)

### ✅ D-3: 완료 통지 모델 — **Callback-only, Caller 가 flag 관리**
**결정: (A) Per-request callback**. `on_done` 함수포인터 **필수**.

**주요 패턴**:
```c
/* Caller 측 — Device Driver 의 Pre-Op state machine */
typedef struct {
    volatile bool        ack_received;
    AGR_SDO_Tx_State_e   last_state;
    AGR_SDO_AbortCode_t  last_abort;
} my_device_t;
static my_device_t s_dev;

static void _on_sdo_done(AGR_SDO_Transaction_t* tx,
                          const uint8_t* data, uint8_t len,
                          AGR_SDO_AbortCode_t abort, void* user_ctx)
{
    my_device_t* dev = (my_device_t*)user_ctx;
    dev->last_state = tx->state;
    dev->last_abort = abort;
    dev->ack_received = true;
}

/* State machine 에서 */
switch (state) {
    case STATE_WRITE_CONFIG:
        s_dev.ack_received = false;
        AGR_CANFD_Master_Request(ctx, slave, 0x6040, 0, /*write*/true,
                                  &ctrl_word, 2, 1000,
                                  _on_sdo_done, &s_dev);
        state = STATE_WAIT_CONFIG_ACK;
        break;

    case STATE_WAIT_CONFIG_ACK:
        if (s_dev.ack_received) {
            if (s_dev.last_state == AGR_SDO_TX_DONE) state = STATE_NEXT;
            else                                     state = STATE_ERROR;
        }
        break;
}
```

→ **Callback 에서 caller 의 flag 세팅, main loop 의 state machine 이 flag 확인**. 이게 SDO Response 성공/실패 확인의 **표준 패턴**.

**왜 callback-only (polling/blocking 기각) rationale — Core 헤더 주석 필수**:
```c
/*
 * WHY CALLBACK-ONLY (no polling, no blocking)
 * ===========================================
 *
 * (1) Per-request callback (not global hook):
 *     Slave-side on_sdo_request is global because incoming requests target
 *     unknown OD indices. Master-side is opposite — we initiate each
 *     request and know its context. Per-request callback keeps caller
 *     code close to the request, enables distinct user_ctx, and avoids
 *     forcing the caller to re-dispatch by (index, subindex).
 *
 * (2) No polling mode:
 *     Allowing on_done == NULL would let caller peek tx->state directly.
 *     This creates coupling (caller reading middleware internals) and
 *     offers no benefit over callback setting a caller-owned flag.
 *     State-machine patterns — the natural way to drive PnP init
 *     sequences (Write → ack flag → next Write) — work cleanly with
 *     callback + caller flag.
 *
 * (3) No blocking primitive:
 *     Building OS semaphore/mutex into AGR_MW would force RTOS
 *     dependency and break middleware portability. Blocking, when
 *     needed, is a 10-line caller-side wrap:
 *         osSemaphoreId_t sem = osSemaphoreNew(...);
 *         AGR_CANFD_Master_Request(..., release_sem_cb, sem);
 *         osSemaphoreAcquire(sem, timeout);
 *     BareMetal callers use the flag + state-machine pattern instead.
 */
```

### ✅ D-4: Blocking mode
**결정: (A') Blocking 빌트인 제거**. Callback-only + caller-side 조립. RTOS 독립.

### ✅ D-5: 프레임 범위
**결정**: Transport 별 "single-frame" 원칙.

- CAN-FD: CiA 301 CS + 최대 60B data (frame 64B - 4B CS/index/sub)
- UDP: 단일 packet 내 처리 (MTU 1500B 여유)
- Serial: 단일 COBS 프레임
- CoE: SOEM 내부 처리, caller 는 single-call

60B 초과 데이터는 **별도 FTP** (SDO 범위 외).

### ✅ D-6: CoE Abort Code 매핑
**결정: SOEM `ec_errlist` 파싱**. CoE 에 맞는 best pattern.

`EC_ERR_TYPE_SDO_ERROR` 에 32-bit CiA 301 AbortCode 포함 → `AGR_SDO_AbortCode_t` 로 cast (값 동일). Slave/Index/SubIndex 매칭 기반 walker.

### ✅ D-7: API 레이어 구조
**결정: Core 선언 + Transport 구현** (PnP Master DI 패턴).

- `DOP/Core/agr_sdo_master.h/c` — Transaction 관리, ProcessResponse, Tick
- `AGR_{Transport}_Master_Request()` 각 transport 별 구현

### ✅ D-8: Timeout Tick Source
**결정: DI 함수포인터 `get_tick_ms`**. 기존 PnP 패턴.

### ✅ D-9: UDP 기존 `s_sdo_cb`
**결정: 기존 유지 + Master 신규 add**. SDO CS byte 로 Master/Slave 분기.

---

## 5. Core Layer API — `agr_sdo_master.h/c`

```c
/* agr_sdo_master.h */

#ifndef AGR_SDO_MASTER_H
#define AGR_SDO_MASTER_H

#include <stdint.h>
#include <stdbool.h>
#include "agr_dop_types.h"

#ifndef AGR_SDO_MASTER_MAX_PENDING
#define AGR_SDO_MASTER_MAX_PENDING   8    /**< Per-context 동시 in-flight 최대 (slave 수에 따라 조정) */
#endif

#ifndef AGR_SDO_MAX_DATA
#define AGR_SDO_MAX_DATA             60   /**< CiA 301 extended payload (CAN-FD 64B - 4B 헤더) */
#endif

typedef enum {
    AGR_SDO_TX_IDLE,
    AGR_SDO_TX_PENDING,        /**< 송신됨, 응답 대기 */
    AGR_SDO_TX_DONE,           /**< 정상 완료 */
    AGR_SDO_TX_ABORTED,        /**< Slave 가 abort 응답 */
    AGR_SDO_TX_TIMEOUT,        /**< 응답 없음 */
} AGR_SDO_Tx_State_e;

typedef struct AGR_SDO_Transaction {
    /* 매칭 필드 (전 transport 공통) */
    uint8_t                slave_id;
    uint16_t               index;
    uint8_t                subindex;
    bool                   is_write;

    /* 상태 */
    AGR_SDO_Tx_State_e     state;
    uint32_t               sent_tick_ms;
    uint32_t               timeout_ms;

    /* 완료 통지 (callback-only, 필수) */
    AGR_SDO_CompletionCb_t on_done;
    void*                  user_ctx;

    /* 데이터 */
    uint8_t                data[AGR_SDO_MAX_DATA];
    uint8_t                data_len;
    AGR_SDO_AbortCode_t    abort_code;
} AGR_SDO_Transaction_t;

/**
 * @brief Request 완료 콜백 — Per-request 단위
 *
 * @details
 * Caller 는 이 callback 에서 **자체 flag/state 를 세팅**. AGR_MW 내부
 * state (tx->state) 를 outside 에서 폴링하지 말고, caller 가 소유한
 * my_device->ack_received 같은 flag 를 세팅하는 것이 권장 패턴.
 *
 * @param tx           완료된 transaction. tx->state 로 결과 확인 가능.
 * @param data         Read 성공 시 수신 데이터. 그 외 NULL.
 * @param data_len     data 바이트 수.
 * @param abort_code   tx->state == AGR_SDO_TX_ABORTED 일 때만 유효.
 * @param user_ctx     Request 시 전달한 context (caller 자유 사용).
 *
 * @note  Callback context 는 transport 구현에 따름:
 *        - CAN-FD / Serial: Rx ISR 또는 상위 thread
 *        - UDP: tcpip_thread
 *        - CoE: Request 호출한 caller thread (sync)
 *        ISR 안전 보장 필요 시 caller 가 flag-only 처리.
 */
typedef void (*AGR_SDO_CompletionCb_t)(
    AGR_SDO_Transaction_t* tx,
    const uint8_t*         data,
    uint8_t                data_len,
    AGR_SDO_AbortCode_t    abort_code,
    void*                  user_ctx);

typedef struct {
    AGR_SDO_Transaction_t    pool[AGR_SDO_MASTER_MAX_PENDING];
    uint32_t               (*get_tick_ms)(void);
} AGR_SDO_Master_t;

void AGR_SDO_Master_Init(AGR_SDO_Master_t* m, uint32_t (*get_tick_ms)(void));

/**
 * @brief Transaction 할당
 * @return NULL if (1) 풀 가득 참, (2) 같은 slave 에 in-flight 이미 존재
 *         (single-in-flight per slave enforce)
 */
AGR_SDO_Transaction_t* AGR_SDO_Master_AllocTx(AGR_SDO_Master_t* m, uint8_t slave_id);

/**
 * @brief 응답 처리 — Transport 가 SDO response/abort 수신 시 호출
 *
 * @details
 * (from_slave, rsp.index, rsp.subindex) 매칭으로 in-flight transaction 찾기.
 * 매칭 성공 시 state 업데이트 + on_done 호출 + 풀 반환.
 *
 * @return 0 매칭 성공, <0 orphan response (폐기)
 */
int32_t AGR_SDO_Master_ProcessResponse(AGR_SDO_Master_t*    m,
                                        uint8_t              from_slave,
                                        const AGR_SDO_Msg_t* rsp);

/**
 * @brief Timeout 체크 — Transport task 또는 main loop 에서 주기 호출
 */
void AGR_SDO_Master_Tick(AGR_SDO_Master_t* m, uint32_t current_ms);

#endif /* AGR_SDO_MASTER_H */
```

### Transport-specific API (전 transport 공통 시그니처)

```c
int32_t AGR_CANFD_Master_Request(
    AGR_DOP_Ctx_t*          ctx,
    uint8_t                 slave_id,
    uint16_t                index,
    uint8_t                 subindex,
    bool                    is_write,
    const uint8_t*          data,          /* NULL if read */
    uint8_t                 data_len,
    uint32_t                timeout_ms,
    AGR_SDO_CompletionCb_t  on_done,       /* 필수 */
    void*                   user_ctx);
```

동일 시그니처로 `AGR_Serial_Master_Request`, `AGR_UDP_Master_Request`, `AGR_COE_Master_Request`.

---

## 6. Transport 별 구현

### 6.1 CAN-FD
- **Wire format 변경 없음**: 현 "CiA 301 CS + 60B extended payload" 유지
- `ProcessRxMessage` 의 `case AGR_CAN_FUNC_SDO_TX` 채우기:
  ```c
  case AGR_CAN_FUNC_SDO_TX:
      if (can_id == AGR_CANFD_GetSDOResponseID(ctx->node_id)) {
          AGR_SDO_Msg_t rsp;
          if (AGR_SDO_Decode(data, len, &rsp) == 0) {
              AGR_SDO_Master_ProcessResponse(&ctx->sdo_master, source_node, &rsp);
          }
          return 0;
      }
      return 1;
  ```
- `AGR_CANFD_Master_Request()` — Transaction alloc → SDO encode → tx_func 송신

### 6.2 Serial
- **Wire format 변경 없음**: `AGR_SERIAL_MSG_SDO_REQ/RSP` 유지
- `agr_dop_serial.c:296` TODO 자리:
  ```c
  case AGR_SERIAL_MSG_SDO_RSP: {
      AGR_SDO_Msg_t rsp;
      if (AGR_SDO_Decode(payload, (uint8_t)payload_len, &rsp) == 0) {
          AGR_SDO_Master_ProcessResponse(&dop->sdo_master, source_node, &rsp);
      }
      break;
  }
  ```

### 6.3 UDP
- **Wire format 변경 없음**: `AGR_UDP_TYPE_SDO_REQ/RSP` 유지
- `_OnSdoReceived` 에서 CS byte 로 Master/Slave 분기:
  ```c
  uint8_t cs = payload[0];
  bool is_response = (cs == 0x60) || ((cs & 0xE0) == 0x40) || (cs == 0x80);

  if (is_response) {
      AGR_SDO_Msg_t rsp;
      if (AGR_SDO_Decode(payload, (uint8_t)payload_len, &rsp) == 0) {
          AGR_SDO_Master_ProcessResponse(&s_udp_ctx->sdo_master, src_id, &rsp);
      }
  } else {
      if (s_sdo_cb != NULL) s_sdo_cb(msg_type, src_id, payload, (uint8_t)payload_len);
  }
  ```

### 6.4 CoE
- **Wire format 변경 없음**: SOEM 그대로 사용
- `AGR_COE_Master_Request()` 는 sync wrapper — SOEM 호출 후 즉시 callback:
  ```c
  int32_t AGR_COE_Master_Request(..., on_done, user_ctx) {
      AGR_SDO_Transaction_t* tx = AGR_SDO_Master_AllocTx(&mctx->sdo_master, slave);
      if (!tx) return -1;
      tx->state = AGR_SDO_TX_PENDING;
      /* ...fill tx... */

      int wkc = is_write ? mctx->sdo_write(slave, idx, sub, data, size)
                         : mctx->sdo_read (slave, idx, sub, tx->data, &read_size);

      if (wkc > 0) {
          tx->state = AGR_SDO_TX_DONE;
      } else {
          tx->state = AGR_SDO_TX_ABORTED;
          tx->abort_code = _ExtractSdoAbortFromSOEM(slave, idx, sub);
      }
      on_done(tx, tx->data, tx->data_len, tx->abort_code, user_ctx);
      return 0;
  }
  ```

---

## 7. Phase 계획

| Phase | 작업 | 범위 | 소요 |
|:---:|---|---|:---:|
| **P0** | Design 확정 | — | ✅ 완료 |
| **P1** | Core API — `agr_sdo_master.h/c` + Transaction pool + Mock unit test | Core | 3~5일 |
| **P2** | CAN-FD 구현 — SDO_TX case 채움, `AGR_CANFD_Master_Request` | CAN-FD | 2일 |
| **P3** | Serial 구현 — TODO 자리 Response 처리 | Serial | 1일 |
| **P4** | UDP 구현 — `_OnSdoReceived` CS 분기 + Master path | UDP | 1일 |
| **P5** | CoE 구현 — `AGR_COE_Master_Request` + SOEM `ec_errlist` walker | CoE | 2일 |
| **P6** | Device Driver migration (선택적) — use case 별 | 소비 모듈 | use case 발생 시 |

**AGR_MW-internal (P1~P5)**: ~1.5 주.

---

## 8. Migration 전략

### 기본 원칙 — Additive
기존 API 전부 유지:
- `AGR_CANFD_SendSDO` / `AGR_Serial_SendSDO` / `AGR_UDP_SendSDO` — fire-and-forget caller 그대로
- `AGR_COE_Master_SDORead/Write` — blocking sync 그대로, 신규 `_Request` 는 abort code 얻고 싶은 케이스용

신규 Master API 채택 대상 (use case 발생 시):
- PnP Pre-Op SDO 시퀀스 (response 기반 state 전이)
- RiskMngr v2 fault ack/clear
- 런타임 config UI (Write 후 응답 확인)
- Slave discovery (0x1000, 0x1008 Read)

---

### 구체 예시 — XM IMU Hub Driver Migration

현 `Extension_Module/XM_FW/Devices/AGR/IMU_Module/imu_hub_drv.c` 기준. **AGR_MW 를 우회해서 자체 구현된 SDO response pipeline 을 공식 API 로 전환**.

#### 현재 구조 (AGR_MW 우회)

```c
/*
 * ─── State Enum (불변 유지) ────────────────────────────────
 */
typedef enum {
    IMUHUB_PRE_OP_IDLE,
    IMUHUB_PRE_OP_SEND_PDO_MAP_A,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,    /* ← Response 기다리는 state */
    IMUHUB_PRE_OP_SEND_PDO_MAP_B,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,
    IMUHUB_PRE_OP_SEND_IMU_MASK_REQ,
    IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP,
    IMUHUB_PRE_OP_SEND_NMT_START,
    IMUHUB_PRE_OP_COMPLETE
} ImuHub_PreOpState_t;

/*
 * ─── Step Array ─────────────────────────────────────────────
 */
static const ImuHub_PreOpStep_t s_pre_op_steps[] = {
    { SEND_PDO_MAP_A,    WAIT_PDO_MAP_A,    _Step_SendPdoMapA,    5000, "TPDO1 Mapping" },
    { SEND_PDO_MAP_B,    WAIT_PDO_MAP_B,    _Step_SendPdoMapB,    5000, "TPDO2 Mapping" },
    { SEND_IMU_MASK_REQ, WAIT_IMU_MASK_RSP, _Step_SendImuMaskReq, 5000, "IMU Mask Read" },
    { SEND_NMT_START,    COMPLETE,          _Step_SendNmtStart,   5000, "NMT START" },
};

/*
 * ─── SEND step: Fire-and-forget (response 확인 불가) ────────
 */
static int _Step_SendPdoMapA(void) {
    return AGR_CANFD_SendSDOWrite(&s_imu_hub_inst.dop_ctx,
                                  IMUHUB_OD_IDX_PDO_MAPPING_A, 0,
                                  s_pdo_map_a, sizeof(s_pdo_map_a));
}
static int _Step_SendImuMaskReq(void) {
    AGR_SDO_Msg_t sdo_req = {
        .cs = AGR_SDO_CS_UPLOAD_INIT_REQ,
        .index = IMUHUB_OD_IDX_IMU_CONN_MASK,
        .subindex = 0, .data_len = 0
    };
    return AGR_CANFD_SendSDO(&s_imu_hub_inst.dop_ctx,
                             AGR_NODE_ID_IMU_HUB, &sdo_req);
}

/*
 * ─── State Machine — SEND → WAIT 전이 ──────────────────────
 */
static void _RunPreOpStateMachine(void) {
    for (uint8_t i = 0; i < PRE_OP_STEP_COUNT; i++) {
        const ImuHub_PreOpStep_t* step = &s_pre_op_steps[i];

        if (s_imu_hub_inst.pre_op_state == step->send_state) {
            int result = step->action();
            if (result >= 0) {
                s_imu_hub_inst.pre_op_state = step->wait_state;
                s_imu_hub_inst.last_sdo_tx_time = IOIF_TIM_GetTick();
                s_imu_hub_inst.sdo_retry_count = 0;
            }
            return;
        }
        if (s_imu_hub_inst.pre_op_state == step->wait_state) {
            return;   /* Response 기다림 */
        }
    }
}

/*
 * ─── CAN 프레임 직접 파싱 (AGR_MW 우회) ─────────────────────
 */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len) {
    uint16_t fnc_code = can_id & 0x780;

    if (fnc_code == 0x580) {     /* ⚠️ AGR_MW 가 해야 할 일을 Device 가 함 */
        AGR_SDO_Msg_t sdo_msg;
        if (AGR_SDO_Decode(data, len, &sdo_msg) == 0) {
            _OnSdoResponse(&sdo_msg);
        }
        return;
    }
    /* ... PDO 처리 ... */
}

/*
 * ─── Response 처리 — WAIT → 다음 SEND 전이 ─────────────────
 */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response) {
    uint8_t cs = response->cs & 0xE0;

    if (cs == AGR_SDO_CS_DOWNLOAD_INIT_RSP) {
        if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_A) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
        else if (response->index == IMUHUB_OD_IDX_PDO_MAPPING_B) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
    }
    else if ((response->cs & 0xE0) == 0x40) {   /* Upload Response */
        if (response->index == IMUHUB_OD_IDX_IMU_CONN_MASK) {
            s_imu_hub_inst.imu_connected_mask = response->data[0];
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
    }
    /* ⚠️ Abort code 처리 누락 — 현 구조는 실패 원인 파싱 안 함 */
}

/*
 * ─── Timeout & Retry ─────────────────────────────────────────
 */
void ImuHub_Drv_RunPeriodic(void) {
    uint32_t current_ms = IOIF_TIM_GetTick();

    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP)
    {
        if (current_ms - s_imu_hub_inst.last_sdo_tx_time > 5000) {
            if (s_imu_hub_inst.sdo_retry_count < 3) {
                s_imu_hub_inst.sdo_retry_count++;
                /* WAIT → SEND (재시도) */
                if (...WAIT_PDO_MAP_A) pre_op_state = SEND_PDO_MAP_A;
                /* ... */
            } else {
                pre_op_state = IDLE;
                g_xm_bus_diag.sdo_timeout_count++;
            }
        }
    }

    _RunPreOpStateMachine();
}
```

**문제점**:
1. `ImuHub_Drv_ProcessCANMessage` 가 AGR_MW transport 우회 (0x580 직접 파싱)
2. Timeout/retry 로직이 Device Driver 안에 중복 구현 (다른 driver 들도 동일 패턴 복붙)
3. Abort code 파싱 없음 — Slave 가 SDO Abort (잘못된 index 등) 응답해도 단순 "Response 안 왔음" 으로만 보임
4. `_OnSdoResponse` 의 CS 매칭이 index 기반 — 같은 index 에 여러 state 공유 시 오매칭 가능

#### Migration 후 구조 (AGR_MW Master pipeline)

```c
/*
 * ─── State Enum — 그대로 ────────────────────────────────────
 */
/* 변경 없음 — SEND/WAIT 패턴 유지 */

/*
 * ─── Step Array — action 시그니처 변경 ────────────────────
 */
typedef int (*ImuHub_PreOpAction_t)(ImuHub_DrvInst_t* self);

static const ImuHub_PreOpStep_t s_pre_op_steps[] = {
    { SEND_PDO_MAP_A,    WAIT_PDO_MAP_A,    _Step_SendPdoMapA,    5000, "TPDO1 Mapping" },
    /* ... 동일 ... */
};

/*
 * ─── SEND step: Master_Request + callback 지정 ────────────
 */
static int _Step_SendPdoMapA(ImuHub_DrvInst_t* self) {
    return AGR_CANFD_Master_Request(
        &self->dop_ctx,
        AGR_NODE_ID_IMU_HUB,
        IMUHUB_OD_IDX_PDO_MAPPING_A, 0,
        /*is_write*/ true,
        s_pdo_map_a, sizeof(s_pdo_map_a),
        /*timeout_ms*/ 5000,
        _on_sdo_done, self);   /* ← callback + user_ctx */
}

static int _Step_SendImuMaskReq(ImuHub_DrvInst_t* self) {
    return AGR_CANFD_Master_Request(
        &self->dop_ctx,
        AGR_NODE_ID_IMU_HUB,
        IMUHUB_OD_IDX_IMU_CONN_MASK, 0,
        /*is_write*/ false,       /* Upload(Read) */
        NULL, 0,
        5000,
        _on_sdo_done, self);
}

/*
 * ─── State Machine — 거의 동일 ─────────────────────────────
 */
static void _RunPreOpStateMachine(ImuHub_DrvInst_t* self) {
    for (uint8_t i = 0; i < PRE_OP_STEP_COUNT; i++) {
        const ImuHub_PreOpStep_t* step = &s_pre_op_steps[i];

        if (self->pre_op_state == step->send_state) {
            int result = step->action(self);
            if (result >= 0) {
                self->pre_op_state = step->wait_state;
                self->sdo_retry_count = 0;
                /* last_sdo_tx_time 제거 — timeout 은 AGR_MW 가 관리 */
            }
            return;
        }
        if (self->pre_op_state == step->wait_state) {
            return;   /* callback 이 state 전이 담당 */
        }
    }
}

/*
 * ─── CAN 프레임 우회 분기 제거 ─────────────────────────────
 */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len) {
    uint16_t fnc_code = can_id & 0x780;

    /* ❌ if (fnc_code == 0x580) {...} — 제거. AGR_MW 가 처리 */

    if (fnc_code == 0x180) { /* TPDO1 */ ... }
    if (fnc_code == 0x280) { /* TPDO2 */ ... }
}

/*
 * ─── Response 처리 — AGR_MW 호출 callback ──────────────────
 */
static void _on_sdo_done(AGR_SDO_Transaction_t* tx,
                          const uint8_t* data, uint8_t data_len,
                          AGR_SDO_AbortCode_t abort, void* user_ctx)
{
    ImuHub_DrvInst_t* self = (ImuHub_DrvInst_t*)user_ctx;

    if (tx->state == AGR_SDO_TX_DONE) {
        /* 성공 — WAIT → 다음 SEND 전이 */
        if (tx->is_write && tx->index == IMUHUB_OD_IDX_PDO_MAPPING_A) {
            self->pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
        }
        else if (tx->is_write && tx->index == IMUHUB_OD_IDX_PDO_MAPPING_B) {
            self->pre_op_state = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ;
        }
        else if (!tx->is_write && tx->index == IMUHUB_OD_IDX_IMU_CONN_MASK) {
            self->imu_connected_mask = data[0];
            self->pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
        }
        self->sdo_retry_count = 0;
    }
    else if (tx->state == AGR_SDO_TX_TIMEOUT) {
        /* Timeout — Retry 로직 (AGR_MW 가 아니라 Device 가 결정) */
        if (self->sdo_retry_count < 3) {
            self->sdo_retry_count++;
            /* WAIT → SEND (재시도) — state 를 이전 SEND 상태로 */
            self->pre_op_state = _PreOpGetSendStateFromWait(self->pre_op_state);
        } else {
            self->pre_op_state = IMUHUB_PRE_OP_IDLE;
            g_xm_bus_diag.sdo_timeout_count++;
        }
    }
    else if (tx->state == AGR_SDO_TX_ABORTED) {
        /* Slave abort — 새로 처리 가능해진 에러 경로 */
        LOG_ERR("IMU Hub SDO abort: idx=0x%04X sub=%u code=0x%08X",
                 tx->index, tx->subindex, abort);

        /* IMU Mask 조회는 optional — abort 되도 NMT START 진행 */
        if (!tx->is_write && tx->index == IMUHUB_OD_IDX_IMU_CONN_MASK) {
            self->pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
        } else {
            self->pre_op_state = IMUHUB_PRE_OP_IDLE;   /* 치명적 — 재시작 */
        }
    }
}

/*
 * ─── RunPeriodic — Timeout 로직 단순화 ─────────────────────
 */
void ImuHub_Drv_RunPeriodic(void) {
    /* ❌ Timeout/retry 체크 block 전체 제거 — AGR_MW_Master_Tick 이 담당 */

    /* AGR_MW SDO Master tick (pnp_task 가 dop_ctx 에 대해 호출) */
    /* AGR_SDO_Master_Tick(&s_imu_hub_inst.dop_ctx.sdo_master, IOIF_TIM_GetTick()); */
    /*   ↑ 이건 pnp_task.c 가 전체 DOP ctx 에 대해 한 번만 호출하는 게 깔끔 */

    _RunPreOpStateMachine(&s_imu_hub_inst);
}
```

#### Migration 후 이점
1. **AGR_MW 레이어 경계 회복** — Device Driver 가 CAN frame 직접 파싱 안 함. 0x580 은 AGR_MW 소관.
2. **Timeout/retry 중복 제거** — `timeout_ms` 파라미터로 AGR_MW 에 위임. Retry 정책만 Device 레벨.
3. **Abort code 처리 가능** — SDO Abort 응답을 `AGR_SDO_TX_ABORTED` + `abort_code` 로 수신. 에러 로그/복구 정밀화.
4. **Transaction 매칭 견고** — `tx->is_write` + `tx->index` + `tx->subindex` 로 정확 매칭 (현재는 CS byte + index 만)
5. **Code 재사용** — FES/EMG Hub driver 도 동일 패턴으로 migration. Boilerplate 감소.

#### 변경 범위 (파일별)

| 파일 | 변경 |
|---|---|
| `imu_hub_drv.c` | `_Step_*` signature 에 `self` 추가, `AGR_CANFD_SendSDO*` → `AGR_CANFD_Master_Request`, `_OnSdoResponse` → `_on_sdo_done`, `ImuHub_Drv_ProcessCANMessage` 의 `fnc_code == 0x580` 분기 제거, `ImuHub_Drv_RunPeriodic` 의 timeout block 제거 |
| `fes_hub_drv.c` | 동일 패턴 적용 |
| `emg_hub_drv.c` | 동일 패턴 적용 |
| `pnp_task.c` | DOP ctx 에 대해 `AGR_SDO_Master_Tick` 주기 호출 추가 (선택, 100ms 주기) |

#### 마이그레이션 순서 권장
1. AGR_MW P1~P5 완료 → submodule pointer bump
2. XM repo 에서 Driver 1개 (IMU) 만 migration PR → QA loopback
3. Green 확인 후 FES / EMG / CM 순차 migration
4. 각 Driver migration 된 후 `SendSDO` / `SendSDOWrite` caller 0건 확인되면 fire-and-forget API deprecation 고려 (장기)

---

## 9. Non-Goals

- ❌ **CiA 1301 USDO 전환** — AGR 이득 없음, migration 비용 과다, DOP 의미론 통일성 훼손
- ❌ **Multi-in-flight per slave** — AGR use case 에 없음. Single-in-flight per slave 충분
- ❌ **Segmented/Block transfer** — 60B single-frame 으로 AGR use case 커버. 대량은 별도 FTP
- ❌ **Polling mode API** — `on_done == NULL` 허용 X. Callback-only + caller flag 패턴
- ❌ **Blocking mode 빌트인** — RTOS 의존 회피. Caller 자체 wrap
- ❌ **ISR-safe callback 보증** — Context 는 transport 에 따름. Caller 가 필요 시 deferred
- ❌ **CANopen LSS / Gateway (CiA 309)** — 별도 기능

---

## 10. 검증

### P1 — Core
- [ ] Mock transport — Read/Write/Abort/Timeout 각 시나리오
- [ ] Transaction pool 가득 참 시 `AllocTx` NULL
- [ ] 같은 slave 에 in-flight 존재 시 `AllocTx` NULL (single-in-flight enforce)
- [ ] `AGR_SDO_Master_Tick` timeout 정확도 (±10ms)

### P2 — CAN-FD
- [ ] 실 HW loopback — XM ↔ SM SDO Read 0x1000
- [ ] Abort code 전파 — 존재하지 않는 index → `AGR_SDO_ABORT_NOT_EXIST`
- [ ] 여러 slave 에 병렬 요청 정상 매칭 (같은 slave 내부는 1-in-flight)

### P3 — Serial
- [ ] USB-CDC loopback — Read/Write/Abort
- [ ] 기존 caller 회귀 0 (wire format 미변경)

### P4 — UDP
- [ ] AM ↔ CM SDO Read/Write
- [ ] 기존 `s_sdo_cb` 경로 회귀 없음
- [ ] CS 분기 — Response 만 Master path 로

### P5 — CoE
- [ ] SOEM 통합 loopback — Read/Write 성공 / `wkc ≤ 0` abort 추출
- [ ] `ec_errlist` walker — `EC_ERR_TYPE_SDO_ERROR` 매칭
- [ ] Sync wrapper — 즉시 callback

---

## 11. 관련 Plan / 메모리

- `plan_common_infra_review.md` §Claim A — int32_t 규약
- `plan_risk_mngr_v2.md` — Fault ack/clear (P6 migration 트리거)
- `plan_pdo_mapping_unified.md` — PDO Mapping SDO Write
- `plan_tech_debt.md §T3` — 본 plan 으로 승격 대체

---

## 12. Kickoff 조건

P1 착수 트리거:
- RiskMngr v2 fault ack flow 구현 단계
- Sensor-Studio 류 UI 런타임 config 요구
- Slave discovery / 버전 호환성 체크 자동화 needs

현 시점 — Design 확정. P1 은 kickoff 대기.

---

## 13. Future Extension — tx_id 추가 여지 (참고만, 현 plan 범위 외)

향후 **multi-in-flight 이 실제로 필요해지는 날** (예: Sensor-Studio 가 여러 채널 config 를 동시 SDO 로 설정) 이 오면, 본 plan 을 깨지 않고 **tx_id 필드만 backward-compatible 하게 추가** 가능:

### Option A — CS byte 의 reserved bit 활용
CiA 301 Command Specifier 는 8-bit 중 하위 몇 bit 만 의미 있음. 예:
- Download Initiate Request: CS = `0x2_` — 상위 3 bit `001` 고정, 하위 5 bit = e/s/n flags
- 일부 reserved bit 에 **Protocol Version flag** 1 bit 정의 → 1 이면 "확장 포맷" 시그널

확장 포맷일 때:
- CS byte 바로 다음 1B 에 tx_id 삽입
- 기존 Index/SubIndex 위치 1B 밀림
- 기존 구현은 flag=0 이라 기존 포맷으로 계속 해석 — backward compat

### Option B — 전용 CS 값 할당
CiA 301 이 정의하지 않은 CS 값 (예: 0x80~0x9F 중 abort 제외 범위) 을 "extended SDO" 로 명명:
- 새 CS 값 사용 시에만 tx_id 포함 payload
- 기존 CS 는 그대로 → 구 Slave 는 새 요청을 "unknown CS" 로 abort

### Option C — CAN-ID 확장 (CAN-FD 한정)
CAN-FD 의 29-bit extended CAN-ID 에 tx_id 비트 할당. Standard 11-bit ID 와 공존 가능:
- 기존: `0x580 + node` (Standard)
- 확장: Extended ID 에 tx_id 포함 (29-bit 활용)
- Transport-specific 이라 UDP/Serial 은 다른 방식 필요

### 권장 트리거 및 설계 원칙
- **실제 use case 발생 후 착수** (premature extension 금지)
- **Backward compat 필수** — 구 Slave / 구 Master 와 섞여도 정상 동작
- **전 transport 일관성 유지** — CAN-FD 에만 적용하고 UDP/Serial 방치하면 DOP 의미론 통일성 깨짐 (본 plan 이 USDO 기각한 이유와 동일)

현재는 **실제 요구가 없으므로 tx_id 도입 비용 > 이득**. 단 **설계 여지는 열어둠**.
