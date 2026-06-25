# AGR_MW — SDO Explicit-Length Implementation Plan

> **Status**: rev3.4 (2026-06-09) — **P1 Core + transport follow-up 완료, P2 Unit 92 checks PASS (gcc 6.3 + clang 12 + CTest/Ninja, exit 0)**. P3(모듈 균일 submodule update) 대기. 미커밋(A1/A2).
> _이전: rev3.3 — XM repo 정정(ARC_ExtensionBoard, 정상 submodule) / 6모듈 균일 update / hard cutover._
> **Owner**: HyundoKim
> **Scope**: AGR_MW DOP Core (SDO encode/decode + OD write 검증 + EMCY) + **6 submodule consumer**: XM(ARC_ExtensionBoard) / IMU / EMG / FES / CM-WH(WalkON_H_CM_Temp) / MD(SAM3x_FW·SAM32_FW_new)
> **Origin**: FES ES-vector(0x6300, 6B) SDO Write 가 CAN-FD DLC 패딩으로 깨지는 현상.
> **관련**: `plan_sdo_master.md`(§D-5), `pdo_mapping_format.md`(self-delimiting), 메모리 `project_pdo_encode_uint16`
> **Philosophy**: DETERMINISTIC > OBVIOUS — 길이를 DLC에서 추론하지 않고 송신측 명시. **전 SDO 단일 포맷(0x21/0x41), 구형 미지원.**

---

## 1. 문제 정의 (실증)
```c
/* agr_sdo_protocol.c:217 — 비-expedited 길이를 프레임 길이로 역산 */
out_msg->data_len = in_len - SDO_HEADER_SIZE;
```
CAN-FD `in_len` = DLC 양자화 물리 길이 → native DLC 에 안 맞으면 패딩이 `data_len` 부풀림. 6B ES-vector: 10B→DLC12→in_len=12→data_len=8 vs size6 → 거부. 영향 = `4+N` 이 native DLC 아닌 모든 `N>4`. ≤4B/PDO매핑(self-delim) 무사 → 미발견 이유.

---

## 2. 설계 결정 (확정, rev3.3)

| # | 결정 | 근거 |
|---|---|---|
| **D-1** | 모든 SDO 단일 length-bearing 포맷: write=`0x21`, read-resp=`0x41` (e=0, s=1, 2B 명시 길이). ≤4B 포함 | 예외 0 |
| **D-2** | 구형(expedited 0x2x / frame-len 0x20 / 0x43\|n) **전면 미지원** — 송신 안 함 + 수신 시 **abort `INVALID_CS`**. config-gate/legacy 분기 없음 | 단일 포맷, 혼동 제거 |
| **D-3** | size 필드 **2B (uint16 LE)**. `data_len`·SDO API 전부 uint16. `data[]` 버퍼(`MAX_DATA_SIZE`)는 별도 RAM knob | UDP/Serial 단일패킷>255. transport-agnostic |
| **D-4** | 수신 `declared > MAX_DATA_SIZE` / frame 미수용 → OD write 안 함 + abort + diag 카운터 + EMCY(**코드 TODO**) | 오염 방지 + 관측성 |
| **D-5** | Decoder = `0x21`/`0x41`/`0x40`(read-req,데이터0)/`0x60`/`0x80` 만 수용. 그 외 length-bearing → abort. `f0106cd` expedited 분기 제거 | 단일 포맷 |
| **D-6** | `entry->size` uint8 유지 (OD 엔트리 ≤255B) | type 부담 회피 |

> **Ceiling**: 와이어 size 65535 / `data_len` uint16 / `data[]` 60(config) / **CAN-FD 단일프레임 data ≤ 58**(64-6) / OD entry ≤255.
> **Cutover**: 구형 미지원 = 전 모듈 동시 hard cutover. **6모듈 전부 비양산 → 동시 플래시 가능(사용자 확정).**

---

## 3. 와이어 포맷 (단일)
### 3.1 Write Request — `CS = 0x21` (전 크기)
```
 byte0  byte1-2   byte3    byte4-5          byte6 ...
[0x21][ index ][subidx][ size (u16 LE) ][ data ... ]
CAN-FD data ≤ 58B   1B write:7B   6B ESvec:12B(DLC12, size로 확정)
```
- 길이는 encode 가 `msg->data_len` 에서 자동 기록 (호출자는 `data_len=sizeof` 만 전달).
### 3.2 Read Response — `CS = 0x41` — `[0x41][idx][sub][size u16][data...]`
### 3.3 미지원 (Rejected) — `0x2F/0x2B/0x27/0x23`·`0x43|n`·`0x20`: 송신 안 함 + 수신 시 abort `INVALID_CS`.
### 3.4 변경 없음 — Read Req `0x40`(데이터0) / Write Resp `0x60`(8B) / Abort `0x80`(4B).

---

## 4. AGR_MW Core 변경 (파일별)
### 4.1 `DOP/agr_dop_types.h`
1. **주석 수정(1순위)** — line 197-198 "expedited 60바이트 e=1 유지" → 정정(명시 길이, e=0).
2. CS: `..._DOWNLOAD_INIT_REQ_SIZED=0x21`, `..._UPLOAD_INIT_RSP_SIZED=0x41`. `*_EXP`·`0x20` deprecated/제거 표시.
3. **`AGR_SDO_Msg_t.data_len`: uint8→uint16**. 동반 API uint16.
4. `AGR_DOP_Ctx_t` 진단: `AGR_DOP_SdoDiag_t { uint16_t sdo_oversize_count, sdo_decode_err_count, last_bad_index; }`.
5. `AGR_SDO_MAX_DATA_SIZE` `#ifndef`(기본60). `SDO_SIZE_FIELD`(2)·`SDO_EXPLICIT_OFFSET`(6).

### 4.2 `DOP/Core/agr_sdo_protocol.c`
- **`AGR_SDO_Encode(msg, out_buf, out_buf_len)`** — ★ buf_len 인자 추가. 0x21/0x41 → byte4-5=size, data@6, `need=6+data_len`; `need>out_buf_len → -2`.
- **`AGR_SDO_Decode`** — 0x21/0x41: `s=1` 확인, `sz=buf[4]|buf[5]<<8`(`sz>MAX→-3`), data@6. `0x40`(read-req): data_len=0. `e=1`(0x2x)/`s=0 download`(0x20) → abort `INVALID_CS`(-4). `f0106cd` expedited 분기 삭제.
- **`AGR_SDO_CreateWriteReq`** — `cs=0x21` 무조건. **`_ProcessSDOReadRequest`** — `cs=0x41` 무조건.
- **`_ProcessSDOWriteRequest`** — clamp 불필요(명시 길이 정확). `AGR_OD_WriteValue` 가드 방어선 유지.

### 4.3 `DOP/Core/agr_od.c` — 로직 변경 없음, len uint16.
### 4.4 Transport (구현 완료 반영, rev3.4)
- **CAN-FD** (`agr_dop_canfd.c`): Encode 호출부 `sizeof(buf)` 전달. RX 디코드 실패 → `sdo_diag.sdo_decode_err_count++` (`-3` 시 `sdo_oversize_count++` + `last_bad_index`) + abort(`-3`→`DATA_LONG` / else `INVALID_CS`) `tx_func` 응답.
- **Serial** (`agr_dop_serial.c`): ★ `AGR_SDO_Decode(payload, (uint8_t)…)` **절단 버그 교정 → `(uint16_t)`** (256B+ COBS 프레임). 디코드 실패 → diag 증가 + abort `AGR_Serial_SendSDO`. Encode 호출부 `sizeof(payload)`.
- **CoE** (`agr_dop_coe_slave.c`): ~~변경 없음~~ **정정** — Decode 실패 시 diag 카운터 증가(`-3`→oversize)만 추가, **abort 는 SOEM mailbox 계층 처리**. Encode 는 `MAX+6` bound.
- **UDP** (`agr_dop_udp.c`): transport 에서 SDO **decode 안 함** (raw → `s_sdo_cb` 콜백, device-layer 책임) → diag 배선 대상 아님. Encode 호출부 `sizeof(payload)`.
- **EMCY**: 코드값 **TODO 유지** — 현재 abort + `sdo_diag` 카운터(Risk Manager 폴링)가 interim. `AGR_*_SendEmergency` 배선은 error_dictionary 등록 후.

---

## 5. Rollout (hard cutover — 구형 미지원)
**제약**: 신형↔구형 혼재 시 양방향 SDO 단절(구형 디코더는 0x21 오인, 신형은 0x20 거부) → **전 모듈 동시 cutover.** 6모듈 전부 비양산 → 벤치 일괄 플래시 가능.

**AGR_MW 갱신 = 전 모듈 균일 git submodule update** (정석, 전부 submodule 확인됨 §6.0):
```
git -C {module} submodule update --remote {path}/AGR_MW   # 새 0x21 commit 으로
# → 각 모듈 repo 에 submodule pointer 커밋 + 빌드
```
**절차**: ① AGR_MW Develop 에 0x21 commit → ② 6 모듈 submodule update + pointer 커밋 → **③ 6 모듈 동시 빌드·플래시·SDO 테스트**. 부분 플래시 상태 SDO 테스트 금지.

---

## 6. 모듈 검증 (직접 read, 2026-06-09)
### 6.0 통합·capability
| 모듈 (경로) | AGR_MW 통합 | sdo_protocol.c vs source | hand-rolled 디코더 | 판정 |
|---|---|---|---|---|
| **XM** (ARC_ExtensionBoard/Extension_Module/XM_FW) | ✅ submodule @`f8bad0f` | **동일(diff=0)**, node_id 사용 0 | 없음 (device 3곳 `AGR_SDO_Decode` 호출) | ✅ clean update |
| **IMU** (IMU_Hub_Module) | ✅ submodule @`28325e8` | 동일 | 없음 | ✅ |
| **EMG** (EMG_Hub_Module) | ✅ submodule @`28325e8` | 동일 | 없음 | ✅ |
| **FES** (FES_HUB_MODULE) | ✅ submodule @`28325e8` | 동일 | 없음 | ✅ |
| **CM-WH** (WalkON_H_CM_Temp) | ✅ submodule @`28325e8` | 동일 | 없음 | ✅ |
| **MD** (SAM3x_FW/SAM32_FW_new) | ✅ submodule @`0159d0e` | 동일 | 없음 | ✅ (bring-up 중) |
| MD `SAM32_FW`(구) | ❌ 별도 `DOP_Mngr` | 미사용 | — | N/A legacy |

→ **6모듈 전부 정상 submodule + content 동일(diff=0) + hand-rolled 디코더 0** → 균일 `git submodule update`. **XM 특수성 없음**(정정: 이전 "flat-vendored/stale" 은 stray `Extension_Module` 오인).

### 6.1 플래그
- **node_id**: 실제 XM(ARC)은 AGR_MW 소비자코드 node_id 사용 0 → **이미 System/Config 이전 완료(비이슈).** (AGR_MW HEAD 는 `agr_dop_node_id.h` 미보유.)
- **XM Dev/Release 구분**: **`ARC_ExtensionBoard` = XM Dev**(전체 소스, AGR_MW submodule @`f8bad0f` — 작업 대상). **`Extension_Module` = XM Release**(웹 튜토리얼/SDK 배포, XM_FW static-lib 화로 내부 은닉 → AGR_MW flat 스냅샷, **작업 금지**). → **0x21 반영 후 XM Release SDK 재생성**은 Dev 반영 뒤 release pipeline 으로 별도 (`module/32-xm-sdk-pipeline`, A30 stub 빌드 금지).
- **MD 통신 = SAM32_FW_new 전용**. 구 MD(SAM32_FW) legacy — 대상 아님.
- **CM-H10 (Central_Control_Module) = 작업 금지** (A19).
- **CM-WH `md_drv_v3.c:570`** `if(data_len>4) /* expedited only */`: 0x21 후 무해, 주석 cleanup 권장(별도).

---

## 7. Phase
| P | 작업 | 산출 |
|:--:|---|---|
| P0 | plan rev3.3 확정 | ✅ |
| P1 | Core: 주석/CS/diag/MAX + `data_len` uint16 + Encode(buf_len)/Decode(0x21-only) + CreateWriteReq(0x21)/ReadResp(0x41) + transport diag/EMCY(TODO) + 직접 호출부 audit | ✅ (8파일 + Serial 절단 교정 + CoE diag, thread-safety-audit PASS) |
| P2 | Unit(host): roundtrip 1/4/5/6/37/58B, 0x41(37B), oversize→-3, 구형→-4 INVALID_CS, CAN-FD 59→-2, OD 0x6300 burst_ms, read 0x40→0x41, 짧은프레임/NULL | ✅ **92 checks 0 fail** (`test/`, gcc 6.3 + clang 12 + CTest/Ninja) |
| P3 | 6 모듈 균일 submodule update + pointer 커밋 → 동시 빌드 (명시 승인 필요, A27) | green |
| P4 | 6 모듈 동시 cutover(플래시) → ES-vector burst_ms + CM-WH↔MD + oversize→EMCY HW loopback | green |
| P5 | (옵션) `data[]` 확대 + entry->size uint16 — UDP/CoE 대용량 시 | 트리거 시 |

---

## 8. 검증
**Unit** ✅ **구현·통과** — [`test/test_sdo_explicit_length.c`](../test/test_sdo_explicit_length.c) (T1~T10, 92 checks): Encode(0x21,6)=12B & byte4-5=`06 00` / Decode(0x21,padded16,size6)→6(DLC 무관) / 0x41(37)→37 / oversize 61→-3 write無·경계60 OK / 구형 0x20·0x23/27/2B/2F·0x43/47/4B/4F → -4 INVALID_CS / CAN-FD Encode 58→64·59→-2 / OD 0x6300 burst_ms=500 반영·on_write 1회·구형 len16 거부 / read 0x40→0x41 / 짧은프레임·NULL 가드. 실행: `test/README.md`.
**HW**: 6모듈 동시 플래시 후 — ES-vector 5필드 일치 / 매핑 0x21 PnP / CM-WH↔MD(0x21) / XM 응답 디코드 / Serial·UDP roundtrip / oversize→abort+EMCY. (부분 플래시 SDO 테스트 금지.)

---

## 9. Risks
| Risk | 완화 |
|---|---|
| 구형 미지원 hard cutover → mixed-firmware 단절 | 6모듈 동시 플래시(전부 비양산), 부분 상태 테스트 금지 |
| `AGR_SDO_Encode` 시그니처(buf_len) → 직접 호출부 깨짐 | P1 전 모듈 직접 호출부 audit |
| CAN-FD 6B 헤더 + MAX60=66>64 오버플로 | Encode buf_len + data≤58 거부 |
| EMCY 코드 미확정 | **TODO** — error_dictionary 등록 후 확정 |
| MD(SAM32_FW_new) bring-up 진행중 → bump 충돌 | MD dev 와 시점 조율 |
| 다중 repo 동시 release | 모듈 owner 간 release window |
| OD write 콜백(`_OnESVectorWrite`) thread-safety | `thread-safety-audit` Skill 필수 |
| 모듈 경로 혼동(XM=ARC 교훈) | 워크스페이스 추가 후 실제 경로 재확인 |
| V1 H10 CM / 구 MD 수정 | 작업 금지 (A19/legacy) |

---

## 10. Non-Goals
❌ Segmented/Block(대량=FTP) ❌ 구형 backward-compat ❌ `data[]`확대/entry->size uint16(P5) ❌ CoE 와이어 변경 ❌ CM-H10/구MD 수정 ❌ stray Extension_Module 작업 ❌ 자동 commit/push/submodule 조작(A1/A2/A27 — 명시 시에만)
