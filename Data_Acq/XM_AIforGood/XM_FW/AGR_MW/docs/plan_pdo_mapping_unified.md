# AGR_MW — Unified PDO Mapping Format Upgrade

> **Status**: Planning (2026-04-21)
> **Owner**: HyundoKim
> **Scope**: AGR_MW Core (Transport-Agnostic), 모든 DOP Transport 공통
> **Trigger**: XM ↔ FES Hub TPDO 37B 전송 과정에서 Mapping 선언-실제 불일치 발견

---

## 1. 배경 — 왜 지금 바꿔야 하는가

### 1.1 현재 상태 (2026-04-21 기준)

AGR_MW 의 PDO Mapping 엔진은 **CiA 301 원본 4B 엔트리 포맷을 그대로 채택**.

| 필드 | Bit | 제약 |
|------|-----|------|
| Index | 16 | 0x0000 ~ 0xFFFF OK |
| SubIndex | 8 | 0x00 ~ 0xFF OK |
| **BitLength** | **8** | **max 255 bits = 31.875 B** |

코드 증거:
- `AGR_MW/DOP/Core/agr_pdo_engine.h:22-28` — 주석 "PDO Mapping Format - CANopen 표준 4B" 명시
- `AGR_MW/DOP/Core/agr_pdo_engine.c:107` — `uint8_t bit_len = data[i];` 8bit 파싱
- `AGR_MW/DOP/agr_dop_types.h` — `AGR_PDO_MapItem_t::bit_length` = uint8_t

### 1.2 문제 노출 사례 — FES Hub TPDO 37B

- FES Slave OD `0x7018.0x00` entry size = **37 B** (`sizeof(XM_Drv_TpdoPayload_t)`)
- Master 측 `s_tpdo_map[]` 선언: `BitLen=0x80 (128bit = 16B)` — 37B 표현 **불가능**이라 임의 축소
- 실제 동작: `AGR_PDO_Encode()` 가 `bit_length` 를 무시하고 `entry->size` (37B) 로 송신
- 결과: **선언 16B / 실제 37B** 영구 불일치, 분석 툴·외부 스택 호환 불가

### 1.3 설계상의 근본 원인

AGR_MW 는 이미 **Transport-Agnostic** 설계:
- CAN-FD (단일 프레임 64 B)
- EtherCAT CoE (PDO 공간 1 KB+)
- UDP (MTU 1500 B)
- Serial / USB-CDC / COBS (가변)

**위 Transport 중 Classical CAN(8B)에 제약되는 경우는 하나도 없다.** 그럼에도 Mapping 선언 포맷만 Classical CAN 시대(CiA 301, 1993 ~ 2000년대 초) 제약을 짊어지고 있음. 이 제약이 CAN-FD / CoE / UDP / Serial 전부에 전염되어, 한 Entry 당 31B 이상의 OD 객체는 Mapping 으로 선언할 수 없다.

### 1.4 언제 깨지는가 — Risk Matrix

| # | 트리거 | 영향도 | 시점 |
|---|--------|--------|------|
| R-1 | AGR_PDO_Encode / Decode 에서 `bit_length` 를 다시 존중하도록 리팩토링 | **치명적** — 모든 > 31B TPDO 즉시 깨짐 | 언제든지 (코드 리뷰 1줄) |
| R-2 | 외부 CANopen / EtherCAT 분석 툴 (CANoe, TwinCAT, CAN-View) 와 상호운용 | 높음 — TPDO 파싱 오류 | Sensor-Studio / PC GUI 기능 확대 시 |
| R-3 | CiA 1301 (CANopen FD) 표준 호환 필요 시점 | 중간 — 인증/호환 리스크 | Field Deployment 시 |
| R-4 | 새 엔지니어가 Mapping 주석 믿고 OD 재설계 | 중간 — 설계 오판 | 신규 온보딩 시 |

**현재 AGR SDO 60B atomic 제약** 때문에, 다중 subindex 분할로 우회하는 CiA 301 표준 방식도 불가 (`28 entry × 4B = 113B > 60B`).

---

## 2. 요구사항 — Unified Core 의 조건

### 2.1 기능 요구사항

- **F-1**: 단일 Mapping Entry 가 **최소 64 B** (CAN-FD 프레임 크기), 이상적으로 **최대 1 KB** 선언 가능
- **F-2**: CAN-FD / CoE / UDP / Serial 모든 Transport 에서 **동일 포맷** 사용
- **F-3**: CiA 301 (Classical) 호환 OPT-IN 플래그 (필요 시 레거시 4B 모드 선택)
- **F-4**: CiA 1301 (CANopen FD) USDO 와 공존 가능
- **F-5**: OD 의 TYPE_BLOB (가변 크기 구조체) 을 단일 Entry 로 표현
- **F-6**: SDO 단일 프레임 (60 B atomic) 안에 Mapping Write 완전 수납 (세그멘테이션 회피)

### 2.2 비기능 요구사항

- **NF-1**: 기존 AGR_PDO_Encode / Decode 의 `entry->size` 기반 동작 **유지** (이미 올바름)
- **NF-2**: 기존 SM 드라이버 (IMU / EMG / FES) 의 OD 구조 **변경 없이** Mapping 선언만 업그레이드
- **NF-3**: 마이그레이션은 **Transport 1개씩** 단계적 적용 (Big-Bang 금지)
- **NF-4**: 하위 호환 기간 동안 구 포맷 파싱 유지 (deprecation warning)

---

## 3. 설계 옵션 비교

### 옵션 A — 8 B Entry 확장 (BitLength → 16 bit Length)

```
┌──────────────────┬──────────────┬──────────────┬──────────────────┐
│  Index (16 bit)  │ Sub (8 bit)  │ Rsvd (8 bit) │  Length (32 bit) │
└──────────────────┴──────────────┴──────────────┴──────────────────┘
   bits 0..15         bits 16..23    bits 24..31      bits 32..63
```

- Length 필드를 **32bit (bytes 단위)** 로 확장. 최대 4 GB (실용상 무한)
- SDO Mapping Write: `8B × N + 1B(count)` = 64B 프레임 내 최대 7 entries
- **장점**: 표현력 완전, 단순, 분석 툴이 header 만 보면 포맷 판별 가능
- **단점**: 기존 4B 포맷과 바이너리 비호환 → Protocol Version 필요

### 옵션 B — 6 B Entry 확장 (BitLength → 16 bit)

```
┌──────────────────┬──────────────┬──────────────────┐
│  Index (16 bit)  │ Sub (8 bit)  │  BitLen (24 bit) │
└──────────────────┴──────────────┴──────────────────┘
```

- BitLength 필드를 24bit 로 확장. 최대 2 MB
- **장점**: 원본 포맷 유지하며 필드만 확장 (파서 최소 변경)
- **단점**: 6B 라 64B 프레임에 10개까지 수납, CoE 도구 호환성 미검증

### 옵션 C — 원본 4B 유지 + "Length=0 → use OD entry->size" 공식화

- CiA 301 에서는 미정의. AGR 자체 확장으로 명문화.
- 이미 `AGR_PDO_ApplyMapFromSDO` 에 구현됨 (`bit_len != 0 ? bit_len : entry->size * 8`)
- **장점**: 변경 0
- **단점**: 선언력 완전 포기, 분석 툴은 여전히 BitLen=0 을 오류로 해석

### 권장 — 옵션 A (8B Entry)

| 기준 | A (8B) | B (6B) | C (유지) |
|------|--------|--------|----------|
| 표현력 | 4 GB | 2 MB | entry->size 의존 |
| 파서 복잡도 | 낮음 | 중간 | 없음 |
| 분석 툴 | 필요 시 스키마 공유 | 애매 | 혼란 지속 |
| 향후 확장 (메타데이터 등) | Rsvd 필드로 여유 | 없음 | 불가 |
| 마이그레이션 | 버전 플래그 필요 | 경계 불명확 | 불필요 |

---

## 4. 제안 설계 (옵션 A 기반)

### 4.1 Wire Format

**SDO Mapping Write Data** (CiA 301 "0x1A00.xx" Sub-Index Write 대체):

```
Byte 0    : Format Version    (0x00 = Legacy 4B, 0x01 = Extended 8B)
Byte 1    : Number of Mapped Objects (N)
Byte 2..  : Entry[0..N-1], 각 Entry 는 Version 에 따라 4B 또는 8B

// Version = 0x01, 8B Entry:
Byte 0..1 : Index (LE)
Byte 2    : SubIndex
Byte 3    : Flags (bit 0 = sign, bit 1..7 = reserved)
Byte 4..7 : Length in bytes (LE uint32)
```

SDO 단일 프레임 60B 내 수납:
- Legacy (4B entry): `1 + 1 + 14×4 = 58B` → 14 entries
- Extended (8B entry): `1 + 1 + 7×8 = 58B` → 7 entries

**향후 SDO Block Transfer 도입 시 제약 제거 가능**.

### 4.2 Core API 변경

```c
// Before
typedef struct {
    uint16_t od_index;
    uint8_t  od_subindex;
    uint8_t  bit_length;    // max 255 bits
} AGR_PDO_MapItem_t;

// After
typedef struct {
    uint16_t od_index;
    uint8_t  od_subindex;
    uint8_t  flags;
    uint32_t length_bytes;  // extended
} AGR_PDO_MapItem_t;
```

- `AGR_PDO_ApplyMapFromSDO()` — version byte 분기 추가 (legacy 4B + extended 8B)
- `AGR_PDO_Encode() / Decode()` — `entry->size` 기반 동작 유지 (변경 없음, 이미 올바름)
- `AGR_PDO_AddMap()` — 시그니처 유지, 내부에서 `length_bytes = entry->size`

### 4.3 Transport 별 적용

| Transport | 변경 범위 | 위험도 |
|-----------|----------|--------|
| CAN-FD (`agr_dop_canfd.c`) | SDO Mapping Write payload 만 | 낮음 — Encode/Decode 영향 없음 |
| CoE Master / Slave (`agr_dop_coe_*.c`) | 동일 — SDO 프레임 payload | 낮음 |
| UDP (`agr_dop_udp.c`) | SDO payload | 낮음 |
| Serial / COBS (`agr_dop_serial.c`) | SDO payload | 낮음 |

Transport 코드 자체는 **AGR_PDO_Encode() 의 return len** 을 그대로 전송하므로 수정 불필요. Mapping SDO 송·수신 로직만 버전 분기.

### 4.4 OD Index 할당

- CiA 301: TPDO Mapping = `0x1A00 ~ 0x1A03` (Sub 0 = count, Sub N = Entry N)
- AGR 확장: **동일 0x1A00 사용, Sub 0 = Format Version + Count 묶음**
  - `Sub 0` (uint8): bit 7 = Extended flag, bit 0..6 = Count
  - `Sub 1..N` (uint32 or uint64): Entry (포맷은 Sub 0 의 Extended flag 에 따름)

구 Slave 가 `Sub 0 = 0x01` (count=1) 로 Write 받으면 기존 대로 4B entry 파싱. 신 Slave 가 `Sub 0 = 0x81` (Extended + count=1) 로 Write 받으면 8B entry 파싱.

**하위 호환 + 명확한 스위치**.

---

## 5. 마이그레이션 전략 (Phase)

### Phase 0 — 단기 임시 우회 (이번 주)

- FES `s_tpdo_map[]` 의 BitLen=0 으로 설정 → AGR_MW 의 기존 관용 동작(`bit_len != 0 ? bit_len : entry->size * 8`) 이용
- Rev1.1 + Rev2.0 양쪽 1바이트 수정
- 주석 정리 (`/* BitLen=0 → AGR_MW auto-resolve from OD entry size (37B) */`)
- **목적**: R-1 (리팩토링 시 즉시 깨짐) 리스크 제거. Phase 1 완료 전까지 버팀.

### Phase 1 — AGR_MW Core 확장 (2~3주)

- `AGR_PDO_MapItem_t` 필드 확장 (uint8_t → uint32_t length_bytes)
- `AGR_PDO_ApplyMapFromSDO()` 에 Format Version 분기 추가
- `AGR_PDO_AddMapExt()` 신규 API (length_bytes 명시 파라미터)
- Unit test (legacy 4B + extended 8B 양방향 파싱 검증)
- 문서: AGR_MW/docs/pdo_mapping_format.md

### Phase 2 — SM 드라이버 전환 (1주)

- FES `s_tpdo_map[]` → Extended 8B 포맷 (37B 명시 선언)
- IMU / EMG Hub 도 TPDO > 31B 경우 Extended 전환
- 각 Slave 레포 커밋 (센서모듈 각자 대화세션에서 진행)

### Phase 3 — Cross-Transport 검증 (2주)

- XM ↔ FES (CAN-FD): 37B TPDO Mapping 선언 = 실제 = 분석 툴 일치
- AM ↔ CM (UDP): UDP PDO Mapping 확장 포맷 검증
- EtherCAT CoE Slave: CoE Master 측 XDD / ESI 파일 업데이트
- Serial: Sensor-Studio GUI Mapping 파싱기 업그레이드

### Phase 4 — Legacy Deprecation (분기 단위)

- 모든 SM / XM / CM / AM 이 Extended 포맷 사용 확인 후, Legacy 4B 파서 deprecation warning 추가
- 1분기 후 Legacy 파서 제거 옵션 (Compile flag `AGR_PDO_LEGACY_MAPPING=OFF`)

---

## 6. 참고 — 영향받는 코드 포인터

### AGR_MW Core
- `DOP/Core/agr_pdo_engine.h:22-28` — 포맷 주석 업데이트 대상
- `DOP/Core/agr_pdo_engine.c:76-135` — `AGR_PDO_ApplyMapFromSDO` 파싱 로직
- `DOP/Core/agr_pdo_engine.c:141-215` — Encode / Decode (변경 없음 예상)
- `DOP/agr_dop_types.h:169` — `AGR_PDO_MapItem_t` 필드 변경

### 사용처 (Grep 결과 기준)
- `DOP/Transport/CAN_FD/agr_dop_canfd.c:180`
- `DOP/Transport/CoE/agr_dop_coe_master.c:220`
- `DOP/Transport/CoE/agr_dop_coe_slave.c:215, 272`
- `DOP/Transport/UDP/agr_dop_udp.c:104`
- `DOP/Transport/Serial/agr_dop_serial.c:156`

### Device Driver (XM 측, Master)
- `Extension_Module/XM_FW/Devices/AGR/FES_Module/fes_hub_drv.c:115-119` (s_tpdo_map)
- `Extension_Module/XM_FW/Devices/AGR/IMU_Module/imu_hub_drv.c` (s_tpdo_map)
- `Extension_Module/XM_FW/Devices/AGR/EMG_Module/emg_hub_drv.c` (s_tpdo_map)

### Slave 측 OD 정의 (각 Sensor Module 레포)
- `IMU_Hub_Module/.../xm_drv.c` — 0x7010 IMU TPDO OD entry
- `EMG_Hub_Module/.../xm_drv.c` — 0x6010 EMG TPDO OD entry
- `FES_HUB_MODULE/.../xm_drv.c` — 0x7018 FES TPDO OD entry (37B)

---

## 7. 미해결 / 추가 검토 항목

- CiA 1301 공식 문서 확보 후 USDO / PDO Mapping 포맷 실제 상태 팩트체크 필요 (현재는 AGR 자체 정의로 진행 가정)
- CoE 기반 EtherCAT Master (TwinCAT, SOEM) 가 Extended 포맷을 수용 가능한지 — SOEM 소스 코드 확인 필요
- OD Index 0x1A00 의 Sub 0 bit 7 를 Extended flag 로 재정의하는 것이 CiA 301 과 충돌하지 않는지 (`Sub 0 = 0x00 ~ 0x40` 만 표준 정의)
- Mapping Entry 내 Flag 필드의 활용도 (sign bit 외에 byte-swap, endian 표기 등)

---

## 8. 관련 메모리

- `project_agr_mw_pdo_mapping_upgrade.md` — 결정 요약 + Why + How to apply
- `project_dop_whitepaper_series.md` — DOP V3 백서 시리즈 (해당 업그레이드 완료 시 09권으로 편입 검토)
- `reference_agr_sdo_canfd_extension.md` — AGR SDO 60B atomic 제약 (본 설계가 이 제약 내에서 수납 가능하도록 설계됨)
