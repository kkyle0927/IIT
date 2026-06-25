# AGR_MW — PDO Mapping Wire Format

> **Status**: Active (2026-04-21)
> **Scope**: `AGR_PDO_ApplyMapFromSDO()` 가 수신하는 SDO payload 의 바이트 레이아웃 정의
> **관련 설계 문서**: [`plan_pdo_mapping_unified.md`](plan_pdo_mapping_unified.md)

---

## 1. 개요

AGR_MW 는 Transport-Agnostic (CAN-FD / EtherCAT CoE / UDP / Serial) 프레임워크이며, PDO Mapping 은 다음 두 포맷을 모두 수용한다.

| 포맷 | 크기/Entry | 최대 표현 | 용도 |
|---|---|---|---|
| **Legacy 4B** | 4 B | 31.875 B (255 bits) | CiA 301 호환 (deprecation 예정) |
| **Extended 8B** | 8 B | 4 GB | AGR 기본 — CAN-FD 64B, CoE/UDP 대형 payload 대응 |

포맷 판별은 **SDO payload 의 첫 바이트 (Header byte)** 의 bit 7 로 수행된다.

---

## 2. Header byte (data[0])

```
  bit 7        6      5      4      3      2      1      0
┌──────┬──────────────────────────────────────────────────┐
│ EXT  │          Number of mapped objects (N, 0..127)    │
└──────┴──────────────────────────────────────────────────┘
```

- **bit 7 (EXT)**: `0` = Legacy 4B entry, `1` = Extended 8B entry
- **bit 0..6 (Count)**: 매핑 엔트리 수 (최대 127). `0` 은 매핑 클리어 효과.

---

## 3. Legacy 4B Entry (EXT = 0)

CiA 301 원본 포맷과 바이트 단위 호환.

```
  byte 0    byte 1    byte 2    byte 3
┌─────────┬─────────┬─────────┬─────────┐
│ BitLen  │ SubIdx  │ Index L │ Index H │
└─────────┴─────────┴─────────┴─────────┘
```

- **BitLen** (uint8): 매핑 크기 (bits). 최대 255 bits = 31.875 B.
- **SubIdx** (uint8): OD Sub-Index.
- **Index** (uint16 LE): OD Index.

### Legacy 의 BitLen = 0 관용 동작

CiA 301 미정의. AGR 확장으로 `BitLen = 0 → AGR_MW 가 OD entry->size 를 사용` 을 허용한다. Phase 1 에서는 호환 유지 목적으로 보존하며, **Legacy 포맷 deprecation 시점에 함께 제거**한다.

### SDO 단일 프레임 수납량

- AGR SDO atomic = 60 B
- Header 1B + Entry 4B × N → **N ≤ 14**

---

## 4. Extended 8B Entry (EXT = 1)

AGR 자체 확장. Transport-Agnostic 전 영역에서 권장.

```
  byte 0    byte 1    byte 2    byte 3    byte 4    byte 5    byte 6    byte 7
┌─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┐
│ Index L │ Index H │ SubIdx  │  Flags  │ Len B0  │ Len B1  │ Len B2  │ Len B3  │
└─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┘
```

- **Index** (uint16 LE): OD Index.
- **SubIdx** (uint8): OD Sub-Index.
- **Flags** (uint8):
  - bit 0: signed (1 = signed integer/float, 0 = unsigned/raw)
  - bit 1..7: reserved (0 으로 송신, 수신 측 무시)
- **Length** (uint32 LE): 매핑 크기 (bytes). 최대 4 GB.
  - `0` 은 Legacy 와 동일하게 OD entry->size fallback 을 적용.

### SDO 단일 프레임 수납량

- AGR SDO atomic = 60 B
- Header 1B + Entry 8B × N → **N ≤ 7**

AGR_MW 의 기본 `AGR_PDO_MAP_MAX_ENTRIES = 32` 보다 작으므로 32 개 이상을 한 번에 전송하려면 SDO Block Transfer 도입 필요 (현재 roadmap 밖).

---

## 5. 런타임 구조체 (`AGR_PDO_MapItem_t`)

Wire format 해석 결과는 Legacy / Extended 구분 없이 다음 구조체로 통일 보관된다.

```c
typedef struct {
    uint16_t od_index;       // OD Entry Index
    uint8_t  od_subindex;    // OD Entry Sub-Index
    uint8_t  flags;          // Extended 포맷의 Flags byte (Legacy 는 0)
    uint32_t length_bytes;   // 매핑 크기 (bytes)
} AGR_PDO_MapItem_t;
```

런타임 API 변경점:
- `AGR_PDO_AddMap()` — 시그니처 유지, 내부에서 `length_bytes = entry->size`.
- `AGR_PDO_AddMapExt()` — **신규**, `length_bytes` 와 `flags` 를 명시. BLOB 타입 OD 엔트리 (예: 37 B 구조체 TPDO payload) 를 단일 엔트리로 선언할 때 사용.

---

## 6. 마이그레이션 상태 (Phase 추적)

| Phase | 내용 | 상태 |
|-------|------|------|
| Phase 1 | AGR_MW Core 확장 (구조체 + ApplyMapFromSDO + AddMapExt) | **완료** (2026-04-21) |
| Phase 2 | SM 드라이버 `s_tpdo_map[]` Extended 포맷 전환 | SM 각 레포 별도 세션 |
| Phase 3 | Cross-Transport 검증 (CAN-FD / CoE / UDP / Serial) | TBD |
| Phase 4 | Legacy 4B 파서 deprecation → 제거 | 분기 단위 |

`feedback_cia_full_compliance` 정책: Legacy 는 **deprecation release 를 반드시 거친 뒤** 제거. 현재는 하위 호환 목적으로 유지.

---

## 7. 외부 소비자 주의사항

AGR_MW 는 `AGR_PDO_MapItem_t::bit_length` (uint8) 를 **`length_bytes` (uint32) 로 교체**했다. 외부 레포에서 구조체 필드를 직접 접근하던 코드는 호환성 영향:

| 레포 / 파일 | 영향 | 대응 |
|---|---|---|
| `EMG_Hub_Module/EMG_Hub_FW/System/Comm/USB/emg_dop_serial.c:327` (`_BuildTpdoMappingBlob`) | 컴파일 에러 (`bit_length` 필드 없음) | Legacy 4B blob 직렬화 시 `(uint8_t)(items[i].length_bytes * 8u)` 로 계산하여 BitLength byte 생성 |

구조체 사용하지 않고 `AGR_PDO_Encode()/Decode()/ApplyMapFromSDO()` API 만 호출하는 소비자는 영향 없음.
