# AGR_MW DOP — Host Unit Tests

AGR_MW 는 `module_type: library` (소비 모듈의 CMake 에서 빌드). 이 디렉토리는 그와
**독립적인 호스트 테스트**로, DOP Core(순수 C, HAL/RTOS 무의존)만 컴파일해 SDO
프로토콜 로직을 검증한다. 펌웨어 빌드/플래시와 무관하므로 PR 게이트/CI 에서 즉시 실행 가능.

## 실행

### 1) 원샷 (gcc/clang)
```sh
gcc -std=c11 -Wall -Wextra -I DOP -I DOP/Core \
    test/test_sdo_explicit_length.c \
    DOP/Core/agr_sdo_protocol.c DOP/Core/agr_od.c -o test/test_sdo
./test/test_sdo        # exit 0 = 전체 통과
```

### 2) CMake + CTest
```sh
cmake -S test -B test/build
cmake --build test/build
ctest --test-dir test/build --output-on-failure
```

## 커버리지 — `test_sdo_explicit_length.c`

| # | 검증 |
|---|------|
| T1 | 0x21 write 인코딩 레이아웃 (12B, byte4-5 = size LE) |
| T2 | **핵심 회귀** — DLC 패딩(16B 프레임)에도 `data_len` 은 size 필드(6)에서 확정 |
| T3 | CreateWriteReq→Encode→Decode 라운드트립 1/4/5/6/37/58 B |
| T4 | 0x41 read-resp 37B 라운드트립 |
| T5 | declared > MAX(60) → `-3` 거부(데이터 미반영) / 경계 60 수용 |
| T6 | 구형 expedited(0x23/27/2B/2F·0x43/47/4B/4F)·frame-len(0x20) → `-4` INVALID_CS |
| T7 | CAN-FD 단일프레임 58B 상한 / Encode 버퍼 오버플로 가드(59B→`-2`) |
| T8 | OD 통합 — FES ES-vector(0x6300, 6B, burst_ms=500) write 가 6바이트 모두 반영 + on_write 콜백 + 구형(len16) 거부 회귀 가드 |
| T9 | read 경로 0x40 → ProcessRequest → 0x41 응답(data_len=6) |
| T10 | 짧은 프레임(<4, 0x21<6) / NULL 가드 |

> 빌드 산출물(`test_sdo*.exe`)은 `.gitignore` 의 `*.exe` 로 비추적.
> 참조: `docs/plan_sdo_explicit_length.md` §8.
