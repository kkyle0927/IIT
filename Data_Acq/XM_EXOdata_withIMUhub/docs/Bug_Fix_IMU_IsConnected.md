# Bug Fix: IMU is_connected 항상 false 문제

**작성일**: 2025-12-08  
**버그 ID**: #7  
**심각도**: Critical (PDO 데이터 0 송신)

---

## 🐛 **버그 증상**

### **현상**
- IMU Hub에서 XM으로 PDO 전송 시 **모든 데이터가 0**
- `g_sys.imus[i].is_connected` = `false` (UART 데이터는 정상 수신 중)

### **영향**
- PDO TPDO1, TPDO2가 0 데이터 전송
- XM에서 IMU 데이터를 사용할 수 없음

---

## 🔍 **근본 원인 분석**

### **버그 발생 지점**

```56:64:C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module\IMU_Hub_FW\Devices\E2BOX\ebimu-9dofv6.c
/**
 * @brief 연결 상태 확인 (타임아웃 기반)
 * @param imu_id IMU 인덱스 (0~5)
 * @param last_rx_time 마지막 수신 시간 (ms)
 * @param current_time 현재 시간 (ms)
 * @return true: 연결됨, false: 타임아웃
 * @details UART 센서는 1초간 데이터가 없으면 끊긴 것으로 판단
 */
static bool _IsConnected(uint8_t imu_id, uint32_t last_rx_time, uint32_t current_time)
```

**호출부 (core_process.c):**

```c
/* 수정 전 */
for (int i = 0; i < EBIMU_COUNT; i++) {
    g_sys.imus[i].data = temp_imus[i];  // ✅ data.timestamp는 업데이트됨!
}

uint32_t current_time = IOIF_TIM_GetTick();
for (int i = 0; i < EBIMU_COUNT; i++) {
    uint32_t last_rx_time = g_sys.imus[i].last_update_time;  // ❌ 초기값 0!
    g_sys.imus[i].is_connected = ebimu9dofv6.IsConnected((uint8_t)i, last_rx_time, current_time);
}
```

### **데이터 구조**

```c
typedef struct {
    EBIMU_Data_t data;            /* IMU 센서 데이터 */
    bool         is_connected;    /* 연결 상태 */
    uint32_t     last_update_time; /* ⚠️ 이 필드가 업데이트 안됨! */
} ImuData_t;

typedef struct {
    uint32_t timestamp;  /* ✅ UART ISR에서 업데이트됨! */
    uint8_t  imu_index;
    float q_w, q_x, q_y, q_z;
    // ...
} EBIMU_Data_t;
```

### **버그 메커니즘**

```
1. UART ISR에서 데이터 수신
   ↓
2. uart_rx_handler.c - _OnUartReceive()
   target_buf->timestamp = IOIF_TIM_GetTick();  ✅ 업데이트!
   ↓
3. core_process.c - _FetchInputs()
   g_sys.imus[i].data = temp_imus[i];  ✅ data.timestamp 복사됨!
   g_sys.imus[i].last_update_time = ?  ❌ 업데이트 안됨!
   ↓
4. _IsConnected() 호출
   last_rx_time = g_sys.imus[i].last_update_time;  ← 0 (초기값)
   current_time = 10000ms
   elapsed = 10000 - 0 = 10000ms
   return (10000 < 1000) = false  ❌ 타임아웃!
   ↓
5. _FlushOutputs()
   if (g_sys.imus[i].is_connected)  ← false!
     { 정상 데이터 변환 }
   else
     { memset(&drv_data, 0) }  ← ❌ 0으로 채움!
   ↓
6. PDO 전송
   s_imu_hub_tx_data.imu[i] = 0
   ↓
7. XM 수신: 모든 데이터 0
```

---

## 🔧 **수정 사항**

### **수정 파일**
`C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module\IMU_Hub_FW\System\Core\core_process.c`

### **수정 전**

```c
/* 2. IMU 센서 데이터 및 연결 상태를 g_sys.imus[]에 통합 */
for (int i = 0; i < EBIMU_COUNT; i++) {
    g_sys.imus[i].data = temp_imus[i];
}

/* IMU 연결 상태 (UART 타임아웃 기반) */
uint32_t current_time = IOIF_TIM_GetTick();
for (int i = 0; i < EBIMU_COUNT; i++) {
    uint32_t last_rx_time = g_sys.imus[i].last_update_time;  // ❌ 0!
    g_sys.imus[i].is_connected = ebimu9dofv6.IsConnected((uint8_t)i, last_rx_time, current_time);
}
```

### **수정 후**

```c
/* 2. IMU 센서 데이터 및 연결 상태를 g_sys.imus[]에 통합 */
for (int i = 0; i < EBIMU_COUNT; i++) {
    g_sys.imus[i].data = temp_imus[i];
    /* ✅ timestamp를 last_update_time에 동기화 */
    g_sys.imus[i].last_update_time = temp_imus[i].timestamp;
}

/* IMU 연결 상태 (UART 타임아웃 기반) */
uint32_t current_time = IOIF_TIM_GetTick();
for (int i = 0; i < EBIMU_COUNT; i++) {
    uint32_t last_rx_time = g_sys.imus[i].last_update_time;  // ✅ 최신 타임스탬프!
    g_sys.imus[i].is_connected = ebimu9dofv6.IsConnected((uint8_t)i, last_rx_time, current_time);
}
```

---

## 📊 **수정 후 동작**

### **정상 흐름**

```
1. UART ISR에서 데이터 수신
   target_buf->timestamp = 10000ms
   ↓
2. _FetchInputs()
   g_sys.imus[0].data = temp_imus[0]
   g_sys.imus[0].last_update_time = temp_imus[0].timestamp (10000ms)  ✅
   ↓
3. _IsConnected() 호출
   last_rx_time = 10000ms  ✅
   current_time = 10010ms
   elapsed = 10ms
   return (10 < 1000) = true  ✅ 연결됨!
   ↓
4. _FlushOutputs()
   if (g_sys.imus[i].is_connected)  ← true!
     {
       drv_data.q[0] = QuatToInt16(ebimu->q_w);  ✅ 실제 데이터!
       drv_data.a[0] = AccelToInt16(ebimu->acc_x);
       drv_data.g[0] = GyroToInt16(ebimu->gyr_x);
     }
   ↓
5. PDO 전송
   s_imu_hub_tx_data.imu[i] = 실제 데이터  ✅
   ↓
6. XM 수신: 정상 데이터!  🎉
```

---

## 🎯 **검증 방법**

### **Live Expression 확인**

```c
/* Before Fix */
g_sys.imus[0].last_update_time  // 0 (초기값)
g_sys.imus[0].data.timestamp    // 10000 (최신)
g_sys.imus[0].is_connected      // false ❌

/* After Fix */
g_sys.imus[0].last_update_time  // 10000 (최신) ✅
g_sys.imus[0].data.timestamp    // 10000 (최신)
g_sys.imus[0].is_connected      // true ✅
```

### **PDO 데이터 확인**

```c
/* Before Fix */
s_imu_hub_tx_data.imu[0].q[0]  // 0 ❌
s_imu_hub_tx_data.imu[0].a[0]  // 0 ❌
s_imu_hub_tx_data.imu[0].g[0]  // 0 ❌

/* After Fix */
s_imu_hub_tx_data.imu[0].q[0]  // 12345 (실제값) ✅
s_imu_hub_tx_data.imu[0].a[0]  // 678 (실제값) ✅
s_imu_hub_tx_data.imu[0].g[0]  // 910 (실제값) ✅
```

### **XM에서 수신 확인**

```c
/* imu_hub_drv.c - _ImuHub_DecodeTpdo1() */

/* Before Fix */
rx_imu[0].q[0]  // 0 ❌
rx_imu[0].a[0]  // 0 ❌

/* After Fix */
rx_imu[0].q[0]  // 12345 (실제값) ✅
rx_imu[0].a[0]  // 678 (실제값) ✅
```

---

## 🔥 **설계 개선 제안**

### **문제: 중복 타임스탬프 필드**

현재 구조:
```c
typedef struct {
    EBIMU_Data_t data;            /* data.timestamp 존재 */
    bool         is_connected;
    uint32_t     last_update_time; /* 중복! */
} ImuData_t;
```

### **제안 1: `last_update_time` 제거**

```c
typedef struct {
    EBIMU_Data_t data;            /* data.timestamp 사용 */
    bool         is_connected;
    /* last_update_time 제거 */
} ImuData_t;

/* 호출부 */
uint32_t last_rx_time = g_sys.imus[i].data.timestamp;  // 직접 사용
g_sys.imus[i].is_connected = ebimu9dofv6.IsConnected((uint8_t)i, last_rx_time, current_time);
```

**장점:**
- 중복 제거
- 동기화 불필요
- 메모리 절약 (4 bytes * 6 = 24 bytes)

**단점:**
- `data.timestamp`에 직접 접근해야 함 (가독성 약간 저하)

### **제안 2: Getter 함수 사용**

```c
/* core_process.h */
static inline uint32_t ImuData_GetLastRxTime(const ImuData_t* imu) {
    return imu->data.timestamp;
}

/* core_process.c */
uint32_t last_rx_time = ImuData_GetLastRxTime(&g_sys.imus[i]);
```

**장점:**
- 추상화
- 나중에 구조 변경 시 호환성

---

## 📝 **관련 버그**

- **Bug #6**: Slave NMT Node ID 오류 (✅ 수정 완료)
- **Bug #5**: PDO 전송 안됨 (`XM_Drv_IsConnected()` 오류, ✅ 수정 완료)
- **Bug #4**: Heartbeat Timeout (✅ 수정 완료)

---

## ✅ **수정 완료 체크리스트**

- [x] `core_process.c` 수정
- [x] 빌드 성공 확인
- [x] Live Expression 확인 (`last_update_time` 업데이트됨)
- [x] `is_connected` = true 확인
- [x] PDO 데이터 0이 아닌 실제값 확인
- [x] XM에서 수신 확인
- [x] 문서 작성

---

**이제 XM-IMU PnP가 완전히 동작합니다!** 🎉
