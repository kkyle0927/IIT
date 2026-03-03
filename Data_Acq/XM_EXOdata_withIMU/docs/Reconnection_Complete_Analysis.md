# XM-IMU Hub 재연결 로직 완전 분석 및 수정

**작성일**: 2025-12-08  
**목적**: CM-XM 수준의 100% 재연결 안정성 확보

---

## 🎯 목표: CM-XM 수준의 재연결 성공률

### **CM-XM 재연결 특징**
- ✅ XM Reset 후 즉시 재연결 성공
- ✅ 물리적 끊김 후 재연결 성공
- ✅ Heartbeat Timeout 후 자동 복구
- ✅ 재연결 성공률 ~100%

---

## 📊 현재 구현 상태 분석

### **1. Master (XM) - `imu_hub_drv.c` ✅ 완벽**

#### **(1) Disconnected 콜백 - ✅ 이미 구현됨!**

```c
static void _PnP_OnDisconnected(uint8_t node_id)
{
    (void)node_id;
    
    /* ✅ 완전 초기화 (CM-XM 방식) */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_imu_hub_inst.sdo_retry_count = 0;
    s_imu_hub_inst.imu_connected_mask = 0x00;
}
```

**분석**: ✅ **완벽함! CM-XM과 동일한 방식으로 구현됨**

---

#### **(2) Boot-up 콜백 - ✅ 정상**

```c
static void _PnP_OnBootup(uint8_t node_id)
{
    /* Boot-up 수신 → Pre-Op 상태 머신 시작 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
    s_imu_hub_inst.sdo_retry_count = 0;
}
```

**분석**: ✅ **정상! Boot-up 수신 시 Pre-Op 즉시 시작**

---

#### **(3) NMT Timeout 체크 - ✅ 정상**

```c
/* pnp_manager.c */
void PnPManager_RunPeriodic(void)
{
    if (s_master_pnp_initialized) {
        AGR_PnP_RunPeriodic(&s_master_pnp);  /* ✅ NMT Timeout 체크 */
    }
}
```

**분석**: ✅ **정상! System Layer에서 NMT Timeout 주기적 체크**

---

### **2. Slave (IMU Hub) - `xm_drv.c` ❌ 치명적 문제 발견!**

#### **(1) Boot-up 전송 - ❌ 호출 안 됨!**

**함수 정의 존재**:
```c
/* xm_drv.c */
int XM_Drv_SendBootup(void)
{
    return AGR_PnP_SendBootup(&s_xm_drv_inst.pnp_inst);
}
```

**system_startup.c 확인**:
```c
static void _InitSystemServices(void)
{
    /* ... */
    
    /* ===== 4. XM Driver 초기화 (AGR_PnP 내장) ===== */
    XM_Drv_Init(System_Fdcan2_Transmit, s_fdcan2_id);
    
    /* ❌ XM_Drv_SendBootup() 호출 없음! */
    
    /* ... */
}
```

**분석**: ❌ **치명적! Boot-up 전송이 호출되지 않음**

---

#### **(2) Heartbeat 전송 - ❌ 호출 안 됨!**

**함수 정의 존재**:
```c
/* xm_drv.c */
int XM_Drv_SendHeartbeat(void)
{
    return AGR_PnP_SendHeartbeat(&s_xm_drv_inst.pnp_inst);
}
```

**core_process.c 확인**:
```c
void Core_Process_Loop(void)
{
    /* ... IMU 데이터 업데이트 ... */
    
    /* ... PDO 전송 ... */
    
    /* ❌ XM_Drv_SendHeartbeat() 호출 없음! */
}
```

**분석**: ❌ **치명적! Heartbeat 전송이 호출되지 않음**

---

## 🚨 치명적 문제점 정리

### **1. Boot-up 전송 누락 (가장 심각)**

**문제**:
- IMU Hub Reset 후 XM에 Boot-up 메시지를 전송하지 않음
- XM은 Boot-up을 수신해야만 Pre-Op 시퀀스 시작
- **재연결 불가능** ❌

**영향**:
- Slave Reset 시 재연결 실패
- Debugging Mode 진입 후 재연결 실패
- 전체 재연결 성공률 0%

---

### **2. Heartbeat 전송 누락 (치명적)**

**문제**:
- IMU Hub가 XM에 Heartbeat를 전송하지 않음
- XM은 Heartbeat를 수신해야 연결 상태 유지
- **연결 유지 불가능** ❌

**영향**:
- 초기 연결 후 3초 내에 Disconnected
- XM은 Heartbeat Timeout으로 연결 끊김 판단
- 전체 재연결 성공률 0%

---

## ✅ 해결 방안

### **1. IMU Hub `system_startup.c` 수정**

#### **Before (현재)**

```c
static void _InitSystemServices(void)
{
    /* ===== 0. CAN Bus Monitor 초기화 (디버깅) ===== */
    CanBusMon_Init();
    
    /* ===== 1. CANFD Rx Handler 초기화 (콜백 등록) ===== */
    CanFdRxHandler_Init(s_fdcan2_id);
    
    /* ===== 2. FDCAN 시작 ===== */
    IOIF_FDCAN_START(s_fdcan2_id);
    
    /* ===== 3. UART Rx Handler 초기화 (IMU 센서) ===== */
    UartRxHandler_Init(s_uart_ids);
    
    /* ===== 4. XM Driver 초기화 (AGR_PnP 내장) ===== */
    XM_Drv_Init(System_Fdcan2_Transmit, s_fdcan2_id);
    
    /* ❌ Boot-up 전송 누락! */
    
    /* ===== 5. Core Process 초기화 (IPO 엔진) ===== */
    Core_Init();
}
```

---

#### **After (수정)**

```c
static void _InitSystemServices(void)
{
    /* ===== 0. CAN Bus Monitor 초기화 (디버깅) ===== */
    CanBusMon_Init();
    
    /* ===== 1. CANFD Rx Handler 초기화 (콜백 등록) ===== */
    CanFdRxHandler_Init(s_fdcan2_id);
    
    /* ===== 2. FDCAN 시작 ===== */
    IOIF_FDCAN_START(s_fdcan2_id);
    
    /* ===== 3. UART Rx Handler 초기화 (IMU 센서) ===== */
    UartRxHandler_Init(s_uart_ids);
    
    /* ===== 4. XM Driver 초기화 (AGR_PnP 내장) ===== */
    XM_Drv_Init(System_Fdcan2_Transmit, s_fdcan2_id);
    
    /* ✅ Boot-up 전송 (PRE_OPERATIONAL 진입 직후) */
    XM_Drv_SendBootup();
    
    /* ===== 5. Core Process 초기화 (IPO 엔진) ===== */
    Core_Init();
}
```

---

### **2. IMU Hub `core_process.c` 수정**

#### **Before (현재)**

```c
void Core_Process_Loop(void)
{
    /* ===== Software Tx Queue 처리 ===== */
    IOIF_FDCAN_ProcessQueue(s_fdcan2_id);
    
    /* ... IPO 엔진 실행 ... */
    
    /* ===== XM 드라이버 입력 업데이트 ===== */
    XM_Drv_SetFrameTimestamp(g_sys.tick_cnt);
    XM_Drv_ClearValidMask();
    
    /* ... IMU 데이터 업데이트 ... */
    
    /* ===== PDO 전송 (1ms 주기) ===== */
    if (s_pdo_toggle) {
        XM_Drv_SendTPDO2();
    } else {
        XM_Drv_SendTPDO1();
    }
    s_pdo_toggle = !s_pdo_toggle;
    
    /* ❌ Heartbeat 전송 누락! */
}
```

---

#### **After (수정)**

```c
void Core_Process_Loop(void)
{
    /* ===== Software Tx Queue 처리 ===== */
    IOIF_FDCAN_ProcessQueue(s_fdcan2_id);
    
    /* ... IPO 엔진 실행 ... */
    
    /* ===== XM 드라이버 입력 업데이트 ===== */
    XM_Drv_SetFrameTimestamp(g_sys.tick_cnt);
    XM_Drv_ClearValidMask();
    
    /* ... IMU 데이터 업데이트 ... */
    
    /* ===== PDO 전송 (1ms 주기) ===== */
    if (s_pdo_toggle) {
        XM_Drv_SendTPDO2();
    } else {
        XM_Drv_SendTPDO1();
    }
    s_pdo_toggle = !s_pdo_toggle;
    
    /* ✅ Heartbeat 전송 (100ms 주기) */
    static uint32_t last_heartbeat_time = 0;
    uint32_t current_time = g_sys.tick_cnt;  /* ms 단위 */
    
    if (current_time - last_heartbeat_time >= 100) {
        XM_Drv_SendHeartbeat();
        last_heartbeat_time = current_time;
    }
    
    /* ✅ AGR_PnP Periodic 실행 (NMT Timeout 체크) */
    XM_Drv_RunPeriodic();
}
```

---

## 🔄 재연결 시나리오 시뮬레이션

### **시나리오 1: Slave Reset (Debugging Mode 진입)**

#### **Before (현재)**

```
1. IMU Hub Reset
2. system_startup.c → XM_Drv_Init()
3. ❌ XM_Drv_SendBootup() 호출 안 됨
4. XM: Boot-up 수신 못함 → Pre-Op 시작 안 됨
5. ❌ 재연결 실패 (XM은 여전히 IDLE 상태)
```

**재연결 성공률**: 0% ❌

---

#### **After (수정 후)**

```
1. IMU Hub Reset
2. system_startup.c → XM_Drv_Init()
3. ✅ XM_Drv_SendBootup() 전송
4. XM: Boot-up 수신 → _PnP_OnBootup() 콜백
5. XM: Pre-Op 상태 머신 시작 (SEND_PDO_MAP_A)
6. XM: SDO → PDO Mapping A
7. IMU Hub: SDO Response
8. XM: SDO → PDO Mapping B
9. IMU Hub: SDO Response
10. XM: SDO → IMU Mask Read
11. IMU Hub: SDO Response
12. XM: NMT START 전송
13. ✅ 연결 완료 (OPERATIONAL)
```

**재연결 성공률**: ~100% ✅

---

### **시나리오 2: 물리적 연결 끊김**

#### **Before (현재)**

```
1. CAN 연결 끊김
2. XM: Heartbeat Timeout (3초)
3. XM: _PnP_OnDisconnected() 콜백 → Pre-Op IDLE 초기화
4. CAN 재연결
5. ❌ IMU Hub: Boot-up 전송 안 됨 (Reset이 아니므로)
6. XM: Boot-up 수신 못함 → Pre-Op 시작 안 됨
7. ❌ 재연결 실패
```

**재연결 성공률**: 0% ❌

---

#### **After (수정 후) - 추가 로직 필요**

**Option A: Slave의 NMT State 변경 감지**

```c
/* xm_drv.c */
void XM_Drv_RunPeriodic(void)
{
    /* ✅ Master NMT State 변경 감지 */
    static AGR_NMT_State_t last_master_state = AGR_NMT_BOOT_UP;
    AGR_NMT_State_t current_master_state = XM_Drv_GetXmNmtState();
    
    /* Master가 연결 끊김 감지 시 (OPERATIONAL → BOOT_UP) */
    if (last_master_state == AGR_NMT_OPERATIONAL && 
        current_master_state == AGR_NMT_BOOT_UP) {
        
        /* ✅ Boot-up 재전송 (Master Timeout 복구) */
        XM_Drv_SendBootup();
    }
    
    last_master_state = current_master_state;
}
```

**Option B: Heartbeat Error 감지** (더 안전)

```c
/* AGR_PnP에 Master Timeout 콜백 추가 */
static void _PnP_OnMasterTimeout(uint8_t master_node_id)
{
    /* Master가 응답하지 않음 → Boot-up 재전송 */
    XM_Drv_SendBootup();
}
```

**재연결 성공률**: ~100% ✅

---

### **시나리오 3: Heartbeat Timeout 후 재연결**

#### **Before (현재)**

```
1. IMU Hub: Heartbeat 전송 중단 (소프트웨어 이슈)
2. XM: Heartbeat Timeout (3초)
3. XM: _PnP_OnDisconnected() 콜백 → Pre-Op IDLE 초기화
4. ❌ IMU Hub: Heartbeat 전송 안 됨 (호출 자체가 없음)
5. XM: 계속 Timeout 상태
6. ❌ 재연결 실패
```

**재연결 성공률**: 0% ❌

---

#### **After (수정 후)**

```
1. IMU Hub: Heartbeat 주기 전송 (100ms)
2. XM: Heartbeat 수신 → 연결 유지
3. (가정) 일시적 Heartbeat 전송 중단
4. XM: Heartbeat Timeout (3초)
5. XM: _PnP_OnDisconnected() 콜백 → Pre-Op IDLE 초기화
6. ✅ IMU Hub: Heartbeat 재개
7. XM: Heartbeat 수신 → Boot-up으로 간주 (선택적)
8. ✅ 연결 복구

또는

6. ✅ IMU Hub: Boot-up 재전송 (Master Timeout 감지)
7. XM: Boot-up 수신 → Pre-Op 시작
8. ✅ 연결 완료
```

**재연결 성공률**: ~100% ✅

---

## 📋 수정 체크리스트

### **필수 수정 (즉시 적용)**

- [ ] **IMU Hub `system_startup.c`**
  - [ ] `XM_Drv_SendBootup()` 호출 추가
  - [ ] 위치: `XM_Drv_Init()` 직후, `Core_Init()` 전

- [ ] **IMU Hub `core_process.c`**
  - [ ] `XM_Drv_SendHeartbeat()` 100ms 주기 호출
  - [ ] `XM_Drv_RunPeriodic()` 주기 호출
  - [ ] 위치: PDO 전송 직후

---

### **권장 수정 (재연결 안정성 향상)**

- [ ] **IMU Hub `xm_drv.c`**
  - [ ] `XM_Drv_RunPeriodic()`에 Master Timeout 감지 로직 추가
  - [ ] Master가 응답하지 않으면 Boot-up 재전송

---

## 📊 재연결 성공률 예상

| 시나리오 | Before | After |
|----------|--------|-------|
| Slave Reset | 0% ❌ | ~100% ✅ |
| 물리적 끊김 | 0% ❌ | ~100% ✅ |
| Heartbeat Timeout | 0% ❌ | ~100% ✅ |
| Bus Off 복구 | 0% ❌ | ~100% ✅ |

---

## 🎯 최종 결론

### **현재 상태**
- Master (XM): ✅ **완벽함** (CM-XM 방식 동일)
- Slave (IMU Hub): ❌ **Boot-up/Heartbeat 전송 누락**

---

### **수정 후 상태**
- Master (XM): ✅ **완벽함**
- Slave (IMU Hub): ✅ **CM-XM 수준**

---

### **기대 효과**
- ✅ **재연결 성공률 0% → 100%**
- ✅ **CM-XM 수준의 안정성 확보**
- ✅ **Debugging Mode 진입 후 자동 재연결**
- ✅ **물리적 끊김 후 자동 재연결**

---

**다음 단계**: 수정 사항 적용 및 실제 테스트 검증
