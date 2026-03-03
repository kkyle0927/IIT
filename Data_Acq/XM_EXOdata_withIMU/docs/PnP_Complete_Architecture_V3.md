# PnP 완전 아키텍처 가이드 V3

**작성일**: 2025-12-08  
**목적**: NMT / PnP / DOP / Device Driver 역할 완전 분리 및 명확화

---

## 🚨 **V2에서 V3으로 변경된 핵심 사항**

### **V2 (기존)**
- ❌ UART 센서도 PnP 사용
- ❌ SIMPLE Mode vs FULL Mode
- ❌ 콜백 체인 4단계 (NMT → PnP 내부 → PnP → Device)

### **V3 (현재)**
- ✅ **Protocol-based (CANopen)만 PnP 사용**
- ✅ **UART 센서는 PnP 없이 직접 처리**
- ✅ **Mode 구분 제거 (FULL만 사용)**
- ✅ **콜백 체인 간소화**

---

## 📊 **4개 레이어 역할 분리**

```
┌─────────────────────────────────────────────────┐
│  [4] Device Driver (imu_hub_drv.c, xm_drv.c)   │  ← 응용 로직
│  - Pre-Op 시퀀스 정의 (Step Array)              │
│  - PDO 데이터 변환 (float ↔ int16)              │
│  - 연결 상태 관리                                │
├─────────────────────────────────────────────────┤
│  [3] AGR_PnP (agr_pnp.c)                       │  ← PnP 추상화
│  - Pre-Op State Machine                        │
│  - Heartbeat Timeout 관리                       │
│  - Master ↔ Slave 연결 상태                    │
├─────────────────────────────────────────────────┤
│  [2] AGR_DOP (agr_dop.c)                       │  ← CANopen 데이터
│  - SDO Request/Response (0x600/0x580)          │
│  - PDO Mapping (0x1800/0x1A00)                 │
│  - Object Dictionary                            │
├─────────────────────────────────────────────────┤
│  [1] AGR_NMT (agr_nmt.c)                       │  ← CANopen NMT
│  - Boot-up (0x700, [0x00])                     │
│  - Heartbeat (0x700, [0x05/0x7F])              │
│  - NMT Command (0x000)                          │
│  - State Management (BOOT_UP/PRE_OP/OP/STOP)   │
└─────────────────────────────────────────────────┘
```

---

## 1️⃣ **AGR_NMT: CANopen NMT 프로토콜**

### **역할**
- CANopen NMT 프로토콜 처리
- 상태 관리 (BOOT_UP → PRE_OPERATIONAL → OPERATIONAL → STOPPED)
- Heartbeat Timeout 검사

### **제공 API**

| API | 역할 | 사용처 |
|-----|------|--------|
| `AGR_NMT_Init` | NMT 인스턴스 초기화 | Master/Slave 초기화 |
| `AGR_NMT_InitEx` | NMT + 콜백 초기화 | Master/Slave 초기화 |
| `AGR_NMT_ProcessMessage` | Boot-up/Heartbeat 처리 | CAN 수신 핸들러 |
| `AGR_NMT_UpdateActivity` | Activity 갱신 + 상태 전이 | ProcessMessage 내부 |
| `AGR_NMT_CheckTimeout` | Heartbeat Timeout 검사 | Periodic Task |
| `AGR_NMT_SetState` | 상태 강제 변경 | NMT Command 처리 |
| `AGR_NMT_GetState` | 현재 상태 조회 | Device Driver |

### **콜백**

```c
typedef void (*AGR_NMT_StateChangedCb_t)(AGR_NMT_State_t old_state, 
                                          AGR_NMT_State_t new_state, 
                                          void* user_ctx);

typedef void (*AGR_NMT_TimeoutCb_t)(void* user_ctx);
```

**호출 시점**:
- `on_state_changed`: NMT 상태 변경 시 (BOOT_UP → PRE_OP → OPERATIONAL)
- `on_timeout`: Heartbeat Timeout 시 (OPERATIONAL → STOPPED)

---

## 2️⃣ **AGR_DOP: CANopen SDO/PDO 프로토콜**

### **역할**
- SDO Request/Response 인코딩/디코딩
- PDO 인코딩/디코딩
- Object Dictionary 관리

### **제공 API**

| API | 역할 | 사용처 |
|-----|------|--------|
| `AGR_DOP_Init` | DOP 초기화 + OD 등록 | Master/Slave 초기화 |
| `AGR_DOP_ProcessRxMessage` | SDO/PDO 수신 처리 | CAN 수신 핸들러 |
| `AGR_DOP_EncodeSDO` | SDO 인코딩 | SDO Request 전송 |
| `AGR_DOP_DecodeSDO` | SDO 디코딩 | SDO Response 수신 |
| `AGR_DOP_SendSDOWrite` | SDO Write 전송 | Master Pre-Op |
| `AGR_DOP_SendSDORead` | SDO Read 전송 | Master Pre-Op |

### **콜백**

```c
typedef void (*AGR_DOP_OnSDORequest_t)(uint8_t node_id, 
                                        uint16_t index, 
                                        uint8_t subindex, 
                                        const uint8_t* data, 
                                        uint8_t len);

typedef void (*AGR_DOP_OnPDOReceived_t)(uint8_t pdo_num, 
                                         const uint8_t* data, 
                                         uint8_t len);
```

**호출 시점**:
- `on_sdo_request`: SDO Request 수신 시 (Slave만)
- `on_pdo_received`: PDO 수신 시 (Master/Slave 모두)

---

## 3️⃣ **AGR_PnP: PnP 추상화 레이어**

### **역할**
- Master: Pre-Op State Machine 관리
- Slave: Master 연결 상태 관리
- Heartbeat Timeout 자동 관리
- 연결/끊김 이벤트 추상화

### **제공 API**

| API | 역할 | Master | Slave |
|-----|------|:------:|:-----:|
| `AGR_PnP_Init` | PnP 초기화 | ✅ | ✅ |
| `AGR_PnP_RegisterDevice` | Slave Device 등록 | ✅ | ❌ |
| `AGR_PnP_RegisterMaster` | Master 등록 | ❌ | ✅ |
| `AGR_PnP_RunPeriodic` | Periodic 실행 | ✅ | ✅ |
| `AGR_PnP_SendBootup` | Boot-up 전송 | ❌ | ✅ |
| `AGR_PnP_SendHeartbeat` | Heartbeat 전송 | ✅ | ✅ |
| `AGR_PnP_SendNmtCommand` | NMT 명령 전송 | ✅ | ❌ |

### **콜백**

#### **Master 콜백**

```c
typedef struct {
    void (*on_bootup)(uint8_t node_id);
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
    void (*on_run_pre_op)(uint8_t node_id);  /* ✅ Pre-Op 단계 실행 */
    void (*on_connected)(uint8_t node_id);
    void (*on_disconnected)(uint8_t node_id);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_Callbacks_t;
```

**호출 시점**:
- `on_bootup`: Slave Boot-up 수신 시
- `on_nmt_change`: Slave NMT 상태 변경 시
- `on_run_pre_op`: Pre-Op 단계마다 주기적으로 (Step Array 실행)
- `on_connected`: Slave OPERATIONAL 진입 시
- `on_disconnected`: Slave STOPPED 진입 시
- `on_error`: Timeout, 에러 발생 시

#### **Slave 콜백**

```c
typedef struct {
    void (*on_connected)(uint8_t node_id);
    void (*on_disconnected)(uint8_t node_id);
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_Callbacks_t;
```

**호출 시점**:
- `on_connected`: Master Heartbeat 수신 시작
- `on_disconnected`: Master Heartbeat Timeout
- `on_nmt_change`: Master NMT 상태 변경 시 (선택 사항)
- `on_error`: Timeout, 에러 발생 시

---

## 4️⃣ **Device Driver: 응용 로직**

### **Master (imu_hub_drv.c)**

#### **역할**
- Pre-Op 시퀀스 정의 (Step Array)
- SDO Write 실행 (PDO Mapping, 설정값)
- PDO 수신 및 데이터 변환
- Slave 연결 상태 관리

#### **구현 함수**

| 함수 | 역할 |
|------|------|
| `ImuHub_Drv_Init` | Master PnP 등록 + 콜백 설정 |
| `ImuHub_Drv_ProcessCANMessage` | CAN 메시지 라우팅 (NMT/SDO/PDO) |
| `ImuHub_Drv_RunPeriodic` | Pre-Op Timeout 체크 |
| `_PnP_OnBootup` | Pre-Op 초기화 |
| `_PnP_OnNmtChange` | Pre-Op State Machine |
| `_PnP_RunPreOp` | Step Array 실행 (SDO 전송) |
| `_OnSdoResponse` | SDO Response 처리 (Pre-Op 전이) |
| `_ImuHub_DecodeTpdo1/2` | PDO 디코딩 + 데이터 저장 |

---

### **Slave (xm_drv.c)**

#### **역할**
- SDO Request 처리 (OD Read/Write)
- PDO 수신 및 데이터 변환
- PDO 전송 (데이터 인코딩)
- Master 연결 상태 관리

#### **구현 함수**

| 함수 | 역할 |
|------|------|
| `XM_Drv_Init` | Slave PnP 등록 + 콜백 설정 |
| `XM_Drv_ProcessCANMessage` | CAN 메시지 라우팅 (NMT/SDO/PDO) |
| `XM_Drv_SendTPDO1/2` | PDO 전송 (데이터 인코딩) |
| `_PnP_OnConnected` | 연결 상태 업데이트 |
| `_PnP_OnDisconnected` | 끊김 상태 업데이트 |
| `_OnSdoRequest` | SDO Request 처리 (OD 접근) |
| `_OnPdoReceived` | PDO 디코딩 + 데이터 저장 |

---

## 🔄 **PnP 전체 시퀀스 (Master ↔ Slave)**

### **1. Boot-up Phase**

```
[Slave - IMU Hub]
  └─ 1. XM_Drv_Init()
      └─ AGR_PnP_Init(SLAVE)
      └─ AGR_PnP_RegisterMaster(XM)
      └─ AGR_PnP_SendBootup()  /* 0x708, [0x00] */

[Master - XM]
  └─ 2. CAN RX: Boot-up (0x708, [0x00])
      └─ ImuHub_Drv_ProcessCANMessage()
          └─ AGR_NMT_ProcessMessage()
              └─ inst->state = BOOT_UP
              └─ AGR_NMT_UpdateActivity()
                  └─ inst->state = PRE_OPERATIONAL  /* ✅ FULL Mode만 사용 */
                  └─ _NotifyStateChange(BOOT_UP, PRE_OP)
                      ↓
          └─ _OnNmtStateChanged (agr_pnp.c 내부)
              └─ dev->callbacks.on_bootup(node_id)
                  ↓
          └─ _PnP_OnBootup (imu_hub_drv.c)
              └─ s_imu_hub_inst.pre_op_state = SEND_PDO_MAP_A ✅
```

---

### **2. Pre-Operational Phase (Master → Slave)**

```
[Master - XM]
  └─ 3. ImuHub_Drv_RunPeriodic()
      └─ _PnP_RunPreOp()  /* Step Array 실행 */
          ↓
  └─ 4. Step 1: SEND_PDO_MAP_A
      └─ AGR_DOP_SendSDOWrite(0x1800, 0x01, ...)  /* TPDO1 Mapping */
      └─ s_pre_op_state = WAIT_PDO_MAP_A

[Slave - IMU Hub]
  └─ 5. CAN RX: SDO Request (0x608, [0x2F, 0x00, 0x18, 0x01, ...])
      └─ XM_Drv_ProcessCANMessage()
          └─ AGR_DOP_ProcessRxMessage()
              └─ s_dop_ctx.on_sdo_request()
                  ↓
          └─ _OnSdoRequest (xm_drv.c)
              └─ AGR_DOP_WriteOD(0x1800, 0x01, ...)  /* OD 업데이트 */
              └─ AGR_DOP_SendSDOResponse(OK)  /* 0x588, [0x60, ...] */

[Master - XM]
  └─ 6. CAN RX: SDO Response (0x588, [0x60, ...])
      └─ ImuHub_Drv_ProcessCANMessage()
          └─ AGR_DOP_DecodeSDO(&sdo_msg)
              └─ _OnSdoResponse (imu_hub_drv.c)
                  └─ s_pre_op_state = SEND_PDO_MAP_B ✅

  └─ 7. Step 2~4: 동일한 방식으로 PDO Mapping B, IMU Mask, NMT Start
```

---

### **3. Operational Phase (Data Exchange)**

```
[Master - XM]
  └─ 8. Step 4 완료: NMT START
      └─ AGR_PnP_SendNmtCommand(NMT_CMD_START)  /* 0x000, [0x01, 0x08] */

[Slave - IMU Hub]
  └─ 9. CAN RX: NMT START (0x000, [0x01, 0x08])
      └─ XM_Drv_ProcessCANMessage()
          └─ AGR_NMT_ProcessMessage()
              └─ AGR_NMT_ProcessCommand(NMT_CMD_START)
                  └─ inst->state = OPERATIONAL ✅
                  └─ _NotifyStateChange(PRE_OP, OPERATIONAL)
                      ↓
          └─ _OnNmtStateChanged (agr_pnp.c 내부)
              └─ dev->callbacks.on_nmt_change(PRE_OP, OPERATIONAL)
                  ↓
          └─ _PnP_OnNmtChange (xm_drv.c)
              └─ (상태 업데이트)

  └─ 10. 매 1ms: PDO 전송
      └─ XM_Drv_SendTPDO1/2()
          └─ AGR_DOP_SendPDO(TPDO1/2, data, len)  /* 0x188/0x288 */

[Master - XM]
  └─ 11. CAN RX: PDO (0x188/0x288)
      └─ ImuHub_Drv_ProcessCANMessage()
          └─ _ImuHub_DecodeTpdo1/2()
              └─ s_imu_hub_inst.imu_data[] 업데이트 ✅
```

---

### **4. Heartbeat Phase (Connection Monitoring)**

```
[Master - XM]
  └─ 12. 매 1s: Heartbeat 전송
      └─ AGR_PnP_RunPeriodic()
          └─ AGR_PnP_SendHeartbeat()  /* 0x702, [0x05] */

[Slave - IMU Hub]
  └─ 13. CAN RX: Heartbeat (0x702, [0x05])
      └─ XM_Drv_ProcessCANMessage()
          └─ AGR_NMT_ProcessMessage()
              └─ AGR_NMT_UpdateActivity()  /* Activity 갱신 */

  └─ 14. Timeout 체크 (5s)
      └─ AGR_PnP_RunPeriodic()
          └─ AGR_NMT_CheckTimeout()
              └─ if (elapsed > 5000ms)
                  └─ inst->state = STOPPED ❌
                  └─ _PnP_OnDisconnected()
```

---

## 📋 **콜백 체인 간소화**

### **V2 (기존 - 4단계)**

```
[1] AGR_NMT 콜백
      ↓
[2] AGR_PnP 내부 핸들러 (_OnNmtStateChanged)
      ↓
[3] AGR_PnP 콜백 (AGR_PnP_Device_t.callbacks)
      ↓
[4] Device Driver 콜백 (_PnP_OnBootup, _PnP_OnNmtChange)
```

**문제**: 너무 복잡함!

---

### **V3 (개선 - 3단계)**

```
[1] AGR_NMT 콜백
      ↓
[2] AGR_PnP 내부 핸들러 (_OnNmtStateChanged)
      ↓
[3] Device Driver 콜백 (_PnP_OnBootup, _PnP_OnNmtChange)
```

**개선**: AGR_PnP_Device_t.callbacks를 Device Driver 콜백으로 직접 연결

---

## 🚀 **Legacy 제거 및 간소화**

### **1. AGR_NMT_Mode_t 제거**

**Before**:
```c
typedef enum {
    AGR_NMT_MODE_SIMPLE = 0,  /* ❌ UART 센서용 (제거) */
    AGR_NMT_MODE_FULL = 1,    /* ✅ CANopen PnP용 (유지) */
} AGR_NMT_Mode_t;
```

**After**:
```c
/* ✅ Mode 필드 제거 - CANopen만 사용 */
/* AGR_NMT_UpdateActivity에서 무조건 PRE_OPERATIONAL로 전이 */
```

---

### **2. UART 센서 (EBIMU) PnP 제거**

**Before**:
```c
/* ebimu-9dofv6.c */
AGR_PnP_Init(...);  /* ❌ PnP 사용 */
```

**After**:
```c
/* ebimu-9dofv6.c */
/* ✅ PnP 없이 직접 연결 상태 관리 (Timeout 기반) */
bool ebimu9dofv6.IsConnected(uint8_t sensor_id, uint32_t last_rx_time, uint32_t current_time);
```

---

### **3. 콜백 간소화**

**Before**:
```c
/* AGR_PnP_Device_t.callbacks */
typedef struct {
    void (*on_bootup)(uint8_t node_id);
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old, AGR_NMT_State_t new);
    void (*on_run_pre_op)(uint8_t node_id);
    void (*on_connected)(uint8_t node_id);
    void (*on_disconnected)(uint8_t node_id);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_Callbacks_t;
```

**After**:
```c
/* ✅ Master / Slave 역할별로 분리 */

/* Master 전용 콜백 */
typedef struct {
    void (*on_bootup)(uint8_t node_id);
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old, AGR_NMT_State_t new);
    void (*on_run_pre_op)(uint8_t node_id);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_MasterCallbacks_t;

/* Slave 전용 콜백 */
typedef struct {
    void (*on_connected)(uint8_t node_id);
    void (*on_disconnected)(uint8_t node_id);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_SlaveCallbacks_t;
```

---

## 📝 **수정 체크리스트**

| 항목 | 상태 | 비고 |
|------|------|------|
| `AGR_NMT_Mode_t` 제거 | ⬜ TODO | `agr_nmt.h`, `agr_nmt.c` |
| `AGR_NMT_UpdateActivity` 수정 | ⬜ TODO | Mode 체크 제거 |
| EBIMU PnP 제거 확인 | ✅ DONE | 이미 제거됨 |
| 콜백 구조 간소화 | ⬜ TODO | `agr_pnp.h` |
| Master/Slave 콜백 분리 | ⬜ TODO | `agr_pnp.h` |
| 문서 업데이트 | ✅ DONE | 이 문서 |

---

**결론**: 
1. ✅ UART 센서는 PnP 없이 직접 관리
2. ✅ Protocol-based (CANopen)만 PnP 사용
3. ⬜ TODO: `AGR_NMT_Mode_t` 제거
4. ⬜ TODO: 콜백 구조 간소화

이제 코드 수정을 진행할까요?
