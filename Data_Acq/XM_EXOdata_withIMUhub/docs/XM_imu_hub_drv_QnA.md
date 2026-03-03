# XM imu_hub_drv.c 상세 질문 답변

**작성일**: 2025-12-08  
**목적**: XM Master imu_hub_drv.c 코드 리뷰 및 설계 검증

---

## 1. ImuHub_Drv_Init 초기화 위치 ❌

### **문제: 초기화 호출이 누락됨!**

#### **현재 상황**
```c
/* pnp_manager.c:87 */
// ImuHub_Drv_Init()는 system_startup.c에서 이미 호출됨  ← 주석만 있음!
```

#### **확인 결과**
- ❌ `system_startup.c`에서 `ImuHub_Drv_Init()` 호출 **없음**
- ❌ `pnp_manager.c`에서 주석 처리됨
- ❌ **초기화가 전혀 안 되고 있음!**

### **✅ 해결: system_startup.c에 추가 필요**

```c
/* system_startup.c */

#include "imu_hub_drv.h"  /* ✅ 추가 */
#include "ioif_agrb_fdcan.h"

void System_Startup(void)
{
    /* ===== 1. IOIF 초기화 ===== */
    IOIF_TIM_Init();
    IOIF_AGRB_FDCAN_Init();
    
    /* ===== 2. Device Layer 초기화 ===== */
    
    /* ✅ IMU Hub Device Driver 초기화 (Master) */
    ImuHub_Drv_Init(IOIF_AGRB_FDCAN_Transmit, NULL);
    
    /* ===== 3. System Layer 초기화 ===== */
    PnPManager_Init();  /* PnP Manager (Legacy 모듈 관리) */
    
    /* ===== 4. Application Layer 초기화 ===== */
    Core_Process_Init();
}
```

**중요**: ✅ **PnPManager_Init() 전에 ImuHub_Drv_Init() 호출 필요!**

---

## 2. Step Array 리팩토링 ❌

### **현재 상태: Step Array 미구현**

#### **원래 계획 (docs/XM_Step_Array_Refactoring_Plan.md)**
```c
/* Step Array 패턴 (선언적 Pre-Op 시퀀스) */
typedef struct {
    ImuHub_PreOpState_t state;
    ImuHub_PreOpAction_t action;
    uint32_t timeout_ms;
} ImuHub_PreOpStep_t;

static const ImuHub_PreOpStep_t s_pre_op_steps[] = {
    { IMUHUB_PRE_OP_SEND_PDO_MAP_A,  _SendPdoMapA,  1000 },
    { IMUHUB_PRE_OP_SEND_PDO_MAP_B,  _SendPdoMapB,  1000 },
    { IMUHUB_PRE_OP_SEND_IMU_MASK,   _SendImuMask,  1000 },
    { IMUHUB_PRE_OP_SEND_NMT_START,  _SendNmtStart, 1000 },
};
```

#### **현재 구현 (imu_hub_drv.c:524~598)**
```c
/* 하드코딩된 switch-case 방식 (from imu_hub_xm_link_v2.c) */
static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    switch (s_imu_hub_inst.pre_op_state) {
        case IMUHUB_PRE_OP_SEND_PDO_MAP_A:
            /* ... 하드코딩 ... */
            break;
        case IMUHUB_PRE_OP_SEND_PDO_MAP_B:
            /* ... 하드코딩 ... */
            break;
        /* ... */
    }
}
```

### **✅ 판단: Step Array 불필요 (현재 구조 유지 권장)**

**이유**:
1. ✅ **검증된 로직 보존**: `imu_hub_xm_link_v2.c`의 동작 확인된 코드
2. ✅ **가독성**: 현재 switch-case가 직관적
3. ✅ **복잡도**: XM-IMU 연결은 단일 시퀀스 (다양한 Device 없음)
4. ⚠️ **Step Array 이점 미미**: 재사용 필요 없음 (IMU Hub 전용)

**Step Array가 필요한 경우**:
- 다양한 Device (EMG Hub, FES Hub, GRF Hub 등) 추가 시
- 각 Device별로 다른 Pre-Op 시퀀스가 필요할 때

**결론**: ⚠️ **현재는 불필요, 향후 확장 시 고려**

---

## 3. AGR_NMT_InitEx vs 다른 함수 ✅

### **현재 사용 (imu_hub_drv.c:284~289)**
```c
/* Master (XM10) */
AGR_NMT_InitEx(&s_imu_hub_inst.nmt,
               3000,                        /* Heartbeat Timeout: 3s */
               AGR_NODE_ID_IMU_HUB,         /* IMU Hub Node ID (0x08) - Slave 모니터링 */
               NULL,                        /* ✅ 상태 변경 콜백 (AGR_PnP가 처리) */
               NULL,                        /* ✅ Timeout 콜백 (AGR_PnP가 처리) */
               NULL);                       /* User Context */
```

### **Slave (IMU Hub Module) 사용**
```c
/* Slave (IMU Hub Module) - xm_drv.c */
AGR_NMT_Init(&s_nmt, 5000);  /* 간단 초기화 (Timeout만) */
```

### **AGR_NMT 초기화 함수 비교**

| 함수 | 파라미터 | 용도 | Master | Slave |
|------|----------|------|--------|-------|
| **AGR_NMT_Init** | timeout_ms | 기본 초기화 | ⚠️ | ✅ 권장 |
| **AGR_NMT_InitEx** | + node_id, callbacks | 콜백 필요 시 | ✅ 권장 | ✅ |
| **AGR_NMT_InitEx2** | + mode (FULL/SIMPLE) | 동작 모드 선택 | ✅ | ⚠️ |

### **✅ 판단: AGR_NMT_InitEx 적절함**

**이유**:
1. ✅ **Master는 Slave 모니터링**: Node ID 필요 (0x08)
2. ✅ **콜백 NULL 허용**: AGR_PnP가 상태 변경/Timeout 처리
3. ✅ **AGR_NMT_InitEx2도 가능**: Mode는 기본값(FULL) 사용 중

**Slave (IMU Hub) 권장**:
```c
/* Slave는 간단 초기화로 충분 */
AGR_NMT_Init(&s_nmt, 5000);  /* ✅ 현재 사용 중 (적절) */
```

**Master (XM) 현재 상태**:
```c
/* ✅ 적절: Node ID 지정, 콜백은 AGR_PnP가 처리 */
AGR_NMT_InitEx(&s_imu_hub_inst.nmt, 3000, AGR_NODE_ID_IMU_HUB, NULL, NULL, NULL);
```

**결론**: ✅ **현재 사용 방식 적절함** (Master는 InitEx, Slave는 Init)

---

## 4. tx_func 중복 저장 ✅

### **현재 구조 (imu_hub_drv.c)**
```c
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    /* ===== 1. s_imu_hub_inst에 저장 ===== */
    s_imu_hub_inst.tx_func = tx_func;  /* Line 269 */
    
    /* ===== 2. AGR_DOP에 전달 ===== */
    AGR_DOP_Init(&s_imu_hub_inst.dop_ctx,
                 NULL,
                 AGR_NODE_ID_XM,
                 tx_func);  /* Line 295 - DOP 내부에 저장 */
    
    /* ===== 3. AGR_PnP에 전달 ===== */
    AGR_PnP_Config_t pnp_config = {
        .role = AGR_PNP_ROLE_MASTER,
        .my_node_id = AGR_NODE_ID_XM,
        .tx_func = tx_func,  /* Line 301 - PnP 내부에 저장 */
        /* ... */
    };
}
```

### **✅ 판단: 적절함 (각 모듈이 독립적으로 Tx 필요)**

#### **저장 이유**
```
┌────────────────────────────────────────────────────┐
│ s_imu_hub_inst.tx_func                             │
│ - 용도: PDO 전송 시 직접 사용 (Legacy)              │
│ - 예: _ImuHub_SendSdo() (현재 미사용)               │
└────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────┐
│ AGR_DOP 내부 tx_func                                │
│ - 용도: SDO Response 전송 (Master는 사용 안 함)      │
│ - Slave에서만 사용: SDO Request 처리 후 Response 전송│
└────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────┐
│ AGR_PnP 내부 tx_func                                │
│ - 용도: NMT Command, SDO Request, Heartbeat 전송    │
│ - AGR_PnP_SendNmtCommand()                         │
│ - AGR_PnP_SendSDOWrite()                           │
│ - AGR_PnP_SendSDORead()                            │
│ - AGR_PnP_SendHeartbeat()                          │
└────────────────────────────────────────────────────┘
```

#### **실제 사용 흐름**
```c
/* Master (XM10) Pre-Op 시퀀스 */

_PnP_RunPreOp() {
    /* AGR_PnP의 tx_func 사용 */
    AGR_PnP_SendSDOWrite(inst, node_id, IMUHUB_OD_IDX_PDO_MAPPING_A, ...);
    ↓
    AGR_PnP 내부: s_inst->tx_func(can_id, data, len);
}
```

### **✅ 결론: 중복 아님, 각 모듈의 독립성**

**설계 원칙**:
- ✅ **모듈 독립성**: 각 모듈이 자체 Tx 기능 보유
- ✅ **재사용성**: AGR_DOP, AGR_PnP는 다른 프로젝트에서도 사용 가능
- ✅ **의존성 최소화**: Device Layer가 Services Layer에 Tx 함수 제공

**개선 여지**:
- ⚠️ `s_imu_hub_inst.tx_func`는 **현재 미사용** (PDO는 AGR_PnP 사용)
- ⚠️ 향후 Legacy 함수 제거 시 삭제 가능

---

## 5. _ImuHub_SendSdo 함수 용도 ❌

### **함수 정의 (imu_hub_drv.c:744~758)**
```c
static void _ImuHub_SendSdo(const AGR_SDO_Msg_t* sdo_msg)
{
    if (sdo_msg == NULL || s_imu_hub_inst.tx_func == NULL) {
        return;
    }
    
    uint8_t sdo_buf[64];
    int len = AGR_DOP_EncodeSDO(sdo_msg, sdo_buf);
    
    if (len > 0) {
        /* [DOP V2] XM10 → IMU Hub SDO Request (Node ID만 사용) */
        uint32_t can_id = AGR_DOP_GetSDORequestCANID(AGR_NODE_ID_IMU_HUB);  /* 0x608 */
        s_imu_hub_inst.tx_func(can_id, sdo_buf, (uint8_t)len);
    }
}
```

### **✅ 판단: Legacy 함수 (사용되지 않음, 삭제 권장)**

#### **검색 결과**
```bash
# imu_hub_drv.c 전체 검색
grep "_ImuHub_SendSdo" imu_hub_drv.c
→ 결과: 정의만 있고 호출 없음 ❌
```

#### **현재 SDO 전송 방식**
```c
/* ✅ 현재 사용: AGR_PnP API */
_PnP_RunPreOp() {
    /* AGR_PnP가 SDO 인코딩 및 전송 처리 */
    AGR_PnP_SendSDOWrite(inst, node_id, index, subindex, data, len);
    AGR_PnP_SendSDORead(inst, node_id, index, subindex);
}
```

#### **Legacy 함수 흔적**
```c
/* imu_hub_xm_link_v2.c (V2) */
_ImuHub_SendSdoRequest() {
    /* 직접 SDO 인코딩 및 전송 */
    AGR_DOP_EncodeSDO(...);
    tx_func(can_id, buf, len);
}

/* ↓ 리팩토링 후 (V3) */
/* AGR_PnP API로 대체 */
AGR_PnP_SendSDOWrite(...);  /* ✅ 추상화 */
```

### **✅ 결론: 삭제 권장**

```diff
/* imu_hub_drv.c */

- /**
-  * @brief SDO 전송 (Legacy)
-  */
- static void _ImuHub_SendSdo(const AGR_SDO_Msg_t* sdo_msg)
- {
-     /* ... */
- }
```

**삭제 이유**:
- ❌ **호출 없음**: 현재 코드에서 사용되지 않음
- ✅ **AGR_PnP 대체**: `AGR_PnP_SendSDOWrite/Read` 사용
- ✅ **코드 정리**: Dead Code 제거

---

## 6. 재연결 로직 ✅

### **재연결 시나리오**
```
[정상 동작]
XM ↔ IMU Hub: OPERATIONAL (Heartbeat 주기 전송)
    ↓
[연결 끊김]
IMU Hub: 전원 차단 또는 케이블 분리
    ↓
XM: Heartbeat 미수신 (3초 Timeout)
    ↓
AGR_NMT: STOPPED
    ↓
AGR_PnP: _PnP_OnDisconnected() 호출
    ↓
[재연결]
IMU Hub: 부팅 완료 → Boot-up (0x708, 0x00) 전송
    ↓
XM: Boot-up 수신
    ↓
AGR_NMT: BOOT_UP → PRE_OPERATIONAL
    ↓
AGR_PnP: _PnP_OnBootup() 호출
    ↓
Pre-Op 시퀀스 재시작 (PDO Mapping → NMT START)
    ↓
OPERATIONAL 복귀 ✅
```

### **코드 흐름**

#### **1. 연결 끊김 감지**
```c
/* ImuHub_Drv_RunPeriodic() - 10ms 주기 호출 */
void ImuHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ✅ 1. Heartbeat Timeout 체크 (3초) */
    AGR_NMT_CheckTimeout(&s_imu_hub_inst.nmt, current_ms);
    ↓
    /* AGR_NMT 내부: 3초 동안 UpdateActivity 없음 */
    if (current_ms - inst->last_activity_ms > inst->timeout_ms) {
        AGR_NMT_SetState(inst, AGR_NMT_STOPPED);  /* ✅ STOPPED 전환 */
        ↓
        inst->on_timeout(inst->node_id);  /* Timeout 콜백 (NULL이면 미호출) */
    }
    
    /* ✅ 2. AGR_PnP 주기 처리 */
    AGR_PnP_RunPeriodic(&s_imu_hub_inst.pnp_inst);
    ↓
    /* AGR_PnP 내부: NMT 상태 확인 */
    if (AGR_NMT_GetState(&inst->nmt) == AGR_NMT_STOPPED) {
        if (old_state != AGR_NMT_STOPPED) {
            /* ✅ 연결 끊김 콜백 */
            _PnP_OnDisconnected(AGR_NODE_ID_IMU_HUB);
        }
    }
}
```

#### **2. 연결 끊김 콜백**
```c
/* imu_hub_drv.c:484 */
static void _PnP_OnDisconnected(uint8_t node_id)
{
    (void)node_id;
    
    /* ✅ Pre-Op 상태 초기화 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_IDLE;
    s_imu_hub_inst.sdo_retry_count = 0;
    s_imu_hub_inst.imu_connected_mask = 0x00;
    
    /* TODO: User 콜백 호출 (상위 레이어 통지) */
}
```

#### **3. 재연결 (Boot-up 수신)**
```c
/* ImuHub_Drv_ProcessCANMessage() */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    /* ✅ Boot-up (CAN ID 0x708, Data[0] = 0x00) */
    uint16_t fnc_code = can_id & 0x780;
    if (fnc_code == 0x700) {
        AGR_NMT_ProcessMessage(&s_imu_hub_inst.nmt, can_id, data, len, current_ms);
        ↓
        /* AGR_NMT 내부: Boot-up 감지 */
        if (nmt_state_byte == 0x00) {
            AGR_NMT_SetState(inst, AGR_NMT_PRE_OPERATIONAL);
            AGR_NMT_UpdateActivity(inst, current_ms);
            ↓
            /* Boot-up 콜백 */
            inst->on_bootup(inst->node_id);  /* NULL이면 미호출 */
        }
        return;
    }
}

/* AGR_PnP 내부: Boot-up 감지 후 콜백 */
AGR_PnP_RunPeriodic(&s_imu_hub_inst.pnp_inst);
↓
if (AGR_NMT_GetState(&inst->nmt) == AGR_NMT_PRE_OPERATIONAL) {
    if (old_state == AGR_NMT_STOPPED) {
        /* ✅ Boot-up 콜백 */
        _PnP_OnBootup(AGR_NODE_ID_IMU_HUB);
    }
}
```

#### **4. Boot-up 콜백 (Pre-Op 재시작)**
```c
/* imu_hub_drv.c:499 */
static void _PnP_OnBootup(uint8_t node_id)
{
    (void)node_id;
    
    /* ✅ Pre-Op 상태 머신 시작 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
    s_imu_hub_inst.sdo_retry_count = 0;
}
```

#### **5. Pre-Op 시퀀스 진행**
```c
/* _PnP_RunPreOp() - AGR_PnP_RunPeriodic()에서 주기 호출 */
_PnP_RunPreOp(AGR_NODE_ID_IMU_HUB, &s_imu_hub_inst.pnp_inst);
↓
switch (s_imu_hub_inst.pre_op_state) {
    case IMUHUB_PRE_OP_SEND_PDO_MAP_A:
        AGR_PnP_SendSDOWrite(...);  /* TPDO1 Mapping */
        break;
    /* ... PDO Mapping A → B → IMU Mask → NMT START ... */
}
```

### **✅ 재연결 로직 정리**

| 단계 | 이벤트 | 처리 | 상태 전환 |
|------|--------|------|-----------|
| **1. 연결 끊김** | Heartbeat Timeout (3s) | `AGR_NMT_CheckTimeout()` | OPERATIONAL → STOPPED |
| **2. 끊김 감지** | NMT STOPPED | `_PnP_OnDisconnected()` | Pre-Op: IDLE |
| **3. Boot-up 수신** | CAN ID 0x708, 0x00 | `AGR_NMT_ProcessMessage()` | STOPPED → PRE_OPERATIONAL |
| **4. Boot-up 콜백** | NMT PRE_OPERATIONAL | `_PnP_OnBootup()` | Pre-Op: SEND_PDO_MAP_A |
| **5. Pre-Op 진행** | 주기 실행 | `_PnP_RunPreOp()` | PDO Mapping → NMT START |
| **6. OPERATIONAL** | Heartbeat 0x05 | `AGR_NMT_ProcessMessage()` | PRE_OPERATIONAL → OPERATIONAL |
| **7. 연결 완료** | NMT OPERATIONAL | `_PnP_OnConnected()` | Pre-Op: IDLE |

**결론**: ✅ **자동 재연결 지원** (Heartbeat Timeout → Boot-up 수신 → Pre-Op 재시작)

---

## 7. Tx FIFO Full 기본 정책 ⚠️

### **STM32 FDCAN Tx FIFO Full 동작**

#### **하드웨어 동작 (STM32G474)**
```
┌─────────────────────────────────────────────────────┐
│ HAL_FDCAN_AddMessageToTxFifoQ() 호출                 │
│     ↓                                                │
│ Tx FIFO 여유 확인 (GetTxFifoFreeLevel)               │
│     ↓                                                │
│ [Tx FIFO Full (0/3)]                                 │
│     ↓                                                │
│ ❌ HAL_ERROR 반환 (메시지 Drop)                       │
│     ↓                                                │
│ Application에서 처리 필요:                            │
│ - 재시도 (Busy Loop) ← ⚠️ Bus Off 위험!              │
│ - Drop (무시) ← ✅ Error Frame 방지                   │
│ - Software Queue ← ✅ 권장                            │
└─────────────────────────────────────────────────────┘
```

#### **Rx FIFO vs Tx FIFO 비교**

| 구분 | Rx FIFO | Tx FIFO |
|------|---------|---------|
| **Full 시 동작** | Blocking (기본) / Overwrite (설정) | **Drop (HAL_ERROR)** |
| **하드웨어 지원** | Overwrite 모드 제공 | **지원 없음** |
| **Application 책임** | 선택 가능 (Blocking/Overwrite) | **필수 처리 필요** |

#### **Rx FIFO Overwrite 설정 예시**
```c
/* Rx FIFO0 Overwrite 모드 */
hfdcan.Init.RxFifo0OperationMode = FDCAN_RX_FIFO_OVERWRITE;  /* 새 메시지가 오래된 메시지 덮어씀 */

/* Rx FIFO0 Blocking 모드 (기본) */
hfdcan.Init.RxFifo0OperationMode = FDCAN_RX_FIFO_BLOCKING;   /* FIFO Full 시 새 메시지 Drop */
```

### **✅ Tx FIFO Full 기본 정책**

```
❌ 하드웨어 자동 처리 없음
✅ HAL_FDCAN_AddMessageToTxFifoQ() → HAL_ERROR 반환
⚠️ Application이 직접 처리 필요
```

**권장 정책**:
1. ✅ **Software Queue** (최우선)
2. ✅ **Drop (무시)** (Error Frame 방지)
3. ❌ **Busy Loop 재시도** (Bus Off 위험)

---

## 8. TEC 증가 원인 및 Bus Off ✅

### **CAN Error Counter 메커니즘**

#### **에러 카운터 종류**
```
TEC (Transmit Error Counter): 전송 에러 카운터
REC (Receive Error Counter): 수신 에러 카운터

[상태 전환]
TEC < 128, REC < 128: Error Active (정상)
    ↓ (에러 누적)
TEC >= 128 또는 REC >= 128: Error Passive (경고)
    ↓ (계속 에러)
TEC >= 256: Bus Off (전송 중단) ❌
```

### **TEC 증가 시나리오**

#### **시나리오 1: Tx FIFO Full → 메시지 Drop (✅ 안전)**
```
Application: HAL_FDCAN_AddMessageToTxFifoQ()
    ↓
Tx FIFO Full (3/3)
    ↓
HAL: HAL_ERROR 반환 (메시지 Drop)
    ↓
❌ 전송 시도 안 함 (CAN Bus에 메시지 안 보냄)
    ↓
✅ TEC 증가 없음! (Error Frame 발생 안 함)
```

#### **시나리오 2: Busy Loop 재시도 (❌ 위험)**
```
Application: HAL_FDCAN_AddMessageToTxFifoQ()
    ↓
Tx FIFO Full (3/3)
    ↓
HAL: HAL_ERROR 반환
    ↓
Application: while (HAL_ERROR) { retry(); }  /* ⚠️ Busy Loop */
    ↓
Tx FIFO 여유 생김 (1/3)
    ↓
메시지 전송 시도
    ↓
❌ Bus 타이밍 문제 또는 Arbitration Loss
    ↓
Error Frame 발생 (CAN Bus에 전송)
    ↓
✅ TEC += 8 (에러 누적)
    ↓ (반복)
TEC >= 256 → Bus Off ❌
```

#### **시나리오 3: 전송 충돌 (Arbitration Loss)**
```
XM10: TPDO1 전송 시도 (0x188)
    ↓
CM (Control Module): 동시에 0x185 전송 시도
    ↓
Arbitration: 0x185 < 0x188 (CM 승리)
    ↓
XM10: Arbitration Lost (재전송 필요)
    ↓
❌ 여러 번 재전송 실패 시
    ↓
Error Frame 발생
    ↓
TEC += 8
```

#### **시나리오 4: Ack Error (연결 끊김)**
```
XM10: TPDO1 전송 (0x188)
    ↓
IMU Hub: 전원 차단 (연결 끊김)
    ↓
❌ Ack Bit 수신 실패
    ↓
Error Frame 발생
    ↓
TEC += 8 (매 1ms마다)
    ↓
100ms 후: TEC = 800 → Bus Off ❌
```

### **✅ TEC 증가 원인 정리**

| 원인 | TEC 증가 | 발생 조건 |
|------|----------|-----------|
| **Tx FIFO Full (Drop)** | ❌ 증가 안 함 | HAL_ERROR 반환, 전송 안 함 |
| **Busy Loop 재시도** | ✅ 증가 | 타이밍 문제 발생 시 |
| **Arbitration Loss** | ✅ 증가 | 동시 전송, 낮은 우선순위 |
| **Ack Error** | ✅ 증가 (빠름) | Slave 연결 끊김 |
| **Bit Error** | ✅ 증가 | 하드웨어 문제, 케이블 불량 |

### **✅ 결론**

**Tx FIFO Full 자체는 TEC 증가 원인이 아님!**
```
Tx FIFO Full (Drop) → ✅ 안전 (TEC 증가 안 함)
Tx FIFO Full (Busy Loop 재시도) → ❌ 위험 (TEC 증가 가능)
```

**Bus Off 발생 원인**:
1. ✅ **Ack Error** (연결 끊김) - 가장 빠른 Bus Off
2. ✅ **Busy Loop 재시도** - 타이밍 문제로 Error Frame 발생
3. ✅ **Arbitration Loss** - 동시 전송 충돌

**방지 방법**:
1. ✅ **Software Queue** (Tx FIFO Full 시 Queue에 추가)
2. ✅ **Drop (무시)** (재시도하지 않음)
3. ✅ **Bus Off Auto Recovery** (자동 복구)

---

## 📋 최종 정리

| 질문 | 답변 | 상태 | 조치 필요 |
|------|------|------|----------|
| **1. Init 위치** | system_startup.c 누락 | ❌ | ✅ **추가 필요** |
| **2. Step Array** | 미구현 (현재 불필요) | ⚠️ | ⏳ 향후 고려 |
| **3. AGR_NMT_InitEx** | 적절함 (콜백 NULL OK) | ✅ | - |
| **4. tx_func 중복** | 중복 아님 (모듈 독립성) | ✅ | - |
| **5. _ImuHub_SendSdo** | Legacy (미사용) | ❌ | ✅ **삭제 권장** |
| **6. 재연결 로직** | 자동 지원 (Heartbeat 기반) | ✅ | - |
| **7. Tx FIFO Full** | Drop (HAL_ERROR) | ⚠️ | ✅ **Software Queue 추가** |
| **8. TEC 증가** | Error Frame 발생 시 | ✅ | ✅ **Bus Off Recovery** |

---

## 🚀 즉시 조치 필요 사항

### **1. system_startup.c에 ImuHub_Drv_Init() 추가** ⭐⭐⭐⭐⭐
```c
#include "imu_hub_drv.h"

void System_Startup(void)
{
    IOIF_TIM_Init();
    IOIF_AGRB_FDCAN_Init();
    
    /* ✅ 추가 필요! */
    ImuHub_Drv_Init(IOIF_AGRB_FDCAN_Transmit, NULL);
    
    PnPManager_Init();
    Core_Process_Init();
}
```

### **2. _ImuHub_SendSdo() 삭제** ⭐⭐⭐
```diff
- static void _ImuHub_SendSdo(const AGR_SDO_Msg_t* sdo_msg) { ... }
```

### **3. Software Queue 구현** ⭐⭐⭐⭐⭐
```c
/* IMU Hub Module - Tx FIFO Full 대비 */
CanTxQueue_Push(&header, data, CAN_PRIORITY_TPDO);
CanTxQueue_Process(&hfdcan2);  /* Main Loop */
```

### **4. Bus Off Auto Recovery** ⭐⭐⭐⭐⭐
```c
HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
/* 콜백 구현 */
```

---

**결론**: 대부분의 설계는 적절하나, **Init 호출 누락**과 **Tx FIFO 대책**이 즉시 필요합니다! 🎯
