# AGR PnP/NMT 콜백 체인 완전 가이드

**작성일**: 2025-12-08  
**목적**: NMT 콜백과 PnP 콜백의 차이 및 전체 흐름 명확화

---

## 🚨 현재 문제

**콜백이 너무 많아서 헷갈립니다!**

- AGR_NMT 콜백
- AGR_PnP 내부 핸들러
- AGR_PnP_Device_t 콜백
- Device Driver 콜백

**총 4단계!** 😱

---

## 📊 콜백 체인 전체 구조

### **1. AGR_NMT 콜백 (Low-Level)**

**위치**: `agr_nmt.c`  
**역할**: CANopen NMT 프로토콜 처리

```c
typedef struct {
    AGR_NMT_State_t state;
    uint8_t node_id;
    uint32_t timeout_ms;
    AGR_NMT_Mode_t mode;  /* ✅ SIMPLE vs FULL (Legacy 아님!) */
    
    /* 콜백 */
    AGR_NMT_StateChangedCb_t on_state_changed;  /* 상태 변경 */
    AGR_NMT_TimeoutCb_t on_timeout;             /* Heartbeat Timeout */
    void* user_ctx;
} AGR_NMT_Inst_t;
```

**콜백 호출 시점**:
- `on_state_changed`: NMT 상태 변경 시 (BOOT_UP → PRE_OP → OPERATIONAL)
- `on_timeout`: Heartbeat Timeout 시

---

### **2. AGR_PnP 내부 핸들러 (Bridge)**

**위치**: `agr_pnp.c` (static 함수)  
**역할**: NMT 콜백을 받아서 PnP 콜백으로 전달

```c
/* NMT 콜백 → PnP 콜백 변환 */
static void _OnNmtStateChanged(AGR_NMT_State_t old_state, 
                               AGR_NMT_State_t new_state, 
                               void* user_ctx)
{
    AGR_PnP_Device_t* dev = (AGR_PnP_Device_t*)user_ctx;
    
    /* Boot-up 진입 → on_bootup 콜백 */
    if (new_state == AGR_NMT_BOOT_UP) {
        if (dev->callbacks.on_bootup != NULL) {
            dev->callbacks.on_bootup(dev->node_id);
        }
    }
    
    /* 상태 변경 → on_nmt_change 콜백 */
    if (dev->callbacks.on_nmt_change != NULL) {
        dev->callbacks.on_nmt_change(dev->node_id, old_state, new_state);
    }
}
```

**왜 필요한가?**
- AGR_NMT는 범용 모듈 (PnP 몰라도 됨)
- AGR_PnP는 NMT 위에서 동작
- 중간에 Bridge가 필요!

---

### **3. AGR_PnP_Device_t 콜백 (High-Level)**

**위치**: `agr_pnp.h`  
**역할**: Master가 Slave Device를 관리할 때 사용

```c
typedef struct {
    void (*on_bootup)(uint8_t node_id);
    void (*on_nmt_change)(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
    void (*on_run_pre_op)(uint8_t node_id);
    void (*on_connected)(uint8_t node_id);
    void (*on_disconnected)(uint8_t node_id);
    void (*on_error)(uint8_t node_id, const char* msg);
} AGR_PnP_Callbacks_t;
```

**사용처**: `AGR_PnP_Device_t` (Master가 Slave 모니터링)

---

### **4. Device Driver 콜백 (Application)**

**위치**: `imu_hub_drv.c`, `xm_drv.c`  
**역할**: 실제 응용 로직 구현

**Master (imu_hub_drv.c)**:
```c
static void _PnP_OnBootup(uint8_t node_id) {
    /* Pre-Op 초기화 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
}

static void _PnP_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state) {
    /* Pre-Op State Machine 실행 */
}
```

**Slave (xm_drv.c)**:
```c
static void _PnP_OnConnected(uint8_t node_id) {
    /* 연결 상태 업데이트 */
}

static void _PnP_OnDisconnected(uint8_t node_id) {
    /* 끊김 상태 업데이트 */
}
```

---

## 🔄 **Master (XM) 콜백 체인 전체 흐름**

### **Boot-up 수신 시**

```
[IMU Hub - Slave]
  └─ Boot-up 전송 (0x708, [0x00])

[XM - Master]
  └─ [1] AGR_NMT_ProcessMessage (agr_nmt.c)
      └─ inst->state = BOOT_UP
      └─ AGR_NMT_UpdateActivity
          └─ inst->state = PRE_OPERATIONAL
          └─ _NotifyStateChange(BOOT_UP, PRE_OP)
              ↓
  └─ [2] _OnNmtStateChanged (agr_pnp.c - 내부 핸들러)
      └─ dev->callbacks.on_bootup(node_id)
          ↓
  └─ [3] _PnP_OnBootup (imu_hub_drv.c)
      └─ s_imu_hub_inst.pre_op_state = SEND_PDO_MAP_A ✅
      
      └─ dev->callbacks.on_nmt_change(node_id, BOOT_UP, PRE_OP)
          ↓
  └─ [4] _PnP_OnNmtChange (imu_hub_drv.c)
      └─ Pre-Op State Machine 실행 ✅
```

---

## 🔄 **Slave (IMU Hub) 콜백 체인 전체 흐름**

### **Master Heartbeat 수신 시**

```
[XM - Master]
  └─ Heartbeat 전송 (0x702, [0x05])

[IMU Hub - Slave]
  └─ [1] AGR_NMT_ProcessMessage (agr_nmt.c)
      └─ inst->state = OPERATIONAL
      └─ _NotifyStateChange(PRE_OP, OPERATIONAL)
          ↓
  └─ [2] _OnNmtStateChanged (agr_pnp.c - 내부 핸들러)
      └─ ❌ Slave는 AGR_PnP_RegisterMaster 사용
          └─ ❌ NMT 콜백이 설정되지 않음!
```

**문제**: Slave의 NMT 콜백이 NULL!

---

## 🚨 **현재 문제점**

### **Master (XM) - ✅ 정상**

```c
/* AGR_PnP_RegisterDevice (agr_pnp.c) */
AGR_NMT_InitEx(&new_dev->nmt, 
               timeout, 
               device->node_id, 
               _OnNmtStateChanged,  /* ✅ 콜백 자동 설정 */
               _OnNmtTimeout, 
               new_dev);
```

---

### **Slave (IMU Hub) - ❌ 문제**

```c
/* XM_Drv_Init (xm_drv.c) */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000, 
               AGR_NODE_ID_XM, 
               NULL,  /* ❌ on_state_changed: NULL */
               NULL,  /* ❌ on_timeout: NULL */
               NULL);
```

**결과**: Slave가 Master Heartbeat를 받아도 콜백이 호출되지 않음!

---

## ✅ **해결 방법**

### **Option 1: Slave도 NMT 콜백 설정**

```c
/* XM_Drv_Init (xm_drv.c) */

static void _OnMasterNmtStateChanged(AGR_NMT_State_t old_state, 
                                      AGR_NMT_State_t new_state, 
                                      void* user_ctx)
{
    /* Master 상태 변경 → PnP 콜백으로 전달 */
    if (s_imu_hub_pnp_inst.config.callbacks.on_nmt_change != NULL) {
        s_imu_hub_pnp_inst.config.callbacks.on_nmt_change(AGR_NODE_ID_XM, old_state, new_state);
    }
}

AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000, 
               AGR_NODE_ID_XM, 
               _OnMasterNmtStateChanged,  /* ✅ 콜백 설정 */
               NULL, 
               NULL);
```

---

### **Option 2: AGR_PnP_RegisterMaster에서 자동 설정**

`AGR_PnP_RegisterMaster` 함수를 수정하여 Slave의 NMT 콜백도 자동으로 설정:

```c
/* AGR_PnP_RegisterMaster (agr_pnp.c) */
int AGR_PnP_RegisterMaster(AGR_PnP_Inst_t* inst, const AGR_PnP_Master_t* master)
{
    /* ... 기존 코드 ... */
    
    /* ✅ Slave의 NMT 콜백 설정 (Master 모니터링) */
    AGR_NMT_InitEx(&inst->config.nmt, 
                   master->heartbeat_timeout, 
                   master->node_id, 
                   _OnMasterNmtStateChanged,  /* ✅ 내부 핸들러 */
                   _OnMasterNmtTimeout, 
                   inst);
    
    return 0;
}
```

---

## 🎯 **`mode`는 Legacy가 아닙니다!**

```c
/* AGR_NMT_UpdateActivity - Boot-up 상태 전이 */
switch (old_state) {
    case AGR_NMT_BOOT_UP:
        if (inst->mode == AGR_NMT_MODE_SIMPLE) {
            /* Simple Mode: BOOT_UP → OPERATIONAL 직행 */
            /* UART 센서용 (EBIMU-9DOFv6) */
            inst->state = AGR_NMT_OPERATIONAL;
        } else {
            /* Full Mode: BOOT_UP → PRE_OPERATIONAL */
            /* CANopen 기반 PnP (XM-IMU Hub) */
            inst->state = AGR_NMT_PRE_OPERATIONAL;
        }
        break;
}
```

**용도**:
- ✅ **SIMPLE Mode**: UART 센서 등 단순 연결 (설정 없이 바로 OPERATIONAL)
- ✅ **FULL Mode**: CANopen 기반 PnP (PRE_OP에서 SDO 설정 후 OPERATIONAL)

**기본값**: `AGR_NMT_MODE_FULL` (AGR_NMT_Init)

---

## 📋 **콜백 등록 체크리스트**

### **Master (XM) - `imu_hub_drv.c`**

| 레벨 | 콜백 | 설정 위치 | 호출 시점 | 상태 |
|------|------|-----------|----------|------|
| **NMT** | `_OnNmtStateChanged` | `AGR_PnP_RegisterDevice` | Slave NMT 변경 | ✅ 자동 |
| **NMT** | `_OnNmtTimeout` | `AGR_PnP_RegisterDevice` | Slave Timeout | ✅ 자동 |
| **PnP** | `on_bootup` | `ImuHub_Drv_Init` | Slave Boot-up | ✅ |
| **PnP** | `on_nmt_change` | `ImuHub_Drv_Init` | Slave NMT 변경 | ✅ |
| **PnP** | `on_run_pre_op` | `ImuHub_Drv_Init` | Pre-Op 단계 | ✅ |
| **PnP** | `on_error` | `ImuHub_Drv_Init` | 에러 | ✅ |

---

### **Slave (IMU Hub) - `xm_drv.c`**

| 레벨 | 콜백 | 설정 위치 | 호출 시점 | 상태 |
|------|------|-----------|----------|------|
| **NMT** | `on_state_changed` | `XM_Drv_Init` | Master NMT 변경 | ❌ NULL |
| **NMT** | `on_timeout` | `XM_Drv_Init` | Master Timeout | ❌ NULL |
| **PnP** | `on_connected` | `XM_Drv_Init` | Master 연결 | ✅ |
| **PnP** | `on_disconnected` | `XM_Drv_Init` | Master 끊김 | ✅ |
| **PnP** | `on_nmt_change` | `XM_Drv_Init` | Master NMT 변경 | ✅ (비어있음) |
| **PnP** | `on_error` | `XM_Drv_Init` | 에러 | ✅ |
| **DOP** | `on_sdo_request` | `XM_Drv_Init` | SDO 요청 | ✅ |
| **DOP** | `on_pdo_received` | `XM_Drv_Init` | PDO 수신 | ✅ |

---

## 🚀 **권장 사항**

### **1. Slave NMT 콜백 설정**

현재 Slave의 NMT 콜백이 NULL이므로, Master Heartbeat를 받아도 콜백이 호출되지 않습니다.

**제안**: `XM_Drv_Init`에서 NMT 콜백 설정

---

### **2. 콜백 체인 간소화 고려**

현재 4단계 콜백 체인이 너무 복잡합니다:

```
AGR_NMT 콜백 → AGR_PnP 내부 핸들러 → AGR_PnP 콜백 → Device Driver 콜백
```

**대안**:
- Master: 현재 구조 유지 (잘 동작함)
- Slave: AGR_PnP가 NMT를 직접 관리하도록 단순화

---

**결론**: 콜백 체인이 복잡하지만, 각 레벨이 명확한 역할을 가지고 있습니다. Slave의 NMT 콜백만 설정하면 완전해집니다! ✅
