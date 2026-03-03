# Master PnP 리팩토링 완료

**작성일**: 2025-12-08  
**목적**: Master PnP를 System Layer로 통합하여 확장성 극대화

---

## 🎯 리팩토링 목표

1. ✅ **Master PnP를 System Layer로 이동** (Device Driver 독립성)
2. ✅ **Device Driver는 등록만 수행** (단순화)
3. ✅ **확장 시 패턴 반복** (IMU, EMG, FSR, FES 동일 구조)
4. ✅ **디버깅 편의성 향상** (Live Expression에서 Device 이름 표시)

---

## 📋 수정 사항

### **1. `AGR_PnP_Device_t`에 `name` 필드 추가**

**파일**: `agr_pnp.h`

```c
typedef struct {
    const char*       name;              /**< ✅ Device 이름 (디버깅용, e.g., "IMU Hub") */
    uint8_t           node_id;           /**< CANopen Node ID */
    uint32_t          heartbeat_timeout; /**< Heartbeat Timeout (ms) */
    AGR_PnP_Callbacks_t callbacks;       /**< 이벤트 콜백 */
    
    /* 내부 상태 (Private) */
    AGR_NMT_Inst_t         nmt;          /**< NMT 인스턴스 */
    bool                   is_registered; /**< 등록 완료? */
} AGR_PnP_Device_t;
```

**Live Expression에서 확인**:
```c
s_master_pnp.devices[0].name         // "IMU Hub" ✅
s_master_pnp.devices[0].nmt.state    // AGR_NMT_OPERATIONAL
s_master_pnp.devices[1].name         // "EMG" (향후)
```

---

### **2. `pnp_manager.c` - Master PnP 통합**

**파일**: `pnp_manager.c` (System Layer)

```c
/* ===== Master PnP 인스턴스 (DOP V2 전용) ===== */
static AGR_PnP_Inst_t s_master_pnp;  /**< ✅ XM10 Master PnP (모든 DOP V2 Device 관리) */
static bool s_master_pnp_initialized = false;

void PnPManager_Init(AGR_TxFunc_t tx_func)
{
    /* ===== 1. Master PnP 초기화 (DOP V2 전용, XM10 전체) ===== */
    if (!s_master_pnp_initialized) {
        AGR_PnP_Config_t pnp_config = {
            .role = AGR_PNP_ROLE_MASTER,
            .my_node_id = AGR_NODE_ID_XM,
            .tx_func = tx_func,  /* ✅ System Layer에서 제공 */
            .callbacks = { /* Master 공통 콜백 */ }
        };
        AGR_PnP_Init(&s_master_pnp, &pnp_config, HAL_GetTick);
        s_master_pnp_initialized = true;
    }
    
    /* ===== 2. DOP V1 Legacy 모듈 등록 (CM, GRF, XSENS) ===== */
    /* ... */
    
    /* ===== 3. PnP Manager 태스크 생성 ===== */
    /* ... */
}

AGR_PnP_Inst_t* PnPManager_GetMasterPnP(void)
{
    return s_master_pnp_initialized ? &s_master_pnp : NULL;
}

void PnPManager_RunPeriodic(void)
{
    if (s_master_pnp_initialized) {
        AGR_PnP_RunPeriodic(&s_master_pnp, HAL_GetTick());
    }
}
```

---

### **3. `imu_hub_drv.c` - Device 등록만 수행**

**파일**: `imu_hub_drv.c` (Device Layer)

```c
int ImuHub_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Inst_t* master_pnp)
{
    if (tx_func == NULL || master_pnp == NULL) {
        return -1;
    }
    
    /* 1. Device 자체 초기화 */
    s_imu_hub_inst.tx_func = tx_func;
    s_imu_hub_inst.master_pnp = master_pnp;  /* ✅ Master PnP 저장 */
    
    /* 2. AGR_DOP 초기화 (Device 전용) */
    AGR_DOP_Init(&s_imu_hub_inst.dop_ctx, NULL, AGR_NODE_ID_XM, tx_func);
    
    /* 3. Master PnP에 Device 등록 */
    AGR_PnP_Device_t imu_hub_device = {
        .name = "IMU Hub",                  /* ✅ 디버깅용 이름 */
        .node_id = AGR_NODE_ID_IMU_HUB,
        .heartbeat_timeout = 3000,
        .callbacks = {
            .on_bootup = _PnP_OnBootup,        /* ✅ IMU Hub 전용 콜백 */
            .on_nmt_change = _PnP_OnNmtChange,
            .on_run_pre_op = _PnP_RunPreOp,
            .on_error = _PnP_OnError
        }
    };
    AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device);  /* ✅ 등록만 */
    
    return 0;
}
```

**구조체 수정**:
```c
typedef struct {
    AGR_DOP_Ctx_t       dop_ctx;            /**< DOP V2 Context */
    AGR_TxFunc_t        tx_func;            /**< FDCAN Tx 함수 */
    AGR_PnP_Inst_t*     master_pnp;         /**< ✅ Master PnP (System Layer 제공) */
    /* ... */
} ImuHub_DrvInst_t;
```

---

### **4. `system_startup.c` - 통합 초기화**

**파일**: `system_startup.c`

```c
static void _InitSystemServices(void)
{
    /* ... FDCAN, GPIO 등 초기화 ... */
    
    /* ===== PnP Manager 초기화 (Master PnP 생성) ===== */
    // ✅ Master PnP는 System Layer에서 하나만 관리
    PnPManager_Init(System_Fdcan1_Transmit);
    
    /* ===== DOP V2 Device Driver 초기화 (Master PnP에 등록) ===== */
    // ✅ IMU Hub Device 등록
    AGR_PnP_Inst_t* master_pnp = PnPManager_GetMasterPnP();
    if (master_pnp != NULL) {
        ImuHub_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    }
    
    /* 향후 확장: EMG, FSR, FES Device 추가 */
    // EMG_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    // FSR_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    // FES_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    
    /* ... 나머지 서비스 초기화 ... */
}
```

---

## 📊 아키텍처 비교

### **Before (문제점)**

```
Device Layer (imu_hub_drv.c)
  ├─ AGR_PnP_Inst_t s_imu_hub_inst.pnp_inst  ❌ Master PnP가 Device에?
  │   ├─ role = AGR_PNP_ROLE_MASTER
  │   ├─ my_node_id = AGR_NODE_ID_XM
  │   └─ callbacks = { /* ... */ }
  └─ AGR_PnP_RegisterDevice()

Device Layer (emg_drv.c - 향후)
  ├─ AGR_PnP_Inst_t s_emg_inst.pnp_inst      ❌ 또 Master PnP?
  └─ AGR_PnP_RegisterDevice()
```

**문제**:
- ❌ Master PnP가 Device 개수만큼 중복
- ❌ Master 설정이 여러 개
- ❌ 논리적으로 맞지 않음

---

### **After (개선)**

```
System Layer (pnp_manager.c)
  ├─ AGR_PnP_Inst_t s_master_pnp (하나!) ✅
  │   ├─ role = AGR_PNP_ROLE_MASTER
  │   ├─ my_node_id = AGR_NODE_ID_XM
  │   └─ devices[]
  │       ├─ [0] "IMU Hub" (node_id=0x08)  ✅ 디버깅 편의
  │       ├─ [1] "EMG"      (node_id=0x09)
  │       └─ [2] "FSR"      (node_id=0x0A)
  │
  └─ PnPManager_GetMasterPnP() → &s_master_pnp

Device Layer (imu_hub_drv.c)
  ├─ AGR_PnP_Inst_t* master_pnp (참조)  ✅ System Layer 제공
  └─ AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device)

Device Layer (emg_drv.c)
  ├─ AGR_PnP_Inst_t* master_pnp (참조)  ✅ 동일 패턴
  └─ AGR_PnP_RegisterDevice(master_pnp, &emg_device)
```

**개선**:
- ✅ Master PnP는 System Layer에서 하나
- ✅ Device Driver는 등록만 수행
- ✅ 논리적으로 명확
- ✅ 확장 용이

---

## 🔍 디버깅 편의성

### **Live Expression에서 확인**

```c
// STM32CubeIDE Live Expression
s_master_pnp.devices[0].name         // "IMU Hub" ✅ 한눈에 확인
s_master_pnp.devices[0].nmt.state    // AGR_NMT_OPERATIONAL
s_master_pnp.devices[1].name         // "EMG"
s_master_pnp.devices[1].nmt.state    // AGR_NMT_PRE_OPERATIONAL
s_master_pnp.device_count            // 2
```

**Before (device[0], device[1])** ❌ 무슨 Device인지 모름  
**After ("IMU Hub", "EMG")** ✅ 한눈에 확인 가능

---

## 🎯 확장 예시

### **EMG Device 추가**

```c
/* emg_drv.c */
int EMG_Drv_Init(AGR_TxFunc_t tx_func, AGR_PnP_Inst_t* master_pnp)
{
    /* 1. Device 초기화 */
    AGR_DOP_Init(&s_emg_inst.dop_ctx, NULL, AGR_NODE_ID_XM, tx_func);
    
    /* 2. Master PnP에 등록 */
    AGR_PnP_Device_t emg_device = {
        .name = "EMG",                      /* ✅ 디버깅용 이름 */
        .node_id = AGR_NODE_ID_EMG,
        .heartbeat_timeout = 3000,
        .callbacks = { /* EMG 전용 콜백 */ }
    };
    AGR_PnP_RegisterDevice(master_pnp, &emg_device);
    
    return 0;
}
```

```c
/* system_startup.c */
AGR_PnP_Inst_t* master_pnp = PnPManager_GetMasterPnP();
ImuHub_Drv_Init(System_Fdcan1_Transmit, master_pnp);  /* ✅ IMU Hub */
EMG_Drv_Init(System_Fdcan1_Transmit, master_pnp);     /* ✅ EMG 추가 */
```

**확장 시 추가 작업**:
- ✅ `emg_drv.c` 작성 (패턴 복사 후 수정)
- ✅ `system_startup.c`에 한 줄 추가
- ✅ 끝! (Master PnP는 수정 불필요)

---

## 📋 수정 파일 목록

| 파일 | 수정 내용 | 상태 |
|------|-----------|------|
| `agr_pnp.h` | `AGR_PnP_Device_t`에 `name` 필드 추가 | ✅ |
| `pnp_manager.h` | `PnPManager_GetMasterPnP()`, `PnPManager_RunPeriodic()` 추가 | ✅ |
| `pnp_manager.c` | Master PnP 인스턴스 추가, 초기화 수정 | ✅ |
| `imu_hub_drv.h` | `ImuHub_Drv_Init()` 시그니처 변경 | ✅ |
| `imu_hub_drv.c` | Master PnP 참조, 등록만 수행 | ✅ |
| `system_startup.c` | PnPManager 먼저 초기화, Device 등록 | ✅ |

---

## 🎯 최종 결과

### **핵심 개선**

1. ✅ **Master PnP는 System Layer에서 하나만** 관리
2. ✅ **Device Driver는 독립적** (등록만 수행)
3. ✅ **확장 용이** (EMG, FSR, FES 추가 시 패턴 반복)
4. ✅ **디버깅 편의** (Live Expression에서 Device 이름 표시)

### **장점**

- ✅ Device 추가 시 `AGR_PnP_RegisterDevice()` 한 줄만 추가
- ✅ Master PnP는 하나 → Live Expression 디버깅 용이
- ✅ 논리적으로 명확 (Master는 System, Device는 독립)
- ✅ Step Array 패턴과 동일한 확장성

---

**결론**: Master PnP를 System Layer로 통합하여 확장성과 디버깅 편의성을 극대화했습니다! ✅
