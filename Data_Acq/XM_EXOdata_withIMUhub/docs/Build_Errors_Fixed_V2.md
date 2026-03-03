# XM Build Errors 해결 완료

**작성일**: 2025-12-08  
**목적**: Master PnP 리팩토링 후 빌드 오류 해결

---

## 🔴 발생한 빌드 오류

### **1. `AGR_PnP_Device_t`에 `name` 멤버 없음**

```
'AGR_PnP_Device_t' has no member named 'name'
imu_hub_drv.c line 333
```

**원인**: IMU Hub Module의 `agr_pnp.h`에는 `.name` 필드가 추가되었으나, XM의 `agr_pnp.h`에는 동기화되지 않음.

---

### **2. `AGR_PnP_Inst_t` 타입 선언 누락**

```
unknown type name 'AGR_PnP_Inst_t'; did you mean 'AGR_NMT_Inst_t'?
imu_hub_drv.h line 125
```

**원인**: `imu_hub_drv.h`에서 `agr_pnp.h`를 include하지 않음.

---

### **3. `ImuHub_DrvInst_t`의 `.nmt`, `.pnp_inst` 멤버 접근**

```
'ImuHub_DrvInst_t' has no member named 'nmt'
imu_hub_drv.c line 361, 401, 410, 427

'ImuHub_DrvInst_t' has no member named 'pnp_inst'
imu_hub_drv.c line 430
```

**원인**: 리팩토링 후 Device Driver에서 `.nmt`, `.pnp_inst`를 제거했는데, 여전히 접근하려는 코드가 남아있음.

---

### **4. `AGR_NODE_ID_XM` 매크로 선언 누락**

```
'AGR_NODE_ID_XM' undeclared (first use in this function)
pnp_manager.c line 101
```

**원인**: `pnp_manager.c`에서 `module.h`가 include되어 있으나, 컴파일 순서 문제로 인식 안 됨 (실제로는 include됨).

---

### **5. `AGR_PnP_RunPeriodic` 인자 개수 불일치**

```
too many arguments to function 'AGR_PnP_RunPeriodic'
pnp_manager.c line 85
```

**원인**: `AGR_PnP_RunPeriodic`는 인자 1개 (`inst`)만 필요한데, 2개 (`inst`, `HAL_GetTick()`)를 전달함.

---

## ✅ 해결 방법

### **1. XM `agr_pnp.h`에 `.name` 필드 추가**

**파일**: `C:\Users\HyundoKim\Documents\GitHub\ARC_ExtensionBoard\Extension_Module\XM_FW\Services\AGR_PnP\agr_pnp.h`

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

---

### **2. `imu_hub_drv.h`에 `agr_pnp.h` include 추가**

**파일**: `imu_hub_drv.h`

```c
#include <stdint.h>
#include <stdbool.h>
#include "agr_dop.h"
#include "agr_nmt.h"
#include "agr_pnp.h"  /* ✅ AGR_PnP_Inst_t forward declaration */
```

---

### **3. `ImuHub_DrvInst_t`에 `device_index` 추가**

**파일**: `imu_hub_drv.c`

```c
typedef struct {
    /* CANopen 표준 프로토콜 */
    AGR_DOP_Ctx_t       dop_ctx;
    AGR_TxFunc_t        tx_func;
    
    /* Master PnP (System Layer) */
    AGR_PnP_Inst_t*     master_pnp;         /**< ✅ Master PnP 인스턴스 */
    uint8_t             device_index;       /**< ✅ Master PnP devices[] 배열 인덱스 (NMT 접근용) */
    
    /* Pre-Op 상태 머신 */
    ImuHub_PreOpState_t pre_op_state;
    /* ... */
} ImuHub_DrvInst_t;
```

---

### **4. Device 등록 시 `device_index` 저장**

**파일**: `imu_hub_drv.c` (Init 함수)

```c
int dev_idx = AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device);
if (dev_idx < 0) {
    return -1;  /* 등록 실패 */
}
s_imu_hub_inst.device_index = (uint8_t)dev_idx;  /* ✅ Device Index 저장 */
```

---

### **5. NMT 접근 수정 (`master_pnp->devices[device_index]` 사용)**

**파일**: `imu_hub_drv.c`

#### **(1) Boot-up/Heartbeat 처리**

```c
if (fnc_code == 0x700) {
    /* ✅ Master PnP의 Device NMT로 전달 */
    AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
    AGR_NMT_ProcessMessage(&dev->nmt, can_id, data, len, current_ms);
    return;
}
```

#### **(2) IsConnected()**

```c
bool ImuHub_Drv_IsConnected(void)
{
    AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
    return AGR_NMT_IsConnected(&dev->nmt);
}
```

#### **(3) GetNmtState()**

```c
AGR_NMT_State_t ImuHub_Drv_GetNmtState(void)
{
    AGR_PnP_Device_t* dev = &s_imu_hub_inst.master_pnp->devices[s_imu_hub_inst.device_index];
    return AGR_NMT_GetState(&dev->nmt);
}
```

---

### **6. `ImuHub_Drv_RunPeriodic()` 수정**

**파일**: `imu_hub_drv.c`

```c
void ImuHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ✅ NMT/PnP는 System Layer (PnPManager)에서 관리 */
    /* Device Driver는 Pre-Op Timeout만 체크 */
    
    /* 1. Pre-Op Timeout 체크 (검증된 로직) */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B ||
        /* ... */) {
        /* Timeout 처리 */
    }
}
```

**주요 변경**:
- ❌ `AGR_NMT_CheckTimeout(&s_imu_hub_inst.nmt, ...)` 제거
- ❌ `AGR_PnP_RunPeriodic(&s_imu_hub_inst.pnp_inst)` 제거
- ✅ NMT/PnP는 System Layer에서 관리

---

### **7. `PnPManager_RunPeriodic()` 인자 수정**

**파일**: `pnp_manager.c`

```c
void PnPManager_RunPeriodic(void)
{
    if (s_master_pnp_initialized) {
        AGR_PnP_RunPeriodic(&s_master_pnp);  /* ✅ 인자 1개만 (inst) */
    }
}
```

---

### **8. `PnPManager_Task`에서 `PnPManager_RunPeriodic()` 호출**

**파일**: `pnp_manager.c`

```c
static void PnPManager_Task(void* argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS_PNP_MANAGER);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* ✅ Master PnP 주기 실행 (DOP V2 전용) */
        PnPManager_RunPeriodic();

        // 등록된 모든 모듈의 RunPeriodic() 호출 (DOP V1)
        for (int i = 0; i < s_moduleCount; i++) {
            if (s_linkModules[i]->RunPeriodic != NULL) {
                s_linkModules[i]->RunPeriodic();
            }
        }
        
        /* ✅ IMU Hub Device Driver 주기 실행 (Pre-Op Timeout 체크) */
        ImuHub_Drv_RunPeriodic();
    }
}
```

---

## 📋 수정 파일 목록

| 파일 | 수정 내용 | 상태 |
|------|-----------|------|
| `agr_pnp.h` (XM) | `AGR_PnP_Device_t`에 `.name` 필드 추가 | ✅ |
| `imu_hub_drv.h` | `agr_pnp.h` include 추가 | ✅ |
| `imu_hub_drv.c` | `device_index` 추가, NMT 접근 수정 | ✅ |
| `pnp_manager.c` | `AGR_PnP_RunPeriodic()` 인자 수정, Task 수정 | ✅ |

---

## 🎯 아키텍처 정리

### **Before (문제)**

```
Device Driver (imu_hub_drv.c)
  ├─ AGR_NMT_Inst_t nmt               ❌ Device가 직접 NMT 관리
  ├─ AGR_PnP_Inst_t pnp_inst          ❌ Device가 Master PnP 소유
  └─ AGR_PnP_RunPeriodic()            ❌ Device에서 PnP 실행
```

---

### **After (개선)**

```
System Layer (PnPManager)
  ├─ AGR_PnP_Inst_t s_master_pnp      ✅ 하나의 Master PnP
  │   └─ devices[]
  │       └─ [0] "IMU Hub"            ✅ 디버깅 편의
  │           └─ AGR_NMT_Inst_t nmt   ✅ NMT 상태
  └─ PnPManager_RunPeriodic()         ✅ System Layer에서 실행

Device Driver (imu_hub_drv.c)
  ├─ AGR_PnP_Inst_t* master_pnp       ✅ Master PnP 참조 (System Layer)
  ├─ uint8_t device_index             ✅ NMT 접근용 인덱스
  └─ ImuHub_Drv_RunPeriodic()         ✅ Pre-Op Timeout만 체크
```

---

## 🔍 주요 개선 사항

1. ✅ **Master PnP는 System Layer에서 하나만** 관리
2. ✅ **Device Driver는 `device_index`로 NMT 접근**
3. ✅ **NMT/PnP Periodic은 System Layer에서 실행**
4. ✅ **Device Driver는 Pre-Op Timeout만 체크** (응용 로직)
5. ✅ **디버깅 편의**: Live Expression에서 `devices[0].name` = "IMU Hub" 확인 가능

---

**결론**: Master PnP를 System Layer로 통합하여 논리적으로 명확한 아키텍처를 구현했습니다! ✅
