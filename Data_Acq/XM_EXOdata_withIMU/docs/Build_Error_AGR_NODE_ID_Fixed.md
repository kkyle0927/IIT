# XM Build Error 해결: `AGR_NODE_ID_XM` 선언 누락

**작성일**: 2025-12-08  
**목적**: `AGR_NODE_ID_XM` 매크로 선언 누락 오류 해결

---

## 🔴 발생한 빌드 오류

```
Description: 'AGR_NODE_ID_XM' undeclared (first use in this function)
File: pnp_manager.c
Location: /Extension_Module/XM_FW/System/Links/PnP_Manager line 101
Type: C/C++ Problem
```

---

## 🔍 원인 분석

### **1. `AGR_NODE_ID_XM` 사용 위치**

**파일**: `pnp_manager.c` (line 101)

```c
void PnPManager_Init(AGR_TxFunc_t tx_func)
{
    /* ===== 1. Master PnP 초기화 (DOP V2 전용, XM10 전체) ===== */
    if (!s_master_pnp_initialized) {
        AGR_PnP_Config_t pnp_config = {
            .role = AGR_PNP_ROLE_MASTER,
            .my_node_id = AGR_NODE_ID_XM,  /* ❌ 선언되지 않음 */
            .tx_func = tx_func,
            /* ... */
        };
        /* ... */
    }
}
```

---

### **2. `AGR_NODE_ID_XM` 정의 위치**

**파일**: `agr_dop_node_id.h`

```c
/**
 * @brief Extension Module (XM10)
 */
#define AGR_NODE_ID_XM              0x02
```

**정의 파일 경로**:
- IMU Hub: `IMU_Hub_Module\IMU_Hub_FW\Services\AGR_DOP\agr_dop_node_id.h`
- XM: `Extension_Module\XM_FW\Services\AGR_DOP\agr_dop_node_id.h`

---

### **3. Include 누락**

**파일**: `pnp_manager.c`

```c
#include "pnp_manager.h"
#include "module.h"
// #include "agr_pnpmgr.h"  /* AGR PnP Manager (DOP V1 전용) */
#include "cm_xm_link.h"
#include "grf_xm_link.h"
#include "xsens_imu_xm_link.h"
#include "imu_hub_drv.h"
/* ❌ agr_dop_node_id.h가 include되지 않음 */
```

**문제**: `AGR_NODE_ID_XM` 매크로가 정의된 `agr_dop_node_id.h`가 include되지 않아 컴파일 오류 발생.

---

## ✅ 해결 방법

### **`pnp_manager.c`에 `agr_dop_node_id.h` include 추가**

**파일**: `pnp_manager.c`

```c
#include "pnp_manager.h"
#include "module.h"
#include "agr_dop_node_id.h"     /* ✅ AGR_NODE_ID_XM */
// #include "agr_pnpmgr.h"  /* AGR PnP Manager (DOP V1 전용) */
#include "cm_xm_link.h"
#include "grf_xm_link.h"
#include "xsens_imu_xm_link.h"
#include "imu_hub_drv.h"          /* ✅ V3.0: IMU Hub Device Driver (System Link 제거) */
#include "canfd_rx_handler.h"    /* PnP V2 Queue 가져오기 */
#include "ioif_agrb_fdcan.h"     /* IOIF_FDCAN_Msg_t */
```

---

## 📋 수정 파일 목록

| 파일 | 수정 내용 | 상태 |
|------|-----------|------|
| `pnp_manager.c` | `agr_dop_node_id.h` include 추가 | ✅ |

---

## 🎯 결과

### **Before (에러)**

```c
.my_node_id = AGR_NODE_ID_XM,  /* ❌ 선언되지 않음 */
```

**컴파일 오류**:
```
'AGR_NODE_ID_XM' undeclared (first use in this function)
```

---

### **After (해결)**

```c
#include "agr_dop_node_id.h"  /* ✅ AGR_NODE_ID_XM 정의 */

/* ... */

.my_node_id = AGR_NODE_ID_XM,  /* ✅ 0x02 (XM10 Node ID) */
```

**컴파일 성공** ✅

---

## 📚 관련 정보

### **`agr_dop_node_id.h`에 정의된 주요 Node ID**

```c
/* Core Modules */
#define AGR_NODE_ID_BROADCAST       0x00  /* Broadcast */
#define AGR_NODE_ID_CM              0x01  /* Control Module */
#define AGR_NODE_ID_XM              0x02  /* Extension Module (XM10) */

/* Extension Sensor Modules */
#define AGR_NODE_ID_IMU_HUB_A       0x08  /* IMU Hub - Group A (PDO 1) */
#define AGR_NODE_ID_IMU_HUB_B       0x09  /* IMU Hub - Group B (PDO 2) */
#define AGR_NODE_ID_IMU_HUB         0x08  /* IMU Hub - SDO/NMT용 */

/* Actuator Modules */
#define AGR_NODE_ID_MD_RH           0x06  /* Right Hip Motor Driver */
#define AGR_NODE_ID_MD_LH           0x07  /* Left Hip Motor Driver */
```

---

**결론**: `agr_dop_node_id.h`를 include하여 `AGR_NODE_ID_XM` 선언 누락 오류를 해결했습니다! ✅
