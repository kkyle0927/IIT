# AGR_NMT 콜백 설정 누락 수정

**작성일**: 2025-12-08  
**문제**: Boot-up 수신 시 상태 전이는 발생하지만 콜백이 호출되지 않음  
**원인**: `AGR_PnP_RegisterDevice`에서 NMT 콜백을 설정하지 않음  
**해결**: NMT 콜백을 내부 핸들러로 설정하여 `AGR_PnP_Device_t.callbacks`로 전파

---

## 🚨 문제 분석

### **증상**
- XM(Master)이 IMU Hub(Slave)의 Boot-up 메시지를 수신 ✅
- `AGR_NMT_ProcessMessage` 진입 ✅
- `inst->state = BOOT_UP` → `AGR_NMT_UpdateActivity` 호출 ✅
- `inst->state = PRE_OPERATIONAL` 변경 ✅
- `_NotifyStateChange` 호출 ✅
- **그러나 콜백이 호출되지 않음!** ❌

### **원인**

```c
/* AGR_PnP_RegisterDevice - 기존 코드 */
AGR_NMT_Init(&new_dev->nmt, timeout);
new_dev->nmt.node_id = device->node_id;
/* ❌ on_state_changed, on_timeout 콜백 미설정! */
```

**결과**:
- `AGR_NMT_UpdateActivity` 내부에서 상태 전이는 발생 ✅
- `_NotifyStateChange` 호출 시 `inst->on_state_changed == NULL` ❌
- `_PnP_OnBootup`, `_PnP_OnNmtChange` 등이 호출되지 않음 ❌

---

## 🎯 해결 방법

### **1. NMT 콜백 핸들러 추가**

AGR_PnP 내부에 NMT 콜백을 받아서 `AGR_PnP_Device_t.callbacks`로 전파하는 핸들러를 추가:

```c
/**
 * @brief NMT 상태 변경 콜백 (내부 핸들러)
 * @details AGR_NMT에서 호출되어 AGR_PnP_Device_t의 콜백으로 전파
 */
static void _OnNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx)
{
    AGR_PnP_Device_t* dev = (AGR_PnP_Device_t*)user_ctx;
    if (dev == NULL) {
        return;
    }
    
    /* Boot-up 진입 (재연결 신호) */
    if (new_state == AGR_NMT_BOOT_UP && old_state != AGR_NMT_BOOT_UP) {
        if (dev->callbacks.on_bootup != NULL) {
            dev->callbacks.on_bootup(dev->node_id);
        }
    }
    
    /* 상태 변경 알림 */
    if (dev->callbacks.on_nmt_change != NULL) {
        dev->callbacks.on_nmt_change(dev->node_id, old_state, new_state);
    }
}

/**
 * @brief NMT 타임아웃 콜백 (내부 핸들러)
 */
static void _OnNmtTimeout(void* user_ctx)
{
    AGR_PnP_Device_t* dev = (AGR_PnP_Device_t*)user_ctx;
    if (dev == NULL) {
        return;
    }
    
    if (dev->callbacks.on_error != NULL) {
        dev->callbacks.on_error(dev->node_id, "Heartbeat Timeout");
    }
}
```

---

### **2. AGR_PnP_RegisterDevice 수정**

NMT 초기화 시 콜백을 설정:

```c
/* Device 추가 */
AGR_PnP_Device_t* new_dev = &inst->devices[inst->device_count];
memcpy(new_dev, device, sizeof(AGR_PnP_Device_t));

/* NMT 초기화 (콜백 포함) */
uint32_t timeout = (device->heartbeat_timeout > 0) ? device->heartbeat_timeout : AGR_PNP_HEARTBEAT_TIMEOUT;
AGR_NMT_InitEx(&new_dev->nmt, 
               timeout, 
               device->node_id, 
               _OnNmtStateChanged,  /* ✅ 상태 변경 콜백 */
               _OnNmtTimeout,       /* ✅ 타임아웃 콜백 */
               new_dev);            /* ✅ user_ctx로 AGR_PnP_Device_t 전달 */

new_dev->is_registered = true;
inst->device_count++;

return inst->device_count - 1;  /* ✅ Device Index 반환 */
```

---

## 📊 콜백 경로

### **Before (버그 상태)**

```
Boot-up 수신
  └─ AGR_NMT_ProcessMessage
      └─ inst->state = BOOT_UP
      └─ AGR_NMT_UpdateActivity
          └─ inst->state = PRE_OPERATIONAL
          └─ _NotifyStateChange
              └─ if (inst->on_state_changed != NULL)  ❌ NULL!
                  └─ 콜백 호출 안 됨 ❌
```

---

### **After (수정 후)**

```
Boot-up 수신
  └─ AGR_NMT_ProcessMessage
      └─ inst->state = BOOT_UP
      └─ AGR_NMT_UpdateActivity
          └─ inst->state = PRE_OPERATIONAL
          └─ _NotifyStateChange
              └─ if (inst->on_state_changed != NULL)  ✅
                  └─ _OnNmtStateChanged(old, new, user_ctx) ✅
                      ├─ if (new == BOOT_UP)
                      │   └─ dev->callbacks.on_bootup(node_id) ✅
                      │       └─ _PnP_OnBootup(node_id) ✅
                      │
                      └─ dev->callbacks.on_nmt_change(node_id, old, new) ✅
                          └─ _PnP_OnNmtChange(node_id, old, new) ✅
```

---

## 🚀 수정 효과

### **Boot-up 시퀀스 (최종)**

```
[IMU Hub - Slave]
1. Startup → Boot-up 전송 (0x708, [0x00])

[XM - Master]
2. Boot-up 수신 → AGR_NMT_ProcessMessage
3. inst->state = BOOT_UP (리셋)
4. AGR_NMT_UpdateActivity 호출
   └─ switch (BOOT_UP) → inst->state = PRE_OPERATIONAL
   └─ _NotifyStateChange(BOOT_UP → PRE_OP)
       └─ _OnNmtStateChanged ✅
           ├─ dev->callbacks.on_bootup(node_id) ✅
           │   └─ _PnP_OnBootup ✅
           │
           └─ dev->callbacks.on_nmt_change(node_id, BOOT_UP, PRE_OP) ✅
               └─ _PnP_OnNmtChange ✅
                   └─ s_imu_hub_inst.pre_op_state = SEND_PDO_MAP_A ✅
           
5. ImuHub_Drv_RunPeriodic 실행
   └─ Pre-Op Timeout 체크
   └─ Step Array 실행 ✅
       └─ SEND_PDO_MAP_A → SDO 전송 ✅
       
6. Pre-Op 완료 → OPERATIONAL 전환 ✅
```

---

## 📋 수정 파일

| 파일 | 수정 내용 | 상태 |
|------|-----------|------|
| `agr_pnp.c` (XM) | NMT 콜백 핸들러 추가 (`_OnNmtStateChanged`, `_OnNmtTimeout`) | ✅ |
| `agr_pnp.c` (XM) | `AGR_PnP_RegisterDevice`에서 `AGR_NMT_InitEx` 호출 | ✅ |
| `agr_pnp.c` (IMU Hub) | NMT 콜백 핸들러 추가 (`_OnNmtStateChanged`, `_OnNmtTimeout`) | ✅ |
| `agr_pnp.c` (IMU Hub) | `AGR_PnP_RegisterDevice`에서 `AGR_NMT_InitEx` 호출 | ✅ |

---

**결론**: `AGR_PnP_RegisterDevice`에서 NMT 콜백을 설정하지 않아 상태 전이 이벤트가 전파되지 않던 문제를 해결했습니다. 이제 Boot-up 수신 시 `_PnP_OnBootup` → Pre-Op 로직 시작 → SDO 전송이 정상적으로 수행됩니다! ✅
