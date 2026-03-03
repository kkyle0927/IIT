# Pre-Op 무한 루프 버그 수정

**작성일**: 2025-12-08  
**문제**: XM-IMU Hub 간 PnP가 계속 반복되는 현상

---

## 🐛 **발견된 버그 (2개)**

### **Bug 1: IMU Hub (Slave)가 Boot-up을 계속 재전송**

#### **원인**
```c
/* agr_pnp.c - AGR_PnP_RunPeriodic() */
if (inst->config.nmt.state == AGR_NMT_BOOT_UP) {
    /* BOOT_UP 상태 + Master 미연결 → Boot-up 재전송 */
    if (!inst->is_master_connected) {
        result = AGR_PnP_SendBootup(inst);  // ❌ 계속 Boot-up 재전송!
    }
}
```

**문제점**:
- `is_master_connected`가 **절대 `true`가 되지 않음**
- `last_master_hb_time`을 업데이트하는 코드가 없음
- 결과: Slave가 1초마다 Boot-up을 계속 재전송

---

#### **수정**

**1. Slave용 NMT 콜백 추가**
```c
/**
 * @brief Slave용 NMT 상태 변경 콜백 (Master Heartbeat 수신)
 * @details Slave가 Master의 Heartbeat/Boot-up을 받았을 때 시간 갱신
 */
static void _OnSlaveNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx)
{
    AGR_PnP_Inst_t* inst = (AGR_PnP_Inst_t*)user_ctx;
    if (inst == NULL || inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return;
    }
    
    /* ✅ Master의 Heartbeat/Boot-up 수신 → 연결 시간 갱신 */
    inst->last_master_hb_time = inst->get_tick();
    inst->is_master_connected = true;
    
    (void)old_state;
    (void)new_state;
}
```

**2. Master 등록 시 NMT 모니터링 시작**
```c
int AGR_PnP_RegisterMaster(AGR_PnP_Inst_t* inst, const AGR_PnP_Master_t* master)
{
    /* ... */
    
    /* Master 정보 저장 */
    memcpy(&inst->config.master, master, sizeof(AGR_PnP_Master_t));
    
    /* ✅ Master NMT 모니터링 시작 (Heartbeat 수신) */
    AGR_NMT_InitEx(&inst->config.nmt, master->node_id, master->heartbeat_timeout,
                   _OnSlaveNmtStateChanged, NULL, inst);
    
    return 0;
}
```

---

### **Bug 2: Master (XM)가 Boot-up을 받을 때마다 Pre-Op 재시작**

#### **원인**
```c
/* imu_hub_drv.c - _PnP_OnBootup() */
static void _PnP_OnBootup(uint8_t node_id)
{
    /* ❌ 무조건 Pre-Op 재시작 */
    s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
    s_imu_hub_inst.sdo_retry_count = 0;
}
```

**문제점**:
- Boot-up을 받을 때마다 **무조건 Pre-Op을 재시작**
- 이미 Pre-Op이 진행 중(`WAIT_PDO_MAP_A`, `WAIT_PDO_MAP_B` 등)이어도 `SEND_PDO_MAP_A`로 되돌아감
- Slave가 Bug 1로 인해 Boot-up을 계속 보내면, Master도 계속 Pre-Op을 재시작

---

#### **수정**

```c
/**
 * @brief IMU Hub Boot-up 수신 콜백
 */
static void _PnP_OnBootup(uint8_t node_id)
{
    (void)node_id;
    
    /* ✅ Boot-up 수신 → Pre-Op 상태 머신 시작 (IDLE 또는 COMPLETE 상태에서만) */
    /* 이미 Pre-Op 진행 중이면 무시 (중복 Boot-up 방지) */
    if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_IDLE ||
        s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_COMPLETE) {
        s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
        s_imu_hub_inst.sdo_retry_count = 0;
    }
}
```

**수정 내용**:
- ✅ **IDLE 또는 COMPLETE 상태에서만 Pre-Op 시작**
- ✅ **진행 중(`SEND/WAIT`)이면 Boot-up 무시**
- ✅ **중복 Boot-up으로 인한 재시작 방지**

---

## 🔄 **버그 시나리오 (수정 전)**

```
[Slave] Boot-up 전송 (초기화 시 1번)
  ↓
[Master] Boot-up 수신 → Pre-Op 시작 (SEND_PDO_MAP_A)
  ↓
[Master] WAIT_PDO_MAP_A (SDO Response 대기)
  ↓ (1초 경과)
[Slave] is_master_connected = false → Boot-up 재전송! ❌
  ↓
[Master] Boot-up 수신 → Pre-Op 재시작 (SEND_PDO_MAP_A) ❌
  ↓
[Master] WAIT_PDO_MAP_A (SDO Response 대기)
  ↓ (1초 경과)
[Slave] Boot-up 재전송! ❌
  ↓
[무한 반복...]
```

---

## ✅ **수정 후 시나리오**

```
[Slave] Boot-up 전송 (초기화 시 1번)
  ↓
[Master] Boot-up 수신 → Pre-Op 시작 (SEND_PDO_MAP_A)
  ↓
[Master] WAIT_PDO_MAP_A (SDO Response 대기)
  ↓
[Master] Heartbeat 전송 → [Slave] Heartbeat 수신
  ↓
[Slave] is_master_connected = true ✅ (Boot-up 재전송 중단!)
  ↓
[Master] Pre-Op 정상 진행 (A → B → IMU Mask → NMT START)
  ↓
[Master] COMPLETE ✅
  ↓
[Slave] NMT START 수신 → OPERATIONAL 진입 ✅
  ↓
[Slave] Heartbeat 전송 (1초마다)
  ↓
[Master] Slave OPERATIONAL Heartbeat 수신 ✅
  ↓
[연결 완료!]
```

---

## 📊 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Slave Boot-up 재전송** | 1초마다 계속 ❌ | 초기화 시 1번만 ✅ |
| **Master Pre-Op 재시작** | Boot-up마다 재시작 ❌ | 진행 중이면 무시 ✅ |
| **is_master_connected** | 항상 `false` ❌ | Heartbeat 수신 시 `true` ✅ |
| **Pre-Op 완료** | 무한 루프 ❌ | 정상 완료 ✅ |
| **NMT 상태** | 계속 PRE_OPERATIONAL ❌ | OPERATIONAL 진입 ✅ |

---

## 🎯 **수정된 파일**

1. **agr_pnp.c** (IMU Hub & XM 공통)
   - `_OnSlaveNmtStateChanged` 함수 추가
   - `AGR_PnP_RegisterMaster` 수정 (NMT 모니터링 추가)

2. **imu_hub_drv.c** (XM Master)
   - `_PnP_OnBootup` 수정 (중복 Boot-up 무시)

---

**결론**: 두 가지 버그를 수정하여 XM-IMU Hub 간 PnP가 정상적으로 완료됩니다! ✅
