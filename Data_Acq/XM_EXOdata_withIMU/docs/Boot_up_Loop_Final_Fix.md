# Boot-up 무한 루프 최종 수정

**작성일**: 2025-12-08  
**문제**: IMU Hub (Slave)가 계속 Boot-up을 재전송하는 현상

---

## 🐛 **Bug 3: Master Heartbeat를 잘못된 NMT 인스턴스로 처리**

### **핵심 원인**

**IMU Hub (Slave)는 2개의 NMT 인스턴스를 가집니다:**

1. **`s_imu_hub_nmt`**: **Slave 자신의 NMT 상태**
   - Master의 NMT Command (0x000)를 받아 자신의 상태 변경
   - PRE_OPERATIONAL → OPERATIONAL 전환
   
2. **`s_imu_hub_pnp_inst.config.nmt`**: **Master (XM)의 NMT 상태 추적**
   - Master의 Heartbeat (0x700 + XM Node ID)를 받아 Master 연결 상태 확인
   - `is_master_connected` 플래그 업데이트

---

### **잘못된 코드 (Before)**

```c
/* xm_drv.c - XM_Drv_ProcessCANMessage() */

/* 2. Boot-up / Heartbeat (CAN ID 0x700 + Node ID) */
uint16_t fnc_code = can_id & 0x780;
if (fnc_code == 0x700) {
    /* ❌ 버그: 모든 0x700 메시지를 자신의 NMT로 처리! */
    AGR_NMT_ProcessMessage(&s_imu_hub_nmt, (uint16_t)can_id, data, len, current_ms);
    return;
}
```

**문제점**:
- `0x700 + 0x01` (XM Master Heartbeat) → ❌ `s_imu_hub_nmt`로 처리 (잘못됨!)
- `0x700 + 0x02` (IMU Hub 자신) → ❌ `s_imu_hub_nmt`로 처리 (불필요)

**결과**:
1. Master의 Heartbeat를 받아도 `s_imu_hub_pnp_inst.config.nmt`가 업데이트되지 않음
2. `s_imu_hub_pnp_inst.config.nmt.state`가 계속 `AGR_NMT_BOOT_UP`으로 남음
3. `is_master_connected`가 계속 `false`로 남음
4. **1초마다 Boot-up 재전송** ❌

---

### **수정된 코드 (After)**

```c
/* xm_drv.c - XM_Drv_ProcessCANMessage() */

/* 2. Boot-up / Heartbeat (CAN ID 0x700 + Node ID) */
uint16_t fnc_code = can_id & 0x780;
if (fnc_code == 0x700) {
    uint8_t sender_node_id = can_id & 0x7F;  /* ✅ 송신자 Node ID 추출 */
    
    /* ✅ Master (XM)의 Heartbeat → Master 상태 추적 */
    if (sender_node_id == AGR_NODE_ID_XM) {
        AGR_NMT_ProcessMessage(&s_imu_hub_pnp_inst.config.nmt, (uint16_t)can_id, data, len, current_ms);
    }
    /* Slave 자신의 Heartbeat → 무시 (자신이 보낸 것) */
    /* 다른 Node의 Heartbeat → 무시 */
    
    return;
}
```

**수정 내용**:
1. ✅ **Node ID 추출**: `sender_node_id = can_id & 0x7F`
2. ✅ **Master Heartbeat**: `s_imu_hub_pnp_inst.config.nmt`로 처리
3. ✅ **자신의 Heartbeat**: 무시 (자신이 보낸 것)

---

## 🔄 **정상 동작 흐름**

```
[IMU Hub 초기화]
  ↓
s_imu_hub_nmt.state = AGR_NMT_BOOT_UP ✅ (자신의 상태)
s_imu_hub_pnp_inst.config.nmt.state = AGR_NMT_BOOT_UP ✅ (Master 상태 추적)
is_master_connected = false
  ↓
[IMU Hub] Boot-up 전송 (0x700 + 0x02, data[0] = 0x00)
  ↓
[XM] Boot-up 수신 → Pre-Op 시작
  ↓
[XM] Heartbeat 전송 (0x700 + 0x01, data[0] = 0x05)
  ↓
[IMU Hub] Master Heartbeat 수신 (0x701)
  ↓
sender_node_id = 0x01 == AGR_NODE_ID_XM ✅
  ↓
AGR_NMT_ProcessMessage(&s_imu_hub_pnp_inst.config.nmt, 0x701, [0x05], 1, ...)
  ↓
s_imu_hub_pnp_inst.config.nmt.state = AGR_NMT_OPERATIONAL ✅
  ↓
_OnSlaveNmtStateChanged() 콜백 호출 ✅
  ↓
inst->last_master_hb_time = inst->get_tick() ✅
inst->is_master_connected = true ✅
  ↓
[IMU Hub] AGR_PnP_RunPeriodic()
  ↓
if (inst->config.nmt.state == AGR_NMT_BOOT_UP)  ← ❌ false (OPERATIONAL)
  ↓
Heartbeat 전송 (Boot-up 재전송 중단!) ✅
```

---

## 📊 **NMT 인스턴스 역할 분리**

| 인스턴스 | 역할 | 업데이트 주체 | 용도 |
|---------|------|-------------|------|
| **`s_imu_hub_nmt`** | Slave 자신의 NMT 상태 | NMT Command (0x000) | Master의 명령으로 상태 전환 |
| **`s_imu_hub_pnp_inst.config.nmt`** | Master (XM)의 NMT 상태 | Master Heartbeat (0x701) | Master 연결 상태 모니터링 |

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Master Heartbeat 처리** | `s_imu_hub_nmt`로 처리 ❌ | `config.nmt`로 처리 ✅ |
| **`config.nmt.state`** | 계속 `BOOT_UP` ❌ | `OPERATIONAL` ✅ |
| **`is_master_connected`** | 항상 `false` ❌ | `true` ✅ |
| **Boot-up 재전송** | 1초마다 계속 ❌ | 초기화 시 1번만 ✅ |
| **Heartbeat 전송** | 안됨 ❌ | 1초마다 정상 전송 ✅ |

---

## 🔧 **수정된 파일**

1. **xm_drv.c** (IMU Hub Module - Slave)
   - `XM_Drv_ProcessCANMessage()` 수정
   - Master Heartbeat를 `s_imu_hub_pnp_inst.config.nmt`로 처리

2. **imu_hub_drv.c** (XM - Master)
   - 동일한 패턴으로 수정 (Master는 Slave Heartbeat 처리)

---

**결론**: Master의 Heartbeat를 올바른 NMT 인스턴스로 처리하여 Boot-up 무한 루프가 해결됩니다! ✅
