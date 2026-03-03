# Heartbeat Timeout 버그 수정

**작성일**: 2025-12-08  
**문제**: OPERATIONAL 상태에서 Heartbeat 수신 시 시간 갱신 안됨 → 3초 후 Timeout

---

## 🐛 **Bug 4: Heartbeat 수신 시 상태 변경이 없으면 시간 갱신 안됨**

### **타이밍 분석**

| 모듈 | RunPeriodic 주기 | 실행 방식 | Heartbeat 전송 주기 | Heartbeat Timeout |
|------|-----------------|----------|-------------------|------------------|
| **XM (Master)** | **100ms** | RTOS Task (`PnPManager_Task`) | 1초 | 3초 |
| **IMU Hub (Slave)** | **1ms** | Bare-metal (`core_process.c`) | 1초 | 3초 |

---

### **문제 시나리오**

```
[0초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
[XM] Heartbeat 수신
  ↓
  AGR_NMT_ProcessMessage(&dev->nmt, ...)
  ↓
  old_state = OPERATIONAL, new_state = OPERATIONAL
  ↓
  if (old_state != new_state)  ← ❌ false! 콜백 호출 안됨!
  ↓
  last_master_hb_time 갱신 안됨 ❌

[1초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
  (동일하게 콜백 호출 안됨)

[2초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
  (동일하게 콜백 호출 안됨)

[3초] XM: Timeout 체크
  ↓
  current_time - last_master_hb_time > 3000ms ✅
  ↓
  is_master_connected = false ❌
  ↓
  Boot-up 재전송 시작! ❌
```

---

### **잘못된 코드 (Before)**

```c
/* agr_nmt.c - AGR_NMT_ProcessMessage() */

/* Heartbeat 수신 */
else if (nmt_state_byte == AGR_NMT_OPERATIONAL || ...) {
    AGR_NMT_State_t old_state = inst->state;  // OPERATIONAL
    AGR_NMT_State_t new_state = (AGR_NMT_State_t)nmt_state_byte;  // OPERATIONAL
    
    inst->state = new_state;
    AGR_NMT_UpdateActivity(inst, current_ms);  /* ✅ NMT 레벨 시간 갱신 */
    
    if (old_state != new_state) {  /* ❌ false! */
        _NotifyStateChange(inst, old_state, new_state);  /* 호출 안됨! */
    }
    return 0;
}
```

**결과:**
- `AGR_NMT_UpdateActivity`: ✅ `inst->last_activity_ms` 갱신됨 (NMT 레벨)
- `_NotifyStateChange`: ❌ 호출 안됨
- `_OnSlaveNmtStateChanged`: ❌ 호출 안됨
- `last_master_hb_time`: ❌ 갱신 안됨 (PnP 레벨)

---

### **수정된 코드 (After)**

```c
/* agr_nmt.c - AGR_NMT_ProcessMessage() */

/* Heartbeat 수신 */
else if (nmt_state_byte == AGR_NMT_OPERATIONAL || ...) {
    AGR_NMT_State_t old_state = inst->state;
    AGR_NMT_State_t new_state = (AGR_NMT_State_t)nmt_state_byte;
    
    inst->state = new_state;
    AGR_NMT_UpdateActivity(inst, current_ms);  /* ✅ NMT 레벨 시간 갱신 */
    
    /* ✅ Heartbeat 수신 시 항상 콜백 호출 (상태 변경 여부 무관) */
    /* 이유: Slave가 Master Heartbeat를 받을 때 시간 갱신 필요 */
    _NotifyStateChange(inst, old_state, new_state);
    
    return 0;
}
```

**결과:**
- `AGR_NMT_UpdateActivity`: ✅ `inst->last_activity_ms` 갱신됨
- `_NotifyStateChange`: ✅ 항상 호출됨
- `_OnSlaveNmtStateChanged`: ✅ 항상 호출됨
- `last_master_hb_time`: ✅ 매 Heartbeat마다 갱신됨!

---

### **정상 시나리오 (After)**

```
[0초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
[XM] Heartbeat 수신
  ↓
  AGR_NMT_ProcessMessage(&dev->nmt, ...)
  ↓
  AGR_NMT_UpdateActivity() → last_activity_ms 갱신 ✅
  ↓
  _NotifyStateChange() → 항상 호출 ✅
  ↓
  _OnSlaveNmtStateChanged()
    ↓
    last_master_hb_time = get_tick() ✅
    is_master_connected = true ✅

[1초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
  last_master_hb_time 갱신 ✅

[2초] IMU Hub → XM: Heartbeat (0x05, OPERATIONAL)
  ↓
  last_master_hb_time 갱신 ✅

[3초] XM: Timeout 체크
  ↓
  current_time - last_master_hb_time = 1000ms < 3000ms ✅
  ↓
  연결 유지! ✅
```

---

## 📊 **Timeout 분석 전체 요약**

### **XM (Master) - IMU Hub 모니터링**

| 항목 | 값 | 비고 |
|------|------|------|
| **RunPeriodic 주기** | 100ms | RTOS Task |
| **Heartbeat 수신** | 매 1초 | IMU Hub가 전송 |
| **시간 갱신** | 매 1초 | `_OnNmtStateChanged` 콜백 |
| **Timeout 체크** | 매 100ms | `_CheckTimeouts` |
| **Timeout 기준** | 3초 | `heartbeat_timeout = 3000ms` |
| **안전 마진** | ✅ **2초** | Timeout 전 2번 더 받을 수 있음 |

---

### **IMU Hub (Slave) - XM 모니터링**

| 항목 | 값 | 비고 |
|------|------|------|
| **RunPeriodic 주기** | 1ms | Bare-metal |
| **Heartbeat 수신** | 매 1초 | XM이 전송 |
| **시간 갱신** | 매 1초 | `_OnSlaveNmtStateChanged` 콜백 ✅ |
| **Timeout 체크** | 매 1ms | `_CheckTimeouts` |
| **Timeout 기준** | 3초 | `heartbeat_timeout = 3000ms` |
| **안전 마진** | ✅ **2초** | Timeout 전 2번 더 받을 수 있음 |

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Heartbeat 수신 시 콜백 호출** | 상태 변경 시만 ❌ | 항상 호출 ✅ |
| **`last_master_hb_time` 갱신** | 최초 1번만 ❌ | 매 1초마다 ✅ |
| **OPERATIONAL 유지 시 Timeout** | 3초 후 발생 ❌ | 발생 안함 ✅ |
| **Boot-up 재전송** | 3초마다 반복 ❌ | 초기화 시 1번만 ✅ |
| **연결 안정성** | 불안정 ❌ | 안정적 ✅ |

---

## 🔧 **수정된 파일**

1. **agr_nmt.c** (IMU Hub Module & XM 공통)
   - `AGR_NMT_ProcessMessage()` 수정
   - Heartbeat 수신 시 항상 `_NotifyStateChange` 호출

---

**결론**: Heartbeat 수신 시 상태 변경 여부와 무관하게 항상 콜백을 호출하여 시간 갱신이 정상적으로 이루어집니다! ✅

---

## 📝 **추가 개선 사항 (선택)**

현재는 정상 동작하지만, 성능 최적화를 위해 다음을 고려할 수 있습니다:

### **옵션 1: 콜백에서 중복 처리 방지**

```c
static void _OnSlaveNmtStateChanged(AGR_NMT_State_t old_state, AGR_NMT_State_t new_state, void* user_ctx)
{
    AGR_PnP_Inst_t* inst = (AGR_PnP_Inst_t*)user_ctx;
    if (inst == NULL || inst->config.role != AGR_PNP_ROLE_SLAVE) {
        return;
    }
    
    /* ✅ 항상 시간 갱신 (Heartbeat 수신) */
    inst->last_master_hb_time = inst->get_tick();
    inst->is_master_connected = true;
    
    /* 상태 변경 시에만 추가 처리 (Optional) */
    if (old_state != new_state) {
        /* 상태 변경 로그, User 콜백 호출 등 */
    }
}
```

**현재 코드는 이미 최적화되어 있어 추가 수정 불필요합니다!** ✅
