# AGR_NMT Boot-up 처리 버그 수정

**작성일**: 2025-12-08  
**수정일**: 2025-12-08 (Bug #2 추가)  
**문제**: Boot-up 메시지 수신 시 상태 전이 로직이 실행되지 않음
**원인**: 
- Bug #1: 상태를 먼저 변경한 후 `AGR_NMT_UpdateActivity` 호출
- Bug #2: Boot-up 수신 시 현재 상태가 PRE_OP/OPERATIONAL이면 전이 안 됨
**해결**: Boot-up 수신 시 상태를 BOOT_UP으로 리셋 후 `AGR_NMT_UpdateActivity` 호출

---

## 🚨 치명적인 버그 발견!

### **증상**
- XM(Master)가 IMU Hub(Slave)의 Boot-up 메시지를 수신
- `AGR_NMT_ProcessMessage` 진입 확인
- 그러나 **Pre-Op 단계가 진행되지 않음**
- SDO 전송이 시작되지 않음

---

## 🔍 버그 원인 분석

### **잘못된 로직 흐름 (Before)**

```c
/* AGR_NMT_ProcessMessage - Boot-up 처리 */
if (nmt_state_byte == 0x00) {
    AGR_NMT_State_t old_state = inst->state;  /* Step 1: old_state = BOOT_UP */
    inst->state = AGR_NMT_PRE_OPERATIONAL;    /* Step 2: ❌ 여기서 먼저 PRE_OP로 변경! */
    AGR_NMT_UpdateActivity(inst, current_ms);  /* Step 3: ❌ 호출 시 이미 PRE_OP 상태 */
    
    if (old_state != AGR_NMT_PRE_OPERATIONAL) {
        _NotifyStateChange(inst, old_state, AGR_NMT_PRE_OPERATIONAL);
    }
    return 0;
}
```

그러면 `AGR_NMT_UpdateActivity` 내부에서:

```c
void AGR_NMT_UpdateActivity(AGR_NMT_Inst_t* inst, uint32_t current_ms)
{
    AGR_NMT_State_t old_state = inst->state;  /* ❌ inst->state는 이미 PRE_OP */
    
    inst->last_activity_ms = current_ms;      /* ✅ Activity는 갱신됨 */
    
    /* 상태 전이 로직 (모드에 따라 다름) */
    switch (old_state) {
        case AGR_NMT_BOOT_UP:  /* ❌ 여기에 절대 진입하지 못함! */
            if (inst->mode == AGR_NMT_MODE_SIMPLE) {
                inst->state = AGR_NMT_OPERATIONAL;  /* 실행 안 됨 */
            } else {
                inst->state = AGR_NMT_PRE_OPERATIONAL;  /* 실행 안 됨 */
            }
            _NotifyStateChange(inst, old_state, inst->state);  /* 실행 안 됨 */
            break;
            
        case AGR_NMT_PRE_OPERATIONAL:  /* ✅ 여기로 진입 */
            /* 상태 유지 (활동 시간만 갱신됨) */
            break;  /* ❌ 아무것도 하지 않음 */
    }
}
```

**결과**:
- ❌ `mode`에 따른 상태 전이가 실행되지 않음
- ❌ `_NotifyStateChange` 콜백이 호출되지 않음
- ❌ Pre-Op 로직이 시작되지 않음
- ❌ SDO 전송이 시작되지 않음

---

## 🎯 왜 이런 버그가 발생했나?

### **설계 의도 vs 실제 구현**

**설계 의도**:
```
Boot-up 수신 → AGR_NMT_UpdateActivity 호출 → mode 확인 → 상태 전이 → 콜백
```

**실제 구현**:
```
Boot-up 수신 → 상태 먼저 변경 → AGR_NMT_UpdateActivity 호출 → ❌ switch-case 불일치
```

**근본 원인**:
- `AGR_NMT_ProcessMessage`에서 Boot-up/Heartbeat 메시지 수신 시 **상태를 직접 변경**
- 그 후 `AGR_NMT_UpdateActivity`를 호출하여 "상태 전이 로직"을 실행하려고 시도
- 그러나 `UpdateActivity` 내부의 `switch (old_state)`는 **변경 전 상태**를 기대
- **상태가 이미 변경되어 switch-case가 매칭되지 않음**

---

## ✅ 해결 방법 (Bug #1)

### **수정된 로직 (After)**

```c
/* AGR_NMT_ProcessMessage - Boot-up 처리 */
if (nmt_state_byte == 0x00) {
    /* ✅ Boot-up 감지: UpdateActivity에서 상태 전이 처리 */
    /* 현재 state는 BOOT_UP 또는 STOPPED일 것 */
    AGR_NMT_UpdateActivity(inst, current_ms);  /* Activity 갱신 + 상태 전이 */
    return 0;  /* 처리 완료 */
}
```

**변경 사항**:
1. ❌ **삭제**: `inst->state = AGR_NMT_PRE_OPERATIONAL;` (상태를 먼저 변경하지 않음)
2. ❌ **삭제**: `if (old_state != AGR_NMT_PRE_OPERATIONAL) { _NotifyStateChange(...); }` (중복 제거)
3. ✅ **유지**: `AGR_NMT_UpdateActivity`가 모든 상태 전이를 담당

---

## 🚨 두 번째 버그 발견! (Bug #2)

### **문제: Boot-up 수신 시 현재 상태가 PRE_OP/OPERATIONAL이면?**

Bug #1을 수정했지만, 또 다른 문제가 있었습니다:

**시나리오**:
1. XM과 IMU Hub가 연결되어 OPERATIONAL 상태
2. IMU Hub가 디버깅 모드로 리셋됨
3. IMU Hub가 Boot-up 재전송 (0x708, [0x00])
4. XM의 NMT 상태는? → **아직 PRE_OPERATIONAL 또는 OPERATIONAL!**
5. `AGR_NMT_UpdateActivity` 호출 시:
   ```c
   switch (inst->state) {  /* PRE_OPERATIONAL 또는 OPERATIONAL */
       case AGR_NMT_BOOT_UP:   /* ❌ 여기에 진입 못함 */
       case AGR_NMT_STOPPED:   /* ❌ 여기에 진입 못함 */
       case AGR_NMT_PRE_OPERATIONAL:  /* ✅ 여기로 진입 */
       case AGR_NMT_OPERATIONAL:
           /* 상태 유지 (아무것도 안 함) */
           break;  /* ❌ Pre-Op 로직이 시작되지 않음! */
   }
   ```

**결과**: 타임아웃 발생 전까지 재연결이 안 됨!

---

## ✅ 최종 해결 방법 (Bug #2 수정)

### **수정된 로직 (Final)**

```c
/* AGR_NMT_ProcessMessage - Boot-up 처리 */
if (nmt_state_byte == 0x00) {
    /* ✅ Boot-up 감지: 상태를 BOOT_UP으로 리셋 */
    /* 
     * [중요] Boot-up은 "재시작" 신호이므로 현재 상태와 무관하게 
     * BOOT_UP으로 리셋한 후 UpdateActivity에서 전이 처리
     */
    AGR_NMT_State_t old_state = inst->state;
    inst->state = AGR_NMT_BOOT_UP;
    AGR_NMT_UpdateActivity(inst, current_ms);  /* Activity 갱신 + 상태 전이 */
    
    /* 상태가 변경되었다면 콜백 호출 */
    if (old_state != AGR_NMT_BOOT_UP) {
        _NotifyStateChange(inst, old_state, AGR_NMT_BOOT_UP);
    }
    return 0;  /* 처리 완료 */
}
```

**변경 사항**:
1. ✅ **추가**: `AGR_NMT_State_t old_state = inst->state;` (이전 상태 저장)
2. ✅ **추가**: `inst->state = AGR_NMT_BOOT_UP;` (무조건 BOOT_UP으로 리셋)
3. ✅ **유지**: `AGR_NMT_UpdateActivity(inst, current_ms);` (상태 전이)
4. ✅ **추가**: `_NotifyStateChange(inst, old_state, AGR_NMT_BOOT_UP);` (상태 변경 알림)

---

### **올바른 흐름 (Final - Bug #2 수정 후)**

```
1. Boot-up 메시지 수신 (현재 상태와 무관)
   └─ old_state = inst->state  /* BOOT_UP, STOPPED, PRE_OP, OPERATIONAL 모두 가능 */
   └─ inst->state = AGR_NMT_BOOT_UP  /* ✅ 무조건 BOOT_UP으로 리셋! */
   └─ _NotifyStateChange(old_state, BOOT_UP)  /* ✅ BOOT_UP 진입 알림 */

2. AGR_NMT_UpdateActivity(inst, current_ms) 호출
   └─ old_state = inst->state  /* BOOT_UP */
   └─ switch (old_state):
       └─ case AGR_NMT_BOOT_UP:  /* ✅ 여기로 진입! */
           ├─ mode == SIMPLE → inst->state = OPERATIONAL
           └─ mode == FULL → inst->state = PRE_OPERATIONAL
           └─ _NotifyStateChange(BOOT_UP, PRE_OP)  /* ✅ PRE_OP 진입 알림 */

3. Pre-Op 로직 시작 ✅
   └─ _PnP_OnConnected 콜백 호출
   └─ s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A
   └─ SDO 전송 시작 ✅
```

**중요한 차이점**:
- Bug #1 수정: `UpdateActivity` 전에 상태를 변경하지 않음
- **Bug #2 수정**: Boot-up 수신 시 **무조건 BOOT_UP으로 리셋** → 항상 전이 로직 실행

---

## 📊 수정 효과

### **Before (버그 상태)**

| 단계 | 동작 | 결과 |
|------|------|------|
| 1. Boot-up 수신 | `inst->state = PRE_OP` 먼저 변경 | ✅ |
| 2. UpdateActivity | `switch (PRE_OP)` → 아무것도 안 함 | ❌ |
| 3. 상태 전이 | 실행 안 됨 | ❌ |
| 4. 콜백 호출 | `_NotifyStateChange` 실행 안 됨 | ❌ |
| 5. Pre-Op 로직 | `_PnP_OnConnected` 호출 안 됨 | ❌ |
| 6. SDO 전송 | 시작 안 됨 | ❌ |

---

### **After (수정 후)**

| 단계 | 동작 | 결과 |
|------|------|------|
| 1. Boot-up 수신 | 상태 유지 (BOOT_UP 또는 STOPPED) | ✅ |
| 2. UpdateActivity | `switch (BOOT_UP)` → 전이 로직 실행 | ✅ |
| 3. 상태 전이 | `inst->state = PRE_OP` 변경 | ✅ |
| 4. 콜백 호출 | `_NotifyStateChange` 실행 | ✅ |
| 5. Pre-Op 로직 | `_PnP_OnConnected` 호출 | ✅ |
| 6. SDO 전송 | 시작됨 | ✅ |

---

## 🔍 Heartbeat 처리는 왜 문제없나?

Heartbeat 처리 로직도 동일한 패턴이지만, **이미 연결된 상태**에서 수신되므로 문제가 덜 심각합니다:

```c
/* Heartbeat (0x04: STOPPED, 0x05: OPERATIONAL, 0x7F: PRE_OPERATIONAL) */
else if (nmt_state_byte == AGR_NMT_STOPPED || 
         nmt_state_byte == AGR_NMT_OPERATIONAL || 
         nmt_state_byte == AGR_NMT_PRE_OPERATIONAL) {
    AGR_NMT_State_t old_state = inst->state;
    AGR_NMT_State_t new_state = (AGR_NMT_State_t)nmt_state_byte;
    
    inst->state = new_state;  /* ⚠️ 동일한 패턴 */
    AGR_NMT_UpdateActivity(inst, current_ms);
    
    if (old_state != new_state) {
        _NotifyStateChange(inst, old_state, new_state);
    }
    return 0;
}
```

**Heartbeat는 상태 전이가 필요 없음**:
- Heartbeat는 **이미 연결된 상태**를 확인하는 용도
- `AGR_NMT_UpdateActivity` 내부의 `switch (PRE_OP)` 또는 `switch (OPERATIONAL)`은 "상태 유지" 케이스
- 따라서 큰 문제가 없음

**그러나 일관성을 위해 Heartbeat도 수정하는 것이 좋습니다** (향후 개선 사항).

---

## 🎯 `mode`는 Legacy인가?

**아닙니다!** `mode`는 매우 중요합니다:

```c
/* AGR_NMT_UpdateActivity - Boot-up 상태 전이 */
case AGR_NMT_BOOT_UP:
    if (inst->mode == AGR_NMT_MODE_SIMPLE) {
        /* Simple Mode: BOOT_UP → OPERATIONAL 직행 */
        /* UART 센서 등 데이터 수신 즉시 연결됨으로 간주 */
        inst->state = AGR_NMT_OPERATIONAL;
    } else {
        /* Full Mode: BOOT_UP → PRE_OPERATIONAL */
        /* CANopen 호환: 설정 단계를 거쳐야 함 */
        inst->state = AGR_NMT_PRE_OPERATIONAL;
    }
    _NotifyStateChange(inst, old_state, inst->state);
    break;
```

**용도**:
- ✅ **SIMPLE Mode**: UART 센서 등 단순 연결 (BOOT_UP → OPERATIONAL 직행)
- ✅ **FULL Mode**: CANopen 기반 PnP (BOOT_UP → PRE_OP → SDO 설정 → OPERATIONAL)

**XM의 IMU Hub는 FULL Mode를 사용해야 합니다!**

---

## 📋 수정 파일

| 파일 | 수정 내용 | 상태 |
|------|-----------|------|
| `agr_nmt.c` (XM) | Boot-up 처리 시 상태를 먼저 변경하지 않도록 수정 | ✅ |

---

## 🚀 기대 효과

### **Boot-up 시퀀스 (수정 후)**

```
[IMU Hub - Slave]
1. Startup → Boot-up 전송 (0x708, [0x00])

[XM - Master]
2. Boot-up 수신 → AGR_NMT_ProcessMessage
3. AGR_NMT_UpdateActivity 호출
   └─ switch (BOOT_UP) → inst->state = PRE_OPERATIONAL
   └─ _NotifyStateChange(BOOT_UP → PRE_OP)
       └─ _PnP_OnConnected 콜백 호출 ✅
           └─ s_imu_hub_inst.pre_op_state = SEND_PDO_MAP_A
           
4. ImuHub_Drv_RunPeriodic 실행
   └─ Pre-Op Timeout 체크
   └─ Step Array 실행 ✅
       └─ SEND_PDO_MAP_A → SDO 전송 ✅
       
5. Pre-Op 완료 → OPERATIONAL 전환 ✅
```

---

**결론**: Boot-up 수신 시 `AGR_NMT_UpdateActivity`가 상태 전이를 담당하도록 수정하여 PnP 로직이 정상 동작하도록 했습니다! ✅
