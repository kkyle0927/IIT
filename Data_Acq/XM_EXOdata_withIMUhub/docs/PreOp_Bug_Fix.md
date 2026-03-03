# Pre-Op State Machine Bug 수정

**작성일**: 2025-12-08  
**문제**: XM의 `pre_op_state`가 계속 왔다갔다하는 현상

---

## 🐛 **발견된 버그**

### **Bug 1: Timeout 재시도 로직 오류**

#### **Before (잘못된 코드)**
```c
if (current_ms - s_imu_hub_inst.last_sdo_tx_time > 1000) {  /* 1s Timeout */
    if (s_imu_hub_inst.sdo_retry_count < 3) {
        s_imu_hub_inst.sdo_retry_count++;
        
        /* ❌ 잘못된 전환: WAIT → 다음 단계 SEND */
        if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;  // ❌ B로 점프!
        } else if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ;  // ❌ 다음으로 점프!
        }
    }
}
```

**문제점**:
- Timeout 발생 시 **같은 단계를 재시도**해야 하는데, **다음 단계로 잘못 점프**함
- `WAIT_A` → `SEND_B`로 가면, A 단계는 실패했는데 B로 진행됨
- 이로 인해 상태가 **왔다갔다함**

---

#### **After (수정된 코드)**
```c
if (current_ms - s_imu_hub_inst.last_sdo_tx_time > 5000) {  /* ✅ 5s Timeout */
    if (s_imu_hub_inst.sdo_retry_count < 3) {
        s_imu_hub_inst.sdo_retry_count++;
        
        /* ✅ 올바른 전환: WAIT → 같은 단계 SEND (재시도) */
        if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;  // ✅ A 재시도
        } else if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B) {
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;  // ✅ B 재시도
        } else if (s_imu_hub_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
            /* IMU Mask는 Optional → Skip */
            s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_NMT_START;
            s_imu_hub_inst.sdo_retry_count = 0;
        }
    }
}
```

**수정 내용**:
1. ✅ **Timeout 5초로 변경** (Step Array의 5000ms에 맞춤)
2. ✅ **재시도 로직 수정**: `WAIT_A` → `SEND_A` (같은 단계 재시도)
3. ✅ **IMU Mask는 Optional**: Timeout 시 Skip하고 NMT START로 진행

---

## 📊 **Pre-Op State Machine 흐름**

### **정상 흐름**

```
[1] IDLE
      ↓ (Boot-up 수신)
[2] SEND_PDO_MAP_A
      ↓ (SDO Write 전송)
[3] WAIT_PDO_MAP_A
      ↓ (SDO Response 수신)
[4] SEND_PDO_MAP_B
      ↓ (SDO Write 전송)
[5] WAIT_PDO_MAP_B
      ↓ (SDO Response 수신)
[6] SEND_IMU_MASK_REQ
      ↓ (SDO Read 전송)
[7] WAIT_IMU_MASK_RSP
      ↓ (SDO Response 수신)
[8] SEND_NMT_START
      ↓ (NMT START 전송)
[9] COMPLETE
      ↓ (Slave OPERATIONAL Heartbeat 수신)
      ✅ 연결 완료!
```

---

### **Timeout 재시도 흐름 (수정 후)**

```
[3] WAIT_PDO_MAP_A
      ↓ (5초 Timeout)
[2] SEND_PDO_MAP_A  ✅ 같은 단계 재시도!
      ↓ (SDO Write 재전송)
[3] WAIT_PDO_MAP_A
      ↓ (SDO Response 수신)
[4] SEND_PDO_MAP_B  ✅ 다음 단계로 정상 진행
```

---

### **버그로 인한 잘못된 흐름 (수정 전)**

```
[3] WAIT_PDO_MAP_A
      ↓ (1초 Timeout)
[4] SEND_PDO_MAP_B  ❌ 다음 단계로 잘못 점프!
      ↓ (SDO Write 전송)
[5] WAIT_PDO_MAP_B
      ↓ (1초 Timeout)
[6] SEND_IMU_MASK_REQ  ❌ 또 다음으로!
      ↓ (계속 왔다갔다...)
```

---

## 🔄 **Step Array 패턴**

```c
static const ImuHub_PreOpStep_t s_pre_op_steps[] = {
    /* send_state,                wait_state,                action,               timeout, description */
    { IMUHUB_PRE_OP_SEND_PDO_MAP_A,    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,    _Step_SendPdoMapA,     5000,    "TPDO1 Mapping" },
    { IMUHUB_PRE_OP_SEND_PDO_MAP_B,    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,    _Step_SendPdoMapB,     5000,    "TPDO2 Mapping" },
    { IMUHUB_PRE_OP_SEND_IMU_MASK_REQ, IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP, _Step_SendImuMaskReq,  5000,    "IMU Mask Read" },
    { IMUHUB_PRE_OP_SEND_NMT_START,    IMUHUB_PRE_OP_COMPLETE,          _Step_SendNmtStart,    5000,    "NMT START" },
};
```

**Step Array의 장점**:
- ✅ **확장성**: 새로운 단계 추가가 쉬움
- ✅ **가독성**: 선언적 시퀀스 정의
- ✅ **유지보수**: Timeout 등 설정 일괄 관리

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Timeout 시간** | 1초 (짧음) | 5초 (Step Array와 일치) |
| **재시도 로직** | ❌ 다음 단계로 점프 | ✅ 같은 단계 재시도 |
| **상태 안정성** | ❌ 왔다갔다함 | ✅ 순차적 진행 |
| **Pre-Op 완료** | ⚠️ 불안정 | ✅ 안정적 |

---

## 📝 **Live Expression으로 확인하기**

```c
/* Master (XM) - imu_hub_drv.c */
s_imu_hub_inst.pre_op_state        /* Pre-Op 상태 (0~8) */
s_imu_hub_inst.sdo_retry_count     /* 재시도 횟수 (0~3) */
s_imu_hub_inst.last_sdo_tx_time    /* 마지막 SDO 전송 시간 */
s_imu_hub_inst.master_pnp->devices[0].nmt.state  /* Slave NMT 상태 */
```

**정상 시퀀스**:
1. `pre_op_state = 1` (SEND_PDO_MAP_A)
2. `pre_op_state = 2` (WAIT_PDO_MAP_A)
3. `pre_op_state = 3` (SEND_PDO_MAP_B)
4. `pre_op_state = 4` (WAIT_PDO_MAP_B)
5. `pre_op_state = 5` (SEND_IMU_MASK_REQ)
6. `pre_op_state = 6` (WAIT_IMU_MASK_RSP)
7. `pre_op_state = 7` (SEND_NMT_START)
8. `pre_op_state = 8` (COMPLETE) ✅
9. `nmt.state = 5` (OPERATIONAL) ✅

---

**결론**: Timeout 재시도 로직 버그를 수정하여 Pre-Op 상태가 순차적으로 안정적으로 진행됩니다! ✅
