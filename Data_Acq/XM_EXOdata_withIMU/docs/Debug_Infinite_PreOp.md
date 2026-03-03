# Debug: XM 무한 PRE_OP 문제

**작성일**: 2025-12-08  
**증상**: IMU Hub의 `my_nmt`가 OPERATIONAL인데 XM이 무한 PRE_OP 상태

---

## 🐛 **증상**

```
IMU Hub (Slave):
  s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL (0x05) ✅
  
XM (Master):
  s_imu_hub_inst.master_pnp->devices[0].nmt.state = AGR_NMT_PRE_OPERATIONAL (0x7F) ❌
  s_imu_hub_inst.pre_op_state = 계속 진행 중 (1~7 반복)
```

**문제:**
- IMU Hub가 OPERATIONAL 상태인데
- XM은 IMU Hub가 PRE_OPERATIONAL이라고 생각함
- NMT 상태 동기화 실패!

---

## 🔍 **가능한 원인 체크리스트**

### **1. IMU Hub가 Heartbeat를 전송하는가?**

**확인사항:**

```c
/* IMU Hub - Live Expression */
s_imu_hub_pnp_inst.my_nmt  // NULL이 아니어야 함! ✅
s_imu_hub_pnp_inst.my_nmt->state  // AGR_NMT_OPERATIONAL (0x05) ✅
s_imu_hub_pnp_inst.last_hb_sent_time  // 계속 증가? ✅
```

**예상 동작:**

```c
/* agr_pnp.c - AGR_PnP_SendHeartbeat() */
if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
    nmt_state = (inst->my_nmt != NULL) 
                ? (uint8_t)inst->my_nmt->state  // ✅ OPERATIONAL (0x05)
                : (uint8_t)inst->config.nmt.state;  // Fallback
}

/* 전송: CAN ID 0x708, Data [0x05] */
```

**확인방법:**
- CAN 버스 모니터: `0x708, [0x05]` 메시지 확인
- IMU Hub Live Expression: `s_imu_hub_pnp_inst.my_nmt != NULL`

---

### **2. XM이 Heartbeat를 수신하는가?**

**확인사항:**

```c
/* XM - Live Expression */
s_imu_hub_inst.master_pnp->devices[0].nmt.last_activity_ms  // 계속 증가? ✅
```

**예상 동작:**

```c
/* XM - imu_hub_drv.c - ImuHub_Drv_ProcessCANMessage() */
if (fnc_code == 0x700) {  // Heartbeat
    AGR_PnP_ProcessMessage(master_pnp, can_id, data, len);
      ↓
    AGR_NMT_ProcessMessage(&devices[0].nmt, 0x708, data, len);
      ↓
    devices[0].nmt.state = data[0];  // ✅ 0x05 (OPERATIONAL)로 업데이트!
    devices[0].nmt.last_activity_ms = current_ms;
}
```

**확인방법:**
- XM Live Expression: `devices[0].nmt.last_activity_ms` 증가 확인
- CAN 버스 모니터: `0x708` 메시지 확인

---

### **3. devices[0].nmt가 제대로 초기화되었는가?**

**확인사항:**

```c
/* XM - Live Expression */
s_imu_hub_inst.device_index  // 0이어야 함 ✅
s_imu_hub_inst.master_pnp  // NULL이 아니어야 함 ✅
s_imu_hub_inst.master_pnp->devices[0].node_id  // 0x08 (IMU Hub) ✅
s_imu_hub_inst.master_pnp->devices[0].nmt.node_id  // 0x08 ✅
```

**예상 초기화:**

```c
/* imu_hub_drv.c - ImuHub_Drv_Init() */
AGR_PnP_Device_t imu_hub_device = {
    .name = "IMU Hub Module",
    .node_id = AGR_NODE_ID_IMU_HUB,  // 0x08
    .heartbeat_timeout = 5000,
    .callbacks = { ... }
};

int dev_idx = AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device);
s_imu_hub_inst.device_index = dev_idx;
s_imu_hub_inst.master_pnp = master_pnp;
```

---

### **4. Pre-Op 시퀀스가 NMT START를 전송했는가?**

**확인사항:**

```c
/* XM - Live Expression */
s_imu_hub_inst.pre_op_state  // COMPLETE (8)이어야 함 ✅
```

**예상 흐름:**

```
Pre-Op 시작 (Boot-up 수신)
  ↓
PDO Mapping A (SEND → WAIT → ACK)
  pre_op_state = 1 → 2 → 3
  ↓
PDO Mapping B (SEND → WAIT → ACK)
  pre_op_state = 3 → 4 → 5
  ↓
IMU Mask Read (SEND → WAIT → ACK)
  pre_op_state = 5 → 6 → 7
  ↓
NMT START 전송
  pre_op_state = 7
  CAN: 0x000, [0x01, 0x08]  ✅ START 전송!
  ↓
Pre-Op 완료
  pre_op_state = 8 (COMPLETE)
```

**확인방법:**
- XM Live Expression: `s_imu_hub_inst.pre_op_state`
- CAN 버스 모니터: `0x000, [0x01, 0x08]` 메시지 확인

---

### **5. IMU Hub가 NMT START를 받았는가?**

**확인사항:**

```c
/* IMU Hub - Live Expression */
s_imu_hub_nmt.state  // AGR_NMT_OPERATIONAL (0x05) ✅
s_imu_hub_nmt.node_id  // 0x08 ✅
```

**예상 동작:**

```c
/* IMU Hub - xm_drv.c - XM_Drv_ProcessCANMessage() */
if (can_id == 0x000) {  // NMT Command
    AGR_NMT_ProcessMessage(&s_imu_hub_nmt, 0x000, data, len);
      ↓
    if (data[0] == 0x01 && data[1] == 0x08) {  // START to Node 0x08
        s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL;  ✅
    }
}
```

---

## 🔧 **디버깅 순서**

### **Step 1: Heartbeat 전송 확인 (IMU Hub)**

```c
/* IMU Hub - xm_drv.c - XM_Drv_Init() */

/* ✅ 이 줄이 있는지 확인! */
AGR_PnP_SetMyNmt(&s_imu_hub_pnp_inst, &s_imu_hub_nmt);
```

**Live Expression:**
```c
s_imu_hub_pnp_inst.my_nmt  // ⚠️ NULL이면 문제!
```

**만약 NULL이면:**
- `AGR_PnP_SetMyNmt()` 호출이 누락됨
- 빌드 오류가 있었을 가능성

---

### **Step 2: Heartbeat 데이터 확인 (CAN 버스)**

**CAN 버스 모니터에서 확인:**

```
# IMU Hub Heartbeat
ID: 0x708
Data: [0x??]  ← 이 값이 뭔가?

예상값:
- 0x00: BOOT_UP
- 0x04: STOPPED
- 0x05: OPERATIONAL ✅ 이게 나와야 함!
- 0x7F: PRE_OPERATIONAL

만약 0x7F (PRE_OP)가 나오면:
  → my_nmt가 NULL이거나
  → config.nmt.state를 사용 중 (Master 추적용!)
```

---

### **Step 3: XM 수신 확인**

**Live Expression:**

```c
/* XM */
s_imu_hub_inst.master_pnp->devices[0].nmt.last_activity_ms  // 증가하는지?
s_imu_hub_inst.master_pnp->devices[0].nmt.state  // 0x05 (OP)인지 0x7F (PRE_OP)인지?
```

**만약 `last_activity_ms`가 증가하지 않으면:**
- XM이 Heartbeat를 수신하지 못함
- `ImuHub_Drv_ProcessCANMessage()`가 호출되지 않음
- CAN 버스 연결 문제

**만약 `last_activity_ms`는 증가하는데 `state`가 PRE_OP면:**
- Heartbeat 데이터가 0x7F (PRE_OP)
- IMU Hub가 잘못된 상태 전송 중

---

### **Step 4: Pre-Op 완료 확인**

**Live Expression:**

```c
/* XM */
s_imu_hub_inst.pre_op_state  // 8 (COMPLETE)인지?

만약 1~7 사이를 계속 왔다갔다하면:
  → SDO Response를 받지 못함
  → Timeout 발생
  → 재시도 반복

확인:
  s_imu_hub_inst.sdo_retry_count  // 계속 증가?
  s_imu_hub_inst.last_sdo_tx_time  // 5초마다 갱신?
```

---

## 🎯 **가장 가능성 높은 원인**

### **원인 1: `my_nmt`가 NULL**

```c
/* IMU Hub - xm_drv.c - XM_Drv_Init() */

/* ❌ 이 줄이 누락되었을 가능성! */
AGR_PnP_SetMyNmt(&s_imu_hub_pnp_inst, &s_imu_hub_nmt);
```

**결과:**
- `my_nmt == NULL`
- `AGR_PnP_SendHeartbeat()`에서 Fallback 사용
- `config.nmt.state` 전송 (Master 추적용!)
- Master 추적용 NMT는 항상 PRE_OP 또는 BOOT_UP 상태
- XM이 PRE_OP로 인식!

**해결:**
- `xm_drv.c` 빌드 확인
- `AGR_PnP_SetMyNmt()` 호출 확인

---

### **원인 2: 컴파일 오류로 수정 적용 안됨**

**확인:**
- IMU Hub Module 프로젝트 빌드 성공?
- `Debug\IMU_Hub_Module.elf` 파일 생성?
- 플래시 다운로드 성공?

**해결:**
- Clean Build
- Rebuild All
- 플래시 다운로드 재시도

---

### **원인 3: 잘못된 코드 복사**

**확인:**

```c
/* agr_pnp.c - AGR_PnP_SendHeartbeat() */

/* ✅ 이렇게 되어 있는지 확인! */
if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
    nmt_state = (inst->my_nmt != NULL) 
                ? (uint8_t)inst->my_nmt->state  // ✅ 이거!
                : (uint8_t)inst->config.nmt.state;
} else {
    nmt_state = AGR_NMT_OPERATIONAL;
}

/* ❌ 만약 이렇게 되어 있으면 문제! */
nmt_state = (inst->config.role == AGR_PNP_ROLE_SLAVE) 
            ? (uint8_t)inst->config.nmt.state  // ❌ 잘못됨!
            : AGR_NMT_OPERATIONAL;
```

---

## ✅ **즉시 확인할 것**

### **Live Expression (IMU Hub)**

```c
/* 1순위 */
s_imu_hub_pnp_inst.my_nmt  // ⚠️ NULL이면 문제! (0xXXXXXXXX 주소여야 함)

/* 2순위 */
s_imu_hub_pnp_inst.my_nmt->state  // 0x05 (OPERATIONAL) ✅

/* 3순위 */
s_imu_hub_nmt.state  // 0x05 (OPERATIONAL) ✅
```

### **Live Expression (XM)**

```c
/* 1순위 */
s_imu_hub_inst.master_pnp->devices[0].nmt.state  // 0x7F (PRE_OP) ❌ 또는 0x05 (OP) ✅

/* 2순위 */
s_imu_hub_inst.master_pnp->devices[0].nmt.last_activity_ms  // 증가? ✅

/* 3순위 */
s_imu_hub_inst.pre_op_state  // 8 (COMPLETE) ✅ 또는 1~7 (진행중) ❌
```

### **CAN 버스 모니터**

```
# IMU Hub Heartbeat
ID: 0x708, Data: [0x??]  ← 이게 0x05 (OPERATIONAL)인지 확인!

# NMT START (XM → IMU Hub)
ID: 0x000, Data: [0x01, 0x08]  ← 전송되었는지 확인!
```

---

## 🔥 **긴급 수정**

만약 `s_imu_hub_pnp_inst.my_nmt == NULL`이면:

```c
/* IMU Hub - xm_drv.c - XM_Drv_Init() */

/* 이 줄 추가 (AGR_PnP_RegisterMaster 다음에) */
AGR_PnP_SetMyNmt(&s_imu_hub_pnp_inst, &s_imu_hub_nmt);
```

**빌드 후 다시 테스트!**

---

**이 값들을 알려주시면 정확한 원인을 찾아드리겠습니다!** 🙏
