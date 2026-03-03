# Bug Fix: NMT STOPPED 및 동기화 문제

**작성일**: 2025-12-08  
**버그 ID**: #8  
**심각도**: Critical (PDO 전송 중단, 무한 연결 상태)

---

## 🐛 **버그 증상**

### **문제 1: PDO 전송이 timestamp 5498ms에 멈춤**
- IMU Hub Module의 NMT가 `AGR_NMT_STOPPED`로 변경
- Heartbeat는 계속 주고받고 있어서 재연결 시도 안함
- `s_imu_hub_nmt.state` = `AGR_NMT_STOPPED`

### **문제 2: NMT 상태 동기화 실패**
- Slave (IMU Hub):
  - `s_imu_hub_nmt.state` = `AGR_NMT_STOPPED` ✅
  - `s_imu_hub_pnp_inst.is_master_connected` = `true` ❌
- Master (XM):
  - `devices[0].nmt.state` = `AGR_NMT_OPERATIONAL` ❌
- **Master와 Slave 간 NMT 상태가 동기화되지 않음!**

---

## 🔍 **근본 원인 분석**

### **문제 1의 원인: Slave 자신의 NMT를 Timeout 체크**

```444:450:C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module\IMU_Hub_FW\Devices\AGR\eXtension_Module\xm_drv.c
    AGR_NMT_InitEx(&s_imu_hub_nmt, 
                   5000,                      /* Heartbeat Timeout: 5s (Slave는 사용 안함) */
                   AGR_NODE_ID_IMU_HUB,       /* Slave 자신의 Node ID (0x02) */
                   NULL,                      /* Slave는 자신의 NMT 콜백 불필요 */
                   NULL,
                   NULL);
```

```549:550:C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module\IMU_Hub_FW\Devices\AGR\eXtension_Module\xm_drv.c
    /* 1. CANopen NMT Heartbeat Timeout 체크 */
    AGR_NMT_CheckTimeout(&s_imu_hub_nmt, current_ms);
```

**문제:**
- `s_imu_hub_nmt`는 **Slave 자신의 NMT** 상태
- Timeout이 5000ms로 설정됨
- **Slave는 자기 자신에게 Heartbeat를 보내지 않음!**
- `last_activity_ms`가 업데이트 안됨
- **5초 후 자동으로 STOPPED!**

### **문제 2의 원인: Heartbeat에 잘못된 NMT 상태 전송**

```417:426:C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module\IMU_Hub_FW\Services\AGR_PnP\agr_pnp.c
    /* Heartbeat CAN 메시지 전송 */
    /* CANopen 표준: CAN ID = 0x700 + node_id, Data = [current_nmt_state] */
    uint8_t nmt_state = (inst->config.role == AGR_PNP_ROLE_SLAVE) 
                        ? (uint8_t)inst->config.nmt.state 
                        : AGR_NMT_OPERATIONAL;  /* Master는 항상 OPERATIONAL로 전송 */
    
    uint8_t hb_msg[1] = { nmt_state };
    uint32_t can_id = HEARTBEAT_CAN_ID(inst->config.my_node_id);
    
    return inst->config.tx_func(can_id, hb_msg, 1);
```

**문제:**
- Slave는 `inst->config.nmt.state` 전송 ⬅️ **이건 Master 추적용!**
- **Slave 자신의 NMT 상태는 `s_imu_hub_nmt.state`인데 전송 안됨!**
- Master는 Slave의 Heartbeat를 받아도 상태를 모름

---

## 🔧 **수정 사항**

### **수정 1: Slave 자신의 NMT는 Timeout 체크 안함**

**xm_drv.c (IMU Hub):**

```c
/* 수정 전 */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000,  /* ❌ Timeout 설정 → 5초 후 STOPPED! */
               AGR_NODE_ID_IMU_HUB,
               NULL, NULL, NULL);

/* 수정 후 */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               0,  /* ✅ Timeout 비활성화 (Slave는 자신을 타임아웃 체크 안함) */
               AGR_NODE_ID_IMU_HUB,
               NULL, NULL, NULL);
```

```c
/* 수정 전 */
void XM_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ❌ Slave 자신의 NMT를 타임아웃 체크! */
    AGR_NMT_CheckTimeout(&s_imu_hub_nmt, current_ms);
    
    AGR_PnP_RunPeriodic(&s_imu_hub_pnp_inst);
}

/* 수정 후 */
void XM_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* ✅ Slave 자신의 NMT는 Timeout 체크 안함! 
     * (자기 자신에게 Heartbeat 안보냄)
     */
    
    AGR_PnP_RunPeriodic(&s_imu_hub_pnp_inst);
}
```

---

### **수정 2: Heartbeat에 Slave 자신의 NMT 상태 전송**

**agr_pnp.h:**

```c
struct AGR_PnP_Inst_s {
    AGR_PnP_Config_t    config;
    AGR_PnP_Device_t    devices[AGR_PNP_MAX_DEVICES];
    uint8_t             device_count;
    
    /* Slave 전용: Master 연결 상태 */
    bool                is_master_connected;
    uint32_t            last_master_hb_time;
    uint32_t            last_hb_sent_time;
    AGR_NMT_Inst_t*     my_nmt;  /* ✅ 추가: Slave 자신의 NMT 포인터 */
    
    /* 공통 */
    bool                is_initialized;
    uint32_t (*get_tick)(void);
};

/* ✅ 새로운 API 추가 */
void AGR_PnP_SetMyNmt(AGR_PnP_Inst_t* inst, AGR_NMT_Inst_t* my_nmt);
```

**agr_pnp.c:**

```c
/* 수정 전 */
int AGR_PnP_SendHeartbeat(AGR_PnP_Inst_t* inst)
{
    uint8_t nmt_state = (inst->config.role == AGR_PNP_ROLE_SLAVE) 
                        ? (uint8_t)inst->config.nmt.state  /* ❌ Master 추적용! */
                        : AGR_NMT_OPERATIONAL;
    
    uint8_t hb_msg[1] = { nmt_state };
    uint32_t can_id = HEARTBEAT_CAN_ID(inst->config.my_node_id);
    
    return inst->config.tx_func(can_id, hb_msg, 1);
}

/* 수정 후 */
void AGR_PnP_SetMyNmt(AGR_PnP_Inst_t* inst, AGR_NMT_Inst_t* my_nmt)
{
    if (inst != NULL) {
        inst->my_nmt = my_nmt;
    }
}

int AGR_PnP_SendHeartbeat(AGR_PnP_Inst_t* inst)
{
    uint8_t nmt_state;
    
    if (inst->config.role == AGR_PNP_ROLE_SLAVE) {
        /* ✅ Slave는 자신의 NMT 상태 전송 (my_nmt가 등록되어 있으면 사용) */
        nmt_state = (inst->my_nmt != NULL) 
                    ? (uint8_t)inst->my_nmt->state  /* ✅ Slave 자신의 상태! */
                    : (uint8_t)inst->config.nmt.state;  /* Fallback */
    } else {
        /* Master는 항상 OPERATIONAL로 전송 */
        nmt_state = AGR_NMT_OPERATIONAL;
    }
    
    uint8_t hb_msg[1] = { nmt_state };
    uint32_t can_id = HEARTBEAT_CAN_ID(inst->config.my_node_id);
    
    return inst->config.tx_func(can_id, hb_msg, 1);
}
```

**xm_drv.c (IMU Hub):**

```c
/* 초기화 시 Slave 자신의 NMT 등록 */
AGR_PnP_RegisterMaster(&s_imu_hub_pnp_inst, &xm_master);

/* ✅ 추가: Slave 자신의 NMT 등록 (Heartbeat 전송 시 사용) */
AGR_PnP_SetMyNmt(&s_imu_hub_pnp_inst, &s_imu_hub_nmt);

AGR_PnP_SendBootup(&s_imu_hub_pnp_inst);
```

---

## 📊 **수정 후 동작**

### **정상 흐름 (문제 1 해결)**

```
1. IMU Hub Init
   s_imu_hub_nmt: timeout = 0  ✅ 타임아웃 비활성화
   ↓
2. XM_Drv_RunPeriodic()
   AGR_NMT_CheckTimeout(&s_imu_hub_nmt, ...) ❌ 호출 안함!
   ↓
3. 5초 후
   s_imu_hub_nmt.state = OPERATIONAL  ✅ 계속 유지!
```

### **정상 흐름 (문제 2 해결)**

```
1. IMU Hub Init
   AGR_PnP_SetMyNmt(&s_imu_hub_pnp_inst, &s_imu_hub_nmt);
   ↓
2. Heartbeat 전송 (IMU Hub → XM)
   nmt_state = my_nmt->state  ✅ Slave 자신의 NMT 상태!
   CAN: 0x708, [0x05] (OPERATIONAL)
   ↓
3. XM 수신
   devices[0].nmt.state = OPERATIONAL  ✅ 동기화!
   ↓
4. IMU Hub가 STOPPED로 변경되면
   nmt_state = my_nmt->state = 0x04 (STOPPED)
   CAN: 0x708, [0x04]  ✅ STOPPED 상태 전송!
   ↓
5. XM 수신
   devices[0].nmt.state = STOPPED  ✅ 동기화!
   devices[0].nmt.last_activity_ms = current_ms
   ↓
6. XM의 AGR_NMT_CheckTimeout()
   elapsed > timeout → STOPPED  ✅ 타임아웃 감지!
```

---

## 🎯 **검증 방법**

### **Live Expression (IMU Hub)**

```c
/* 5초 후 */
s_imu_hub_nmt.state  // AGR_NMT_OPERATIONAL (0x05) ✅ STOPPED 안됨!
s_imu_hub_nmt.timeout_ms  // 0 ✅ 타임아웃 비활성화
```

### **Live Expression (XM)**

```c
/* IMU Hub가 STOPPED 상태일 때 */
s_imu_hub_inst.master_pnp->devices[0].nmt.state  // AGR_NMT_STOPPED (0x04) ✅ 동기화!
```

### **CAN 버스 모니터**

```
/* IMU Hub가 OPERATIONAL일 때 */
ID: 0x708, Data: [0x05]  ✅ OPERATIONAL 전송

/* IMU Hub가 STOPPED일 때 */
ID: 0x708, Data: [0x04]  ✅ STOPPED 전송
```

---

## 📝 **관련 버그**

- **Bug #7**: `is_connected` 항상 false (✅ 수정 완료)
- **Bug #6**: Slave NMT Node ID 오류 (✅ 수정 완료)
- **Bug #5**: PDO 전송 안됨 (✅ 수정 완료)

---

## ✅ **수정 완료 체크리스트**

- [x] `xm_drv.c`: NMT Timeout 비활성화 (0으로 설정)
- [x] `xm_drv.c`: `AGR_NMT_CheckTimeout()` 호출 제거
- [x] `agr_pnp.h`: `my_nmt` 포인터 추가
- [x] `agr_pnp.h`: `AGR_PnP_SetMyNmt()` API 추가
- [x] `agr_pnp.c`: `AGR_PnP_SetMyNmt()` 구현
- [x] `agr_pnp.c`: `AGR_PnP_SendHeartbeat()` 수정 (my_nmt 사용)
- [x] `xm_drv.c`: 초기화 시 `AGR_PnP_SetMyNmt()` 호출
- [x] 빌드 성공 확인
- [x] Live Expression 확인 (5초 후에도 OPERATIONAL 유지)
- [x] NMT 동기화 확인 (Heartbeat에 올바른 상태 전송)
- [x] 문서 작성

---

**이제 XM-IMU PnP가 안정적으로 동작하며, NMT 상태도 정확히 동기화됩니다!** 🎉
