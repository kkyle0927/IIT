# XM ↔ IMU Hub PnP 전체 시퀀스 완벽 분석

**작성일**: 2025-12-08  
**문제**: PDO 데이터가 0으로 수신됨

---

## 🔄 **전체 시퀀스 (초기화 → PDO 전송)**

### **Phase 0: 초기화 단계**

#### **IMU Hub (Slave) 초기화**
```c
/* 파일: xm_drv.c - XM_Drv_Init() */

/* 1. 변수 초기화 */
memset(&s_imu_hub_nmt, 0, sizeof(AGR_NMT_Inst_t));
memset(&s_imu_hub_pnp_inst, 0, sizeof(AGR_PnP_Inst_t));
memset(&s_pdo_metadata, 0, sizeof(XM_Drv_PdoMetadata_t));
memset(&s_imu_hub_tx_data, 0, sizeof(XM_TxData_t));  // ✅ IMU 데이터 0으로 초기화

/* 2. Slave 자신의 NMT 초기화 */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000,
               AGR_NODE_ID_IMU_HUB,  // 0x08 ✅
               NULL, NULL, NULL);

/* 3. AGR_DOP 초기화 */
AGR_DOP_Init(&s_dop_ctx, 
             &AGR_DOP_GetODEntry,
             s_od_entries,
             s_od_count,
             _Drv_TxFunc_Queue,  // ✅ Tx 함수
             _OnSdoRequest,      // ✅ SDO Request 콜백
             _OnPdoReceived);    // ✅ PDO 수신 콜백

/* 4. AGR_PnP 초기화 (Slave) */
AGR_PnP_Config_t pnp_config = {
    .role = AGR_PNP_ROLE_SLAVE,
    .node_id = AGR_NODE_ID_IMU_HUB,  // 0x08
    .get_tick = IOIF_TIM_GetTick,
    .tx_func = _Drv_TxFunc_Queue,
    .nmt = s_imu_hub_nmt,  // ✅ Slave 자신의 NMT
    .dop_ctx = &s_dop_ctx,
    .heartbeat_period = 1000,
    .heartbeat_timeout = 0,  // Slave는 Timeout 안함
    .callbacks = { ... }
};
AGR_PnP_Init(&s_imu_hub_pnp_inst, &pnp_config);

/* 5. Master (XM) 등록 */
AGR_PnP_Master_t master = {
    .node_id = AGR_NODE_ID_XM,  // 0x02
    .heartbeat_timeout = 3000
};
AGR_PnP_RegisterMaster(&s_imu_hub_pnp_inst, &master);
// ✅ 내부에서 config.nmt 초기화 (Master 추적용)
```

#### **XM (Master) 초기화**
```c
/* 파일: imu_hub_drv.c - ImuHub_Drv_Init() */

/* 1. Master PnP 초기화 */
AGR_PnP_Config_t master_pnp_config = {
    .role = AGR_PNP_ROLE_MASTER,
    .node_id = AGR_NODE_ID_XM,  // 0x02
    .get_tick = IOIF_TIM_GetTick,
    .tx_func = tx_func,
    .nmt = { 0 },  // Master는 자신의 NMT 불필요
    .dop_ctx = NULL,  // Master는 DOP 불필요
    .heartbeat_period = 1000,
    .heartbeat_timeout = 0,
    .callbacks = { ... }
};
AGR_PnP_Init(master_pnp, &master_pnp_config);

/* 2. IMU Hub (Slave) Device 등록 */
AGR_PnP_Device_t imu_hub_device = {
    .node_id = AGR_NODE_ID_IMU_HUB,  // 0x08
    .name = "IMU Hub Module",
    .heartbeat_timeout = 3000,
    .callbacks = {
        .on_bootup = _PnP_OnBootup,        // ✅ Boot-up 콜백
        .on_nmt_change = _PnP_OnNmtChange,  // ✅ NMT 변경 콜백
        .on_run_pre_op = _PnP_RunPreOp,    // ✅ Pre-Op 실행 콜백
        .on_error = _PnP_OnError,
        ...
    }
};
int dev_idx = AGR_PnP_RegisterDevice(master_pnp, &imu_hub_device);
// ✅ devices[0]에 등록됨
// ✅ devices[0].nmt 초기화 (Slave 추적용)
```

---

### **Phase 1: Boot-up 전송 (IMU Hub → XM)**

```
[0초] IMU Hub 초기화 완료
  ↓
system_startup.c - System_Startup()
  ↓
XM_Drv_Init(IOIF_FDCAN_2)  // AGR_PnP 초기화
  ↓
AGR_PnP_SendBootup(&s_imu_hub_pnp_inst)
  ↓
CAN ID: 0x708 (0x700 + 0x08)
Data: [0x00]  ← Boot-up 메시지
  ↓
XM 수신 ✅
```

**코드 흐름:**
```c
/* IMU Hub - system_startup.c */
System_Startup()
  → XM_Drv_Init(IOIF_FDCAN_2)
    → AGR_PnP_Init(&s_imu_hub_pnp_inst, ...)
    → AGR_PnP_SendBootup(&s_imu_hub_pnp_inst)
      → CAN: 0x708, [0x00]

/* XM - canfd_rx_handler.c */
HAL_FDCAN_RxFifo0Callback()
  → CANFD_RxHandler_ProcessFrame()
    → ImuHub_Drv_ProcessCANMessage(0x708, [0x00], 1, ...)
      → AGR_PnP_ProcessMessage(master_pnp, 0x708, [0x00], ...)
        → AGR_NMT_ProcessMessage(&devices[0].nmt, 0x708, [0x00], ...)
          → devices[0].nmt.state = AGR_NMT_BOOT_UP
          → _OnSlaveNmtStateChanged() 호출
            → devices[0].callbacks.on_bootup(0x08) ✅
              → _PnP_OnBootup(0x08)
                → s_imu_hub_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A ✅
```

---

### **Phase 2: Pre-Op 시퀀스 (XM → IMU Hub)**

#### **Step 1: PDO Mapping A 전송**

```
[XM Periodic - 10ms마다]
ImuHub_Drv_RunPeriodic()
  ↓
AGR_PnP_RunPeriodic(master_pnp)
  ↓
_RunMasterPreOpStateMachine(inst, &devices[0])
  ↓
devices[0].callbacks.on_run_pre_op(0x08, master_pnp)
  ↓
_PnP_RunPreOp(0x08, master_pnp)
  ↓
pre_op_state == IMUHUB_PRE_OP_SEND_PDO_MAP_A ✅
  ↓
_Step_SendPdoMapA(0x08, master_pnp)
  ↓
AGR_PnP_SendSDOWrite(master_pnp, 0x08, 0x2010, 0, pdo_mapping_a, 64)
  ↓
CAN ID: 0x608 (0x600 + 0x08)
Data: [SDO Write, 0x10, 0x20, 0x00, ...PDO Mapping A...]
  ↓
IMU Hub 수신 ✅
```

**IMU Hub 처리:**
```c
/* xm_drv.c - XM_Drv_ProcessCANMessage() */
0x608 수신 (SDO)
  ↓
AGR_PnP_ProcessMessage(&s_imu_hub_pnp_inst, 0x608, ...)
  ↓
AGR_DOP_ProcessRxMessage(&s_dop_ctx, 0x608, ...)
  ↓
_OnSdoRequest(request, response)
  ↓
OD Entry 0x2010 찾기 (s_od_entries)
  ↓
memcpy(s_pdo_mapping_a, request->data, 64)  // ✅ PDO Mapping A 저장
  ↓
WriteCb 호출: _OnPdoMappingA_Set()  // ✅ 핵심!
  ↓
AGR_DOP_ClearTxPDOMapN(&s_dop_ctx, 1)
AGR_DOP_ApplyTxPDOMapFromSDON(&s_dop_ctx, 1, s_pdo_mapping_a, 64)
  ↓
s_dop_ctx.tx_pdo_map[0] 설정 완료 ✅
  ↓
SDO Response 전송
  ↓
CAN ID: 0x588 (0x580 + 0x08)
Data: [SDO ACK, 0x10, 0x20, 0x00]
```

**XM 수신:**
```c
/* imu_hub_drv.c - ImuHub_Drv_ProcessCANMessage() */
0x588 수신 (SDO Response)
  ↓
AGR_PnP_ProcessMessage(master_pnp, 0x588, ...)
  ↓
AGR_DOP_ProcessRxMessage(master_pnp->dop_ctx, ...)
  ↓
_OnSdoResponse(response)
  ↓
response->index == 0x2010 && (response->cs & 0xE0) == 0x60 ✅
  ↓
pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B ✅
```

#### **Step 2: PDO Mapping B 전송**
- **동일한 흐름**
- OD Entry: `0x2011`
- WriteCb: `_OnPdoMappingB_Set()`
- 결과: `s_dop_ctx.tx_pdo_map[1]` 설정 완료 ✅

#### **Step 3: IMU Connected Mask 조회 (Optional)**
- OD Entry: `0x2000`
- 읽기 전용 (Read Only)
- XM이 IMU 연결 상태 조회

#### **Step 4: NMT START 전송**

```
pre_op_state == IMUHUB_PRE_OP_SEND_NMT_START
  ↓
_Step_SendNmtStart(0x08, master_pnp)
  ↓
AGR_PnP_SendNmtCommand(master_pnp, 0x08, AGR_NMT_CMD_START)
  ↓
CAN ID: 0x000
Data: [0x01, 0x08]  ← START to Node 0x08
  ↓
IMU Hub 수신 ✅
```

**IMU Hub 처리:**
```c
/* xm_drv.c - XM_Drv_ProcessCANMessage() */
0x000 수신 (NMT Command)
  ↓
AGR_NMT_ProcessMessage(&s_imu_hub_nmt, 0x000, [0x01, 0x08], ...)
  ↓
target_node_id == 0x08 == s_imu_hub_nmt.node_id ✅
  ↓
AGR_NMT_ProcessCommand(&s_imu_hub_nmt, AGR_NMT_CMD_START)
  ↓
s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL ✅
  ↓
_NotifyStateChange() → (콜백 없음)
```

**XM 완료:**
```c
pre_op_state = IMUHUB_PRE_OP_COMPLETE ✅
```

---

### **Phase 3: Heartbeat 교환**

#### **IMU Hub → XM**
```c
/* xm_drv.c - XM_Drv_RunPeriodic() */
AGR_PnP_RunPeriodic(&s_imu_hub_pnp_inst)
  ↓
매 1초마다:
  ↓
AGR_PnP_SendHeartbeat(&s_imu_hub_pnp_inst)
  ↓
CAN ID: 0x708 (0x700 + 0x08)
Data: [0x05]  ← OPERATIONAL
```

#### **XM → IMU Hub**
```c
/* imu_hub_drv.c - ImuHub_Drv_RunPeriodic() */
AGR_PnP_RunPeriodic(master_pnp)
  ↓
매 1초마다:
  ↓
AGR_PnP_SendHeartbeat(master_pnp)
  ↓
CAN ID: 0x702 (0x700 + 0x02)
Data: [0x05]  ← OPERATIONAL
```

**IMU Hub 수신:**
```c
/* xm_drv.c - XM_Drv_ProcessCANMessage() */
0x702 수신 (Master Heartbeat)
  ↓
fnc_code == 0x700 && sender_node_id == AGR_NODE_ID_XM (0x02) ✅
  ↓
AGR_NMT_ProcessMessage(&s_imu_hub_pnp_inst.config.nmt, 0x702, [0x05], ...)
  ↓
config.nmt.state = AGR_NMT_OPERATIONAL
config.nmt.last_activity_ms = current_ms
  ↓
_NotifyStateChange() → _OnSlaveNmtStateChanged()
  ↓
is_master_connected = true ✅
last_master_hb_time = current_ms ✅
```

---

### **Phase 4: PDO 전송 (IMU Hub → XM)**

#### **IMU Hub - 데이터 준비**
```c
/* core_process.c - _FlushOutputs() */

/* 1. IsConnected 체크 */
if (!XM_Drv_IsConnected()) {  
    // (s_imu_hub_nmt.state == OPERATIONAL) && is_master_connected
    return;  // ❌ 전송 안함
}

/* 2. 타임스탬프 설정 */
XM_Drv_SetFrameTimestamp(g_sys.tick_cnt);
  ↓
s_pdo_metadata.timestamp_low  = (tick_cnt >> 0) & 0xFF;
s_pdo_metadata.timestamp_mid  = (tick_cnt >> 8) & 0xFF;
s_pdo_metadata.timestamp_high = (tick_cnt >> 16) & 0xFF;

/* 3. Valid Mask 초기화 */
XM_Drv_ClearValidMask();
  ↓
s_pdo_metadata.valid_mask = 0;

/* 4. IMU 데이터 업데이트 */
for (int i = 0; i < 6; i++) {
    XM_Drv_ImuData_t drv_data;
    
    if (g_sys.imus[i].is_connected) {  // ⚠️ 이게 false면 0 전송!
        /* UART에서 받은 데이터 변환 */
        drv_data.q[0] = XM_Drv_QuatToInt16(ebimu->q_w);
        drv_data.q[1] = XM_Drv_QuatToInt16(ebimu->q_x);
        ... (Quaternion, Accel, Gyro)
    } else {
        memset(&drv_data, 0, sizeof(XM_Drv_ImuData_t));  // ❌ 0으로 채움!
    }
    
    XM_Drv_UpdateImuData(i, &drv_data);
      ↓
    memcpy(&s_imu_hub_tx_data.imu[i], &drv_data, sizeof(...));  // ✅ 저장
}

/* 5. IMU Connected Mask 설정 */
uint8_t connected_mask = 0;
for (int i = 0; i < 6; i++) {
    if (g_sys.imus[i].is_connected) {
        connected_mask |= (1 << i);
    }
}
XM_Drv_SetImuConnectedMask(connected_mask);
  ↓
s_imu_hub_rx_data.imu_connected_mask = connected_mask;
```

#### **IMU Hub - PDO 전송**
```c
/* 6. PDO 전송 (번갈아) */
if (s_pdo_toggle) {
    XM_Drv_SendTPDO2();
} else {
    XM_Drv_SendTPDO1();
}
s_pdo_toggle = !s_pdo_toggle;
```

**TPDO1 전송:**
```c
/* xm_drv.c - XM_Drv_SendTPDO1() */
AGR_DOP_EncodeTxPDON(&s_dop_ctx, 1, pdo_buf, 64)
  ↓
s_dop_ctx.tx_pdo_map[0] 참조  // ✅ PDO Mapping A
  ↓
for (각 map item) {
    OD Entry 찾기 (index, subindex)
      ↓
    entry->data_ptr에서 데이터 읽기  // ⚠️ 핵심!
      ↓
    pdo_buf에 복사
}
  ↓
return len;  // 인코딩된 데이터 길이
  ↓
CAN 전송: 0x188 (0x180 + 0x08), pdo_buf, len
```

---

## 🚨 **문제 발생 지점 분석**

### **의심 1: `g_sys.imus[i].is_connected` = false?**

```c
/* core_process.c - _FetchInputs() */
for (int i = 0; i < EBIMU_COUNT; i++) {
    uint32_t last_rx_time = g_sys.imus[i].last_update_time;
    g_sys.imus[i].is_connected = ebimu9dofv6.IsConnected(i, last_rx_time, current_time);
}
```

**확인:**
- Live Expression: `g_sys.imus[0].is_connected`, `g_sys.imus[1].is_connected`, ...
- **false면 PDO 데이터가 0!** ❌

---

### **의심 2: Object Dictionary 포인터가 잘못?**

```c
/* xm_drv.c - s_od_entries[] */
/* PDO로 전송될 데이터 */
{ 0x4000, 0x00, AGR_TYPE_BLOB, 4, AGR_ACCESS_RO, &s_pdo_metadata, NULL },
{ 0x4100, 0x00, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[0], NULL },
{ 0x4101, 0x00, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[1], NULL },
{ 0x4102, 0x00, AGR_TYPE_BLOB, 32, AGR_ACCESS_RO, &s_imu_hub_tx_data.imu[2], NULL },
...
```

**확인:**
- Live Expression: `s_imu_hub_tx_data.imu[0].q[0]`, `s_imu_hub_tx_data.imu[0].q[1]`, ...
- **0이면 데이터가 안들어감!** ❌

---

### **의심 3: PDO Mapping이 잘못?**

```c
/* imu_hub_drv.c - s_pdo_mapping_a[] */
static uint8_t s_pdo_mapping_a[64] = {
    6,  // Number of mapped objects
    
    /* 1. Metadata (4B) */
    32, 0, 0x00, 0x40,  // 0x4000 (metadata)
    
    /* 2. IMU 0 (32B) */
    0, 1, 0x00, 0x41,  // 0x4100 (imu[0])
    
    /* 3. IMU 1 (32B) */
    0, 1, 0x01, 0x41,  // 0x4101 (imu[1])
    
    /* 4. IMU 2 (32B) */
    0, 1, 0x02, 0x41,  // 0x4102 (imu[2])
    ...
};
```

**확인:**
- Live Expression: `s_dop_ctx.tx_pdo_map[0].count`
- **0이면 PDO Mapping 안됨!** ❌

---

## 🎯 **Live Expression 확인 체크리스트**

### **IMU Hub (Slave)**

```c
/* 1. UART 센서 연결 상태 */
g_sys.imus[0].is_connected  // ⚠️ 가장 중요!
g_sys.imus[1].is_connected
g_sys.imus[2].is_connected
g_sys.imus[3].is_connected
g_sys.imus[4].is_connected
g_sys.imus[5].is_connected

/* 2. UART 센서 데이터 */
g_sys.imus[0].data.q_w  // Quaternion
g_sys.imus[0].data.q_x
g_sys.imus[0].data.acc_x  // Accel
g_sys.imus[0].data.gyr_x  // Gyro

/* 3. XM Driver 내부 데이터 */
s_imu_hub_tx_data.imu[0].q[0]  // ⚠️ 여기가 0이면 문제!
s_imu_hub_tx_data.imu[0].q[1]
s_imu_hub_tx_data.imu[0].a[0]
s_imu_hub_tx_data.imu[0].g[0]

/* 4. PDO Mapping 설정 */
s_dop_ctx.tx_pdo_map[0].count  // >0이어야 함!
s_dop_ctx.tx_pdo_map[1].count  // >0이어야 함!

/* 5. NMT 상태 */
s_imu_hub_nmt.state  // 0x05 (OPERATIONAL)
s_imu_hub_nmt.node_id  // 0x08

/* 6. Master 연결 상태 */
s_imu_hub_pnp_inst.is_master_connected  // true
s_imu_hub_pnp_inst.config.nmt.state  // 0x05
```

---

## 🔧 **디버깅 추천 순서**

### **1단계: UART 센서 확인**
```c
// ⚠️ 브레이크포인트: _FlushOutputs() 시작
if (g_sys.imus[0].is_connected == false) {
    /* ❌ UART 센서가 연결 안됨! */
    /* → uart_rx_handler.c 확인 */
    /* → UART 초기화 확인 */
    /* → IDLE 인터럽트 확인 */
}
```

### **2단계: 데이터 변환 확인**
```c
// ⚠️ 브레이크포인트: XM_Drv_UpdateImuData() 직후
if (s_imu_hub_tx_data.imu[0].q[0] == 0) {
    /* ❌ 데이터 변환 실패! */
    /* → XM_Drv_QuatToInt16() 확인 */
    /* → ebimu->q_w 값 확인 */
}
```

### **3단계: PDO 인코딩 확인**
```c
// ⚠️ 브레이크포인트: AGR_DOP_EncodeTxPDON() 내부
if (len == 0) {
    /* ❌ PDO Mapping이 비어있음! */
    /* → s_dop_ctx.tx_pdo_map[0].count 확인 */
    /* → _OnPdoMappingA_Set() 호출 여부 확인 */
}
```

---

**가장 의심스러운 것: `g_sys.imus[i].is_connected` = false**

PDO 데이터가 0이라면, 대부분의 경우 **UART 센서가 연결되지 않았거나** **데이터가 업데이트되지 않는 것**이 원인입니다!
