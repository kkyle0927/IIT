# PnP 코드 실행 흐름 다이어그램

**작성일**: 2025-12-08  
**목적**: 코드와 순서도를 1:1 매칭

---

## 📦 **Phase 0: 초기화**

```
┌─────────────────────────────────────────────────────────────┐
│                     IMU Hub (Slave)                          │
├─────────────────────────────────────────────────────────────┤
│ 파일: system_startup.c                                       │
│ 함수: System_Startup()                                       │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_Init(IOIF_FDCANx_t fdcan_id)                    │
│   ↓                                                          │
│ Line 422-423: memset(&s_imu_hub_nmt, 0)                      │
│               memset(&s_imu_hub_pnp_inst, 0)                 │
│   ↓                                                          │
│ Line 443-448: AGR_NMT_InitEx(&s_imu_hub_nmt,                 │
│                               5000,                          │
│                               AGR_NODE_ID_IMU_HUB, ← 0x08    │
│                               NULL, NULL, NULL)              │
│   ↓                                                          │
│ Line 451-467: AGR_DOP_Init(&s_dop_ctx, ...)                  │
│   ↓                                                          │
│ Line 470-485: AGR_PnP_Init(&s_imu_hub_pnp_inst, ...)         │
│               role = AGR_PNP_ROLE_SLAVE                      │
│               node_id = AGR_NODE_ID_IMU_HUB (0x08)           │
│   ↓                                                          │
│ Line 488-492: AGR_PnP_RegisterMaster(...)                    │
│               Master node_id = AGR_NODE_ID_XM (0x02)         │
│               ✅ config.nmt 초기화 (Master 추적)              │
│   ↓                                                          │
│ Line 495-498: AGR_PnP_SendBootup(&s_imu_hub_pnp_inst)        │
│               ✅ CAN: 0x708, [0x00]                          │
│                                                              │
│ ✅ 초기화 완료!                                               │
│   s_imu_hub_nmt.state = BOOT_UP (자신)                       │
│   config.nmt.state = BOOT_UP (Master 추적)                   │
│   s_imu_hub_tx_data.imu[0~5] = 0                             │
│   s_pdo_metadata = 0                                         │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                        XM (Master)                           │
├─────────────────────────────────────────────────────────────┤
│ 파일: system_startup.c                                       │
│ 함수: System_Startup()                                       │
│   ↓                                                          │
│ 파일: imu_hub_drv.c                                          │
│ 함수: ImuHub_Drv_Init(master_pnp, tx_func)                   │
│   ↓                                                          │
│ Line 299-321: AGR_PnP_Init(master_pnp, ...)                  │
│               role = AGR_PNP_ROLE_MASTER                     │
│               node_id = AGR_NODE_ID_XM (0x02)                │
│   ↓                                                          │
│ Line 324-387: AGR_PnP_RegisterDevice(master_pnp, ...)        │
│               devices[0].node_id = 0x08 (IMU Hub)            │
│               devices[0].name = "IMU Hub Module"             │
│               devices[0].callbacks = {                       │
│                 .on_bootup = _PnP_OnBootup,                  │
│                 .on_run_pre_op = _PnP_RunPreOp,              │
│                 ...                                          │
│               }                                              │
│   ↓                                                          │
│ ✅ 초기화 완료!                                               │
│   s_imu_hub_inst.pre_op_state = IDLE (0)                     │
│   devices[0].nmt.state = BOOT_UP                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 📡 **Phase 1: Boot-up 전송**

```
┌─────────────────────────────────────────────────────────────┐
│                     IMU Hub → XM                             │
├─────────────────────────────────────────────────────────────┤
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_SendBootup(inst)                               │
│ Line: inst->config.tx_func(0x700 + node_id, [0x00], 1)      │
│   ↓                                                          │
│ CAN ID: 0x708 (0x700 + 0x08)                                 │
│ Data: [0x00] ← Boot-up                                       │
└─────────────────────────────────────────────────────────────┘
                              ↓
                      [FDCAN H/W 전송]
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                        XM 수신                               │
├─────────────────────────────────────────────────────────────┤
│ 파일: canfd_rx_handler.c                                     │
│ 함수: HAL_FDCAN_RxFifo0Callback()                            │
│   ↓                                                          │
│ 함수: CANFD_RxHandler_ProcessFrame()                         │
│   can_id = 0x708                                             │
│   data = [0x00]                                              │
│   ↓                                                          │
│ 파일: imu_hub_drv.c                                          │
│ 함수: ImuHub_Drv_ProcessCANMessage(0x708, [0x00], 1)         │
│   ↓                                                          │
│ Line 408-410: fnc_code = 0x700 (Boot-up/Heartbeat)          │
│   ↓                                                          │
│ Line 411-415: AGR_PnP_ProcessMessage(master_pnp, ...)        │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_ProcessMessage()                               │
│   ↓                                                          │
│ Line: AGR_NMT_ProcessMessage(&devices[0].nmt, 0x708, ...)    │
│   ↓                                                          │
│ 파일: agr_nmt.c                                              │
│ 함수: AGR_NMT_ProcessMessage()                               │
│ Line 217-232: Boot-up 감지 (data[0] == 0x00)                │
│               inst->state = AGR_NMT_BOOT_UP                  │
│               AGR_NMT_UpdateActivity()                       │
│                 ↓                                            │
│               inst->state = AGR_NMT_PRE_OPERATIONAL          │
│               _NotifyStateChange(BOOT_UP → PRE_OP)           │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: _OnNmtStateChanged() (내부)                            │
│   ↓                                                          │
│ Line: devices[0].callbacks.on_bootup(0x08) 호출              │
│   ↓                                                          │
│ 파일: imu_hub_drv.c                                          │
│ 함수: _PnP_OnBootup(0x08)                                    │
│ Line 572-580: pre_op_state == IDLE or COMPLETE 확인          │
│               ✅ pre_op_state = SEND_PDO_MAP_A (1)           │
│               sdo_retry_count = 0                            │
│                                                              │
│ ✅ Pre-Op 시작!                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔄 **Phase 2-1: PDO Mapping A 전송**

```
┌─────────────────────────────────────────────────────────────┐
│                      XM → IMU Hub                            │
├─────────────────────────────────────────────────────────────┤
│ 파일: imu_hub_drv.c                                          │
│ 함수: ImuHub_Drv_RunPeriodic()                               │
│ Line 459-462: AGR_PnP_RunPeriodic(master_pnp)                │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_RunPeriodic(inst)                              │
│ Line 459-473: Master 전용 로직                               │
│   for (i = 0; i < device_count; i++)                         │
│     if (dev->nmt.state == PRE_OPERATIONAL)                   │
│       _RunMasterPreOpStateMachine(inst, dev)                 │
│   ↓                                                          │
│ 함수: _RunMasterPreOpStateMachine(inst, &devices[0])         │
│ Line 603-608: dev->callbacks.on_run_pre_op(0x08, inst)       │
│   ↓                                                          │
│ 파일: imu_hub_drv.c                                          │
│ 함수: _PnP_RunPreOp(0x08, master_pnp)                        │
│ Line 599-630: Step Array 실행                                │
│   ↓                                                          │
│ Line 608-622: for (i = 0; i < PRE_OP_STEP_COUNT; i++)       │
│               step = &s_pre_op_steps[i]                      │
│                 ↓                                            │
│               if (pre_op_state == step->send_state)          │
│                 ↓                                            │
│               pre_op_state == SEND_PDO_MAP_A ✅              │
│               step->action(node_id, inst) 호출               │
│   ↓                                                          │
│ 함수: _Step_SendPdoMapA(0x08, master_pnp)                    │
│ Line 641-645: AGR_PnP_SendSDOWrite(inst, 0x08,               │
│                                    0x2010, 0,                │
│                                    s_pdo_map_a, 17)          │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_SendSDOWrite()                                 │
│ Line 200-231: SDO Download Expedited/Segmented 인코딩        │
│   ↓                                                          │
│ Line 232: inst->config.tx_func(0x600 + 0x08, sdo_msg, len)  │
│   ↓                                                          │
│ CAN ID: 0x608 (0x600 + 0x08)                                 │
│ Data: [0x23, 0x10, 0x20, 0x00, ...PDO Mapping A...]          │
│       ↑ Expedited Download (4 bytes)                         │
│                                                              │
│ 파일: imu_hub_drv.c                                          │
│ Line 616-620: pre_op_state = WAIT_PDO_MAP_A (2)              │
│               last_sdo_tx_time = current_ms                  │
└─────────────────────────────────────────────────────────────┘
                              ↓
                      [FDCAN H/W 전송]
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                     IMU Hub 수신                             │
├─────────────────────────────────────────────────────────────┤
│ 파일: canfd_rx_handler.c                                     │
│ 함수: HAL_FDCAN_RxFifo0Callback()                            │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_ProcessCANMessage(0x608, [SDO...], ...)         │
│ Line 609-620: fnc_code = 0x600 (SDO)                         │
│               AGR_PnP_ProcessMessage()                       │
│                 ↓                                            │
│               AGR_DOP_ProcessRxMessage(&s_dop_ctx, ...)      │
│   ↓                                                          │
│ 파일: agr_dop.c                                              │
│ 함수: AGR_DOP_ProcessRxMessage()                             │
│ Line: 확인 후 ctx->on_sdo_request(request, response) 호출    │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: _OnSdoRequest(request, response)                       │
│ Line 879-947: OD Entry 찾기                                  │
│   ↓                                                          │
│ Line 881: entry = _FindODEntry(0x2010, 0x00)                 │
│   ↓                                                          │
│ Line 906-916: SDO Download (Write) 처리                      │
│               memcpy(s_pdo_mapping_a, request->data, 17)     │
│               ✅ s_pdo_mapping_a[] 저장!                     │
│   ↓                                                          │
│ Line 943-946: WriteCb 호출                                   │
│               if (entry->write_cb != NULL)                   │
│                 entry->write_cb()  ← _OnPdoMappingA_Set      │
│   ↓                                                          │
│ 함수: _OnPdoMappingA_Set()                                   │
│ Line 954-961: AGR_DOP_ClearTxPDOMapN(&s_dop_ctx, 1)          │
│               AGR_DOP_ApplyTxPDOMapFromSDON(                 │
│                   &s_dop_ctx, 1,                             │
│                   s_pdo_mapping_a, 64)                       │
│   ↓                                                          │
│ 파일: agr_dop.c                                              │
│ 함수: AGR_DOP_ApplyTxPDOMapFromSDON()                        │
│ Line: s_dop_ctx.tx_pdo_map[0] 파싱 및 설정                   │
│       ✅ count = 4                                           │
│       ✅ items[0] = {0x3000, 0x00, ...}                      │
│       ✅ items[1] = {0x6000, 0x60, ...}                      │
│       ✅ items[2] = {0x6001, 0x60, ...}                      │
│       ✅ items[3] = {0x6002, 0x60, ...}                      │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: _OnSdoRequest()                                        │
│ Line 918-926: SDO Response 생성                              │
│               response->cs = 0x60 (ACK)                      │
│               response->index = 0x2010                       │
│   ↓                                                          │
│ 파일: agr_dop.c                                              │
│ 함수: AGR_DOP_ProcessRxMessage()                             │
│ Line: SDO Response 전송                                      │
│       CAN: 0x588, [0x60, 0x10, 0x20, 0x00]                   │
│                                                              │
│ ✅ PDO Mapping A 설정 완료!                                   │
└─────────────────────────────────────────────────────────────┘
                              ↓
                      [FDCAN H/W 전송]
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                       XM 수신                                │
├─────────────────────────────────────────────────────────────┤
│ 파일: imu_hub_drv.c                                          │
│ 함수: ImuHub_Drv_ProcessCANMessage(0x588, [0x60...], ...)    │
│ Line 417-422: fnc_code = 0x580 (SDO Response)                │
│               AGR_PnP_ProcessMessage()                       │
│                 ↓                                            │
│               AGR_DOP_ProcessRxMessage(master_pnp->dop_ctx)  │
│                 ↓                                            │
│               _OnSdoResponse(response)                       │
│   ↓                                                          │
│ 함수: _OnSdoResponse(response)                               │
│ Line 707-719: cs & 0xE0 == 0x60 (ACK Download)               │
│               response->index == 0x2010 확인                 │
│               ✅ TPDO1 Mapping 완료!                         │
│                 ↓                                            │
│               pre_op_state = SEND_PDO_MAP_B (3)              │
│               sdo_retry_count = 0                            │
│                                                              │
│ ✅ 다음 단계로!                                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 🔄 **Phase 2-2: PDO Mapping B (동일 흐름)**

```
XM: _Step_SendPdoMapB() → SDO Write (0x2011)
  ↓
IMU Hub: _OnPdoMappingB_Set() → tx_pdo_map[1] 설정
  ↓
XM: _OnSdoResponse() → pre_op_state = SEND_IMU_MASK_REQ (5)
```

---

## 🔄 **Phase 2-3: IMU Mask Read**

```
XM: _Step_SendImuMaskReq() → SDO Read (0x2000)
  ↓
IMU Hub: _OnSdoRequest() → SDO Upload Response
  ↓
XM: _OnSdoResponse() → imu_connected_mask = 0x3F
                    → pre_op_state = SEND_NMT_START (7)
```

---

## 🚀 **Phase 2-4: NMT START**

```
┌─────────────────────────────────────────────────────────────┐
│                      XM → IMU Hub                            │
├─────────────────────────────────────────────────────────────┤
│ 파일: imu_hub_drv.c                                          │
│ 함수: _PnP_RunPreOp()                                        │
│   ↓                                                          │
│ 함수: _Step_SendNmtStart(0x08, master_pnp)                   │
│ Line 667-670: AGR_PnP_SendNmtCommand(inst, 0x08, START)      │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_SendNmtCommand()                               │
│ Line 175-196: NMT 메시지 구성                                │
│               nmt_msg[0] = 0x01 (START)                      │
│               nmt_msg[1] = 0x08 (Target Node ID)             │
│   ↓                                                          │
│ Line 196: tx_func(0x000, nmt_msg, 2)                         │
│   ↓                                                          │
│ CAN ID: 0x000                                                │
│ Data: [0x01, 0x08]                                           │
│   ↓                                                          │
│ Line: pre_op_state = COMPLETE (8) ✅                         │
└─────────────────────────────────────────────────────────────┘
                              ↓
                      [FDCAN H/W 전송]
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                     IMU Hub 수신                             │
├─────────────────────────────────────────────────────────────┤
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_ProcessCANMessage(0x000, [0x01, 0x08], ...)     │
│ Line 579-584: can_id == 0x000 확인                           │
│               ✅ NMT Command!                                │
│                 ↓                                            │
│               AGR_NMT_ProcessMessage(&s_imu_hub_nmt, ...)    │
│   ↓                                                          │
│ 파일: agr_nmt.c                                              │
│ 함수: AGR_NMT_ProcessMessage()                               │
│ Line 179-189: can_id == 0x000 확인                           │
│               cmd_byte = 0x01 (START)                        │
│               target_node_id = 0x08                          │
│                 ↓                                            │
│               inst->node_id == 0x08 ✅ 일치!                 │
│                 ↓                                            │
│               AGR_NMT_ProcessCommand(inst, START)            │
│   ↓                                                          │
│ 함수: AGR_NMT_ProcessCommand()                               │
│ Line 255-273: switch (cmd) {                                 │
│                 case AGR_NMT_CMD_START:                      │
│                   if (old_state == PRE_OP)                   │
│                     new_state = OPERATIONAL ✅               │
│               }                                              │
│                 ↓                                            │
│               inst->state = OPERATIONAL                      │
│               _NotifyStateChange(PRE_OP → OPERATIONAL)       │
│   ↓                                                          │
│ ✅ s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL (0x05)          │
│                                                              │
│ ✅ Slave OPERATIONAL 진입!                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 💓 **Phase 3: Heartbeat 교환**

```
┌─────────────────────────────────────────────────────────────┐
│                  IMU Hub → XM (매 1초)                       │
├─────────────────────────────────────────────────────────────┤
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_RunPeriodic()                                   │
│ Line 546-552: AGR_PnP_RunPeriodic(&s_imu_hub_pnp_inst)       │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_RunPeriodic() - Slave 로직                     │
│ Line 488-499: role == SLAVE                                  │
│               current - last_hb_sent >= 1000ms?              │
│                 ↓                                            │
│               AGR_PnP_SendHeartbeat(inst)                    │
│   ↓                                                          │
│ 함수: AGR_PnP_SendHeartbeat()                                │
│ Line 137-148: CAN: 0x708, [0x05]                             │
│               ✅ IMU Hub Heartbeat 전송!                     │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                    XM → IMU Hub (매 1초)                     │
├─────────────────────────────────────────────────────────────┤
│ 파일: imu_hub_drv.c                                          │
│ 함수: ImuHub_Drv_RunPeriodic()                               │
│ Line 459-462: AGR_PnP_RunPeriodic(master_pnp)                │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: AGR_PnP_RunPeriodic() - Master 로직                    │
│ Line 475-484: role == MASTER                                 │
│               current - last_hb_sent >= 1000ms?              │
│                 ↓                                            │
│               AGR_PnP_SendHeartbeat(inst)                    │
│   ↓                                                          │
│ 함수: AGR_PnP_SendHeartbeat()                                │
│ Line 137-148: CAN: 0x702, [0x05]                             │
│               ✅ XM Heartbeat 전송!                          │
└─────────────────────────────────────────────────────────────┘
                              ↓
                      [IMU Hub 수신]
                              ↓
┌─────────────────────────────────────────────────────────────┐
│             IMU Hub: Master Heartbeat 처리                   │
├─────────────────────────────────────────────────────────────┤
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_ProcessCANMessage(0x702, [0x05], ...)           │
│ Line 586-606: fnc_code = 0x700                               │
│               sender_node_id = 0x02 (XM)                     │
│                 ↓                                            │
│               sender == AGR_NODE_ID_XM ✅                    │
│                 ↓                                            │
│               AGR_NMT_ProcessMessage(                        │
│                 &s_imu_hub_pnp_inst.config.nmt, ...)         │
│   ↓                                                          │
│ 파일: agr_nmt.c                                              │
│ 함수: AGR_NMT_ProcessMessage()                               │
│ Line 237-253: Heartbeat 처리 (data[0] = 0x05)                │
│               inst->state = OPERATIONAL                      │
│               AGR_NMT_UpdateActivity()                       │
│               _NotifyStateChange() ← 항상 호출!              │
│   ↓                                                          │
│ 파일: agr_pnp.c                                              │
│ 함수: _OnSlaveNmtStateChanged()                              │
│ Line: inst->last_master_hb_time = current_ms ✅              │
│       inst->is_master_connected = true ✅                    │
│                                                              │
│ ✅ Master 연결 상태 갱신!                                     │
└─────────────────────────────────────────────────────────────┘
```

---

## 📤 **Phase 4: PDO 전송 (매 1ms)**

```
┌─────────────────────────────────────────────────────────────┐
│                 IMU Hub: 1ms ISR                             │
├─────────────────────────────────────────────────────────────┤
│ 파일: core_process.c                                         │
│ 함수: Core_RunLoop()                                         │
│ Line 197-251: 매 1ms마다 실행                                │
│   ↓                                                          │
│ Line 213: _FetchInputs()                                     │
│   ↓                                                          │
│ 함수: _FetchInputs()                                         │
│ Line 265-283: UART에서 IMU 데이터 수집                       │
│   ↓                                                          │
│ Line 267: UartRxHandler_FetchAllImus(temp_imus)              │
│   ↓                                                          │
│ Line 270-272: g_sys.imus[i].data = temp_imus[i]              │
│   ↓                                                          │
│ Line 275-279: IMU 연결 상태 확인                             │
│               for (i = 0; i < 6; i++)                        │
│                 is_connected = ebimu9dofv6.IsConnected(...)  │
│                 g_sys.imus[i].is_connected = is_connected    │
│                 ⚠️⚠️⚠️ 여기가 false면 PDO 데이터 0!           │
│   ↓                                                          │
│ Line 218: User_Loop()                                        │
│   ↓                                                          │
│ Line 233: _FlushOutputs()                                    │
│   ↓                                                          │
│ 함수: _FlushOutputs()                                        │
│ Line 300-302: if (!XM_Drv_IsConnected()) return              │
│               ✅ 통과 (OPERATIONAL + Master 연결)             │
│   ↓                                                          │
│ Line 305-306: XM_Drv_SetFrameTimestamp(tick_cnt)             │
│               XM_Drv_ClearValidMask()                        │
│   ↓                                                          │
│ Line 314-353: for (i = 0; i < 6; i++)                        │
│   ↓                                                          │
│ Line 317: if (g_sys.imus[i].is_connected)                    │
│             ⚠️⚠️⚠️ 여기서 분기!                              │
│   ↓                      ↓                                   │
│ true                   false                                 │
│   ↓                      ↓                                   │
│ Line 322-340:          Line 348:                             │
│ drv_data.q[0] =        memset(&drv_data, 0)                  │
│   QuatToInt16(q_w)     ❌ 0으로 채움!                        │
│ drv_data.a[0] =                                              │
│   AccelToInt16(acc_x)                                        │
│ drv_data.g[0] =                                              │
│   GyroToInt16(gyr_x)                                         │
│   ↓                      ↓                                   │
│   └──────────┬───────────┘                                   │
│              ↓                                               │
│ Line 352: XM_Drv_UpdateImuData(i, &drv_data)                 │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_UpdateImuData()                                 │
│ Line 650-658: memcpy(&s_imu_hub_tx_data.imu[i],              │
│                      &drv_data, ...)                         │
│               ✅ 저장 완료!                                   │
│   ↓                                                          │
│ 파일: core_process.c                                         │
│ Line 356-362: connected_mask 계산                            │
│               XM_Drv_SetImuConnectedMask(mask)               │
│   ↓                                                          │
│ Line 383-388: PDO 전송 (번갈아)                              │
│               if (s_pdo_toggle)                              │
│                 XM_Drv_SendTPDO2()                            │
│               else                                           │
│                 XM_Drv_SendTPDO1()                            │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ 함수: XM_Drv_SendTPDO1()                                     │
│ Line 673-697: AGR_DOP_EncodeTxPDON(&s_dop_ctx, 1, ...)       │
│   ↓                                                          │
│ 파일: agr_dop.c                                              │
│ 함수: AGR_DOP_EncodeTxPDON()                                 │
│ Line 576-606: for (i = 0; i < map->count; i++)               │
│                 entry = FindODEntry(item->index, sub)        │
│                 memcpy(out_buf, entry->data_ptr, size)       │
│                 ⚠️ entry->data_ptr = &s_imu_hub_tx_data      │
│   ↓                                                          │
│ return len = 64                                              │
│   ↓                                                          │
│ 파일: xm_drv.c                                               │
│ Line 695-696: tx_func(0x188, pdo_buf, 64)                    │
│               ✅ CAN 전송!                                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 **결론**

**순서도 기준 핵심 체크포인트:**

```
Phase 4 - Line 317:
  ↓
if (g_sys.imus[i].is_connected)  ← ⚠️⚠️⚠️ 여기!
  ↓
  false → PDO 데이터 = 0 ❌
  true  → PDO 데이터 = 실제값 ✅
```

**Live Expression 1개만 확인하세요:**

```c
g_sys.imus[0].is_connected
```

- **`false`**: UART 센서 문제! (확정)
- **`true`**: 다음 단계 확인 필요 (`g_sys.imus[0].data.q_w`)

---

**이 값을 알려주시면 즉시 해결하겠습니다!** 🔥
