# XM ↔ IMU Hub PnP 완전 순서도

**작성일**: 2025-12-08  
**목적**: XM-IMU Hub PnP 단계를 코드와 함께 완벽하게 이해

---

## 📊 **전체 순서도**

```
┌─────────────────────────────────────────────────────────────────┐
│                    Phase 0: 초기화 (Power-on)                    │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: System_Startup()          │
        │   XM: System_Startup()               │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: XM_Drv_Init()             │
        │   - s_imu_hub_nmt 초기화             │
        │     node_id = 0x08                   │
        │   - s_imu_hub_pnp_inst 초기화        │
        │     role = SLAVE                     │
        │   - s_dop_ctx 초기화                 │
        │   - s_imu_hub_tx_data = 0            │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: ImuHub_Drv_Init()              │
        │   - master_pnp 초기화                │
        │     role = MASTER                    │
        │   - devices[0] 등록                  │
        │     node_id = 0x08                   │
        │   - s_imu_hub_inst.pre_op_state = 0  │
        └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                Phase 1: Boot-up 전송 (IMU Hub)                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: AGR_PnP_SendBootup()      │
        │   CAN ID: 0x708                      │
        │   Data: [0x00]                       │
        └──────────────────────────────────────┘
                              ↓
                      [CAN Bus 전송]
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: 수신 (FDCAN ISR)               │
        │   HAL_FDCAN_RxFifo0Callback()        │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: CANFD_RxHandler_ProcessFrame() │
        │   → ImuHub_Drv_ProcessCANMessage()   │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: AGR_PnP_ProcessMessage()       │
        │   → AGR_NMT_ProcessMessage()         │
        │     devices[0].nmt.state = BOOT_UP   │
        │   → AGR_NMT_UpdateActivity()         │
        │     devices[0].nmt.state = PRE_OP    │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _NotifyStateChange()           │
        │   → agr_pnp.c - _OnNmtStateChanged() │
        │   → _PnP_OnBootup(0x08)              │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: s_imu_hub_inst.pre_op_state    │
        │   = IMUHUB_PRE_OP_SEND_PDO_MAP_A (1) │
        └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│         Phase 2: Pre-Op 시퀀스 (XM → IMU Hub)                    │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ╔══════════════════════════════════════╗
        ║  Step 1: PDO Mapping A 전송           ║
        ╚══════════════════════════════════════╝
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: ImuHub_Drv_RunPeriodic()       │
        │   → AGR_PnP_RunPeriodic(master_pnp)  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: devices[0].nmt.state == PRE_OP │
        │   → _RunMasterPreOpStateMachine()    │
        │   → _PnP_RunPreOp(0x08, master_pnp)  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: Step Array 실행                │
        │   pre_op_state == SEND_PDO_MAP_A     │
        │   → _Step_SendPdoMapA(0x08, ...)     │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: AGR_PnP_SendSDOWrite()         │
        │   node_id: 0x08                      │
        │   index: 0x2010 (PDO Mapping A)      │
        │   data: s_pdo_map_a[] (17 bytes)     │
        │     [0x04,                           │
        │      0x20,0x00,0x00,0x30,  ← 0x3000.0x00 (Metadata)
        │      0xA0,0x60,0x00,0x60,  ← 0x6000.0x60 (IMU 0)
        │      0xA0,0x60,0x01,0x60,  ← 0x6001.0x60 (IMU 1)
        │      0xA0,0x60,0x02,0x60]  ← 0x6002.0x60 (IMU 2)
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: CAN 전송                       │
        │   CAN ID: 0x608 (0x600 + 0x08)       │
        │   Data: [SDO Download, 0x10, 0x20, ...]
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: pre_op_state                   │
        │   = IMUHUB_PRE_OP_WAIT_PDO_MAP_A (2) │
        │   last_sdo_tx_time = current_ms      │
        └──────────────────────────────────────┘
                              ↓
                      [CAN Bus 전송]
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: 수신 (FDCAN ISR)          │
        │   HAL_FDCAN_RxFifo0Callback()        │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: CANFD_RxHandler_ProcessFrame()
        │   → XM_Drv_ProcessCANMessage()       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: CAN ID 0x608 확인         │
        │   fnc_code = 0x600 (SDO)             │
        │   → AGR_PnP_ProcessMessage()         │
        │   → AGR_DOP_ProcessRxMessage()       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: _OnSdoRequest()           │
        │   request->index = 0x2010            │
        │   request->subindex = 0x00           │
        │   request->data_len = 17             │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: OD Entry 찾기             │
        │   _FindODEntry(0x2010, 0x00)         │
        │   → s_od_entries[...]                │
        │     { 0x2010, 0x00, BLOB, 64, WO,    │
        │       s_pdo_mapping_a,               │
        │       _OnPdoMappingA_Set }  ← 찾음!  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: SDO Write 처리            │
        │   memcpy(s_pdo_mapping_a,            │
        │          request->data, 17)          │
        │   ✅ s_pdo_mapping_a[] 저장 완료     │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: WriteCb 호출              │
        │   _OnPdoMappingA_Set()               │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: PDO Mapping 적용          │
        │   AGR_DOP_ClearTxPDOMapN(&s_dop_ctx, 1)
        │   AGR_DOP_ApplyTxPDOMapFromSDON(     │
        │       &s_dop_ctx, 1,                 │
        │       s_pdo_mapping_a, 64)           │
        │                                      │
        │   ✅ s_dop_ctx.tx_pdo_map[0] 설정:   │
        │     count = 4                        │
        │     items[0] = {0x3000, 0x00, ...}   │
        │     items[1] = {0x6000, 0x60, ...}   │
        │     items[2] = {0x6001, 0x60, ...}   │
        │     items[3] = {0x6002, 0x60, ...}   │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: SDO Response 전송         │
        │   CAN ID: 0x588 (0x580 + 0x08)       │
        │   Data: [0x60, 0x10, 0x20, 0x00]     │
        │         ↑ ACK                        │
        └──────────────────────────────────────┘
                              ↓
                      [CAN Bus 전송]
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: 수신 (FDCAN ISR)               │
        │   → ImuHub_Drv_ProcessCANMessage()   │
        │   → AGR_PnP_ProcessMessage()         │
        │   → AGR_DOP_ProcessRxMessage()       │
        │   → _OnSdoResponse()                 │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: SDO Response 확인              │
        │   response->cs & 0xE0 == 0x60 (ACK)  │
        │   response->index == 0x2010          │
        │   → PDO Mapping A 완료!              │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: pre_op_state                   │
        │   = IMUHUB_PRE_OP_SEND_PDO_MAP_B (3) │
        │   sdo_retry_count = 0                │
        └──────────────────────────────────────┘
                              ↓
        ╔══════════════════════════════════════╗
        ║  Step 2: PDO Mapping B 전송           ║
        ║  (Step 1과 동일한 흐름)               ║
        ╚══════════════════════════════════════╝
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _Step_SendPdoMapB(0x08, ...)   │
        │   → SDO Write (0x2011)               │
        │   → s_pdo_map_b[] 전송               │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: _OnPdoMappingB_Set()      │
        │   ✅ s_dop_ctx.tx_pdo_map[1] 설정:   │
        │     count = 4                        │
        │     items[0] = {0x3000, 0x00, ...}   │
        │     items[1] = {0x6003, 0x60, ...}   │
        │     items[2] = {0x6004, 0x60, ...}   │
        │     items[3] = {0x6005, 0x60, ...}   │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _OnSdoResponse()               │
        │   response->index == 0x2011          │
        │   → PDO Mapping B 완료!              │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: pre_op_state                   │
        │   = IMUHUB_PRE_OP_SEND_IMU_MASK_REQ (5)
        └──────────────────────────────────────┘
                              ↓
        ╔══════════════════════════════════════╗
        ║  Step 3: IMU Connected Mask 조회      ║
        ║  (Optional)                          ║
        ╚══════════════════════════════════════╝
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _Step_SendImuMaskReq(0x08, ...)│
        │   → SDO Read (0x2000)                │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: SDO Upload Response       │
        │   response->data[0] = 0x3F (0b111111)│
        │   → IMU 0~5 모두 연결                │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _OnSdoResponse()               │
        │   imu_connected_mask = 0x3F          │
        │   pre_op_state                       │
        │   = IMUHUB_PRE_OP_SEND_NMT_START (7) │
        └──────────────────────────────────────┘
                              ↓
        ╔══════════════════════════════════════╗
        ║  Step 4: NMT START 전송               ║
        ╚══════════════════════════════════════╝
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: _Step_SendNmtStart(0x08, ...)  │
        │   → AGR_PnP_SendNmtCommand()         │
        │     (master_pnp, 0x08, START)        │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: CAN 전송                       │
        │   CAN ID: 0x000 (NMT Command)        │
        │   Data: [0x01, 0x08]                 │
        │          ↑     ↑                     │
        │          START  Target Node ID       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: pre_op_state                   │
        │   = IMUHUB_PRE_OP_COMPLETE (8)       │
        └──────────────────────────────────────┘
                              ↓
                      [CAN Bus 전송]
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: 수신 (FDCAN ISR)          │
        │   CAN ID: 0x000                      │
        │   Data: [0x01, 0x08]                 │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: XM_Drv_ProcessCANMessage()│
        │   can_id == 0x000 ✅                 │
        │   → AGR_NMT_ProcessMessage(          │
        │       &s_imu_hub_nmt, ...)           │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: NMT Command 처리          │
        │   target_node_id = 0x08              │
        │   s_imu_hub_nmt.node_id = 0x08 ✅    │
        │   → 일치! 처리 진행                  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: AGR_NMT_ProcessCommand()  │
        │   cmd = AGR_NMT_CMD_START (0x01)     │
        │   old_state = PRE_OPERATIONAL        │
        │   new_state = OPERATIONAL ✅         │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: s_imu_hub_nmt.state       │
        │   = AGR_NMT_OPERATIONAL (0x05) ✅    │
        └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│         Phase 3: Heartbeat 교환 (양방향)                         │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: AGR_PnP_RunPeriodic()     │
        │   (매 1초마다)                       │
        │   → AGR_PnP_SendHeartbeat()          │
        │     CAN ID: 0x708                    │
        │     Data: [0x05] (OPERATIONAL)       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: Heartbeat 수신                 │
        │   → devices[0].nmt.state = OPERATIONAL
        │   → devices[0].nmt.last_activity_ms  │
        │     = current_ms                     │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: AGR_PnP_RunPeriodic()          │
        │   (매 1초마다)                       │
        │   → AGR_PnP_SendHeartbeat()          │
        │     CAN ID: 0x702                    │
        │     Data: [0x05] (OPERATIONAL)       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: Master Heartbeat 수신     │
        │   CAN ID: 0x702                      │
        │   fnc_code = 0x700                   │
        │   sender_node_id = 0x02 (XM) ✅      │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: AGR_NMT_ProcessMessage()  │
        │   → config.nmt로 처리 (Master 추적)  │
        │   config.nmt.state = OPERATIONAL     │
        │   config.nmt.last_activity_ms = ...  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: _NotifyStateChange()      │
        │   → _OnSlaveNmtStateChanged()        │
        │   is_master_connected = true ✅      │
        │   last_master_hb_time = current_ms   │
        └──────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│         Phase 4: PDO 전송 (IMU Hub → XM)                         │
└─────────────────────────────────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   IMU Hub: Core_RunLoop()            │
        │   (매 1ms마다)                       │
        │   TIM ISR → Core_RunLoop()           │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [1] _FetchInputs()                 │
        │   ├─ UART에서 IMU 데이터 수집        │
        │   │  UartRxHandler_FetchAllImus()    │
        │   │  → g_sys.imus[0~5].data          │
        │   │                                  │
        │   └─ IMU 연결 상태 확인              │
        │      ebimu9dofv6.IsConnected()       │
        │      → g_sys.imus[i].is_connected    │
        │        ⚠️ 여기가 false면 PDO 데이터 0!
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [2] User_Loop()                    │
        │   (사용자 제어 로직)                 │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3] _FlushOutputs()                │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3-1] XM_Drv_IsConnected() 체크    │
        │   → (s_imu_hub_nmt.state == OP)      │
        │      && is_master_connected          │
        │   ✅ true → 진행                     │
        │   ❌ false → return (PDO 안보냄)     │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3-2] Metadata 설정                │
        │   XM_Drv_SetFrameTimestamp(tick_cnt) │
        │   → s_pdo_metadata.timestamp = ...   │
        │   XM_Drv_ClearValidMask()            │
        │   → s_pdo_metadata.valid_mask = 0    │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3-3] IMU 데이터 변환 및 업데이트  │
        │   for (i = 0; i < 6; i++)            │
        └──────────────────────────────────────┘
                              ↓
                 ┌─────────────────────┐
                 │ g_sys.imus[i]       │
                 │ .is_connected?      │
                 └─────────────────────┘
                  ↓                ↓
            ✅ true            ❌ false
                  ↓                ↓
        ┌─────────────────┐  ┌─────────────────┐
        │ UART 데이터 변환 │  │ memset(0)       │
        │ q_w → q[0]      │  │ drv_data = 0    │
        │ q_x → q[1]      │  │                 │
        │ acc_x → a[0]    │  │                 │
        │ gyr_x → g[0]    │  │                 │
        └─────────────────┘  └─────────────────┘
                  ↓                ↓
                  └────────┬────────┘
                           ↓
        ┌──────────────────────────────────────┐
        │   XM_Drv_UpdateImuData(i, &drv_data) │
        │   → memcpy(&s_imu_hub_tx_data.imu[i],│
        │            &drv_data, ...)            │
        │                                      │
        │   ✅ s_imu_hub_tx_data.imu[i] 저장   │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3-4] Valid Mask 설정              │
        │   connected_mask = 0                 │
        │   for (i = 0; i < 6; i++)            │
        │     if (g_sys.imus[i].is_connected)  │
        │       connected_mask |= (1 << i)     │
        │   XM_Drv_SetImuConnectedMask(mask)   │
        │   → s_imu_hub_rx_data.imu_connected_mask
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   [3-5] PDO 전송 (번갈아)            │
        │   if (s_pdo_toggle)                  │
        │     XM_Drv_SendTPDO2()               │
        │   else                               │
        │     XM_Drv_SendTPDO1()               │
        └──────────────────────────────────────┘
                              ↓
                  ┌───────────────────┐
                  │ TPDO1 전송?       │
                  └───────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM_Drv_SendTPDO1()                 │
        │   → AGR_DOP_EncodeTxPDON(ctx, 1, ...)│
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   AGR_DOP_EncodeTxPDON()             │
        │   map = &ctx->tx_pdo_map[0]          │
        │   ✅ count = 4                       │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   for (i = 0; i < 4; i++)            │
        │     item = &map->items[i]            │
        │     ├─ item[0]: index=0x3000, sub=0x00
        │     ├─ item[1]: index=0x6000, sub=0x60
        │     ├─ item[2]: index=0x6001, sub=0x60
        │     └─ item[3]: index=0x6002, sub=0x60
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   각 item마다:                       │
        │   entry = FindODEntry(index, sub)    │
        │   → s_od_entries에서 찾기            │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   item[0]: 0x3000.0x00 찾기          │
        │   → entry->data_ptr = &s_pdo_metadata│
        │   → memcpy(pdo_buf[0~3],             │
        │            s_pdo_metadata, 4)        │
        │   ✅ Metadata 복사 (4B)              │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   item[1]: 0x6000.0x60 찾기          │
        │   → entry->data_ptr =                │
        │     &s_imu_hub_tx_data.imu[0].q      │
        │   → memcpy(pdo_buf[4~23],            │
        │            imu[0].q, 20)             │
        │   ⚠️ IMU 0 데이터 복사 (20B)         │
        │     {q[4], a[3], g[3]}               │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   item[2]: 0x6001.0x60 찾기          │
        │   → IMU 1 데이터 복사 (20B)          │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   item[3]: 0x6002.0x60 찾기          │
        │   → IMU 2 데이터 복사 (20B)          │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   AGR_DOP_EncodeTxPDON()             │
        │   return len = 64 (4+20+20+20)       │
        │   ✅ pdo_buf[] 완성                  │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM_Drv_SendTPDO1()                 │
        │   CAN ID: 0x188 (0x180 + 0x08)       │
        │   Data: pdo_buf[0~63]                │
        │   return s_dop_ctx.tx_func(...)      │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   _Drv_TxFunc_Queue()                │
        │   → IOIF_FDCAN_Transmit() 직접 시도  │
        │   ✅ 성공 → 전송 완료                │
        │   ❌ FIFO Full → Queue에 추가        │
        └──────────────────────────────────────┘
                              ↓
                      [CAN Bus 전송]
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: 수신 (FDCAN ISR)               │
        │   CAN ID: 0x188                      │
        │   Data: [64 bytes PDO]               │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: ImuHub_Drv_ProcessCANMessage() │
        │   fnc_code = 0x180 (TPDO1)           │
        │   → AGR_PnP_ProcessMessage()         │
        │   → AGR_DOP_ProcessRxMessage()       │
        │   → _OnPdoReceived()                 │
        │   → _ImuHub_DecodeTpdo1(data, 64)    │
        └──────────────────────────────────────┘
                              ↓
        ┌──────────────────────────────────────┐
        │   XM: TPDO1 디코딩                   │
        │   offset = 0                         │
        │   ├─ Metadata (4B)                   │
        │   │  memcpy(&rx_metadata, data[0~3]) │
        │   ├─ IMU 0 (20B)                     │
        │   │  memcpy(&rx_imu[0], data[4~23])  │
        │   ├─ IMU 1 (20B)                     │
        │   │  memcpy(&rx_imu[1], data[24~43]) │
        │   └─ IMU 2 (20B)                     │
        │      memcpy(&rx_imu[2], data[44~63]) │
        │                                      │
        │   ✅ XM이 IMU 데이터 수신 완료!      │
        └──────────────────────────────────────┘
                              ↓
                    [1ms마다 반복]
```

---

## 🚨 **PDO 데이터가 0인 이유 (체크리스트)**

### **순서도 기준 확인**

```
[Phase 4 - _FetchInputs()]
  ↓
  g_sys.imus[i].is_connected = ?
    ↓
    ❌ false → memset(&drv_data, 0) → s_imu_hub_tx_data.imu[i] = 0
    ↓
    ✅ true → UART 데이터 변환 → s_imu_hub_tx_data.imu[i] = 실제값
```

---

## 📋 **Live Expression 확인 (우선순위)**

### **1순위: UART 센서 연결**
```c
g_sys.imus[0].is_connected  // false면 PDO 데이터 0!
g_sys.imus[1].is_connected
g_sys.imus[2].is_connected
```

### **2순위: UART 데이터**
```c
g_sys.imus[0].data.q_w  // 0이면 UART 데이터 안옴!
g_sys.imus[0].data.acc_x
g_sys.imus[0].data.gyr_x
g_sys.imus[0].last_update_time  // 0이면 한번도 안받음!
```

### **3순위: 변환된 데이터**
```c
s_imu_hub_tx_data.imu[0].q[0]  // 0이면 변환 실패!
s_imu_hub_tx_data.imu[0].a[0]
s_imu_hub_tx_data.imu[0].g[0]
```

### **4순위: PDO Mapping**
```c
s_dop_ctx.tx_pdo_map[0].count  // 0이면 PDO Mapping 안됨!
s_dop_ctx.tx_pdo_map[0].items[1].od_index  // 0x6000이어야 함
```

---

## 🎯 **코드 위치 맵**

| 단계 | 파일 | 함수 | 변수 |
|------|------|------|------|
| **초기화** | xm_drv.c (IMU Hub) | `XM_Drv_Init()` | `s_imu_hub_nmt`, `s_imu_hub_pnp_inst` |
| **초기화** | imu_hub_drv.c (XM) | `ImuHub_Drv_Init()` | `master_pnp`, `devices[0]` |
| **Boot-up** | agr_pnp.c (IMU Hub) | `AGR_PnP_SendBootup()` | - |
| **Pre-Op** | imu_hub_drv.c (XM) | `_PnP_RunPreOp()` | `pre_op_state` |
| **PDO Mapping** | xm_drv.c (IMU Hub) | `_OnPdoMappingA_Set()` | `tx_pdo_map[0]` |
| **NMT START** | agr_nmt.c (IMU Hub) | `AGR_NMT_ProcessCommand()` | `s_imu_hub_nmt.state` |
| **Heartbeat** | agr_pnp.c (공통) | `AGR_PnP_SendHeartbeat()` | `last_hb_sent_time` |
| **데이터 수집** | core_process.c (IMU Hub) | `_FetchInputs()` | `g_sys.imus[]` |
| **데이터 변환** | core_process.c (IMU Hub) | `_FlushOutputs()` | `s_imu_hub_tx_data` |
| **PDO 전송** | xm_drv.c (IMU Hub) | `XM_Drv_SendTPDO1()` | - |
| **PDO 인코딩** | agr_dop.c (IMU Hub) | `AGR_DOP_EncodeTxPDON()` | `tx_pdo_map[0]` |
| **PDO 수신** | imu_hub_drv.c (XM) | `_ImuHub_DecodeTpdo1()` | `rx_imu[]` |

---

## 🔍 **데이터 흐름 (IMU Hub)**

```
[UART] ───→ [uart_rx_handler] ───→ [g_sys.imus[].data]
                                             ↓
                                    [ebimu9dofv6.IsConnected()]
                                             ↓
                                    [g_sys.imus[].is_connected]
                                             ↓
                                          [분기]
                                    ↓              ↓
                                true            false
                                    ↓              ↓
                            [데이터 변환]      [memset(0)]
                                    ↓              ↓
                                    └──────┬───────┘
                                           ↓
                              [s_imu_hub_tx_data.imu[]]
                                           ↓
                              [AGR_DOP_EncodeTxPDON()]
                                           ↓
                                    [pdo_buf[64]]
                                           ↓
                              [FDCAN Tx (0x188/0x288)]
                                           ↓
                                       [CAN Bus]
                                           ↓
                                        [XM 수신]
```

---

## ⚠️ **99% 확신: UART 센서 문제**

**PDO 데이터가 0인 이유:**

```c
/* core_process.c - _FlushOutputs() */
if (g_sys.imus[i].is_connected) {  // ⚠️ 이게 false!
    /* 정상 데이터 변환 */
    drv_data.q[0] = XM_Drv_QuatToInt16(ebimu->q_w);
    ...
} else {
    memset(&drv_data, 0, sizeof(XM_Drv_ImuData_t));  // ❌ 0으로 채움!
}
```

**확인:**
1. Live Expression: `g_sys.imus[0].is_connected`
2. Live Expression: `g_sys.imus[0].last_update_time`
3. Live Expression: `g_sys.imus[0].data.q_w`

---

## ✅ **즉시 확인!**

```c
g_sys.imus[0].is_connected = ?  // ⚠️ false면 UART 센서 문제!
```

**만약 `false`라면:**
- UART 센서가 물리적으로 연결 안됨
- UART 초기화 안됨
- UART 통신 실패
- Timeout 발생

**만약 `true`라면:**
- `g_sys.imus[0].data.q_w` 확인
- `s_imu_hub_tx_data.imu[0].q[0]` 확인
- 변환 로직 확인
