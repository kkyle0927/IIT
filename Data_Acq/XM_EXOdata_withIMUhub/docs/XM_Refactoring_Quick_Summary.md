# XM (Master) 빠른 리팩토링 전략

**작성일**: 2025-12-08  
**목표**: 검증된 PnP 로직 유지하면서 CANopen 표준 적용

---

## 🎯 핵심 전략

### **검증된 Pre-Op 로직 유지** ✅

`imu_hub_xm_link_v2.c`의 Pre-Op 상태 머신을 `imu_hub_drv.c`로 복사:

```c
/* Pre-Op 상태 */
typedef enum {
    IMUHUB_PRE_OP_IDLE,
    IMUHUB_PRE_OP_SEND_PDO_MAP_A,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,
    IMUHUB_PRE_OP_SEND_PDO_MAP_B,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,
    IMUHUB_PRE_OP_SEND_IMU_MASK_REQ,
    IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP,
    IMUHUB_PRE_OP_SEND_NMT_START,
    IMUHUB_PRE_OP_COMPLETE
} ImuHub_PreOpState_t;
```

---

## 📋 최소 변경사항 (빌드 성공 목표)

### **1. Legacy 코드 제거**

#### `ImuHub_DrvInst_t` 구조체
```c
/* Before (Legacy) */
typedef struct {
    ImuHub_Callbacks_t callbacks;  /* ❌ 제거 */
    ImuHub_TxData_t tx_data;       /* ❌ Master는 rx만 사용 */
    ...
} ImuHub_DrvInst_t;

/* After (CANopen 표준) */
typedef struct {
    AGR_NMT_Inst_t nmt;           /* ✅ 추가 */
    AGR_PnP_Inst_t pnp_inst;      /* ✅ 추가 */
    ImuHub_PreOpState_t pre_op_state;  /* ✅ Pre-Op 상태 */
    ImuHub_RxData_t rx_data;      /* ✅ Master는 rx만 */
    ...
} ImuHub_DrvInst_t;
```

#### OD Index 정의
```c
/* ❌ 제거 (사용 안함) */
#define IMUHUB_OD_IDX_NMT_STATE     0x1000
#define IMUHUB_OD_IDX_SYNC_STATES   0x1002

/* ✅ 이미 정의됨 */
#define IMUHUB_OD_IDX_IMU_CONN_MASK 0x2000
#define IMUHUB_OD_IDX_PDO_MAPPING_A 0x2010
#define IMUHUB_OD_IDX_PDO_MAPPING_B 0x2011
```

---

### **2. ImuHub_Drv_Init() 수정**

```c
/* Before (Legacy) */
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    s_inst.callbacks = *callbacks;  /* ❌ */
    s_inst.tx_data = ...;  /* ❌ */
    ...
}

/* After (CANopen 표준) */
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    (void)callbacks;  /* ✅ Deprecated, AGR_PnP 사용 */
    
    /* AGR_NMT 초기화 */
    AGR_NMT_InitEx(&s_inst.nmt, 3000, AGR_NODE_ID_IMU_HUB, NULL, NULL, NULL);
    
    /* AGR_DOP 초기화 */
    AGR_DOP_Init(&s_inst.dop_ctx, NULL, AGR_NODE_ID_XM, tx_func);
    
    /* AGR_PnP 초기화 */
    AGR_PnP_Config_t pnp_config = {
        .role = AGR_PNP_ROLE_MASTER,
        .my_node_id = AGR_NODE_ID_XM,
        .tx_func = tx_func,
        .callbacks = {
            .on_connected = _PnP_OnConnected,
            .on_disconnected = _PnP_OnDisconnected,
            .on_bootup = _PnP_OnBootup,
            .on_nmt_change = _PnP_OnNmtChange,
            .on_run_pre_op = _PnP_RunPreOp,  /* ✅ 검증된 Pre-Op 로직 */
            .on_error = _PnP_OnError
        }
    };
    AGR_PnP_Init(&s_inst.pnp_inst, &pnp_config, IOIF_TIM_GetTick);
    AGR_PnP_RegisterDevice(&s_inst.pnp_inst, &imu_hub_device);
}
```

---

### **3. Legacy 콜백 제거**

```c
/* ❌ 제거 (사용 안함) */
static void _ImuHub_OnSdoBootup(...) { ... }
static void _ImuHub_OnSdoHeartbeat(...) { ... }
static void _ImuHub_OnSdoSyncStates(...) { ... }
static void _ImuHub_OnPdoMappingAck(...) { ... }

/* ✅ AGR_PnP 콜백으로 대체 */
static void _PnP_OnBootup(uint8_t node_id) {
    /* Pre-Op 시작 */
    s_inst.pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_A;
}

static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst) {
    /* ✅ imu_hub_xm_link_v2.c의 _Link_RunPreOp 로직 복사 */
    switch (s_inst.pre_op_state) {
        case IMUHUB_PRE_OP_SEND_PDO_MAP_A:
            /* SDO Write: 0x2010 */
            ...
            break;
        case IMUHUB_PRE_OP_WAIT_PDO_MAP_A:
            /* SDO Response 대기 */
            break;
        ...
    }
}
```

---

### **4. ImuHub_Drv_ProcessCANMessage() 수정**

```c
/* Before (Legacy) */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    if (can_id == ...) {
        if (s_inst.callbacks.OnBootup) {  /* ❌ */
            s_inst.callbacks.OnBootup();
        }
    }
}

/* After (CANopen 표준) */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    /* 1. Boot-up / Heartbeat (CAN ID 0x700 + Node ID) */
    uint16_t fnc_code = can_id & 0x780;
    if (fnc_code == 0x700) {
        AGR_NMT_ProcessMessage(&s_inst.nmt, can_id, data, len, current_ms);
        return;
    }
    
    /* 2. SDO Response (CAN ID 0x580 + Node ID) */
    if (fnc_code == 0x580) {
        AGR_PnP_ProcessMessage(&s_inst.pnp_inst, (uint32_t)can_id, data, len);
        return;
    }
    
    /* 3. PDO (CAN ID 0x180/0x280 + Node ID) */
    if (fnc_code == 0x180 || fnc_code == 0x280) {
        /* PDO 처리 with Mutex */
        return;
    }
}
```

---

### **5. Public API 추가**

```c
bool ImuHub_Drv_IsConnected(void)
{
    return AGR_NMT_IsConnected(&s_inst.nmt);
}

AGR_NMT_State_t ImuHub_Drv_GetNmtState(void)
{
    return AGR_NMT_GetState(&s_inst.nmt);
}

void ImuHub_Drv_RunPeriodic(void)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    AGR_NMT_CheckTimeout(&s_inst.nmt, current_ms);
    AGR_PnP_RunPeriodic(&s_inst.pnp_inst);
    
    /* Pre-Op Timeout 체크 (검증된 로직) */
    if (s_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_A ||
        s_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_PDO_MAP_B ||
        s_inst.pre_op_state == IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP) {
        if (current_ms - s_inst.last_sdo_tx_time > 1000) {
            if (s_inst.sdo_retry_count < 3) {
                s_inst.sdo_retry_count++;
                /* 재시도 로직 */
            }
        }
    }
}
```

---

## 🚀 작업 순서

1. ✅ **agr_pnp.c 수정** (이미 완료)
2. ⏳ **imu_hub_drv.c 구조체 수정**
   - `callbacks`, `tx_data` 제거
   - `AGR_NMT_Inst_t`, `AGR_PnP_Inst_t` 추가
3. ⏳ **ImuHub_Drv_Init() 리팩토링**
   - AGR_NMT, AGR_PnP 초기화
4. ⏳ **Pre-Op 로직 복사**
   - `_PnP_RunPreOp()` 구현
5. ⏳ **ImuHub_Drv_ProcessCANMessage() 수정**
   - CAN ID 기반 분기
6. ⏳ **Public API 추가**
   - `IsConnected`, `GetNmtState`, `RunPeriodic`

---

## ✅ 검증 완료 후

1. ✅ XM 빌드 성공
2. ✅ XM-IMU 연결 테스트
3. ⏳ **Step Array 패턴** (선택사항, 나중에)

---

**핵심**: 검증된 Pre-Op 로직은 그대로 유지하고, CANopen 표준 메시지 처리만 통합! ✅
