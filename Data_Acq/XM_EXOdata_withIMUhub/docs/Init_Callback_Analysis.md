# Init 함수 매개변수 및 Tx 함수 차이 분석

**작성일**: 2025-12-08  
**목적**: XM vs IMU Hub Module 초기화 차이 분석

---

## 1. ImuHub_Callbacks_t* callbacks 사용 여부 ❌

### **문제: callbacks 파라미터 미사용**

#### **함수 시그니처 (imu_hub_drv.h:132)**
```c
/**
 * @brief IMU Hub 드라이버 초기화
 * @param tx_func FDCAN Transmit 함수 포인터
 * @param callbacks System Layer 콜백 포인터  ← ⚠️ 사용 안 됨!
 */
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks);
```

#### **실제 구현 (imu_hub_drv.c:260~319)**
```c
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    if (tx_func == NULL) {
        return;
    }
    
    /* ❌ callbacks는 어디에도 사용되지 않음! */
    /* ❌ (void)callbacks; 조차 없음 */
    
    memset(&s_imu_hub_inst, 0, sizeof(ImuHub_DrvInst_t));
    
    /* 1. Tx 함수 저장 */
    s_imu_hub_inst.tx_func = tx_func;
    
    /* ... AGR_NMT, AGR_DOP, AGR_PnP 초기화 ... */
    
    /* ✅ AGR_PnP 콜백만 사용 (callbacks 파라미터 무시) */
    AGR_PnP_Config_t pnp_config = {
        .callbacks = {
            .on_connected = _PnP_OnConnected,       /* ✅ 내부 함수 */
            .on_disconnected = _PnP_OnDisconnected, /* ✅ 내부 함수 */
            .on_bootup = _PnP_OnBootup,             /* ✅ 내부 함수 */
            .on_nmt_change = _PnP_OnNmtChange,      /* ✅ 내부 함수 */
            .on_run_pre_op = _PnP_RunPreOp,         /* ✅ 내부 함수 */
            .on_error = _PnP_OnError                /* ✅ 내부 함수 */
        }
    };
}
```

### **ImuHub_Callbacks_t 정의 (imu_hub_drv.h:113~119)**
```c
/**
 * @brief System Layer 콜백 (PnP 이벤트)
 */
typedef struct {
    void (*OnBootup)(void);                         /**< IMU Hub Boot-up 수신 */
    void (*OnHeartbeat)(uint8_t nmt_state);         /**< Heartbeat 수신 */
    void (*OnNmtStateChange)(uint8_t new_state);    /**< NMT 상태 변경 */
    void (*OnSyncStates)(void);                     /**< Sync States 응답 */
    void (*OnPdoMappingAck)(uint8_t group);         /**< PDO Mapping 설정 완료 */
} ImuHub_Callbacks_t;
```

### **✅ 판단: Legacy 파라미터 (삭제 가능)**

#### **이유**
1. ❌ **전혀 사용되지 않음**: 코드 어디에도 `callbacks` 참조 없음
2. ✅ **AGR_PnP가 대체**: 콜백은 `AGR_PnP_Config_t`로 전달
3. ✅ **Device Layer 내부 처리**: 상위 레이어 콜백 필요 없음

#### **원래 의도 (V1/V2 시절)**
```c
/* V1/V2: System Link가 Device Layer 콜백 등록 */
ImuHub_Callbacks_t callbacks = {
    .OnBootup = ImuHub_XM_Link_OnBootup,
    .OnHeartbeat = ImuHub_XM_Link_OnHeartbeat,
    /* ... */
};
ImuHub_Drv_Init(tx_func, &callbacks);
```

#### **V3.0 변경 (System Link 제거)**
```c
/* V3.0: Device Layer가 AGR_PnP 직접 사용 */
ImuHub_Drv_Init(tx_func, NULL);  /* callbacks 불필요 */
```

### **✅ 해결: callbacks 파라미터 제거**

```diff
/* imu_hub_drv.h */
- void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks);
+ void ImuHub_Drv_Init(AGR_TxFunc_t tx_func);

/* imu_hub_drv.c */
- void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
+ void ImuHub_Drv_Init(AGR_TxFunc_t tx_func)
{
    if (tx_func == NULL) {
        return;
    }
    
    /* ... */
}

/* system_startup.c */
- ImuHub_Drv_Init(IOIF_AGRB_FDCAN_Transmit, NULL);
+ ImuHub_Drv_Init(IOIF_AGRB_FDCAN_Transmit);
```

### **추가 정리: ImuHub_Callbacks_t 구조체도 삭제?**

#### **옵션 1: 완전 삭제** ⭐ (권장)
```diff
/* imu_hub_drv.h */
- /**
-  * @brief System Layer 콜백 (PnP 이벤트)
-  */
- typedef struct {
-     void (*OnBootup)(void);
-     void (*OnHeartbeat)(uint8_t nmt_state);
-     void (*OnNmtStateChange)(uint8_t new_state);
-     void (*OnSyncStates)(void);
-     void (*OnPdoMappingAck)(uint8_t group);
- } ImuHub_Callbacks_t;
```

#### **옵션 2: 향후 확장 고려 유지** ⚠️
```c
/* 향후 User Callback 필요 시 (현재 불필요) */
typedef struct {
    void (*on_connected)(void);    /* OPERATIONAL 진입 */
    void (*on_disconnected)(void); /* 연결 끊김 */
    void (*on_error)(uint8_t error_code);
} ImuHub_UserCallbacks_t;

void ImuHub_Drv_RegisterCallbacks(const ImuHub_UserCallbacks_t* callbacks);
```

**권장**: ✅ **옵션 1 (완전 삭제)** - 현재 불필요, YAGNI 원칙

---

## 2. Tx 함수 전달 방식 차이 ✅

### **XM (Master) - 직접 전달**
```c
/* imu_hub_drv.c:260~319 */
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    /* 1. Device Layer에 저장 */
    s_imu_hub_inst.tx_func = tx_func;  /* ✅ 직접 저장 */
    
    /* 2. AGR_DOP에 직접 전달 */
    AGR_DOP_Init(&s_imu_hub_inst.dop_ctx,
                 NULL,
                 AGR_NODE_ID_XM,
                 tx_func);  /* ✅ 직접 전달 */
    
    /* 3. AGR_PnP에 직접 전달 */
    AGR_PnP_Config_t pnp_config = {
        .role = AGR_PNP_ROLE_MASTER,
        .my_node_id = AGR_NODE_ID_XM,
        .tx_func = tx_func,  /* ✅ 직접 전달 */
        /* ... */
    };
}
```

### **IMU Hub Module (Slave) - Wrapper 사용**
```c
/* xm_drv.c:389~471 */
int XM_Drv_Init(int (*tx_func)(uint32_t, const uint8_t*, uint8_t))
{
    /* 1. Device Layer에 저장 */
    s_tx_func = tx_func;  /* ✅ static 변수에 저장 */
    
    /* 2. AGR_DOP에 Wrapper 전달 */
    AGR_DOP_Init(&s_dop_ctx, 
                 &s_od_table, 
                 AGR_NODE_ID_IMU_HUB, 
                 _Drv_TxFunc);  /* ✅ Wrapper 함수 전달 */
    
    /* 3. AGR_PnP에 Wrapper 전달 */
    AGR_PnP_Config_t pnp_config = {
        .role = AGR_PNP_ROLE_SLAVE,
        .my_node_id = AGR_NODE_ID_IMU_HUB,
        .tx_func = _Drv_TxFunc,  /* ✅ Wrapper 함수 전달 */
        /* ... */
    };
}
```

### **Wrapper 함수 (_Drv_TxFunc)**
```c
/* xm_drv.c:123~136 */
/**
 * @brief Device Layer Tx 함수 (AGR_DOP, AGR_PnP → FDCAN Wrapper)
 * @details AGR_DOP, AGR_PnP는 AGR_TxFunc_t 타입을 사용하므로,
 *          s_tx_func(int 반환)를 AGR_TxFunc_t(void 반환)로 변환합니다.
 */
static void _Drv_TxFunc(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    if (s_tx_func == NULL) {
        return;
    }
    
    /* ✅ 반환값 무시 (AGR_TxFunc_t는 void) */
    (void)s_tx_func(can_id, data, len);
}
```

### **✅ 판단: 차이 이유 (반환 타입 불일치)**

#### **타입 비교**
```c
/* AGR_TxFunc_t (AGR_DOP, AGR_PnP 사용) */
typedef void (*AGR_TxFunc_t)(uint32_t can_id, const uint8_t* data, uint8_t len);
```

**XM (Master)**:
```c
/* IOIF_AGRB_FDCAN_Transmit 타입 */
void IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);
/* ✅ AGR_TxFunc_t와 완벽히 일치! */
```

**IMU Hub Module (Slave)**:
```c
/* XM_Drv_Init 파라미터 타입 */
int (*tx_func)(uint32_t, const uint8_t*, uint8_t);
/* ❌ AGR_TxFunc_t와 반환 타입 불일치 (int vs void) */

/* s_tx_func 타입 */
static int (*s_tx_func)(uint32_t, const uint8_t*, uint8_t);
/* ❌ AGR_TxFunc_t와 반환 타입 불일치 */
```

#### **차이 발생 원인**
```
┌────────────────────────────────────────────────────┐
│ XM (Master) - STM32H7                              │
│                                                    │
│ IOIF_AGRB_FDCAN_Transmit: void 반환               │
│     ↓ (완벽 일치)                                  │
│ AGR_TxFunc_t: void 반환                            │
│     ↓ (직접 전달 가능)                             │
│ AGR_DOP, AGR_PnP                                   │
└────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────┐
│ IMU Hub Module (Slave) - STM32G4                   │
│                                                    │
│ IOIF_AGRB_FDCAN_Transmit: int 반환 (Legacy)       │
│     ↓ (타입 불일치!)                               │
│ AGR_TxFunc_t: void 반환                            │
│     ↓ (Wrapper 필요)                               │
│ _Drv_TxFunc: void 반환 (int 무시)                  │
│     ↓ (변환 완료)                                  │
│ AGR_DOP, AGR_PnP                                   │
└────────────────────────────────────────────────────┘
```

### **✅ 해결: IMU Hub Module도 통일**

#### **옵션 1: IOIF 반환 타입 변경** ⭐ (권장)
```diff
/* IMU Hub Module - ioif_agrb_fdcan.h */
- int IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);
+ void IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);

/* IMU Hub Module - ioif_agrb_fdcan.c */
- int IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len)
+ void IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len)
{
-     return HAL_FDCAN_AddMessageToTxFifoQ(...) == HAL_OK ? 0 : -1;
+     (void)HAL_FDCAN_AddMessageToTxFifoQ(...);  /* 반환값 무시 */
}

/* xm_drv.c */
- int XM_Drv_Init(int (*tx_func)(uint32_t, const uint8_t*, uint8_t))
+ int XM_Drv_Init(AGR_TxFunc_t tx_func)  /* ✅ 통일 */
{
-     s_tx_func = tx_func;
-     AGR_DOP_Init(&s_dop_ctx, &s_od_table, AGR_NODE_ID_IMU_HUB, _Drv_TxFunc);
+     /* Wrapper 불필요 */
+     AGR_DOP_Init(&s_dop_ctx, &s_od_table, AGR_NODE_ID_IMU_HUB, tx_func);
}

/* _Drv_TxFunc 삭제 */
- static void _Drv_TxFunc(uint32_t can_id, const uint8_t* data, uint8_t len) { ... }
```

#### **옵션 2: Wrapper 유지** (현재 상태)
```c
/* 현재 구조 유지 (작동은 하지만 일관성 부족) */
int (*tx_func)(uint32_t, const uint8_t*, uint8_t);  /* Legacy */
_Drv_TxFunc(can_id, data, len);  /* Wrapper */
```

### **✅ 권장: 옵션 1 (통일)**

**이유**:
1. ✅ **일관성**: XM과 IMU Hub Module 동일한 시그니처
2. ✅ **단순화**: Wrapper 제거, 직접 전달
3. ✅ **표준 준수**: `AGR_TxFunc_t` 표준 타입 사용
4. ⚠️ **반환값 무시**: FDCAN Tx 실패 감지 불가 (Software Queue로 대체)

---

## 📊 비교 요약

| 항목 | XM (Master) | IMU Hub Module (Slave) | 이유 |
|------|-------------|------------------------|------|
| **callbacks 파라미터** | 사용 안 함 ❌ | N/A | Legacy (V1/V2 잔재) |
| **tx_func 타입** | `AGR_TxFunc_t` (void) | `int (*)` | IOIF 반환 타입 차이 |
| **Wrapper 사용** | 없음 (직접 전달) | 있음 (`_Drv_TxFunc`) | 타입 불일치 해결 |
| **코드 복잡도** | 단순 ✅ | 복잡 ⚠️ | Wrapper 추가 |

---

## 🚀 즉시 조치 필요

### **1. callbacks 파라미터 제거** ⭐⭐⭐⭐⭐

```diff
/* imu_hub_drv.h */
- void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks);
+ void ImuHub_Drv_Init(AGR_TxFunc_t tx_func);

/* imu_hub_drv.c */
- void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
+ void ImuHub_Drv_Init(AGR_TxFunc_t tx_func)

/* ImuHub_Callbacks_t 구조체 삭제 (선택 사항) */
- typedef struct { ... } ImuHub_Callbacks_t;
```

### **2. IMU Hub Module Tx 함수 통일** ⭐⭐⭐⭐

```diff
/* ioif_agrb_fdcan.h */
- int IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);
+ void IOIF_AGRB_FDCAN_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len);

/* xm_drv.c */
- int XM_Drv_Init(int (*tx_func)(uint32_t, const uint8_t*, uint8_t))
+ int XM_Drv_Init(AGR_TxFunc_t tx_func)
{
-     s_tx_func = tx_func;
-     AGR_DOP_Init(&s_dop_ctx, &s_od_table, AGR_NODE_ID_IMU_HUB, _Drv_TxFunc);
+     AGR_DOP_Init(&s_dop_ctx, &s_od_table, AGR_NODE_ID_IMU_HUB, tx_func);
}

- static void _Drv_TxFunc(...) { ... }  /* 삭제 */
```

---

## ✅ 최종 결론

### **질문 1: callbacks 파라미터 필요?**
❌ **불필요 (Legacy 잔재, 삭제 권장)**

### **질문 2: Tx 함수 전달 방식 차이?**
✅ **반환 타입 불일치 (int vs void) → IMU Hub Module IOIF 수정 권장**

---

**핵심**: 
1. ❌ `callbacks` 파라미터는 V3.0에서 불필요 (AGR_PnP가 대체)
2. ✅ Tx 함수는 `AGR_TxFunc_t` (void 반환)로 통일 권장
3. ✅ Wrapper (`_Drv_TxFunc`) 제거하여 코드 단순화

이렇게 수정하면 **XM과 IMU Hub Module의 일관성**이 높아집니다! 🎯
