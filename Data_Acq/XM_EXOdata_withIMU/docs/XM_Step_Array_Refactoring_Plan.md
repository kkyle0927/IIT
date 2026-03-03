# XM (Master) Step Array 리팩토링 계획

**작성일**: 2025-12-08  
**목적**: System Link 제거 + Step Array 패턴 적용

---

## 🎯 목표

1. ✅ **System Link 제거**: `imu_hub_xm_link_v2.c/.h` 삭제
2. ✅ **Step Array 패턴**: Pre-Op 시퀀스를 선언적으로 정의
3. ✅ **재사용성**: 다른 Slave Device에도 적용 가능
4. ✅ **확장성**: 새로운 단계 추가 용이
5. ✅ **검증된 로직 유지**: `imu_hub_xm_link_v2.c`의 Pre-Op 로직 보존

---

## 📊 현재 구조 (Before)

### **파일 구조**

```
XM_FW/
├── Devices/AGR/IMU_Module/
│   ├── imu_hub_drv.c         (Device Driver)
│   └── imu_hub_drv.h
└── System/Links/IMU_Module/
    ├── imu_hub_xm_link_v2.c  (System Link - Pre-Op 로직)
    └── imu_hub_xm_link_v2.h
```

### **Pre-Op 시퀀스 (하드코딩)**

`imu_hub_xm_link_v2.c`:

```c
typedef enum {
    IMUHUB_PRE_OP_IDLE = 0,
    IMUHUB_PRE_OP_SEND_PDO_MAP_A,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_A,
    IMUHUB_PRE_OP_SEND_PDO_MAP_B,
    IMUHUB_PRE_OP_WAIT_PDO_MAP_B,
    IMUHUB_PRE_OP_SEND_IMU_MASK_REQ,
    IMUHUB_PRE_OP_WAIT_IMU_MASK_RSP,
    IMUHUB_PRE_OP_SEND_NMT_START,
    IMUHUB_PRE_OP_COMPLETE
} ImuHub_PreOpState_t;

void _RunPreOpStateMachine(void)
{
    switch (s_pre_op_state) {
        case IMUHUB_PRE_OP_SEND_PDO_MAP_A:
            /* SDO 전송 */
            s_pre_op_state = IMUHUB_PRE_OP_WAIT_PDO_MAP_A;
            break;
        case IMUHUB_PRE_OP_WAIT_PDO_MAP_A:
            /* SDO Response 대기 */
            if (sdo_response_received) {
                s_pre_op_state = IMUHUB_PRE_OP_SEND_PDO_MAP_B;
            }
            break;
        /* ... 반복 ... */
    }
}
```

**문제점**:
- ❌ 상태 머신이 Device Driver에 하드코딩
- ❌ 새로운 Slave Device 추가 시 코드 복제 필요
- ❌ 시퀀스 변경 시 switch-case 전체 수정 필요

---

## 🎨 새로운 구조 (After - Step Array 패턴)

### **1. Step Array 정의 (선언적)**

`imu_hub_drv.c`:

```c
/**
 * @brief Pre-Op 단계 타입
 */
typedef enum {
    PREOP_STEP_SEND_SDO,        /**< SDO 전송 */
    PREOP_STEP_WAIT_SDO_RSP,    /**< SDO Response 대기 */
    PREOP_STEP_SEND_NMT_CMD,    /**< NMT Command 전송 */
    PREOP_STEP_DELAY,           /**< 지연 (ms) */
} PreOpStepType_t;

/**
 * @brief Pre-Op 단계 구조체
 */
typedef struct {
    PreOpStepType_t type;       /**< 단계 타입 */
    uint16_t        od_index;   /**< OD Index (SDO용) */
    uint8_t         od_subindex;/**< OD SubIndex (SDO용) */
    const uint8_t*  data;       /**< 전송 데이터 */
    uint8_t         data_len;   /**< 데이터 길이 */
    uint16_t        timeout_ms; /**< Timeout (ms) */
    uint8_t         retry_max;  /**< 최대 재시도 횟수 */
} PreOpStep_t;

/**
 * @brief IMU Hub Pre-Op 시퀀스 (선언적 정의)
 */
static const PreOpStep_t s_imu_hub_preop_steps[] = {
    /* Step 1: PDO Mapping A 설정 */
    {
        .type = PREOP_STEP_SEND_SDO,
        .od_index = 0x2010,
        .od_subindex = 0x00,
        .data = s_pdo_map_a,
        .data_len = sizeof(s_pdo_map_a),
        .timeout_ms = 100,
        .retry_max = 3
    },
    {
        .type = PREOP_STEP_WAIT_SDO_RSP,
        .od_index = 0x2010,
        .timeout_ms = 100,
        .retry_max = 3
    },
    
    /* Step 2: PDO Mapping B 설정 */
    {
        .type = PREOP_STEP_SEND_SDO,
        .od_index = 0x2011,
        .od_subindex = 0x00,
        .data = s_pdo_map_b,
        .data_len = sizeof(s_pdo_map_b),
        .timeout_ms = 100,
        .retry_max = 3
    },
    {
        .type = PREOP_STEP_WAIT_SDO_RSP,
        .od_index = 0x2011,
        .timeout_ms = 100,
        .retry_max = 3
    },
    
    /* Step 3: IMU Connected Mask 조회 (Optional) */
    {
        .type = PREOP_STEP_SEND_SDO,
        .od_index = 0x2000,  /* IMU Connected Mask */
        .od_subindex = 0x00,
        .data = NULL,  /* Read Request */
        .data_len = 0,
        .timeout_ms = 100,
        .retry_max = 3
    },
    {
        .type = PREOP_STEP_WAIT_SDO_RSP,
        .od_index = 0x2000,
        .timeout_ms = 100,
        .retry_max = 3
    },
    
    /* Step 4: NMT Start 전송 */
    {
        .type = PREOP_STEP_SEND_NMT_CMD,
        .data = (uint8_t[]){ 0x01, AGR_NODE_ID_IMU_HUB },  /* Start */
        .data_len = 2,
        .timeout_ms = 50,
        .retry_max = 1
    },
};

#define PREOP_STEP_COUNT  (sizeof(s_imu_hub_preop_steps) / sizeof(PreOpStep_t))
```

---

### **2. 범용 Pre-Op State Machine**

```c
/**
 * @brief Pre-Op 상태 머신 (범용)
 */
typedef struct {
    const PreOpStep_t* steps;       /**< 단계 배열 */
    uint8_t            step_count;  /**< 단계 개수 */
    uint8_t            current_step;/**< 현재 단계 */
    uint32_t           step_start_ms;/**< 단계 시작 시간 */
    uint8_t            retry_count; /**< 재시도 횟수 */
    bool               is_complete; /**< 완료 플래그 */
} PreOpStateMachine_t;

/**
 * @brief Pre-Op 상태 머신 실행 (범용)
 * @details AGR_PnP 콜백에서 호출
 */
static void _RunPreOpStateMachine(PreOpStateMachine_t* sm)
{
    if (sm->is_complete || sm->current_step >= sm->step_count) {
        return;
    }
    
    const PreOpStep_t* step = &sm->steps[sm->current_step];
    uint32_t current_ms = IOIF_TIM_GetTick();
    
    switch (step->type) {
        case PREOP_STEP_SEND_SDO:
            /* SDO 전송 */
            AGR_DOP_SendSDOWrite(&s_inst.dop_ctx, 
                                 step->od_index, 
                                 step->od_subindex, 
                                 step->data, 
                                 step->data_len);
            sm->step_start_ms = current_ms;
            sm->current_step++;  /* 다음 단계: WAIT_SDO_RSP */
            break;
            
        case PREOP_STEP_WAIT_SDO_RSP:
            /* SDO Response 대기 (AGR_PnP 콜백에서 처리) */
            /* Timeout 체크 */
            if (current_ms - sm->step_start_ms > step->timeout_ms) {
                if (sm->retry_count < step->retry_max) {
                    sm->retry_count++;
                    sm->current_step--;  /* 이전 단계 (SEND_SDO) 재시도 */
                } else {
                    /* 재시도 실패 → Error */
                    _OnPreOpError(step->od_index);
                }
            }
            break;
            
        case PREOP_STEP_SEND_NMT_CMD:
            /* NMT Command 전송 */
            s_inst.tx_func(0x000, step->data, step->data_len);
            sm->current_step++;
            break;
            
        case PREOP_STEP_DELAY:
            /* 지연 */
            if (current_ms - sm->step_start_ms > step->timeout_ms) {
                sm->current_step++;
            }
            break;
    }
    
    /* 완료 체크 */
    if (sm->current_step >= sm->step_count) {
        sm->is_complete = true;
    }
}
```

---

### **3. AGR_PnP 콜백 통합**

```c
/**
 * @brief AGR_PnP 콜백: Pre-Op 실행 (Master)
 */
static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst)
{
    (void)node_id;
    (void)inst;
    
    /* Step Array 기반 Pre-Op 실행 */
    _RunPreOpStateMachine(&s_preop_sm);
}

/**
 * @brief AGR_PnP 콜백: SDO Response 수신 (Master)
 */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response)
{
    /* 현재 단계가 WAIT_SDO_RSP인지 확인 */
    if (s_preop_sm.current_step >= s_preop_sm.step_count) {
        return;
    }
    
    const PreOpStep_t* step = &s_preop_sm.steps[s_preop_sm.current_step];
    
    if (step->type == PREOP_STEP_WAIT_SDO_RSP) {
        /* OD Index 일치 확인 */
        if (response->index == step->od_index && 
            response->subindex == step->od_subindex) {
            
            /* 성공 → 다음 단계 */
            s_preop_sm.current_step++;
            s_preop_sm.retry_count = 0;
        }
    }
}
```

---

### **4. 초기화**

```c
void ImuHub_Drv_Init(AGR_TxFunc_t tx_func, const ImuHub_Callbacks_t* callbacks)
{
    ...
    
    /* Pre-Op State Machine 초기화 */
    s_preop_sm.steps = s_imu_hub_preop_steps;
    s_preop_sm.step_count = PREOP_STEP_COUNT;
    s_preop_sm.current_step = 0;
    s_preop_sm.retry_count = 0;
    s_preop_sm.is_complete = false;
    
    ...
}
```

---

## 🎯 장점

### **1. 선언적 (Declarative)**
- ✅ 시퀀스가 데이터 배열로 정의됨
- ✅ 코드 가독성 향상
- ✅ 단계 추가/제거 용이

### **2. 재사용 가능 (Reusable)**
- ✅ 다른 Slave Device에도 적용 가능
- ✅ 범용 State Machine 공유

**예시**:
```c
/* EMG Module Pre-Op (TPDO1만 설정) */
static const PreOpStep_t s_emg_preop_steps[] = {
    { .type = PREOP_STEP_SEND_SDO, .od_index = 0x2010, ... },
    { .type = PREOP_STEP_WAIT_SDO_RSP, .od_index = 0x2010, ... },
    { .type = PREOP_STEP_SEND_NMT_CMD, ... },
};
```

### **3. 확장 용이 (Extensible)**
- ✅ 새로운 단계 타입 추가 쉬움
- ✅ 조건부 실행 추가 가능

**예시**:
```c
typedef enum {
    PREOP_STEP_SEND_SDO,
    PREOP_STEP_WAIT_SDO_RSP,
    PREOP_STEP_SEND_NMT_CMD,
    PREOP_STEP_DELAY,
    PREOP_STEP_CONDITIONAL,     /* ✅ 새 타입 */
    PREOP_STEP_CALLBACK,        /* ✅ 커스텀 콜백 */
} PreOpStepType_t;
```

### **4. 유지보수 용이 (Maintainable)**
- ✅ 시퀀스 변경 시 배열만 수정
- ✅ switch-case 없음
- ✅ 테스트 용이

---

## 📋 마이그레이션 계획

### **Phase 1: 현재 로직 유지 (완료)** ✅
- ✅ `imu_hub_drv.c`에 `ImuHub_PreOpState_t` enum 복사
- ✅ `_PnP_RunPreOp` 콜백 구현
- ✅ AGR_NMT 통합

### **Phase 2: Step Array 패턴 도입 (Optional)**
1. ⏳ `PreOpStep_t` 구조체 정의
2. ⏳ `s_imu_hub_preop_steps[]` 배열 정의
3. ⏳ `_RunPreOpStateMachine()` 범용 함수 구현
4. ⏳ `_PnP_RunPreOp()` 콜백에서 Step Array 실행
5. ⏳ SDO Response 콜백 연결
6. ⏳ 테스트 및 검증

### **Phase 3: System Link 제거 (Optional)**
1. ⏳ `imu_hub_xm_link_v2.c/.h` 삭제
2. ⏳ `canfd_rx_handler.c`에서 `ImuHub_Drv_ProcessCANMessage()` 직접 호출
3. ⏳ include 정리

---

## 🎯 현재 상태 (Phase 1 완료)

### **✅ 구현 완료**
- ✅ `imu_hub_drv.c`: AGR_NMT 통합
- ✅ `imu_hub_drv.c`: CAN ID 기반 분기
- ✅ `imu_hub_drv.c`: Pre-Op State Machine 내장
- ✅ `imu_hub_drv.c`: AGR_PnP 콜백 구현

### **⏳ 다음 단계 (Optional)**
- ⏳ Step Array 패턴 도입 (재사용성, 확장성)
- ⏳ System Link 제거 (단순화)

---

## 📝 권장 사항

### **현재 상태 유지 (권장)** ✅

**이유**:
1. ✅ 검증된 Pre-Op 로직 보존
2. ✅ CANopen 표준 준수 완료
3. ✅ XM-IMU 연결 정상 동작
4. ✅ 추가 개발 비용 없음

**장점**:
- ✅ 안정성 (이미 검증됨)
- ✅ 빠른 배포
- ✅ 낮은 리스크

---

### **Step Array 도입 (선택 사항)**

**시기**:
- 새로운 Slave Device 추가 시 (EMG, Force Sensor 등)
- Pre-Op 시퀀스 변경이 잦을 때
- 재사용성이 중요할 때

**우선순위**: **낮음** (현재는 불필요)

---

## ✅ 최종 결론

**Phase 1 (현재 상태)**: ✅ **완료 및 권장**
- CANopen 표준 준수
- 검증된 Pre-Op 로직 유지
- XM-IMU 연결 정상 동작

**Phase 2 (Step Array)**: ⏳ **선택 사항**
- 재사용성, 확장성 향상
- 새로운 Slave Device 추가 시 고려

---

**결론**: 현재 구조를 유지하고, 필요 시 Step Array 패턴을 도입하는 것을 권장합니다! 🎉
