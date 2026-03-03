# SDO Response 레이어 분리 명확화

**작성일**: 2025-12-08  
**목적**: CANopen 표준 준수 및 명확한 레이어 분리

---

## 🎯 사용자 지적

> "SDO는 CANopen 표준 프로토콜이잖아. SDO Response는 DOP로 처리한 것을 PnP가 사용해야 하는 구조가 맞지 않을까?"

**완전히 정확합니다!** ✅

---

## ❌ 이전 구조 (명확하지 않음)

```c
/* Before */
if (fnc_code == 0x580) {
    AGR_PnP_ProcessMessage(...);  // ❌ PnP가 SDO를 직접 처리?
}
```

### **문제점**
1. ❌ SDO는 CANopen 표준 프로토콜 → **AGR_DOP**가 처리해야 함
2. ❌ AGR_PnP가 SDO를 직접 디코딩 → 레이어 책임 혼란
3. ❌ 명확하지 않은 레이어 분리

---

## ✅ 수정된 구조 (명확한 레이어 분리)

### **레이어 책임**

| 레이어 | 책임 | 예시 |
|--------|------|------|
| **AGR_DOP** | CANopen 표준 프로토콜 처리 | SDO 디코딩, PDO 디코딩, NMT 처리 |
| **AGR_PnP** | 연결 관리 (DOP 활용) | Boot-up, Heartbeat, 연결 상태 |
| **Device Driver** | 응용 로직 | Pre-Op 상태 머신, OD 관리 |

### **수정된 코드**

```c
/* imu_hub_drv.c */
void ImuHub_Drv_ProcessCANMessage(uint16_t can_id, uint8_t* data, uint8_t len)
{
    uint32_t current_ms = IOIF_TIM_GetTick();
    uint16_t fnc_code = can_id & 0x780;
    
    /* 1. Boot-up / Heartbeat (0x700) */
    if (fnc_code == 0x700) {
        AGR_NMT_ProcessMessage(&s_inst.nmt, can_id, data, len, current_ms);
        return;
    }
    
    /* 2. SDO Response (0x580) */
    if (fnc_code == 0x580) {
        /* ✅ AGR_DOP가 SDO 디코딩 (CANopen 표준 프로토콜 처리) */
        AGR_SDO_Msg_t sdo_msg;
        if (AGR_DOP_DecodeSDO(data, len, &sdo_msg) == 0) {
            /* Device Driver가 Pre-Op 상태 전환 (응용 로직) */
            _OnSdoResponse(&sdo_msg);
        }
        return;
    }
    
    /* 3. PDO (0x180/0x280) */
    if (fnc_code == 0x180 || fnc_code == 0x280) {
        /* ✅ AGR_DOP가 PDO 디코딩 (CANopen 표준) */
        if (IMUHUB_DATA_LOCK()) {
            if (fnc_code == 0x180) {
                _ImuHub_DecodeTpdo1(data, len);
            } else {
                _ImuHub_DecodeTpdo2(data, len);
            }
            s_inst.is_data_ready = true;
            IMUHUB_DATA_UNLOCK();
        }
        return;
    }
}
```

---

## 📊 메시지 흐름 (명확화)

### **SDO Response 처리**

```
1. ISR → canfd_rx_handler.c
   ↓
2. ImuHub_Drv_ProcessCANMessage()
   ↓ (CAN ID 0x580 감지)
3. AGR_DOP_DecodeSDO()
   ↓ (CANopen 표준 프로토콜 처리)
4. _OnSdoResponse()
   ↓ (Device Driver 응용 로직)
5. Pre-Op 상태 전환
```

**핵심**: 
- ✅ **AGR_DOP**: CANopen 표준 프로토콜 처리
- ✅ **Device Driver**: 응용 로직 (Pre-Op)
- ✅ **AGR_PnP**: 연결 관리 (NMT, Boot-up, Heartbeat만)

---

## 🎯 AGR_PnP 역할 명확화

### **AGR_PnP가 처리하는 것** ✅
1. ✅ **Boot-up (0x700)**: 연결 시작 신호
2. ✅ **Heartbeat (0x700)**: 연결 유지 확인
3. ✅ **NMT 상태 변경**: OPERATIONAL 진입 감지
4. ✅ **연결 상태 관리**: is_connected 플래그

### **AGR_PnP가 처리하지 않는 것** ❌
1. ❌ **SDO 디코딩**: AGR_DOP가 처리
2. ❌ **PDO 디코딩**: AGR_DOP가 처리
3. ❌ **Pre-Op 로직**: Device Driver가 처리

---

## 📋 Forward Declaration 정리

```c
/* ===== AGR_PnP 콜백 (연결 관리) ===== */
static void _PnP_OnConnected(uint8_t node_id);
static void _PnP_OnDisconnected(uint8_t node_id);
static void _PnP_OnBootup(uint8_t node_id);
static void _PnP_OnNmtChange(uint8_t node_id, AGR_NMT_State_t old_state, AGR_NMT_State_t new_state);
static void _PnP_RunPreOp(uint8_t node_id, AGR_PnP_Inst_t* inst);
static AGR_PnP_RecoveryAction_t _PnP_OnError(uint8_t node_id, AGR_PnP_Error_t error_code);

/* ===== Device Driver 콜백 (응용 로직) ===== */
static void _OnSdoResponse(const AGR_SDO_Msg_t* response);  /* Pre-Op 상태 전환 */
```

---

## ✅ 최종 정리

| 항목 | Before | After |
|------|--------|-------|
| **SDO 디코딩** | AGR_PnP | ✅ AGR_DOP |
| **Pre-Op 로직** | AGR_PnP 콜백 | ✅ Device Driver |
| **연결 관리** | AGR_PnP | ✅ AGR_PnP (유지) |

---

## 🎯 핵심 원칙

### **레이어 분리**
```
[Application] Device Driver (imu_hub_drv.c)
      ↓ 사용
[Service] AGR_PnP (연결 관리)
      ↓ 사용
[Service] AGR_DOP (CANopen 표준)
      ↓
[Hardware] CAN Bus
```

### **명확한 책임**
- ✅ **AGR_DOP**: "무엇을(What)" - CANopen 표준 프로토콜
- ✅ **AGR_PnP**: "연결(Connection)" - NMT, Boot-up, Heartbeat
- ✅ **Device Driver**: "어떻게(How)" - Pre-Op, 응용 로직

---

## 🚀 향후 개선 (Optional)

### **방안: AGR_DOP 콜백 추가**
```c
/* agr_dop.h */
typedef struct {
    void (*on_sdo_response)(const AGR_SDO_Msg_t* response);
    void (*on_sdo_request)(const AGR_SDO_Msg_t* request, AGR_SDO_Msg_t* response);
} AGR_DOP_Callbacks_t;

/* imu_hub_drv.c */
if (fnc_code == 0x580) {
    AGR_DOP_ProcessSDOResponse(&s_inst.dop_ctx, can_id, data, len);
    // DOP가 on_sdo_response 콜백 호출
}
```

**장점:** 더욱 명확한 레이어 분리  
**현재:** AGR_DOP_DecodeSDO + Device Driver 처리로도 충분히 명확함 ✅

---

## ✅ 최종 결론

**사용자의 지적이 정확했습니다!** 👍

### **개선 사항**
1. ✅ **AGR_DOP가 SDO 디코딩** (CANopen 표준)
2. ✅ **Device Driver가 Pre-Op 처리** (응용 로직)
3. ✅ **AGR_PnP는 연결 관리만** (NMT, Boot-up, Heartbeat)

### **명확성 확보**
- ✅ 레이어 책임 명확
- ✅ CANopen 표준 준수
- ✅ 코드 가독성 향상

---

**결론**: SDO는 CANopen 표준 프로토콜이므로 AGR_DOP가 처리하고, Device Driver가 응용 로직을 담당합니다! ✅
