# 콜백 구조 최종 정리

**작성일**: 2025-12-08  
**목적**: NMT/PnP/DOP 콜백 체인 명확화 및 역할 분리

---

## ✅ **주요 개선 사항**

### **1. AGR_NMT_Mode_t 제거**
- ❌ **삭제**: `AGR_NMT_MODE_SIMPLE` (UART 센서용)
- ✅ **유지**: CANopen 표준만 사용 (무조건 PRE_OPERATIONAL로 전이)

### **2. 콜백 주석 명확화**
- ✅ **Master vs Slave** 역할 명시
- ✅ **레이어별 역할** 명시 (NMT/DOP/PnP/Device)
- ✅ **콜백 호출 경로** 명시

### **3. 코드 가독성 향상**
- ✅ **주석 블록** 추가 (Layer별 구분)
- ✅ **콜백 역할** 인라인 주석

---

## 📊 **최종 콜백 구조**

### **3단계 콜백 체인**

```
[1] AGR_NMT (CANopen NMT 프로토콜)
      ↓ on_state_changed
[2] AGR_PnP 내부 핸들러 (_OnNmtStateChanged)
      ↓ dev->callbacks.on_bootup / on_nmt_change
[3] Device Driver (_PnP_OnBootup, _PnP_OnNmtChange)
      ↓ 응용 로직 (Pre-Op 초기화, State Machine)
```

---

## 🎯 **레이어별 역할**

### **[Layer 1] AGR_NMT**
**파일**: `agr_nmt.c/.h`  
**역할**: CANopen NMT 프로토콜 처리

| 기능 | 설명 |
|------|------|
| `AGR_NMT_ProcessMessage` | Boot-up/Heartbeat 수신 처리 |
| `AGR_NMT_UpdateActivity` | Activity 갱신 + 상태 전이 (BOOT_UP → PRE_OP) |
| `AGR_NMT_CheckTimeout` | Heartbeat Timeout 검사 |
| `AGR_NMT_ProcessCommand` | NMT Command 처리 (START, STOP, RESET) |

**콜백**:
- `on_state_changed`: 상태 변경 시 (`_OnNmtStateChanged`로 설정)
- `on_timeout`: Heartbeat Timeout 시 (`_OnNmtTimeout`로 설정)

---

### **[Layer 2] AGR_DOP**
**파일**: `agr_dop.c/.h`  
**역할**: CANopen SDO/PDO 프로토콜 처리

| 기능 | 설명 |
|------|------|
| `AGR_DOP_ProcessRxMessage` | SDO/PDO 수신 처리 |
| `AGR_DOP_SendSDOWrite` | SDO Write 전송 (Master) |
| `AGR_DOP_SendSDORead` | SDO Read 전송 (Master) |
| `AGR_DOP_SendPDO` | PDO 전송 (Slave) |
| `AGR_DOP_DecodeSDO` | SDO 디코딩 (Master) |

**콜백**:
- `on_sdo_request`: SDO Request 수신 시 (`_OnSdoRequest`로 설정, Slave only)
- `on_pdo_received`: PDO 수신 시 (`_OnPdoReceived`로 설정, Master/Slave)

---

### **[Layer 3] AGR_PnP**
**파일**: `agr_pnp.c/.h`  
**역할**: PnP 추상화 (State Machine, Timeout 관리)

| 기능 | 설명 |
|------|------|
| `AGR_PnP_Init` | PnP 초기화 (Master/Slave) |
| `AGR_PnP_RegisterDevice` | Slave Device 등록 (Master only) |
| `AGR_PnP_RegisterMaster` | Master 등록 (Slave only) |
| `AGR_PnP_RunPeriodic` | Periodic 실행 (Timeout 체크, Heartbeat 전송) |
| `_OnNmtStateChanged` | NMT 콜백 → Device 콜백 변환 (내부 핸들러) |
| `_OnNmtTimeout` | Timeout 콜백 → Device 에러 콜백 (내부 핸들러) |

**콜백** (Device Driver에서 등록):

#### **Master 콜백**
- `on_bootup`: Slave Boot-up 수신 → Pre-Op 초기화
- `on_nmt_change`: Slave NMT 변경 → Pre-Op State Machine
- `on_run_pre_op`: Pre-Op 단계 실행 (Step Array)
- `on_error`: 에러 처리

#### **Slave 콜백**
- `on_connected`: Master 연결 완료
- `on_disconnected`: Master 연결 끊김
- `on_nmt_change`: Master NMT 변경 (선택 사항)
- `on_error`: 에러 처리

---

### **[Layer 4] Device Driver**
**파일**: `imu_hub_drv.c` (Master), `xm_drv.c` (Slave)  
**역할**: 응용 로직 (Pre-Op 시퀀스, 데이터 변환)

#### **Master (imu_hub_drv.c)**

| 콜백 | 역할 |
|------|------|
| `_PnP_OnBootup` | Pre-Op 초기화 (`pre_op_state = SEND_PDO_MAP_A`) |
| `_PnP_OnNmtChange` | Pre-Op State Machine 실행 |
| `_PnP_RunPreOp` | Step Array 실행 (SDO 전송) |
| `_OnSdoResponse` | SDO Response → Pre-Op 상태 전이 |
| `_ImuHub_DecodeTpdo1/2` | PDO 디코딩 + 데이터 저장 |

#### **Slave (xm_drv.c)**

| 콜백 | 역할 |
|------|------|
| `_PnP_OnConnected` | Master 연결 완료 (상태 업데이트) |
| `_PnP_OnDisconnected` | Master 연결 끊김 (상태 업데이트) |
| `_PnP_OnNmtChange` | Master NMT 변경 (선택 사항, 비어있음) |
| `_OnSdoRequest` | SDO Request → OD 접근 |
| `_OnPdoReceived` | PDO 수신 → 데이터 업데이트 |

---

## 🔄 **Master (XM) 콜백 흐름**

### **Boot-up 수신 시**

```
[IMU Hub - Slave]
  └─ Boot-up 전송 (0x708, [0x00])

[XM - Master]
  └─ [1] AGR_NMT_ProcessMessage (agr_nmt.c)
      └─ inst->state = BOOT_UP
      └─ AGR_NMT_UpdateActivity
          └─ inst->state = PRE_OPERATIONAL  /* ✅ CANopen 표준 */
          └─ _NotifyStateChange(BOOT_UP, PRE_OP)
              ↓
  └─ [2] _OnNmtStateChanged (agr_pnp.c - 내부 핸들러)
      └─ dev->callbacks.on_bootup(node_id)
          ↓
  └─ [3] _PnP_OnBootup (imu_hub_drv.c - Device Driver)
      └─ s_imu_hub_inst.pre_op_state = SEND_PDO_MAP_A ✅
```

---

### **Pre-Op 실행 시**

```
[XM - Master]
  └─ ImuHub_Drv_RunPeriodic()
      └─ [3] _PnP_RunPreOp (imu_hub_drv.c - Device Driver)
          └─ Step Array 실행
              └─ [2] AGR_DOP_SendSDOWrite (agr_dop.c)
                  └─ [1] tx_func() (FDCAN 전송)
```

---

### **SDO Response 수신 시**

```
[IMU Hub - Slave]
  └─ SDO Response 전송 (0x588, [0x60, ...])

[XM - Master]
  └─ [1] AGR_DOP_DecodeSDO (agr_dop.c)
      └─ sdo_msg 파싱
          ↓
  └─ [3] _OnSdoResponse (imu_hub_drv.c - Device Driver)
      └─ s_pre_op_state = SEND_PDO_MAP_B ✅
```

---

## 🔄 **Slave (IMU Hub) 콜백 흐름**

### **Master Heartbeat 수신 시**

```
[XM - Master]
  └─ Heartbeat 전송 (0x702, [0x05])

[IMU Hub - Slave]
  └─ [1] AGR_NMT_ProcessMessage (agr_nmt.c)
      └─ inst->state = OPERATIONAL
      └─ AGR_NMT_UpdateActivity
          └─ 상태 유지 (Activity만 갱신)
```

---

### **SDO Request 수신 시**

```
[XM - Master]
  └─ SDO Request 전송 (0x608, [0x2F, ...])

[IMU Hub - Slave]
  └─ [1] AGR_DOP_ProcessRxMessage (agr_dop.c)
      └─ s_dop_ctx.on_sdo_request()
          ↓
  └─ [3] _OnSdoRequest (xm_drv.c - Device Driver)
      └─ AGR_DOP_WriteOD() → OD 업데이트 ✅
      └─ AGR_DOP_SendSDOResponse() → 응답 전송 ✅
```

---

## 📋 **수정된 파일**

| 파일 | 수정 내용 | XM | IMU Hub |
|------|-----------|:--:|:-------:|
| `agr_nmt.h` | Mode enum/필드 제거, 주석 명확화 | ✅ | ✅ |
| `agr_nmt.c` | Mode 로직 제거, 간소화 | ✅ | ✅ |
| `agr_pnp.h` | 콜백 주석 명확화 (Master/Slave 구분) | ✅ | ✅ |
| `agr_pnp.c` | 내부 핸들러 주석 명확화 | ✅ | ✅ |
| `imu_hub_drv.c` | 콜백 역할 주석 추가, 등록 주석 명확화 | ✅ | - |
| `xm_drv.c` | 콜백 역할 주석 추가, 등록 주석 명확화 | - | ✅ |

---

## 🎯 **핵심 정리**

### **콜백 체인 (3단계)**
```
AGR_NMT → AGR_PnP 내부 핸들러 → Device Driver
```

### **레이어별 역할**
```
[1] AGR_NMT: CANopen NMT 프로토콜
[2] AGR_DOP: CANopen SDO/PDO 프로토콜
[3] AGR_PnP: PnP 추상화
[4] Device Driver: 응용 로직
```

### **Master vs Slave 콜백**

| 콜백 | Master | Slave |
|------|:------:|:-----:|
| `on_bootup` | ✅ Pre-Op 초기화 | ❌ |
| `on_nmt_change` | ✅ Pre-Op State Machine | ⚠️ 선택 사항 |
| `on_run_pre_op` | ✅ Step Array 실행 | ❌ |
| `on_connected` | ❌ | ✅ Master 연결 |
| `on_disconnected` | ❌ | ✅ Master 끊김 |
| `on_error` | ✅ | ✅ |
| `on_sdo_request` | ❌ | ✅ OD 접근 |
| `on_pdo_received` | ✅ 데이터 수신 | ✅ 데이터 수신 |

---

**결론**: 콜백 구조가 명확해졌으며, Mode Legacy 제거로 코드가 간소화되었습니다! ✅
