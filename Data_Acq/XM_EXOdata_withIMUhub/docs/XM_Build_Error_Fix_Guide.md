# XM (Master) 빌드 에러 해결 가이드

**작성일**: 2025-12-08  
**상황**: XM의 `imu_hub_drv.c`와 관련 파일들이 Legacy 코드 사용 중

---

## 🚨 문제 상황

XM (Master) 프로젝트는 아직 **구버전 코드**를 사용하고 있어 다수의 빌드 에러 발생:

### **주요 에러**
1. ❌ `ImuHub_DrvInst_t`에 `callbacks` 멤버 없음
2. ❌ `ImuHub_DrvInst_t`에 `tx_data` 멤버 없음 
3. ❌ `IMUHUB_OD_IDX_NMT_STATE` 등 OD Index 정의 없음
4. ❌ `agr_pnp.c` 구문 에러
5. ❌ `imu_hub_xm_link_v2.c` Legacy 코드 문제

---

## ✅ 해결 방법

### **방법 1: Legacy 구조 유지 (빠름)** ⏱️

XM-IMU 연결이 이미 동작하고 있다면, **현재 구조 유지**를 권장합니다:

1. ✅ `imu_hub_xm_link_v2.c/.h` 유지
2. ✅ Legacy `imu_hub_drv.c` 유지
3. ⚠️ 빌드 에러만 최소한으로 수정

**장점**:
- ✅ 빠른 수정
- ✅ 검증된 로직 유지
- ✅ 낮은 리스크

**단점**:
- ⚠️ CANopen 표준 미준수 (System Link 존재)
- ⚠️ IMU Hub Module과 코드 불일치

---

### **방법 2: 전체 리팩토링 (권장)** 🎯

IMU Hub Module에서 이미 완료한 리팩토링을 XM에도 적용:

#### **Phase 1: agr_pnp.c 수정** (이미 완료 ✅)
- ✅ IMU Hub Module의 `agr_pnp.c` 수정 사항 동일 적용
- ✅ `old_state` 변수 scope 수정
- ✅ `else` 절 제거

#### **Phase 2: imu_hub_drv.c 리팩토링**

**파일 교체**:
```
IMU_Hub_Module/IMU_Hub_FW/Devices/AGR/Extension_Module/xm_drv.c (Slave)
                                    ↓
XM/XM_FW/Devices/AGR/IMU_Module/imu_hub_drv.c (Master)
```

**핵심 변경사항**:
1. ✅ `ImuHub_DrvInst_t` 구조체 수정
   - `callbacks` 제거 → AGR_PnP 콜백 사용
   - `tx_data` 제거 → Master는 rx_data만 사용
   - `AGR_NMT_Inst_t nmt` 추가
   - `AGR_PnP_Inst_t pnp_inst` 추가

2. ✅ `ImuHub_Drv_ProcessCANMessage()` 리팩토링
   ```c
   /* Before (Legacy) */
   if (can_id == ...) {
       if (s_inst.callbacks.OnBootup) {
           s_inst.callbacks.OnBootup();
       }
   }
   
   /* After (CANopen 표준) */
   uint16_t fnc_code = can_id & 0x780;
   if (fnc_code == 0x700) {
       AGR_NMT_ProcessMessage(&s_inst.nmt, can_id, data, len, current_ms);
       return;
   }
   ```

3. ✅ `ImuHub_Drv_Init()` 리팩토링
   ```c
   /* Before (Legacy) */
   s_inst.callbacks = *callbacks;
   
   /* After (CANopen 표준) */
   AGR_NMT_InitEx(&s_inst.nmt, 3000, AGR_NODE_ID_IMU_HUB, ...);
   AGR_DOP_Init(&s_inst.dop_ctx, NULL, AGR_NODE_ID_XM, tx_func);
   AGR_PnP_Init(&s_inst.pnp_inst, &pnp_config, IOIF_TIM_GetTick);
   ```

4. ✅ Public API 추가
   ```c
   bool ImuHub_Drv_IsConnected(void);
   AGR_NMT_State_t ImuHub_Drv_GetNmtState(void);
   void ImuHub_Drv_RunPeriodic(void);
   ```

#### **Phase 3: System Link 제거**
1. ✅ `imu_hub_xm_link_v2.c/.h` 삭제
2. ✅ `canfd_rx_handler.c`에서 `ImuHub_Drv_ProcessCANMessage()` 직접 호출
3. ✅ CMakeLists.txt 정리

---

## 📋 수정 우선순위

### **즉시 수정 (빌드 통과)** 🚨

1. ✅ `agr_pnp.c` 구문 에러 (IMU Hub와 동일)
   - `old_state` 변수 scope 수정
   - `else` 절 제거

2. ⚠️ `imu_hub_xm_link_v2.c` Legacy 코드
   - `AGR_PnP_Device_t`에서 `mode` 제거
   - `AGR_PNP_MODE_PROTOCOL` 제거

### **선택 사항 (리팩토링)** ⏳

3. ⏳ `imu_hub_drv.c` 전체 리팩토링
   - CANopen 표준 준수
   - AGR_NMT, AGR_DOP, AGR_PnP 통합

---

## 🎯 권장 사항

### **현재 상황에서 최선의 선택**

1. ✅ **agr_pnp.c 수정** (이미 적용했을 것)
2. ⚠️ **imu_hub_xm_link_v2.c 최소 수정**
   - `dev->mode` 사용 제거
   - `AGR_PNP_MODE_PROTOCOL` 제거
3. ⏳ **imu_hub_drv.c 리팩토링은 나중에**
   - 현재 XM-IMU 연결이 동작 중이면 유지
   - Step Array 패턴 도입 시 같이 진행

---

## 🚀 다음 단계

1. ✅ **IMU Hub Module 테스트**
   - 빌드 성공 확인
   - 실제 하드웨어 테스트

2. ⏳ **XM 리팩토링 결정**
   - IMU Hub 동작 확인 후
   - 필요 시 XM도 동일하게 리팩토링

---

## 📝 결론

**권장**: IMU Hub Module 테스트 먼저 진행 후, XM 리팩토링 여부 결정 🎯

- ✅ IMU Hub는 이미 리팩토링 완료
- ⏳ XM은 현재 Legacy 구조 유지 가능
- 🎯 동작 확인 후 단계적 적용

---

**현재로서는 IMU Hub Module만 빌드 성공하면 충분합니다!** ✅
