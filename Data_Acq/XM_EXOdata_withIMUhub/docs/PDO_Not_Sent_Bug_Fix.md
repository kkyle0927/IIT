# PDO 전송 안되는 버그 수정

**작성일**: 2025-12-08  
**문제**: OPERATIONAL 상태인데 PDO가 전송되지 않음

---

## 🐛 **Bug 5: `XM_Drv_IsConnected()`가 잘못된 NMT 상태 확인**

### **문제 분석**

**IMU Hub (Slave)는 2개의 NMT 상태를 추적합니다:**

1. **`s_imu_hub_nmt`**: **Slave 자신의 NMT 상태**
   - Master의 NMT Command를 받아 전환
   - PRE_OPERATIONAL → OPERATIONAL (NMT START 수신)
   - **PDO를 보내려면 이 상태가 OPERATIONAL이어야 함!**

2. **`s_imu_hub_pnp_inst.config.nmt`**: **Master (XM)의 NMT 상태**
   - Master Heartbeat를 받아 추적
   - Master 연결 상태 모니터링 용도

---

### **잘못된 코드 (Before)**

```c
/* agr_pnp.c - AGR_PnP_IsMasterConnected() */
bool AGR_PnP_IsMasterConnected(const AGR_PnP_Inst_t* inst)
{
    /* ❌ Master의 NMT 상태를 확인 (잘못됨!) */
    return inst->is_master_connected &&
           (inst->config.nmt.state == AGR_NMT_OPERATIONAL);
}
```

```c
/* xm_drv.c - XM_Drv_IsConnected() */
bool XM_Drv_IsConnected(void)
{
    /* ❌ Master 상태만 확인 (Slave 자신의 상태는 확인 안함!) */
    return AGR_PnP_IsMasterConnected(&s_imu_hub_pnp_inst);
}
```

**문제점:**
1. Master가 OPERATIONAL이어도, **Slave가 PRE_OPERATIONAL이면 PDO 보내면 안됨!**
2. Slave가 OPERATIONAL인지 확인하지 않음
3. **`core_process.c`의 `if (!XM_Drv_IsConnected())` 체크 실패**
4. **PDO 전송 안됨!** ❌

---

### **수정된 코드 (After)**

```c
/* agr_pnp.c - AGR_PnP_IsMasterConnected() */
bool AGR_PnP_IsMasterConnected(const AGR_PnP_Inst_t* inst)
{
    /* ✅ Master Heartbeat 수신 여부만 확인 */
    /* Slave 자신의 NMT 상태는 Device Driver에서 확인 */
    return inst->is_master_connected;
}
```

```c
/* xm_drv.c - XM_Drv_IsConnected() */
bool XM_Drv_IsConnected(void)
{
    /* ✅ 두 조건 모두 충족해야 연결됨 */
    /* 1. Slave 자신이 OPERATIONAL (NMT START 받음) */
    /* 2. Master Heartbeat 수신 중 (is_master_connected) */
    return (s_imu_hub_nmt.state == AGR_NMT_OPERATIONAL) &&
           AGR_PnP_IsMasterConnected(&s_imu_hub_pnp_inst);
}
```

---

## 🔄 **PDO 전송 흐름 (수정 후)**

```
[1] IMU Hub NMT START 수신
  ↓
  s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL ✅

[2] XM Heartbeat 수신
  ↓
  s_imu_hub_pnp_inst.is_master_connected = true ✅

[3] core_process.c - _FlushOutputs()
  ↓
  if (!XM_Drv_IsConnected())
    ↓
    (s_imu_hub_nmt.state == OPERATIONAL) ✅ true
    &&
    (is_master_connected) ✅ true
    ↓
    ✅ 통과! (PDO 전송 진행)

[4] IMU 데이터 업데이트
  ↓
  XM_Drv_UpdateImuData(0~5, ...)

[5] PDO 전송
  ↓
  if (s_pdo_toggle) {
      XM_Drv_SendTPDO2() → TPDO2 전송 ✅
  } else {
      XM_Drv_SendTPDO1() → TPDO1 전송 ✅
  }
```

---

## 📊 **NMT 상태 역할 분리**

| NMT 인스턴스 | 역할 | 확인 대상 | 용도 |
|-------------|------|----------|------|
| **`s_imu_hub_nmt`** | **Slave 자신** | Slave가 OPERATIONAL인지 | **PDO 전송 가능 여부** ✅ |
| **`config.nmt`** | **Master 추적** | Master가 살아있는지 | Master 연결 상태 |

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **`XM_Drv_IsConnected()` 조건** | Master OPERATIONAL만 ❌ | Slave OPERATIONAL + Master 연결 ✅ |
| **PDO 전송 조건** | 불충분 ❌ | 완전 ✅ |
| **PDO 전송** | 안됨 ❌ | 정상 전송 ✅ |
| **XM PDO 수신** | 없음 ❌ | 1ms마다 수신 ✅ |

---

## 🔧 **수정된 파일**

1. **agr_pnp.c** (IMU Hub & XM 공통)
   - `AGR_PnP_IsMasterConnected()` 간소화
   - Master 연결 여부만 확인

2. **xm_drv.c** (IMU Hub Module - Slave)
   - `XM_Drv_IsConnected()` 수정
   - Slave 자신의 OPERATIONAL 상태 + Master 연결 체크

---

**결론**: Slave 자신의 NMT 상태를 올바르게 확인하여 PDO 전송이 정상적으로 이루어집니다! ✅
