# XM-IMU Hub PnP 버그 수정 완료 요약

**작성일**: 2025-12-08  
**총 버그 개수**: 6개  
**상태**: ✅ 모두 수정 완료

---

## 📋 **발견 및 수정된 버그 목록**

### **Bug 1: Timeout 재시도 로직 오류**

**증상**: Pre-Op 상태가 왔다갔다함

**원인**: Timeout 시 같은 단계를 재시도해야 하는데, 다음 단계로 점프

**수정**: `WAIT_A` → `SEND_A` (같은 단계 재시도)

**파일**: `imu_hub_drv.c` (XM Master)

---

### **Bug 2: Master가 Boot-up을 받을 때마다 Pre-Op 재시작**

**증상**: Pre-Op이 완료되지 않고 계속 반복

**원인**: `_PnP_OnBootup()`이 무조건 Pre-Op을 재시작

**수정**: IDLE 또는 COMPLETE 상태에서만 Pre-Op 시작, 진행 중이면 무시

**파일**: `imu_hub_drv.c` (XM Master)

---

### **Bug 3: Master Heartbeat를 잘못된 NMT 인스턴스로 처리**

**증상**: Slave가 1초마다 Boot-up을 계속 재전송

**원인**: Master Heartbeat를 Slave 자신의 NMT (`s_imu_hub_nmt`)로 처리

**수정**: Master Heartbeat는 `s_imu_hub_pnp_inst.config.nmt`로 처리

**파일**: `xm_drv.c` (IMU Hub Slave)

---

### **Bug 4: OPERATIONAL 상태에서 Heartbeat 수신 시 시간 갱신 안됨**

**증상**: 연결 후 3초 뒤 Timeout 발생, Boot-up 재전송 시작

**원인**: 상태 변경이 없으면 콜백 호출 안됨 → `last_master_hb_time` 갱신 안됨

**수정**: Heartbeat 수신 시 항상 `_NotifyStateChange` 호출

**파일**: `agr_nmt.c` (공통)

---

### **Bug 5: `XM_Drv_IsConnected()`가 잘못된 NMT 상태 확인**

**증상**: OPERATIONAL인데 PDO가 전송되지 않음

**원인**: Master의 NMT 상태(`config.nmt.state`)를 확인, Slave 자신의 상태 확인 안함

**수정**: Slave 자신의 `s_imu_hub_nmt.state == OPERATIONAL` 확인 추가

**파일**: `agr_pnp.c`, `xm_drv.c` (IMU Hub Slave)

---

### **Bug 6: Slave NMT의 Node ID가 잘못 설정됨**

**증상**: NMT START를 받아도 OPERATIONAL로 전환 안됨

**원인**: `s_imu_hub_nmt.node_id = AGR_NODE_ID_XM` (0x01) - 잘못됨!

**수정**: `s_imu_hub_nmt.node_id = AGR_NODE_ID_IMU_HUB` (0x02)

**파일**: `xm_drv.c` (IMU Hub Slave)

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Pre-Op 완료** | 무한 루프 ❌ | 정상 완료 ✅ |
| **Boot-up 재전송** | 1초마다 ❌ | 초기화 시 1번만 ✅ |
| **Heartbeat 시간 갱신** | 최초 1번만 ❌ | 매 1초마다 ✅ |
| **NMT START 처리** | 무시됨 ❌ | 정상 처리 ✅ |
| **Slave OPERATIONAL 전환** | 안됨 ❌ | 전환됨 ✅ |
| **PDO 전송** | 안됨 ❌ | 1ms마다 전송 ✅ |
| **XM PDO 수신** | 없음 ❌ | 정상 수신 ✅ |

---

## 🔄 **정상 PnP 전체 시퀀스**

```
[0초] IMU Hub 초기화
  ↓
  s_imu_hub_nmt.state = BOOT_UP (자신)
  config.nmt.state = BOOT_UP (Master 추적)
  ↓
  Boot-up 전송 (0x700 + 0x02, [0x00])

[0.1초] XM Boot-up 수신
  ↓
  Pre-Op 시작 (SEND_PDO_MAP_A)

[0.2초] XM → IMU Hub: SDO Write (PDO Mapping A)
  ↓
  IMU Hub: _OnPdoMappingA_Set() → TPDO1 Mapping 설정 ✅
  ↓
  IMU Hub → XM: SDO Response (ACK)

[0.3초] XM 수신 → SEND_PDO_MAP_B
  ↓
  XM → IMU Hub: SDO Write (PDO Mapping B)
  ↓
  IMU Hub: _OnPdoMappingB_Set() → TPDO2 Mapping 설정 ✅
  ↓
  IMU Hub → XM: SDO Response (ACK)

[0.4초] XM 수신 → SEND_IMU_MASK_REQ
  ↓
  XM → IMU Hub: SDO Read (0x2000)
  ↓
  IMU Hub → XM: SDO Response (imu_connected_mask)

[0.5초] XM 수신 → SEND_NMT_START
  ↓
  XM → IMU Hub: NMT START (0x000, [0x01, 0x02])

[0.6초] IMU Hub NMT START 수신
  ↓
  s_imu_hub_nmt.state = OPERATIONAL ✅
  ↓
  XM: pre_op_state = COMPLETE ✅

[1초] IMU Hub Heartbeat 전송 (0x702, [0x05])
  ↓
  XM: Slave OPERATIONAL 확인 ✅

[1초] XM Heartbeat 전송 (0x701, [0x05])
  ↓
  IMU Hub: Master OPERATIONAL 확인 ✅
  ↓
  is_master_connected = true ✅

[1ms마다] IMU Hub: PDO 전송
  ↓
  _FlushOutputs()
    ↓
    XM_Drv_IsConnected()
      ↓
      (s_imu_hub_nmt.state == OPERATIONAL) ✅ true
      &&
      (is_master_connected) ✅ true
      ↓
      return true ✅
    ↓
    XM_Drv_SendTPDO1() / XM_Drv_SendTPDO2() ✅
    ↓
    TPDO1, TPDO2 번갈아 전송 (1ms마다) ✅

[XM] PDO 수신 및 디코딩 ✅
  ↓
  IMU 데이터 업데이트 ✅
```

---

## 📊 **NMT 인스턴스 역할 정리**

### **IMU Hub (Slave)**

| 인스턴스 | 역할 | Node ID | 메시지 | 상태 전환 |
|---------|------|---------|--------|----------|
| **`s_imu_hub_nmt`** | **Slave 자신** | **0x02** (IMU Hub) | NMT Command (0x000) | PRE_OP → OPERATIONAL |
| **`config.nmt`** | **Master 추적** | **0x01** (XM) | Master Heartbeat (0x701) | 연결 상태 모니터링 |

### **XM (Master)**

| 인스턴스 | 역할 | Node ID | 메시지 | 상태 전환 |
|---------|------|---------|--------|----------|
| **`master_pnp->devices[0].nmt`** | **Slave 추적** | **0x02** (IMU Hub) | Slave Heartbeat (0x702) | 연결 상태 모니터링 |

---

## 🔧 **수정된 파일 목록**

| 파일 | 수정 내용 | 버그 번호 |
|------|---------|----------|
| **imu_hub_drv.c** (XM) | Timeout 재시도 로직 수정 | #1 |
| **imu_hub_drv.c** (XM) | Boot-up 중복 처리 방지 | #2 |
| **xm_drv.c** (IMU Hub) | Master Heartbeat 라우팅 수정 | #3 |
| **agr_nmt.c** (공통) | Heartbeat 시 항상 콜백 호출 | #4 |
| **agr_pnp.c** (공통) | `IsMasterConnected` 간소화 | #5 |
| **xm_drv.c** (IMU Hub) | `IsConnected` 조건 수정 | #5 |
| **xm_drv.c** (IMU Hub) | Slave NMT Node ID 수정 | #6 |
| **agr_pnp.c** (공통) | `_OnSlaveNmtStateChanged` 추가 | #3, #4 |
| **agr_pnp.c** (공통) | `RegisterMaster` NMT 모니터링 추가 | #3 |

---

## ✅ **검증 체크리스트**

### **Live Expression으로 확인**

**IMU Hub (Slave)**:
```c
/* Slave 자신의 NMT */
s_imu_hub_nmt.state              // 0x05 (OPERATIONAL) ✅
s_imu_hub_nmt.node_id            // 0x02 (IMU Hub) ✅
s_imu_hub_nmt.last_activity_ms   // 매 1ms 갱신 ✅

/* Master 추적 */
s_imu_hub_pnp_inst.config.nmt.state         // 0x05 (OPERATIONAL) ✅
s_imu_hub_pnp_inst.config.nmt.node_id       // 0x01 (XM) ✅
s_imu_hub_pnp_inst.is_master_connected      // true ✅
s_imu_hub_pnp_inst.last_master_hb_time      // 매 1초 갱신 ✅

/* PDO */
s_dop_ctx.tx_pdo_map[0].count    // >0 (TPDO1 Mapping 설정됨) ✅
s_dop_ctx.tx_pdo_map[1].count    // >0 (TPDO2 Mapping 설정됨) ✅
```

**XM (Master)**:
```c
/* Slave 추적 */
s_imu_hub_inst.master_pnp->devices[0].nmt.state     // 0x05 (OPERATIONAL) ✅
s_imu_hub_inst.master_pnp->devices[0].nmt.node_id   // 0x02 (IMU Hub) ✅

/* Pre-Op */
s_imu_hub_inst.pre_op_state      // 8 (COMPLETE) ✅
```

---

## 🎉 **결론**

**6개의 버그를 모두 수정하여 XM-IMU Hub 간 PnP와 PDO 전송이 완벽하게 동작합니다!**

- ✅ Pre-Op 정상 완료
- ✅ Boot-up은 초기화 시 1번만
- ✅ Heartbeat 정상 교환 (1초마다)
- ✅ NMT START 정상 처리
- ✅ 양측 OPERATIONAL 진입
- ✅ PDO 전송 시작 (1ms마다 TPDO1/TPDO2 번갈아)
- ✅ XM에서 PDO 수신 및 디코딩
- ✅ 연결 안정성 확보
