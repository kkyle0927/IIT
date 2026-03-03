# NMT Node ID 버그 수정

**작성일**: 2025-12-08  
**문제**: Slave가 NMT START를 받아도 OPERATIONAL로 전환 안됨

---

## 🐛 **Bug 6: Slave NMT의 Node ID가 잘못 설정됨**

### **문제 코드**

```c
/* xm_drv.c - XM_Drv_Init() */

/* ❌ 잘못된 초기화 */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000,
               AGR_NODE_ID_XM,       /* ❌ XM의 Node ID (0x01) */
               NULL, NULL, NULL);
```

**문제점:**
- `s_imu_hub_nmt`는 **Slave 자신의 NMT 상태**를 관리하는 인스턴스
- Node ID가 `AGR_NODE_ID_XM` (0x01)으로 잘못 설정됨
- **`AGR_NODE_ID_IMU_HUB` (0x02)여야 함!**

---

### **영향 분석**

#### **NMT Command 처리 실패**

```c
/* agr_nmt.c - AGR_NMT_ProcessMessage() */

if (can_id == 0x000 && len >= 2) {
    uint8_t cmd_byte = data[0];       // 0x01 (START)
    uint8_t target_node_id = data[1]; // 0x02 (IMU Hub)
    
    /* ❌ 불일치: target_node_id(0x02) != inst->node_id(0x01) */
    if (target_node_id == 0 || target_node_id == inst->node_id) {
        /* 처리 안됨! */
    }
}
```

**결과:**
1. ❌ Master가 NMT START 전송 (0x000, [0x01, 0x02])
2. ❌ IMU Hub가 NMT Command를 무시 (Node ID 불일치)
3. ❌ Slave가 OPERATIONAL로 전환 안됨
4. ❌ `s_imu_hub_nmt.state`가 계속 PRE_OPERATIONAL
5. ❌ `XM_Drv_IsConnected()` = false
6. ❌ PDO 전송 안됨

---

### **수정된 코드**

```c
/* xm_drv.c - XM_Drv_Init() */

/* ✅ 올바른 초기화 */
AGR_NMT_InitEx(&s_imu_hub_nmt, 
               5000,
               AGR_NODE_ID_IMU_HUB,  /* ✅ Slave 자신의 Node ID (0x02) */
               NULL, NULL, NULL);
```

---

### **정상 흐름 (수정 후)**

```
[1] XM (Master): NMT START 전송
  ↓
  CAN ID 0x000, data[0] = 0x01 (START), data[1] = 0x02 (IMU Hub)

[2] IMU Hub (Slave): NMT Command 수신
  ↓
  XM_Drv_ProcessCANMessage(0x000, [0x01, 0x02], 2, ...)
  ↓
  AGR_NMT_ProcessMessage(&s_imu_hub_nmt, ...)
  ↓
  target_node_id = 0x02 == inst->node_id (0x02) ✅ 일치!
  ↓
  AGR_NMT_ProcessCommand(AGR_NMT_CMD_START)
  ↓
  s_imu_hub_nmt.state = AGR_NMT_OPERATIONAL ✅

[3] core_process.c: PDO 전송 가능
  ↓
  XM_Drv_IsConnected()
    ↓
    (s_imu_hub_nmt.state == OPERATIONAL) ✅ true
    &&
    (is_master_connected) ✅ true
    ↓
    return true ✅

[4] PDO 전송
  ↓
  XM_Drv_SendTPDO1() / XM_Drv_SendTPDO2() ✅
  ↓
  XM에서 PDO 수신 ✅
```

---

## 📊 **IMU Hub (Slave)의 2개 NMT 인스턴스**

| 인스턴스 | 역할 | Node ID | 메시지 | 상태 전환 |
|---------|------|---------|--------|----------|
| **`s_imu_hub_nmt`** | **Slave 자신** | **0x02** (IMU Hub) | NMT Command (0x000) | PRE_OP → OPERATIONAL |
| **`config.nmt`** | **Master 추적** | **0x01** (XM) | Master Heartbeat (0x701) | 연결 상태 모니터링 |

---

## 🎯 **수정 효과**

| 항목 | Before | After |
|------|:------:|:-----:|
| **Slave NMT Node ID** | 0x01 (XM) ❌ | 0x02 (IMU Hub) ✅ |
| **NMT START 처리** | 무시됨 ❌ | 정상 처리 ✅ |
| **Slave OPERATIONAL 전환** | 안됨 ❌ | 전환됨 ✅ |
| **`XM_Drv_IsConnected()`** | false ❌ | true ✅ |
| **PDO 전송** | 안됨 ❌ | 정상 전송 ✅ |

---

## 🔧 **수정된 파일**

1. **xm_drv.c** (IMU Hub Module - Slave)
   - `XM_Drv_Init()` 수정
   - `s_imu_hub_nmt` Node ID를 `AGR_NODE_ID_IMU_HUB`로 수정

2. **agr_pnp.c** (공통)
   - `AGR_PnP_IsMasterConnected()` 간소화
   - Master 연결 여부만 확인

3. **xm_drv.c** (IMU Hub Module - Slave)
   - `XM_Drv_IsConnected()` 수정
   - Slave OPERATIONAL + Master 연결 체크

---

**결론**: Node ID 버그를 수정하여 NMT START 처리가 정상 동작하고 PDO 전송이 시작됩니다! ✅
