# PDO 전송 디버깅 체크리스트

**작성일**: 2025-12-08  
**문제**: PDO가 전송되지 않음

---

## ✅ **필수 확인 사항**

### **1. 빌드 성공 여부**
```bash
# IMU Hub Module
cd C:\Users\HyundoKim\Documents\GitHub\IMU_Hub_Module\IMU_HUB_Module
# Debug\IMU_Hub_Module.elf 파일 존재 확인

# XM
cd C:\Users\HyundoKim\Documents\GitHub\ARC_ExtensionBoard\Extension_Module
# Debug\Extension_Module.elf 파일 존재 확인
```

### **2. Live Expression 확인**

**IMU Hub (Slave)**:
```c
/* Slave 자신의 NMT */
s_imu_hub_nmt.node_id            // 0x08 (IMU Hub) ✅
s_imu_hub_nmt.state              // 0x05 (OPERATIONAL) ✅

/* Master 추적 */
s_imu_hub_pnp_inst.is_master_connected      // true ✅
s_imu_hub_pnp_inst.config.nmt.node_id       // 0x01 (XM) ✅
s_imu_hub_pnp_inst.config.nmt.state         // 0x05 (OPERATIONAL) ✅

/* PDO Mapping */
s_dop_ctx.tx_pdo_map[0].count    // >0 ✅
s_dop_ctx.tx_pdo_map[1].count    // >0 ✅

/* XM_Drv_IsConnected() 조건 */
(s_imu_hub_nmt.state == AGR_NMT_OPERATIONAL)  // true ✅
AGR_PnP_IsMasterConnected(&s_imu_hub_pnp_inst)  // true ✅
```

**XM (Master)**:
```c
/* Pre-Op */
s_imu_hub_inst.pre_op_state                // 8 (COMPLETE) ✅

/* Slave 추적 */
s_imu_hub_inst.master_pnp->devices[0].node_id   // 0x08 (IMU Hub) ✅
s_imu_hub_inst.master_pnp->devices[0].nmt.state // 0x05 (OPERATIONAL) ✅
```

---

## 🔍 **단계별 확인**

### **Step 1: IMU Hub Boot-up 전송**
- **확인**: CAN 버스 모니터에서 `0x708, [0x00]` 메시지 확인
- **타이밍**: 초기화 직후 1번만

### **Step 2: XM Pre-Op 시작**
- **확인**: `s_imu_hub_inst.pre_op_state` 변화
  - `IDLE` (0) → `SEND_PDO_MAP_A` (1)
- **타이밍**: Boot-up 수신 직후

### **Step 3: PDO Mapping A 전송**
- **확인**: CAN 버스 모니터에서 `0x608, [SDO Write...]` 확인
- **타이밍**: Step 2 직후

### **Step 4: PDO Mapping A ACK**
- **확인**: CAN 버스 모니터에서 `0x588, [SDO ACK...]` 확인
- **확인**: `s_imu_hub_inst.pre_op_state` = `SEND_PDO_MAP_B` (3)

### **Step 5: PDO Mapping B 전송**
- **확인**: CAN 버스 모니터에서 `0x608, [SDO Write...]` 확인

### **Step 6: PDO Mapping B ACK**
- **확인**: CAN 버스 모니터에서 `0x588, [SDO ACK...]` 확인
- **확인**: `s_imu_hub_inst.pre_op_state` = `SEND_IMU_MASK_REQ` (5)

### **Step 7: NMT START 전송**
- **확인**: CAN 버스 모니터에서 `0x000, [0x01, 0x08]` 확인
- **확인**: `s_imu_hub_inst.pre_op_state` = `COMPLETE` (8)
- **타이밍**: PDO Mapping 완료 후

### **Step 8: IMU Hub NMT START 수신**
- **확인**: `s_imu_hub_nmt.state` = `AGR_NMT_OPERATIONAL` (0x05)
- **확인**: `XM_Drv_IsConnected()` = true

### **Step 9: PDO 전송 시작**
- **확인**: CAN 버스 모니터에서 `0x188, [TPDO1 data...]` 확인
- **확인**: CAN 버스 모니터에서 `0x288, [TPDO2 data...]` 확인
- **타이밍**: 1ms마다 번갈아 전송

---

## 🚨 **문제 발생 시 확인**

### **Boot-up이 계속 반복되면?**
- **원인**: `is_master_connected` = false
- **확인**: Master Heartbeat가 Slave로 제대로 라우팅되는지
- **파일**: `xm_drv.c` - `XM_Drv_ProcessCANMessage()`

### **Pre-Op이 완료되지 않으면?**
- **원인**: SDO Timeout 또는 ACK 수신 안됨
- **확인**: `s_imu_hub_inst.pre_op_state`가 어디서 멈췄는지
- **확인**: CAN 버스 모니터에서 SDO 응답 확인

### **NMT START를 받았는데 OPERATIONAL로 안가면?**
- **원인**: `s_imu_hub_nmt.node_id` ≠ 0x08
- **확인**: `s_imu_hub_nmt.node_id` Live Expression
- **파일**: `xm_drv.c` - `XM_Drv_Init()`

### **OPERATIONAL인데 PDO가 안나가면?**
- **원인**: `XM_Drv_IsConnected()` = false
- **확인**: 
  - `s_imu_hub_nmt.state` = OPERATIONAL?
  - `is_master_connected` = true?
- **파일**: `agr_pnp.c`, `xm_drv.c`

### **PDO Mapping이 비어있으면?**
- **원인**: WriteCb 미호출 또는 실패
- **확인**: `s_dop_ctx.tx_pdo_map[0].count`, `s_dop_ctx.tx_pdo_map[1].count`
- **파일**: `xm_drv.c` - `_OnPdoMappingA_Set()`, `_OnPdoMappingB_Set()`

---

## 🔧 **긴급 디버깅 코드**

### **IMU Hub - core_process.c - _FlushOutputs()**

```c
static void _FlushOutputs(void)
{
    /* ✅ 디버깅: IsConnected 상태 확인 */
    bool is_connected = XM_Drv_IsConnected();
    bool nmt_operational = (s_imu_hub_nmt.state == AGR_NMT_OPERATIONAL);
    bool master_connected = AGR_PnP_IsMasterConnected(&s_imu_hub_pnp_inst);
    
    /* ⚠️ 브레이크포인트 설정: 여기서 멈춰서 값 확인 */
    if (!is_connected) {
        /* PDO 전송 안됨! 이유 확인:
         * - nmt_operational = false? → NMT START 안받음
         * - master_connected = false? → Master Heartbeat 안받음
         */
        return;
    }
    
    /* ... 나머지 코드 ... */
}
```

### **XM - imu_hub_drv.c - ImuHub_Drv_RunPeriodic()**

```c
void ImuHub_Drv_RunPeriodic(void)
{
    /* ✅ 디버깅: Pre-Op 상태 확인 */
    uint8_t pre_op_state = s_imu_hub_inst.pre_op_state;
    
    /* ⚠️ 브레이크포인트 설정: SEND_NMT_START에서 멈춤 */
    if (pre_op_state == IMUHUB_PRE_OP_SEND_NMT_START) {
        /* NMT START 전송 직전!
         * Step Array가 제대로 실행되는지 확인
         */
    }
    
    /* ⚠️ 브레이크포인트 설정: COMPLETE에서 멈춤 */
    if (pre_op_state == IMUHUB_PRE_OP_COMPLETE) {
        /* Pre-Op 완료!
         * 이제 Slave가 OPERATIONAL로 가야 함
         */
    }
    
    /* ... 나머지 코드 ... */
}
```

---

## 📝 **CAN 버스 모니터 필터 설정**

```
# Boot-up (IMU Hub)
ID: 0x708, Data: [0x00]

# NMT Command (XM → IMU Hub)
ID: 0x000, Data: [0x01, 0x08]  # START to Node 0x08

# SDO Write (XM → IMU Hub)
ID: 0x608, Data: [SDO Write...]

# SDO Response (IMU Hub → XM)
ID: 0x588, Data: [SDO ACK...]

# TPDO1 (IMU Hub → XM)
ID: 0x188, Data: [IMU 0,1,2 data...]

# TPDO2 (IMU Hub → XM)
ID: 0x288, Data: [IMU 3,4,5 data...]

# Heartbeat (IMU Hub)
ID: 0x708, Data: [0x05]  # OPERATIONAL

# Heartbeat (XM)
ID: 0x702, Data: [0x05]  # OPERATIONAL
```

---

## 🎯 **최종 확인 사항**

- [ ] IMU Hub 빌드 성공 (Debug\IMU_Hub_Module.elf 존재)
- [ ] XM 빌드 성공 (Debug\Extension_Module.elf 존재)
- [ ] IMU Hub: `s_imu_hub_nmt.node_id` = 0x08
- [ ] IMU Hub: `s_imu_hub_nmt.state` = 0x05 (OPERATIONAL)
- [ ] IMU Hub: `is_master_connected` = true
- [ ] IMU Hub: `tx_pdo_map[0].count` > 0, `tx_pdo_map[1].count` > 0
- [ ] XM: `pre_op_state` = 8 (COMPLETE)
- [ ] XM: `devices[0].nmt.state` = 0x05 (OPERATIONAL)
- [ ] CAN 버스에 TPDO1, TPDO2 메시지 확인

---

**모두 ✅ 면 PDO가 정상 전송됩니다!**


