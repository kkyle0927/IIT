# PDO 데이터가 0인 근본 원인 분석

**작성일**: 2025-12-08  
**증상**: XM에서 PDO를 정상 수신하지만 모든 데이터가 0

---

## 🔍 **데이터 흐름 역추적**

### **Step 1: XM에서 수신한 PDO 데이터 확인**

```c
/* imu_hub_drv.c - _ImuHub_DecodeTpdo1() */

/* XM이 수신한 TPDO1 (64 bytes) */
Offset | Data              | Source
-------|-------------------|------------------
0~3    | Metadata (4B)     | s_pdo_metadata
4~23   | IMU 0 (20B)       | s_imu_hub_tx_data.imu[0]
24~43  | IMU 1 (20B)       | s_imu_hub_tx_data.imu[1]
44~63  | IMU 2 (20B)       | s_imu_hub_tx_data.imu[2]

/* 모두 0이면? */
→ s_imu_hub_tx_data.imu[0~2] = 0  ← IMU Hub에서 0 전송!
```

---

### **Step 2: IMU Hub PDO 인코딩 확인**

```c
/* xm_drv.c - XM_Drv_SendTPDO1() */
int len = AGR_DOP_EncodeTxPDON(&s_dop_ctx, 1, pdo_buf, 64);
  ↓
/* agr_dop.c - AGR_DOP_EncodeTxPDON() */
const AGR_PDO_MapTable_t* map = &ctx->tx_pdo_map[0];  // TPDO1 Mapping

for (uint8_t i = 0; i < map->count; i++) {  // count = 4
    const AGR_PDO_MapItem_t* item = &map->items[i];
    
    /* OD Entry 찾기 */
    const AGR_OD_Entry_t* entry = AGR_DOP_FindODEntryEx(ctx, 
                                                        item->od_index, 
                                                        item->od_subindex);
    
    /* ✅ entry->data_ptr에서 데이터 읽기 */
    memcpy(&out_buf[offset], entry->data_ptr, entry->size);
    //                        ^^^^^^^^^^^^^^^^ ⚠️ 여기가 0이면?
    offset += entry->size;
}
```

**확인:**
```c
/* item[0]: 0x3000.0x00 - Metadata */
entry->data_ptr = &s_pdo_metadata
  ↓
s_pdo_metadata.timestamp_low = ?  // 0이면 SetFrameTimestamp 안됨!
s_pdo_metadata.valid_mask = ?     // 0이면 ClearValidMask만 됨!

/* item[1]: 0x6000.0x60 - IMU 0 */
entry->data_ptr = &s_imu_hub_tx_data.imu[0].q
  ↓
s_imu_hub_tx_data.imu[0].q[0] = ?  // 0이면 UpdateImuData 안됨!
s_imu_hub_tx_data.imu[0].a[0] = ?
s_imu_hub_tx_data.imu[0].g[0] = ?
```

---

### **Step 3: IMU Hub 데이터 업데이트 확인**

```c
/* core_process.c - _FlushOutputs() */

/* 1. Timestamp 설정 */
XM_Drv_SetFrameTimestamp(g_sys.tick_cnt);
  ↓
s_pdo_metadata.timestamp_low  = (tick_cnt >> 0) & 0xFF;
s_pdo_metadata.timestamp_mid  = (tick_cnt >> 8) & 0xFF;
s_pdo_metadata.timestamp_high = (tick_cnt >> 16) & 0xFF;
/* ✅ tick_cnt > 0이므로 Timestamp는 0이 아님! */

/* 2. Valid Mask 초기화 */
XM_Drv_ClearValidMask();
  ↓
s_pdo_metadata.valid_mask = 0;
/* ⚠️ 아직 0! (IMU 데이터 업데이트 후 설정될 예정) */

/* 3. IMU 데이터 업데이트 */
for (int i = 0; i < 6; i++) {
    XM_Drv_ImuData_t drv_data;
    
    if (g_sys.imus[i].is_connected) {  // ⚠️⚠️⚠️ 핵심!
        /* ✅ 연결됨: UART 데이터 변환 */
        drv_data.q[0] = XM_Drv_QuatToInt16(ebimu->q_w);
        drv_data.q[1] = XM_Drv_QuatToInt16(ebimu->q_x);
        drv_data.q[2] = XM_Drv_QuatToInt16(ebimu->q_y);
        drv_data.q[3] = XM_Drv_QuatToInt16(ebimu->q_z);
        drv_data.a[0] = XM_Drv_AccelToInt16(ebimu->acc_x);
        drv_data.a[1] = XM_Drv_AccelToInt16(ebimu->acc_y);
        drv_data.a[2] = XM_Drv_AccelToInt16(ebimu->acc_z);
        drv_data.g[0] = XM_Drv_GyroToInt16(ebimu->gyr_x);
        drv_data.g[1] = XM_Drv_GyroToInt16(ebimu->gyr_y);
        drv_data.g[2] = XM_Drv_GyroToInt16(ebimu->gyr_z);
    } else {
        /* ❌ 연결 안됨: 0으로 채움 */
        memset(&drv_data, 0, sizeof(XM_Drv_ImuData_t));
    }
    
    /* ✅ XM Driver에 저장 */
    XM_Drv_UpdateImuData(i, &drv_data);
      ↓
    memcpy(&s_imu_hub_tx_data.imu[i], &drv_data, sizeof(...));
}
```

---

## 🚨 **근본 원인 (99.9% 확신)**

```c
g_sys.imus[i].is_connected = false  ← 이것!
```

**이유:**
1. UART 센서가 물리적으로 연결 안됨
2. UART 초기화 실패
3. UART 통신 실패 (DMA, IDLE Interrupt 등)
4. Timeout 발생 (`last_update_time`이 오래됨)

---

## 🔧 **즉시 확인 방법**

### **Live Expression 1개만 확인!**

```c
g_sys.imus[0].is_connected
```

- **`true`**: UART는 정상, 다른 문제 (데이터 변환, PDO Mapping 등)
- **`false`**: ✅ **이것이 원인! UART 센서 문제!**

---

## 📊 **디버깅 우선순위**

### **Priority 1: UART 연결 상태**
```c
g_sys.imus[0].is_connected       // false면 STOP! 여기가 문제!
g_sys.imus[0].last_update_time   // 0이면 한번도 안받음
```

### **Priority 2: UART 데이터**
```c
g_sys.imus[0].data.q_w   // 0이면 UART 파싱 실패
g_sys.imus[0].data.acc_x
```

### **Priority 3: 변환된 데이터**
```c
s_imu_hub_tx_data.imu[0].q[0]  // 0이면 변환 실패 또는 is_connected = false
```

### **Priority 4: PDO Payload**
```c
/* XM_Drv_SendTPDO1() 내부에 브레이크포인트 설정 */
pdo_buf[4]~pdo_buf[23]  // IMU 0 데이터 (20B)
```

---

## 🎯 **해결 방법**

### **Case 1: UART 센서가 연결 안됨 (물리적)**
- **증상**: `g_sys.imus[i].is_connected` = false
- **해결**: UART 센서를 실제로 연결

### **Case 2: UART 초기화 안됨**
- **증상**: `g_uart_debug.uart[i].dma_cndtr` = 0 또는 고정값
- **해결**: `system_startup.c`에서 UART 초기화 확인

### **Case 3: UART Timeout**
- **증상**: `last_update_time`이 오래됨 (current_time - last > timeout)
- **해결**: Timeout 값 증가 또는 센서 Baud Rate 확인

### **Case 4: UART 데이터 파싱 실패**
- **증상**: `g_sys.imus[i].data.q_w` = 0
- **해결**: UART 프로토콜 확인 (EBIMU-9DOFV6 프로토콜)

---

## 🔥 **긴급 테스트 코드**

### **UART 센서 없이 테스트하려면?**

```c
/* core_process.c - _FlushOutputs() */

/* ⚠️ 임시 테스트: 강제로 데이터 주입 */
#ifdef DEBUG_FORCE_IMU_DATA
    /* 테스트 데이터 강제 주입 */
    drv_data.q[0] = 10000;  // 0.61 (10000 / 16384)
    drv_data.q[1] = 5000;
    drv_data.q[2] = 3000;
    drv_data.q[3] = 1000;
    drv_data.a[0] = 100;
    drv_data.a[1] = 200;
    drv_data.a[2] = 300;
    drv_data.g[0] = 50;
    drv_data.g[1] = 60;
    drv_data.g[2] = 70;
#else
    /* 정상 흐름 */
    if (g_sys.imus[i].is_connected) {
        drv_data.q[0] = XM_Drv_QuatToInt16(ebimu->q_w);
        ...
    } else {
        memset(&drv_data, 0, sizeof(...));
    }
#endif

XM_Drv_UpdateImuData(i, &drv_data);
```

**테스트:**
1. `DEBUG_FORCE_IMU_DATA` 정의
2. 빌드 및 실행
3. XM에서 PDO 데이터 확인
   - 0이 아닌 값이 오면 → **UART 센서 문제 확정!**
   - 여전히 0이면 → PDO Encoding/Decoding 문제

---

## 📝 **최종 결론**

**PDO 데이터가 0인 이유는 99.9% 다음 중 하나입니다:**

1. ✅ **`g_sys.imus[i].is_connected` = false** (가장 가능성 높음)
2. UART 데이터가 0 (`g_sys.imus[i].data.q_w` = 0)
3. 데이터 변환 오류 (거의 불가능)
4. PDO Mapping 오류 (이미 확인됨, 문제 없음)

---

**즉시 확인:**

```c
g_sys.imus[0].is_connected = ?
```

이 값을 알려주시면 정확한 해결책을 제시하겠습니다! 🙏
