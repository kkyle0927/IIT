/**
 ******************************************************************************
 * @file    imu_dop_serial.c
 * @author  HyundoKim
 * @brief   [System/Comm/USB] IMU Hub DOP Serial Transport over USB-CDC
 * @version 1.0
 * @date    2026-03-26
 *
 * @details
 * [OD Index Map — IMU Hub]
 *
 * Communication Profile (0x1000~0x1FFF):
 *   0x1000.00  Device Type        UINT32  RO   0x00080000 (IMU Hub)
 *   0x1001.00  Error Register     UINT8   RO
 *   0x1017.00  Heartbeat Time     UINT16  RW   1000 ms
 *   0x1018.00  Identity Count     UINT8   RO   4
 *   0x1018.01  Vendor ID          UINT32  RO   0x414E474C ("ANGL")
 *   0x1018.02  Product Code       UINT32  RO   0x00080010 (Node 0x08, HW Rev1.0)
 *   0x1018.03  Revision Number    UINT32  RO   0x00010000 (v1.0.0)
 *   0x1018.04  Serial Number      UINT32  RO   0x00000001
 *
 * IMU Device Profile (0x6000~):
 *   0x6000.00~05  Quaternion W    FLOAT32 RO   ch0~5
 *   0x6001.00~05  Quaternion X    FLOAT32 RO
 *   0x6002.00~05  Quaternion Y    FLOAT32 RO
 *   0x6003.00~05  Quaternion Z    FLOAT32 RO
 *   0x6010.00~05  Accel X         FLOAT32 RO   m/s² (or g)
 *   0x6011.00~05  Accel Y         FLOAT32 RO
 *   0x6012.00~05  Accel Z         FLOAT32 RO
 *   0x6020.00~05  Sensor Status   UINT8   RO   ImuSensorType_t
 *   0x6030.00     Stream Rate     UINT8   RW   ms (default 10)
 *   0x6031.00     Active Ch Mask  UINT8   RW   bit0~5
 *   0x6040.00     Command         UINT8   RW   0=stop, 1=stream, 2=calibrate
 *
 * [TPDO1 Mapping — Default]
 *   CH0 Quat W(4) + X(4) + Y(4) + Z(4) + AccX(4) + AccY(4) + AccZ(4) = 28B
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "imu_dop_serial.h"
#include "imu_cdc.h"
#include "module.h"

/* AGR_MW DOP */
#include "agr_dop_types.h"
#include "agr_dop_config.h"
#include "Core/agr_od.h"
#include "Core/agr_od_comm_profile.h"
#include "Core/agr_sdo_protocol.h"
#include "Core/agr_pdo_engine.h"
#include "Transport/Serial/agr_dop_serial.h"

/* UartRxHandler는 System Layer */
#include "uart_rx_handler.h"  /* UartRxHandler_GetSensorType() */

/* Node ID */
#include "imu_node_id.h"

/* Peripheral Test — HW access */
#include "io_manager.h"
#include "ioif_agrb_tim.h"
#include "ioif_agrb_fdcan.h"
#include "system_startup.h"  /* System_GetFdcanHandle() — V2: extern 대신 getter */
#include "main.h"            /* IMU_PWR_EN_Pin/Port */

#include <string.h>

/**
 *-----------------------------------------------------------
 * IMU COMMAND DEFINITIONS (SDO Write → on_write callback)
 *-----------------------------------------------------------
 */

#define IMU_CMD_STOP            0x00
#define IMU_CMD_STREAM_START    0x01
#define IMU_CMD_CALIBRATE       0x02

/**
 *-----------------------------------------------------------
 * PERIPHERAL TEST DEFINITIONS (SDO 0x7F00)
 *-----------------------------------------------------------
 */

#define PERIPH_TEST_IDLE            0x00
#define PERIPH_TEST_UART_IMU0       0x01  /**< USART2 (IMU CH0) — 수신 확인 */
#define PERIPH_TEST_UART_IMU1       0x02  /**< USART3 (IMU CH1) — 수신 확인 */
#define PERIPH_TEST_UART_IMU2       0x03  /**< UART4 (IMU CH2) — 수신 확인 */
#define PERIPH_TEST_UART_IMU3       0x04  /**< UART5 (IMU CH3) — 수신 확인 */
#define PERIPH_TEST_UART_IMU4       0x05  /**< LPUART1 (IMU CH4) — 수신 확인 */
#define PERIPH_TEST_UART_IMU5       0x06  /**< USART1 (IMU CH5) — 수신 확인 */
#define PERIPH_TEST_TIMER           0x07  /**< TIM6/TIM7/TIM15 tick 확인 */
#define PERIPH_TEST_LED             0x08  /**< RGB LED 점등 (PC7/PC8/PC9 + PC6 Power) */
#define PERIPH_TEST_BUTTON          0x09  /**< FUNC_BTN_1 (PB4), FUNC_BTN_2 (PB5) 읽기 */
#define PERIPH_TEST_IMU_POWER       0x0A  /**< IMU_PWR_EN (PA8) 토글 */
#define PERIPH_TEST_FDCAN           0x0B  /**< FDCAN2 Tx FIFO free level */
#define PERIPH_TEST_RUN_ALL         0xFF  /**< 전체 순차 실행 */

#define PERIPH_RESULT_IDLE          0x00
#define PERIPH_RESULT_PASS          0x01
#define PERIPH_RESULT_FAIL          0x02

/**
 *-----------------------------------------------------------
 * OD DATA VARIABLES (live data — 읽기 시 최신 값 반환)
 *-----------------------------------------------------------
 */

/* Communication Profile */
static uint32_t s_device_type   = AGR_DEVICE_TYPE(0x0008, 0);  /* IMU Hub, profile 0 (custom) */
static uint8_t  s_error_reg     = 0;
static uint16_t s_hb_time_ms    = IMU_DOP_HEARTBEAT_PERIOD_MS;
static uint8_t  s_id_count      = 4;
static uint32_t s_vendor_id     = AGR_VENDOR_ID;
static uint32_t s_product_code  = 0x00080010;   /* Node 0x08, HW Rev1.0 */
static uint32_t s_revision      = 0x00010000;   /* v1.0.0 */
static uint32_t s_serial_num    = 0x00000001;

/* IMU Device Profile — Quaternion (RO, 6ch) */
static float s_quat_w[MAX_IMU_CHANNELS] = {0};
static float s_quat_x[MAX_IMU_CHANNELS] = {0};
static float s_quat_y[MAX_IMU_CHANNELS] = {0};
static float s_quat_z[MAX_IMU_CHANNELS] = {0};

/* IMU Device Profile — Acceleration (RO, 6ch) */
static float s_accel_x[MAX_IMU_CHANNELS] = {0};
static float s_accel_y[MAX_IMU_CHANNELS] = {0};
static float s_accel_z[MAX_IMU_CHANNELS] = {0};

/* IMU Device Profile — Gyroscope (RO, 6ch) */
static float s_gyro_x[MAX_IMU_CHANNELS] = {0};
static float s_gyro_y[MAX_IMU_CHANNELS] = {0};
static float s_gyro_z[MAX_IMU_CHANNELS] = {0};

/* IMU Device Profile — Sensor Status (RO, 6ch) */
static uint8_t s_sensor_status[MAX_IMU_CHANNELS] = {0};

/* Control tick (RO) — 1ms counter for seq gap detection */
static uint32_t s_ctrl_tick_ms = 0;

/* IMU Device Profile — Config (RW) */
static uint8_t s_stream_rate_ms = 1;    /**< TPDO period (ms), default 1 = 1kHz */
static uint8_t s_active_ch_mask = 0x03; /**< Bit0~5: ch0~5 active, default ch0+ch1 */
static uint8_t s_command        = 0;    /**< 0=stop, 1=stream, 2=calibrate */

/* Peripheral Test (0x7F00) */
static uint8_t  s_test_cmd       = 0;     /**< 테스트 명령 (write → 즉시 실행) */
static uint8_t  s_test_result    = 0;     /**< 마지막 결과: IDLE/PASS/FAIL */
static uint32_t s_test_detail    = 0;     /**< 테스트별 상세 값 */
static uint16_t s_test_pass_bits = 0;     /**< Run All: 비트별 PASS (bit N = test N) */
static uint16_t s_test_fail_bits = 0;     /**< Run All: 비트별 FAIL (bit N = test N) */

/* Application → System 데이터 주입 (V2: g_sys 직접 참조 제거) */
static ImuDopSerial_ChannelData_t s_live_imus[IMU_DOP_MAX_CHANNELS] = {0};

/**
 *-----------------------------------------------------------
 * ON_WRITE CALLBACKS (SDO Write 완료 시 호출)
 *-----------------------------------------------------------
 */

static void _OnWrite_StreamRate(void);
static void _OnWrite_Command(void);
static void _OnWrite_TestCmd(void);

/**
 *-----------------------------------------------------------
 * OBJECT DICTIONARY TABLE
 *-----------------------------------------------------------
 */

static const AGR_OD_Entry_t s_od_entries[] = {
    /* ===== Communication Profile (0x1000~0x1FFF) ===== */
    AGR_OD_COMM_DEVICE_TYPE(&s_device_type),
    AGR_OD_COMM_ERROR_REGISTER(&s_error_reg),
    AGR_OD_COMM_HEARTBEAT_TIME(&s_hb_time_ms),
    AGR_OD_COMM_IDENTITY_COUNT(&s_id_count),
    AGR_OD_COMM_VENDOR_ID(&s_vendor_id),
    AGR_OD_COMM_PRODUCT_CODE(&s_product_code),
    AGR_OD_COMM_REVISION(&s_revision),
    AGR_OD_COMM_SERIAL(&s_serial_num),

    /* ===== IMU Device Profile (0x6000~) — Quaternion W (RO, 6ch) ===== */
    { 0x6000, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[0], NULL, "ch0_quat_w", "" },
    { 0x6000, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[1], NULL, "ch1_quat_w", "" },
    { 0x6000, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[2], NULL, "ch2_quat_w", "" },
    { 0x6000, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[3], NULL, "ch3_quat_w", "" },
    { 0x6000, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[4], NULL, "ch4_quat_w", "" },
    { 0x6000, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_w[5], NULL, "ch5_quat_w", "" },

    /* ===== Quaternion X (RO, 6ch) ===== */
    { 0x6001, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[0], NULL, "ch0_quat_x", "" },
    { 0x6001, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[1], NULL, "ch1_quat_x", "" },
    { 0x6001, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[2], NULL, "ch2_quat_x", "" },
    { 0x6001, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[3], NULL, "ch3_quat_x", "" },
    { 0x6001, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[4], NULL, "ch4_quat_x", "" },
    { 0x6001, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_x[5], NULL, "ch5_quat_x", "" },

    /* ===== Quaternion Y (RO, 6ch) ===== */
    { 0x6002, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[0], NULL, "ch0_quat_y", "" },
    { 0x6002, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[1], NULL, "ch1_quat_y", "" },
    { 0x6002, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[2], NULL, "ch2_quat_y", "" },
    { 0x6002, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[3], NULL, "ch3_quat_y", "" },
    { 0x6002, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[4], NULL, "ch4_quat_y", "" },
    { 0x6002, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_y[5], NULL, "ch5_quat_y", "" },

    /* ===== Quaternion Z (RO, 6ch) ===== */
    { 0x6003, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[0], NULL, "ch0_quat_z", "" },
    { 0x6003, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[1], NULL, "ch1_quat_z", "" },
    { 0x6003, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[2], NULL, "ch2_quat_z", "" },
    { 0x6003, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[3], NULL, "ch3_quat_z", "" },
    { 0x6003, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[4], NULL, "ch4_quat_z", "" },
    { 0x6003, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_quat_z[5], NULL, "ch5_quat_z", "" },

    /* ===== Accel X (RO, 6ch) ===== */
    { 0x6010, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[0], NULL, "ch0_accel_x", "m/s2" },
    { 0x6010, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[1], NULL, "ch1_accel_x", "m/s2" },
    { 0x6010, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[2], NULL, "ch2_accel_x", "m/s2" },
    { 0x6010, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[3], NULL, "ch3_accel_x", "m/s2" },
    { 0x6010, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[4], NULL, "ch4_accel_x", "m/s2" },
    { 0x6010, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_x[5], NULL, "ch5_accel_x", "m/s2" },

    /* ===== Accel Y (RO, 6ch) ===== */
    { 0x6011, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[0], NULL, "ch0_accel_y", "m/s2" },
    { 0x6011, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[1], NULL, "ch1_accel_y", "m/s2" },
    { 0x6011, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[2], NULL, "ch2_accel_y", "m/s2" },
    { 0x6011, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[3], NULL, "ch3_accel_y", "m/s2" },
    { 0x6011, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[4], NULL, "ch4_accel_y", "m/s2" },
    { 0x6011, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_y[5], NULL, "ch5_accel_y", "m/s2" },

    /* ===== Accel Z (RO, 6ch) ===== */
    { 0x6012, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[0], NULL, "ch0_accel_z", "m/s2" },
    { 0x6012, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[1], NULL, "ch1_accel_z", "m/s2" },
    { 0x6012, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[2], NULL, "ch2_accel_z", "m/s2" },
    { 0x6012, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[3], NULL, "ch3_accel_z", "m/s2" },
    { 0x6012, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[4], NULL, "ch4_accel_z", "m/s2" },
    { 0x6012, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_accel_z[5], NULL, "ch5_accel_z", "m/s2" },

    /* ===== Gyroscope X (RO, 6ch) ===== */
    { 0x6013, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[0], NULL, "ch0_gyro_x", "deg/s" },
    { 0x6013, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[1], NULL, "ch1_gyro_x", "deg/s" },
    { 0x6013, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[2], NULL, "ch2_gyro_x", "deg/s" },
    { 0x6013, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[3], NULL, "ch3_gyro_x", "deg/s" },
    { 0x6013, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[4], NULL, "ch4_gyro_x", "deg/s" },
    { 0x6013, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_x[5], NULL, "ch5_gyro_x", "deg/s" },

    /* ===== Gyroscope Y (RO, 6ch) ===== */
    { 0x6014, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[0], NULL, "ch0_gyro_y", "deg/s" },
    { 0x6014, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[1], NULL, "ch1_gyro_y", "deg/s" },
    { 0x6014, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[2], NULL, "ch2_gyro_y", "deg/s" },
    { 0x6014, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[3], NULL, "ch3_gyro_y", "deg/s" },
    { 0x6014, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[4], NULL, "ch4_gyro_y", "deg/s" },
    { 0x6014, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_y[5], NULL, "ch5_gyro_y", "deg/s" },

    /* ===== Gyroscope Z (RO, 6ch) ===== */
    { 0x6015, 0x00, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[0], NULL, "ch0_gyro_z", "deg/s" },
    { 0x6015, 0x01, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[1], NULL, "ch1_gyro_z", "deg/s" },
    { 0x6015, 0x02, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[2], NULL, "ch2_gyro_z", "deg/s" },
    { 0x6015, 0x03, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[3], NULL, "ch3_gyro_z", "deg/s" },
    { 0x6015, 0x04, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[4], NULL, "ch4_gyro_z", "deg/s" },
    { 0x6015, 0x05, AGR_TYPE_FLOAT32, 4, AGR_ACCESS_RO, &s_gyro_z[5], NULL, "ch5_gyro_z", "deg/s" },

    /* ===== Sensor Status (RO, 6ch) ===== */
    { 0x6020, 0x00, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[0], NULL, "ch0_status", "" },
    { 0x6020, 0x01, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[1], NULL, "ch1_status", "" },
    { 0x6020, 0x02, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[2], NULL, "ch2_status", "" },
    { 0x6020, 0x03, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[3], NULL, "ch3_status", "" },
    { 0x6020, 0x04, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[4], NULL, "ch4_status", "" },
    { 0x6020, 0x05, AGR_TYPE_UINT8, 1, AGR_ACCESS_RO, &s_sensor_status[5], NULL, "ch5_status", "" },

    /* ===== Control Tick — 1ms counter for seq gap detection ===== */
    { 0x6050, 0x00, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_ctrl_tick_ms, NULL,
      "ctrl_tick_ms",   "ms" },

    /* ===== Config (RW) ===== */
    { 0x6030, 0x00, AGR_TYPE_UINT8, 1, AGR_ACCESS_RW, &s_stream_rate_ms, _OnWrite_StreamRate,
      "stream_rate",    "ms" },
    { 0x6031, 0x00, AGR_TYPE_UINT8, 1, AGR_ACCESS_RW, &s_active_ch_mask, NULL,
      "active_ch_mask", "" },
    { 0x6040, 0x00, AGR_TYPE_UINT8, 1, AGR_ACCESS_RW, &s_command,        _OnWrite_Command,
      "command",        "" },

    /* ===== Peripheral Test (0x7F00) ===== */
    { 0x7F00, 0x00, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_test_cmd,       _OnWrite_TestCmd,
      "test_cmd",       "" },
    { 0x7F00, 0x01, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, &s_test_result,    NULL,
      "test_result",    "" },
    { 0x7F00, 0x02, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_test_detail,    NULL,
      "test_detail",    "" },
    { 0x7F00, 0x03, AGR_TYPE_UINT16, 2, AGR_ACCESS_RO, &s_test_pass_bits, NULL,
      "test_pass_bits", "" },
    { 0x7F00, 0x04, AGR_TYPE_UINT16, 2, AGR_ACCESS_RO, &s_test_fail_bits, NULL,
      "test_fail_bits", "" },
};

#define IMU_OD_ENTRY_COUNT  (sizeof(s_od_entries) / sizeof(s_od_entries[0]))

/**
 *-----------------------------------------------------------
 * TPDO1 MAPPING (Static Definition) — 6ch × 10필드 + 6 status + 1 ctrl_tick = 67 entries, 250B
 *-----------------------------------------------------------
 * Per channel: Quat W(4)+X(4)+Y(4)+Z(4) + AccX(4)+AccY(4)+AccZ(4)
 *              + GyrX(4)+GyrY(4)+GyrZ(4) = 40B
 * Tail: CH0~CH5 sensor_status (1B each) = 6B
 * Total: 6 × 40 + 6 = 246B. 미연결 채널은 0으로 전송 (sensor_status=0으로 판별)
 */

#define TPDO_CH_SIZE    40  /* 1채널당 바이트 수 (Quat4+Acc3+Gyr3 × 4B) */
#define TPDO_STATUS_OFFSET  (TPDO_CH_SIZE * MAX_IMU_CHANNELS)  /* 240 */

static const AGR_PDO_MapEntry_t s_tpdo1_map[] = {
    /* CH0 (offset 0~39) */
    { 0x6000, 0x00,  0, 4 },  { 0x6001, 0x00,  4, 4 },  { 0x6002, 0x00,  8, 4 },  { 0x6003, 0x00, 12, 4 },
    { 0x6010, 0x00, 16, 4 },  { 0x6011, 0x00, 20, 4 },  { 0x6012, 0x00, 24, 4 },
    { 0x6013, 0x00, 28, 4 },  { 0x6014, 0x00, 32, 4 },  { 0x6015, 0x00, 36, 4 },
    /* CH1 (offset 40~79) */
    { 0x6000, 0x01, 40, 4 },  { 0x6001, 0x01, 44, 4 },  { 0x6002, 0x01, 48, 4 },  { 0x6003, 0x01, 52, 4 },
    { 0x6010, 0x01, 56, 4 },  { 0x6011, 0x01, 60, 4 },  { 0x6012, 0x01, 64, 4 },
    { 0x6013, 0x01, 68, 4 },  { 0x6014, 0x01, 72, 4 },  { 0x6015, 0x01, 76, 4 },
    /* CH2 (offset 80~119) */
    { 0x6000, 0x02, 80, 4 },  { 0x6001, 0x02, 84, 4 },  { 0x6002, 0x02, 88, 4 },  { 0x6003, 0x02, 92, 4 },
    { 0x6010, 0x02, 96, 4 },  { 0x6011, 0x02, 100, 4 }, { 0x6012, 0x02, 104, 4 },
    { 0x6013, 0x02, 108, 4 }, { 0x6014, 0x02, 112, 4 }, { 0x6015, 0x02, 116, 4 },
    /* CH3 (offset 120~159) */
    { 0x6000, 0x03, 120, 4 }, { 0x6001, 0x03, 124, 4 }, { 0x6002, 0x03, 128, 4 }, { 0x6003, 0x03, 132, 4 },
    { 0x6010, 0x03, 136, 4 }, { 0x6011, 0x03, 140, 4 }, { 0x6012, 0x03, 144, 4 },
    { 0x6013, 0x03, 148, 4 }, { 0x6014, 0x03, 152, 4 }, { 0x6015, 0x03, 156, 4 },
    /* CH4 (offset 160~199) */
    { 0x6000, 0x04, 160, 4 }, { 0x6001, 0x04, 164, 4 }, { 0x6002, 0x04, 168, 4 }, { 0x6003, 0x04, 172, 4 },
    { 0x6010, 0x04, 176, 4 }, { 0x6011, 0x04, 180, 4 }, { 0x6012, 0x04, 184, 4 },
    { 0x6013, 0x04, 188, 4 }, { 0x6014, 0x04, 192, 4 }, { 0x6015, 0x04, 196, 4 },
    /* CH5 (offset 200~239) */
    { 0x6000, 0x05, 200, 4 }, { 0x6001, 0x05, 204, 4 }, { 0x6002, 0x05, 208, 4 }, { 0x6003, 0x05, 212, 4 },
    { 0x6010, 0x05, 216, 4 }, { 0x6011, 0x05, 220, 4 }, { 0x6012, 0x05, 224, 4 },
    { 0x6013, 0x05, 228, 4 }, { 0x6014, 0x05, 232, 4 }, { 0x6015, 0x05, 236, 4 },
    /* Sensor Status (offset 240~245) — 0=unknown, 1=connected, 2+=error */
    { 0x6020, 0x00, 240, 1 }, { 0x6020, 0x01, 241, 1 }, { 0x6020, 0x02, 242, 1 },
    { 0x6020, 0x03, 243, 1 }, { 0x6020, 0x04, 244, 1 }, { 0x6020, 0x05, 245, 1 },
    /* ctrl_tick_ms (offset 246~249) — 1ms counter for seq gap detection */
    { 0x6050, 0x00, 246, 4 },
};

#define TPDO_TOTAL_SIZE  (TPDO_STATUS_OFFSET + MAX_IMU_CHANNELS + 4)  /* 250B */

static const AGR_PDO_Def_t s_tpdo1_def = {
    .cob_id       = 0,  /* Serial transport에서는 미사용 (MsgType으로 구분) */
    .map_entries  = s_tpdo1_map,
    .map_count    = sizeof(s_tpdo1_map) / sizeof(s_tpdo1_map[0]),
    .payload_size = TPDO_TOTAL_SIZE,  /* 250B */
};

/**
 *-----------------------------------------------------------
 * DOP + SERIAL CONTEXT
 *-----------------------------------------------------------
 */

static AGR_DOP_Ctx_t    s_dop_ctx;
static AGR_Serial_Ctx_t s_serial_ctx;

/**
 *-----------------------------------------------------------
 * TIMING
 *-----------------------------------------------------------
 */

static uint32_t s_last_tpdo_ms = 0;
static uint32_t s_last_hb_ms   = 0;

/**
 *-----------------------------------------------------------
 * PRIVATE FUNCTION PROTOTYPES
 *-----------------------------------------------------------
 */

static int32_t _CdcTxFunc(const uint8_t* data, uint32_t len);
static void    _UpdateOdFromLiveData(void);

/**
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS
 *-----------------------------------------------------------
 */

void ImuDopSerial_Init(void)
{
    /* ===== 1. DOP Context 초기화 ===== */
    memset(&s_dop_ctx, 0, sizeof(s_dop_ctx));

    s_dop_ctx.od.entries     = s_od_entries;
    s_dop_ctx.od.entry_count = IMU_OD_ENTRY_COUNT;

    s_dop_ctx.node_id        = AGR_NODE_ID_IMU_HUB;
    s_dop_ctx.target_node_id = 0;  /* Serial: point-to-point, 특정 대상 없음 */
    s_dop_ctx.tx_func        = NULL;  /* CAN-FD tx — Serial에서는 미사용 */

    /* Static PDO 정의 */
    s_dop_ctx.tx_pdo_defs    = &s_tpdo1_def;
    s_dop_ctx.tx_pdo_count   = 1;
    s_dop_ctx.rx_pdo_defs    = NULL;
    s_dop_ctx.rx_pdo_count   = 0;

    /* ===== 2. Default TPDO1 매핑 로드 (USB-CDC: PnP 없이 즉시 스트리밍) ===== */
    for (uint8_t i = 0; i < s_tpdo1_def.map_count; i++) {
        AGR_PDO_AddMap(&s_dop_ctx.tx_pdo_map[0],
                       &s_dop_ctx.od,
                       s_tpdo1_map[i].od_index,
                       s_tpdo1_map[i].od_subindex);
    }

    /* ===== 3. Serial Transport 초기화 ===== */
    AGR_Serial_Init(&s_serial_ctx, &s_dop_ctx, _CdcTxFunc);

    /* ===== 4. Timing 초기화 ===== */
    s_last_tpdo_ms = 0;
    s_last_hb_ms   = 0;
}

void ImuDopSerial_FeedRxData(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) return;

    AGR_Serial_ProcessRxData(&s_serial_ctx, data, len);
}

/**
 * @brief [Main Task — TIM16 1ms] IPO Output 직후 TPDO 전송
 * @details Period divider 내장: s_stream_rate_ms마다 실제 전송.
 *          OD 변수 갱신 → TPDO 인코딩 → COBS → CDC ring buffer (non-blocking).
 */
void ImuDopSerial_SendTpdo(uint32_t now_ms)
{
    if (!ImuCdc_IsConnected()) return;

    /* Period divider: 1ms 호출 중 N ms마다만 실행 */
    if (now_ms - s_last_tpdo_ms < s_stream_rate_ms) return;
    s_last_tpdo_ms = now_ms;

    /* OD 변수를 최신 센서 상태로 갱신 */
    s_ctrl_tick_ms = now_ms;
    _UpdateOdFromLiveData();

    /* TPDO1 전송 */
    AGR_Serial_SendTxPDO(&s_serial_ctx, 1);
}

/**
 * @brief [System Task — TIM7 1ms] Heartbeat 전송
 */
void ImuDopSerial_RunPeriodic(uint32_t now_ms)
{
    if (!ImuCdc_IsConnected()) return;

    /* Heartbeat 전송 (1초 주기) */
    if (now_ms - s_last_hb_ms >= s_hb_time_ms) {
        s_last_hb_ms = now_ms;
        AGR_Serial_SendHeartbeat(&s_serial_ctx, 0x05);  /* Operational */
    }
}

bool ImuDopSerial_IsActive(void)
{
    return s_serial_ctx.initialized && ImuCdc_IsConnected();
}

/**
 *-----------------------------------------------------------
 * PRIVATE FUNCTIONS
 *-----------------------------------------------------------
 */

/**
 * @brief CDC TX 래퍼 — AGR_Serial_TxFunc_t 시그니처
 */
static int32_t _CdcTxFunc(const uint8_t* data, uint32_t len)
{
    if (!ImuCdc_IsConnected()) return -1;
    if (len > UINT16_MAX) return -2;

    int ret = ImuCdc_Send(data, (uint16_t)len);
    return (ret == 0) ? 0 : -1;
}

/**
 * @brief OD 변수를 g_sys 실시간 센서 데이터에서 갱신
 * @note  TPDO 전송 전에 호출하여 최신 값을 보장.
 */
static void _UpdateOdFromLiveData(void)
{
    bool any_active = false;

    for (uint8_t ch = 0; ch < MAX_IMU_CHANNELS; ch++) {
        /* Active Ch Mask 체크 — 비활성 채널은 OD 갱신 스킵 (0 유지) */
        if (!(s_active_ch_mask & (1U << ch))) {
            s_sensor_status[ch] = (uint8_t)IMU_SENSOR_UNKNOWN;
            continue;
        }

        const ImuDopSerial_ChannelData_t *imu = &s_live_imus[ch];

        if (imu->is_connected) {
            s_quat_w[ch] = imu->q_w;
            s_quat_x[ch] = imu->q_x;
            s_quat_y[ch] = imu->q_y;
            s_quat_z[ch] = imu->q_z;

            s_accel_x[ch] = imu->acc_x;
            s_accel_y[ch] = imu->acc_y;
            s_accel_z[ch] = imu->acc_z;

            s_gyro_x[ch] = imu->gyr_x;
            s_gyro_y[ch] = imu->gyr_y;
            s_gyro_z[ch] = imu->gyr_z;

            s_sensor_status[ch] = imu->sensor_type;
            any_active = true;
        } else {
            s_sensor_status[ch] = (uint8_t)IMU_SENSOR_UNKNOWN;
        }
    }

    /* Error Register 갱신 — 활성 채널 중 연결된 센서 없으면 Communication error */
    s_error_reg = any_active ? 0 : AGR_ERR_REG_COMMUNICATION;
}

/**
 *-----------------------------------------------------------
 * ON_WRITE CALLBACKS
 *-----------------------------------------------------------
 */

static void _OnWrite_StreamRate(void)
{
    /* Clamp to valid range: 1~250 ms */
    if (s_stream_rate_ms < 1) s_stream_rate_ms = 1;
    if (s_stream_rate_ms > 250) s_stream_rate_ms = 250;
}

static void _OnWrite_Command(void)
{
    switch (s_command) {
        case IMU_CMD_STOP:
            /* 스트리밍 중지 — TPDO 전송 억제는 별도 flag 필요 시 추가 */
            break;

        case IMU_CMD_STREAM_START:
            /* 스트리밍 시작 — 현재 구조에서는 CDC 연결 시 자동 전송 */
            break;

        case IMU_CMD_CALIBRATE:
            /* TODO: 센서 캘리브레이션 트리거 */
            break;

        default:
            break;
    }
}

/**
 *-----------------------------------------------------------
 * PERIPHERAL TEST
 *-----------------------------------------------------------
 */

/* FDCAN handle — V2: System_GetFdcanHandle() 사용 */

/**
 * @brief 단일 페리페럴 테스트 실행
 * @param test_id 테스트 ID (PERIPH_TEST_xxx)
 * @return true=PASS, false=FAIL
 * @note s_test_detail에 테스트별 상세 값 기록
 */
static bool _RunSingleTest(uint8_t test_id)
{
    switch (test_id) {

    case PERIPH_TEST_UART_IMU0:
    case PERIPH_TEST_UART_IMU1:
    case PERIPH_TEST_UART_IMU2:
    case PERIPH_TEST_UART_IMU3:
    case PERIPH_TEST_UART_IMU4:
    case PERIPH_TEST_UART_IMU5: {
        /* UART IMU CH(N): 센서 연결 + 최근 수신 확인 */
        uint8_t ch = test_id - PERIPH_TEST_UART_IMU0;
        uint32_t now = IOIF_TIM_GetTick();
        uint32_t last_ts = s_live_imus[ch].last_update_time;
        bool connected = s_live_imus[ch].is_connected;
        /* detail: [connected:8 | age_ms:24] */
        uint32_t age_ms = (last_ts > 0) ? (now - last_ts) : 0xFFFFFF;
        s_test_detail = ((uint32_t)(connected ? 1 : 0) << 24) | (age_ms & 0x00FFFFFF);
        return connected;
    }

    case PERIPH_TEST_TIMER: {
        /* TIM tick 카운터 — tick > 0이면 PASS */
        uint32_t tick = IOIF_TIM_GetTick();
        s_test_detail = tick;
        return (tick > 0);
    }

    case PERIPH_TEST_LED: {
        /* GPIO: RGB LED → Error 표시 후 PASS (시각 확인용) */
        IO_SetLinkState(IO_LINK_STATE_ERROR);
        s_test_detail = (uint32_t)IO_LINK_STATE_ERROR;
        return true;
    }

    case PERIPH_TEST_BUTTON: {
        /* GPIO: 버튼 2개 상태 읽기 — 항상 PASS (값만 보고) */
        uint8_t btn1 = IO_GetButtonState(1) ? 1 : 0;
        uint8_t btn2 = IO_GetButtonState(2) ? 1 : 0;
        s_test_detail = ((uint32_t)btn1 << 1) | (uint32_t)btn2;
        return true;
    }

    case PERIPH_TEST_IMU_POWER: {
        /* GPIO: IMU_PWR_EN (PA8) 토글 — 항상 PASS */
        HAL_GPIO_TogglePin(IMU_PWR_EN_GPIO_Port, IMU_PWR_EN_Pin);
        s_test_detail = 1;  /* toggled */
        return true;
    }

    case PERIPH_TEST_FDCAN: {
        /* FDCAN2: Tx FIFO free level > 0이면 PASS */
        uint32_t fifo_free = IOIF_FDCAN_GetTxFifoFreeLevel(System_GetFdcanHandle());
        s_test_detail = fifo_free;
        return (fifo_free > 0);
    }

    default:
        s_test_detail = 0;
        return false;
    }
}

/**
 * @brief SDO Write 0x7F00.00 콜백 — 테스트 실행
 */
static void _OnWrite_TestCmd(void)
{
    uint8_t cmd = s_test_cmd;

    if (cmd == PERIPH_TEST_IDLE) {
        s_test_result = PERIPH_RESULT_IDLE;
        s_test_detail = 0;
        s_test_pass_bits = 0;
        s_test_fail_bits = 0;
        return;
    }

    if (cmd == PERIPH_TEST_RUN_ALL) {
        /* 전체 테스트 순차 실행 */
        s_test_pass_bits = 0;
        s_test_fail_bits = 0;
        uint8_t pass_count = 0;
        uint8_t fail_count = 0;

        static const uint8_t s_all_tests[] = {
            PERIPH_TEST_UART_IMU0,
            PERIPH_TEST_UART_IMU1,
            PERIPH_TEST_UART_IMU2,
            PERIPH_TEST_UART_IMU3,
            PERIPH_TEST_UART_IMU4,
            PERIPH_TEST_UART_IMU5,
            PERIPH_TEST_TIMER,
            PERIPH_TEST_LED,
            PERIPH_TEST_BUTTON,
            PERIPH_TEST_IMU_POWER,
            PERIPH_TEST_FDCAN,
        };

        for (uint8_t i = 0; i < sizeof(s_all_tests); i++) {
            bool ok = _RunSingleTest(s_all_tests[i]);
            if (ok) {
                s_test_pass_bits |= (1U << s_all_tests[i]);
                pass_count++;
            } else {
                s_test_fail_bits |= (1U << s_all_tests[i]);
                fail_count++;
            }
        }

        /* detail: [pass_count:8 | fail_count:8 | total:16] */
        s_test_detail = ((uint32_t)pass_count << 24) | ((uint32_t)fail_count << 16)
                      | (uint32_t)(pass_count + fail_count);
        s_test_result = (fail_count == 0) ? PERIPH_RESULT_PASS : PERIPH_RESULT_FAIL;

        /* LED 원복 */
        IO_SetLinkState(IO_LINK_STATE_OPERATIONAL);
        return;
    }

    /* 단일 테스트 */
    bool ok = _RunSingleTest(cmd);
    s_test_result = ok ? PERIPH_RESULT_PASS : PERIPH_RESULT_FAIL;
}

/* ===================================================================
 * PUBLIC — 데이터 주입 (Application → System)
 * =================================================================== */

void ImuDopSerial_UpdateLiveData(const ImuDopSerial_ChannelData_t *channels, uint8_t count)
{
    if (channels == NULL) return;
    uint8_t n = (count > IMU_DOP_MAX_CHANNELS) ? IMU_DOP_MAX_CHANNELS : count;
    for (uint8_t i = 0; i < n; i++) {
        s_live_imus[i] = channels[i];
    }
}
