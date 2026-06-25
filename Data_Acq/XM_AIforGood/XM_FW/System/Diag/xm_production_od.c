/**
 ******************************************************************************
 * @file    xm_production_od.c
 * @author  HyundoKim
 * @brief   XM Production / SI 검증 OD 정의 + DOP Slave 컨텍스트
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_production_od.h"
#include "xm_periph_stimulus.h"
#include <string.h>

/* ===== Bound OD variables ===== */

/* 0x1000 Device Type (RO) */
static uint32_t s_device_type    = XM_DEVICE_TYPE;

/* 0x1008 Manufacturer Device Name (RO, BLOB) */
static const char s_device_name[] = "XM Extension Module";

/* 0x100A Manufacturer SW Version (RO, BLOB) — Git short SHA + build date */
#ifndef XM_FW_VERSION_STR
#define XM_FW_VERSION_STR  "XM-FW dev"
#endif
static const char s_fw_version[] = XM_FW_VERSION_STR;

/* 0x1018 Identity Object (CiA 301) — Vendor / Product / Revision / Serial */
static uint32_t s_vendor_id      = XM_VENDOR_ID;
static uint32_t s_product_code   = XM_PRODUCT_CODE;
#if defined(XM_HW_REV2_0)
static uint32_t s_revision_no    = XM_REVISION_REV2_0;
#else
static uint32_t s_revision_no    = XM_REVISION_REV1_1;
#endif
static uint32_t s_serial_no      = 0u;  /* SetSerialNumber 로 주입 */

/* 0x7000 Test Mode Enable (RW) — 0=Normal, 1=Test/Production */
static uint8_t  s_test_mode      = 0u;

/* 0x7E00 Peripheral Stimulus toggles (RW) — 회로 SI 측정용.
 * sub 0 = entry count (RO), sub 1~8 = stimulus 개별 ON/OFF.
 * subindex 는 XM_Stimulus_e 와 1:1 매핑.
 */
static uint8_t  s_stim_count     = (uint8_t)(XM_STIM_COUNT_ - 1);  /* 8 */
static uint8_t  s_stim_state[XM_STIM_COUNT_] = {0};                /* idx 0 미사용 */

/* Forward callbacks */
static void _OnTestModeWrite(void);
static void _OnStim_Fdcan1_Write(void);
static void _OnStim_Fdcan2_Write(void);
static void _OnStim_SpiPsram_Write(void);
static void _OnStim_SpiRtc_Write(void);
static void _OnStim_SpiLed_Write(void);
static void _OnStim_UartGrf_Write(void);
static void _OnStim_Mdio_Write(void);
static void _OnStim_Rmii_Write(void);

/* ===== OD Table ===== */

static const AGR_OD_Entry_t s_od_entries[] = {
    /* Communication Profile */
    { 0x1000, 0x00, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_device_type,    NULL,              "device_type",    "" },
    { 0x1008, 0x00, AGR_TYPE_BLOB,   sizeof(s_device_name) - 1u, AGR_ACCESS_RO, (void*)s_device_name, NULL, "device_name", "" },
    { 0x100A, 0x00, AGR_TYPE_BLOB,   sizeof(s_fw_version)  - 1u, AGR_ACCESS_RO, (void*)s_fw_version,  NULL, "fw_version",  "" },
    { 0x1018, 0x01, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_vendor_id,      NULL,              "vendor_id",      "" },
    { 0x1018, 0x02, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_product_code,   NULL,              "product_code",   "" },
    { 0x1018, 0x03, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_revision_no,    NULL,              "revision_no",    "" },
    { 0x1018, 0x04, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, &s_serial_no,      NULL,              "serial_no",      "" },

    /* Production / Test Mode */
    { 0x7000, 0x00, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_test_mode,      _OnTestModeWrite,  "test_mode",      "" },

    /* 0x7E00 Peripheral Stimulus (회로 SI 측정용 — 오실로) */
    { 0x7E00, 0x00, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, &s_stim_count,                NULL,                    "stim_count",     ""    },
    { 0x7E00, 0x01, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_FDCAN1],    _OnStim_Fdcan1_Write,    "stim_fdcan1",    "0/1" },
    { 0x7E00, 0x02, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_FDCAN2],    _OnStim_Fdcan2_Write,    "stim_fdcan2",    "0/1" },
    { 0x7E00, 0x03, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_SPI_PSRAM], _OnStim_SpiPsram_Write,  "stim_spi_psram", "0/1" },
    { 0x7E00, 0x04, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_SPI_RTC],   _OnStim_SpiRtc_Write,    "stim_spi_rtc",   "0/1" },
    { 0x7E00, 0x05, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_SPI_LED],   _OnStim_SpiLed_Write,    "stim_spi_led",   "0/1" },
    { 0x7E00, 0x06, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_UART_GRF],  _OnStim_UartGrf_Write,   "stim_uart_grf",  "0/1" },
    { 0x7E00, 0x07, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_MDIO],      _OnStim_Mdio_Write,      "stim_mdio",      "0/1" },
    { 0x7E00, 0x08, AGR_TYPE_UINT8,  1, AGR_ACCESS_RW, &s_stim_state[XM_STIM_RMII],      _OnStim_Rmii_Write,      "stim_rmii",      "0/1" },

    /* 0x7E10 Stimulus Diagnostics (RO) — 모든 stimulus err_count 노출.
     * v3 (HW팀 피드백) 에서 FDCAN/UART/MDIO/RMII 도 가비지 TX stimulus 활성화 →
     * 전체 8개 sub 모두 진단 카운터 보강. sensor-studio polling 으로 모니터링.
     */
    { 0x7E10, 0x00, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_stim_psram_alloc_failed,           NULL, "stim_psram_alloc_fail", "0/1" },
    { 0x7E10, 0x01, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_FDCAN1],    NULL, "stim_err_fdcan1",       ""    },
    { 0x7E10, 0x02, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_FDCAN2],    NULL, "stim_err_fdcan2",       ""    },
    { 0x7E10, 0x03, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_SPI_PSRAM], NULL, "stim_err_spi_psram",    ""    },
    { 0x7E10, 0x04, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_SPI_RTC],   NULL, "stim_err_spi_rtc",      ""    },
    { 0x7E10, 0x05, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_SPI_LED],   NULL, "stim_err_spi_led",      ""    },
    { 0x7E10, 0x06, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_UART_GRF],  NULL, "stim_err_uart_grf",     ""    },
    { 0x7E10, 0x07, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_MDIO],      NULL, "stim_err_mdio",         ""    },
    { 0x7E10, 0x08, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_err_count[XM_STIM_RMII],      NULL, "stim_err_rmii",         ""    },
    /* [2026-05-14] FDCAN Ch2 stimulus enable 거부 사유 — "ON 후 즉시 OFF" 시 GUI 원인 표시.
     * 0=없음, 1=SM 연결 중, 2=TEC 누적, 3=Restricted Mode 진입 실패 */
    { 0x7E10, 0x09, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_stim_fdcan2_reject_reason,         NULL, "stim_fdcan2_reject",    "0..3" },

    /* 0x7E20 Stimulus Run Counter (RO) — 매 100ms (v2 task 분리 후) step 호출 시 ++.
     * sensor-studio polling 으로 이전 값 대비 증가하면 "실행 중" 검증.
     * 1Hz polling 기준 약 10씩 증가하면 정상 (100ms 주기 × 10/s).
     * v3: 전체 8개 stimulus 모두 노출.
     */
    { 0x7E20, 0x01, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_FDCAN1],    NULL, "stim_run_fdcan1",       ""    },
    { 0x7E20, 0x02, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_FDCAN2],    NULL, "stim_run_fdcan2",       ""    },
    { 0x7E20, 0x03, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_SPI_PSRAM], NULL, "stim_run_spi_psram",    ""    },
    { 0x7E20, 0x04, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_SPI_RTC],   NULL, "stim_run_spi_rtc",      ""    },
    { 0x7E20, 0x05, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_SPI_LED],   NULL, "stim_run_spi_led",      ""    },
    { 0x7E20, 0x06, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_UART_GRF],  NULL, "stim_run_uart_grf",     ""    },
    { 0x7E20, 0x07, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_MDIO],      NULL, "stim_run_mdio",         ""    },
    { 0x7E20, 0x08, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_run_count[XM_STIM_RMII],      NULL, "stim_run_rmii",         ""    },

    /* 0x7E30 Connected Status (RO) — 평시 트래픽 indicator. 1ms tick 마다 IsConnected()
     * API 결과 mirror. sensor-studio 비활성 row (USB/FDCAN/UART) status_pill 갱신용.
     */
    { 0x7E30, 0x00, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_xm_status_connected[XM_STIM_NONE],   NULL, "status_usb_connected",    "0/1" },
    { 0x7E30, 0x01, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_xm_status_connected[XM_STIM_FDCAN1], NULL, "status_fdcan1_cm",        "0/1" },
    { 0x7E30, 0x02, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_xm_status_connected[XM_STIM_FDCAN2], NULL, "status_fdcan2_sm",        "0/1" },
    { 0x7E30, 0x06, AGR_TYPE_UINT8,  1, AGR_ACCESS_RO, (void*)&g_xm_status_connected[XM_STIM_UART_GRF], NULL, "status_uart_grf",       "0/1" },

    /* 0x7E40 reject_reason 은 0x7E10:09 (Diagnostics 그룹) 로 통합됨 — 중복 제거 */

    /* 0x7E60 Stimulus Last Data (RO) — 각 stimulus 마지막 송/수신 데이터 (uint32).
     * sensor-studio 가 hex 로 표시 → "실제 데이터가 흐른다" 직관 검증 (오실로 없이).
     * Sub 의미는 xm_periph_stimulus.h 의 g_stim_last_data 주석 참조.
     */
    { 0x7E60, 0x01, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_FDCAN1],    NULL, "stim_data_fdcan1",      "hex" },
    { 0x7E60, 0x02, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_FDCAN2],    NULL, "stim_data_fdcan2",      "hex" },
    { 0x7E60, 0x03, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_SPI_PSRAM], NULL, "stim_data_spi_psram",   "hex" },
    { 0x7E60, 0x04, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_SPI_RTC],   NULL, "stim_data_spi_rtc",     "hex" },
    { 0x7E60, 0x05, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_SPI_LED],   NULL, "stim_data_spi_led",     "hex" },
    { 0x7E60, 0x06, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_UART_GRF],  NULL, "stim_data_uart_grf",    "hex" },
    { 0x7E60, 0x07, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_MDIO],      NULL, "stim_data_mdio",        "hex" },
    { 0x7E60, 0x08, AGR_TYPE_UINT32, 4, AGR_ACCESS_RO, (void*)&g_stim_last_data[XM_STIM_RMII],      NULL, "stim_data_rmii",        "hex" },
};

/* ===== DOP / Serial Contexts ===== */

static AGR_DOP_Ctx_t    s_dop_ctx;
static AGR_Serial_Ctx_t s_serial_ctx;
static bool             s_initialized = false;

/* AGR_Serial_Init 은 cdc_dop_router 에서 호출 (TX 함수 바인딩과 동시에).
 * 여기서는 OD 테이블만 dop_ctx 에 결선한다.
 */
void XM_ProductionOD_Init(void)
{
    if (s_initialized) return;

    memset(&s_dop_ctx, 0, sizeof(s_dop_ctx));
    s_dop_ctx.od.entries     = s_od_entries;
    s_dop_ctx.od.entry_count = (uint16_t)(sizeof(s_od_entries) / sizeof(s_od_entries[0]));
    s_dop_ctx.node_id        = XM_PRODUCTION_NODE_ID;
    s_dop_ctx.target_node_id = 0u;
    s_dop_ctx.tx_func        = NULL;  /* Serial Transport 자체 TX 함수 사용 */

    memset(&s_serial_ctx, 0, sizeof(s_serial_ctx));

    s_initialized = true;
}

AGR_Serial_Ctx_t* XM_ProductionOD_GetSerialCtx(void)
{
    return s_initialized ? &s_serial_ctx : NULL;
}

AGR_DOP_Ctx_t* XM_ProductionOD_GetDopCtx(void)
{
    return s_initialized ? &s_dop_ctx : NULL;
}

bool XM_ProductionOD_IsTestModeEnabled(void)
{
    return s_test_mode != 0u;
}

void XM_ProductionOD_SetSerialNumber(uint32_t serial)
{
    s_serial_no = serial;
}

/* ===== Callbacks ===== */

static void _OnTestModeWrite(void)
{
    /* Phase 0: 게이트 비트만 토글. 페리페럴 진입/이탈 액션은 Phase 2 에서 추가.
     * Test Mode 진입 시 PhAI Auto-Stream 차단은 cdc_dop_router 가 책임진다.
     */
}

/* 0x7E00 stimulus write callbacks — 각각 해당 stimulus 토글 적용.
 * AGR_OD 의 on_write 콜백 시그니처가 인자 없음이라 subindex 별 wrapper 8개.
 * 미구현 stimulus (Phase 2.1 기준 FDCAN1 외 전부) 는 SetEnabled 가 false 반환
 * 하지만, OD value 자체는 이미 갱신되어 있으므로 사용자에게 "수락된 것처럼"
 * 보일 수 있다. → 거부 시 OD value 도 원복.
 */
#define STIM_WRITE_HANDLER(STIM_ID)                                          \
    do {                                                                     \
        bool on = (s_stim_state[STIM_ID] != 0u);                             \
        if (!XM_Stimulus_SetEnabled((STIM_ID), on)) {                        \
            s_stim_state[STIM_ID] = 0u;  /* 미구현 거부 시 OFF 로 원복 */    \
        }                                                                    \
    } while (0)

static void _OnStim_Fdcan1_Write(void)   { STIM_WRITE_HANDLER(XM_STIM_FDCAN1);    }
static void _OnStim_Fdcan2_Write(void)   { STIM_WRITE_HANDLER(XM_STIM_FDCAN2);    }
static void _OnStim_SpiPsram_Write(void) { STIM_WRITE_HANDLER(XM_STIM_SPI_PSRAM); }
static void _OnStim_SpiRtc_Write(void)   { STIM_WRITE_HANDLER(XM_STIM_SPI_RTC);   }
static void _OnStim_SpiLed_Write(void)   { STIM_WRITE_HANDLER(XM_STIM_SPI_LED);   }
static void _OnStim_UartGrf_Write(void)  { STIM_WRITE_HANDLER(XM_STIM_UART_GRF);  }
static void _OnStim_Mdio_Write(void)     { STIM_WRITE_HANDLER(XM_STIM_MDIO);      }
static void _OnStim_Rmii_Write(void)     { STIM_WRITE_HANDLER(XM_STIM_RMII);      }
