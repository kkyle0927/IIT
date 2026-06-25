/**
 ******************************************************************************
 * @file    xm_periph_stimulus.c
 * @author  HyundoKim
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "xm_periph_stimulus.h"
#include "system_startup.h"   /* System_GetSpi*Id / GetFdcan*Id / GetExternalUartId */
#include "ioif_agrb_spi.h"
#include "ioif_agrb_fdcan.h"  /* IOIF_FDCAN_Transmit — FDCAN1/2 dummy frame TX */
#include "ioif_agrb_eth_mdio.h" /* ioif_eth_mdio.read — MDIO/RMII PHY register access */
#include "ioif_agrb_uart.h"   /* IOIF_UART_Write_DMA — USART2 가비지 TX (BUSY-safe) */
#include "led_manager.h"      /* LedManager_SetChannelSuspend — LED write 토글 시 평시 status LED skip */
#include "pca9957.h"          /* PCA9957_UpdateAll — 24채널 PWM 평시 path 일괄 토글 */
#include "rtl8201f.h"         /* RTL8201F_PHY_ADDR — MDIO read target */
#include "cm_drv.h"           /* CM_Drv_IsConnected — FDCAN1 status indicator */
#include "imu_hub_drv.h"      /* ImuHub_Drv_IsConnected — FDCAN2 status indicator */
#include "emg_hub_drv.h"      /* EmgHub_Drv_IsConnected — FDCAN2 status indicator */
#include "fes_hub_drv.h"      /* FesHub_Drv_IsConnected — FDCAN2 status indicator */
#include "xm_api_usb.h"       /* XM_IsUsbStreamConnected — USB CDC status indicator */
#include "mdaf-25-6850.h"     /* MarvelDex_IsOnline — GRF L/R 연결 감지 (RX timeout) */

#include "cmsis_os2.h"        /* FreeRTOS task — 별도 stimulus 실행 컨텍스트 */
#include "FreeRTOS.h"
#include "task.h"             /* vTaskDelayUntil — 정확한 100ms 주기 */

#include <stdatomic.h>
#include <string.h>

/* ── Task 구성 ───────────────────────────────────────────────────────────
 *
 * 이전 (v1, 2026-05-13 오전) — XM_USB_ProcessPeriodic (UserTask 1kHz) 안에서
 * XM_Stimulus_Process() 호출. LED stimulus 가 1ms 당 PCA9957 24-register write
 * (≈ 840μs SPI duplex DMA + semaphore round-trip) 누적 → UserTask 1ms 한도
 * 초과 → USB CDC RX/TX cascade 지연 → HardFault → NVIC_SystemReset 무한리셋
 * 재현 (commit 5998f31 환경, 사용자 보고 2026-05-13 16:08).
 *
 * v2 (현재) — Stimulus 전용 별도 FreeRTOS task:
 *   - priority osPriorityBelowNormal: UserTask (Realtime6) 보다 낮음 → 항상
 *     preempt 되어 UserTask 의 1ms 주기 보장. stimulus 가 SPI 점유 중이면
 *     scheduler 가 빼앗아 UserTask 실행 후 복귀.
 *   - 주기 500ms (vTaskDelayUntil): UserTask 1ms 대비 1/500 호출 빈도 →
 *     SPI bus 와 USB CDC 사이 경합 최소. SI 측정 신호 (오실로 trigger 가능한
 *     2Hz frame rate) 캡처에 충분. 100ms→500ms 상향 (2026-05-14 HW팀 피드백):
 *     "안전·robust 우선, 더 느려도 무방". FDCAN auto-OFF 까지 시간도 4s → 20s
 *     로 늘어나 SM 미연결 환경 SI 측정 여유 확대.
 *   - stack 4KB: PCA9957 chain + ioif_spi DMA 도 깊지 않음. 여유 충분.
 *
 * tick 의미: 500ms 단위 counter. LED 1Hz blink 는 tick & 1u (500ms 마다 toggle).
 */
#define XM_STIM_TASK_PERIOD_MS    (500u)
#define XM_STIM_TASK_STACK_BYTES  (4096u)

/* ethernetif.c USER CODE BEGIN 4 영역에 정의 — IOIF MDIO 인스턴스 ID getter.
 * LwIP low_level_init 완료 후 valid, 그 전엔 IOIF_ETH_MDIO_NOT_ALLOCATED 반환. */
extern IOIF_ETH_MDIOx_t XM_Ethernetif_GetMdioId(void);

/* USART2 (External UART, Rev2.0 PD5/PD6) HAL handle — Core/Src/main.c 가 정의.
 * IOIF_UART_Transmit 우회 직접 HAL TX (poll, 1ms timeout) — stimulus 한 burst 만. */
extern UART_HandleTypeDef huart2;

/* FDCAN1/2 IOIF 인스턴스 ID getter — system_startup.c 가 정의. 헤더 prototype 미존재
 * 라 stimulus.c local extern 으로 노출. 평시 다른 모듈(예: canfd_rx_handler) 도 동일
 * extern 패턴 사용. */
extern IOIF_FDCANx_t System_GetFDCAN1_Id(void);
extern IOIF_FDCANx_t System_GetFDCAN2_Id(void);

/* FDCAN2 HAL handle — Restricted Operation Mode 전환용 (Core/Src/main.c 가 정의).
 * SM 미연결 환경에서 ACK 없이 무한 송신 가능하게 CCCR.ASM 비트만 토글 (filter/
 * notification 보존, 수십μs 내 mode 전환). */
extern FDCAN_HandleTypeDef hfdcan2;

/* GRF L/R UART instance getter — system_startup.c 가 정의. UART7 = GRF L (Left foot),
 * UART8 = GRF R (Right foot). 사용자 정정 2026-05-14: UART GRF stimulus 대상은
 * Xsens (USART2) 가 아니라 MarvelDex GRF 2개 (UART7/8). */
extern IOIF_UARTx_t System_GetGrfLUartId(void);
extern IOIF_UARTx_t System_GetGrfRUartId(void);

/* LwIP RX packet 카운터 — ethernetif.c USER CODE 4 에 정의. ethernetif_input
 * task 가 매 RX pbuf 처리 시 ++ (ARP/ICMP/IP 모두 포함). RMII/MDIO stimulus 가
 * polling 으로 "PC 와 LAN 연결 + ping 응답 중" 직관 표시. */
extern volatile uint32_t g_eth_rx_packet_count;

/* PSRAM memory-mapped region (STM32H7 QUADSPI).
 * 평시 ioif_psram.start_memory_mapped() 가 부팅 시 호출되어 QSPI 가 read-only
 * mapped 모드 영구 점유 상태. stimulus 는 동일 path (memcpy from mapped region)
 * 로 read 만 발생 → semaphore 충돌 없음, mode 전환 불필요.
 */
#define XM_PSRAM_MAPPED_BASE   (0x90000000U)
#define XM_PSRAM_STIM_READ_LEN (64u)

/* ── 카탈로그 정책 (2026-05-13 v3 재조정 — HW팀 피드백 반영) ─────────────
 *
 * 이전 v2 정책 ("FDCAN/UART/MDIO/RMII stimulus 불필요") 는 HW팀이 상대물
 * (CM/SM/Xsens) 없이 보드 단독 SI 측정 시 라인 신호가 안 보인다는 피드백으로
 * v3 에서 "선택 가능한 가비지 TX" 로 변경.
 *
 * 정책 원칙:
 *  1) 상대물 연결 상태에선 stimulus OFF — 평시 트래픽이 SI 측정 신호.
 *  2) 미연결 상태에선 stimulus ON — dummy/garbage TX 로 라인만 토글.
 *     (수신측 ACK 못 받아도 TX 라인 신호는 정상 발생.)
 *
 *  ✓ FDCAN CH1   : Reserved ID 0x7FE 로 8B dummy TX. CM 연결 시 OFF 권장
 *                  (CM PDO 와 ID 영역 분리되어 충돌 가능성은 낮음).
 *  ✓ FDCAN CH2   : 동일하게 ID 0x7FD 로 8B dummy TX. SM PnP/PDO 와 ID 분리.
 *  ✓ UART GRF    : USART2 (Rev2.0 PD5/PD6) 로 0x55/0xAA 2B 패턴 TX. Xsens MTi
 *                  명령 preamble (0xFA) 과 달라 명령 오해 위험 없음.
 *  ✓ SPI PSRAM   : QUADSPI memory-mapped 64B read — IO0~3 양방향 신호.
 *  ✓ SPI RTC     : MCP79510 SRREAD(0x05) 2B duplex — MOSI/MISO 모두 토글.
 *  ✓ SPI LED     : PCA9957 24채널 PWM write loop — MOSI 풍부, LED 시각 토글.
 *  ✓ MDIO        : RTL8201F PHY ID register read loop — MDC/MDIO 신호 발생.
 *  ✓ RMII        : PHY BMSR(0x01) read loop. 진짜 REF_CLK/TX_EN/TXD 측정은
 *                  PHY link up 상태 평시 LwIP 트래픽 (ARP/DHCP/ping) 으로 측정.
 *                  Stimulus 는 MAC↔PHY MDC/MDIO + ETH CSR access 만 자극.
 *
 * SPI Read 와 MOSI 신호 (HW팀 자주 묻는 질문):
 *  SPI 는 본질 full-duplex 라 Read 도 MOSI 에 cmd/dummy 바이트 출력. RTC SRREAD
 *  은 MOSI 에 0x05+0x00, PSRAM memory-mapped read 는 QUADSPI IO0~3 에 cmd/addr/
 *  dummy 자동 출력. PSRAM 별도 write task 분리 불필요.
 */

/* ── Step function 시그니처 ──────────────────────────────────────────────
 * tick: 활성화 이후 증가하는 카운터 (1ms 단위, atomic 보호 X — 단일 호출자).
 *       payload 에 실어 RX 측에서 정상 순차 수신 검증 가능.
 */
typedef void (*StepFn_t)(uint32_t tick);

typedef struct {
    atomic_bool  enabled;
    StepFn_t     step;
    uint32_t     counter;
    const char*  name;
} StimulusSlot_t;

static StimulusSlot_t s_slots[XM_STIM_COUNT_];

/* 진단 카운터 — 1ms tick context 에서 단일 writer, sensor-studio polling 단일
 * reader. uint32 단일-word write 는 ARM Cortex-M7 에서 atomic, race 없음.
 * extern 노출 (xm_periph_stimulus.h) — xm_production_od.c 가 OD 0x7E10 으로 매핑.
 */
volatile uint8_t  g_stim_psram_alloc_failed = 0u;
volatile uint32_t g_stim_err_count[XM_STIM_COUNT_] = {0};
volatile uint32_t g_stim_run_count[XM_STIM_COUNT_] = {0};
volatile uint8_t  g_xm_status_connected[XM_STIM_COUNT_] = {0};
volatile uint32_t g_stim_last_data[XM_STIM_COUNT_] = {0};

/* FDCAN Ch2 stimulus enable 거부 사유 — 마지막 SetEnabled(FDCAN2, true) 시점 기록.
 *   0 = 거부 없음 (정상 진입 또는 시도 없음)
 *   1 = SM 연결 중 (ImuHub/EmgHub/FesHub IsConnected 어느 것이라도 true)
 *   2 = TEC 누적 (>= XM_STIM_FDCAN_TEC_AUTODISABLE)
 *   3 = Restricted Mode 진입 실패 (HAL_FDCAN_Stop/Start 실패)
 * sensor-studio polling 으로 OD 0x7E40:02 read → "starting → OFF" 시 즉시 원인 표시. */
volatile uint8_t  g_stim_fdcan2_reject_reason = 0u;

/* ── Stimulus step 구현 ────────────────────────────────────────────── */

/* FDCAN Reserved ID — CM/SM 의 정상 PDO/SDO ID 영역과 분리.
 * CANopen 표준 ID 할당: 0x000(NMT)~0x7E0(SDO Tx) 까지 의미 부여, 0x7E1~0x7FF
 * 영역은 reserved. stimulus 가 이 영역만 사용해 평시 트래픽과 충돌 회피.
 */
#define XM_STIM_FDCAN_CH1_ID  (0x7FEu)
#define XM_STIM_FDCAN_CH2_ID  (0x7FDu)

/* CAN TEC 임계 — Error Warning(96) 진입 전 빠른 자동 보호.
 * 사용자 보고 2026-05-14: CH2 ON 후 ~128초 만에 보드 무한리셋. 96 임계가
 * 작동했지만 그 이전에 다른 fault (Bus-Off 자동 복구 race / IOIF SW Queue
 * 누적 등) 가 우선 발생한 것으로 추정. 임계 낮춤 + bus_status flag 추가
 * 감시로 빠른 OFF 보호.
 *
 * 32 = TEC 가 1~8/frame 단위 증가 시 1~4초 안에 도달. 평시 ACK 환경 (CM/SM
 * 연결) 에선 TEC 0~8 사이 유지 → stimulus 사용 무방. */
#define XM_STIM_FDCAN_TEC_AUTODISABLE  (32u)

static void _StepFdcan1(uint32_t tick)
{
    /* FDCAN CH1 stimulus — Reserved ID 8B dummy TX.
     *
     * 평시 CM 연결 시 SYNC_Ch1 1ms + PDO/SDO 가 활발 → stimulus OFF 권장.
     * CM 미연결 상태에서 stimulus ON 시 ACK 못 받아 TEC 증가하지만 TX 라인
     * (CAN_TX, FDCAN_TX) 신호는 정상 발생 → SI 측정 가능.
     *
     * tick 을 payload 에 실어 logic analyzer 에서 순차성 검증 가능.
     */
    IOIF_FDCANx_t id = System_GetFDCAN1_Id();
    if (id == IOIF_FDCAN_INVALID_ID) return;

    /* 보호 1: bus_status 의 어떤 flag (BusOff/Warning/ErrorPassive) 라도 set 시
     * 즉시 auto-OFF. HAL 자동 Bus-Off 복구 race 회피. */
    uint8_t lec = 0, bus_status = 0;
    if (IOIF_FDCAN_GetBusStatus(id, &lec, &bus_status) == AGRBStatus_OK &&
        bus_status != 0u) {
        atomic_store(&s_slots[XM_STIM_FDCAN1].enabled, false);
        g_stim_last_data[XM_STIM_FDCAN1] = ((uint32_t)bus_status << 24) | 0xBA5Eu;
        return;
    }

    /* 보호 2: TEC 임계 (32) 초과 시 auto-OFF — Bus-Off 진입 전 빠른 보호 */
    uint8_t tec = 0, rec = 0;
    if (IOIF_FDCAN_GetErrorCounters(id, &tec, &rec) == AGRBStatus_OK &&
        tec >= XM_STIM_FDCAN_TEC_AUTODISABLE) {
        atomic_store(&s_slots[XM_STIM_FDCAN1].enabled, false);
        g_stim_last_data[XM_STIM_FDCAN1] = ((uint32_t)tec << 16) | 0xBA5Eu;
        return;
    }

    uint8_t data[8] = {
        0xAAu, 0x55u, 0xAAu, 0x55u,
        (uint8_t)(tick),
        (uint8_t)(tick >> 8),
        (uint8_t)(tick >> 16),
        (uint8_t)(tick >> 24),
    };
    if (IOIF_FDCAN_Transmit(id, XM_STIM_FDCAN_CH1_ID, data, sizeof(data)) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_FDCAN1]++;
    }
    /* 상위 16b = TEC (외부 노드 미연결 환경 진단), 하위 16b = tick LSB. */
    g_stim_last_data[XM_STIM_FDCAN1] = ((uint32_t)tec << 16) | (tick & 0xFFFFu);
}

static void _StepFdcan2(uint32_t tick)
{
    /* FDCAN CH2 stimulus — 동일 정책, Reserved ID 0x7FD.
     * 평시 SM (IMU/EMG/FES) 미연결 상태에서 라인 자극 용도.
     * SetEnabled 진입 시 Restricted Operation Mode (CCCR.ASM=1) 로 전환 → TEC
     * 누적 자체가 hardware 에서 차단. step 보호 로직은 mode 전환 race / 다른
     * fault 안전망 역할. */
    IOIF_FDCANx_t id = System_GetFDCAN2_Id();
    if (id == IOIF_FDCAN_INVALID_ID) return;

    /* Restricted Mode 활성 여부 — CCCR.ASM=1 이면 TEC 누적 자체가 hardware 차원
     * 차단. bus_status (EP/EW) 도 다른 노드의 error frame 으로 set 될 수 있으나
     * BusOff 진입 안 함 → stimulus 동작에 영향 없음. 따라서 두 보호 로직 모두
     * Restricted 일 땐 skip. Normal Mode (mode 전환 race) 일 때만 안전망 발동. */
    bool is_restricted = (hfdcan2.Instance->CCCR & FDCAN_CCCR_ASM) != 0u;

    uint8_t lec = 0, bus_status = 0;
    uint8_t tec = 0, rec = 0;
    /* TEC read — 표시용 (last_data 에 인코딩) */
    (void)IOIF_FDCAN_GetErrorCounters(id, &tec, &rec);

    if (!is_restricted) {
        /* 보호 1: bus_status flag (BusOff/Warning/ErrorPassive) — Normal 한정 */
        if (IOIF_FDCAN_GetBusStatus(id, &lec, &bus_status) == AGRBStatus_OK &&
            bus_status != 0u) {
            atomic_store(&s_slots[XM_STIM_FDCAN2].enabled, false);
            g_stim_last_data[XM_STIM_FDCAN2] = ((uint32_t)bus_status << 24) | 0xBA5Eu;
            return;
        }
        /* 보호 2: TEC 임계 (32) 초과 — Normal 한정 (Restricted 에선 누적 X) */
        if (tec >= XM_STIM_FDCAN_TEC_AUTODISABLE) {
            atomic_store(&s_slots[XM_STIM_FDCAN2].enabled, false);
            g_stim_last_data[XM_STIM_FDCAN2] = ((uint32_t)tec << 16) | 0xBA5Eu;
            return;
        }
    }

    uint8_t data[8] = {
        0x55u, 0xAAu, 0x55u, 0xAAu,
        (uint8_t)(tick),
        (uint8_t)(tick >> 8),
        (uint8_t)(tick >> 16),
        (uint8_t)(tick >> 24),
    };
    if (IOIF_FDCAN_Transmit(id, XM_STIM_FDCAN_CH2_ID, data, sizeof(data)) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_FDCAN2]++;
    }
    g_stim_last_data[XM_STIM_FDCAN2] = ((uint32_t)tec << 16) | (tick & 0xFFFFu);
}

static void _StepUartGrf(uint32_t tick)
{
    /* GRF L (UART7) + GRF R (UART8) 양쪽 stimulus.
     *
     * 정책 (2026-05-14 v3 HW팀 피드백):
     *   - GRF 모듈 미연결: 0x55/0xAA TX 가비지 송신 → TX 라인 SI 측정용.
     *   - GRF 모듈 연결됨: TX 송신 skip (평시 데이터 흐름 보호) + online 상태만
     *     마지막 데이터에 인코딩. GRF 모듈 자체가 1kHz 로 데이터 송신 → 보드는
     *     이미 RX 처리 중 (MarvelDex driver) → 오실로 RX 라인 측정으로 충분.
     *
     * MarvelDex_IsOnline(ch): UART RX 타임아웃 기반 (500ms 무수신 → STOPPED).
     *   ch 0 = Left foot, ch 1 = Right foot.
     *
     * last_data 인코딩 (GUI _fmt_uart 디코더와 1:1 매칭):
     *   [31:24] = 0xC0 marker (GRF online 표시) | tx_ok flags (L bit0, R bit1)
     *   [23:16] = online flags (L bit0, R bit1)
     *   [15:0]  = tick LSB
     *
     * marker 0xC0 (Connected) vs 0x00 (None) 로 GUI 가 mode 판정 후 분기 표시.
     */
    bool online_l = MarvelDex_IsOnline(MARVELDEX_CH_LEFT);
    bool online_r = MarvelDex_IsOnline(MARVELDEX_CH_RIGHT);
    uint32_t online_flags = (online_l ? 0x1u : 0u) | (online_r ? 0x2u : 0u);

    IOIF_UARTx_t id_l = System_GetGrfLUartId();
    IOIF_UARTx_t id_r = System_GetGrfRUartId();
    static const uint8_t pattern[2] = { 0x55u, 0xAAu };

    bool ok_l = false, ok_r = false;
    /* 미연결 측에만 TX 가비지 송신 — 연결된 측은 평시 RX 데이터 흐름 보호 위해 skip */
    if (!online_l && id_l != IOIF_UART_ID_NOT_ALLOCATED) {
        ok_l = (IOIF_UART_Write_DMA(id_l, pattern, sizeof(pattern)) == AGRBStatus_OK);
    }
    if (!online_r && id_r != IOIF_UART_ID_NOT_ALLOCATED) {
        ok_r = (IOIF_UART_Write_DMA(id_r, pattern, sizeof(pattern)) == AGRBStatus_OK);
    }
    /* err_count 는 미연결 측 TX 실패 시에만 누적 (online 측은 의도적 skip). */
    if ((!online_l && !ok_l) || (!online_r && !ok_r)) {
        g_stim_err_count[XM_STIM_UART_GRF]++;
    }

    uint32_t tx_flags = (ok_l ? 0x1u : 0u) | (ok_r ? 0x2u : 0u);
    /* marker 0xC0 = "online byte present", 평시 0x00. GUI 가 byte24 = 0xC0 보면
     * "GRF 연결됨 L/R" 표시, 0x00 이면 TX 가비지 모드로 디코딩. */
    uint32_t marker = (online_flags != 0u) ? 0xC0u : 0x00u;
    g_stim_last_data[XM_STIM_UART_GRF] =
        ((marker | tx_flags) << 24) |
        (online_flags << 16) |
        (tick & 0xFFFFu);
}

static void _StepMdio(uint32_t tick)
{
    /* MDIO stimulus — RTL8201F PHY ID register (0x02 PHYIDR1) read loop.
     *
     * ethernetif.c USER CODE 4 의 XM_Ethernetif_GetMdioId() 로 평시 IOIF MDIO
     * 인스턴스 공유. LwIP low_level_init 완료 전 (boot 초기) 에는 NOT_ALLOCATED
     * 반환 → skip. read 자체는 PHY 동작 무관 (status register read 만).
     *
     * MDC clock + MDIO data line 신호 모두 SI 측정 가능. 1ms 마다 1 transaction
     * = 16+16 bit clock cycle (≈ 약 12μs @ 2.5MHz MDC).
     */
    IOIF_ETH_MDIOx_t id = XM_Ethernetif_GetMdioId();
    if (id == IOIF_ETH_MDIO_NOT_ALLOCATED) return;
    uint32_t val = 0u;  /* IOIF MDIO API value 가 uint32_t* — HAL 호환 */
    if (ioif_eth_mdio.read(id, RTL8201F_PHY_ADDR, 0x02u, &val) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_MDIO]++;
    }
    /* last_data 인코딩 (GUI _fmt_mdio 디코더와 1:1 매칭):
     *   [31:16] = ETH RX packet count LSB (PC ping 으로 평시 트래픽 활동 확인)
     *   [15:0]  = PHYIDR1 값 (RTL8201F 정상 = 0x001C)
     *
     * MDIO read 가 성공하면 PHY 자체는 응답 — 그 위에 PC LAN 연결 + ping 활동을
     * RX 카운터 증가 추세로 확인. HW팀이 "MDIO 라인 측정" + "ICMP 응답 확인"
     * 동시 검증 가능. */
    g_stim_last_data[XM_STIM_MDIO] =
        ((g_eth_rx_packet_count & 0xFFFFu) << 16) | (val & 0xFFFFu);
    (void)tick;
}

static void _StepRmii(uint32_t tick)
{
    /* RMII stimulus — PHY BMSR(0x01, Basic Mode Status Register) read loop.
     *
     * 한계: RMII line 의 진짜 신호 (REF_CLK 50MHz, TX_EN, TXD0/1, CRS_DV,
     * RXD0/1) 는 PHY-MAC 사이라 stimulus 가 MAC TX 발생시켜야 함.
     * HAL_ETH_Transmit 은 LwIP TX descriptor 와 충돌 위험 (1ms task 에서 호출
     * 시 race). 따라서 stimulus 는 MDC/MDIO 만 자극.
     *
     * 진짜 RMII TX_EN/TXD 측정 권장 방법:
     *   1) PHY link up 후 PC 가 보드 IP 로 ping → 평시 LwIP ICMP reply 가 RMII
     *      TX 라인 토글.
     *   2) DHCP client 활성 시 boot 직후 DHCP DISCOVER broadcast 송신 → RMII TX.
     *   3) (후속) PHY internal loopback 모드 stimulus — 별도 옵션 plan.
     *
     * BMSR(0x01) read 는 PHY 동작 무관 (link status 만 보고) → 안전.
     */
    IOIF_ETH_MDIOx_t id = XM_Ethernetif_GetMdioId();
    if (id == IOIF_ETH_MDIO_NOT_ALLOCATED) return;
    uint32_t val = 0u;  /* IOIF MDIO API value 가 uint32_t* — HAL 호환 */
    if (ioif_eth_mdio.read(id, RTL8201F_PHY_ADDR, 0x01u, &val) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_RMII]++;
    }
    /* BMSR (Basic Mode Status Register) — bit2=link up, bit11=Auto-Neg complete.
     * 정상 RTL8201F link down 시 약 0x7849, 100M FD link up 시 0x782D 근방.
     *
     * [2026-05-14] PC LAN ping/traffic 활동 인코딩 — ethernetif.c 의 RX packet
     * 카운터 (low_level_input 에서 ++). GUI 가 polling 간 차이로 "활동 중" 검증.
     *   [31:16] = g_eth_rx_packet_count LSB 16b
     *   [15:0]  = BMSR
     */
    uint32_t pkt_low16 = g_eth_rx_packet_count & 0xFFFFu;
    g_stim_last_data[XM_STIM_RMII] = (pkt_low16 << 16) | (val & 0xFFFFu);
    (void)tick;
}

/* QUADSPI PSRAM stimulus 결과를 받는 local sink — DTCM/D1 RAM 임의 위치.
 * memcpy destination 이라 컴파일러가 dead-store 로 제거하지 않도록 volatile.
 */
static volatile uint8_t s_psram_stim_sink[XM_PSRAM_STIM_READ_LEN];

static void _StepSpiPsram(uint32_t tick)
{
    /* QUADSPI PSRAM (APS6404L) stimulus — memory-mapped 영역 64B read.
     *
     * 평시 부팅 시 ioif_psram.start_memory_mapped() 가 QSPI 를 mapped read
     * 모드로 영구 점유. 그 상태에서 alloc/write 호출은 semaphore deadlock.
     * stimulus 는 mapped region (0x90000000~) 에서 직접 memcpy → 평시 path
     * 와 동일, mode 전환/semaphore 불필요.
     *
     * tick 하위 12-bit (mod 4KB) 를 offset 으로 사용 → 4KB 영역 rotation read.
     * QUADSPI controller 가 1 byte 별 read 명령 자동 발행 → SCK / IO0~3 / NCS
     * 라인 신호 발생. memcpy 64B 가 적당한 SI 관측 burst (수십 μs).
     */
    const uint8_t* psram_base = (const uint8_t*)XM_PSRAM_MAPPED_BASE;
    uint32_t offset = (tick * XM_PSRAM_STIM_READ_LEN) & 0xFFFu;  /* 4KB rotation */
    memcpy((void*)s_psram_stim_sink, psram_base + offset, XM_PSRAM_STIM_READ_LEN);
    /* memcpy 자체는 실패 path 없음 — read fault 발생 시 BusFault HardFault.
     * err_count 누적은 QSPI controller 가 hang 같은 묵시적 실패 만에 의미.
     * 현 시점에서는 fault dump 인프라가 별도 처리 → err_count 미사용.
     */
    /* 첫 4B read 결과 — PSRAM 응답값. 비초기화 영역이면 0xFF 또는 random byte.
     * 값이 매번 같아도 tick 별 offset 변화로 GUI 값이 변함. */
    g_stim_last_data[XM_STIM_SPI_PSRAM] =
        ((uint32_t)s_psram_stim_sink[0]      ) |
        ((uint32_t)s_psram_stim_sink[1] <<  8) |
        ((uint32_t)s_psram_stim_sink[2] << 16) |
        ((uint32_t)s_psram_stim_sink[3] << 24);
}

static void _StepSpiRtc(uint32_t tick)
{
    /* SPI2 (MCP79510 RTC) stimulus — SRREAD (Status Register Read) 2B duplex.
     * cmd 0x05 (MCP79510_CMD_SRREAD) + 1B dummy → Status Register 1B 반환.
     * Timekeeping register 와 무관 → RTC 시간 변경 0%. Status register read
     * 도 부작용 없음. NSS+SCK+MOSI+MISO 모두 SI 측정 가능.
     */
    IOIF_SPIx_t id = System_GetSpi2RtcId();
    if (id == IOIF_SPI_ID_NOT_ALLOCATED) return;

    uint8_t tx[2] = { 0x05u /* MCP79510_CMD_SRREAD */, 0x00u };
    uint8_t rx[2] = { 0 };
    (void)tick;
    if (ioif_spi.duplex(id, tx, rx, sizeof(tx)) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_SPI_RTC]++;
    }
    /* SR (Status Register) 값 — RTC 정상이면 일정 값 유지, 통신 실패 시 0x00. */
    g_stim_last_data[XM_STIM_SPI_RTC] = (uint32_t)rx[1];
}

static void _StepSpiLed(uint32_t tick)
{
    /* SPI5 (PCA9957 LED Driver) stimulus — PCA9957_UpdateAll() 평시 path 활용.
     *
     * 동작: tick 단위 500ms 기준, (tick & 1) → 500ms ON / 500ms OFF = 1Hz blink.
     *       24채널 모두 동일 패턴.
     *
     * 부담 분석 (v2 task 분리 후, 500ms 주기):
     *   - 500ms 마다 PCA9957_UpdateAll (24 × SPI duplex DMA ≈ 840μs).
     *     500ms 대비 0.17% 점유 (이전 100ms 대비 5× 여유).
     *   - Stimulus Task priority BelowNormal → UserTask (Realtime6) 가 1ms 마다
     *     preempt 가능. SPI mutex 잡힌 채로 preempt 되어도 UserTask 는 SPI 안 씀
     *     (FDCAN/USB/GPIO 만 사용) → 영향 없음.
     *
     * LedManager_SetChannelSuspend(true) 가 평시 channel LED 갱신 차단 →
     * stimulus 가 PCA9957 register 독점. SetEnabled 가 호출.
     */
    bool on_phase = (tick & 1u) != 0u;
    uint8_t pwm = on_phase ? 0xFFu : 0x00u;

    uint8_t buf[24];
    memset(buf, pwm, sizeof(buf));
    if (PCA9957_UpdateAll(buf) != AGRBStatus_OK) {
        g_stim_err_count[XM_STIM_SPI_LED]++;
    }
    /* 현재 PWM 값 — GUI 에 0xFF (ON phase) / 0x00 (OFF phase) 토글 시각화 */
    g_stim_last_data[XM_STIM_SPI_LED] = (uint32_t)pwm;
}

/* 미구현 stimulus (MDIO / RMII) 는 step 슬롯에 NULL → SetEnabled 가 false 반환. */

/* ── Task wrapper ─────────────────────────────────────────────────────── */

static osThreadId_t s_stim_task_handle = NULL;

/* 별도 stimulus task 본체 — 100ms 주기로 enabled stimulus step 일괄 실행 +
 * 평시 connected 상태 mirror. UserTask 의 1ms 주기와 완전 분리 → SPI/USB
 * cascade 없음. priority BelowNormal 이므로 UserTask 가 항상 우선.
 *
 * 시작 시 자가 vTaskSuspend(NULL) — sensor-studio 미연결 상태에선 dorman.
 * USB CDC connect rising edge 에서 XM_Stimulus_OnUsbConnect() 가 vTaskResume.
 * disconnect 는 task 자가 polling (100ms latency) → stimulus 자동 OFF +
 * 자가 vTaskSuspend(NULL) → 다음 connect 까지 dorman.
 */
static void _StimulusTask(void* argument)
{
    (void)argument;

    /* 시작 시 dorman — sensor-studio 가 연결되어 OnUsbConnect hook 호출 시 resume */
    vTaskSuspend(NULL);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(XM_STIM_TASK_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if (!XM_IsUsbStreamConnected()) {
            /* sensor-studio 미연결 → 모든 stimulus OFF (LedManager_SetChannelSuspend
             * 평시 복원 등 자동) 후 자가 dorman. */
            for (int i = 1; i < XM_STIM_COUNT_; i++) {
                if (atomic_load(&s_slots[i].enabled)) {
                    XM_Stimulus_SetEnabled((XM_Stimulus_e)i, false);
                }
            }
            vTaskSuspend(NULL);
            /* resume 후 timing 리셋 → 첫 vTaskDelayUntil 이 정확히 100ms 주기 시작 */
            xLastWakeTime = xTaskGetTickCount();
            continue;
        }

        XM_Stimulus_Process();
    }
}

void XM_Stimulus_OnUsbConnect(void)
{
    /* USB CDC connect rising edge — xm_api_usb.c 의 ProcessPeriodic hook 에서 호출.
     * task handle 이 아직 할당 안 됐거나 (boot 초기) 이미 실행 중이면 무시 (idempotent).
     */
    if (s_stim_task_handle != NULL) {
        vTaskResume(s_stim_task_handle);
    }
}

/* ── Public API ───────────────────────────────────────────────────────── */

void XM_Stimulus_Init(void)
{
    for (int i = 0; i < XM_STIM_COUNT_; i++) {
        atomic_store(&s_slots[i].enabled, false);
        s_slots[i].step    = NULL;
        s_slots[i].counter = 0u;
        s_slots[i].name    = "?";
    }

    /* v3 카탈로그 (2026-05-13 HW팀 피드백 반영) — 8개 stimulus 전 row 활성화.
     * 평시 상대물 연결 시엔 OFF 권장 (bus 경쟁 회피), 미연결 SI 측정 시 ON. */
    s_slots[XM_STIM_FDCAN1].step    = _StepFdcan1;
    s_slots[XM_STIM_FDCAN1].name    = "fdcan1";   /* Reserved ID 0x7FE 8B TX */

    s_slots[XM_STIM_FDCAN2].step    = _StepFdcan2;
    s_slots[XM_STIM_FDCAN2].name    = "fdcan2";   /* Reserved ID 0x7FD 8B TX */

    s_slots[XM_STIM_SPI_PSRAM].step = _StepSpiPsram;
    s_slots[XM_STIM_SPI_PSRAM].name = "spi_psram";

    s_slots[XM_STIM_SPI_RTC].step   = _StepSpiRtc;
    s_slots[XM_STIM_SPI_RTC].name   = "spi_rtc";

    s_slots[XM_STIM_SPI_LED].step   = _StepSpiLed;
    s_slots[XM_STIM_SPI_LED].name   = "spi_led";

    s_slots[XM_STIM_UART_GRF].step  = _StepUartGrf;
    s_slots[XM_STIM_UART_GRF].name  = "uart_grf"; /* USART2 0x55/0xAA 2B TX */

    s_slots[XM_STIM_MDIO].step      = _StepMdio;
    s_slots[XM_STIM_MDIO].name      = "mdio";     /* PHYIDR1(0x02) read loop */

    s_slots[XM_STIM_RMII].step      = _StepRmii;
    s_slots[XM_STIM_RMII].name      = "rmii";     /* BMSR(0x01) read loop */

    /* 별도 stimulus task 생성 — UserTask (1kHz) 와 분리, 100ms 주기.
     * priority BelowNormal: UserTask Realtime6 보다 낮음 → SPI/PCA9957 처리
     * 중에도 1ms UserTask 가 preempt 으로 즉시 깨어남 → USB CDC cascade 차단.
     */
    static const osThreadAttr_t s_stim_task_attr = {
        .name       = "StimulusTask",
        .stack_size = XM_STIM_TASK_STACK_BYTES,
        .priority   = osPriorityBelowNormal,
    };
    s_stim_task_handle = osThreadNew(_StimulusTask, NULL, &s_stim_task_attr);
}

void XM_Stimulus_Process(void)
{
    for (int i = 1; i < XM_STIM_COUNT_; i++) {
        StimulusSlot_t* slot = &s_slots[i];
        if (atomic_load(&slot->enabled) && slot->step != NULL) {
            slot->step(slot->counter++);
            g_stim_run_count[i]++;  /* sensor-studio polling 으로 "실행 중" 검증 */
        }
    }
    /* connected status mirror 는 XM_Stimulus_UpdateConnectedStatus() 로 분리.
     * stimulus task 가 USB 미연결 시 자가 suspend 라서 status update 도 멈추는
     * 모순 회피 — UserTask (XM_USB_ProcessPeriodic, 1ms) 가 직접 호출. */
}

void XM_Stimulus_UpdateConnectedStatus(void)
{
    /* USB / FDCAN / UART 평시 트래픽 connected 상태 mirror — sensor-studio
     * status row indicator. UserTask 1ms 주기에서 호출되어 항상 최신 (stimulus
     * task suspend 와 무관).
     *
     * NONE / 0 = USB CDC (DTR active),
     * 1 = FDCAN1 CM (boolean),
     * 2 = FDCAN2 SM (bitmap: bit0=IMU, bit1=EMG, bit2=FES) — non-zero = connected
     *     2026-05-14 v2: 어떤 SM 인지 GUI 에 표시하기 위해 bitmap 으로 확장.
     *     boolean 호환 유지 (any bit set = connected).
     * 6 = UART GRF (Xsens 미구현 시 0 유지).
     */
    g_xm_status_connected[XM_STIM_NONE]    = XM_IsUsbStreamConnected() ? 1u : 0u;
    g_xm_status_connected[XM_STIM_FDCAN1]  = CM_Drv_IsConnected() ? 1u : 0u;
    {
        uint8_t sm_bitmap = 0u;
        if (ImuHub_Drv_IsConnected()) sm_bitmap |= 0x01u;
        if (EmgHub_Drv_IsConnected()) sm_bitmap |= 0x02u;
        if (FesHub_Drv_IsConnected()) sm_bitmap |= 0x04u;
        g_xm_status_connected[XM_STIM_FDCAN2] = sm_bitmap;
    }
    /* UART GRF (Xsens MTi) attach 상태는 별도 API 미존재 — 후속 plan. */
}

/* FDCAN2 Restricted Operation Mode 전환 (CCCR.ASM 비트만 토글).
 *
 * STM32H7 FDCAN Restricted Operation Mode:
 *   - 외부 라인 (CAN_TX, CAN_RX) 정상 신호 → 오실로 SI 측정 가능
 *   - ACK 미수신해도 TEC 누적 안 함 → SM 미연결 환경 무한 송신 안전
 *   - RX 측은 정상 수신 + 자동 ACK 가능 (peer 가 전송 시)
 *   - Bus-Off 진입 없음 → 무한리셋 회피
 *
 * 시퀀스 (IOIF_FDCAN_SetRestrictedMode 내부 캡슐화 — Stop/Start 사이 CCCR.ASM 만 변경):
 *   1. HAL_FDCAN_Stop → CCCR.INIT=1 (INIT mode 진입)
 *   2. CCCR.CCE=1 (Configure Change Enable)
 *   3. CCCR.ASM = restricted ? 1 : 0
 *   4. CCCR.CCE=0
 *   5. HAL_FDCAN_Start → CCCR.INIT=0 (정상 동작 재개)
 *
 * filter/notification config 는 그대로 보존 — HAL_FDCAN_Init 재호출 안 함.
 * 전환 시간 ~수십μs (HAL Stop/Start poll 기반).
 *
 * 주의: 진입 시점 TEC 가 누적된 상태면 Restricted Mode 진입 후에도 stimulus
 *       step 의 보호 로직(`tec >= 32`)이 즉시 trip. enable 진입 전 TEC 사전
 *       체크 필수 (XM_Stimulus_SetEnabled 에서 가드).
 */
static AGRBStatusDef _SetFdcan2RestrictedMode(bool restricted)
{
    /* Stop→CCE→CCCR.ASM 토글→Start 시퀀스를 IOIF 로 캡슐화 (HAL/레지스터 직접조작 제거).
     * 진입 가드(SM 미연결 / TEC 누적)는 호출부 XM_Stimulus_SetEnabled 가 유지. */
    return IOIF_FDCAN_SetRestrictedMode(System_GetFDCAN2_Id(), restricted);
}

bool XM_Stimulus_SetEnabled(XM_Stimulus_e id, bool enabled)
{
    if ((int)id <= (int)XM_STIM_NONE || (int)id >= (int)XM_STIM_COUNT_) return false;
    if (s_slots[id].step == NULL) return false;  /* 미구현 stimulus 거부 */

    /* FDCAN2 stimulus 전용 전처리 — Restricted Operation Mode 진입 (enable) 또는
     * Normal Mode 복원 (disable). enable 진입 시 SM 연결 / TEC 누적 가드. */
    if (id == XM_STIM_FDCAN2) {
        if (enabled) {
            /* SM 연결 시 stimulus 거부 — Restricted Mode 가 평시 PDO retransmit
             * 차단해 SM 통신 fragility 유발. SM 미연결 SI 측정 전용 사용. */
            if (ImuHub_Drv_IsConnected() || EmgHub_Drv_IsConnected() ||
                FesHub_Drv_IsConnected()) {
                g_stim_fdcan2_reject_reason = 1u;  /* SM 연결 거부 */
                return false;
            }
            /* TEC pre-check 제거 (2026-05-14 사용자 피드백):
             * Restricted Operation Mode 진입 후엔 TEC 누적 자체가 hardware 차원에서
             * 차단됨 (STM32H7 ref manual CCCR.ASM=1 사양). 진입 시점의 누적 TEC 가
             * 임계 이상이라도 Restricted 진입 후엔 더 증가 안 함 + step 보호 로직도
             * Restricted 일 땐 skip (아래 _StepFdcan2 가드).
             * → "TEC 누적 — 보드 재부팅 필요" false alarm 제거. */
            /* Restricted Operation Mode 진입 */
            if (_SetFdcan2RestrictedMode(true) != AGRBStatus_OK) {
                g_stim_fdcan2_reject_reason = 3u;  /* HAL Stop/Start 실패 */
                return false;
            }
            /* 모든 가드 통과 — 정상 진입 */
            g_stim_fdcan2_reject_reason = 0u;
        } else {
            /* disable — Normal Mode 복원. 실패해도 stimulus 자체는 stop 되므로
             * return true 유지 (mode 가 Restricted 로 stuck 되면 다음 enable 또는
             * 평시 SM 통신 시 사용자 인지 가능). */
            (void)_SetFdcan2RestrictedMode(false);
            /* OFF 호출은 reject_reason clear — UI 상 stale 표시 방지 */
            g_stim_fdcan2_reject_reason = 0u;
        }
    }

    if (enabled) {
        s_slots[id].counter = 0u;  /* 시작 시 카운터 리셋 */
    }
    atomic_store(&s_slots[id].enabled, enabled);

    /* SPI LED stimulus 는 LEDOUT0~5 register 를 매 ms 덮어쓰므로 평시
     * led_manager 의 PCA9957 갱신과 충돌. enable 동안 평시 channel LED
     * 표시 (EMG/IMU/GRF/USB status) 를 일시 중단하고 stimulus 종료 시
     * 자동 복원한다.
     */
    if (id == XM_STIM_SPI_LED) {
        LedManager_SetChannelSuspend(enabled);
    }

    /* RMII PHY internal loopback 진입 코드는 LwIP/HAL_ETH state machine 과 race
     * (사용자 보고 2026-05-14: RMII ON 후 ~3초 만에 보드 reset). BMCR write 가
     * link state change 트리거 → ethernetif link polling 또는 HAL_ETH reinit
     * 과 MDIO bus race → HardFault/NVIC_SystemReset.
     *
     * 현재는 BMCR loopback 진입 비활성. RMII row 의 step 함수는 BMSR read 만
     * 수행 (link 상태 폴링) — MDIO row 와 동일 backend. 진짜 RMII TX_EN/TXD
     * 측정은 PHY link up 상태 평시 LwIP ARP/ping 트래픽 또는 외부 packet 생성
     * 으로 진행. PHY loopback 옵션은 ETH stack 분리/suspend 인프라 추가 후
     * 별도 plan ([[project_phase2_mdio_rmii_deferred]] 참고).
     */
    return true;
}

bool XM_Stimulus_IsEnabled(XM_Stimulus_e id)
{
    if ((int)id <= (int)XM_STIM_NONE || (int)id >= (int)XM_STIM_COUNT_) return false;
    return atomic_load(&s_slots[id].enabled);
}

const char* XM_Stimulus_GetName(XM_Stimulus_e id)
{
    if ((int)id <= (int)XM_STIM_NONE || (int)id >= (int)XM_STIM_COUNT_) return "?";
    return s_slots[id].name;
}

void XM_Stimulus_GetDiag(XM_Stimulus_Diag_t* out)
{
    if (out == NULL) return;
    out->psram_alloc_failed = g_stim_psram_alloc_failed;
    for (int i = 0; i < XM_STIM_COUNT_; i++) {
        out->err_count[i] = g_stim_err_count[i];
    }
}
