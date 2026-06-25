/**
 ******************************************************************************
 * @file    system_startup.c
 * @author  HyundoKim
 * @brief   [System/Core] IMU Hub 시스템 초기화 및 부팅 시퀀스 관리
 * @version 3.0 (아키텍처 리팩토링 완료)
 * @date    Dec 3, 2025
 *
 * @details
 * IMU Hub의 전체 초기화 순서를 관리하고, 센서 리셋 및 통신 시작을 제어합니다.
 * 
 * [부팅 시퀀스]
 * 1. IOIF 계층 초기화 (FDCAN, UART, TIM, GPIO)
 * 2. 센서 완전 리셋 시퀀스 (Power Cycle + Pin Discharge)
 *    - 센서 전원 OFF
 *    - _ForceDischargeUartPins() - UART 핀 방전 (제품별 핀 맵핑 사용)
 *    - 방전 대기 (IOIF_UART_GetRecommendedDischargeTime_ms())
 *    - _RestoreUartPins() - UART 핀 복구 (제품별 핀 맵핑 사용)
 *    - 센서 전원 ON
 * 3. System Services 초기화
 *    - CANFD Rx Handler
 *    - UART Rx Handler (IMU 센서)
 *    - PnP Manager (XM Driver, XM Link)
 *    - Core Process (IPO 엔진)
 * 4. TIM 시작 (1ms 주기 제어 루프)
 * 
 * [아키텍처 리팩토링 (v3.0)]
 * - System Layer: 제품별 핀 맵핑 테이블 관리 (s_uart_pin_map[])
 * - IOIF Layer: 범용 GPIO/UART API 제공 (재사용 가능)
 * - 책임 분리: 제품 특화 vs. 범용 기능
 * 
 * [중요] "좀비 모드" 문제 해결:
 * 센서 전원을 껐을 때 UART TX/RX 핀을 통한 누설 전류로 인해
 * 센서가 완전히 꺼지지 않는 문제를 방전 시퀀스로 해결합니다.
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "system_startup.h"
#include "module.h"  /* System Layer 설정 */
#include "main.h" // HAL Handles

/* IOIF Layer */
#include "ioif_agrb_fdcan.h"
#include "ioif_agrb_uart.h"
#include "ioif_agrb_tim.h"
#include "ioif_agrb_gpio.h"  /* GPIO 범용 API (SetOutputLow, RestoreAFMode) */
#include "ioif_agrb_dwt.h"   /* DWT Cycle Counter (loop 시간 측정용) */

/* System Layer */
#include "canfd_rx_handler.h"
#include "uart_rx_handler.h"
#include "xm_drv.h"
#include "io_manager.h"
#include "imu_cdc.h"       /* ImuCdc_Init() */
#include "imu_dop_serial.h" /* ImuDopSerial_Init() */

/* PnP Service (V4.0: Slave API 분리) */
#include "agr_pnp_slave.h"
#include "imu_node_id.h"   /* AGR_NODE_ID_IMU_HUB, AGR_NODE_ID_XM */

/**
 *-----------------------------------------------------------
 * PRIVATE DEFINITIONS AND MACROS
 *-----------------------------------------------------------
 */

/**
 *-----------------------------------------------------------
 * PRIVATE ENUMERATIONS AND TYPES
 *-----------------------------------------------------------
 */

/**
 * @brief [Debug] Tx 통계 구조체
 */
 typedef struct {
    uint32_t total_calls;        /**< 총 호출 횟수 */
    uint32_t fifo_full_count;    /**< FIFO Full 발생 횟수 */
    uint32_t success_count;      /**< 전송 성공 횟수 */
    uint32_t last_can_id;        /**< 마지막 전송 CAN ID */
    uint32_t fifo_free_level;    /**< 현재 FIFO 여유 공간 */
    uint32_t min_fifo_level;     /**< 최소 FIFO 여유 (최대 부하 추적) */
    
    /* CAN ID별 전송 횟수 (상위 4개만) */
    struct {
        uint32_t can_id;
        uint32_t count;
    } top_tx[4];
    
    /* 100ms 동안의 전송 횟수 (Burst 감지) */
    uint32_t burst_count;
    uint32_t last_reset_ms;

    /* PnP Slave path 계측 (System_Fdcan2_Transmit 경유 HB/Bootup) —
     * xm_drv 의 _Drv_TxFunc_Queue 경로 카운터는 HB 를 거치지 않아 항상 0 이므로
     * 여기서 별도 집계해 XM 수신 측 hb_count 와 교차 일치 검증 가능하도록 한다. */
    uint32_t hb_pnp_direct_count;     /**< Heartbeat (CAN-ID 0x700+Node) 직접 송신 횟수 */
    uint32_t bootup_pnp_count;        /**< Bootup (CAN-ID 0x700+Node, len==1, data[0]==0) 송신 횟수 */
} Fdcan2_TxStats_t;

/**
 *-----------------------------------------------------------
 * PUBLIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

/* HAL Handles (main.c에서 extern) */
extern FDCAN_HandleTypeDef hfdcan2;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_lpuart1_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim7;

/* On-board LEDs */
IOIF_GPIOx_t g_gpio_func_led1_id = IOIF_GPIO_NOT_INITIALIZED; // PB8
IOIF_GPIOx_t g_gpio_func_led2_id = IOIF_GPIO_NOT_INITIALIZED; // PB9

/* On-board Buttons */
IOIF_GPIOx_t g_gpio_func_btn1_id = IOIF_GPIO_NOT_INITIALIZED; // PC10
IOIF_GPIOx_t g_gpio_func_btn2_id = IOIF_GPIO_NOT_INITIALIZED; // PC11

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* TIM — Public: main.c에서 IOIF_TIM_SetCallback()으로 App 콜백 주입 */
IOIF_TIMx_t g_main_tim_id = IOIF_TIM_NOT_INITIALIZED;
IOIF_TIMx_t g_sys_tim_id  = IOIF_TIM_NOT_INITIALIZED;

/* CAN */
IOIF_FDCANx_t g_fdcan2_id = IOIF_FDCAN_INVALID_ID;  /* Public: Application Layer에서 extern 참조 */

/* UART (6개 IMU) — 파일 내부 전용. 외부에서는 IOIF_UART_* API 경유 접근. */
static IOIF_UARTx_t  s_uart_ids[MAX_IMU_CHANNELS];

/* s_imu_sensor_types[] 제거 → Auto-Detect (V5.0)
 * 센서 타입은 ISR에서 Dual-Parser Probing으로 자동 감지합니다.
 * UartRxHandler_GetSensorType(ch)로 확인 가능.
 */

static IOIF_GPIOx_t s_imu_pwr_id; // IMU 전원 제어 핀 핸들

/* [디버깅] UART 초기화 실패 추적 */
volatile uint32_t g_uart_init_failed_mask = 0;  // 비트마스크 (bit 0~5)
/* [디버깅] fdcan tx fifo full 추적 */
static Fdcan2_TxStats_t s_fdcan2_tx_stats = {0};

/* ===== PnP Slave 인스턴스 (V4.0: System Layer에서 생성) =====
 * [변경 이유]
 * - 기존: xm_drv.c 내부에 AGR_PnP_Inst_t 생성 (Driver가 PnP 인스턴스 관리)
 * - 변경: System Layer에서 AGR_PnP_Slave_t 생성 → Driver에 주입
 * - 효과: PnP 라이프사이클을 System이 관리, Driver는 콜백만 등록
 * 
 * [.cursorrules 준수]
 * - "Slave PnP instance initialization should be handled by the System"
 * - Node당 1개의 Slave 인스턴스 (IMU Hub Node = 0x0D)
 */
static AGR_PnP_Slave_t s_slave_pnp;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _InitIoInterfaces(void);
static void _InitSystemServices(void);
static void _ForceDischargeUartPins(void);
static void _RestoreUartPins(void);

static void _UpdateTxStats(uint32_t can_id);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IMU Hub 시스템 부팅 및 초기화
 * @details main() 함수에서 단 한 번만 호출되어야 합니다.
 */
void System_Startup(void)
{
    /* ===== STEP 1: IOIF 계층 초기화 (H/W 준비) ===== */
    _InitIoInterfaces();

    /* ===================================================================
     * STEP 2: 센서 완전 리셋 시퀀스 (Power Cycle + Pin Discharge)
     * 
     * [아키텍처 리팩토링 완료]
     * - System Layer: 제품별 핀 맵핑 관리
     * - IOIF Layer: 범용 GPIO/UART API만 제공
     * 
     * [문제] 센서 전원(VCC)만 껐다 켜면 일부 센서가 "좀비 모드"에 빠짐:
     *        - UART TX/RX 핀을 통한 누설 전류로 센서가 완전히 꺼지지 않음
     *        - 리셋도 안 되고, 통신도 안 되는 먹통 상태
     * 
     * [해결] 전원 OFF 동안 UART 핀을 GPIO Output LOW로 강제 방전
     *        → 누설 전류 차단 + 잔류 전하 제거 → 완전한 리셋 보장
     * =================================================================== */
    
    /* 2-1. 센서 전원 OFF */
    IOIF_GPIO_RESET(s_imu_pwr_id);

    /* 2-2. UART 핀 강제 방전 (System Layer에서 제품별 핀 맵핑 사용) */
    _ForceDischargeUartPins();

    /* 2-3. 방전 시간 대기 (module.h 설정 사용) */
    IOIF_TIM_Delay(SENSOR_DISCHARGE_TIME_MS);
    
    /* 2-4. UART 핀 복구 (System Layer에서 제품별 핀 맵핑 사용) */
    _RestoreUartPins();

    /* ===== STEP 3: System Services 초기화 (S/W Logic) ===== */
    _InitSystemServices();

    /* ===== STEP 4: IMU Power On ===== */
    IOIF_GPIO_SET(s_imu_pwr_id);
    IOIF_TIM_Delay(SENSOR_STABILIZATION_TIME_MS);

    /* [NOTE] Timer Start는 System_StartTimers()에서 수행 — main.c가 App Init 완료 후 호출 */
}

/**
 * @brief TIM ISR 콜백 등록 + 타이머 시작 (Composition Root에서 호출)
 * @param main_loop  Main Control Timer (TIM16, 1ms) 콜백 — App IPO RunLoop
 * @param sys_loop   System Timer (TIM7, 1ms) 콜백 — System Task RunLoop
 * @details main.c에서 App Init 완료 후 마지막에 호출.
 *          타이머 시작 전까지는 ISR이 발생하지 않으므로 안전한 초기화 보장.
 */
void System_StartTimers(void)
{
    /* 타이머 시작 — 이 시점부터 ISR 활성화 (콜백은 main.c IOIF_TIM_SetCallback 으로 주입) */
    IOIF_TIM_Start_IT(g_main_tim_id);
    /* TIM7(System) 미start — ImuSystemTask 가 단일 MainTask(TIM16) ISR 로 fold 됨.
     * FDCAN Tx single-writer(BareMetal): 송신 컨텍스트를 TIM16 하나로 통일.
     * (htim7 MX_TIM7_Init 생성코드는 잔존하나 ISR 미활성 — A20: MX_ 영역 무수정) */
    /* IOIF_TIM_Start_IT(g_sys_tim_id); */
}

/**
 * @brief FDCAN2 핸들 getter (V2 아키텍처: extern 대신 getter 사용)
 * @return FDCAN2 IOIF 핸들
 */
IOIF_FDCANx_t System_GetFdcanHandle(void)
{
    return g_fdcan2_id;
}

/**
 * @brief FDCAN2 채널을 통해 CAN 메시지를 전송하는 래퍼 함수.
 * @param can_id CAN ID (11-bit 또는 29-bit)
 * @param data 전송할 데이터 포인터
 * @param len 데이터 길이 (0~64 bytes)
 * @return 0=성공, <0=에러
 * @note AGR_TxFunc_t 타입과 호환됩니다.
 *
 * [설계 의도 — PnP Slave 전용 경로]
 * PnP Slave(agr_pnp_slave)가 tx_func으로 이 함수를 등록하여 Bootup/Heartbeat 전송.
 * FIFO Full 시 음수 반환 → PnP 내부에서 last_sent_ms 갱신 안 함 → 다음 500ms에
 * 재시도. HB/Bootup은 재전송이 안전하므로 SW Queue fallback 없이 직접 전송한다
 * (SW Queue에 들어가면 TPDO 슬롯 예약 로직과 얽혀 드레인 지연 가능).
 *
 * 반면 DOP ctx의 tx_func(_Drv_TxFunc_Queue)은 HB/NMT를 SW Queue로 보낸다.
 * 두 경로가 혼재하지만 실제 Bootup/HB는 PnP Slave 경로로만 흘러가므로 일관성 유지.
 */
int System_Fdcan2_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    /* [Debug] Tx 통계 업데이트 */
    s_fdcan2_tx_stats.total_calls++;
    s_fdcan2_tx_stats.last_can_id = can_id;
    
    /* FIFO 여유 공간 확인 (통계용, 전송은 막지 않음) */
    uint32_t free_level = IOIF_FDCAN_GetTxFifoFreeLevel(g_fdcan2_id);
    s_fdcan2_tx_stats.fifo_free_level = free_level;
    
    /* 최소 여유 공간 추적 (최대 부하 감지) */
    if (s_fdcan2_tx_stats.min_fifo_level == 0) {
        s_fdcan2_tx_stats.min_fifo_level = 32;  /* 초기값 */
    }
    if (free_level < s_fdcan2_tx_stats.min_fifo_level) {
        s_fdcan2_tx_stats.min_fifo_level = free_level;
    }
    
    /* CAN ID별 전송 횟수 업데이트 */
    _UpdateTxStats(can_id);
    
    /* 100ms 동안 Burst 감지 */
    uint32_t current_ms = IOIF_TIM_GetTick();
    if (current_ms - s_fdcan2_tx_stats.last_reset_ms >= 100) {
        s_fdcan2_tx_stats.burst_count = 0;
        s_fdcan2_tx_stats.last_reset_ms = current_ms;
    }
    s_fdcan2_tx_stats.burst_count++;
    
    /* Tx 전송 (HAL이 내부에서 FIFO Full 체크함) */
    int result = (int)IOIF_FDCAN_Transmit(g_fdcan2_id, can_id, (uint8_t*)data, len);

    if (result == 0) {
        s_fdcan2_tx_stats.success_count++;

        /* PnP Slave path 는 HB + Bootup 만 흘러옴. CAN-ID 0x700+Node 로 판별하여
         * 메시지 타입별 카운터 증가 (Live Expression 에서 XM hb_count 와 교차 비교). */
        if ((can_id & 0x780U) == 0x700U) {
            if (len == 1U && data != NULL && data[0] == 0x00U) {
                s_fdcan2_tx_stats.bootup_pnp_count++;
            } else {
                s_fdcan2_tx_stats.hb_pnp_direct_count++;
            }
        }
    } else {
        /* FIFO Full 또는 기타 에러 */
        s_fdcan2_tx_stats.fifo_full_count++;
    }

    return result;
}

/**
 * @brief PnP Slave path Tx 통계 조회 (HB/Bootup)
 * @param[out] hb_count     HB 송신 누적 (NULL 허용)
 * @param[out] bootup_count Bootup 송신 누적 (NULL 허용)
 *
 * @details xm_drv 의 _Drv_TxFunc_Queue path 카운터는 현재 설계상 HB 를 거치지 않아
 *   항상 0. 실제 HB 계측은 이 함수로 조회한다. XM 수신 측 hb_count 와 교차 일치
 *   검증 가능.
 */
void System_GetFdcan2PnpTxStats(uint32_t* hb_count, uint32_t* bootup_count)
{
    if (hb_count     != NULL) *hb_count     = s_fdcan2_tx_stats.hb_pnp_direct_count;
    if (bootup_count != NULL) *bootup_count = s_fdcan2_tx_stats.bootup_pnp_count;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IOIF 계층 초기화 (Hardware Abstraction Layer)
 * @details 
 * 모든 하드웨어 주변 장치(FDCAN, UART, TIM, GPIO)를 IOIF 계층을 통해 초기화합니다.
 * 
 * [초기화 대상]
 * - IMU 전원 제어 핀 (PA8)
 * - FDCAN2 (XM10 통신용)
 * - UART 6개 (EBIMU-9DOFV6 센서)
 * - TIM16 (1ms 주기 제어 루프)
 * - GPIO (LED, Button)
 */
static void _InitIoInterfaces(void)
{
    /* ===== 0a. DWT Cycle Counter 활성화 (loop 시간 측정 기반) =====
     * 누락 시 IOIF_DWT_GetCycles() 가 항상 0 → loop_avg/max_us 전부 0 상태.
     * 반드시 첫 step. */
    (void)IOIF_DWT_Init();

    /* ===== 0. IMU Power Pin 초기화 (PA8) ===== */
    /* 센서 전원 제어를 위한 GPIO 출력 핀 */
    IOIF_GPIO_INITIALIZE(s_imu_pwr_id, GPIOA, GPIO_PIN_8, IOIF_GPIO_Mode_Output);

    /* ===== 1. FDCAN2 초기화 ===== */
    if (IOIF_FDCAN_ASSIGN(g_fdcan2_id, &hfdcan2) == AGRBStatus_OK) {
        /*
         * [FDCAN Hardware Filter 설정 - AGR-DOP V3 (Node ID v3.0)]
         *
         * [XM10 → IMU Hub 메시지]
         * 1. XM10 Bootup/Heartbeat: 0x702 (0x700 + AGR_NODE_ID_XM=0x02)
         * 2. SDO Request:           0x60D (0x600 + AGR_NODE_ID_IMU_HUB=0x0D)
         * 3. NMT:                   0x000 (Broadcast)
         * 4. SYNC:                  0x080 (Broadcast)
         *
         * [CubeMX 설정]
         * - Std Filters Nbr: 28 (STM32G4 Max)
         * - 각 메시지별 개별 Mask Filter 사용
         */
        FDCAN_FilterTypeDef filter_config;
        
        /* 
         * [필터 타입 선택]
         * - FDCAN_FILTER_DUAL: 2개 ID를 Exact match (더 명확)
         * - FDCAN_FILTER_MASK: 마스크 필터 (유연성)
         * 
         * [현재 선택: FDCAN_FILTER_DUAL]
         * - 각 필터가 2개 ID를 받을 수 있음 (필터 절약)
         * - Exact match 의도가 명확
         */
        
        /* Filter 0: NMT (0x000) + SYNC (0x080) - Broadcast 메시지 */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType = FDCAN_FILTER_DUAL,  /* Dual: 2개 ID Exact match */
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x000,  /* NMT */
            .FilterID2 = 0x080,  /* SYNC */
        };
        IOIF_FDCAN_ConfigFilter(g_fdcan2_id, &filter_config);
        
        /* Filter 1: SDO Request (0x60D) + XM10 Heartbeat (0x702) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 1,
            .FilterType = FDCAN_FILTER_DUAL,  /* Dual: 2개 ID Exact match */
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = (0x600 + AGR_NODE_ID_IMU_HUB),  /* SDO Request to IMU Hub (0x60D) */
            .FilterID2 = (0x700 + AGR_NODE_ID_XM),       /* XM Heartbeat/Bootup (0x702) */
        };
        IOIF_FDCAN_ConfigFilter(g_fdcan2_id, &filter_config);
        
        /* FDCAN START는 RX 콜백 등록 후에 수행 (_InitSystemServices에서) */
    }
    
    /* ===== 2. UART 초기화 (6개 EBIMU) ===== */
    IOIF_UART_Config_t imu_uart_conf = {
        .baudrate = IOIF_UART_Baudrate_921600,
        .rxMode   = IOIF_UART_MODE_IDLE_EVENT,
        .rx_event_callback = NULL
    };
    
    /* UART 인스턴스 할당 및 초기화 (6개 IMU 센서) */
    /* 초기화 실패 시 비트마스크에 기록 (디버깅용) */
    if (IOIF_UART_AssignInstance(&s_uart_ids[0], &huart1, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 0);
    }
    if (IOIF_UART_AssignInstance(&s_uart_ids[1], &huart2, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 1);
    }
    if (IOIF_UART_AssignInstance(&s_uart_ids[2], &huart3, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 2);
    }
    if (IOIF_UART_AssignInstance(&s_uart_ids[3], &huart4, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 3);
    }
    if (IOIF_UART_AssignInstance(&s_uart_ids[4], &huart5, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 4);
    }
    if (IOIF_UART_AssignInstance(&s_uart_ids[5], &hlpuart1, &imu_uart_conf) != AGRBStatus_OK) {
        g_uart_init_failed_mask |= (1 << 5);
    }
    
    /* [참고] UART/DMA 인터럽트는 stm32g4xx_hal_msp.c의 HAL_UART_MspInit()에서 활성화됨 */
    /* CubeMX가 생성한 코드에서 이미 처리되므로 여기서 중복 설정할 필요 없음 */
    
    /* ===== 3. Timer 초기화 ===== */
    /* 3-1. Main Timer: TIM16 (1ms, Priority 2) — 콜백은 main.c에서 IOIF_TIM_SetCallback()으로 주입 */
    IOIF_TIM_Assign(&g_main_tim_id, &htim16);

    /* 3-2. System Timer: TIM7 (1ms, Priority 4) — 콜백은 main.c에서 IOIF_TIM_SetCallback()으로 주입 */
    IOIF_TIM_Assign(&g_sys_tim_id, &htim7);

    /* ===== 4. I/O Manager 초기화 (LED, Button) ===== */
    /* GPIO 할당 및 초기화는 IO_Manager_Init 내부에서 수행 */
    IO_Manager_Init();
}

/**
 * @brief System Services 초기화
 * @details 모든 시스템 서비스를 초기화하고 연결합니다.
 */
static void _InitSystemServices(void)
{
    /* ===== 1. CANFD Rx Handler 초기화 (콜백 등록) ===== */
    /* ISR에서 XM_Drv_ProcessCANMessage를 직접 호출 */
    CanFdRxHandler_Init(g_fdcan2_id);
    
    /* ===== 2. FDCAN 시작 ===== */
    /* [중요] RX 콜백 등록 후 FDCAN을 시작해야 ISR에서 메시지를 처리할 수 있음 */
    IOIF_FDCAN_START(g_fdcan2_id);
    
    /* ===== 3. UART Rx Handler 초기화 (IMU 센서, Auto-Detect) ===== */
    UartRxHandler_Init(s_uart_ids, MAX_IMU_CHANNELS);
    
    /* ===== 4. PnP Slave 초기화 (V4.0: System Layer에서 관리) =====
     * [변경 이유]
     * - 기존: XM_Drv_Init() 내부에서 AGR_PnP 인스턴스 생성
     * - 변경: System Layer에서 AGR_PnP_Slave_t 초기화 → XM_Drv_Init()에 주입
     * - 효과: PnP 라이프사이클을 System이 관리, 일관된 아키텍처
     * 
     * [파라미터]
     * - my_node_id: AGR_NODE_ID_IMU_HUB (0x0D)
     * - master_node_id: AGR_NODE_ID_XM (0x02)
     * - timeout_ms: 5000ms (Breakpoint 디버깅 대응)
     * - tx_func: System_Fdcan2_Transmit (SW Queue 지원)
     * - get_tick: IOIF_TIM_GetTick (1ms 해상도)
     */
    AGR_PnP_Slave_Init(&s_slave_pnp,
                         AGR_NODE_ID_IMU_HUB,        /* 나의 Node ID (0x0D) */
                         AGR_NODE_ID_XM,             /* Master Node ID (0x02) — HB CAN-ID 0x702 */
                         5000,                       /* Master HB Timeout (5s) */
                         System_Fdcan2_Transmit,     /* CAN Tx (SW Queue) */
                         IOIF_TIM_GetTick);          /* Tick 함수 */
    
    /* ===== 5. XM Driver 초기화 (PnP Slave 주입) =====
     * tx_func 주입 없음 — Tx 는 xm_drv 내부 _Drv_TxFunc_Queue 로 일원화(single-writer).
     * PnP Slave 만 System_Fdcan2_Transmit 사용(위 4번에서 주입). */
    XM_Drv_Init(g_fdcan2_id, &s_slave_pnp);

    /* ===== 6. USB CDC 초기화 ===== */
    ImuCdc_Init();

    /* ===== 7. DOP Serial Transport 초기화 (USB-CDC) ===== */
    ImuDopSerial_Init();

    /* [NOTE] Application Task 초기화는 main.c에서 수행 (System→App 역방향 의존 제거) */
}

/**
 * @brief [IMU Hub 전용] UART 핀 맵핑 테이블
 * @details 각 UART의 TX/RX 핀 정보를 정의합니다.
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      tx_pin;
    uint16_t      rx_pin;
    uint32_t      af_number;  // Alternate Function 번호
} UART_PinMap_t;

static const UART_PinMap_t s_uart_pin_map[] = {
    /* USART1: PC4(TX), PC5(RX) */
    { .port = GPIOC, .tx_pin = GPIO_PIN_4, .rx_pin = GPIO_PIN_5, .af_number = GPIO_AF7_USART1 },
    /* USART2: PA2(TX), PA3(RX) */
    { .port = GPIOA, .tx_pin = GPIO_PIN_2, .rx_pin = GPIO_PIN_3, .af_number = GPIO_AF7_USART2 },
    /* USART3: PB10(TX), PB11(RX) */
    { .port = GPIOB, .tx_pin = GPIO_PIN_10, .rx_pin = GPIO_PIN_11, .af_number = GPIO_AF7_USART3 },
    /* UART4: PC10(TX), PC11(RX) */
    { .port = GPIOC, .tx_pin = GPIO_PIN_10, .rx_pin = GPIO_PIN_11, .af_number = GPIO_AF5_UART4 },
    /* UART5: PC12(TX), PD2(RX) - RX는 별도 포트 */
    { .port = GPIOC, .tx_pin = GPIO_PIN_12, .rx_pin = 0, .af_number = GPIO_AF5_UART5 },
    /* LPUART1: PC1(TX), PC0(RX) */
    { .port = GPIOC, .tx_pin = GPIO_PIN_1, .rx_pin = GPIO_PIN_0, .af_number = GPIO_AF8_LPUART1 },
};

/**
 * @brief [IMU Hub 전용] 센서 전원 리셋을 위한 UART 핀 방전
 * @details 
 * 모든 UART TX/RX 핀을 GPIO Output LOW로 설정하여 누설 전류를 차단합니다.
 * 
 * [아키텍처]
 * - System Layer: 제품별 핀 맵핑 정보 보유
 * - IOIF Layer: 범용 GPIO API 사용 (IOIF_GPIO_SetOutputLow)
 */
static void _ForceDischargeUartPins(void)
{
    for (uint32_t i = 0; i < sizeof(s_uart_pin_map) / sizeof(UART_PinMap_t); i++) {
        const UART_PinMap_t* map = &s_uart_pin_map[i];
        
        /* TX 핀을 GPIO Output LOW로 변경 */
        IOIF_GPIO_SetOutputLow(map->port, map->tx_pin);
        
        /* RX 핀을 GPIO Output LOW로 변경 */
        if (map->rx_pin != 0) {  // UART5 RX는 PD2 (별도 처리)
            IOIF_GPIO_SetOutputLow(map->port, map->rx_pin);
        }
    }
    
    /* UART5 RX (PD2) 별도 처리 */
    IOIF_GPIO_SetOutputLow(GPIOD, GPIO_PIN_2);
}

/**
 * @brief [IMU Hub 전용] UART 핀을 AF 모드로 복구
 * @details 
 * 모든 UART TX/RX 핀을 다시 Alternate Function 모드로 복구합니다.
 * 
 * [아키텍처]
 * - System Layer: 제품별 핀 맵핑 정보 보유
 * - IOIF Layer: 범용 GPIO API 사용 (IOIF_GPIO_RestoreAFMode)
 * - IOIF Layer: 범용 UART API 사용 (IOIF_UART_ResetState)
 */
static void _RestoreUartPins(void)
{
    /* UART 핀을 AF 모드로 복구 */
    for (uint32_t i = 0; i < sizeof(s_uart_pin_map) / sizeof(UART_PinMap_t); i++) {
        const UART_PinMap_t* map = &s_uart_pin_map[i];
        
        /* TX 핀 복구 */
        IOIF_GPIO_RestoreAFMode(map->port, map->tx_pin, map->af_number, GPIO_NOPULL);
        
        /* RX 핀 복구 */
        if (map->rx_pin != 0) {
            IOIF_GPIO_RestoreAFMode(map->port, map->rx_pin, map->af_number, GPIO_NOPULL);
        }
    }
    
    /* UART5 RX (PD2) 복구 */
    IOIF_GPIO_RestoreAFMode(GPIOD, GPIO_PIN_2, GPIO_AF5_UART5, GPIO_NOPULL);
    
    /* 모든 UART 상태 리셋 */
    IOIF_UART_ResetState(&huart1);
    IOIF_UART_ResetState(&huart2);
    IOIF_UART_ResetState(&huart3);
    IOIF_UART_ResetState(&huart4);
    IOIF_UART_ResetState(&huart5);
    IOIF_UART_ResetState(&hlpuart1);
}

/**
 * @brief CAN ID별 전송 횟수 업데이트
 */
static void _UpdateTxStats(uint32_t can_id)
{
    /* Top 4에 있는지 확인 */
    for (int i = 0; i < 4; i++) {
        if (s_fdcan2_tx_stats.top_tx[i].can_id == can_id) {
            s_fdcan2_tx_stats.top_tx[i].count++;
            return;
        }
    }
    
    /* 새로운 CAN ID → 비어있는 슬롯에 추가 */
    for (int i = 0; i < 4; i++) {
        if (s_fdcan2_tx_stats.top_tx[i].count == 0) {
            s_fdcan2_tx_stats.top_tx[i].can_id = can_id;
            s_fdcan2_tx_stats.top_tx[i].count = 1;
            return;
        }
    }
}

