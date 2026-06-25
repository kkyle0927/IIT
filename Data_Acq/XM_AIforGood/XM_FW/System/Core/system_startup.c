/**
 ******************************************************************************
 * @file    system_startup.c
 * @author  HyundoKim
 * @brief   시스템 초기화 및 부팅 시퀀스 관리
 * @version 0.1
 * @date    Oct 14, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */

#include "system_startup.h"

#include "stm32h7xx_hal.h"

#include "module.h"

#include "data_object_dictionaries.h"
#include "agr_dop_config.h"          /* AGR_CAN_ID_SYNC (0x080) */

// 모든 IOIF 헤더 포함
#include "ioif_agrb_fdcan.h"
#include "ioif_agrb_gpio.h"
#include "ioif_agrb_uart.h"
#include "ioif_agrb_adc.h"
#include "ioif_agrb_spi.h"
#include "ioif_agrb_fs.h"
#include "ioif_agrb_tim.h"
#include "ioif_agrb_psram.h"

// 생성할 시스템 서비스 헤더 포함
#include "pnp_task.h"           /* ✅ V4.0: PnP Task (PnPManager 대체) */
#include "canfd_rx_handler.h"
#include "usb_mode_handler.h"
#include "uart_rx_handler.h"
#include "data_logger.h"
#include "cdc_handler.h"
#include "cdc_dop_router.h"     /* sensor-studio GUI 용 DOP Serial 라우터 */
#include "xm_periph_stimulus.h" /* 회로 SI 측정용 페리페럴 stimulus 생성기 */
#include "led_manager.h"
#include "button_manager.h"
#include "external_io.h"

#include "imu_hub_drv.h"
#include "emg_hub_drv.h"
#include "fes_hub_drv.h"
#include "mti-630.h"

// Rev2.0 신규 디바이스 드라이버
#include "mcp79510.h"
#include "pca9957.h"


// RTOS 및 HAL 핸들 참조
// FreeRTOS 및 CMSIS-OS 관련 헤더
#include "cmsis_os2.h"
#include "main.h" // HAL 핸들(hfdcan1) 및 RTOS 태스크 핸들을 extern으로 참조

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
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;

// ✅ [Global Export] TIM2 IOIF ID (external_io.c에서 사용)
IOIF_TIMx_t g_tim2_id = IOIF_TIM_INVALID_ID;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi5;

extern QSPI_HandleTypeDef hqspi;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;

extern osThreadId_t DefaultTaskHandle;
extern osThreadId_t UserTaskHandle;
extern osThreadId_t StartupTaskHandle;

// On-board LEDs (Rev2.0: PB8/9/10 → PH13/14/15)
IOIF_GPIOx_t  g_gpio_func_led1_id = IOIF_GPIO_NOT_INITIALIZED; // PH13
IOIF_GPIOx_t  g_gpio_func_led2_id = IOIF_GPIO_NOT_INITIALIZED; // PH14
IOIF_GPIOx_t  g_gpio_func_led3_id = IOIF_GPIO_NOT_INITIALIZED; // PH15

// On-board Buttons (Rev2.0: PC10/11/12 → PC11/12/13)
IOIF_GPIOx_t  g_gpio_func_btn1_id = IOIF_GPIO_NOT_INITIALIZED; // PC11
IOIF_GPIOx_t  g_gpio_func_btn2_id = IOIF_GPIO_NOT_INITIALIZED; // PC12
IOIF_GPIOx_t  g_gpio_func_btn3_id = IOIF_GPIO_NOT_INITIALIZED; // PC13

/* [REMOVED — Rev2.0] UART4 핸들 삭제됨. External UART는 USART2(PD5/PD6) 사용. */

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* --- 핸들(ID) 저장을 위한 static 변수 --- */
// USB
static IOIF_ADCx_t   s_adc1_id = IOIF_ADC_NOT_INITIALIZED; // (hadc1: EXT_ADC PB0/PB1/PF11/PF12, 16-bit)
static IOIF_GPIOx_t  s_gpio_usb_pwr_id = IOIF_GPIO_NOT_INITIALIZED;
static IOIF_GPIOx_t  s_gpio_usb_vbus_id = IOIF_GPIO_NOT_INITIALIZED;
static IOIF_GPIOx_t  s_gpio_usb_ufp_id = IOIF_GPIO_NOT_INITIALIZED;
// USB 제어 태스크에 주입할 설정 구조체
static TaskUSBControlTask_Init_t s_usbControlInitStruct;

// CAN — Rev2.0: Ch1(CM, DOP V1) + Ch2(Sensor, DOP V2) 분리
static IOIF_FDCANx_t s_fdcan1_id = IOIF_FDCAN_INVALID_ID;  // Ch1: XM↔CM
static IOIF_FDCANx_t s_fdcan2_id = IOIF_FDCAN_INVALID_ID;  // Ch2: XM↔Sensor Module
/* UART */
static IOIF_UARTx_t  s_usart2_id = IOIF_UART_ID_NOT_ALLOCATED;  // Rev2.0: External UART (PD5/PD6)
static IOIF_UARTx_t  s_uart7_id = IOIF_UART_ID_NOT_ALLOCATED;
static IOIF_UARTx_t  s_uart8_id = IOIF_UART_ID_NOT_ALLOCATED;

/* SPI (Rev2.0: RTC + LED Driver) */
static IOIF_SPIx_t   s_spi2_rtc_id = IOIF_SPI_ID_NOT_ALLOCATED;     // SPI2: MCP79510 RTC (SW NSS)
static IOIF_GPIOx_t  s_gpio_spi2_cs_id = IOIF_GPIO_NOT_INITIALIZED; // PI0: SPI2 SW NSS (MCP79510 CS)
static IOIF_SPIx_t   s_spi5_led_drv_id = IOIF_SPI_ID_NOT_ALLOCATED; // SPI5: PCA9957 LED Driver (SW NSS)
static IOIF_GPIOx_t  s_gpio_spi5_cs_id = IOIF_GPIO_NOT_INITIALIZED; // PK1: SPI5 SW NSS (PCA9957 CS)

// On-board LEDs
static IOIF_GPIOx_t  s_gpio_pwr_led_id = IOIF_GPIO_NOT_INITIALIZED; // PC6
static IOIF_GPIOx_t  s_gpio_rgb_r_id = IOIF_GPIO_NOT_INITIALIZED;   // PC7
static IOIF_GPIOx_t  s_gpio_rgb_g_id = IOIF_GPIO_NOT_INITIALIZED;   // PC8
static IOIF_GPIOx_t  s_gpio_rgb_b_id = IOIF_GPIO_NOT_INITIALIZED;   // PC9

// External IO 핸들
static IOIF_GPIOx_t s_dio_ids[EXT_DIO_COUNT]; // 8개
static IOIF_ADCx_t  s_adc2_id = IOIF_ADC_NOT_INITIALIZED; // (hadc2: USB CC PF13/PF14, 12-bit)

// Sensor Hub Modules Power Enable (Rev2.0: PG -2 시프트)
static IOIF_GPIOx_t s_gpio_pwr_emg_id = IOIF_GPIO_NOT_INITIALIZED; // PG2 (was PG4)
static IOIF_GPIOx_t s_gpio_pwr_fes_id = IOIF_GPIO_NOT_INITIALIZED; // PG4 (was PG6)
static IOIF_GPIOx_t s_gpio_pwr_imu_id = IOIF_GPIO_NOT_INITIALIZED; // PG6 (was PG8)
static IOIF_GPIOx_t s_gpio_pwr_hmmg_id = IOIF_GPIO_NOT_INITIALIZED; // PG8 (was PG10)
static IOIF_GPIOx_t s_gpio_pwr_left_grf_id = IOIF_GPIO_NOT_INITIALIZED; // PG10 (was PG12)
static IOIF_GPIOx_t s_gpio_pwr_right_grf_id = IOIF_GPIO_NOT_INITIALIZED; // PG12 (was PG14)

// Rev2.0 신규 GPIO
static IOIF_GPIOx_t s_gpio_ext_pwr_sel_id = IOIF_GPIO_NOT_INITIALIZED;    // PE3: EXT_PWR_SEL_5V (3.3V/5V 선택)
static IOIF_GPIOx_t s_gpio_phy_rst_id = IOIF_GPIO_NOT_INITIALIZED;        // PC2: MCU_PHY_RST (ETH PHY HW Reset)
static IOIF_GPIOx_t s_gpio_led_drv_nreset_id = IOIF_GPIO_NOT_INITIALIZED; // PD8: LED_DRV_nRESET (PCA9957)
static IOIF_GPIOx_t s_gpio_led_drv_noe_id = IOIF_GPIO_NOT_INITIALIZED;    // PD9: LED_DRV_nOE (PCA9957)
static IOIF_GPIOx_t s_gpio_rtc_nint_id = IOIF_GPIO_NOT_INITIALIZED;       // PD4: RTC_nINT (MCP79510, 미사용)

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _InitIoInterfaces(void);
static void _InitSystemServices(void);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief XM10 시스템의 모든 기반 서비스를 초기화하고 시작합니다.
 * @details main() 함수에서 RTOS 스케줄러가 시작되기 전에 단 한 번만 호출되어야 합니다.
 */
void System_Startup(void)
{
    // 1. 하위 드라이버(IOIF) 계층을 초기화합니다.
    //    이 단계에서 FDCAN 하드웨어가 준비되고, s_fdcan1_id가 발급됩니다.
    _InitIoInterfaces();

    // 2. 시스템 서비스 계층을 초기화합니다.
    //    이 단계에서 IOIF 드라이버에 콜백을 등록하고, 백그라운드 태스크를 생성합니다.
    _InitSystemServices();
}

/**
 * @brief 초기화된 FDCAN1의 IOIF 핸들(ID)을 반환합니다.
 * @details 다른 시스템 모듈(예: canfd_rx_handler)이 IOIF 드라이버에 접근하기 위해 사용합니다.
 * @return FDCAN1의 IOIF_FDCANx_t 핸들.
 */
IOIF_FDCANx_t System_GetFDCAN1_Id(void)
{
    return s_fdcan1_id;
}

/**
 * @brief FDCAN1 채널을 통해 CAN 메시지를 전송하는 래퍼 함수.
 * @param can_id CAN ID (11-bit 또는 29-bit)
 * @param data 전송할 데이터 포인터
 * @param len 데이터 길이 (0~64 bytes)
 * @return 0=성공, <0=에러
 * @note AGR_TxFunc_t 타입과 호환됩니다.
 * 
 * [Week 8] Tx FIFO 체크 추가 (CAN Bus Off 방지)
 */
/**
 * @brief [Debug] Tx 통계 구조체
 */
typedef struct {
    uint32_t total_calls;        /**< 총 호출 횟수 */
    uint32_t fifo_full_count;    /**< FIFO Full 발생 횟수 */
    uint32_t hal_error_count;    /**< HAL 기타 에러 횟수 (Bus Off 등) */
    uint32_t success_count;      /**< 전송 성공 횟수 */
    uint32_t last_can_id;        /**< 마지막 전송 CAN ID */
    uint32_t last_hal_error;     /**< 마지막 HAL 에러 코드 */
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
} FdcanTxStats_t;

static FdcanTxStats_t s_fdcan1_tx_stats = {0};
static FdcanTxStats_t s_fdcan2_tx_stats = {0};  /* Rev2.0: Ch2 Tx 통계 */

/**
 * @brief CAN ID별 전송 횟수 업데이트 (Ch1/Ch2 공용)
 * @param[in,out] stats  대상 Tx 통계 구조체
 * @param[in]     can_id 전송된 CAN ID
 */
static void _UpdateTxStats(FdcanTxStats_t* stats, uint32_t can_id)
{
    /* Top 4에 있는지 확인 */
    for (int i = 0; i < 4; i++) {
        if (stats->top_tx[i].can_id == can_id) {
            stats->top_tx[i].count++;
            return;
        }
    }

    /* 새로운 CAN ID → 비어있는 슬롯에 추가 */
    for (int i = 0; i < 4; i++) {
        if (stats->top_tx[i].count == 0) {
            stats->top_tx[i].can_id = can_id;
            stats->top_tx[i].count = 1;
            return;
        }
    }
}

int System_Fdcan1_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    /* ✅ [Debug] Tx 통계 업데이트 */
    s_fdcan1_tx_stats.total_calls++;
    s_fdcan1_tx_stats.last_can_id = can_id;
    
    /* FIFO 여유 공간 확인 (통계용, 전송은 막지 않음) */
    uint32_t free_level = IOIF_FDCAN_GetTxFifoFreeLevel(s_fdcan1_id);
    s_fdcan1_tx_stats.fifo_free_level = free_level;
    
    /* 최소 여유 공간 추적 (최대 부하 감지) */
    if (s_fdcan1_tx_stats.min_fifo_level == 0) {
        s_fdcan1_tx_stats.min_fifo_level = 32;  /* 초기값 */
    }
    if (free_level < s_fdcan1_tx_stats.min_fifo_level) {
        s_fdcan1_tx_stats.min_fifo_level = free_level;
    }
    
    /* CAN ID별 전송 횟수 업데이트 */
    _UpdateTxStats(&s_fdcan1_tx_stats, can_id);
    
    /* 100ms 동안 Burst 감지 */
    uint32_t current_ms = IOIF_TIM_GetTick();
    if (current_ms - s_fdcan1_tx_stats.last_reset_ms >= 100) {
        s_fdcan1_tx_stats.burst_count = 0;
        s_fdcan1_tx_stats.last_reset_ms = current_ms;
    }
    s_fdcan1_tx_stats.burst_count++;
    
    /* Tx 전송 (HAL이 내부에서 FIFO Full 체크함) */
    AGRBStatusDef result = IOIF_FDCAN_Transmit(s_fdcan1_id, can_id, (uint8_t*)data, len);
    
    if (result == AGRBStatus_OK) {
        s_fdcan1_tx_stats.success_count++;
        return 0;
    } else {
        /* 에러 분류 */
        if (free_level == 0) {
            /* FIFO Full (최초 체크 값 사용) */
            s_fdcan1_tx_stats.fifo_full_count++;
        } else {
            /* 기타 HAL 에러 (Bus Off, Error Passive 등) */
            s_fdcan1_tx_stats.hal_error_count++;
            s_fdcan1_tx_stats.last_hal_error = (uint32_t)result;
        }
        return -1;
    }
}

/**
 * @brief 초기화된 FDCAN2의 IOIF 핸들(ID)을 반환합니다.
 * @details Rev2.0 Ch2 (XM↔Sensor Module, DOP V2) 전용.
 * @return FDCAN2의 IOIF_FDCANx_t 핸들.
 */
IOIF_FDCANx_t System_GetFDCAN2_Id(void)
{
    return s_fdcan2_id;
}

IOIF_GPIOx_t System_GetExtPwrSelGpioId(void)
{
    return s_gpio_ext_pwr_sel_id;
}

IOIF_UARTx_t System_GetExternalUartId(void)
{
    return s_usart2_id;
}

/* GRF L (Left foot) UART instance — UART7 (DMA2_Stream2/3). */
IOIF_UARTx_t System_GetGrfLUartId(void)
{
    return s_uart7_id;
}

/* GRF R (Right foot) UART instance — UART8. */
IOIF_UARTx_t System_GetGrfRUartId(void)
{
    return s_uart8_id;
}

IOIF_SPIx_t System_GetSpi2RtcId(void)
{
    return s_spi2_rtc_id;
}

IOIF_SPIx_t System_GetSpi5LedDrvId(void)
{
    return s_spi5_led_drv_id;
}

/**
 * @brief FDCAN2 채널(Ch2)을 통해 CAN 메시지를 전송하는 래퍼 함수.
 * @note  Sensor Module(DOP V2) 전용 채널. AGR_TxFunc_t 타입과 호환.
 * @param[in] can_id CAN ID (11-bit 또는 29-bit).
 * @param[in] data  전송할 데이터의 포인터 (const).
 * @param[in] len   전송할 데이터의 길이 (0~64 bytes).
 * @return 0 on success, <0 on error.
 */
int System_Fdcan2_Transmit(uint32_t can_id, const uint8_t* data, uint8_t len)
{
    /* [Debug] Tx 통계 업데이트 */
    s_fdcan2_tx_stats.total_calls++;
    s_fdcan2_tx_stats.last_can_id = can_id;

    /* FIFO 여유 공간 확인 */
    uint32_t free_level = IOIF_FDCAN_GetTxFifoFreeLevel(s_fdcan2_id);
    s_fdcan2_tx_stats.fifo_free_level = free_level;

    /* 최소 여유 공간 추적 */
    if (s_fdcan2_tx_stats.min_fifo_level == 0) {
        s_fdcan2_tx_stats.min_fifo_level = 32;
    }
    if (free_level < s_fdcan2_tx_stats.min_fifo_level) {
        s_fdcan2_tx_stats.min_fifo_level = free_level;
    }

    /* CAN ID별 전송 횟수 업데이트 */
    _UpdateTxStats(&s_fdcan2_tx_stats, can_id);

    /* 100ms Burst 감지 */
    uint32_t current_ms = IOIF_TIM_GetTick();
    if (current_ms - s_fdcan2_tx_stats.last_reset_ms >= 100) {
        s_fdcan2_tx_stats.burst_count = 0;
        s_fdcan2_tx_stats.last_reset_ms = current_ms;
    }
    s_fdcan2_tx_stats.burst_count++;

    /* Tx 전송 */
    AGRBStatusDef result = IOIF_FDCAN_Transmit(s_fdcan2_id, can_id, (uint8_t*)data, len);

    if (result == AGRBStatus_OK) {
        s_fdcan2_tx_stats.success_count++;
        return 0;
    } else {
        if (free_level == 0) {
            s_fdcan2_tx_stats.fifo_full_count++;
        } else {
            s_fdcan2_tx_stats.hal_error_count++;
            s_fdcan2_tx_stats.last_hal_error = (uint32_t)result;
        }
        return -1;
    }
}

/* ===== CiA 301 SYNC 전송 (Phase 1: Ch1 XM↔CM) =====
 * UserTask에서 호출: _FetchAllInputs() 직후, Read↔SYNC 동기화.
 *
 * [의도된 예외 — IOIF 캡슐화 규칙의 문서화된 예외]
 * HAL 직접 호출 (IOIF Tx Mutex 우회) — UserTask는 단일 Tx 경로이므로 경합 없음.
 * DETERMINISTIC 우선: 1kHz SYNC 핫패스에서 uncontended mutex 오버헤드 회피.
 *
 * 프레임 포맷 = Classic CAN (FDFormat=CLASSIC_CAN, BRS_OFF) — 의도적.
 * Ch1 의 PDO/SDO(System_Fdcan1_Transmit→IOIF)는 FD+BRS 인데 SYNC 만 Classic.
 * IOIF_FDCAN_Transmit 은 FD+BRS 라 그대로 치환하면 프로덕션 CM 링크의 SYNC
 * 와이어포맷이 바뀐다 → 지금은 현 상태 유지 (2026-06-24 결정).
 *
 * TODO(추후): SYNC 를 FD+BRS 로 통일 후 IOIF_FDCAN_Transmit 으로 캡슐화 (Q3 완결).
 *   전제: CM 수신단 FD SYNC 허용 검증 (A19 인접 — 프로덕션 와이어포맷 변경).
 */

static uint8_t s_sync_counter_ch1 = 0;

void System_SendSync_Ch1(void)
{
    FDCAN_TxHeaderTypeDef txHeader = {
        .Identifier          = AGR_CAN_ID_SYNC,
        .IdType              = FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = FDCAN_DLC_BYTES_1,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = FDCAN_BRS_OFF,
        .FDFormat            = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0
    };
    uint8_t data[1] = { ++s_sync_counter_ch1 };
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data);
}

/* ===== CiA 301 SYNC 전송 (Phase 2: Ch2 XM↔SM) =====
 * UserTask에서 호출: _FetchAllInputs() 직후, SM TPDO Phase-Lock용.
 * IOIF Tx 사용 — Ch2는 PnP/SDO 등 다중 Tx 소스 존재 → Mutex 보호 필요.
 * SM은 SYNC 수신 후 ≤1ms 내 TPDO 응답 → 다음 사이클 _FetchAllInputs에서 수신.
 */

static uint8_t s_sync_counter_ch2 = 0;

void System_SendSync_Ch2(void)
{
    uint8_t data[1] = { ++s_sync_counter_ch2 };
    IOIF_FDCAN_Transmit(s_fdcan2_id, AGR_CAN_ID_SYNC, data, 1);
}

/**
 * @brief [RTOS 태스크] "강한(strong)" 정의의 StartupTask 구현부.
 * @details main.c에서 생성된 __weak StartStartupTask를 덮어씁니다.
 * 시스템 초기화를 총괄하고, 완료되면 다른 태스크를 깨운 뒤 자신을 삭제합니다.
 */
void StartStartupTask(void *argument)
{
    // 1. 모든 드라이버, 서비스, IOIF를 초기화하는 메인 함수 호출
    System_Startup();

    // 2. 스케줄러를 중지하여 다른 태스크들을 원자적(atomic)으로 재개
    vTaskSuspendAll();

    // 3. main.c에서 생성된 다른 태스크들을 재개(Resume)
    //    (main.c의 `RTOS_THREADS` 영역 초기에 Task 생성 후, kernel 시작 전 osThreadSuspend(UserTaskHandle) 등이 추가되어야 함)
    if (DefaultTaskHandle != NULL) {
        osThreadResume(DefaultTaskHandle);
    }
    if (UserTaskHandle != NULL) {
        osThreadResume(UserTaskHandle);
    }

    // 4. 스케줄러를 다시 시작합니다.
    xTaskResumeAll();

    // 5. StartupTask는 임무를 완수했으므로 스스로를 삭제합니다.
    vTaskDelete(NULL);
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief IOIF 계층을 통해 보드의 모든 주변 장치 드라이버를 초기화합니다.
 * @details 이 함수는 RTOS에 의존하지 않으며, 순수 하드웨어 드라이버만 초기화합니다.
 */
static void _InitIoInterfaces(void)
{
    // --- On-board Button IOIF 초기화 ---
    // (누르면 LOW이므로 PullUp 설정, Active-Low)
    IOIF_GPIO_Initialize_t button_config = {
      .mode = IOIF_GPIO_Mode_Input,
      .pull = IOIF_GPIO_PullUp, // Pull-up (Active-Low: 누르면 LOW)
    };

    // Function Buttons (PC11, PC12, PC13) — Rev2.0: 1핀 시프트 (PC10→PSRAM QSPI)
    IOIF_GPIO_INITIALIZE(g_gpio_func_btn1_id, FUNC_BTN_1_GPIO_Port, FUNC_BTN_1_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_REINITIALIZE(g_gpio_func_btn1_id, &button_config);
    
    IOIF_GPIO_INITIALIZE(g_gpio_func_btn2_id, FUNC_BTN_2_GPIO_Port, FUNC_BTN_2_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_REINITIALIZE(g_gpio_func_btn2_id, &button_config);
    
    IOIF_GPIO_INITIALIZE(g_gpio_func_btn3_id, FUNC_BTN_3_GPIO_Port, FUNC_BTN_3_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_REINITIALIZE(g_gpio_func_btn3_id, &button_config);

    // Power LED (PC6)
    IOIF_GPIO_INITIALIZE(s_gpio_pwr_led_id, POWER_ON_LED_GPIO_Port, POWER_ON_LED_Pin, IOIF_GPIO_Mode_Output);
    
    // RGB LED (PC7, PC8, PC9)
    IOIF_GPIO_INITIALIZE(s_gpio_rgb_r_id, CM_LED_R_GPIO_Port, CM_LED_R_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_rgb_g_id, CM_LED_G_GPIO_Port, CM_LED_G_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_rgb_b_id, CM_LED_B_GPIO_Port, CM_LED_B_Pin, IOIF_GPIO_Mode_Output);

    // Function LEDs (PB8, PB9, PB10)
    IOIF_GPIO_INITIALIZE(g_gpio_func_led1_id, FUNC_LED_1_GPIO_Port, FUNC_LED_1_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(g_gpio_func_led2_id, FUNC_LED_2_GPIO_Port, FUNC_LED_2_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(g_gpio_func_led3_id, FUNC_LED_3_GPIO_Port, FUNC_LED_3_Pin, IOIF_GPIO_Mode_Output);

    /* ===== FDCAN1 (Ch1): XM↔CM 전용, DOP V1 ===== */
    if (IOIF_FDCAN_ASSIGN(s_fdcan1_id, &hfdcan1) == AGRBStatus_OK) {
        /*
         * [Rev2.0] Ch1 HW 필터 — CM(DOP V1) 전용
         * AS-IS: V1+V2 혼재 6개 필터 (단일 버스)
         * TO-BE: CM 전용 2개 필터 (Ch2로 Sensor 분리)
         *
         * Filter 0: CM→XM SDO(0x212) + PDO(0x312) — Dual Exact
         * Filter 1: NMT Broadcast(0x000) + 예비 — Dual
         */
        FDCAN_FilterTypeDef filter_config;

        /* Filter 0: DOP V1 CM Messages (0x212, 0x312) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType = FDCAN_FILTER_DUAL,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x212,  /* CM → XM SDO */
            .FilterID2 = 0x312,  /* CM → XM PDO */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);

        /* Filter 1: NMT Broadcast (0x000) + 예비 */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 1,
            .FilterType = FDCAN_FILTER_DUAL,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x000,  /* NMT Broadcast */
            .FilterID2 = 0x000,  /* 예비 슬롯 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);

        /* Filter 2: DOP V2 EMCY (0x081 ~ 0x08F) — CM 미래 확장 대비
         * Global Filter 가 REJECT 이므로 명시 필터 없으면 HW 단에서 drop.
         * 0x080 SYNC 는 XM 자체 송신이므로 수신 불필요 → 0x081 부터.
         * 현재 CM(DOP V1) 은 EMCY 미사용이나 향후 확장 시 바로 수신 가능.
         * canfd_rx_handler diag counter 로 집계 (on_emergency 콜백 미구현). */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 2,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x081,
            .FilterID2 = 0x08F,
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
    }

    /* ===== FDCAN2 (Ch2): XM↔Sensor Module 전용, DOP V2 (CiA 301) ===== */
    if (IOIF_FDCAN_ASSIGN(s_fdcan2_id, &hfdcan2) == AGRBStatus_OK) {
        /*
         * [Rev2.0] Ch2 HW 필터 — Sensor Module(DOP V2) 전용
         * Node 8~15: IMU Hub(8), EMG(9), FES(10), FSR(11), ...
         *
         * Filter 0: Heartbeat/Boot-up (0x708~0x70F) — Range
         * Filter 1: SDO Response      (0x588~0x58F) — Range
         * Filter 2: TPDO1             (0x188~0x18F) — Range
         * Filter 3: TPDO2             (0x288~0x28F) — Range
         * Filter 4: NMT Broadcast     (0x000) + 예비 — Dual
         */
        FDCAN_FilterTypeDef filter_config;

        /* Filter 0: DOP V2 Heartbeat/Boot-up (0x708 ~ 0x70F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x708,  /* IMU Hub (Node 8) */
            .FilterID2 = 0x70F,  /* 최대 Node 15까지 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);

        /* Filter 1: DOP V2 SDO Response (0x588 ~ 0x58F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 1,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x588,
            .FilterID2 = 0x58F,
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);

        /* Filter 2: DOP V2 TPDO1 (0x188 ~ 0x18F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 2,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x188,
            .FilterID2 = 0x18F,
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);

        /* Filter 3: DOP V2 TPDO2 (0x288 ~ 0x28F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 3,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x288,
            .FilterID2 = 0x28F,
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);

        /* Filter 4: NMT Broadcast (0x000) + Command Vector ACK (0x68C — FES) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 4,
            .FilterType = FDCAN_FILTER_DUAL,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x000,  /* NMT Broadcast */
            .FilterID2 = 0x680 + 0x0C,  /* 0x68C — FES Command Vector ACK (fes_hub_drv.h 계약) */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);

        /* Filter 5: DOP V2 EMCY (0x081 ~ 0x08F) — Sensor Module EMCY 경로
         * EMG/FES/IMU Hub 가 내부 fault 발생 시 EMCY 프레임 송신 (CiA 301 §7.2.7).
         * Global Filter REJECT 정책이므로 필터 명시 없으면 HW drop.
         * 0x080 SYNC 는 XM 자체 송신이므로 수신 불필요 → 0x081 부터.
         * 현재는 canfd_rx_handler diag counter 로 집계 (on_emergency 콜백 미구현). */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 5,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x081,
            .FilterID2 = 0x08F,
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan2_id, &filter_config);
    }

    /* [중요] FDCAN START는 RX 콜백 등록 후에 수행 (_InitSystemServices에서) */

    // --- USB 제어용 IOIF 초기화 (main.c에서 이동) ---
    // ADC1 (CC핀 2개 + 외부핀 2개) 그룹을 ID 1개로 할당
    IOIF_ADC_INITIALIZE(s_adc1_id, &hadc1);
    IOIF_GPIO_INITIALIZE(s_gpio_usb_pwr_id, USB_PWR_ON_GPIO_Port, USB_PWR_ON_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_usb_ufp_id, USB_DFP_UFP_GPIO_Port, USB_DFP_UFP_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_usb_vbus_id, USB_OTG_FS_VBUS_GPIO_Port, USB_OTG_FS_VBUS_Pin, IOIF_GPIO_Mode_Input);

    // GPIO Re-Initialize (DFP 모드)
    IOIF_GPIO_Initialize_t ufp_config = {
      .mode = IOIF_GPIO_Mode_Output,
      .pull = IOIF_GPIO_Floating,
      .init_state = false //초기 DFP 모드
    };
    IOIF_GPIO_REINITIALIZE(s_gpio_usb_ufp_id, &ufp_config);

    // --- UART IOIF 초기화 (main.c에서 이동) ---
    // FSR용 UART7, UART8 설정 (Idle Event 모드)
    IOIF_UART_Config_t fsr_config = {
		  .baudrate = IOIF_UART_Baudrate_921600,
          .rxMode = IOIF_UART_MODE_IDLE_EVENT,
		  .bounce_buffer_size = 512,
		  .rx_event_callback = NULL, 
    };
    IOIF_UART_AssignInstance(&s_uart7_id, &huart7, &fsr_config);
    IOIF_UART_AssignInstance(&s_uart8_id, &huart8, &fsr_config);

    // Rev2.0: External UART (USART2, PD5/PD6) — 범용 외부 시리얼 포트
    IOIF_UART_Config_t ext_uart_config = {
        .baudrate = IOIF_UART_Baudrate_921600,
        .rxMode = IOIF_UART_MODE_IDLE_EVENT,
        .bounce_buffer_size = 512,
        .rx_event_callback = NULL,
    };
    IOIF_UART_AssignInstance(&s_usart2_id, &huart2, &ext_uart_config);

    // --- External DIO 핀 IOIF 초기화 (PF3 ~ PF10) ---
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_1], EXT_GPIO_1_GPIO_Port, EXT_GPIO_1_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_2], EXT_GPIO_2_GPIO_Port, EXT_GPIO_2_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_3], EXT_GPIO_3_GPIO_Port, EXT_GPIO_3_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_4], EXT_GPIO_4_GPIO_Port, EXT_GPIO_4_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_5], EXT_GPIO_5_GPIO_Port, EXT_GPIO_5_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_6], EXT_GPIO_6_GPIO_Port, EXT_GPIO_6_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_7], EXT_GPIO_7_GPIO_Port, EXT_GPIO_7_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_8], EXT_GPIO_8_GPIO_Port, EXT_GPIO_8_Pin, IOIF_GPIO_Mode_Input);
    
    // --- ADC2 IOIF 초기화 (Rev2.0: USB CC PF13/PF14, 12-bit) ---
    IOIF_ADC_INITIALIZE(s_adc2_id, &hadc2);

    // --- TIM2 IOIF 초기화 (ADC2/ADC3 트리거용, 10kHz) ---
    // ✅ CubeMX에서 생성된 TIM2를 IOIF에 할당 (external_io.c에서 사용)
    IOIF_TIM_Assign(&g_tim2_id, &htim2);

    // --- Sensor Hub Module Power Enable 핀 IOIF 초기화 (PG4, PG6, PG8, PG10, PG12, PG14) ---
    IOIF_GPIO_INITIALIZE(s_gpio_pwr_emg_id, EMG_PWR_EN_GPIO_Port, EMG_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_fes_id, FES_PWR_EN_GPIO_Port, FES_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_imu_id, IMU_PWR_EN_GPIO_Port, IMU_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_hmmg_id, HMMG_PWR_EN_GPIO_Port, HMMG_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_left_grf_id, L_GRF_PWR_EN_GPIO_Port, L_GRF_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_right_grf_id, R_GRF_PWR_EN_GPIO_Port, R_GRF_PWR_EN_Pin, IOIF_GPIO_Mode_Output);

    // --- Rev2.0 신규 GPIO 초기화 ---
    IOIF_GPIO_INITIALIZE(s_gpio_ext_pwr_sel_id, EXT_PWR_SEL_5V_GPIO_Port, EXT_PWR_SEL_5V_Pin, IOIF_GPIO_Mode_Output);
    // --- Rev2.0 PHY Reset GPIO (RTL8201F) ---
    // HW 회로: PC2 → NPN 트랜지스터 → PHY nRST (극성 반전)
    //   PC2 HIGH(SET)  → 트랜지스터 ON  → nRST LOW  → PHY 리셋
    //   PC2 LOW(RESET) → 트랜지스터 OFF → nRST HIGH → PHY 동작
    // PC2 기본값 = LOW (CubeMX) → nRST=HIGH → PHY는 전원ON 이후 동작 상태 유지
    //
    // [WS5 동기화] PHY HW Reset은 여기서 수행하지 않음.
    // Why: HAL_ETH_Init의 DMA SWR은 REF_CLK(50MHz)이 필요.
    //      전원ON 이후 PHY가 strap pin 기반으로 REF_CLK 출력 중 → HAL_ETH_Init 성공.
    //      여기서 HW Reset하면 REF_CLK 끊김 → DMA SWR timeout → HAL_ETH_Init 실패.
    // Where: ethernetif.c low_level_init Code 1 (HAL_ETH_Init 성공 후 수행)
    IOIF_GPIO_INITIALIZE(s_gpio_phy_rst_id, MCU_PHY_RST_GPIO_Port, MCU_PHY_RST_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_led_drv_nreset_id, LED_DRV_nRESET_GPIO_Port, LED_DRV_nRESET_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_led_drv_noe_id, LED_DRV_nOE_GPIO_Port, LED_DRV_nOE_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_spi2_cs_id, RTC_SPI_NSSA_GPIO_Port, RTC_SPI_NSSA_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_spi5_cs_id, LED_DRV_SPI_NSSA_GPIO_Port, LED_DRV_SPI_NSSA_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_rtc_nint_id, RTC_nINT_GPIO_Port, RTC_nINT_Pin, IOIF_GPIO_Mode_Input);

    // --- Rev2.0 PSRAM IOIF 초기화 (APS6404L 8MB, QSPI Memory-Mapped) ---
    // QSPI HAL은 CubeMX MX_QUADSPI_Init()에서 이미 초기화됨
    // IOIF가 칩 리셋 + Quad Enable + Memory-Mapped Mode 진입까지 수행
#if defined(AGRB_IOIF_PSRAM_ENABLE)
    {
        IOIF_PSRAM_Initialize_t psram_init = {
            .hqspi = &hqspi,
        };
        AGRBStatusDef psram_status = ioif_psram.initialize(&psram_init);
        if (psram_status == AGRBStatus_OK) {
            ioif_psram.start_memory_mapped();
        }
    }
#endif

    // --- Rev2.0 SPI IOIF 초기화 (RTC + LED Driver) ---
    /* SPI2: MCP79510 RTC (SW NSS, 5MHz, Mode 0)
     * [변경] HW NSS → SW NSS (IOIF GPIO CS 관리)
     * [근거] CubeMX에서 PI0을 GPIO Output으로 설정 (SPI_NSS_SOFT)하였으나
     *        IOIF에 CS GPIO를 전달하지 않아 CS가 토글되지 않았음.
     *        → MCP79510 미선택 → MISO 전부 0xFF 반환.
     * [수정] PCA9957(SPI5)과 동일한 SW NSS 패턴 적용. */
    IOIF_SPI_Initialize_t spi2_rtc_init = {
        .hspi = &hspi2,
        .ss = s_gpio_spi2_cs_id,  /* SW NSS — IOIF가 CS 관리 (PI0) */
#if defined(USE_FREERTOS)
        .dma = { .tx_size = 16, .rx_size = 16 },
#endif
        .options = { .isr_mode = { .enable = false } },
        .timeout = 100,
    };
    ioif_spi.assign(&s_spi2_rtc_id, &spi2_rtc_init);

    // SPI5: PCA9957 LED Driver (Software NSS, ≤10MHz, Mode 0)
    IOIF_SPI_Initialize_t spi5_led_init = {
        .hspi = &hspi5,
        .ss = s_gpio_spi5_cs_id,  /* SW NSS — IOIF가 CS 관리 (PK1) */
#if defined(USE_FREERTOS)
        .dma = { .tx_size = 32, .rx_size = 32 },
#endif
        .options = { .isr_mode = { .enable = false } },
        .timeout = 100,
    };
    ioif_spi.assign(&s_spi5_led_drv_id, &spi5_led_init);
}
/**
 * @brief (RTOS)시스템의 안정적인 동작을 위해 백그라운드에서 실행될 모든
 * 서비스 모듈(태스크 포함)을 초기화하고 생성합니다.
 * IOIF로 생성된 메커니즘의 내용을 채우는 정책을 결정하는 공간
 */
static void _InitSystemServices(void)
{
    // --- LED 매니저 서비스 초기화 ---
	LedManager_InitLinkStatusLeds(s_gpio_rgb_r_id, s_gpio_rgb_g_id, s_gpio_rgb_b_id);
    LedManager_InitUserLeds(g_gpio_func_led1_id, g_gpio_func_led2_id, g_gpio_func_led3_id);

    // --- Button 매니저 서비스 초기화 ---
    ButtonManager_Init(g_gpio_func_btn1_id, g_gpio_func_btn2_id, g_gpio_func_btn3_id);

    /* ===== FDCAN Rx 핸들러 초기화 (Ch1 + Ch2 콜백 동시 등록) =====
     * [Rev2.0 변경]
     * - AS-IS: Ch1만 콜백 등록
     * - TO-BE: Ch1(CM) + Ch2(Sensor) 동일 콜백 등록
     *          CAN-ID 기반 라우팅이므로 채널 무관하게 올바른 Device로 전달
     */
    FDCANRxHandler_Init();

    /* ===== PnP Task 초기화 (Ch1/Ch2 Tx 경로 분리) =====
     * [Rev2.0 변경]
     * - AS-IS: PnP_Task_Init(Fdcan1_Tx, tick) — PnP Master + CM 모두 Ch1
     * - TO-BE: PnP_Task_Init(Fdcan2_Tx, Fdcan1_Tx, tick)
     *          PnP Master Heartbeat(0x702) → Ch2 (Sensor 모듈이 XM 생존 확인)
     *          CM SDO/PDO → Ch1 (CM 전용 버스)
     *
     * @note Device Driver Init() 보다 먼저 호출해야 합니다.
     * @note IOIF_FDCAN_START() 앞에서 초기화: AGR_MW bedccac (Slave Bootup 즉시
     *       송신) 이후, master 유효 전에 FDCAN ISR 이 활성화되면 첫 Bootup 이
     *       canfd_rx_handler 에서 조용히 drop 되는 race 회피.
     */
    PnP_Task_Init(System_Fdcan2_Transmit, System_Fdcan1_Transmit, IOIF_TIM_GetTick);

    /* FDCAN 시작 — Ch1 + Ch2 (각각 내부 RxTask 자동 생성).
     * PnP Master 초기화 완료 후 수행 — Slave Bootup race 회피. */
    IOIF_FDCAN_START(s_fdcan1_id);   /* IOIF_FDCRx0 (Ch1, CM) */
    IOIF_FDCAN_START(s_fdcan2_id);   /* IOIF_FDCRx1 (Ch2, Sensor) — Rev2.0 NEW */

    /* ===== DOP V2 Device Driver 초기화 — Ch2 전용 =====
     * [Rev2.0 변경]
     * - AS-IS: ImuHub_Drv_Init(Fdcan1_Tx, ...) — Ch1에서 Sensor 통신
     * - TO-BE: ImuHub_Drv_Init(Fdcan2_Tx, ...) — Ch2 전용 (버스 격리)
     */
    AGR_PnP_Master_t* master_pnp = PnP_Task_GetMaster();
    if (master_pnp != NULL) {
        ImuHub_Drv_Init(System_Fdcan2_Transmit, master_pnp);
        EmgHub_Drv_Init(System_Fdcan2_Transmit, master_pnp);
        FesHub_Drv_Init(System_Fdcan2_Transmit, master_pnp);
    }

    /* 향후 확장: FSR, GRF Device 추가 (모두 Ch2) */
    // GrfHub_Drv_Init(System_Fdcan2_Transmit, master_pnp);

    // --- USB 제어 서비스 초기화 (main.c에서 이동) ---
    // 1. USB 제어 태스크에 전달할 설정 구조체(의존성)를 채웁니다.
    s_usbControlInitStruct.cc_id = s_adc2_id;  /* Rev2.0: USB CC는 ADC2 (PF13/PF14) */
    s_usbControlInitStruct.enable_id = s_gpio_usb_pwr_id;
    s_usbControlInitStruct.vbus_id = s_gpio_usb_vbus_id;
    s_usbControlInitStruct.ufp_id = s_gpio_usb_ufp_id;
    s_usbControlInitStruct.enable_host_mode = true;
    s_usbControlInitStruct.enable_device_mode = true;

    // 2. USB 제어 모듈을 초기화합니다. (이 함수가 내부적으로 태스크를 생성)
    USBControl_Init(&s_usbControlInitStruct);

    // --- UART 수신 핸들러 초기화 (main.c에서 이동) ---
    // (이 함수가 내부적으로 Rx 태스크를 생성)
    UartRxHandler_Init(s_uart7_id, s_uart8_id);

    // DataLogger가 사용하기 전에 FileSystem 모듈(Mutex 등)을 먼저 초기화합니다.
    // 이 호출은 링커가 ioif_agrb_fs.c를 포함하도록 보장합니다.
    ioif_filesystem_init(IOIF_FileSystem_DeviceType_Auto);

    // --- USB MSC Data Logger 서비스 초기화 ---
    // (이 함수가 내부적으로 저순위 로깅 태스크를 생성)
    DataLogger_Init();

    // --- USB CDC Data Streaming 서비스 초기화 ---
    CdcStream_Init();

    // --- CDC ↔ AGR DOP Serial 라우터 초기화 (sensor-studio GUI / HW 양산 검증) ---
    // CdcStream_Init() 이후에 호출되어야 함 (CdcStream_Send 의존)
    CdcDopRouter_Init();

    // --- 페리페럴 stimulus 생성기 초기화 (회로 SI 측정용) ---
    // 각 stimulus slot 의 step 함수 등록. 0x7E00:N SDO Write 로 ON/OFF.
    XM_Stimulus_Init();

    // --- TIM2 시작 (ADC2/ADC3 트리거용, 10kHz) ---
    // ✅ External IO 초기화 **전에** TIM2를 시작해야 ADC2가 정상 동작!
    // ✅ ADC2는 TIM2 트리거에 의존하므로 타이머 먼저 시작 필수
    IOIF_TIM_StartBase(g_tim2_id);

    // --- TIM6 제거: SYNC는 UserTask(core_process.c)에서 직접 전송 ---
    // TIM6 독립 클럭 → UserTask Read와 phase drift 발생 → UserTask 내 호출로 변경

    // --- External IO 서비스 초기화 (ID 주입) ---
    // External IO 모듈에는 adc1과 adc2의 ID를 모두 주입
    ExternalIO_Init(s_dio_ids, s_adc1_id, s_adc2_id);

    // --- Rev2.0: RTC 초기화 (MCP79510, SPI2) ---
    MCP79510_Init(s_spi2_rtc_id);
    MCP79510_EnableBatteryBackup();

    // --- Rev2.0: LED Driver 초기화 (PCA9957, SPI5) ---
    {
        AGRBStatusDef pca_rc = PCA9957_Init(s_spi5_led_drv_id, s_gpio_led_drv_nreset_id, s_gpio_led_drv_noe_id);
        if (pca_rc == AGRBStatus_OK) {
            LedManager_InitChannelLeds();
        }
    }

    // --- Power On LED 켜기 ---
    // 모든 초기화가 완료되었으므로 Power LED(PC6)를 켬
    IOIF_GPIO_SET(s_gpio_pwr_led_id);

    // --- Sensor Module Power Enable ---
    IOIF_GPIO_SET(s_gpio_pwr_emg_id); // PG2 (Rev2.0: -2 시프트)
    IOIF_GPIO_SET(s_gpio_pwr_fes_id); // PG4
    IOIF_GPIO_SET(s_gpio_pwr_imu_id); // PG6
    IOIF_GPIO_SET(s_gpio_pwr_hmmg_id); // PG8
    // IOIF_GPIO_SET(s_gpio_pwr_left_grf_id); // PG10 — GRF 미사용, 전원 OFF 유지
    // IOIF_GPIO_SET(s_gpio_pwr_right_grf_id); // PG12 — GRF 미사용, 전원 OFF 유지

    // --- EXT_PWR_SEL_5V: 기본 3.3V (GPIO_PIN_RESET, CubeMX 초기값 유지) ---

    // --- Rev2.0 LwIP + UDP 소켓 초기화 ---
    // DefaultTask(Low priority)에서 MX_LWIP_Init() → EthUdpSocket_Init() 순서로 수행.
    // StartupTask(Realtime7) 완료 후 DefaultTask가 실행되므로 PHY 리셋이 반드시 선행됨.
    // IOC 재생성에 안전 (USER CODE 블록 내에서 처리).
}
