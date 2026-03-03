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

// 모든 IOIF 헤더 포함
#include "ioif_agrb_fdcan.h"
#include "ioif_agrb_gpio.h"
#include "ioif_agrb_uart.h"
#include "ioif_agrb_adc.h"
#include "ioif_agrb_fs.h"
#include "ioif_agrb_tim.h"

// 생성할 시스템 서비스 헤더 포함
#include "pnp_manager.h"
#include "canfd_rx_handler.h"
#include "usb_mode_handler.h"
#include "uart_rx_handler.h"
#include "data_logger.h"
#include "cdc_handler.h"
#include "led_manager.h"
#include "button_manager.h"
#include "external_io.h"

#include "imu_hub_drv.h"
#include "mti-630.h"

// RTOS 및 HAL 핸들 참조
// FreeRTOS 및 CMSIS-OS 관련 헤더
#include "cmsis_os2.h"
#include "main.h" // HAL 핸들(hfdcan1) 및 RTOS 태스크 핸들을 extern으로 참조

// For Debug
// #include "can_bus_monitor.h"  // File not found - disabled

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

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;

extern osThreadId_t UserTaskHandle;
extern osThreadId_t StartupTaskHandle;

// On-board LEDs
IOIF_GPIOx_t  g_gpio_func_led1_id = IOIF_GPIO_NOT_INITIALIZED; // PB8
IOIF_GPIOx_t  g_gpio_func_led2_id = IOIF_GPIO_NOT_INITIALIZED; // PB9
IOIF_GPIOx_t  g_gpio_func_led3_id = IOIF_GPIO_NOT_INITIALIZED; // PB10

// On-board Buttons
IOIF_GPIOx_t  g_gpio_func_btn1_id = IOIF_GPIO_NOT_INITIALIZED; // PC10
IOIF_GPIOx_t  g_gpio_func_btn2_id = IOIF_GPIO_NOT_INITIALIZED; // PC11
IOIF_GPIOx_t  g_gpio_func_btn3_id = IOIF_GPIO_NOT_INITIALIZED; // PC12

/* 수동으로 관리할 UART4 핸들 */
UART_HandleTypeDef huart4_manual;
DMA_HandleTypeDef hdma_uart4_tx_manual;
DMA_HandleTypeDef hdma_uart4_rx_manual;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

/* --- 핸들(ID) 저장을 위한 static 변수 --- */
// USB
static IOIF_ADCx_t   s_adc1_id = IOIF_ADC_NOT_INITIALIZED; // (hadc1: CC핀 + Ext A0/A1)
static IOIF_GPIOx_t  s_gpio_usb_pwr_id = IOIF_GPIO_NOT_INITIALIZED;
static IOIF_GPIOx_t  s_gpio_usb_vbus_id = IOIF_GPIO_NOT_INITIALIZED;
static IOIF_GPIOx_t  s_gpio_usb_ufp_id = IOIF_GPIO_NOT_INITIALIZED;
// USB 제어 태스크에 주입할 설정 구조체
static TaskUSBControlTask_Init_t s_usbControlInitStruct;

// CAN
static IOIF_FDCANx_t s_fdcan1_id = IOIF_FDCAN_INVALID_ID;
// UART
static IOIF_UARTx_t  s_uart4_id = IOIF_UART_ID_NOT_ALLOCATED; // [신규] IMU용
static IOIF_UARTx_t  s_uart7_id = IOIF_UART_ID_NOT_ALLOCATED;
static IOIF_UARTx_t  s_uart8_id = IOIF_UART_ID_NOT_ALLOCATED;

// On-board LEDs
static IOIF_GPIOx_t  s_gpio_pwr_led_id = IOIF_GPIO_NOT_INITIALIZED; // PC6
static IOIF_GPIOx_t  s_gpio_rgb_r_id = IOIF_GPIO_NOT_INITIALIZED;   // PC7
static IOIF_GPIOx_t  s_gpio_rgb_g_id = IOIF_GPIO_NOT_INITIALIZED;   // PC8
static IOIF_GPIOx_t  s_gpio_rgb_b_id = IOIF_GPIO_NOT_INITIALIZED;   // PC9

// External IO 핸들
static IOIF_GPIOx_t s_dio_ids[EXT_DIO_COUNT]; // 8개
static IOIF_ADCx_t  s_adc2_id = IOIF_ADC_NOT_INITIALIZED; // (hadc2: Ext A2/A3)

// Sensor Hub Modules Power Enable
static IOIF_GPIOx_t s_gpio_pwr_emg_id = IOIF_GPIO_NOT_INITIALIZED; // PG4
static IOIF_GPIOx_t s_gpio_pwr_fes_id = IOIF_GPIO_NOT_INITIALIZED; // PG6
static IOIF_GPIOx_t s_gpio_pwr_imu_id = IOIF_GPIO_NOT_INITIALIZED; // PG8
static IOIF_GPIOx_t s_gpio_pwr_hmmg_id = IOIF_GPIO_NOT_INITIALIZED; // PG10
static IOIF_GPIOx_t s_gpio_pwr_left_grf_id = IOIF_GPIO_NOT_INITIALIZED; // PG12
static IOIF_GPIOx_t s_gpio_pwr_right_grf_id = IOIF_GPIO_NOT_INITIALIZED; // PG14

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

static void _InitIoInterfaces(void);
static void _InitSystemServices(void);
static void _Manual_UART4_MspInit(UART_HandleTypeDef* huart);

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

    // 3. CAN Bus Monitor 초기화
    // CanBusMon_Init();  // Disabled - can_bus_monitor.h not found
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
} Fdcan1_TxStats_t;

static Fdcan1_TxStats_t s_fdcan1_tx_stats = {0};

/**
 * @brief CAN ID별 전송 횟수 업데이트
 */
static void _UpdateTxStats(uint32_t can_id)
{
    /* Top 4에 있는지 확인 */
    for (int i = 0; i < 4; i++) {
        if (s_fdcan1_tx_stats.top_tx[i].can_id == can_id) {
            s_fdcan1_tx_stats.top_tx[i].count++;
            return;
        }
    }
    
    /* 새로운 CAN ID → 비어있는 슬롯에 추가 */
    for (int i = 0; i < 4; i++) {
        if (s_fdcan1_tx_stats.top_tx[i].count == 0) {
            s_fdcan1_tx_stats.top_tx[i].can_id = can_id;
            s_fdcan1_tx_stats.top_tx[i].count = 1;
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
    _UpdateTxStats(can_id);
    
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

/* 외부에서 호출할 모드 전환 함수 */
bool System_Switch_To_IMU_Mode(void)
{
    // 1. 사용 중이던 ADC/DMA 정지
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    
    // 2. 핀을 ADC 모드에서 해제 (필수는 아니지만 안전을 위해)
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

    // 3. UART4 핸들 설정
    huart4_manual.Instance = UART4;
    huart4_manual.Init.BaudRate = 921600;
    huart4_manual.Init.WordLength = UART_WORDLENGTH_8B;
    huart4_manual.Init.StopBits = UART_STOPBITS_1;
    huart4_manual.Init.Parity = UART_PARITY_NONE;
    huart4_manual.Init.Mode = UART_MODE_TX_RX;
    huart4_manual.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4_manual.Init.OverSampling = UART_OVERSAMPLING_16;
    // H7/G4는 UART 하드웨어 자체에 FIFO가 있습니다. 이걸 켜야 DMA 요청이 줄어듭니다.
    huart4_manual.FifoMode = UART_FIFOMODE_ENABLE;

    // Advanced Features 설정
    // 초기화할 기능 명시: DMA Error 설정과 Overrun 설정을 건드리겠다고 선언
    huart4_manual.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_DMADISABLEONERROR_INIT |
    							 	 	 	 	 UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;

    // 1. DMA on RX Error: Enable
    // (PE, FE 등 에러가 발생해도 DMA가 꺼지지 않게 설정 -> 노이즈 내성 강화)
    huart4_manual.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_ENABLEONRXERROR;

    // 2. Overrun: Disable
    // (기본값은 Overrun 감지 Enable(0)이므로, Disable(1) 하려면 명시적으로 써야 함)
    // 이렇게 하면 ORE 플래그가 하드웨어적으로 발생하지 않음.
    huart4_manual.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_ENABLE;
    // -------------------------------------------------------------------------
    
    // 4. MspInit 함수 바꿔치기 (이게 핵심!)
    // HAL_UART_Init은 내부적으로 HAL_UART_MspInit을 호출하는데, 
    // 이를 우리가 만든 _Manual_UART4_MspInit으로 대체해야 함.
    // 하지만 weak 함수라 덮어쓰기가 애매하므로, 직접 호출하거나
    // HAL_UART_Init 호출 전에 MspInit을 먼저 수행하는 트릭을 씁니다.
    _Manual_UART4_MspInit(&huart4_manual); // GPIO/DMA/Clock 설정

    if (HAL_UART_Init(&huart4_manual) != HAL_OK) {
        return false;
    }

    // UART FIFO 설정은 HAL_UART_Init 이후에 확장 함수로 수행해야 함
	// 1. TX/RX FIFO Threshold 설정
	// 순서: Threshold 설정 후 모드 Enable
	HAL_UARTEx_SetTxFifoThreshold(&huart4_manual, UART_TXFIFO_THRESHOLD_1_4);
	HAL_UARTEx_SetRxFifoThreshold(&huart4_manual, UART_RXFIFO_THRESHOLD_1_4);

	// 2. FIFO 모드 활성화
	HAL_UARTEx_EnableFifoMode(&huart4_manual);
	// -------------------------------------------------------------------------

    // 5. IOIF 및 Device 초기화
    // [수정] Xsens IMU용 UART4 설정 (Idle Event 모드)
    IOIF_UART_Config_t imu_config = {
		  .baudrate = IOIF_UART_Baudrate_921600,
          .rxMode = IOIF_UART_MODE_IDLE_EVENT,
		  .bounce_buffer_size = 0, // (Idle 모드는 불필요)
		  .rx_event_callback = NULL, 
    };

    // s_uart4_id에 할당 (s_uart4_id는 static 변수이므로 접근 가능한 함수 필요하거나 여기서 처리)
    IOIF_UART_AssignInstance(&s_uart4_id, &huart4_manual, &imu_config);
    
    // UartRxHandler에 알림 (재초기화)
    Uart4Rx_XsensIMU_Init(s_uart4_id);

    return true;
}

/* ===================================================================
 * Interrupt Service Routine Wrappers (ISR 대행 함수)
 * User Project의 stm32h7xx_it.c에서 호출할 함수들입니다.
 * =================================================================== */

/**
 * @brief UART4 RX DMA 인터럽트 처리 대행
 */
void System_ISR_DMA_UART4_RX_Manual(void)
{
    HAL_DMA_IRQHandler(&hdma_uart4_rx_manual);
}

/**
 * @brief UART4 TX DMA 인터럽트 처리 대행
 */
void System_ISR_DMA_UART4_TX_Manual(void)
{
    HAL_DMA_IRQHandler(&hdma_uart4_tx_manual);
}

/**
 * @brief UART4 글로벌 인터럽트 처리 대행 (IDLE 감지용)
 */
void System_ISR_UART4_Manual(void)
{
    HAL_UART_IRQHandler(&huart4_manual);
}

#if defined(USE_FREERTOS_DMA)
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
    if (UserTaskHandle != NULL) {
        osThreadResume(UserTaskHandle);
    }
    // (다른 Application Task가 있다면 여기서 마저 재개)

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
    // (누르면 HIGH이므로 PullDown 설정)
	// TODO: 누르면 LOW이니 다시 수정해야함!
    IOIF_GPIO_Initialize_t button_config = {
      .mode = IOIF_GPIO_Mode_Input,
      .pull = IOIF_GPIO_PullDown, // Pull-down
    };

    // Function Buttons (PC10, PC11, PC12)
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

    // FDCAN1 IOIF 드라이버 초기화
    // 1. main.c에서 초기화된 HAL 핸들(&hfdcan1)을 IOIF 계층에 '할당(ASSIGN)'하여 ID(s_fdcan1_id)를 발급받습니다.
    if (IOIF_FDCAN_ASSIGN(s_fdcan1_id, &hfdcan1) == AGRBStatus_OK) {
        /* 
         * [FDCAN Hardware Filter 설정 - DOP V1 + V2 혼재 최적화]
         * 
         * [수신 대상]
         * 1. **CM (Node 0x01) - DOP V1**:
         *    - CM → XM SDO:     0x212 (0x200 + 1(src) + 2(dst))
         *    - CM → XM PDO:     0x312 (0x300 + 1(src) + 2(dst))
         * 
         * 2. **IMU Hub (Node 0x08) - DOP V2**:
         *    - Boot-up/Heartbeat: 0x708 (0x700 + 8)
         *    - SDO Response:      0x588 (0x580 + 8)
         *    - TPDO1:             0x188 (0x180 + 8)
         *    - TPDO2:             0x288 (0x280 + 8)
         * 
         * 3. **센서 모듈 (Node 9~15) - DOP V2**:
         *    - EMG(9): 0x709, 0x589, 0x189, 0x289
         *    - FES(10): 0x70A, 0x58A, 0x18A, 0x28A
         *    - FSR(11): 0x70B, 0x58B, 0x18B, 0x28B
         *    - ... (Node 12~15)
         * 
         * [차단 대상 - Hardware Level]
         * - **MD (Node 0x07) - DOP V1**: XM10과 직접 통신 안함
         *   - MD → CM: 0x271, 0x371 (차단!)
         *   - CM → MD: 0x217, 0x317 (차단!)
         * 
         * [필터 전략]
         * - **Filter 0**: DOP V2 Heartbeat/Boot-up (0x708~0x70F) - Range
         * - **Filter 1**: DOP V2 SDO Response (0x588~0x58F) - Range
         * - **Filter 2**: DOP V2 TPDO1 (0x188~0x18F) - Range
         * - **Filter 3**: DOP V2 TPDO2 (0x288~0x28F) - Range
         * - **Filter 4**: DOP V1 CM Messages (0x212, 0x312) - Dual
         * - **Filter 5**: NMT Broadcast (0x000) - Dual (예비 슬롯 포함)
         * 
         * [효과]
         * ✅ MD 메시지 Hardware 차단 → ISR 오버헤드 제거
         * ✅ 센서 추가 시 필터 재설정 불필요 (0x08~0x0F 범위 커버)
         * ✅ DOP V1/V2 명확히 구분
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
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
        
        /* Filter 1: DOP V2 SDO Response (0x588 ~ 0x58F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 1,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x588,  /* IMU Hub (Node 8) */
            .FilterID2 = 0x58F,  /* 최대 Node 15까지 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
        
        /* Filter 2: DOP V2 TPDO1 (0x188 ~ 0x18F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 2,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x188,  /* IMU Hub (Node 8) */
            .FilterID2 = 0x18F,  /* 최대 Node 15까지 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
        
        /* Filter 3: DOP V2 TPDO2 (0x288 ~ 0x28F) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 3,
            .FilterType = FDCAN_FILTER_RANGE,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x288,  /* IMU Hub (Node 8) */
            .FilterID2 = 0x28F,  /* 최대 Node 15까지 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
        
        /* Filter 4: DOP V1 CM Messages (0x212, 0x312) */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 4,
            .FilterType = FDCAN_FILTER_DUAL,  /* 2개 ID Exact match */
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x212,  /* CM → XM SDO */
            .FilterID2 = 0x312,  /* CM → XM PDO */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
        
        /* Filter 5: NMT Broadcast (0x000) + 예비 슬롯 */
        filter_config = (FDCAN_FilterTypeDef){
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 5,
            .FilterType = FDCAN_FILTER_DUAL,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x000,  /* NMT Broadcast (DOP V2용, 현재 미사용) */
            .FilterID2 = 0x000,  /* 예비 슬롯 */
        };
        IOIF_FDCAN_ConfigFilter(s_fdcan1_id, &filter_config);
    }
    
    /* [중요] FDCAN START는 RX 콜백 등록 후에 수행 (StartupTask에서) */

    // --- USB 제어용 IOIF 초기화 (main.c에서 이동) ---
    // [수정] ADC1 (CC핀 2개 + 외부핀 2개) 그룹을 ID 1개로 할당
    IOIF_ADC_INITIALIZE(s_adc1_id, &hadc1);
    IOIF_GPIO_INITIALIZE(s_gpio_usb_pwr_id, USB_PWR_ON_GPIO_Port, USB_PWR_ON_Pin, IOIF_GPIO_Mode_Output);
    IOIF_GPIO_INITIALIZE(s_gpio_usb_ufp_id, USB_OTG_UFP_ID_GPIO_Port, USB_OTG_UFP_ID_Pin, IOIF_GPIO_Mode_Output);
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

    // --- External DIO 핀 IOIF 초기화 (PF3 ~ PF10) ---
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_1], EXT_GPIO_1_GPIO_Port, EXT_GPIO_1_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_2], EXT_GPIO_2_GPIO_Port, EXT_GPIO_2_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_3], EXT_GPIO_3_GPIO_Port, EXT_GPIO_3_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_4], EXT_GPIO_4_GPIO_Port, EXT_GPIO_4_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_5], EXT_GPIO_5_GPIO_Port, EXT_GPIO_5_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_6], EXT_GPIO_6_GPIO_Port, EXT_GPIO_6_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_7], EXT_GPIO_7_GPIO_Port, EXT_GPIO_7_Pin, IOIF_GPIO_Mode_Input);
    IOIF_GPIO_INITIALIZE(s_dio_ids[EXT_DIO_8], EXT_GPIO_8_GPIO_Port, EXT_GPIO_8_Pin, IOIF_GPIO_Mode_Input);
    
    // --- External ADC 핀 IOIF 초기화 (PA0, PA0_C, PA1, PA1_C) ---
    // [수정] ADC2 (외부핀 2개) 그룹을 ID 1개로 할당
    IOIF_ADC_INITIALIZE(s_adc2_id, &hadc2);

    // --- Sensor Hub Module Power Enable 핀 IOIF 초기화 (PG4, PG6, PG8, PG10, PG12, PG14) ---
    IOIF_GPIO_INITIALIZE(s_gpio_pwr_emg_id, EMG_PWR_EN_GPIO_Port, EMG_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_fes_id, FES_PWR_EN_GPIO_Port, FES_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_imu_id, IMU_PWR_EN_GPIO_Port, IMU_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_hmmg_id, HMMG_PWR_EN_GPIO_Port, HMMG_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_left_grf_id, L_GRF_PWR_EN_GPIO_Port, L_GRF_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
	IOIF_GPIO_INITIALIZE(s_gpio_pwr_right_grf_id, R_GRF_PWR_EN_GPIO_Port, R_GRF_PWR_EN_Pin, IOIF_GPIO_Mode_Output);
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

    // CAN 수신 핸들러 초기화 (RX 콜백 등록)
    // 이 함수 내부에서 스스로 큐를 생성하고, IOIF에 콜백을 등록하며,
    CanFdRxHandler_Init();

    // FDCAN 시작 (✅ RTOS/Bare-metal 공통 함수)
    IOIF_FDCAN_START(s_fdcan1_id);

    /* ===== PnP Manager 초기화 (Master PnP 생성) ===== */
    // ✅ Master PnP는 System Layer에서 하나만 관리
    PnPManager_Init(System_Fdcan1_Transmit);
    
    /* ===== DOP V2 Device Driver 초기화 (Master PnP에 등록) ===== */
    // ✅ IMU Hub Device 등록
    AGR_PnP_Inst_t* master_pnp = PnPManager_GetMasterPnP();
    if (master_pnp != NULL) {
        ImuHub_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    }
    
    /* 향후 확장: EMG, FSR, FES Device 추가 */
    // EMG_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    // FSR_Drv_Init(System_Fdcan1_Transmit, master_pnp);
    // FES_Drv_Init(System_Fdcan1_Transmit, master_pnp);

    // --- USB 제어 서비스 초기화 (main.c에서 이동) ---
    // 1. USB 제어 태스크에 전달할 설정 구조체(의존성)를 채웁니다.
    s_usbControlInitStruct.cc_id = s_adc1_id;
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
    ioif_filesystem_init();

    // --- USB MSC Data Logger 서비스 초기화 ---
    // (이 함수가 내부적으로 저순위 로깅 태스크를 생성)
    DataLogger_Init();

    // --- USB CDC Data Streaming 서비스 초기화 ---
    CdcStream_Init();

    // --- External IO 서비스 초기화 (ID 주입) ---
    // [수정] External IO 모듈에는 adc1과 adc2의 ID를 모두 주입
    ExternalIO_Init(s_dio_ids, s_adc1_id, s_adc2_id);

    // --- Power On LED 켜기 ---
    // 모든 초기화가 완료되었으므로 Power LED(PC6)를 켬
    IOIF_GPIO_SET(s_gpio_pwr_led_id);

    // --- Sensor Module Power Enable ---
    IOIF_GPIO_SET(s_gpio_pwr_emg_id); // PG4
    IOIF_GPIO_SET(s_gpio_pwr_fes_id); // PG6
    IOIF_GPIO_SET(s_gpio_pwr_imu_id); // PG8
    IOIF_GPIO_SET(s_gpio_pwr_hmmg_id); // PG10
    // IOIF_GPIO_SET(s_gpio_pwr_left_grf_id); // PG12
    // IOIF_GPIO_SET(s_gpio_pwr_right_grf_id); // PG14
}

/* UART4 하드웨어 수동 초기화 함수 (CubeMX가 안 만들어주므로 직접 작성) */
static void _Manual_UART4_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if(huart->Instance == UART4)
    {
        /* 1. 클럭 활성화 */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE(); // DMA2 클럭 확인

        /* 2. GPIO 재설정 (ADC Analog -> UART Alternate Function) */
        // PA0 -> UART4_TX, PA1 -> UART4_RX
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // 디지털 통신 모드
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART4; // AF8이 UART4임 (Datasheet 확인)
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* 3. DMA 수동 설정 (RX) */
        // 주의: ADC가 쓰고 있는 DMA Stream과 겹치지 않는지 확인해야 함!
        // UART4_RX는 보통 DMA1_Stream2 를 많이 씀 (CubeMX 참조)
        // DMA1_Stream0 : ADC1(EXT ADC input)
        // DMA1_Stream1 : ADC2(EXT ADC input)
        // DMA1_Stream2 : UART7(GRF Rx)
        // DMA1_Stream3 : UART8(GRF Rx)
        // DMA2_Stream0 : UART4(Rx)
        hdma_uart4_rx_manual.Instance = DMA2_Stream0; 
        hdma_uart4_rx_manual.Init.Request = DMA_REQUEST_UART4_RX;
        hdma_uart4_rx_manual.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_uart4_rx_manual.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_uart4_rx_manual.Init.MemInc = DMA_MINC_ENABLE;
        hdma_uart4_rx_manual.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_uart4_rx_manual.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        /* Circular Mode 설정 */
        hdma_uart4_rx_manual.Init.Mode = DMA_CIRCULAR; 

        /* 우선순위 최상위 (6채널 동시성 보장) */
        hdma_uart4_rx_manual.Init.Priority = DMA_PRIORITY_VERY_HIGH;

        /* FIFO 활성화 및 Threshold 설정 */
        hdma_uart4_rx_manual.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
        // Quarter Full (1/4): UART FIFO가 4바이트(전체 16B 기준) 찼을 때 버스 요청
        // Full로 하면 반응성 느려짐, 1Byte로 하면 버스 부하 큼. Quarter가 UART에 적절.
        hdma_uart4_rx_manual.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;

        /* Burst 설정 (UART는 1바이트씩 오므로 Single이 필수) */
        // 이걸 설정 안 하면 FIFO 켰을 때 오동작할 수 있음
        hdma_uart4_rx_manual.Init.MemBurst = DMA_MBURST_SINGLE;
        hdma_uart4_rx_manual.Init.PeriphBurst = DMA_PBURST_SINGLE;
                
        if (HAL_DMA_Init(&hdma_uart4_rx_manual) != HAL_OK) { /* Error */ }
        __HAL_LINKDMA(huart, hdmarx, hdma_uart4_rx_manual); // Rx 연결

        /* 4. [추가] DMA 수동 설정 (TX) - DMA2 Stream 1 */
        // (주의: CubeMX에서 DMA2 Stream 1가 비어있는지 확인 필수!)
        // hdma_uart4_tx_manual.Instance = DMA2_Stream1;
        // hdma_uart4_tx_manual.Init.Request = DMA_REQUEST_UART4_TX;
        // hdma_uart4_tx_manual.Init.Direction = DMA_MEMORY_TO_PERIPH; // 메모리 -> 페리페럴
        // hdma_uart4_tx_manual.Init.PeriphInc = DMA_PINC_DISABLE;
        // hdma_uart4_tx_manual.Init.MemInc = DMA_MINC_ENABLE;         // 메모리 주소 증가
        // hdma_uart4_tx_manual.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        // hdma_uart4_tx_manual.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        // hdma_uart4_tx_manual.Init.Mode = DMA_NORMAL;                // Tx는 Normal 모드 (한 번 보내고 끝)
        // hdma_uart4_tx_manual.Init.Priority = DMA_PRIORITY_LOW;    // Rx보다 낮아도 됨
        // hdma_uart4_tx_manual.Init.FIFOMode = DMA_FIFOMODE_ENABLE;

        // if (HAL_DMA_Init(&hdma_uart4_tx_manual) != HAL_OK) { /* Error */ }
        // __HAL_LINKDMA(huart, hdmatx, hdma_uart4_tx_manual); // Tx 연결 (중요!)

        /* 5. 인터럽트 설정 */
        // UART4 Global
        HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
        // DMA Rx
        HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        // // [추가] DMA Tx
        // HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
        // HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    }
}

#endif  // defined(USE_FREERTOS_DMA)
