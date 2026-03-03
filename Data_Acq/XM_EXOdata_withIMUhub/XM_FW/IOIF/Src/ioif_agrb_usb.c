/**
 ******************************************************************************
 * @file    ioif_agrb_usb.c
 * @author  HyundoKim
 * @brief   [IOIF Layer] USB Host/Device 드라이버 추상화
 * @details
 * - STM32 CubeMX에서 생성된 Host/Device 라이브러리를 제어하는 IOIF API입니다.
 * - 이 모듈은 스택(Stack)의 초기화/종료, CDC 송수신만 담당합니다.
 * - [중요] 모드(Host/Device)를 결정하고 핀(VBUS/CC)을 제어하는 로직은
 * System Layer의 'usb_mode_handler'가 담당합니다.
 * @version 0.1 (Refactored)
 * @date    Nov 10, 2025
 *
 * @copyright Copyright (c) 2025 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */
#include "ioif_agrb_usb.h"
#if defined(AGRB_IOIF_USB_ENABLE)

#include "ioif_agrb_fs.h"

// HAL 드라이버 및 CubeMX 생성 코드
#include "usb_host.h"       // hUsbHostFS
#include "usbh_msc.h"       // USBH_MSC_CLASS
#include "usb_device.h"     // hUsbDeviceFS
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"    // CDC_Transmit_FS 및 콜백 등록 함수

#if defined(USE_FREERTOS_DMA)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

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
 * @brief 이 모듈의 내부 동작 상태
 */
typedef enum {
    IOIF_USB_MODE_IDLE = 0,
    IOIF_USB_MODE_HOST,
    IOIF_USB_MODE_DEVICE,
} IOIF_USB_Mode_e;

/**
 *-----------------------------------------------------------
 * PULBIC (GLOBAL) VARIABLES
 *-----------------------------------------------------------
 */

// CubeMX에서 생성된 핸들 (extern)
extern USBH_HandleTypeDef hUsbHostFS;
extern USBD_HandleTypeDef hUsbDeviceFS;

// 모듈 내부 상태 (이중 초기화/해제 방지)
volatile bool g_is_host_initialized = false;
volatile bool g_is_device_initialized = false;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) VARIABLES
 *------------------------------------------------------------
 */

// System Layer에서 등록한 Host 이벤트 콜백
static IOIF_USB_HostCallback_t s_host_callback = NULL;

/* [수정] IOIF가 System Layer의 콜백을 저장할 포인터 */
static IOIF_USB_CDC_TxCallback_t s_cdc_tx_callback = NULL;
static IOIF_USB_CDC_RxCallback_t s_cdc_rx_callback = NULL;

/**
 *------------------------------------------------------------
 * STATIC (PRIVATE) FUNCTION PROTOTYPES
 *------------------------------------------------------------
 */

// USBH_Init()에 등록할 내부 콜백 함수
static void _internal_host_callback(USBH_HandleTypeDef *phost, uint8_t id);

// USBD_CDC_ItfTypeDef의 TxCpltCallback에 등록할 내부 콜백 함수
static void _internal_cdc_tx_complete_callback(void);

// USBD_CDC_ItfTypeDef의 ReceiveCallback에 등록할 내부 콜백 함수
static void _internal_cdc_rx_complete_callback(uint8_t* data, uint32_t length);

/**
 *------------------------------------------------------------
 * PUBLIC FUNCTIONS
 *------------------------------------------------------------
 */

/* ===================================================================
 * HOST 모드 API
 * =================================================================== */

AGRBStatusDef ioif_usb_host_init(IOIF_USB_HostCallback_t user_callback)
{
    if (g_is_host_initialized) return AGRBStatus_OK; // 이미 초기화됨
    if (g_is_device_initialized) return AGRBStatus_BUSY; // Device 모드 사용 중

    // [!! 핵심 수정 !!] 
    // 초기화 시작 전, 하드웨어를 확실하게 리셋합니다.
    
    // 1. 리셋을 위해 클럭을 강제로 켭니다. (DeInit에 의해 꺼져있을 수 있음)
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    // 2. 하드웨어 강제 리셋 (Reset Pulse)
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    vTaskDelay(pdMS_TO_TICKS(2)); // 짧은 펄스 유지
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();

    // 3. 리셋 후 안정화 대기 (중요)
    vTaskDelay(pdMS_TO_TICKS(10));

    g_is_host_initialized = true;
    s_host_callback = user_callback;

    // 3. [스택 초기화]
    // 이제 완전히 깨끗해진 페리페럴을 Host 모드로 초기화합니다.
    if (USBH_Init(&hUsbHostFS, _internal_host_callback, HOST_FS) != USBH_OK) {
        g_is_host_initialized = false; // [롤백] 실패 시 플래그 원복
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK) {
        USBH_DeInit(&hUsbHostFS); // 실패 시 DeInit
        g_is_host_initialized = false; // [롤백] 실패 시 플래그 원복
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBH_Start(&hUsbHostFS) != USBH_OK) {
        USBH_DeInit(&hUsbHostFS); // 실패 시 DeInit
        g_is_host_initialized = false; // [롤백] 실패 시 플래그 원복
        return AGRBStatus_INITIAL_FAILED;
    }

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_usb_host_deinit(void)
{
    // 1. 이미 비활성화 상태면 리턴
    if (!g_is_host_initialized) {
        return AGRBStatus_OK;
    }

    // 2. 스택 및 RTOS 태스크 종료 (여기서 클럭이 꺼질 수 있음)
    USBH_DeInit(&hUsbHostFS);
    s_host_callback = NULL;
    g_is_host_initialized = false; 
    
    // 3. [핵심 수정] 하드웨어 강제 리셋 시퀀스
    // 리셋을 먹이기 위해 클럭을 잠깐 켭니다.
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE(); 
    
    // 리셋 펄스 인가
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    vTaskDelay(pdMS_TO_TICKS(2));
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    
    // 4. (선택 사항) 다시 클럭을 꺼서 저전력 상태로 만듦
    // 다음 init에서 어차피 켜므로 안 꺼도 무방하지만, 정석은 끄는 것입니다.
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    // 5. 안정화 대기
    // (1~10ms 정도면 충분)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    return AGRBStatus_OK;
}

bool ioif_usb_host_is_port_enabled(void)
{
    if (!g_is_host_initialized) {
        return false;
    }
    return (USBH_IsPortEnabled(&hUsbHostFS) != 0U);
}

/* ===================================================================
 * DEVICE 모드 API
 * =================================================================== */

AGRBStatusDef ioif_usb_device_init(IOIF_USB_CDC_TxCallback_t user_tx_callback, IOIF_USB_CDC_RxCallback_t user_rx_callback)
{
    if (g_is_device_initialized) return AGRBStatus_OK; // 이미 초기화됨
    if (g_is_host_initialized) return AGRBStatus_BUSY; // Host 모드 사용 중

    // [!! 핵심 수정 !!] Device Init에도 동일하게 적용
    
    // 1. 클럭 활성화
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    
    // 2. 하드웨어 강제 리셋
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    vTaskDelay(pdMS_TO_TICKS(2));
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    
    // 3. 안정화 대기
    vTaskDelay(pdMS_TO_TICKS(10));

    g_is_device_initialized = true;
    s_cdc_tx_callback = user_tx_callback;
    s_cdc_rx_callback = user_rx_callback;

    // MX_USB_DEVICE_Init()의 핵심 로직 수행
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
        g_is_device_initialized = false; // [롤백]
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
        USBD_DeInit(&hUsbDeviceFS);
        g_is_device_initialized = false; // [롤백]
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
        USBD_DeInit(&hUsbDeviceFS);
        g_is_device_initialized = false; // [롤백]
        return AGRBStatus_INITIAL_FAILED;
    }

    // [중요] usbd_cdc_if.c가 제공하는 콜백 등록 함수 호출
    CDC_Register_Callbacks(_internal_cdc_tx_complete_callback, _internal_cdc_rx_complete_callback);

    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        USBD_DeInit(&hUsbDeviceFS);
        g_is_device_initialized = false; // [롤백]
        return AGRBStatus_INITIAL_FAILED;
    }

    return AGRBStatus_OK;
}

AGRBStatusDef ioif_usb_device_deinit(void)
{
    if (!g_is_device_initialized) {
        return AGRBStatus_OK;
    }

    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    
    // 등록된 콜백 초기화 (usbd_cdc_if.c에 구현 필요)
    s_cdc_tx_callback = NULL;
    s_cdc_rx_callback = NULL;
    CDC_Register_Callbacks(NULL, NULL);

    g_is_device_initialized = false;

    // 하드웨어 강제 리셋 시퀀스
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE(); // 클럭 켜고
    
    __HAL_RCC_USB_OTG_FS_FORCE_RESET(); // 리셋하고
    vTaskDelay(pdMS_TO_TICKS(2));
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET(); // 풀고
    
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE(); // 다시 끄고 (선택)

    vTaskDelay(pdMS_TO_TICKS(10)); // 안정화

    return AGRBStatus_OK;
}

/**
 *------------------------------------------------------------
 * STATIC FUNCTIONS
 *------------------------------------------------------------
 */

/**
 * @brief [내부] USB Host 스택의 이벤트를 수신하여 System Layer 콜백으로 전달
 */
static void _internal_host_callback(USBH_HandleTypeDef *phost, uint8_t id)
{
    if (s_host_callback != NULL) {
        // 이벤트를 상위(System) 계층으로 그대로 전달
        s_host_callback(id);
    }
}

/**
 * @brief [내부] CDC TX 완료 콜백
 */
static void _internal_cdc_tx_complete_callback(void)
{
    if (s_cdc_tx_callback != NULL) {
        // 이벤트를 상위(System) 계층으로 그대로 전달
        s_cdc_tx_callback();
    }
}

/**
 * @brief [내부] CDC RX 완료 콜백
 */
static void _internal_cdc_rx_complete_callback(uint8_t* data, uint32_t length)
{
    if (s_cdc_rx_callback != NULL) {
        // 이벤트를 상위(System) 계층으로 그대로 전달
        s_cdc_rx_callback(data, length);
    }
}

#endif /* AGRB_IOIF_USB_ENABLE */
