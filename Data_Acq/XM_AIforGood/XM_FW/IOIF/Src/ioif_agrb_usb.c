/**
 ******************************************************************************
 * @file    ioif_agrb_usb.c
 * @author  HyundoKim
 * @brief   [IOIF Layer] USB 3-mode 드라이버 (CDC_ONLY / COMPOSITE / DRP)
 * @details
 * 모드 분기 매트릭스:
 *   - MODE_CDC_ONLY  : Device CDC 단일 (SM 계열 — 현재 master OFF)
 *   - MODE_COMPOSITE : Device MSC+CDC (CM — SD 노출 + 시리얼 콘솔)
 *   - MODE_DRP       : Device CDC ↔ Host MSC 런타임 스위칭 (XM)
 *
 * 공통 prologue (모든 모드):
 *   1) 이미 init 상태면 OK 반환
 *   2) DRP 모드: Host 동작 중이면 BUSY
 *   3) NVIC disable → CLK enable → FORCE_RESET → RELEASE (spurious IRQ 방지)
 *   4) CRS 초기화 (H7 HSI48 → USB SOF 동기화)
 *   5) MX_USB_DEVICE_Init() 위임 — 클래스 등록/Start 는 CubeMX 생성 코드 담당
 *   6) CDC 콜백 shim 등록
 *
 * NVIC priority 책임:
 *   - IOIF 가 명시 설정한다 (Device=5, Host=6 — CubeMX __weak MspInit 과 동일값).
 *     이유: 프로젝트 측 hal_msp.c strong override 가 NVIC 설정을 생략하는 경우가
 *     많고 (XM 의 경우 전부 누락), 그러면 default priority=0 으로 enable 되어
 *     FreeRTOS vPortValidateInterruptPriority() assertion 에 걸린다
 *     (USBH_OS_PutMessage → osMessageQueuePut → xQueueGenericSendFromISR 경로).
 *   - Device=5 : configMAX_SYSCALL_INTERRUPT_PRIORITY 경계 — FromISR API 허용.
 *   - Host=6   : Device/FDCAN 보다 낮아 CAN RX 지연 방지.
 *
 * @copyright Copyright (c) 2026 Angel Robotics Co., Ltd. All rights reserved.
 ******************************************************************************
 */
#include "ioif_agrb_usb.h"
#if defined(AGRB_IOIF_USB_ENABLE)

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#if defined(AGRB_IOIF_USB_MODE_COMPOSITE)
  #include "usbd_storage_if.h"
  #include "usb_composite_service.h"   /* Plan v6 Step 3.2: DTR ISR hook forward */
  #ifdef USE_USBD_COMPOSITE
    #include "usbd_composite_builder.h"
  #endif
#endif

#if defined(AGRB_IOIF_USB_MODE_DRP)
  #include "usb_host.h"
  #include "usbh_core.h"
  #include "usbh_msc.h"
#endif

#if defined(USE_FREERTOS)
  #include "FreeRTOS.h"
  #include "task.h"
#endif

/*
 *-----------------------------------------------------------
 * PRIVATE MACROS
 *-----------------------------------------------------------
 */

#if defined(USE_FREERTOS)
    #define IOIF_USB_SAFE_DELAY_MS(ms)  do {                                \
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {            \
            vTaskDelay(pdMS_TO_TICKS(ms));                                  \
        } else {                                                            \
            HAL_Delay(ms);                                                  \
        }                                                                   \
    } while(0)
#else
    #define IOIF_USB_SAFE_DELAY_MS(ms)  HAL_Delay(ms)
#endif

/*
 *-----------------------------------------------------------
 * PUBLIC VARIABLES
 *-----------------------------------------------------------
 */

volatile bool g_is_device_initialized = false;

#if defined(AGRB_IOIF_USB_MODE_DRP)
volatile bool g_is_host_initialized = false;
extern USBH_HandleTypeDef hUsbHostFS;
#endif

#ifdef DEBUG
/* [DEBUG] 공통 OTG 레지스터 진단 */
volatile uint32_t dbg_usb_gahbcfg  = 0;
volatile uint32_t dbg_usb_gintmsk  = 0;
volatile uint32_t dbg_usb_gintsts  = 0;
volatile uint32_t dbg_usb_gusbcfg  = 0;
volatile uint32_t dbg_usb_nvic_en  = 0;
volatile uint32_t dbg_usb_rcc_ahb1 = 0;
volatile uint32_t dbg_usb_rcc_cr   = 0;

#if defined(AGRB_IOIF_USB_MODE_COMPOSITE)
/* [DEBUG] Composite descriptor/state — NumClasses 는 2 여야 함 */
volatile uint32_t dbg_usb_dev_state     = 0;
volatile uint32_t dbg_usb_num_classes   = 0;
volatile uint32_t dbg_usb_cfg_desc_sz   = 0;
volatile uint8_t  dbg_usb_cfg_desc_hdr[9] = {0};
volatile uint32_t dbg_usb_class0_active = 0;
volatile uint32_t dbg_usb_class1_active = 0;
volatile uint32_t dbg_usb_class0_numep  = 0;
volatile uint32_t dbg_usb_class1_numep  = 0;
#endif

#if defined(AGRB_IOIF_USB_MODE_DRP)
/* [DEBUG] Host HPRT */
volatile uint32_t dbg_usb_hprt = 0;
#endif
#endif /* DEBUG */

/*
 *-----------------------------------------------------------
 * STATIC VARIABLES
 *-----------------------------------------------------------
 */

static IOIF_USB_CDC_TxCallback_t  s_cdc_tx_callback  = NULL;
static IOIF_USB_CDC_RxCallback_t  s_cdc_rx_callback  = NULL;
static IOIF_USB_CDC_DtrCallback_t s_cdc_dtr_callback = NULL;

/* COMPOSITE state machine (SD ownership, DTR deferred, CDC active) moved to
 * CM_FW/System/Comm/USB/usb_composite_service.c (Plan v6 Step 3.2). IOIF is
 * now a pure HAL adapter for USB. */

#if defined(AGRB_IOIF_USB_MODE_DRP)
static IOIF_USB_HostCallback_t s_host_callback = NULL;
#endif

/*
 *-----------------------------------------------------------
 * STATIC PROTOTYPES
 *-----------------------------------------------------------
 */

static void _internal_cdc_tx_complete_callback(void);
static void _internal_cdc_rx_complete_callback(uint8_t *data, uint32_t length);
static void _internal_cdc_dtr_callback(uint8_t dtr);

#if defined(AGRB_IOIF_USB_MODE_DRP)
static void _internal_host_callback(USBH_HandleTypeDef *phost, uint8_t id);
#endif

/*
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — Device (공통)
 *-----------------------------------------------------------
 */

AGRBStatusDef ioif_usb_device_init(IOIF_USB_CDC_TxCallback_t tx_cb,
                                    IOIF_USB_CDC_RxCallback_t rx_cb)
{
    if (g_is_device_initialized)
    {
        return AGRBStatus_OK;
    }

#if defined(AGRB_IOIF_USB_MODE_DRP)
    if (g_is_host_initialized)
    {
        return AGRBStatus_BUSY;
    }
#endif

    /* 공통 prologue — spurious IRQ 방지 */
    HAL_NVIC_DisableIRQ(OTG_FS_EP1_OUT_IRQn);
    HAL_NVIC_DisableIRQ(OTG_FS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    IOIF_USB_SAFE_DELAY_MS(2);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    IOIF_USB_SAFE_DELAY_MS(10);

    s_cdc_tx_callback = tx_cb;
    s_cdc_rx_callback = rx_cb;

    /* COMPOSITE state (s_sd_owner / s_cdc_active) lives in
     * usb_composite_service. No IOIF-side reset needed — the service
     * re-evaluates hw state each poll. */

    /* CRS — H7 HSI48 ±3% → USB FS 허용(±0.25%) 이내로 보정 */
#if defined(STM32H743xx) || defined(STM32H750xx)
    {
        RCC_CRSInitTypeDef crs = {0};
        __HAL_RCC_CRS_CLK_ENABLE();
        crs.Prescaler             = RCC_CRS_SYNC_DIV1;
        crs.Source                = RCC_CRS_SYNC_SOURCE_USB1;
        crs.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
        crs.ReloadValue           = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
        crs.ErrorLimitValue       = 34;
        crs.HSI48CalibrationValue = 32;
        HAL_RCCEx_CRSConfig(&crs);
    }
#endif

    /* Device 초기화 — CubeMX 생성 MX_USB_DEVICE_Init() 에 전적 위임
     * (클래스 등록 / USBD_CDC_RegisterInterface / USBD_Start 모두 포함) */
    MX_USB_DEVICE_Init();

    /* NVIC — 프로젝트 측 hal_msp.c strong override 가 priority 설정을 생략하는
     * 경우가 많아 IOIF 가 명시 설정한 뒤 enable.
     * Device=5 : configMAX_SYSCALL_INTERRUPT_PRIORITY 경계 → FreeRTOS FromISR API 허용.
     *            CubeMX __weak HAL_PCD_MspInit 기본값과 동일. */
#if defined(STM32H743xx) || defined(STM32H750xx)
    HAL_NVIC_SetPriority(OTG_FS_EP1_OUT_IRQn, 5, 0);
    HAL_NVIC_SetPriority(OTG_FS_EP1_IN_IRQn,  5, 0);
    HAL_NVIC_SetPriority(OTG_FS_IRQn,         5, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_EP1_OUT_IRQn);
    HAL_NVIC_EnableIRQ(OTG_FS_EP1_IN_IRQn);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
#endif

    /* CDC 콜백 shim 연결 */
    CDC_Register_Callbacks(_internal_cdc_tx_complete_callback,
                           _internal_cdc_rx_complete_callback);

#ifdef DEBUG
    /* [DEBUG] 공통 레지스터 스냅샷 */
    dbg_usb_gahbcfg  = USB_OTG_FS->GAHBCFG;
    dbg_usb_gintmsk  = USB_OTG_FS->GINTMSK;
    dbg_usb_gintsts  = USB_OTG_FS->GINTSTS;
    dbg_usb_gusbcfg  = USB_OTG_FS->GUSBCFG;
    dbg_usb_nvic_en  = NVIC_GetEnableIRQ(OTG_FS_IRQn);
    dbg_usb_rcc_ahb1 = RCC->AHB1ENR;
    dbg_usb_rcc_cr   = RCC->CR;

  #if defined(AGRB_IOIF_USB_MODE_COMPOSITE)
    {
        extern USBD_HandleTypeDef hUsbDeviceFS;
        dbg_usb_dev_state     = hUsbDeviceFS.dev_state;
        dbg_usb_num_classes   = hUsbDeviceFS.NumClasses;
        dbg_usb_class0_active = hUsbDeviceFS.tclasslist[0].Active;
        dbg_usb_class1_active = hUsbDeviceFS.tclasslist[1].Active;
        dbg_usb_class0_numep  = hUsbDeviceFS.tclasslist[0].NumEps;
        dbg_usb_class1_numep  = hUsbDeviceFS.tclasslist[1].NumEps;
        if (hUsbDeviceFS.pConfDesc != NULL)
        {
            uint8_t *cd = (uint8_t *)hUsbDeviceFS.pConfDesc;
            dbg_usb_cfg_desc_sz = (uint32_t)cd[2] | ((uint32_t)cd[3] << 8);
            for (int i = 0; i < 9; i++) { dbg_usb_cfg_desc_hdr[i] = cd[i]; }
        }
    }
  #endif
#endif /* DEBUG */

    g_is_device_initialized = true;
    return AGRBStatus_OK;
}

AGRBStatusDef ioif_usb_device_deinit(void)
{
    if (!g_is_device_initialized)
    {
        return AGRBStatus_OK;
    }

#if defined(AGRB_IOIF_USB_MODE_COMPOSITE)
    /* MSC media 비활성화 — DeInit 중 SCSI 콜백에서 SD 접근 차단 */
    USBD_Storage_SetMediaActive(false);
    /* Composite: USBD_DeInit 이 내부적으로 USBD_LL_Stop + 클래스 DeInit 수행 */
    USBD_DeInit(&hUsbDeviceFS);
#else
    /* CDC_ONLY / DRP: pre-refactor 패턴 — Stop 명시 호출 */
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
#endif

    s_cdc_tx_callback  = NULL;
    s_cdc_rx_callback  = NULL;
    s_cdc_dtr_callback = NULL;
    CDC_Register_Callbacks(NULL, NULL);
    CDC_Register_DTR_Callback(NULL);

    g_is_device_initialized = false;

    /* COMPOSITE state reset (s_sd_owner / s_cdc_active) now lives in
     * usb_composite_service — no IOIF-side touch on deinit. */

    /* HW 강제 리셋 */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    IOIF_USB_SAFE_DELAY_MS(2);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    IOIF_USB_SAFE_DELAY_MS(10);

    return AGRBStatus_OK;
}

void ioif_usb_device_register_dtr_callback(IOIF_USB_CDC_DtrCallback_t callback)
{
    s_cdc_dtr_callback = callback;
    CDC_Register_DTR_Callback(_internal_cdc_dtr_callback);
}

/*
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — COMPOSITE 전용 (HAL 얇은 어댑터만)
 *-----------------------------------------------------------
 * SD 소유권 state machine, DTR deferred 처리, CDC active 플래그는
 * System/Comm/USB/usb_composite_service.c 로 이전되었다 (Plan v6 Step 3.2).
 * IOIF 는 MSC media-active setter 등 얇은 HAL 래퍼만 제공한다.
 */
#if defined(AGRB_IOIF_USB_MODE_COMPOSITE)

void ioif_usb_set_media_active(bool active)
{
    USBD_Storage_SetMediaActive(active);
}

#endif /* MODE_COMPOSITE */

/*
 *-----------------------------------------------------------
 * PUBLIC FUNCTIONS — DRP 전용 (Host MSC)
 *-----------------------------------------------------------
 */
#if defined(AGRB_IOIF_USB_MODE_DRP)

AGRBStatusDef ioif_usb_host_init(IOIF_USB_HostCallback_t user_callback)
{
    if (g_is_host_initialized)
    {
        return AGRBStatus_OK;
    }
    if (g_is_device_initialized)
    {
        return AGRBStatus_BUSY;
    }

    /* 공통 prologue — Device 와 동일 패턴 */
    HAL_NVIC_DisableIRQ(OTG_FS_EP1_OUT_IRQn);
    HAL_NVIC_DisableIRQ(OTG_FS_EP1_IN_IRQn);
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);

    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    IOIF_USB_SAFE_DELAY_MS(2);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    IOIF_USB_SAFE_DELAY_MS(10);

    s_host_callback = user_callback;

    if (USBH_Init(&hUsbHostFS, _internal_host_callback, HOST_FS) != USBH_OK)
    {
        s_host_callback = NULL;
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
    {
        USBH_DeInit(&hUsbHostFS);
        s_host_callback = NULL;
        return AGRBStatus_INITIAL_FAILED;
    }
    if (USBH_Start(&hUsbHostFS) != USBH_OK)
    {
        USBH_DeInit(&hUsbHostFS);
        s_host_callback = NULL;
        return AGRBStatus_INITIAL_FAILED;
    }

    /* NVIC — Device 와 동일 이유로 IOIF 가 priority 명시 설정 후 enable.
     * Host=6 : configMAX_SYSCALL(5) 보다 낮아 FromISR API 허용. FDCAN(5)
     *          및 Device(5) 보다도 낮아 CAN RX / CDC 타이밍 간섭 최소화.
     *          USBH_OS_PutMessage → osMessageQueuePut 경로에서 필수.
     *          CubeMX __weak HAL_HCD_MspInit 기본값과 동일. */
    HAL_NVIC_SetPriority(OTG_FS_EP1_OUT_IRQn, 6, 0);
    HAL_NVIC_SetPriority(OTG_FS_EP1_IN_IRQn,  6, 0);
    HAL_NVIC_SetPriority(OTG_FS_IRQn,         6, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_EP1_OUT_IRQn);
    HAL_NVIC_EnableIRQ(OTG_FS_EP1_IN_IRQn);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

#ifdef DEBUG
    dbg_usb_gahbcfg  = USB_OTG_FS->GAHBCFG;
    dbg_usb_gintmsk  = USB_OTG_FS->GINTMSK;
    dbg_usb_gintsts  = USB_OTG_FS->GINTSTS;
    dbg_usb_gusbcfg  = USB_OTG_FS->GUSBCFG;
    dbg_usb_hprt     = *(__IO uint32_t *)((uint32_t)USB_OTG_FS + 0x440U);
    dbg_usb_nvic_en  = NVIC_GetEnableIRQ(OTG_FS_IRQn);
    dbg_usb_rcc_ahb1 = RCC->AHB1ENR;
    dbg_usb_rcc_cr   = RCC->CR;
#endif

    g_is_host_initialized = true;
    return AGRBStatus_OK;
}

AGRBStatusDef ioif_usb_host_deinit(void)
{
    if (!g_is_host_initialized)
    {
        return AGRBStatus_OK;
    }

    USBH_DeInit(&hUsbHostFS);
    s_host_callback = NULL;
    g_is_host_initialized = false;

    /* HW 강제 리셋 */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    IOIF_USB_SAFE_DELAY_MS(2);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    IOIF_USB_SAFE_DELAY_MS(10);

    return AGRBStatus_OK;
}

bool ioif_usb_host_is_port_enabled(void)
{
    if (!g_is_host_initialized)
    {
        return false;
    }
    return (USBH_IsPortEnabled(&hUsbHostFS) != 0U);
}

#endif /* MODE_DRP */

/*
 *-----------------------------------------------------------
 * STATIC FUNCTIONS
 *-----------------------------------------------------------
 */

static void _internal_cdc_tx_complete_callback(void)
{
    if (s_cdc_tx_callback != NULL)
    {
        s_cdc_tx_callback();
    }
}

static void _internal_cdc_rx_complete_callback(uint8_t *data, uint32_t length)
{
    if (s_cdc_rx_callback != NULL)
    {
        s_cdc_rx_callback(data, length);
    }
}

/**
 * @brief  DTR 콜백 (ISR context)
 * @details
 * - COMPOSITE: mutex 불가 → Service 로 forward (Service 가 deferred 처리)
 * - CDC_ONLY/DRP: 앱 콜백 직접 호출 (deferred 상태 머신 없음)
 */
static void _internal_cdc_dtr_callback(uint8_t dtr)
{
#if defined(AGRB_IOIF_USB_MODE_COMPOSITE)
    UsbService_OnDtrEventFromIsr(dtr);
#endif

    if (s_cdc_dtr_callback != NULL)
    {
        s_cdc_dtr_callback(dtr);
    }
}

#if defined(AGRB_IOIF_USB_MODE_DRP)
static void _internal_host_callback(USBH_HandleTypeDef *phost, uint8_t id)
{
    (void)phost;
    if (s_host_callback != NULL)
    {
        s_host_callback(id);
    }
}
#endif

#endif /* AGRB_IOIF_USB_ENABLE */
