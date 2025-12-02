#include "ioif_agrb_usb.h"
#if defined(AGRB_IOIF_USB_ENABLE)
//기존 usb_host.h 는 참조용으로만 사용되며, 실제 소스코드는 좀 더 우리에게 알맞게 구성해야 한다
//#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#if defined(USE_FREERTOS_DMA)
#include "FreeRTOS.h"
#include "semphr.h"

//CDC는 TX, RX 각각 세마포어 필요

SemaphoreHandle_t __cdc_tx_semaphore = NULL;
SemaphoreHandle_t __cdc_rx_semaphore = NULL;

#define CDC_CREATE_SEMAPHORES()   do {                           \
    if (__cdc_tx_semaphore == NULL) {                                 \
        __cdc_tx_semaphore = xSemaphoreCreateBinary();                \
        if (__cdc_tx_semaphore != NULL) {                             \
            xSemaphoreGive(__cdc_tx_semaphore);                       \
        } else {                                                      \
            return AGRBStatus_SEMAPHORE_ERROR;                        \
        }                                                             \
    }                                                                 \
    if (__cdc_rx_semaphore == NULL) {                                 \
        __cdc_rx_semaphore = xSemaphoreCreateBinary();                \
        if (__cdc_rx_semaphore != NULL) {                             \
            xSemaphoreGive(__cdc_rx_semaphore);                       \
        } else {                                                      \
            if (__cdc_tx_semaphore != NULL) {                         \
                vSemaphoreDelete(__cdc_tx_semaphore);                 \
                __cdc_tx_semaphore = NULL;                            \
            }                                                         \
            return AGRBStatus_SEMAPHORE_ERROR;                        \
        }                                                             \
    }                                                                 \
} while(0)

#define CDC_TX_SEMAPHORE_ACQUIRE()   do {                           \
    if (__cdc_tx_semaphore != NULL) {                                 \
        if ( xSemaphoreTake(__cdc_tx_semaphore, pdMS_TO_TICKS(IOIF_USB_DELAY_FOR_STABLIZE_MS)) != pdTRUE ) { \
            return AGRBStatus_TIMEOUT;                              \
        }                                                           \
    } else {                                                        \
        return AGRBStatus_NOT_INITIALIZED;                          \
    }                                                               \
} while(0)

#define CDC_TX_SEMAPHORE_RELEASE()  do {                           \
    if (__cdc_tx_semaphore != NULL) {                                 \
        if ( xSemaphoreGive(__cdc_tx_semaphore) != pdTRUE ) {        \
            return AGRBStatus_SEMAPHORE_ERROR;                      \
        }                                                           \
    } else {                                                        \
        return AGRBStatus_NOT_INITIALIZED;                          \
    }                                                               \
} while(0)

#define CDC_TX_SEMAPHORE_RELEASE_ISR()   do {                           \
    if (__cdc_tx_semaphore != NULL) {                                 \
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;                  \
        if ( xSemaphoreGiveFromISR(__cdc_tx_semaphore,                   \
            &xHigherPriorityTaskWoken) != pdTRUE ) {                    \
            return AGRBStatus_SEMAPHORE_ERROR;                          \
        }                                                               \
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                   \
    } else {                                                            \
        return AGRBStatus_NOT_INITIALIZED;                              \
    }                                                                   \
} while(0)

#define CDC_RX_SEMAPHORE_ACQUIRE()   do {                           \
    if (__cdc_rx_semaphore != NULL) {                                 \
        if ( xSemaphoreTake(__cdc_rx_semaphore, pdMS_TO_TICKS(IOIF_USB_DELAY_FOR_STABLIZE_MS)) != pdTRUE ) { \
            return AGRBStatus_TIMEOUT;                              \
        }                                                           \
    } else {                                                        \
        return AGRBStatus_NOT_INITIALIZED;                          \
    }                                                               \
} while(0)

#define CDC_RX_SEMAPHORE_RELEASE()   do {                           \
    if (__cdc_rx_semaphore != NULL) {                                 \
        if ( xSemaphoreGive(__cdc_rx_semaphore) != pdTRUE ) {        \
            return AGRBStatus_SEMAPHORE_ERROR;                      \
        }                                                           \
    } else {                                                        \
        return AGRBStatus_NOT_INITIALIZED;                          \
    }                                                               \
} while(0)

#else

#define CDC_CREATE_SEMAPHORES()
#define CDC_TX_SEMAPHORE_ACQUIRE()
#define CDC_TX_SEMAPHORE_RELEASE()
#define CDC_TX_SEMAPHORE_RELEASE_ISR()
#define CDC_RX_SEMAPHORE_ACQUIRE()
#define CDC_RX_SEMAPHORE_RELEASE()

#endif

#if defined(AGRB_IOIF_FILESYSTEM_ENABLE)
#include "ioif_agrb_fs.h"
#endif
 
typedef enum {
    USB_Mode_IDLE = 0,
    USB_Mode_Host,
    USB_Mode_Device,
} USB_Mode_e;

static USB_Mode_e _usb_current_mode = USB_Mode_IDLE;
static IOIF_USB_Host_SupportedClass_e _usb_current_host_class = IOIF_USB_Host_SupportedClass_UNKNOWN;
static IOIF_USB_Device_SupportedClass_e _usb_current_device_class = IOIF_USB_Device_SupportedClass_CDC; //현재는 CDC 고정
/*********************************** */
extern USBH_HandleTypeDef hUsbHostFS;
extern ApplicationTypeDef Appli_state;
/*********************************** */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
/*********************************** */
 

static void _usb_host_background_process(USBH_HandleTypeDef *phost, uint8_t id);
//static AGRBStatusDef _usb_host_subroutine_init(IOIF_USB_Host_SupportedClass_e class);
static AGRBStatusDef _usb_host_subroutine_deinit(IOIF_USB_Host_SupportedClass_e class);
static bool _usb_host_register_class(IOIF_USB_Host_SupportedClass_e class);

/********************************** */
static void _usb_device_cdc_tx_complete_callback(void* param);

/********************************** */


AGRBStatusDef ioif_usb_host_initialize(IOIF_USB_Host_SupportedClass_e class)
{
    if ( _usb_current_mode != USB_Mode_IDLE ) return AGRBStatus_NOT_ALLOWED; //이미 Host 또는 Device 모드인 경우
    
    bool init_success = false;

    do {        
        //MX_USB_HOST_Init();

        if (USBH_Init(&hUsbHostFS, _usb_host_background_process, HOST_FS) != USBH_OK) break;
        if ( _usb_host_register_class(class) == false ) break;
        if (USBH_Start(&hUsbHostFS) != USBH_OK) break;    
//
        init_success = true;
    } while(0);
    
    if ( init_success == false ) {
        ioif_usb_host_deinitialize();
        return AGRBStatus_INITIAL_FAILED;
    }
     
    vTaskDelay(pdMS_TO_TICKS(IOIF_USB_DELAY_FOR_STABLIZE_MS)); //초기화 후 약간의 안정화 시간 필요

    //if ( !_usb_host_subroutine_init(class) ) return AGRBStatus_INITIAL_FAILED;

    _usb_current_mode = USB_Mode_Host;
    //_usb_current_host_class = class;

    return AGRBStatus_OK;
}

static uint32_t _debug_host_deinit_counter;
static uint32_t _debug_deinit_stage = 100;
static USBH_StatusTypeDef _debug_deinit_status = USBH_ERROR_SPEED_UNKNOWN;
bool ioif_usb_host_deinitialize(void)
{
    //// 이미 Host 모드가 아닐 경우 무시
    if ( _usb_current_mode != USB_Mode_Host ) return false;
 
    _debug_host_deinit_counter++;

    _usb_current_mode = USB_Mode_IDLE; 

    // USBH_Stop(&hUsbHostFS);  

    //todo: Host 모드 관련 추가 정리 작업 필요
    switch (_usb_current_host_class)
    {
        case IOIF_USB_Host_SupportedClass_MSC:
            AGRBFileSystem.deinit();
            break;
        default:
            break;
    }    

    _debug_deinit_status = USBH_DeInit(&hUsbHostFS); 

    _usb_current_host_class = IOIF_USB_Host_SupportedClass_UNKNOWN;

    return true;
}

//많이 빠져있음에 주의 지금은 CDC만 지원
AGRBStatusDef ioif_usb_device_initialize(IOIF_USB_Device_Initialize_t* init)
{
    if ( _usb_current_mode != USB_Mode_IDLE ) return AGRBStatus_NOT_ALLOWED; //이미 Host 또는 Device 모드인 경우

    bool init_success = false;

    // USBD Init
    // USBD_RegisterClass
    do {       
        CDC_CREATE_SEMAPHORES();

        if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) break;
        if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) break;
        if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) break;
        if (USBD_Start(&hUsbDeviceFS) != USBD_OK) break;
        //MX_USB_DEVICE_Init();

        init_success = true;

    } while(0);

    if ( init_success == false ) {
        ioif_usb_device_deinitialize();
        return AGRBStatus_INITIAL_FAILED;
    }

    //전송 완료 콜백 함수 등록
    CDC_Assign_Tx_Complete_Callback(_usb_device_cdc_tx_complete_callback);

    vTaskDelay(pdMS_TO_TICKS(IOIF_USB_DELAY_FOR_STABLIZE_MS)); //초기화 후 약간의 안정화 시간 필요

    _usb_current_mode = USB_Mode_Device;    
    _usb_current_device_class = IOIF_USB_Device_SupportedClass_CDC; //현재는 CDC 고정

    return AGRBStatus_OK;
}

bool ioif_usb_ready_for_dual_role(void)
{
    //Dual Role에 사용되는 기초 핸들러를 초기화한다.
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) return false;
    if (USBH_Init(&hUsbHostFS, _usb_host_background_process, HOST_FS) != USBH_OK) return false;

    return true;
}

bool ioif_usb_device_deinitialize(void)
{
    if (_usb_current_mode != USB_Mode_Device) return false;

    _usb_current_mode = USB_Mode_IDLE;

    switch( _usb_current_device_class )
    {
        case IOIF_USB_Device_SupportedClass_CDC:
            break;
    }

    _debug_deinit_stage = USBD_DeInit(&hUsbDeviceFS);

    _usb_current_device_class = IOIF_USB_Device_SupportedClass_UNKNOWN;

    return;
}

IOIF_USB_Host_SupportedClass_e ioif_usb_get_current_host_class(void)
{
    if ( _usb_current_mode != USB_Mode_Host ) return (IOIF_USB_Host_SupportedClass_e)IOIF_USB_Host_SupportedClass_UNKNOWN;
    
    if( hUsbHostFS.pActiveClass == NULL ) return (IOIF_USB_Host_SupportedClass_e)IOIF_USB_Host_SupportedClass_UNKNOWN;

    switch( hUsbHostFS.pActiveClass->ClassCode )
    {
        case IOIF_USB_Host_SupportedClass_MSC:
            return (IOIF_USB_Host_SupportedClass_e)hUsbHostFS.pActiveClass->ClassCode;
        default:
            return (IOIF_USB_Host_SupportedClass_e)IOIF_USB_Host_SupportedClass_UNKNOWN;
    }
}

bool ioif_usb_get_port_enabled(void)
{
    switch(_usb_current_mode)
    {
        case USB_Mode_Host:
            return (USBH_IsPortEnabled(&hUsbHostFS) != 0U);
        case USB_Mode_Device:
            //todo
            return false;
        default:
            return false;
    }
}
 
uint32_t _debug_host_process_counter = 0;
void _usb_host_background_process(USBH_HandleTypeDef *phost, uint8_t id)
{    
    _debug_host_process_counter++;
    switch(id)
    {
        case HOST_USER_SELECT_CONFIGURATION:
        break;

        case HOST_USER_DISCONNECTION:
        Appli_state = APPLICATION_DISCONNECT;
        // [추가] 파일 시스템에 비활성화 이벤트 전달
        ioif_filesystem_HandleHostEvent(false);
        break;

        case HOST_USER_CLASS_ACTIVE:
        Appli_state = APPLICATION_READY;
        // [추가] 파일 시스템에 활성화 이벤트 전달
        ioif_filesystem_HandleHostEvent(true);
        break;

        case HOST_USER_CONNECTION:
        Appli_state = APPLICATION_START;
        break;

        default:
        break;
    }
}

static bool _usb_host_register_class(IOIF_USB_Host_SupportedClass_e class)
{
    switch( class )
    {
        case IOIF_USB_Host_SupportedClass_MSC:
            if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK) 
                return false;
            break;
        default:
            return false;
    }
    
    return true;
}

static AGRBStatusDef _debug_host_subroutine = AGRBStatus_OK;

AGRBStatusDef _usb_host_subroutine_init(IOIF_USB_Host_SupportedClass_e class)
{
    switch( class )
    {
        case IOIF_USB_Host_SupportedClass_MSC:
        {
            //if ( ( _debug_host_subroutine = IOIF_FILESYSTEM_INITIALIZE(IOIF_FileSystem_DeviceType_USBH_MSC) ) != AGRBStatus_OK )
            //    return false;
            if ( AGRBFileSystem.init() != IOIF_FileSystem_OK )
                return false;
        } break;
        default:
        // 현재 class가 초기화됨
            switch( _usb_current_host_class )
            {
                case IOIF_USB_Host_SupportedClass_MSC:
                    AGRBFileSystem.deinit();
                    break;
                default:
                    break;
            }

            return false;
    }

    _usb_current_host_class = class;
    
    return true;    
}

AGRBStatusDef ioif_usb_cdc_send_data(uint8_t* data, uint32_t length)
{
    if ( _usb_current_mode != USB_Mode_Device ) return AGRBStatus_NOT_ALLOWED;
    if ( _usb_current_device_class != IOIF_USB_Device_SupportedClass_CDC ) return AGRBStatus_NOT_ALLOWED;

    if (length == 0 || length > APP_TX_DATA_SIZE) return AGRBStatus_BUFFER_OVERFLOW; //길이 초과

    CDC_TX_SEMAPHORE_ACQUIRE();
    {
        if ( CDC_Transmit_FS(data, length) != USBD_OK ) {
            CDC_TX_SEMAPHORE_RELEASE();
            return AGRBStatus_ERROR; //전송 실패
        }
    }
    //CDC_TX_SEMAPHORE_RELEASE(); //전송 완료 인터럽트에서 해제
    
    return AGRBStatus_OK;
}

AGRBStatusDef ioif_usb_cdc_receive_data(uint8_t* data, uint32_t max_length, uint32_t* received_length)
{
    if ( _usb_current_mode != USB_Mode_Device ) return AGRBStatus_NOT_ALLOWED;
    if ( _usb_current_device_class != IOIF_USB_Device_SupportedClass_CDC ) return AGRBStatus_NOT_ALLOWED;

    if (max_length == 0 || max_length > APP_RX_DATA_SIZE) return AGRBStatus_BUFFER_OVERFLOW; //길이 초과

    CDC_RX_SEMAPHORE_ACQUIRE();
    {
        uint32_t actual_length = CDC_Receive_Rx_Data(data, max_length);
        if ( received_length != NULL ) *received_length = actual_length;        
    }
    CDC_RX_SEMAPHORE_RELEASE();

    return AGRBStatus_OK;
}

//전송 완료 콜백 함수
static inline void _usb_device_cdc_tx_complete_callback(void* param)
{
    UNUSED(param);
    CDC_TX_SEMAPHORE_RELEASE_ISR();
}

#endif // AGRB_IOIF_USB_ENABLE
