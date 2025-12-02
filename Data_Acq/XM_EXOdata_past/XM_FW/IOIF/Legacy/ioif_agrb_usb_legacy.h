#if !defined(AGRB_IOIF_USB_DISABLE)
#ifndef _IOIF_AGRB_USB_H_
#define _IOIF_AGRB_USB_H_

#include "ioif_agrb_defs.h"
#include "usb_host.h"
#include "usb_device.h"

#include "ioif_agrb_fs.h"

#define IOIF_USB_DEVICE_MANUFACTURER_STRING_MAX_LENGTH   (32)
#define  IOIF_USB_DEVICE_PRODUCT_STRING_MAX_LENGTH       (32)

#define IOIF_USB_DELAY_FOR_STABLIZE_MS    (100) //100ms

typedef enum {
    IOIF_USB_Status_Disconnected = 0,
    IOIF_USB_Status_Connected,
    IOIF_USB_Status_FaultHandling,
    IOIF_USB_Status_NotInitialized,
    
} IOIF_USB_Status_e;

typedef enum {
    IOIF_USB_Host_SupportedClass_MSC = 0x8, //Mass Storage Class

    IOIF_USB_Host_SupportedClass_UNKNOWN = 0xFF,

} IOIF_USB_Host_SupportedClass_e;

typedef enum {
    IOIF_USB_Device_SupportedClass_CDC = 0, //Communication Device Class

    IOIF_USB_Device_SupportedClass_UNKNOWN = 0xFF,
    
} IOIF_USB_Device_SupportedClass_e; 

typedef enum {
    IOIF_USB_Device_English,
    IOIF_USB_Device_Korean,
    IOIF_USB_Device_French,
    IOIF_USB_Device_German,
    IOIF_USB_Device_Italian,
    IOIF_USB_Device_Japanese,

} IOIF_USB_Device_SupportLanguage_e;

typedef struct {
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t manufacturer_string[IOIF_USB_DEVICE_MANUFACTURER_STRING_MAX_LENGTH];
    uint8_t product_string[IOIF_USB_DEVICE_PRODUCT_STRING_MAX_LENGTH];
    IOIF_USB_Device_SupportLanguage_e language;    

    IOIF_USB_Device_SupportedClass_e supported_class;

} IOIF_USB_Device_Initialize_t;

bool ioif_usb_ready_for_dual_role(void);

AGRBStatusDef ioif_usb_host_initialize(IOIF_USB_Host_SupportedClass_e class);
AGRBStatusDef _usb_host_subroutine_init(IOIF_USB_Host_SupportedClass_e class);
bool ioif_usb_host_deinitialize(void);
IOIF_USB_Host_SupportedClass_e ioif_usb_get_current_host_class(void);

AGRBStatusDef ioif_usb_device_initialize(IOIF_USB_Device_Initialize_t* init);
bool ioif_usb_device_deinitialize(void);

bool ioif_usb_get_port_enabled(void);


AGRBStatusDef ioif_usb_cdc_send_data(uint8_t* data, uint32_t length);
AGRBStatusDef ioif_usb_cdc_receive_data(uint8_t* data, uint32_t max_length, uint32_t* received_length);


#endif // _IOIF_AGRB_USBH_H_
#endif // AGRB_IOIF_USBH_DISABLE