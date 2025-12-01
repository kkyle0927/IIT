

#ifndef IOIF_COMMON_INC_IOIF_USB_COMMON_H_
#define IOIF_COMMON_INC_IOIF_USB_COMMON_H_

#include "bsp_tim.h"
#include "bsp_gpio.h"

#include "bsp_usb_fs.h"
#include "usbd_cdc_if.h"

#include "ioif_gpio_common.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS BSP module driver
  * @{
  */
#ifdef BSP_USB_OTG_FS_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/*
typedef int (*IOIF_USBCBPtr_t)	(uint8_t* buf, uint32_t* len);

extern IOIF_USBCBPtr_t ioif_usbRxCBPtr;
*/


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_USBState {
	IOIF_USB_OK,
	IOIF_USB_FAIL,
} IOIF_USBState;

typedef enum _IOIF_USB_Tim_t {
    IOIF_USB_TIM1 = BSP_TIM1,  ///< Timer 1 Identifier
	IOIF_USB_TIM2 = BSP_TIM2,      ///< Timer 2 Identifier
	IOIF_USB_TIM3 = BSP_TIM3,      ///< Timer 3 Identifier
	IOIF_USB_TIM4 = BSP_TIM4,      ///< Timer 4 Identifier
	IOIF_USB_TIM5 = BSP_TIM5,      ///< Timer 5 Identifier
	IOIF_USB_TIM6 = BSP_TIM6,      ///< Timer 6 Identifier
	IOIF_USB_TIM7 = BSP_TIM7,      ///< Timer 7 Identifier
	IOIF_USB_TIM8 = BSP_TIM8,      ///< Timer 8 Identifier
	IOIF_USB_TIM9 = BSP_TIM9,      ///< Timer 9 Identifier
	IOIF_USB_TIM10 = BSP_TIM10,     ///< Timer 10 Identifier
	IOIF_USB_TIM11 = BSP_TIM11,     ///< Timer 11 Identifier
	IOIF_USB_TIM12 = BSP_TIM12,     ///< Timer 12 Identifier
	IOIF_USB_TIM13 = BSP_TIM13,     ///< Timer 13 Identifier
	IOIF_USB_TIM14 = BSP_TIM14,     ///< Timer 14 Identifier
	IOIF_USB_TIM15 = BSP_TIM15,     ///< Timer 15 Identifier
	IOIF_USB_TIM16 = BSP_TIM16,     ///< Timer 16 Identifier
	IOIF_USB_TIM17 = BSP_TIM17,     ///< Timer 17 Identifier
	IOIF_USB_TIM_NULL = BSP_TIM_COUNT // for non-timer
} IOIF_USB_Tim_t;

typedef enum _IOIF_UsbState_t {
    IOIF_USB_PIN_RESET = BSP_GPIO_PIN_RESET,
    IOIF_USB_PIN_SET   = BSP_GPIO_PIN_SET
} IOIF_UsbState_t;


typedef enum _IOIF_USB_MODE {
	IOIF_USB_NONE = BSP_USB_NONE,
	IOIF_USBD_CDC  = BSP_USBD_CDC,
	IOIF_USBH_MSC  = BSP_USBH_MSC,
} IOIF_USB_MODE;


/* USB GPIOs (VBUS, USB_DP_Pull-up) definitions */

#ifdef L30_CM_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_15,		//will be changed
	IOIF_USB_PWR_ON_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_PWR_ON_PIN	  = BSP_GPIO_PIN_6,
	IOIF_USB_OC_DET_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_OC_DET_PIN   = BSP_GPIO_PIN_7,
} IOIF_USB_GPIO;
#endif

#ifdef SUIT_MINICM_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_8,
	IOIF_USB_PWR_ON_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_PWR_ON_PIN	  = BSP_GPIO_PIN_6,
	IOIF_USB_OC_DET_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_OC_DET_PIN   = BSP_GPIO_PIN_7,
} IOIF_USB_GPIO;
#endif

#ifdef WALKON5_CM_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_3,			//will be changed
	IOIF_USB_PWR_ON_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_PWR_ON_PIN	  = BSP_GPIO_PIN_6,
	IOIF_USB_OC_DET_PORT  = BSP_GPIO_PORT_D,
	IOIF_USB_OC_DET_PIN   = BSP_GPIO_PIN_7,
} IOIF_USB_GPIO;
#endif

#ifdef L30_MD_REV06_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_8,		//will be changed
} IOIF_USB_GPIO;
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_15,		//will be changed
} IOIF_USB_GPIO;
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_15,		//will be changed
} IOIF_USB_GPIO;
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED
typedef enum _IOIF_USB_GPIO {
	IOIF_USB_VBUS_PORT 	  = BSP_GPIO_PORT_A,
	IOIF_USB_VBUS_PIN	  = BSP_GPIO_PIN_9,
	IOIF_USB_GPIO_PU_PORT = BSP_GPIO_PORT_A,
	IOIF_USB_GPIO_PU_PIN  = BSP_GPIO_PIN_8,		//will be changed
} IOIF_USB_GPIO;
#endif /* SUIT_MD_ENABLED */


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

//extern IOIF_USBCBPtr_t ioif_usbRxCBPtr;
//IOIF_USBCBPtr_t ioif_usbReadCBPrt = NULL;



/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void 				IOIF_SetUSBDetCB(void);
IOIF_USBState 		IOIF_InitUSB(IOIF_USB_MODE mode, IOIF_USB_Tim_t timer);
IOIF_USB_MODE 		IOIF_IsUSBMode(void);

/* USB Device Mode Functions */

uint32_t 			IOIF_USBD_BufferIsAvailable(void);
bool 	 			IOIF_USBD_BufferByteRead(uint8_t *pData, uint32_t length);
USBRXDataStruct_t*  IOIF_USBD_ReadCB(USBRXDataStruct_t* USBRXData_t);
uint8_t  			IOIF_USBD_Write(uint8_t* pData, uint32_t length, uint32_t timeout_ms);
uint8_t  			IOIF_USBD_Printf(uint8_t* data, ...);
bool 				IOIF_IsUSBD_connected(void);

/* USB Host Mode Functions */
bool				IOIF_IsUSBH_OverCurrentDetected(void);
bool 				IOIF_USBH_PWREN(void);

#if defined(WALKON5_CM_ENABLED)
USBRXDataStruct_t* IOIF_USBReadCB(USBRXDataStruct_t* USBRXData_t);
#endif


#endif /* BSP_USB_OTG_FS_MODULE_ENABLED */

#endif /* IOIF_COMMON_INC_IOIF_USB_COMMON_H_ */
