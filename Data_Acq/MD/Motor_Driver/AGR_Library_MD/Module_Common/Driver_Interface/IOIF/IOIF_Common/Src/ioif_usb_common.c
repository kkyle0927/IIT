

#include "ioif_usb_common.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS BSP module driver
  * @{
  */
#ifdef BSP_USB_OTG_FS_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

volatile bool IsUSBHOverCurrentDet = false;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

//static IOIF_USB_Tim_t usb_timer_source;
IOIF_USB_Tim_t usb_timer_source;
static IOIF_USB_MODE usb_mode = IOIF_USB_NONE;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void USBDVBUSDetCB(uint16_t gpioPin);
#ifdef CM_MODULE
static void USBHOCDetCB(uint16_t gpioPin);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

void IOIF_SetUSBDetCB(void)
{
    BSP_SetGPIOCB(IOIF_USB_VBUS_PIN, BSP_GPIO_EXTI_CALLBACK, USBDVBUSDetCB);
}

void IOIF_SetUSBOCDetCB(void)
{
#ifdef CM_MODULE
    BSP_SetGPIOCB(IOIF_USB_OC_DET_PIN, BSP_GPIO_EXTI_CALLBACK, USBHOCDetCB);
#endif
}


IOIF_USBState IOIF_InitUSB(IOIF_USB_MODE mode, IOIF_USB_Tim_t timer)
{
	usb_timer_source = timer;		//USB timer source init.
	IOIF_USBState ret = IOIF_USB_OK;

	switch(mode){

	case IOIF_USBD_CDC:
		IOIF_SetUSBDetCB();			// Set USB Device Vbus Detection Callback
//		IOIF_SetGPIOCB(MCU_USB_FS_VBUS_Pin, IOIF_GPIO_EXTI_CALLBACK, USBDVBUSDetCB);
		if(BSP_USBInit(BSP_USBD_CDC) != true)
			ret = IOIF_USB_FAIL;
		usb_mode = IOIF_USBD_CDC;
		
//#if defined(WALKON5_CM_ENABLED)
//		USB_SetRXCB(IOIF_USBReadCB);	// Register USB CDC Read Callback
//#endif

		break;
	case IOIF_USBH_MSC:
		IOIF_SetUSBOCDetCB();
		IOIF_USBH_PWREN();				// USB 5V Power Enable
		if(BSP_USBInit(BSP_USBH_MSC) != true)
			ret = IOIF_USB_FAIL;
		usb_mode = IOIF_USBH_MSC;
		break;

	default:
		break;
	}

    //BSP_USBInit(mode);				// BSP USB init.

	return ret;
}

uint32_t IOIF_USBD_BufferIsAvailable(void)
{
	uint32_t length = 0;

	return length = BSP_USBCDCBufferIsAvailable();
}


bool IOIF_USBD_BufferByteRead(uint8_t *pData, uint32_t length)
{
	bool ret = false;

	return ret = BSP_USBCDCBufferRead(pData, length);
}

#if defined(WALKON5_CM_ENABLED)
USBRXDataStruct_t* IOIF_USBReadCB(USBRXDataStruct_t* USBRXData_t)
{
	return USBRXData_t;
}
#endif 


uint8_t IOIF_USBD_Write(uint8_t* pData, uint32_t length, uint32_t timeout_ms)
{
	USBD_StatusTypeDef usb_tx_ret = USBD_BUSY;

	uint8_t ret = 0;
	uint8_t pre_time = 0;
	uint32_t current_time = 0;

	/* USB port is disconnected */
	if(IOIF_IsUSBD_connected() != true)
		return ret = USBD_FAIL;

	/* USB Timeout Source */
	if(usb_timer_source != IOIF_USB_TIM_NULL)
		pre_time = BSP_GetCnt(usb_timer_source);		// get current timer counter (non-blocking code)
	else pre_time = BSP_GetTick();						// use get systick (OS supported)

	/* Infinite loop to complete USB transmit */
	while(usb_tx_ret) {
		if(BSP_USBCDCWrite(pData, length) == USBD_OK)
			break;

		if(usb_timer_source != IOIF_USB_TIM_NULL)
			current_time = BSP_GetCnt(usb_timer_source);
		else current_time = BSP_GetTick();

		if(current_time - pre_time > timeout_ms) {
			ret = USBD_FAIL;
			break;
		}
	}
	return ret;
}

uint8_t IOIF_USBD_Printf(uint8_t* data, ...)			// Variable Arguments
{
	uint8_t msg_buf[512];

	uint8_t ret;
	int32_t length;
	uint32_t timeout_ms = 100;

	va_list args;

	va_start(args, data);

	length = vsnprintf((char*)msg_buf, 512, (char*)data, args);
	ret = IOIF_USBD_Write((uint8_t*)msg_buf, (int32_t)length, timeout_ms);

	va_end(args);

	return ret;
}

bool IOIF_IsUSBD_connected(void)
{
	bool ret = false;

	if(BSP_ReadGPIOPin(IOIF_USB_VBUS_PORT, IOIF_USB_VBUS_PIN) == BSP_GPIO_PIN_SET)
		ret = true;

	return ret;
}

bool IOIF_IsUSBH_OverCurrentDetected(void)
{
	return IsUSBHOverCurrentDet;
}

bool IOIF_USBH_PWREN(void)
{
	bool ret = true;

#ifdef CM_MODULE
	BSP_WriteGPIOPin(IOIF_USB_PWR_ON_PORT, IOIF_USB_PWR_ON_PIN, BSP_GPIO_PIN_SET);
#endif

	return ret;
}

IOIF_USB_MODE IOIF_IsUSBMode(void)
{
	return usb_mode;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void USBDVBUSDetCB(uint16_t gpioPin)
{
	/* Device mode : USB VBUS is powered on, */
//	if(gpioPin == MCU_USB_FS_VBUS_Pin)
	if(gpioPin == IOIF_USB_VBUS_PIN)
	{
		if(BSP_ReadGPIOPin(IOIF_USB_VBUS_PORT, IOIF_USB_VBUS_PIN) == BSP_GPIO_PIN_SET)
			BSP_WriteGPIOPin(IOIF_USB_GPIO_PU_PORT, IOIF_USB_GPIO_PU_PIN, IOIF_USB_PIN_SET);
		else
			BSP_WriteGPIOPin(IOIF_USB_GPIO_PU_PORT, IOIF_USB_GPIO_PU_PIN, IOIF_USB_PIN_RESET);
	}
}

#ifdef CM_MODULE
static void USBHOCDetCB(uint16_t gpioPin)
{
	/* Host mode : USB Over-current is detected, */
	if(gpioPin == IOIF_USB_OC_DET_PIN && BSP_ReadGPIOPin(IOIF_USB_OC_DET_PORT, IOIF_USB_OC_DET_PIN) == BSP_GPIO_PIN_RESET)
	{
		BSP_WriteGPIOPin(IOIF_USB_PWR_ON_PORT, IOIF_USB_PWR_ON_PIN, BSP_GPIO_PIN_RESET);
		IsUSBHOverCurrentDet = true;
	}

	/* USB over-current flag is clear, */
	if(gpioPin == IOIF_USB_OC_DET_PIN && BSP_ReadGPIOPin(IOIF_USB_OC_DET_PORT, IOIF_USB_OC_DET_PIN) == BSP_GPIO_PIN_SET)
	{
		BSP_WriteGPIOPin(IOIF_USB_PWR_ON_PORT, IOIF_USB_PWR_ON_PIN, BSP_GPIO_PIN_SET);	//USB Power Recovered,
		IsUSBHOverCurrentDet = false;
	}
}
#endif

#endif /* BSP_USB_OTG_FS_MODULE_ENABLED */
