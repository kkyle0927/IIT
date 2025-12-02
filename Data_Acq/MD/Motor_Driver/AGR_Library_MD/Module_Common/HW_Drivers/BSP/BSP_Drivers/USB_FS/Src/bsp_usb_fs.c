/**
 * @file bsp_usb_otg_fs.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for USB OTG FS functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "bsp_usb_fs.h"

/** @defgroup USB_OTG_FS USB_OTG_FS
  * @brief USB_OTG_FS HAL BSP module driver
  * @{
  */
#ifdef USB_OTG_FS

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

USBD_HandleTypeDef hUsbDeviceFS;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool BSP_USBInit(BSP_USB_MODE mode)
{
	bool ret= true;

	switch(mode){
	case BSP_USBD_CDC:
		/* Copy to MX USB init. code */

		if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
			ret = false;
		}
		if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
			ret = false;
		}
		if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
			ret = false;
		}
		if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
			ret = false;
		}

		//SCB_InvalidateDCache_by_Addr((USBD_HandleTypeDef*)&hUsbDeviceFS, sizeof(hUsbDeviceFS));

		/* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
		HAL_PWREx_EnableUSBVoltageDetector();

		break;

	case BSP_USBH_MSC:
		/* Copy to MX USB init. code */
#ifdef CM_MODULE
		if(USB_HOST_Mode_Init() != true)
			ret = false;
		break;
#endif

	default :
		break;
	}

	return ret;
}


bool BSP_USBCDCDeinit(void)
{
	bool ret = true;

	if(USBD_DeInit(&hUsbDeviceFS) == USBD_FAIL)
		return ret = false;

	return ret;
}


uint32_t BSP_USBCDCBufferIsAvailable(void)
{
	uint32_t length = 0;

	length = RingBufferIsAvailable(usbCDCringbuff);

	return length;
}


bool BSP_USBCDCBufferRead(uint8_t* data, uint32_t length)
{
	bool ret = false;

	if(RingBufferPop(usbCDCringbuff, data, length) == true)
		ret = true;

	return ret;
}


uint8_t BSP_USBCDCWrite(uint8_t* pData, uint32_t length)
{
	USBD_StatusTypeDef ret = USBD_BUSY;
	/* USBD Status Type Definition : 0 - USBD_OK, 1- USBD_BUSY, 3- USBD_FAIL */
	return ret = CDC_Transmit_FS(pData, length);
}

//
//uint8_t BSP_USBCDCWrite(TIM_HandleTypeDef *ptimerSource, uint8_t* data, uint32_t length, uint32_t timeout_ms)
//{
//	uint8_t ret = 0;
//	uint32_t pre_time = 0;
//	uint32_t current_time = 0;
//
//	USBD_StatusTypeDef usb_tx_ret = USBD_BUSY;
//	/* USBD Status Type Definition : 0 - USBD_OK, 1- USBD_BUSY, 3- USBD_FAIL */
//
//	//pre_time = HAL_GetTick();
//	//pre_time = BSP_GetTick();
//	pre_time = __HAL_TIM_GET_COUNTER(ptimerSource);
//
//	while(usb_tx_ret)								// infinite loop to complete transmit
//	{
//		usb_tx_ret = CDC_Transmit_FS(data, length);
//
//		current_time = __HAL_TIM_GET_COUNTER(ptimerSource);
//		if(current_time - pre_time > timeout_ms)
//		//if(BSP_GetTick() - pre_time > timeout_ms)
//		{
//			ret = USBD_FAIL;
//			break;
//		}
//	}
//	return ret;
//}

/*
uint8_t BSP_USBCDCPrintf(uint8_t* data, ...)			// Variable Arguments
{

	uint8_t msg_buf[512];

	uint8_t ret;
	int32_t length;
	uint32_t timeout_ms = 100;

	va_list args;

	va_start(args, data);

	length = vsnprintf((char*)msg_buf, 512, (char*)data, args);
	ret = BSP_USBCDCWrite((uint8_t*)msg_buf, (int32_t)length, timeout_ms);

	va_end(args);

	return ret;
}
*/


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */



#endif /* USB_OTG_FS */
