/**
 *-----------------------------------------------------------
 *             System Control : LED & Power IC control
 *-----------------------------------------------------------
 * @file ioif_sysctrl.c
 * @date Created on: Dec 20, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#include "ioif_sysctrl.h"


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



/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static volatile bool pwr_btn_pressed = false;

static volatile uint8_t GPIO_HW_REV_0 = 0;
static volatile uint8_t GPIO_HW_REV_1 = 0;
static volatile uint8_t GPIO_HW_REV_2 = 0;

#ifdef _USE_DEBUG_CLI
static IOIF_SysCtrl_t moduleinit_res = 0;
GPIO_PinState test_Ey;
#endif /*_USE_DEBUG_CLI*/

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

#ifdef CM_MODULE
static void SetPWRICCB(uint16_t gpioPin);
#endif


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_SysCtrl_t IOIF_SysCtrlInit(void)
{
	IOIF_SysCtrl_t ret = IOIF_SYS_CTRL_OK;

#ifdef CM_MODULE
	/* Power IC GPIO Callback Registration */
	ret = IOIF_SetGPIOCB(SW_IC_INT_Pin, IOIF_GPIO_EXTI_CALLBACK, SetPWRICCB);
#endif

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif /*_USE_DEBUG_CLI*/

	return ret;
}

bool IOIF_IsPwrBtnPressed(void)
{
	bool ret = false;

	if(pwr_btn_pressed == true)
		ret = true;
	else ret = false;

	return ret;
}


void IOIF_SysPwrOff(void)
{
#ifdef CM_MODULE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, MCU_SW_CLR_Pin, LOW);
#endif
}
void IOIF_SysBlockingDelay(uint32_t ms)
{
	BSP_CommonSysDelay(ms);
}
uint8_t IOIF_HW_RevGPIO(void)
{
	uint8_t ret = 0;

	GPIO_HW_REV_0 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_7);
	GPIO_HW_REV_1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_1);
	GPIO_HW_REV_2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_0);

	ret = (GPIO_HW_REV_2 << 2) | (GPIO_HW_REV_1 << 1) | (GPIO_HW_REV_0 << 0);

	return ret;
}

//IOIF_FWVersion_t IOIF_FW_SetRev(uint8_t t_major, uint8_t t_minor, uint8_t t_patch, uint16_t t_debug){
//	IOIF_FWVersion_t ret = {0,};
//	ret.major = t_major;
//	ret.minor = t_minor;
//	ret.patch = t_patch;
//	ret.debug = t_debug;
//	return ret;
//}


IOIF_FWVersion_t IOIF_FW_SetRev(void){
	IOIF_FWVersion_t ret = {0,};
	ret.major = FW_VER_MAJOR;
	ret.minor = FW_VER_MINOR;
	ret.patch = FW_VER_PATCH;
	ret.debug = FW_VER_DEBUG;
	return ret;
}

//IOIF_SAMCode_t IOIF_SAM_SetCode()

/*void IOIF_MA_CtrlLEDforBattery(uint16_t LED_Pin, IOIF_LEDStatus_t onoff)
{
	if (onoff != IOIF_LED_TOGGLE)
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_Pin, (IOIF_GPIOPinState_t)onoff);
	else
		IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_E, LED_Pin);
}

void IOIF_MA_CtrlLedforBoot(uint16_t LED_Pin, IOIF_LEDStatus_t onoff)
{
	switch(LED_Pin){
	case LED_BOOT_BLUE_Pin:
		if (onoff != IOIF_LED_TOGGLE)
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, LED_BOOT_BLUE_Pin, (IOIF_GPIOPinState_t)onoff);
		else
			IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_E, LED_BOOT_BLUE_Pin);
		break;
	case MCU_BOOT_Pin:
		if (onoff != IOIF_LED_TOGGLE)
			IOIF_WriteGPIOPin(IOIF_GPIO_PORT_C, MCU_BOOT_Pin, (IOIF_GPIOPinState_t)onoff);
		else
			IOIF_ToggleGPIOPin(IOIF_GPIO_PORT_C, MCU_BOOT_Pin);
		break;
	default:
		break;
	}
}*/


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ------------------- Function ------------------- */
#ifdef CM_MODULE
static void SetPWRICCB(uint16_t gpioPin)
{
	if(gpioPin == SW_IC_INT_Pin)
		pwr_btn_pressed = true;
}
#endif



#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunSysCtrl(cli_args_t *args)
{
	bool ret = false;

	if (args->cmpStr(0, "isinit") == true)
	{
		IOIF_SysCtrl_t state = moduleinit_res;

		const char* sysStateStrings[] = {
				"IOIF_SYS_CTRL_OK",
				"IOIF_SYS_CTRL_ERROR"
		};

		CLI_Printf(" * System init state : %s * \r\n",sysStateStrings[state]);
	}
	else if( args->argc == 4 && args->cmpStr(0, "mcu") == true) {

		uint32_t duration = 10;
		uint32_t temp = 0;

		CLI_Printf(" * MCU Temperature data gathering started! * \r\n");
		duration = (uint32_t) args->getData(3);			// param. 1 �� ���� string �����͸� ������ ��ȯ

		if(duration < 10)	ret = false;
		else				ret = true;

		while(ret && CLI_KeepLoop())
		{
			temp = IOIF_ReadCPUTemp();

			CLI_Printf("MCU Temperature : %d deg. \r\n", temp);

			/* Duration */
			CLI_Delay(duration);
		}

	}

	 else if(ret == false) //help
	{
		CLI_Printf(" * System Ctrl * \r\n");
		CLI_Printf("   duration(ms) least value is 10ms \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   system isinit \r\n");
		CLI_Printf("   system mcu temp get [duration(ms)] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   system assist btn + \r\n");
		CLI_Printf("   system delay 100 \r\n");
		CLI_Printf("   system gpio ctrl G 4 set \r\n");
	}
}

#endif /*_USE_DEBUG_CLI*/

