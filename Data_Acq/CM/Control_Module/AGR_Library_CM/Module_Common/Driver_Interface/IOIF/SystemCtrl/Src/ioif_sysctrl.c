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

static volatile bool assist_plus_btn_pressed = false;
static volatile bool assist_minus_btn_pressed = false;

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
static void SetPWRICCB(uint16_t gpioPin);
static void SetAssistBtnCB(uint16_t gpioPin);

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_SysCtrl_t IOIF_SYS_CtrlInit(void)
{
	IOIF_SysCtrl_t ret = IOIF_SYS_CTRL_OK;

#ifdef CM_MODULE
	/* Power IC GPIO Callback Registration */
	ret = IOIF_SetGPIOCB(SW_IC_INT_Pin, IOIF_GPIO_EXTI_CALLBACK, SetPWRICCB);

	/*Assist Button Callback Registration*/
	ret = IOIF_SetGPIOCB(ASSIST_BTN_P_Pin, IOIF_GPIO_EXTI_CALLBACK, SetAssistBtnCB);
	ret = IOIF_SetGPIOCB(ASSIST_BTN_N_Pin, IOIF_GPIO_EXTI_CALLBACK, SetAssistBtnCB);
#endif

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif /*_USE_DEBUG_CLI*/

	return ret;
}

void IOIF_SYS_PwrOff(void)
{
#ifdef CM_MODULE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, MCU_SW_CLR_Pin, LOW);
#endif
}

bool IOIF_SYS_IsPwrBtnPressed(void)
{
	bool ret = false;

	if(pwr_btn_pressed == true)
		ret = true;
	else ret = false;

	return ret;
}

void IOIF_SYS_BlockingDelay(uint32_t ms)
{
	BSP_CommonSysDelay(ms);
}

uint8_t IOIF_SYS_HwRevGPIO(void)
{
	uint8_t ret = 0;

	GPIO_HW_REV_0 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_7);
	GPIO_HW_REV_1 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_1);
	GPIO_HW_REV_2 = IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_0);

	ret = (GPIO_HW_REV_2 << 2) | (GPIO_HW_REV_1 << 1) | (GPIO_HW_REV_0 << 0);

	return ret;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/* ----------------------- FUNCTION ----------------------- */

static void SetPWRICCB(uint16_t gpioPin)
{
#ifdef SUIT_MINICM_ENABLED
	if(gpioPin == SW_IC_INT_Pin)
		pwr_btn_pressed = true;
#endif /* SUIT_MINICM_ENABLED */
}

static void SetAssistBtnCB(uint16_t gpioPin)
{
#ifdef SUIT_MINICM_ENABLED
	if(gpioPin == ASSIST_BTN_P_Pin){
		if ((IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, ASSIST_BTN_P_Pin)) == IOIF_GPIO_PIN_RESET)
			assist_plus_btn_pressed = true;
		else assist_plus_btn_pressed = false;
	}

	if(gpioPin == ASSIST_BTN_N_Pin){
		if ((IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, ASSIST_BTN_N_Pin)) == IOIF_GPIO_PIN_RESET)
			assist_minus_btn_pressed = true;
		else assist_minus_btn_pressed = false;
	}
#endif /* SUIT_MINICM_ENABLED */
}


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
	else if(args->argc == 3 && args->cmpStr(0, "power") == true)
	{
		if (args->cmpStr(2, "exti") == true)
		{
			CLI_Printf(" * Press power button over 3s in 10s. * \r\n");

			while(CLI_KeepLoop()){
				static uint8_t btn_cnt = 0;
				static uint16_t cnt = 0;

				if (pwr_btn_pressed == true)
				{
					btn_cnt++;
					if (btn_cnt > 100) btn_cnt = 100;

					CLI_Printf("Power button pressed %d times. \r\n" , btn_cnt);
					pwr_btn_pressed = false;
					cnt = 0;

				} else
				{
					cnt++;
					if (cnt >= 100) //10s
					{
						CLI_Printf("Error detecting power button. \r\n");
						cnt = 0;
						break;
					}
				}
				/* Duration */
				CLI_Delay(100); //ms
			}

		} else if (args->cmpStr(2, "twice") == true)
		{
			CLI_Printf(" * Press power button twice.    *\r\n");
			CLI_Printf(" * Be aware not to hold button. * \r\n");
			CLI_Printf(" * Quick press is required.     * \r\n");

			static uint16_t time = 0;
			static uint16_t last_click = 0;
			static uint8_t click_cnt = 0;
			static uint16_t click_duration = 0;
			const uint16_t click_time_threshold = 5; // click_time_threshold * CLI Loop = double click checking time

			while(CLI_KeepLoop())
			{
				time++;
				if ((IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, PB_IN_MCU_Pin)) == IOIF_GPIO_PIN_RESET && click_cnt == 0)
				{
					click_cnt = 1;
					last_click = time;
					CLI_Delay(100); //reducing debounding time

				} else if ((IOIF_ReadGPIOPin(IOIF_GPIO_PORT_G, PB_IN_MCU_Pin)) == IOIF_GPIO_PIN_RESET && click_cnt == 1)
				{
					click_duration = time - last_click;
					if (click_duration <= click_time_threshold)
					{
						CLI_Printf("Power button double click detected \r\n");
						click_cnt = 0;
					} else
					{
						last_click = time;
					}

				}

				if(click_cnt == 1 && (time - last_click > click_time_threshold)) click_cnt = 0;

				CLI_Delay(100);
			}

		}else if (args->cmpStr(2, "ctrl") == true)
		{
			CLI_Printf(" * Press power button over 3s in 10s. *\r\n");

			while(CLI_KeepLoop())
			{
				static uint8_t second = 0;
				static uint8_t flag = 0;

				if (pwr_btn_pressed == true)
				{
					CLI_Printf("Power button pressed. The system will be down. \r\n");
					pwr_btn_pressed = false;
					flag = 1;

				} else if (flag == 1)
				{
					second ++;
					if (second == 10) 		CLI_Printf("3 \r\n");
					else if (second == 20)  CLI_Printf("2 \r\n");
					else if (second == 30)  CLI_Printf("1 \r\n");
					else if (second == 40){ CLI_Printf("Bye ~ \r\n"); IOIF_SYS_PwrOff();}

				} else
				{
					static uint16_t cnt = 0;
					cnt++;

					if (cnt == 100) //10s
					{
						CLI_Printf(" !!Error detecting power button.!!\r\n");
						cnt = 0;
						break;
					}
				}

				/* Duration */
				CLI_Delay(100); //ms
			}

		}



	} else if (args->cmpStr(0, "assist") == true)
	{
		if (args->cmpStr(2, "+") == true)
		{
			CLI_Printf(" * Press the assist + button fewer than 100 times. * \r\n");

			while(CLI_KeepLoop())
			{
				static uint8_t btn_cnt = 0;

				if (assist_plus_btn_pressed == true)
				{
					btn_cnt ++;
					if (btn_cnt > 100) btn_cnt = 100;

					CLI_Printf("Assist button pressed %d times. \r\n", btn_cnt);
					assist_plus_btn_pressed = false; //
				}
				/* Duration */
				CLI_Delay(100); //ms
			}

		} else if (args->cmpStr(2, "-") == true)
		{
			CLI_Printf(" * Press the assist - button fewer than 100 times. *\r\n");

			while(CLI_KeepLoop())
			{
				static uint8_t btn_cnt = 0;

				if (assist_minus_btn_pressed == true)
				{
					btn_cnt ++;
					if (btn_cnt > 100) btn_cnt = 100;
					CLI_Printf("Assist button pressed %d times. \r\n", btn_cnt);
					assist_minus_btn_pressed = false; //
				}
				/* Duration */
				CLI_Delay(100); //ms
			}
		} else if (args->cmpStr(2, "both") == true)
		{
			CLI_Printf(" * Press and hold the assist + and - buttons at the same time. * \r\n");

			while(CLI_KeepLoop())
			{
				static uint8_t btn_cnt = 0;

				if (assist_minus_btn_pressed == true && assist_plus_btn_pressed == true )
				{
					btn_cnt ++;
					if (btn_cnt > 100) btn_cnt = 100;
					assist_minus_btn_pressed = false;
					assist_plus_btn_pressed = false;

					CLI_Printf("Assist buttons are pressed %d times. \r\n", btn_cnt);
				}
				/* Duration */
				CLI_Delay(100); //ms
			}
		}

	}  else if (args->argc == 2 && args->cmpStr(0, "delay") == true)
	{
		uint32_t duration = 10;
		duration = (uint32_t) args->getData(1);

		if(duration < 10)	ret = false;
		else				ret = true;
		static	uint16_t cnt = 0 ;
		while(ret && CLI_KeepLoop()){


			CLI_Printf(" Delay term %d \r\n", cnt);
			IOIF_SYS_BlockingDelay(duration);

			cnt++;
		}
		cnt = 0;

	} else if( args->argc == 4 && args->cmpStr(0, "mcu") == true) {

		uint32_t duration = 10;
		uint32_t temp = 0;

		CLI_Printf(" * MCU Temperature data gathering started! * \r\n");
		duration = (uint32_t) args->getData(3);			// param. 1 에 대한 string 데이터를 정수로 변환

		if(duration < 10)	ret = false;
		else				ret = true;

		while(ret && CLI_KeepLoop())
		{
			temp = IOIF_ReadCPUTemp();

			CLI_Printf("MCU Temperature : %d deg. \r\n", temp);

			/* Duration */
			CLI_Delay(duration);
		}

	} else if ( args->argc == 5 && args->cmpStr(0, "gpio") == true)
	{

		char* port;
		char* state;
		uint16_t pin;
		GPIO_TypeDef* GPIO_Port;
		GPIO_PinState pinState;

		port = args->getStr(2);
		pin = (uint16_t) args->getData(3);
		state = args->getStr(4);

		if (pin > 15)
		{
			CLI_Printf(" !! GPIO Pin should be under 15 !! \r\n  " );
			return;
		}

		 static const uint16_t gpio_pins[] = {
		        GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,
		        GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7,
		        GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
		        GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15
		    };

		if (strcmp(state, "set") == 0)
			pinState = GPIO_PIN_SET;
		else if (strcmp(state, "reset") == 0)
			pinState = GPIO_PIN_RESET;
		else
			return;


		switch (port[0]) {
		        case 'A':
		            GPIO_Port = GPIOA;
		            break;
		        case 'B':
		            GPIO_Port = GPIOB;
		            break;
		        case 'C':
		            GPIO_Port = GPIOC;
		            break;
		        case 'D':
		            GPIO_Port = GPIOD;
		            break;
		        case 'E':
		            GPIO_Port = GPIOE;
		            break;
		        case 'F':
		            GPIO_Port = GPIOF;
		            break;
		        case 'G':
		            GPIO_Port = GPIOG;
		            break;
		        default:
		        	CLI_Printf(" !! Check the GPIO Port !! \r\n " );
		            // 잘못된 포트 알파벳 입력 시 반환
		            return;
		    }

		HAL_GPIO_WritePin(GPIO_Port, gpio_pins[pin], pinState);

		if (HAL_GPIO_ReadPin(GPIO_Port, gpio_pins[pin]) == pinState)
			CLI_Printf(" * GPIO_Port_%c, GPIO_Pin_%d %s * \r\n", port[0], pin, state);
		else
			CLI_Printf(" !! GPIO set failed !! " );
	}

	 else if(ret == false) //help
	{
		CLI_Printf(" * System Ctrl * \r\n");
		CLI_Printf("   Buttons and System Ctrl \r\n");
		CLI_Printf("   duration(ms) least value is 10ms \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   system isinit \r\n");
		CLI_Printf("   system power btn [exti/twice/ctrl] \r\n");
		CLI_Printf("   system assist btn [+/-/both] \r\n");
		CLI_Printf("   system mcu temp get [duration(ms)] \r\n");
		CLI_Printf("   system delay [duration(ms)] \r\n");
		CLI_Printf("   system gpio ctrl [port(A to G for capital letters)] [pin number] [set/reset] \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   system assist btn + \r\n");
		CLI_Printf("   system delay 100 \r\n");
		CLI_Printf("   system gpio ctrl G 4 set \r\n");
	}
}

#endif /*_USE_DEBUG_CLI*/

