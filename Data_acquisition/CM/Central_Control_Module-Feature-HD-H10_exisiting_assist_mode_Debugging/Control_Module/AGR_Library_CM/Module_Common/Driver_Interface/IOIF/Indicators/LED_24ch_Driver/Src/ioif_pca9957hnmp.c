/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file ioif_pca9957hnmp.c
 * @date Created on: Sep 14, 2023
 * @author AngelRobotics FW Team
 * @brief Driver code for the PCA9957HNMP LED DRIVER.
 *
 * Todo: Add Annotation
 *
 * Refer to the PCA9957HNMP datasheet and related documents for more information.
 *
 * @ref PCA9957HNMP Datasheet
 */

/* CAUTION!!
 * "IOIF_InitLEDDriver" function must be used.
 */


#include "ioif_pca9957hnmp.h"

/** @defgroup SPI SPI
 * @brief SPI PCA9957HNMP module driver
 * @{
 */

#ifdef IOIF_PCA9957HNMP_ENABLED
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

IOIF_SPI_t spiHandle = IOIF_SPI1;
PCA9957HNMP_CallbackStruct LED_Driver_Callback;


/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
//static uint8_t LedCmdTxData[2] = {0,0};
//static uint8_t LedCmdRxData[2] = {0,0};
static uint8_t LedCmdTxData[2] __attribute__((section(".spi1TxBuff"))) = {0};
static uint8_t LedCmdRxData[2] __attribute__((section(".spi1RxBuff"))) = {0};


#ifdef _USE_DEBUG_CLI
static IOIF_LEDState_t moduleinit_res = IOIF_LED_STATUS_ERROR;
#endif /*_USE_DEBUG_CLI*/
/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
/*Callback Regist*/
static void SpiLedTxCB(void* params);
static void SpiLedTxRxCB(void* params);

/*LED control*/
static void setLEDS(uint8_t led_num, uint8_t pwm_reg, uint8_t iref_reg, uint8_t pwm_value, uint8_t iref_value);
static void setLinedupLEDS(uint8_t nth, uint8_t pwm_reg, uint8_t iref_reg, uint8_t brightness);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool IOIF_LED24chSPIStart(uint8_t reg, uint8_t tdata, uint8_t* rdata, uint8_t timeout, PCA9957HNMP_SPIOP_t operation)
{

	bool ret = true;

	uint8_t packet_size = 2;
	//uint8_t tx_data[2] = {reg, tdata};
	LedCmdTxData[0] = reg; LedCmdTxData[1] = tdata;
	//uint8_t rx_data[2] = {0,};

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET); 		 //CS is low,

	switch(operation)
	{
	case BSP_SPI_TRANSMIT :
	case BSP_SPI_RECEIVE :
	case BSP_SPI_TRANSMIT_RECEIVE :
		if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, (BSP_SPIOP_t)operation) != BSP_OK)
			ret = false;
		break;
	case BSP_SPI_TRANSMIT_IT :
	case BSP_SPI_RECEIVE_IT :
	case BSP_SPI_TRANSMIT_RECEIVE_IT :
	case BSP_SPI_TRANSMIT_DMA :
	case BSP_SPI_RECEIVE_DMA :
	case BSP_SPI_TRANSMIT_RECEIVE_DMA :
		if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, rdata, packet_size, operation) != BSP_OK)
			ret = false;
		break;
	default:
		ret = false;
		break;
	}

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_SET); 		//CS becomes high after transmission complete

	return ret;
}

bool IOIF_LED24chnOECtrl(bool state)
{
	bool ret = true;
	/* Active Low, Output Enable Port : GPIOD, GPIO 9 */
	if(state == true)
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nOE_Pin, IOIF_GPIO_PIN_SET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}
	else
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nOE_Pin, IOIF_GPIO_PIN_RESET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}

	return ret;
}

bool IOIF_LED24chnRSTCtrl(bool state)
{
	bool ret = true;
	/* Active Low, Reset Port : GPIOD, GPIO 9 */
	if(state == true)
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nRESET_Pin, IOIF_GPIO_PIN_SET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}
	else
	{
		if(IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, LED_DRV_nRESET_Pin, IOIF_GPIO_PIN_RESET) != IOIF_GPIO_STATUS_OK)
			ret = false;
	}

	return ret;
}

IOIF_LEDState_t IOIF_LED24chSPIDMAStart(IOIF_SPI_t spiChannel)
{
	IOIF_LEDState_t ret = IOIF_OK;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);		// cs low

	if(BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_RECEIVE_DMA) != BSP_OK)
		return ret = IOIF_LED_STATUS_ERROR;

	return ret;
}

IOIF_LEDState_t IOIF_LED24chDriverInit(IOIF_SPI_t spiChannel)
{
	IOIF_LEDState_t init_res = IOIF_LED_STATUS_OK;

	/* SPI DMA Mode Callback Driver Registration */
	BSP_SetSPICB(spiHandle, BSP_SPI_TX_CPLT_CALLBACK, SpiLedTxCB, NULL);		// Register TX Complete Callback
	BSP_SetSPICB(spiHandle, BSP_SPI_TXRX_CPLT_CALLBACK, SpiLedTxRxCB, NULL);	// Register TX-RX Complete Callback

	if(IOIF_LED24chSPIDMAStart(spiHandle) != IOIF_LED_STATUS_OK)
		return IOIF_LED_STATUS_ERROR;

	//Todo : 함수인자 맞는지 확인

	/* LED Driver Init. */
	LED_Driver_Callback.pca9957hnmp_spi_op 	  = IOIF_LED24chSPIStart;
	LED_Driver_Callback.pca9957hnmp_nOE_ctrl  = IOIF_LED24chnOECtrl;
	LED_Driver_Callback.pca9957hnmp_nRST_ctrl = IOIF_LED24chnRSTCtrl;

	PCA9957HNMP_RegisterCallback(&LED_Driver_Callback);

	if(PCA9957HNMP_InitDrv() != true)
		init_res = IOIF_LED_STATUS_OK;

#ifdef _USE_DEBUG_CLI
	moduleinit_res = init_res;
#endif /*_USE_DEBUG_CLI*/

	return init_res;
}

//----------------------------------------------------------------------------------------//
/*LED Control*/
void IOIF_LED24chBluetooth(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t brightness)
{
	static uint16_t cnt = 0;
	static uint8_t p_ctrl;

	switch (ctrl) {
	case OFF:
		setLEDS(1, PWM18, IREF18, 0x00, 0x00);
		break;

	case ON:
		if(p_ctrl == ctrl) break;
		setLEDS(1, PWM18, IREF18, 0xFF, brightness);
		break;

	case BLINK:
		if (cnt < blinking_period) 			setLEDS(1, PWM18, IREF18, 0xFF, brightness);
		else if (cnt < blinking_period * 2) 	setLEDS(1, PWM18, IREF18, 0x00, 0x00);
		else 						cnt = 0;

		cnt++;
		break;
	}

	p_ctrl = ctrl;
}

void IOIF_LED24chError(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, IOIF_LED24chColor_t color, uint8_t brightness){  //Todo: 함수인자 구조체로 받기

	static IOIF_LED24chColor_t p_color;
	static uint16_t cnt = 0;

	/*led color reset*/
	if (p_color != color)
		setLEDS(2, PWM6, IREF6, 0x00, 0x00);

	switch (ctrl) {
	case OFF:
		setLEDS(2, PWM6, IREF6, 0x00, 0x00);

		break;
	case ON:
		if (p_color == color)	break; // led 드라이버 중복 명령 제거
		if (color == GREEN) 	setLEDS(1, PWM6, IREF6, 0xFF, brightness);
		else if (color == RED)	setLEDS(1, PWM7, IREF7, 0xFF, brightness);

		break;
	case BLINK:
		if (cnt < blinking_period) {
			if (color == GREEN) 	setLEDS(1, PWM6, IREF6, 0xFF, brightness);
			else if (color == RED)	setLEDS(1, PWM7, IREF7, 0xFF, brightness);
		} else if (cnt < blinking_period * 2) {
			setLEDS(2, PWM6, IREF6, 0x00, 0x00); // **변경된 부분**
		} else {
			cnt = 0;
		}
		cnt++;

		break;
	}

	p_color = color;
}

void IOIF_LED24chBattery(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t nth, IOIF_LED24chColor_t color, uint8_t brightness)
{
	static IOIF_LED24chColor_t p_color;
	static uint8_t p_nth;
	static uint16_t cnt = 0;

	/*led reset*/
	if (p_color != color || p_nth != nth)
		setLEDS(6, PWM0, IREF0, 0x00, 0x00);

	switch (ctrl) {
	case OFF:
		setLEDS(6, PWM0, IREF0, 0x00, 0x00);

		break;
	case ON:
		if (p_color == color && p_nth == nth) break;
		if (color == GREEN) 	setLinedupLEDS(nth, PWM0, IREF0, brightness);
		else if (color == RED) 	setLinedupLEDS(nth, PWM1, IREF1, brightness);

		break;
	case BLINK:
		if (cnt < blinking_period) {
			if (color == GREEN) 	setLinedupLEDS(nth, PWM0, IREF0, brightness);
			else if (color == RED) 	setLinedupLEDS(nth, PWM1, IREF1, brightness);
		} else if (cnt < blinking_period * 2) {
			setLEDS(6, PWM0, IREF0, 0x00, 0x00);
		} else {
			cnt = 0;
		}
		cnt++;

		break;
	}

	p_color = color;
	p_nth = nth;
}

void IOIF_LED24chMode(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, IOIF_LED24chColor_t color, uint8_t brightness)
{
	static IOIF_LED24chColor_t p_color;
	static uint16_t cnt = 0;

	/*color reset*/
	if (p_color != color)
		setLEDS(4, PWM19, IREF19, 0x00, 0x00);

	switch (ctrl) {
	case OFF:
		setLEDS(4, PWM19, IREF19, 0x00, 0x00);

		break;
	case ON:
		if (p_color == color) break;
		if (color == WHITE) 	 setLEDS(1, PWM19, IREF19, 0xFF, brightness);
		else if (color == BLUE)  setLEDS(1, PWM20, IREF20, 0xFF, brightness);
		else if (color == RED) 	 setLEDS(1, PWM21, IREF21, 0xFF, brightness);
		else if (color == GREEN) setLEDS(1, PWM22, IREF22, 0xFF, brightness);

		break;
	case BLINK:
		if (cnt < blinking_period) {
			if (color == WHITE) 	 setLEDS(1, PWM19, IREF19, 0xFF, brightness);
			else if (color == BLUE)  setLEDS(1, PWM20, IREF20, 0xFF, brightness);
			else if (color == RED) 	 setLEDS(1, PWM21, IREF21, 0xFF, brightness);
			else if (color == GREEN) setLEDS(1, PWM22, IREF22, 0xFF, brightness);
		} else if (cnt < blinking_period * 2) {
			setLEDS(4, PWM19, IREF19, 0x00, 0x00);
		} else {
			cnt = 0;
		}
		cnt++;
		break;
	}

	p_color = color;
}

void IOIF_LED24chAssist(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t nth, uint8_t brightness)
{
	static uint16_t cnt = 0;
	static uint8_t last_nth;
	static uint8_t last_ctrl;

	if(nth > 10) nth = 10;
	else if (nth < 0) nth = 0;


	switch (ctrl) {
	case OFF:
		setLEDS(10, PWM8, IREF8, 0x00, 0x00);
		break;

	case ON:
		if (nth == last_nth && ctrl != last_ctrl) break;
		setLEDS(nth, PWM8, IREF8, 0xFF, brightness);
		setLEDS(10 - nth, PWM8 + nth, IREF8 + nth, 0x00, 0x00);
		break;

	case BLINK:
		if (cnt < blinking_period) {
			setLEDS(nth, PWM8, IREF8, 0xFF, brightness);
			setLEDS(10 - nth, PWM8 + nth, IREF8 + nth, 0x00, 0x00);
		} else if (cnt < blinking_period * 2) {
			setLEDS(10, PWM8, IREF8, 0x00, 0x00);
		} else {
			cnt = 0;
		}

		cnt++;
		break;
	}
	last_nth = nth;
	last_ctrl = ctrl;
}

void IOIF_LEDforAssistOnebyOne(uint8_t nth, uint8_t brightness)
{
	PCA9957HNMP_WriteReg(PWM7 + nth, 0xFF);
	PCA9957HNMP_WriteReg(IREF7 + nth, brightness);
}

void IOIF_LED24chAll(IOIF_LED24chCtrl_t ctrl, uint8_t brightness)
{
	static IOIF_LED24chCtrl_t p_ctrl = BLINK; //초기에만 설정

//	if(p_ctrl != ctrl){
		switch (ctrl) {
		case OFF:
			IOIF_LED24chError(OFF, LED_NA, CNULL, 0x00);
			IOIF_LED24chBattery(OFF, LED_NA, 3, CNULL, 0x00);
			IOIF_LED24chMode(OFF, LED_NA, CNULL, 0x00);

			PCA9957HNMP_WriteReg(PWMALL, 0x00);
			PCA9957HNMP_WriteReg(IREFALL, 0x00);
			break;

		case ON:
			PCA9957HNMP_WriteReg(PWMALL, 0xFF);
			PCA9957HNMP_WriteReg(IREFALL, 0x40);
			break;
		default:
			break;
		}
//	}

//	p_ctrl = ctrl;

}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/*Callback Function*/
static void SpiLedTxCB(void* params)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_SET);	// cs becomes high if transmit is completed,

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);	// cs becomes low if transmit is starting,
	/* Re-start DMA */
	BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_DMA);
}

static void SpiLedTxRxCB(void* params)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_SET);	// cs becomes high if transmit is completed,

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);	// cs becomes low if transmit is starting,
	/* Re-start DMA */
	BSP_RunSPIDMA((BSP_SPI_t)spiHandle, LedCmdTxData, LedCmdRxData, sizeof(LedCmdRxData), BSP_SPI_TRANSMIT_RECEIVE_DMA);
}

/*LED Control*/
static void setLEDS(uint8_t led_num, uint8_t pwm_reg, uint8_t iref_reg, uint8_t pwm_value, uint8_t iref_value) {
	uint8_t pwm, iref;

	for (uint8_t i = 0; i < led_num; i++) {
		pwm = i + pwm_reg;
		iref = i + iref_reg;
		PCA9957HNMP_WriteReg(pwm, pwm_value);
		PCA9957HNMP_WriteReg(iref, iref_value);
	}
}

static void setLinedupLEDS(uint8_t nth, uint8_t pwm_reg, uint8_t iref_reg, uint8_t brightness) {
	uint8_t pwm, iref;

	for (uint8_t i = 0; i < nth; i++) {
		pwm = i * 2 + pwm_reg;
		iref = i * 2 + iref_reg;
		PCA9957HNMP_WriteReg(pwm, 0xFF);
		PCA9957HNMP_WriteReg(iref, brightness);
	}
}

#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunPca9957hnmp(cli_args_t *args)
{
	bool ret = false;

	uint8_t brigthness = 0;

	if(args->cmpStr(0, "isinit") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		IOIF_StatusTypeDef_t state = moduleinit_res;

		const char* ledStateStrings[] = {
				"IOIF_LED_STATUS_OK",
				"IOIF_LED_STATUS_ERROR",
				"IOIF_LED_STATUS_BUSY",
				"IOIF_LED_STATUS_TIMEOUT"
		};

		CLI_Printf(" * PCA9957HNMP init state : %s * \r\n", ledStateStrings[state]);

	} else if (args->argc == 3 && args->cmpStr(0, "battery") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "green") == true) IOIF_LED24chBattery(ON, LED_NA, 3, GREEN, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chBattery(ON, LED_NA, 3, RED, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->argc == 3 && args->cmpStr(0, "error") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "green") == true) IOIF_LED24chError(ON, LED_NA, GREEN, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chError(ON, LED_NA, RED, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->argc == 3 && args->cmpStr(0, "mode") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;

		if 		(args->cmpStr(1, "white") == true) IOIF_LED24chMode(ON, LED_NA, WHITE, brigthness);
		else if (args->cmpStr(1, "blue") == true) IOIF_LED24chMode(ON, LED_NA, BLUE, brigthness);
		else if (args->cmpStr(1, "red") == true) IOIF_LED24chMode(ON, LED_NA, RED, brigthness);
		else if (args->cmpStr(1, "green") == true) IOIF_LED24chMode(ON, LED_NA, GREEN, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	}else if (args->argc == 4 && args->cmpStr(0, "assist") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		uint8_t stage = 0;

		stage = (uint8_t) args->getData(2);
		if(stage > 10) stage = 10;
		brigthness = (uint8_t) args->getData(3);
		if (brigthness > 255) brigthness = 255;

		IOIF_LED24chAssist(ON, LED_NA, stage, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	} else if (args->argc == 2 && args->cmpStr(0, "ble") == true)
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		brigthness = (uint8_t) args->getData(1);
		if (brigthness > 255) brigthness = 255;

		IOIF_LED24chBluetooth(ON, LED_NA, brigthness);

		CLI_Printf(" * Check the LED on board * \r\n");

	}  else if (args->argc == 3 && args->cmpStr(1, "on") == true)
	{
		/*init for all previous led ctrl*/
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);


		CLI_Printf(" * Check the LED on board * \r\n");

		brigthness = (uint8_t) args->getData(2);
		if (brigthness > 255) brigthness = 255;
		PCA9957HNMP_WriteReg(PWMALL, 0xFF);
		PCA9957HNMP_WriteReg(IREFALL, brigthness);

	} else if (args->argc == 2 && args->cmpStr(1, "off") == true)
	{
		CLI_Printf(" * Check the LED on board * \r\n");

		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

	}else if (ret == false) //help
	{
		PCA9957HNMP_WriteReg(PWMALL, 0x00);
		PCA9957HNMP_WriteReg(IREFALL, 0x00);

		CLI_Printf(" * LED Display * \r\n");
		CLI_Printf("   Range of data : brightness (0~255), stage(0~10) \r\n");
		CLI_Printf(" * Usage * \r\n");
		CLI_Printf("   pca9957hnmp isinit \r\n");
		CLI_Printf("   pca9957hnmp [battery/error] [green/red] [brightness] \r\n");
		CLI_Printf("   pca9957hnmp mode [green/red/white/blue] [brightness]\r\n");
		CLI_Printf("   pca9957hnmp assist led [stage] [brightness]\r\n");
		CLI_Printf("   pca9957hnmp ble [brightness] \r\n");
		CLI_Printf("   pca9957hnmp all on [brightness] \r\n");
		CLI_Printf("   pca9957hnmp all off \r\n");
		CLI_Printf(" * Command Example * \r\n");
		CLI_Printf("   pca9957hnmp error red 50 \r\n");
		CLI_Printf("   pca9957hnmp mode white 50 \r\n");
		CLI_Printf("   pca9957hnmp assist led 5 10 \r\n");
	}

}

#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_PCA9957HNMP_ENABLED */
