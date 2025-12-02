/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file ioif_pca9957hnmp.c
 * @date Created on: Sep 14, 2023
 * @author AngelRobotics HW Team
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

/** @defgroup I2C I2C
  * @brief I2C ICM20608G module driver
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
uint8_t LED_mode1_reg_status[2];
uint8_t LED_mode2_reg_status[2];

uint8_t LED_eflag_0[2];
uint8_t LED_eflag_1[2];
uint8_t LED_eflag_2[2];
uint8_t LED_eflag_3[2];

uint8_t ramp_flag[2];

uint8_t LED_output_delay[2];

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
IOIF_SPI_t spiHandle = IOIF_SPI1;
PCA9957HNMP_CallbackStruct LED_Driver_Callback;
IOIF_StatusTypeDef_t led_spi_status;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static IOIF_StatusTypeDef_t IOIF_SPIBlock(uint8_t reg, uint8_t tdata, uint8_t* rdata, uint8_t timeout, BSP_SPIOP_t operation)
{
	uint8_t packet_size = 2;
    uint8_t tx_data[2] = {reg, tdata};
    //uint8_t rx_data[2] = {0,};

    return led_spi_status = BSP_RunSPIBlock((BSP_SPI_t)spiHandle, tx_data, rdata, packet_size, timeout, operation);
    //BSP_RunSPIBlock((BSP_SPI_t)spiHandle, tx_data, rx_data, packet_size, timeout, operation);
//    HAL_SPI_Transmit(&hspi3, tx_data, 2, timeout);
}

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool IOIF_LED24chRead(uint8_t reg, uint8_t data, uint8_t* rxdata)
{

	bool ret = true;

	uint8_t read_reg = reg << 1;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_10, IOIF_GPIO_PIN_RESET); //NSS

	if(IOIF_SPIBlock((read_reg | 0x01), 0xff, rxdata, 50, BSP_SPI_TRANSMIT) != IOIF_OK)
		ret = false;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_10, IOIF_GPIO_PIN_SET); //NSS
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_10, IOIF_GPIO_PIN_RESET); //NSS

	if(IOIF_SPIBlock(0xff, 0xff, rxdata, 50, BSP_SPI_TRANSMIT_RECEIVE) != IOIF_OK)
		ret = false;

	//OIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_RESET); //nOE
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_10, IOIF_GPIO_PIN_SET); //NSS

	return ret;
}

bool IOIF_LED24chWrite(uint8_t reg, uint8_t data)
{
	bool ret = true;
	uint8_t dummy_packet[2] = {0,};
	uint8_t write_reg = reg << 1;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_7, IOIF_GPIO_PIN_RESET);
	//IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_SET);

	if(IOIF_SPIBlock((write_reg & 0xFE), data, dummy_packet, 50, BSP_SPI_TRANSMIT) != IOIF_OK)
		ret = false;

	//IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_RESET);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_7, IOIF_GPIO_PIN_SET);

	return ret;
}



uint8_t IOIF_LED24chInit(IOIF_SPI_t spiChannel)
{
	uint8_t init_status = 0;

	spiHandle = spiChannel;

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_8, IOIF_GPIO_PIN_SET); //nReset
	//Todo : 함수인자 맞는지 확인

	LED_Driver_Callback.pca9957hnmp_read = IOIF_LED24chRead;
	LED_Driver_Callback.pca9957hnmp_write = IOIF_LED24chWrite;

	PCA9957HNMP_Callback(&LED_Driver_Callback);

	IOIF_LED24chWrite((uint8_t)PWMALL, 0x00);
	IOIF_LED24chWrite((uint8_t)IREFALL, 0x00);

	uint8_t mode2_init = 0b00011001;			 		//clear all error flag
	if(IOIF_LED24chWrite(MODE2, mode2_init) != true) init_status++;

	/* Output Delay Adjustment */
	if(IOIF_LED24chWrite((uint8_t)OFFSET,  0b00001011) != true) init_status++;	// output delay is 1.375us
	//IOIF_LED24chRead(OFFSET, 0xff, LED_output_delay);

	/* Ramp Up Rate Adjustment */
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP0,  0b11011111) !=true) init_status++;		//ramp up rate : 00-3F
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP1,  0b11011111) !=true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP2,  0b11011111) !=true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP3,  0b11011111) !=true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP4,  0b11011111) !=true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)RAMP_RATE_GRP5,  0b11011111) !=true) init_status++;


	/* LED output control */
	if(IOIF_LED24chWrite((uint8_t)LEDOUT0,  0xAA) != true) init_status++;			//default setup, controlled through PWMx Register
	if(IOIF_LED24chWrite((uint8_t)LEDOUT1,  0xAA) != true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)LEDOUT2,  0xAA) != true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)LEDOUT3,  0xAA) != true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)LEDOUT4,  0xAA) != true) init_status++;
	if(IOIF_LED24chWrite((uint8_t)LEDOUT5,  0xAA) != true) init_status++;

	/* LED Driver Output Enable*/

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_9, IOIF_GPIO_PIN_RESET);

	return init_status;
}


/*LED Control*/
void IOIF_LED24chBattery(uint8_t nth, IOIF_LED24chColor_t color, uint8_t brightness) //Todo: 함수인자 구조체로 받기
{ 
	uint8_t pwm  ;
	uint8_t iref ;

	/*if (p_color != color) {
		for (int i = 0; i < 5; i++) {
			pwm  = i + 16;
			iref = i + 40;
			IOIF_LED24chWrite(pwm,  0xFF);
			IOIF_LED24chWrite(iref, brightness);
		}
	}*/
	for (int i = 0; i < 5; i++) {
		pwm  = i + 16;
		iref = i + 40;
		IOIF_LED24chWrite(pwm,  0x00);
		IOIF_LED24chWrite(iref, 0x00);
	}


	switch(color){
	case GREEN:
		for (int i = 1; i < nth+1; i++) {
			pwm  = i * 2 + 14;
			iref = i * 2 + 38;
			IOIF_LED24chWrite(pwm,  0xFF);
			IOIF_LED24chWrite(iref, brightness);
		}
		break;
	case RED:
		for (int i = 1; i < nth+1; i++) {
			pwm  = i * 2 + 14 + 1;
			iref = i * 2 + 38 + 1;
			IOIF_LED24chWrite(pwm,  0xFF);
			IOIF_LED24chWrite(iref, brightness);
		}

		break;
	default:
		break;
	}


	/*
 		case CYAN:
		IOIF_LED24chWrite((uint8_t)IREF0,  brightness);
		IOIF_LED24chWrite((uint8_t)IREF1,  brightness);
		IOIF_LED24chWrite((uint8_t)IREF2,  brightness);
		IOIF_LED24chWrite((uint8_t)PWM0,   0x01);
		IOIF_LED24chWrite((uint8_t)PWM2,   0x1B);
		IOIF_LED24chWrite((uint8_t)PWM1,   0x17);

		break;
	case YELLOW:
		IOIF_LED24chWrite((uint8_t)IREF0, brightness); // least 90
		IOIF_LED24chWrite((uint8_t)IREF1,  brightness);
		IOIF_LED24chWrite((uint8_t)IREF2,  brightness);
		IOIF_LED24chWrite((uint8_t)PWM0,   0xFF); //06 //FF
		IOIF_LED24chWrite((uint8_t)PWM2,   0xEB); //05 //EB
		IOIF_LED24chWrite((uint8_t)PWM1,   0x2A); //01 //2A
	 */

}

void IOIF_LED24chError(IOIF_LED24chColor_t color, uint8_t brightness){  //Todo: 함수인자 구조체로 받기

	static uint8_t p_color;
		if (p_color != color) {
			IOIF_LED24chWrite((uint8_t)IREF6, 0x00);
			IOIF_LED24chWrite((uint8_t)IREF7, 0x00);
			IOIF_LED24chWrite((uint8_t)PWM6,  0x00);
			IOIF_LED24chWrite((uint8_t)PWM7,  0x00);
		}

	switch(color){
	case GREEN:
		IOIF_LED24chWrite((uint8_t)IREF6, brightness);
		IOIF_LED24chWrite((uint8_t)PWM6,  0xFF);

		break;
	case RED:
		IOIF_LED24chWrite((uint8_t)IREF7, brightness);
		IOIF_LED24chWrite((uint8_t)PWM7,  0xFF);

		break;

	default:
			break;
	}

	p_color = color;

}

void IOIF_LED24chAssist(uint8_t nth, uint8_t brightness)
{
	/*ON*/
	for (int i = 1; i < nth+1; i++) {
		uint8_t pwm  = i + 23;
		uint8_t iref = i + 47;
		IOIF_LED24chWrite(pwm,  0xFF);
		IOIF_LED24chWrite(iref, brightness);
	}

	/*OFF*/
	for (int j = nth + 1 ; j < 10 +1 ; j++) { //10:LED num
		uint8_t off_pwm = j + 23;
		uint8_t off_iref = j + 47;
		IOIF_LED24chWrite(off_pwm,  0x00);
		IOIF_LED24chWrite(off_iref, 0x00);
	}
}


void IOIF_LED24chBluetooth(uint8_t brightness)
{
	IOIF_LED24chWrite((uint8_t)IREF18, brightness);
	IOIF_LED24chWrite((uint8_t)PWM18,  0xFF);
}

void IOIF_LED24chMode(IOIF_LED24chColor_t color, uint8_t brightness){

	static uint8_t p_color;
	if (p_color != color) {
		IOIF_LED24chWrite((uint8_t)IREF19, 0x00);
		IOIF_LED24chWrite((uint8_t)IREF20, 0x00);
		IOIF_LED24chWrite((uint8_t)IREF21, 0x00);
		IOIF_LED24chWrite((uint8_t)IREF22, 0x00);
		IOIF_LED24chWrite((uint8_t)PWM19,  0x00);
		IOIF_LED24chWrite((uint8_t)PWM20,  0x00);
		IOIF_LED24chWrite((uint8_t)PWM21,  0x00);
		IOIF_LED24chWrite((uint8_t)PWM22,  0x00);
	}

	switch(color){
	case WHITE:
		IOIF_LED24chWrite((uint8_t)IREF19, brightness);
		IOIF_LED24chWrite((uint8_t)PWM19,  0xFF);

		break;
	case BLUE:
		IOIF_LED24chWrite((uint8_t)IREF20, brightness);
		IOIF_LED24chWrite((uint8_t)PWM20,  0xFF);

		break;
	case RED:
		IOIF_LED24chWrite((uint8_t)IREF21, brightness);
		IOIF_LED24chWrite((uint8_t)PWM21,  0xFF);

		break;
	case GREEN:
		IOIF_LED24chWrite((uint8_t)IREF22, brightness);
		IOIF_LED24chWrite((uint8_t)PWM22,  0xFF);

		break;
	default:
			break;
	}

	p_color = color;
}

/* test needed
 void IOIF_LED24ch4Blink(uint16_t interval_time, FunctionPointer led_drive_func, uint8_t color, uint8_t brightness ){

	static uint16_t cnt = 0;

	if (cnt < interval_time) {
		led_drive_func(color,brightness);
	} else if (cnt < interval_time * 2) {
		led_drive_func(color,0x00);
	} else {
		cnt = 0;
	}

	cnt++;
}

void IOIF_LED24ch4BlinkAssist(uint16_t interval_time, uint8_t nth, uint8_t brightness ){

	static uint16_t cnt = 0;

	if (cnt < interval_time) {
		Drive_LED_Auxiliary(nth,brightness);
	} else if (cnt < interval_time * 2) {
		Drive_LED_Auxiliary(nth,0x00);
	} else {
		cnt = 0;
	}

	cnt++;
}

*/

#endif /* IOIF_PCA9957HNMP_ENABLED */
