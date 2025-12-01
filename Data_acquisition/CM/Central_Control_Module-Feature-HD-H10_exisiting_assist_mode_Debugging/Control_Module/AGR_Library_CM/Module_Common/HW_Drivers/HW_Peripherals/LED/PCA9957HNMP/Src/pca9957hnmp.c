/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file pca9957hnmp.c
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

#include "pca9957hnmp.h"


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
PCA9957HNMP_CallbackStruct pca9957hnmp_callback;



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

bool PCA9957HNMP_InitDrv(void)
{
	bool ret = true;

	uint8_t init_status = 0;

	/* LED Driver Reset hold off */
	pca9957hnmp_callback.pca9957hnmp_nRST_ctrl(true); 			//nReset is de-assertion


	uint8_t mode2_init = 0b00011001;			 		//clear all error flag
	if(PCA9957HNMP_WriteReg(MODE2, mode2_init) != true) init_status++;

	/* Output Delay Adjustment */
	if(PCA9957HNMP_WriteReg((uint8_t)OFFSET,  0b00001011) != true) init_status++;	// output delay is 1.375us
	//IOIF_LEDRead(OFFSET, 0xff, LED_output_delay);

	/* Ramp Up Rate Adjustment */
	if(PCA9957HNMP_WriteReg((uint8_t)RAMP_RATE_GRP0,  0b11011111) !=true) init_status++;		//ramp up rate : 00-3F
	if(PCA9957HNMP_WriteReg((uint8_t)RAMP_RATE_GRP1,  0b11011111) !=true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)RAMP_RATE_GRP2,  0b11011111) !=true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)RAMP_RATE_GRP3,  0b11011111) !=true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)RAMP_RATE_GRP4,  0b11011111) !=true) init_status++;


	/* LED output control */
	if(PCA9957HNMP_WriteReg((uint8_t)LEDOUT0,  0xAA) != true) init_status++;			//default setup, controlled through PWMx Register
	if(PCA9957HNMP_WriteReg((uint8_t)LEDOUT1,  0xAA) != true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)LEDOUT2,  0xAA) != true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)LEDOUT3,  0xAA) != true) init_status++;
	if(PCA9957HNMP_WriteReg((uint8_t)LEDOUT4,  0xAA) != true) init_status++;

	/* LED Driver Output Enable*/

	pca9957hnmp_callback.pca9957hnmp_nOE_ctrl(false); 			//output enabled

	if(init_status > 0)
		ret = false;
	else
		ret = true;

	return ret;
}

bool PCA9957HNMP_WriteReg(uint8_t reg, uint8_t data)
{
	bool ret = true;

	uint8_t dummy_packet[2] = {0,};
	uint8_t write_reg = reg << 1;

	if(pca9957hnmp_callback.pca9957hnmp_spi_op((write_reg & 0xFE), data, dummy_packet, 50, SPI_TRANSMIT_DMA) != true)
		ret = false;

	return ret;
}

bool PCA9957HNMP_ReadReg(uint8_t reg, uint8_t data, uint8_t* rxdata)
{
	bool ret = true;

	uint8_t read_reg = reg << 1;

	/* step 1 : transmit cmd */
	if(pca9957hnmp_callback.pca9957hnmp_spi_op((read_reg | 0x01), 0xff, rxdata, 50, SPI_TRANSMIT_DMA) != true)
		ret = false;

	/* step 2 : transmit & receive packet after sending dummy packet */
	if(pca9957hnmp_callback.pca9957hnmp_spi_op(0xff, 0xff, rxdata, 50, SPI_TRANSMIT_RECEIVE_DMA) != true)
		ret = false;

	return ret;
}


bool PCA9957HNMP_RegisterCallback(PCA9957HNMP_CallbackStruct* led24ch_callback)
{
	bool ret = true;

	if(!led24ch_callback->pca9957hnmp_nOE_ctrl || !led24ch_callback->pca9957hnmp_nRST_ctrl|| !led24ch_callback->pca9957hnmp_spi_op)
		return false;
	pca9957hnmp_callback.pca9957hnmp_spi_op	   = led24ch_callback->pca9957hnmp_spi_op;
	pca9957hnmp_callback.pca9957hnmp_nOE_ctrl  = led24ch_callback->pca9957hnmp_nOE_ctrl;
	pca9957hnmp_callback.pca9957hnmp_nRST_ctrl = led24ch_callback->pca9957hnmp_nRST_ctrl;

	return ret;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
