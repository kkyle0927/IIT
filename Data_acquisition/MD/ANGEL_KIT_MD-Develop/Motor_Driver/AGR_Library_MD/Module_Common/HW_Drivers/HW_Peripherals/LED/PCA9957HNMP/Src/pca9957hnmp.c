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
void pc9957hnmp_read(uint8_t reg, uint8_t value, uint8_t* rxdata){
	pca9957hnmp_callback.pca9957hnmp_read(reg, value, rxdata);
}

void pc9957hnmp_write(uint8_t reg, uint8_t value){
	pca9957hnmp_callback.pca9957hnmp_write(reg, value);
}

void PCA9957HNMP_Callback(PCA9957HNMP_CallbackStruct* led24ch_callback)
{
//	if(!led24ch_callback->pca9957hnmp_read || !led24ch_callback->pca9957hnmp_write)
//		{ return -1 };
	pca9957hnmp_callback.pca9957hnmp_read = led24ch_callback->pca9957hnmp_read;
	pca9957hnmp_callback.pca9957hnmp_write = led24ch_callback->pca9957hnmp_write;

}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
