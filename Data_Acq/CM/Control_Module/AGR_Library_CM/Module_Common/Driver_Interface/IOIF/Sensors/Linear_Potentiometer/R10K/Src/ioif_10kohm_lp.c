/**
 *-----------------------------------------------------------
 *          10KOHM LP - L30 SHOE POTENTIOMETER CONTROL
 *-----------------------------------------------------------
 * @file ioif_10kohm_lp.c
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface implementation for the 10kΩ LP Potentiometer in L30 shoes.
 * 
 * This source file provides the functionality to detect voltage variations 
 * using the 10kΩ LP Potentiometer embedded in L30 shoes. As a wearer steps or 
 * applies pressure on the L30 shoe sole, the resistance changes, leading to 
 * voltage variations. This program reads these variations through ADC and 
 * processes them for further use.
 * 
 * @ref 10KOHM_LP_DATA_SHEET_XXXXXXX.pdf
 */

#include "ioif_10kohm_lp.h"

/** @defgroup ADC ADc
  * @brief ADC Linear Potentiometer 10Kohm module driver
  * @{
  */
#ifdef IOIF_LINEAR_POTEN_ENABLED

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

IOIF_LPState_t IOIF_CalLPVolt(IOIF_LP_t* lp, uint16_t adcBuff)
{
    if (!lp) {
        return IOIF_LP_NULL_POINTER;
    }
    
    if (adcBuff >= IOIF_ADC3_RESOLUTION) {
        return IOIF_LP_ADC_OUT_OF_RANGE;
    }

	lp->lpVolt = (float)adcBuff * IOIF_ADC3_VREF / IOIF_ADC3_RESOLUTION;

    return IOIF_LP_STATUS_OK;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* IOIF_LINEAR_POTEN_ENABLED */