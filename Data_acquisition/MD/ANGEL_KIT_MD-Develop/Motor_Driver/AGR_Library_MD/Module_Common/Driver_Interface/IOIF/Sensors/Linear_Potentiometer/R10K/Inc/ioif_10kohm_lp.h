/**
 *-----------------------------------------------------------
 *         10KOHM LP - L30 SHOE POTENTIOMETER INTERFACE
 *-----------------------------------------------------------
 * @file ioif_10kohm_lp.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface header for the 10kΩ LP Potentiometer in L30 shoes.
 * 
 * This header file provides the interface definitions for the 
 * 10kΩ LP Potentiometer embedded in L30 shoes. The interface includes macros, 
 * type declarations, and function prototypes essential for detecting and 
 * processing voltage variations due to pressure changes on the L30 shoe sole.
 * 
 * @ref 10KOHM_LP_DATA_SHEET_XXXXXXX.pdf
 */

#ifndef R10K_INC_IOIF_10KOHM_LP_H_
#define R10K_INC_IOIF_10KOHM_LP_H_

#include "module.h"

/** @defgroup ADC ADc
  * @brief ADC Linear Potentiometer 10Kohm module driver
  * @{
  */
#ifdef IOIF_LINEAR_POTEN_ENABLED

#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_LPState_t {
    IOIF_LP_STATUS_OK = 0,
	IOIF_LP_NULL_POINTER,
    IOIF_LP_ADC_OUT_OF_RANGE,
} IOIF_LPState_t;

typedef struct _IOIF_LP_t { 
	float lpVolt;        
} IOIF_LP_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

IOIF_LPState_t IOIF_CalLPVolt(IOIF_LP_t* lp, uint16_t adcBuff);


#endif /* IOIF_LINEAR_POTEN_ENABLED */

#endif /* R10K_INC_IOIF_10KOHM_LP_H_ */
