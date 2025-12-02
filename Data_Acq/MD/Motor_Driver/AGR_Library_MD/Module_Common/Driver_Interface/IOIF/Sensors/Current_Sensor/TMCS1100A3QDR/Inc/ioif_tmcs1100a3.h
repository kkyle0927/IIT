/**
 *-----------------------------------------------------------
 *             TMCS1100A3 CURRENT SENSOR INTERFACE
 *-----------------------------------------------------------
 * @file ioif_tmcs1100a3.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface header for the TMCS1100A Current Sensor.
 * 
 * This header file provides the interface definitions for the 
 * TMCS1100A Current Sensor. The interface includes macros, 
 * type declarations, and function prototypes essential for detecting 
 * and processing current feedback in BLDC motor control applications.
 * Sensitivity and current measurement range are the key attributes 
 * that make this sensor crucial for motor driver current control.
 * 
 * @ref TMCS1100A Datasheet
 */

#ifndef TMCS1100A3QDR_INC_IOIF_TMCS1100A3_H_
#define TMCS1100A3QDR_INC_IOIF_TMCS1100A3_H_

#include "module.h"

/** @defgroup ADC ADc
  * @brief ADC TMCS1100A3 Current Sensor module driver
  * @{
  */
#ifdef IOIF_TMCS1100A3_ENABLED

#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_CS3_VOLTAGE        3.3     // V
#define IOIF_CS3_SENSITIVITY    0.200   // V/A
#define IOIF_CS3_FULL_RANGE     14.5    // in Amphere +- 7.25A
#define IOIF_CS3_MAX            7.25    // A


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_CS3State_t {
    IOIF_CS3_STATUS_OK = 0,
	IOIF_CS3_STATUS_ERROR,
} IOIF_CS3State_t;


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




#endif /* IOIF_TMCS1100A3_ENABLED */

#endif /* TMCS1100A3QDR_INC_IOIF_TMCS1100A3_H_ */
