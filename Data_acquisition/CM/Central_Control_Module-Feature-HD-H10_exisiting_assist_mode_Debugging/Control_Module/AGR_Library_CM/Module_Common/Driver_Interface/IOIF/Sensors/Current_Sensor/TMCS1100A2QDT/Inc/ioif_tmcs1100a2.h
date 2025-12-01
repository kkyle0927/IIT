/**
 *-----------------------------------------------------------
 *             TMCS1100A2 CURRENT SENSOR INTERFACE
 *-----------------------------------------------------------
 * @file ioif_tmcs1100a2.h
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

#ifndef TMCS1100A2QDT_INC_IOIF_TMCS1100A2_H_
#define TMCS1100A2QDT_INC_IOIF_TMCS1100A2_H_

#include "module.h"

/** @defgroup ADC ADc
  * @brief ADC TMCS1100A2 Current Sensor module driver
  * @{
  */
#ifdef IOIF_TMCS1100A2_ENABLED

#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_CS2_VOLTAGE        3.3     // V
#define IOIF_CS2_SENSITIVITY    0.100   // V/A
#define IOIF_CS2_FULL_RANGE     29      // in Amphere +- 14.5A
#define IOIF_CS2_MAX            14.5    // A

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_CS2State_t {
    IOIF_CS2_STATUS_OK = 0,
	IOIF_CS2_STATUS_ERROR,
} IOIF_CS2State_t;


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




#endif /* IOIF_TMCS1100A2_ENABLED */

#endif /* TMCS1100A2QDT_INC_IOIF_TMCS1100A2_H_ */
