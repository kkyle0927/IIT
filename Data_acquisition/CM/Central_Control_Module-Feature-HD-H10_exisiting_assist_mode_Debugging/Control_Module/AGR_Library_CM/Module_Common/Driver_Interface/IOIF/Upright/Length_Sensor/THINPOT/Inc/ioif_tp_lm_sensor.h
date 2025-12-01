/**
 *-----------------------------------------------------------
 *               UPRIGHT LENGTH SENSOR INTERFACE
 *-----------------------------------------------------------
 * @file IOIF_TP_LM_SENSOR.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface header for the Upright Length Sensor (Thinpot).
 * 
 * This header file provides the interface definitions for the 
 * Upright Length Sensor, often referred to as Thinpot. The interface 
 * includes macros, type declarations, and function prototypes 
 * essential for converting ADC values to real-world measurements 
 * in millimeters.
 * 
 * @ref THINPOT_DATA_SHEET_Rev_B-2949406.pdf
 */

#ifndef THINPOT_INC_IOIF_TP_LM_SENSOR_H_
#define THINPOT_INC_IOIF_TP_LM_SENSOR_H_

#include "module.h"

/** @defgroup ADC ADc
  * @brief ADC ThinPot Length Sensor module driver
  * @{
  */
#ifdef IOIF_THINPOT_LS_ENABLED

#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_TP_RESISTANCE		10e3	// ohm
#define IOIF_TP_ACTIVE_LENGTH	100		// mm


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration for the Length Sensor state/status.
 */
typedef enum _IOIF_LSState_t {
    IOIF_LS_STATUS_OK = 0,
    IOIF_LS_NULL_POINTER,
    IOIF_LS_ADC_OUT_OF_RANGE,
} IOIF_LSState_t;

/**
 * @brief Structure for the Length Sensor parameters and data.
 */
typedef struct _IOIF_LS_t {
    float slope;
    float offset;
    float mmData;
} IOIF_LS_t;


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

IOIF_LSState_t IOIF_InitLS(IOIF_LS_t* ls, float userSlope, float offset);
IOIF_LSState_t IOIF_SetLSOffset(IOIF_LS_t* ls, float offset);
float IOIF_GetLSmm(IOIF_LS_t* ls, uint32_t adcBuff);


#endif /* IOIF_THINPOT_LS_ENABLED */

#endif /* THINPOT_INC_IOIF_TP_LM_SENSOR_H_ */
