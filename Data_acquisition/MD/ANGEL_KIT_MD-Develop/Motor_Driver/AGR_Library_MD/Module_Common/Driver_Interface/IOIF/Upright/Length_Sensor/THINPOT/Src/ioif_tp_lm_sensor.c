/**
 *-----------------------------------------------------------
 *               UPRIGHT LENGTH SENSOR INTERFACE
 *-----------------------------------------------------------
 * @file IOIF_TP_LM_SENSOR.c
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

#include "ioif_tp_lm_sensor.h"

/** @defgroup ADC ADc
  * @brief ADC ThinPot Length Sensor module driver
  * @{
  */
#ifdef IOIF_THINPOT_LS_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */
#define LPF_ALPHA   0.99   // LPF constant, adjust as needed




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

static float LPF(float currentValue, float previousFilteredValue);



/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_LSState_t IOIF_InitLS(IOIF_LS_t* ls, float userSlope, float offset)
{
    if (!ls) {
        return IOIF_LS_NULL_POINTER;
    }

    // Automatically set the scaling factor based on the active length and ADC range
    // TODO: Varies depending on Upright.
    ls->slope = userSlope;
    // Offset can be set to 0 or adjusted as needed
    ls->offset = offset;

    return IOIF_LS_STATUS_OK;
}

IOIF_LSState_t IOIF_SetLSOffset(IOIF_LS_t* ls, float offset)
{
    if (!ls) {
        return IOIF_LS_NULL_POINTER;
    }

    ls->offset = offset;

    return IOIF_LS_STATUS_OK;
}

static float LPF(float currentValue, float previousFilteredValue)
{
    float filteredValue;
    filteredValue = LPF_ALPHA * currentValue + (1 - LPF_ALPHA) * previousFilteredValue;
    return filteredValue;
}

float IOIF_GetLSmm(IOIF_LS_t* ls, uint32_t adcBuff)
{
    if (!ls) {
        return IOIF_LS_NULL_POINTER;
    }
    
    if (adcBuff >= IOIF_ADC3_RESOLUTION) {
        return IOIF_LS_ADC_OUT_OF_RANGE;
    }

    ls->mmData = (float)adcBuff * ls->slope + ls->offset;

    return ls->mmData;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* IOIF_THINPOT_LS_ENABLED */