/**
 *-----------------------------------------------------------
 *            		MOTOR TEMPERATURE USING NTC
 *-----------------------------------------------------------
 * @file ioif_503ntc.c
 * @date Created on: Aug 20, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface functions for the Motor Temperature Measurement using NTC.
 * 
 * This source file contains the implementation of the functions and routines 
 * for measuring the temperature of the motor using a Negative Temperature Coefficient (NTC) resistor.
 * It provides functionalities such as reading the ADC values, converting them 
 * to voltages, and further processing these voltages to derive temperature values.
 * Calibration and filtering mechanisms are also integrated to improve the accuracy 
 * and stability of the temperature readings.
 * 
 * For more detailed information on NTC temperature measurement and calibration, 
 * refer to the associated documentation and references.
 * 
 * @ref NTC Datasheet
 */

#include "ioif_503ntc.h"

/** @defgroup ADC ADc
  * @brief ADC 504 NTC Motor Temp Sensor module driver
  * @{
  */
#ifdef IOIF_503NTC_ENABLED

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

static float FirstLPF(float currVal, float prevVal, float alpha);
static IOIF_NTCState_t GetNTCVolt(IOIF_NTC_t* ntc, uint16_t adcBuff);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Retrieves the motor temperature using the ADC buffer value.
 * 
 * @param ntc Pointer to the NTC structure.
 * @param adcBuff ADC buffer value.
 * @return IOIF_NTCState_t status indicating the result of the operation.
 */
IOIF_NTCState_t IOIF_GetNTCTemp(IOIF_NTC_t* ntc, uint16_t adcBuff)
{
    // Get the NTC voltage and apply LPF
    uint8_t status = GetNTCVolt(ntc, adcBuff);

    if (status != IOIF_NTC_STATUS_OK) {
        return status;  // Return early if there was an error getting the voltage
    }

    // Calculate the NTC resistance based on the voltage across it and the known series resistance
    ntc->motorR = IOIF_NTC_R * (IOIF_ADC3_VREF / ntc->filtVolt - 1);

    // Now, calculate the temperature based on the derived NTC resistance
    ntc->motorTemp = 1 / (1 / IOIF_NTC_ROOM_TEMP + log(ntc->motorR / IOIF_NTC_R0) / IOIF_NTC_BETA_COEFF) + IOIF_NTC_KEL_TO_CEL;

    return IOIF_NTC_STATUS_OK;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Retrieves the NTC voltage and applies a LPF to it.
 * 
 * @param ntc Pointer to the NTC structure.
 * @param adcBuff ADC buffer value.
 * @return IOIF_NTCState_t status indicating the result.
 */
static IOIF_NTCState_t GetNTCVolt(IOIF_NTC_t* ntc, uint16_t adcBuff)
{
    // Validity checks
    if (!ntc) {
        return IOIF_NTC_STATUS_ERROR;  // Assuming you have an error status
    }
    
    // Convert ADC buffer value to voltage
    ntc->currVolt = (float)adcBuff * IOIF_ADC3_VREF / IOIF_ADC3_RESOLUTION - IOIF_NTC_MEASURED_OFFSET;

    // Apply the LPF
    ntc->filtVolt = FirstLPF(ntc->currVolt, ntc->prevVolt, IOIF_NTC_LPF_WEIGHT);
    
    // Update the previous voltage value for the next iteration
    ntc->prevVolt = ntc->filtVolt;
    
    return IOIF_NTC_STATUS_OK;  // Assuming you have a status indicating success
}


/**
 * @brief Applies a first-order low-pass filter to the input.
 * 
 * @param currValue Current input value.
 * @param prevValue Previous filtered value.
 * @param alpha Weight for the current value. (0 < alpha < 1)
 * @return Filtered value.
 */
static float FirstLPF(float currVal, float prevVal, float alpha)
{
    return alpha * currVal + (1.0 - alpha) * prevVal;
}


#endif /* IOIF_503NTC_ENABLED */
