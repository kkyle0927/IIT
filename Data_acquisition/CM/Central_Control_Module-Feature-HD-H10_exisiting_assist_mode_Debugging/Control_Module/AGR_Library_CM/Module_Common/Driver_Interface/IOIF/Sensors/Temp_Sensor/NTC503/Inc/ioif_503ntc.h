/**
 *-----------------------------------------------------------
 *            HEADER FOR MOTOR TEMPERATURE USING NTC
 *-----------------------------------------------------------
 * @file ioif_503ntc.h
 * @date Created on: Aug 20, 2023
 * @author AngelRobotics HW Team
 * @brief Header file for Motor Temperature Measurement using NTC.
 *
 * This header file provides the declarations and definitions required for
 * measuring the motor temperature using the NTC resistor. It defines the data structures,
 * constants, macros, and function prototypes that are used in the `ioif_ntc.c` implementation.
 *
 * The goal of this module is to provide a reliable and efficient way to monitor
 * and maintain motor temperature within safe operating limits.
 *
 * @note Ensure to include the necessary ADC and mathematical libraries for proper functionality.
 *
 * @ref NTC Datasheet
 */

#ifndef NTC503_INC_IOIF_503NTC_H_
#define NTC503_INC_IOIF_503NTC_H_

#include "module.h"

/** @defgroup ADC ADc
  * @brief ADC 504 NTC Motor Temp Sensor module driver
  * @{
  */
#ifdef IOIF_503NTC_ENABLED

#include <math.h>

#include "ioif_adc_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_NTC_MEASURED_OFFSET    0.1         // Offset value used to correct the measured voltage or temperature.
#define IOIF_NTC_LPF_WEIGHT         0.05        // Weightage for the current value in the low-pass filter (LPF).

#define IOIF_NTC_R                  51e3        // 51kΩ series resistor
#define IOIF_NTC_R0                 50e3        // Resistance value at reference temperature T0 (25°C): 50kohm
#define IOIF_NTC_ROOM_TEMP          298.15      // Room temperature in Kelvin used for NTC calculations.
#define IOIF_NTC_BETA_COEFF         4288.0      // Beta coefficient for the NTC resistor.
#define IOIF_NTC_KEL_TO_CEL         -273.15     // Offset to convert Kelvin to Celsius.


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_NTCState_t {
    IOIF_NTC_STATUS_OK = 0,
    IOIF_NTC_STATUS_ERROR,
} IOIF_NTCState_t;

typedef struct _IOIF_NTC_t{
    float   currVolt;   // Converted NTC ADC to Current Voltage.
    float   prevVolt;   // Converted NTC ADC to Previous Voltage.
	float 	filtVolt;   // Filtered NTC ADC to Voltage.
	float 	motorR;     // Motor Resistance
	float 	motorTemp;  // Motor Temperature
} IOIF_NTC_t;


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

IOIF_NTCState_t IOIF_GetNTCTemp(IOIF_NTC_t* ntc, uint16_t adcBuff);


#endif /* IOIF_503NTC_ENABLED */

#endif /* NTC503_INC_IOIF_503NTC_H_ */
