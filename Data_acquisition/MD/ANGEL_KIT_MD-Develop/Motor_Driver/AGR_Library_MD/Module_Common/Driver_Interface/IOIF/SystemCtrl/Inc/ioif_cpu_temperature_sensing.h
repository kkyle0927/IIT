
/**
 *-----------------------------------------------------------
 *             CPU Temperature Sense & Calculate
 *-----------------------------------------------------------
 * @file ioif_cpu_temperature_sensing.h
 * @date Created on: Nov 2, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#ifndef IOIF_BSP_HEADER_BSP_CPU_TEMPERATURE_SENSING_H_
#define IOIF_BSP_HEADER_BSP_CPU_TEMPERATURE_SENSING_H_

#include "module.h"
/** @defgroup ADC ADC
  * @brief ADC BSP module driver
  * @{
  */
#ifdef IOIF_ICM20608G_ENABLED

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
uint32_t IOIF_ReadCPUTemp(uint16_t* adcBuff);


#endif /* IOIF_ICM20608G_ENABLED */
#endif /* IOIF_BSP_HEADER_BSP_CPU_TEMPERATURE_SENSING_H_ */
