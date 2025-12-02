/**
 *-----------------------------------------------------------
 *   LTC2944 IO INTERFACE BATTERY GAS GAUGE DRIVER HEADER
 *-----------------------------------------------------------
 * @file ioif_ltc2944.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief LTC2944 IO Interface Driver Header.
 *
 * This header file provides the declarations for the functions and data structures required
 * to interact with the LTC2944 Battery Gas Gauge IC at a higher interface level.
 *
 * It offers an API that abstracts the hardware and BSP interactions, providing a simplified 
 * interface for task level applications to monitor and control the LTC2944 IC.
 *
 * @ref LTC2944 Datasheet
 */

#ifndef LTC2944_INC_IOIF_LTC2944_H_
#define LTC2944_INC_IOIF_LTC2944_H_

#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C Battery Monitor module driver
  * @{
  */
#ifdef IOIF_LTC2944_ENABLED

#include "ioif_i2c_common.h"
#include "ltc2944.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_BATTERY_VOLT_MAX 48.0
#define IOIF_BATTERY_VOLT_MIN 42.0

#define IOIF_LTC2944_BUFF_SIZE        32

#define IOIF_LTC2944_TRIALS           20
#define IOIF_LTC2944_STRAT_UP_DELAY   10
#define IOIF_LTC2944_TIMEOUT          50


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the LTC2944.
 */
typedef enum _IOIF_BatState_t {
    IOIF_BAT_STATUS_OK = 0,
    IOIF_BAT_STATUS_ERROR,
    IOIF_BAT_STATUS_BUSY,
    IOIF_BAT_STATUS_TIMEOUT,
} IOIF_BatState_t;

/**
 * @brief Structure to hold the data from the LTC2944.
 */
typedef struct _IOIF_BatData_t {
    float batVol;      ///< Voltage value.
    float batCurr;     ///< Current value.
    float brdTemp;     ///< Temperature value.
    float percentage;
} IOIF_BatData_t;


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

IOIF_BatState_t IOIF_InitBat(IOIF_I2C_t i2c);
IOIF_BatState_t IOIF_GetBatValue(IOIF_BatData_t* batData);


#endif /* IOIF_LTC2944_ENABLED */

#endif /* LTC2944_INC_IOIF_LTC2944_H_ */
