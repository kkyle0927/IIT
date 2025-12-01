/**
 *-----------------------------------------------------------
 *   BATTERY GAS GAUGE DRIVER HEADER
 *-----------------------------------------------------------
 * @file ioif_ltc2944.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 *
 * This header file provides the declarations for the functions and data structures required
 * to interact with the LTC2944 Battery Gas Gauge IC at a higher interface level.
 *
 * It offers an API that abstracts the hardware and BSP interactions, providing a simplified 
 * interface for task level applications to monitor and control the LTC2944 IC.
 */

#ifndef IOIF_BATMONITOR_H_
#define IOIF_BATMONITOR_H_

#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C Battery Monitor module driver
  * @{
  */
#ifdef IOIF_BATMONITOR_ENABLED

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

#define IOIF_BATMONITOR_BUFF_SIZE        32

#define IOIF_BATMONITOR_TRIALS           20
#define IOIF_BATMONITOR_STRAT_UP_DELAY   10
#define IOIF_BATMONITOR_TIMEOUT          50


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the LTC2944.
 */
typedef enum _IOIF_BatMonitor_State_t {
    IOIF_BATMONITOR_STATUS_OK = 0,
    IOIF_BATMONITOR_STATUS_ERROR,
    IOIF_BATMONITOR_STATUS_BUSY,
    IOIF_BATMONITOR_STATUS_TIMEOUT,
} IOIF_BatMonitor_State_t;

/**
 * @brief Structure to hold the data from the LTC2944.
 */
typedef struct _IOIF_BatMonitor_Data_t {
    float batVolt;     /// battery voltage
    float batCurr;     /// battery current
    float brdTemp;     /// board temperature - 편의상 battery 구조체에 포함
    float percentage;
} IOIF_BatMonitor_Data_t;


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

IOIF_BatMonitor_State_t IOIF_BatMonitor_Init(void);
IOIF_BatMonitor_State_t IOIF_BatMonitor_GetValue(IOIF_BatMonitor_Data_t* batData);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunLtc2944(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /* IOIF_BATMONITOR_ENABLED */
#endif /* IOIF_BATMONITOR_H_ */
