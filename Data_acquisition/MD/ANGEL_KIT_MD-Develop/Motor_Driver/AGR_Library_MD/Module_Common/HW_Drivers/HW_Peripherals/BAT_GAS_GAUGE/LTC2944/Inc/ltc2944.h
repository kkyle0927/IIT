/**
 *-----------------------------------------------------------
 *          uint8_t BATTERY GAS GAUGE DRIVER HEADER
 *-----------------------------------------------------------
 * @file ltc2944.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief uint8_t Battery Gas Gauge Driver Interface.
 *
 * This header file defines the interface for the uint8_t Battery Gas Gauge IC.
 * It declares the functions, types, and constants needed to interact with the device.
 * Together with the corresponding implementation in ltc2944.c, it enables monitoring
 * and managing the battery status, including voltage, current, and temperature.
 *
 * The driver interface includes initialization, configuration, and data retrieval functions,
 * allowing for flexible integration into various system architectures.
 *
 * @ref uint8_t Datasheet
 */

#ifndef LTC2944_INC_LTC2944_H_
#define LTC2944_INC_LTC2944_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ltc2944_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define LTC2944_READ_DATA_SIZE          24

#define LTC2944_TOTAL_ARGUMENT          3

#define LTC2944_TRIALS                  10   ///< Number of trials for communication attempts.
#define LTC2944_STRAT_UP_DELAY          10   ///< Startup delay in milliseconds.
#define LTC2944_DATA_TIMEOUT            1    ///< Data timeout in milliseconds.

#define LTC2944_FULLSCALE_VOLTAGE       70.8 ///< Full-scale voltage in volts.
#define LTC2944_FULLSCALE_CURRENT       64   ///< Full-scale current in amperes.
#define LTC2944_FULLSCALE_TEMPERATURE   510  ///< Full-scale temperature in Kelvin.


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Function pointer types for initialization, deinitialization, and I/O operations.
typedef uint8_t (*LTC2944_InitFunc)       (void);
typedef uint8_t (*LTC2944_DeInitFunc)     (void);
typedef uint8_t (*LTC2944_IsDevReadyFunc) (uint16_t); // Device address
typedef uint8_t (*LTC2944_ReadRegFunc)    (uint16_t, uint16_t, uint8_t* , uint16_t); // Device address, Register address, Data, Data size
typedef uint8_t (*LTC2944_WriteRegFunc)   (uint16_t, uint16_t, uint8_t* , uint16_t); // Device address, Register address, Data, Data size
typedef uint8_t (*LTC2944_WaitFunc)       (uint32_t); // Milliseconds to Wait

// Enumeration for LTC2944 status codes.
typedef enum _LTC29444State_t {
    LTC2944_STATUS_OK = 0,
    LTC2944_STATUS_ERROR,
    LTC2944_STATUS_BUSY,
    LTC2944_STATUS_TIMEOUT,
} LTC29444State_t;

// I/O context structure for LTC2944.
typedef struct _LTC2944IOctx_t {
    LTC2944_InitFunc       Init;       ///< Initialization function.
    LTC2944_DeInitFunc     DeInit;     ///< De-initialization function.
    LTC2944_IsDevReadyFunc IsDevReady; ///< Function to check if the device is ready.
    LTC2944_ReadRegFunc    ReadReg;    ///< Function to read from a register.
    LTC2944_WriteRegFunc   WriteReg;   ///< Function to write to a register.
    LTC2944_WaitFunc       Wait;       ///< Function to Wait for a specified time.
} LTC2944IOctx_t;

// Data structure for LTC2944 Data.
typedef struct _LTC2944Data_t {
    float batVol;      ///< Voltage value.
    float batCurr;     ///< Current value.
    float brdTemp;     ///< Temperature value.
} LTC2944Data_t;

// Object handle structure for LTC2944.
typedef struct _LTC2944Object_t {
    uint16_t devAddr;               // Device address
    uint8_t isInit;         // Initialization status
    uint8_t* dataBuff;              // Data Buffer
    LTC2944Data_t batData;           // User data
    LTC2944IOctx_t io;              // I/O context.
} LTC2944Obj_t;



/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 * Add external global variable declarations here, if needed.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

// Set the I/O context for the uint8_t object.
uint8_t LTC2944_SetIoctx(LTC2944Obj_t* ltc2944_obj, LTC2944IOctx_t* ltc2944IOctx); 

// Initialize the uint8_t object.
uint8_t LTC2944_Init(LTC2944Obj_t* ltc2944_obj); 

// Retrieve the battery data values from the uint8_t object.
uint8_t LTC2944_GetValue(LTC2944Obj_t* ltc2944_obj);


#endif /* LTC2944_INC_LTC2944_H_ */
