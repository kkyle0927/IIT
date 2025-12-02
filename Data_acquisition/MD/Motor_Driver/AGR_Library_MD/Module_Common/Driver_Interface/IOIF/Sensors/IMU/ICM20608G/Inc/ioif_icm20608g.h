/**
 *-----------------------------------------------------------
 *                 6AXIS ACC & GYRO IMU DRIVER
 *-----------------------------------------------------------
 * @file ioif_icm20608g.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the ICM20608G accelerometer and gyroscope.
 *
 * This header file provides functionality to interface
 * with the ICM20608G accelerometer and gyroscope, including initialization,
 * data retrieval, and control register configurations.
 * 
 * Refer to the ICM20608G datasheet and related documents for more information.
 *
 * @ref ICM20608G Datasheet
 */

#ifndef ICM20608G_INC_IOIF_ICM20608G_H_
#define ICM20608G_INC_IOIF_ICM20608G_H_

#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C ICM20608G module driver
  * @{
  */
#ifdef IOIF_ICM20608G_ENABLED

#include <string.h>

#include "ioif_i2c_common.h"
#include "icm20608g.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_ICM20608G_BUFF_SIZE        32

#define IOIF_ICM20608G_TRIALS           10
#define IOIF_ICM20608G_STRAT_UP_DELAY   10
#define IOIF_ICM20608G_TIMEOUT          1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the 6-axis IMU.
 */
typedef enum _IOIF_6AxisState_t {
    IOIF_IMU6AXIS_STATUS_OK = 0,
    IOIF_IMU6AXIS_STATUS_ERROR,
    IOIF_IMU6AXIS_STATUS_BUSY,
    IOIF_IMU6AXIS_STATUS_TIMEOUT,
} IOIF_6AxisState_t;

/**
 * @brief Structure to hold the data from the 6-axis IMU.
 */
typedef struct _IOIF_6AxisData_t {
    float accX;
    float accY;
    float accZ;
    float gyrX;
    float gyrY;
    float gyrZ;
    float temp;
} IOIF_6AxisData_t;


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

IOIF_6AxisState_t IOIF_Init6Axis(IOIF_I2C_t i2c);
IOIF_6AxisState_t IOIF_Get6AxisValue(IOIF_6AxisData_t* imuData);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunICM20608G(cli_args_t *args);
#endif /* _USE_DEBUG_CLI */

#endif /* IOIF_ICM20608G_ENABLED */

#endif /* ICM20608G_INC_IOIF_ICM20608G_H_ */
