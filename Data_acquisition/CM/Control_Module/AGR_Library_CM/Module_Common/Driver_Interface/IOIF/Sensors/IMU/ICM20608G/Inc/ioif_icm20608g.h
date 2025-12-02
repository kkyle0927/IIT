/**
 *-----------------------------------------------------------
 *                 6AXIS ACC & GYRO IMU DRIVER
 *-----------------------------------------------------------
 * @file ioif_accgyro.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the accelerometer and gyroscope.
 *
 * This header file provides functionality to interface
 * with the accelerometer and gyroscope, including initialization,
 * data retrieval, and control register configurations.
 *
 */

#ifndef ICM20608G_INC_IOIF_ACCGYRO_H_
#define ICM20608G_INC_IOIF_ACCGYRO_H_

#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C accelerometer & gyroscope module driver
  * @{
  */
#ifdef IOIF_ACCGYRO_ENABLED

#include <string.h>

#include "ioif_i2c_common.h"
#include "icm20608g.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_ACCGYRO_BUFF_SIZE        32

#define IOIF_ACCGYRO_TRIALS           10
#define IOIF_ACCGYRO_STRAT_UP_DELAY   10
#define IOIF_ACCGYRO_TIMEOUT          1

#define IOIF_ACCGYRO_RECOVERY_TRAILS        3       // 3 Trials
#define IOIF_ACCGYRO_RECOVERY_TIME_INTERVAL 20      // 20ms

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the 6-axis IMU.
 */
typedef enum _IOIF_AccGyro_State_t {
    IOIF_ACCGYRO_STATUS_OK = 0,
    IOIF_ACCGYRO_STATUS_ERROR,
    IOIF_ACCGYRO_STATUS_BUSY,
    IOIF_ACCGYRO_STATUS_TIMEOUT,
} IOIF_AccGyro_State_t;

/**
 * @brief Structure to hold the data from the 6-axis IMU.
 */
typedef struct _IOIF_AccGyro_Data_t {
    float accX;
    float accY;
    float accZ;
    float gyrX;
    float gyrY;
    float gyrZ;
    float temp;
} IOIF_AccGyro_Data_t;


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

IOIF_AccGyro_State_t IOIF_AccGyro_Init(void);
IOIF_AccGyro_State_t IOIF_AccGyro_GetValue(IOIF_AccGyro_Data_t* imuData);


#endif /* IOIF_AccGyro_ENABLED */

#endif /* ICM20608G_INC_IOIF_AccGyro_H_ */
