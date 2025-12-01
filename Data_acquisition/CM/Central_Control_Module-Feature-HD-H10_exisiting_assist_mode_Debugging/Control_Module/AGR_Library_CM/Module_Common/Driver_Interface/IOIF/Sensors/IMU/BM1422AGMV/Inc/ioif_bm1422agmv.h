/**
*-----------------------------------------------------------
*                    3AXIS MAG IMU DRIVER
*-----------------------------------------------------------
* @file ioif_magnetometer.h
* @date Created on: Jul 28, 2023
* @author AngelRobotics HW Team
* @brief Driver code for the magnetometer.
*
* This header file provides functionality to interface
* with the BM1422AGMV magnetometer, including initialization,
* data retrieval, and control register configurations.
*
*/

#ifndef IOIF_MAGNETO_H_
#define IOIF_MAGNETO_H_

#include "module.h"

/** @defgroup I2C I2C
  * @brief I2C magnetometer module driver
  * @{
  */
#ifdef IOIF_MAGNETO_ENABLED

#include <string.h>

#include "ioif_i2c_common.h"
#include "bm1422agmv.h"

/**
*-----------------------------------------------------------
*              MACROS AND PREPROCESSOR DIRECTIVES
*-----------------------------------------------------------
* @brief Directives and macros for readability and efficiency.
*/

#define IOIF_MAGNETO_BUFF_SIZE        32

#define IOIF_MAGNETO_TRIALS           10
#define IOIF_MAGNETO_STRAT_UP_DELAY   10
#define IOIF_MAGNETO_TIMEOUT          1


/**
*------------------------------------------------------------
*                     TYPE DECLARATIONS
*------------------------------------------------------------
* @brief Custom data types and structures for the module.
*/

/**
* @brief Enumeration to describe the state of the 6-axis IMU.
*/
typedef enum _IOIF_Magneto_State_t {
   IOIF_MAGNETO_STATUS_OK = 0,
   IOIF_MAGNETO_STATUS_ERROR,
   IOIF_MAGNETO_STATUS_BUSY,
   IOIF_MAGNETO_STATUS_TIMEOUT,
} IOIF_Magneto_State_t;

/**
* @brief Structure to hold the data from the 6-axis IMU.
*/
typedef struct _IOIF_Magneto_Data_t {
   float magX;
   float magY;
   float magZ;
} IOIF_Magneto_Data_t;


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

IOIF_Magneto_State_t IOIF_Magneto_Init(void);
uint8_t IOIF_Magneto_GetValue(IOIF_Magneto_Data_t* magData);


#endif /* IOIF_MAGNETO_ENABLED */

#endif /* IOIF_MAGNETO_H_ */
