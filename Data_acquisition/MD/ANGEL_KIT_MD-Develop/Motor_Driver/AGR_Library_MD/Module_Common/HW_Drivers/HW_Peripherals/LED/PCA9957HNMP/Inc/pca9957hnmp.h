/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file pca9957hnmp.h
 * @date Created on: Sep 14, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the PCA9957HNMP LED DRIVER.
 *
 * Refer to the PCA9957HNMP datasheet and related documents for more information.
 * @ref PCA9957HNMP Datasheet
 */

#ifndef PCA9957HNMP_INC_PCA9957HNMP_H_
#define PCA9957HNMP_INC_PCA9957HNMP_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "pca9957hnmp_regmap.h"

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
/* Callback pointer */
typedef bool (*PCA9957HNMP_read_fptr) (uint8_t reg, uint8_t value, uint8_t* rxdata);
typedef bool (*PCA9957HNMP_write_fptr) (uint8_t reg, uint8_t value);
typedef void (*FunctionPointer)(uint8_t, uint8_t);

typedef struct{
	PCA9957HNMP_read_fptr pca9957hnmp_read;
	PCA9957HNMP_write_fptr pca9957hnmp_write;
} PCA9957HNMP_CallbackStruct;



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

void PCA9957HNMP_Callback(PCA9957HNMP_CallbackStruct* led24ch_callback);

#endif /* PCA9957HNMP_INC_PCA9957HNMP_H_ */
