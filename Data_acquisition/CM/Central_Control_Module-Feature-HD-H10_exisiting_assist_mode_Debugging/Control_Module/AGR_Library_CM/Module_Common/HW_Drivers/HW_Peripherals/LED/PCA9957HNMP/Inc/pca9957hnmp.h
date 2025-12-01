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

typedef enum _PCA9957HNMP_SPIOP_t {
    SPI_TRANSMIT,
    SPI_RECEIVE,
    SPI_TRANSMIT_RECEIVE,
    SPI_TRANSMIT_IT,
    SPI_RECEIVE_IT,
    SPI_TRANSMIT_RECEIVE_IT,
    SPI_TRANSMIT_DMA,
    SPI_RECEIVE_DMA,
    SPI_TRANSMIT_RECEIVE_DMA,
} PCA9957HNMP_SPIOP_t;

/* Callback pointer */
typedef bool (*PCA9957HNMP_spi_operation) (uint8_t reg, uint8_t tdata, uint8_t* rdata, uint8_t timeout, PCA9957HNMP_SPIOP_t operation);
typedef bool (*PCA9957HNMP_nOE_control)   (bool pin_state);
typedef bool (*PCA9957HNMP_nRST_control)  (bool pin_state);

typedef struct{
	PCA9957HNMP_spi_operation pca9957hnmp_spi_op;
	PCA9957HNMP_nOE_control   pca9957hnmp_nOE_ctrl;
	PCA9957HNMP_nRST_control  pca9957hnmp_nRST_ctrl;
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
bool PCA9957HNMP_InitDrv(void);

bool PCA9957HNMP_WriteReg(uint8_t reg, uint8_t data);
bool PCA9957HNMP_ReadReg(uint8_t reg, uint8_t data, uint8_t* rxdata);

bool PCA9957HNMP_RegisterCallback(PCA9957HNMP_CallbackStruct* led24ch_callback);

#endif /* PCA9957HNMP_INC_PCA9957HNMP_H_ */
