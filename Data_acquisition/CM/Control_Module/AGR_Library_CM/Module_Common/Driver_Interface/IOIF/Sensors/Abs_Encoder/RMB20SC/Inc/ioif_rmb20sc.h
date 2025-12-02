/**
 *-----------------------------------------------------------
 *           		RMB20SC ABSOLUTE ENCODER
 *-----------------------------------------------------------
 * @file ioif_rmb20sc.h
 * @date Created on: Aug 20, 2023
 * @author AngelRobotics HW Team
 * @brief Header file for IO Interface functions for the RMB20SC Absolute Encoder.
 * 
 * Contains function prototypes, type definitions, and constants related 
 * to the RMB20SC absolute encoder interface.
 * 
 * @ref RMB20SC Datasheet
 */

#ifndef RMB20SC_INC_IOIF_RMB20SC_H_
#define RMB20SC_INC_IOIF_RMB20SC_H_

#include "module.h"

/** @defgroup SPI SPI
  * @brief SSI RMB20SC Absolute Encoder module driver
  * @{
  */
#ifdef IOIF_RMB20SC_ENABLED

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "ioif_spi_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_ABS_RESOLUTION 0x1FFF

#define IOIF_SPI1_READ_SIZE 1
#define IOIF_SPI3_READ_SIZE 1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_AbsSelect_t {
	IOIF_ABS_ENC_NONE = (uint8_t)0,
	IOIF_ABS_ENC_ACTUATOR_INPUT,  // absolute encoder at reducer input
	IOIF_ABS_ENC_ACTUATOR_OUTPUT, // absolute encoder at reducer output
	IOIF_ABS_ENC_JOINT1,
	IOIF_ABS_ENC_JOINT2,
} IOIF_AbsSelect_t;

typedef enum _IOIF_AbsState_t {
    IOIF_ABSENC_STATUS_OK = (uint8_t)0,
    IOIF_ABSENC_STATUS_NULL_PTR,
	IOIF_ABSENC_STATUS_INVALID_SPI,
	IOIF_ABSENC_STATUS_INVALID_RESOLUTION,
	IOIF_ABSENC_STATUS_INVALID_SAMPL_FREQ,
	IOIF_ABSENC_STATUS_INIT_ERR,
	IOIF_ABSENC_NO_PARAM,
} IOIF_AbsState_t;

 typedef struct _IOIF_AbsEnc_t {

	IOIF_AbsSelect_t location;

	float absDegPrev;
 	float offset;
 	float posDegRaw;			// in deg
 	float posDegMultiTurn;		// in deg
 	float posDeg;				// subtracted by offset, (deg)
 	float velDeg;				// deg/s
 	float samplFreq;

	uint16_t* absBit;
 	uint16_t resolution;

 	int16_t absTurn;

 	int8_t sign;

	uint8_t id;
 } IOIF_AbsEnc_t;


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

IOIF_AbsState_t IOIF_InitAbsEnc(IOIF_SPI_t spi, uint8_t channel, IOIF_AbsEnc_t* absEnc, IOIF_AbsSelect_t absId, uint32_t resolution, float samplFreq);
IOIF_AbsState_t IOIF_SetAbsOffset(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc);
IOIF_AbsState_t IOIF_SetAbsSign(IOIF_AbsEnc_t* absEnc);
IOIF_AbsState_t IOIF_GetPosVelDeg(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc);


#endif /* IOIF_RMB20SC_ENABLED */

#endif /* RMB20SC_INC_IOIF_RMB20SC_H_ */
