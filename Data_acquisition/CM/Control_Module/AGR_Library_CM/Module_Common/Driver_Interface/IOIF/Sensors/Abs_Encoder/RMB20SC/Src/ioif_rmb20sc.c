/**
 *-----------------------------------------------------------
 *            		RMB20SC ABSOLUTE ENCODER
 *-----------------------------------------------------------
 * @file ioif_rmb20sc.c
 * @date Created on: Aug 20, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface functions for the RMB20SC Absolute Encoder.
 * 
 * This source file contains the implementation of the interface functions 
 * for the RMB20SC absolute encoder. It provides functionalities such as 
 * initialization, reading encoder bits, setting offset, setting sign, and 
 * calculating position and velocity in degrees.
 * 
 * For detailed operation and configurations, refer to the RMB20SC datasheet 
 * and related documentation.
 * 
 * @ref RMB20SC Datasheet
 */

#include "ioif_rmb20sc.h"

/** @defgroup SPI SPI
  * @brief SSI RMB20SC Absolute Encoder module driver
  * @{
  */
#ifdef IOIF_RMB20SC_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

#ifdef WALKON5_CM_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi4RxBuff"))) = 0;  // Hip Abs Encdoer
#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi4RxBuff"))) = 0;  // Hip Abs Encdoer
#endif /* L30_CM_ENABLED */

#ifdef L30_MD_REV06_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi1RxBuff"))) = 0;  // Motor Absolute Encoder
static uint16_t rmb20scDmaRxBuff2 __attribute__((section(".spi3RxBuff"))) = 0;  // Ankle Absolute Encoder
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi1RxBuff"))) = 0;  // Motor Absolute Encoder
static uint16_t rmb20scDmaRxBuff2 __attribute__((section(".spi3RxBuff"))) = 0;  // Ankle Absolute Encoder
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi1RxBuff"))) = 0;  // Motor Absolute Encoder
static uint16_t rmb20scDmaRxBuff2 __attribute__((section(".spi3RxBuff"))) = 0;  // Ankle Absolute Encoder
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MINICM_ENABLED 
//  not in use
#endif /* SUIT_MINICM_ENABLED */

#ifdef SUIT_MD_ENABLED
static uint16_t rmb20scDmaRxBuff1 __attribute__((section(".spi1RxBuff"))) = 0;  // Motor Absolute Encoder
static uint16_t rmb20scDmaRxBuff2 __attribute__((section(".spi3RxBuff"))) = 0;  // Ankle Absolute Encoder
#endif /* SUIT_MD_ENABLED */

#ifdef SUIT_WIDM_ENABLED 
//  not in use
#endif /* SUIT_WIDM_ENABLED */


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static bool IsValAbsEnc(IOIF_AbsEnc_t* absEnc);
static bool IsValSPI(IOIF_SPI_t spi);
static bool IsValResolution(uint32_t resolution);
static bool IsValSamplFreq(float samplFreq);
static IOIF_AbsState_t ValAbsParams(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc, uint32_t resolution, float samplFreq);

static IOIF_SPIState_t ReadAbsBit(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc);
static IOIF_SPIState_t CalPosVelDeg(IOIF_AbsEnc_t* absEnc);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/** IOIF_AbsState_t IOIF_InitAbsEnc
 * 
 * Initializes the absolute encoder configuration.
 *
 * @param spi: SPI interface (either IOIF_SPI1 or IOIF_SPI3)
 * @param absEnc: Pointer to an IOIF_AbsEnc_t structure which will be configured
 * @param resolution: Resolution of the encoder
 * @param samplFreq: Sample frequency
 *
 * @return: IOIF_ABSENC_STATUS_OK when the initialization is successful
 */
IOIF_AbsState_t IOIF_InitAbsEnc(IOIF_SPI_t spi, uint8_t channel, IOIF_AbsEnc_t* absEnc, IOIF_AbsSelect_t absId, uint32_t resolution, float samplFreq)
{
    IOIF_AbsState_t status = ValAbsParams(spi, absEnc, resolution, samplFreq);
    if (status != IOIF_ABSENC_STATUS_OK) {
        return status;
    }

    // Check which SPI interface is provided and set the appropriate DMA buffer
    /*switch (absId) {
        #ifdef L30_CM_ENABLED
        case IOIF_HIP_ABS_END:
            absEnc->absBit = &rmb20scDmaRxBuff1;
            break;
        #endif */
    /* L30_CM_ENABLED */
        //#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED) || defined (SUIT_MD_ENABLED)
/*
            if (channel == 1) {
            	absEnc->location = absId;
            	absEnc->absBit = &rmb20scDmaRxBuff1;
            }

            else if (channel == 2) {
            	absEnc->location = absId;
            	absEnc->absBit = &rmb20scDmaRxBuff2;
            }
*/
        //#endif /* L30_MD_REV06_ENABLED OR L30_MD_REV07_ENABLED OR SUIT_MD_ENABLED */

            /*
        default:
            // This should be unreachable due to the validation above, but added for safety.
            return IOIF_ABSENC_STATUS_INVALID_SPI;
    }
    */

    if ((absEnc->location != IOIF_ABS_ENC_NONE) &&
    	(absEnc->location != IOIF_ABS_ENC_ACTUATOR_INPUT) &&
		(absEnc->location != IOIF_ABS_ENC_ACTUATOR_OUTPUT) &&
		(absEnc->location != IOIF_ABS_ENC_JOINT1) &&
		(absEnc->location != IOIF_ABS_ENC_JOINT2))
    {
    	absEnc->location = IOIF_ABS_ENC_NONE;
    }

    if (channel == 1) {
    	//absEnc->location = absId;
    	absEnc->absBit = &rmb20scDmaRxBuff1;
    }

    else if (channel == 2) {
    	//absEnc->location = absId;
    	absEnc->absBit = &rmb20scDmaRxBuff2;
    }


    // Set the resolution and sample frequency
    absEnc->resolution = resolution;
    absEnc->samplFreq = samplFreq;

    // If sign is neither 1 nor -1, set it to 1
    if ((absEnc->sign != 1) && (absEnc->sign != -1)) {
        absEnc->sign = 1;
    }

    if (isnan(absEnc->offset))
    	absEnc->offset = 0;

    return status;
}

/**
 * @brief Sets the offset for the absolute encoder based on its current position.
 * 
 * Reads the current position from the encoder and uses it to set the offset value.
 * 
 * @param spi The SPI interface used for communication.
 * @param absEnc Pointer to the absolute encoder structure.
 * @return IOIF_ABSENC_STATUS_OK if successful, or an error status otherwise.
 */
IOIF_AbsState_t IOIF_SetAbsOffset(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc)
{
	// Validate the SPI using previously defined function
    if (!IsValSPI(spi)) {
        return IOIF_ABSENC_STATUS_INVALID_SPI;
    }

    // Validate the absEnc pointer using previously defined function
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }
	
    uint16_t tAbsBits = 0;

    IOIF_AbsState_t status = ReadAbsBit(spi, absEnc);
    if (status != IOIF_ABSENC_STATUS_OK) {
        return status; // Return the error from the read operation
    }

    tAbsBits = (*(absEnc->absBit)) & (absEnc->resolution);
    absEnc->offset = (float)tAbsBits * (360.0 / (float)absEnc->resolution);  // Convert to 0 ~ 360 range
    absEnc->absTurn = 0;

    return status;
}

/**
 * @brief Sets the sign of the absolute encoder and resets the turn count.
 * 
 * If the current sign is not set or invalid, it defaults to 1. Otherwise, the sign is toggled.
 * 
 * @param absEnc Pointer to the absolute encoder structure.
 * @return IOIF_ABSENC_STATUS_OK if successful, or IOIF_ABSENC_STATUS_NULL_PTR if absEnc is NULL.
 */
IOIF_AbsState_t IOIF_SetAbsSign(IOIF_AbsEnc_t* absEnc)
{
    // Validate the absEnc pointer using previously defined function
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }

    // Check and set the sign of the encoder
    if (absEnc->sign != 1 && absEnc->sign != -1) {
        absEnc->sign = 1;
    } else {
        absEnc->sign *= -1;
    }

    // Reset the absolute turn count
    absEnc->absTurn = 0;

    return IOIF_ABSENC_STATUS_OK;
}

IOIF_AbsState_t IOIF_GetPosVelDeg(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc)
{
	if (absEnc->location != IOIF_ABS_ENC_NONE) {
		ReadAbsBit(spi,absEnc);
		CalPosVelDeg(absEnc);
	} else {
		return IOIF_ABSENC_STATUS_INVALID_SPI;
	}
	return IOIF_ABSENC_STATUS_OK;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Checks the validity of the provided absolute encoder pointer.
 * 
 * @param absEnc Pointer to the absolute encoder structure.
 * @return true if absEnc is non-NULL, false otherwise.
 */
static bool IsValAbsEnc(IOIF_AbsEnc_t* absEnc)
{
	return absEnc != NULL;
}

/**
 * @brief Checks the validity of the provided SPI value.
 * 
 * @param spi SPI interface value to check.
 * @return true if spi is one of the valid SPI options, false otherwise.
 */
static bool IsValSPI(IOIF_SPI_t spi) 
{
    return spi == IOIF_SPI1 || spi == IOIF_SPI3;
}

/**
 * @brief Checks the validity of the provided resolution value.
 * 
 * @param resolution Resolution value to check.
 * @return true if resolution is greater than 0, false otherwise.
 */
static bool IsValResolution(uint32_t resolution) 
{
    return resolution > 0;
}

/**
 * @brief Checks the validity of the provided sample frequency value.
 * 
 * @param samplFreq Sample frequency value to check.
 * @return true if samplFreq is greater than 0, false otherwise.
 */
static bool IsValSamplFreq(float samplFreq) 
{
    return samplFreq > 0;
}

/**
 * @brief Validates the provided parameters for the absolute encoder initialization.
 * 
 * @param spi SPI interface value.
 * @param absEnc Pointer to the absolute encoder structure.
 * @param resolution Resolution of the encoder.
 * @param samplFreq Sample frequency of the encoder.
 * @return IOIF_ABSENC_STATUS_OK if all parameters are valid, or a specific error status otherwise.
 */
static IOIF_AbsState_t ValAbsParams(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc, uint32_t resolution, float samplFreq) 
{    
    // Validate SPI if it's not marked as NO_PARAM
    if (!IsValSPI(spi)) {
        return IOIF_ABSENC_STATUS_INVALID_SPI;
    }

    // Validate the absEnc pointer
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }

    // Validate resolution if it's not marked as NO_PARAM
    if (resolution != IOIF_ABSENC_NO_PARAM && !IsValResolution(resolution)) {
        return IOIF_ABSENC_STATUS_INVALID_RESOLUTION;
    }

    // Validate sample frequency if it's not marked as NO_PARAM
    if (samplFreq != IOIF_ABSENC_NO_PARAM && !IsValSamplFreq(samplFreq)) {
        return IOIF_ABSENC_STATUS_INVALID_SAMPL_FREQ;
    }

    // All parameters are valid
    return IOIF_ABSENC_STATUS_OK;
}

/**
 * @brief Reads the absolute encoder bit using the specified SPI interface.
 * 
 * @param spi The SPI interface to use for the read operation.
 * @param absEnc Pointer to the absolute encoder structure.
 * @return IOIF_SPI_STATUS_OK if successful, or a specific error status otherwise.
 */
static IOIF_SPIState_t ReadAbsBit(IOIF_SPI_t spi, IOIF_AbsEnc_t* absEnc)
{
	// Validate the SPI using previously defined function
    if (!IsValSPI(spi)) {
        return IOIF_ABSENC_STATUS_INVALID_SPI;
    }

    // Validate the absEnc pointer using previously defined function
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }

    // Check and execute based on SPI type
    switch (spi) {
        case IOIF_SPI1:
            return BSP_RunSPIDMA(BSP_SPI1, NULL, (uint8_t*)absEnc->absBit, IOIF_SPI1_READ_SIZE, BSP_SPI_RECEIVE_DMA);
        case IOIF_SPI3:
            return BSP_RunSPIDMA(BSP_SPI3, NULL, (uint8_t*)absEnc->absBit, IOIF_SPI3_READ_SIZE, BSP_SPI_RECEIVE_DMA);
        default:
            // This should be unreachable due to the validation above, but added for safety.
            return IOIF_ABSENC_STATUS_INVALID_SPI;
    }
}

/**
 * @brief Calculates the position and velocity in degrees based on the raw bits from the absolute encoder.
 * 
 * @param spi The SPI interface used for communication.
 * @param absEnc Pointer to the absolute encoder structure.
 * @return IOIF_SPI_STATUS_OK if successful, or a specific error status otherwise.
 */
static IOIF_SPIState_t CalPosVelDeg(IOIF_AbsEnc_t* absEnc)
{
	// Validate the absEnc pointer using previously defined function
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }

    float tPosDeg = 0.0, tPosMulti = 0.0, tMultiply = 0.0;
    uint16_t tRawBits = 0;

	// Extract raw bits from the encoder data
    tRawBits = (*(absEnc->absBit)) & (absEnc->resolution);
    absEnc->posDegRaw = (float)tRawBits * (360.0 / (float)absEnc->resolution);  // Convert to 0~360 range

    // Offset & Convert Range 
    tPosDeg = absEnc->posDegRaw - absEnc->offset;
    if (tPosDeg < 0) { tPosDeg += 360; }

    // Normalize to range -180 ~ 180
    if 		(tPosDeg >= 180) { tPosDeg -= 360; } 
	else if (tPosDeg < -180) { tPosDeg += 360; }

    // Detect multi-turns
    tMultiply = absEnc->posDeg * tPosDeg;
    if (tMultiply < -15000) {
        if (tPosDeg < 0) {
            absEnc->absTurn++;
        } else {
            absEnc->absTurn--;
        }
    }

    // Calculate position and velocity
    tPosMulti = tPosDeg + absEnc->absTurn*360;
    absEnc->velDeg = (tPosMulti - absEnc->posDegMultiTurn)*absEnc->samplFreq;

    absEnc->posDeg = absEnc->sign*tPosDeg;
    absEnc->posDegMultiTurn = absEnc->sign * tPosMulti;

    return IOIF_ABSENC_STATUS_OK;
}


#endif /* IOIF_RMB20SC_ENABLED */
