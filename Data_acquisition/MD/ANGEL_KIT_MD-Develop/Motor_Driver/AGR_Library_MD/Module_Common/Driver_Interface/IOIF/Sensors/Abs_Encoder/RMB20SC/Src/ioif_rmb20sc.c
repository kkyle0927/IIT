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
#ifdef _USE_DEBUG_CLI
IOIF_AbsEnc_t cliAbs1Data={0,};
IOIF_AbsEnc_t cliAbs2Data={0,};
#endif /* _USE_DEBUG_CLI */


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

#ifdef _USE_DEBUG_CLI
static IOIF_AbsState_t moduleinit_res=0;
//static IOIF_AbsSelect_t location_res=0;
#endif /* _USE_DEBUG_CLI */

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
    #ifdef _USE_DEBUG_CLI
        moduleinit_res = status;
    #endif /* _USE_DEBUG_CLI */
        return status;
    }

    absEnc->location = absId;

    if ((absEnc->location != IOIF_ABS_ENC_NONE) &&
            (absEnc->location != IOIF_ABS_ENC_ACTUATOR_INPUT) &&
            (absEnc->location != IOIF_ABS_ENC_ACTUATOR_OUTPUT) &&
            (absEnc->location != IOIF_ABS_ENC_JOINT1) &&
            (absEnc->location != IOIF_ABS_ENC_JOINT2))
    {
        absEnc->location = IOIF_ABS_ENC_NONE;
    }

    if (channel == 1) {
        absEnc->absBit = &rmb20scDmaRxBuff1;
    }

    else if (channel == 2) {
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

    #ifdef _USE_DEBUG_CLI
    moduleinit_res=status;
    //location_res=absEnc->location;
    #endif /* _USE_DEBUG_CLI */

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

    if (absEnc->resolution != 0) {
        absEnc->offset = (float)tAbsBits * (360.0 / (float)absEnc->resolution);  // Convert to 0 ~ 360 range
        absEnc->absTurn = 0;
    } else {
        tAbsBits = 0;
        absEnc->offset = 0.0;
    }

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
	uint16_t usTemp = 0;
	// Validate the absEnc pointer using previously defined function
    if (!IsValAbsEnc(absEnc)) {
        return IOIF_ABSENC_STATUS_NULL_PTR;
    }
    
    absEnc->rawBit = *(absEnc->absBit) & (absEnc->resolution);
    usTemp =*(absEnc->absBit) - absEnc->raw_offset;

	// Extract raw bits from the encoder data
    if (absEnc->resolution != 0) {
        absEnc->rawBitsTo13Bit = usTemp & (absEnc->resolution); // bit mask ex) 13bit resolution = 0x1FFF
        absEnc->posDegRaw = (float)absEnc->rawBitsTo13Bit * (360.0 / (float)absEnc->resolution);  // Convert to 0~360 range
    } else {
        absEnc->rawBitsTo13Bit = 0;
        absEnc->posDegRaw = 0.0;
    }

    // Offset & Convert Range 
    absEnc->posDeg_ConvertRange = absEnc->posDegRaw - absEnc->offset;

//    // Normalize to range -180 ~ 180
//    if 		(absEnc->posDeg_ConvertRange >= 180) { absEnc->posDeg_ConvertRange -= 360; }
//	else if (absEnc->posDeg_ConvertRange < -180) { absEnc->posDeg_ConvertRange += 360; }

    // Normalize to converted range
    if 		(absEnc->posDeg_ConvertRange > 360 - absEnc->offset) { absEnc->posDeg_ConvertRange -= 360; }
	else if (absEnc->posDeg_ConvertRange <  - absEnc->offset) { absEnc->posDeg_ConvertRange += 360; }

    // Calculate the difference between the current and previous position
    absEnc->deltaAbsDeg = absEnc->posDeg_ConvertRange - absEnc->posDegPrev_ConvertRange;

    // Detect multi-turns based on the deltaAbsDeg value and handle the rollover
    if (absEnc->deltaAbsDeg > 359) {
        if (absEnc->posDeg_ConvertRange < 0) {
            absEnc->absTurn++;
        } else {
            absEnc->absTurn--;
        }
    } else if (absEnc->deltaAbsDeg < -359) {
        if (absEnc->posDeg_ConvertRange > 0) {
            absEnc->absTurn--;
        } else {
            absEnc->absTurn++;
        }
    }

    // Calculate position and velocity
    absEnc->posDeg[0] = absEnc->sign * absEnc->posDeg_ConvertRange;
    absEnc->velDeg = (absEnc->posDeg[0] - absEnc->posDeg[1]) * absEnc->samplFreq;

    absEnc->posMultiCal = absEnc->posDeg_ConvertRange + absEnc->absTurn * 360;
    absEnc->posDegMultiTurn[0] = absEnc->sign * absEnc->posMultiCal;
    absEnc->velDegMultiTurn = (absEnc->posDegMultiTurn[0] - absEnc->posDegMultiTurn[1]) * absEnc->samplFreq; //The sample frequency is in Hz, so it's not a division; it should be a multiplication.

	// Save Previous Value
    absEnc->posDegPrev_ConvertRange = absEnc->posDeg_ConvertRange;
	absEnc->posDeg[1] = absEnc->posDeg[0];
	absEnc->posDegMultiTurn[1] = absEnc->posDegMultiTurn[0];

    return IOIF_ABSENC_STATUS_OK;
}

#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */
void CLI_RunRMB20SC1(cli_args_t *args) {
    bool ret = false;
    uint32_t duration = 10;

    if (args->cmpStr(0, "isinit") == true)
        {
            IOIF_AbsState_t state = moduleinit_res;
            const char* absEncStateStrings[] = {
                    "IOIF_ABSENC_STATUS_OK",
                    "IOIF_ABSENC_STATUS_NULL_PTR",
                    "IOIF_ABSENC_STATUS_INVALID_SPI",
                    "IOIF_ABSENC_STATUS_INVALID_RESOLUTION",
                    "IOIF_ABSENC_STATUS_INVALID_SAMPL_FREQ",
                    "IOIF_ABSENC_STATUS_INIT_ERR",
                    "IOIF_ABSENC_NO_PARAM"
            };

            CLI_Printf(" * RMB20SC_1 init state : %s * \r\n",absEncStateStrings[state]);

        } else if(args->argc == 2 && args->cmpStr(0, "getdata") == true) {

            CLI_Printf(" * RMB20SC_1 data gathering started! * \r\n");
            duration = (uint32_t) args->getData(1);         // param. 1 에 대한 string 데이터를 정수로 변환

            if(duration < 10){
                ret = false;
                CLI_Printf(" * Duration too short. Minimum is 10ms. * \r\n");
            }
            else{
                ret = true;
                while(ret && CLI_KeepLoop())
                {
                    IOIF_GetPosVelDeg1(IOIF_SPI1, &cliAbs1Data, &flashsensorObj);
                    CLI_Printf("Abs pos raw: %f deg, Abs pos: %f deg, Abs velo: %f deg/s. \r\n", cliAbs1Data.posDegRaw, cliAbs1Data.posDeg[0], cliAbs1Data.velDeg);
                    CLI_Delay(duration);
                }
            }
        }

        else if(ret == false)       //help
        {
            CLI_Printf(" * Abs encoder CH1 Monitoring * \r\n");
            CLI_Printf("   Get rmb20sc_1 location/raw position/position/velocity sensor data at a specified period of time[ms] \r\n");
            CLI_Printf(" * Usage * \r\n");
            CLI_Printf("   rmb20sc_1 isinit \r\n");
            CLI_Printf("   rmb20sc_1 getdata [duration(ms) (min. 10ms)]\r\n");
            CLI_Printf(" * Command Example * \r\n");
            CLI_Printf("   rmb20sc_1 getdata 100 \r\n");
        }
}

void CLI_RunRMB20SC2(cli_args_t *args) {
    bool ret = false;
    uint32_t duration = 10;

    if (args->cmpStr(0, "isinit") == true)
        {
            IOIF_AbsState_t state = moduleinit_res;
            const char* absEncStateStrings[] = {
                    "IOIF_ABSENC_STATUS_OK",
                    "IOIF_ABSENC_STATUS_NULL_PTR",
                    "IOIF_ABSENC_STATUS_INVALID_SPI",
                    "IOIF_ABSENC_STATUS_INVALID_RESOLUTION",
                    "IOIF_ABSENC_STATUS_INVALID_SAMPL_FREQ",
                    "IOIF_ABSENC_STATUS_INIT_ERR",
                    "IOIF_ABSENC_NO_PARAM"
            };

            CLI_Printf(" * RMB20SC_2 init state : %s * \r\n",absEncStateStrings[state]);

        } else if(args->argc == 2 && args->cmpStr(0, "getdata") == true) {

            CLI_Printf(" * RMB20SC_2 data gathering started! * \r\n");
            duration = (uint32_t) args->getData(1);         // param. 1 에 대한 string 데이터를 정수로 변환

            if(duration < 10){
                ret = false;
                CLI_Printf(" * Duration too short. Minimum is 10ms. * \r\n");
            }
            else{
                ret = true;
                while(ret && CLI_KeepLoop())
                {
                    IOIF_GetPosVelDeg2(IOIF_SPI3, &cliAbs2Data);
                    CLI_Printf("Abs pos raw: %f deg, Abs pos: %f deg, Abs velo: %f deg/s. \r\n", cliAbs2Data.posDegRaw, cliAbs2Data.posDeg[0], cliAbs2Data.velDeg);
                    CLI_Delay(duration);
                }
            }
        }

        else if(ret == false)       //help
        {
            CLI_Printf(" * Abs encoder CH2 Monitoring * \r\n");
            CLI_Printf("   Get rmb20sc_2 location/raw position/position/velocity sensor data at a specified period of time[ms] \r\n");
            CLI_Printf(" * Usage * \r\n");
            CLI_Printf("   rmb20sc_2 isinit \r\n");
            CLI_Printf("   rmb20sc_2 getdata [duration(ms) (min. 10ms)]\r\n");
            CLI_Printf(" * Command Example * \r\n");
            CLI_Printf("   rmb20sc_2 getdata 100 \r\n");
        }
}

#endif /* _USE_DEBUG_CLI */

#endif /* IOIF_RMB20SC_ENABLED */
