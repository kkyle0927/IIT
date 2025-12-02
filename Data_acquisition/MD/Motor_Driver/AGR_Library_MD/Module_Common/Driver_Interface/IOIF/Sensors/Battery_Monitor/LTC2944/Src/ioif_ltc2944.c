/**
 *-----------------------------------------------------------
 *       LTC2944 IO INTERFACE BATTERY GAS GAUGE DRIVER
 *-----------------------------------------------------------
 * @file ioif_ltc2944.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief LTC2944 IO Interface Driver Implementation.
 *
 * This file implements the functions required to interact with the LTC2944 Battery Gas Gauge IC
 * at a higher interface level. It provides an abstraction layer for using the hardware and BSP functions
 * to interact with the LTC2944 IC, making it suitable for use at the task level.
 *
 * This driver is built upon the lower level LTC2944 driver and is designed to simplify the interactions
 * with the IC for higher level tasks and applications.
 *
 * @ref LTC2944 Datasheet
 */

#include "ioif_ltc2944.h"

/** @defgroup I2C I2C
  * @brief I2C Battery Monitor module driver
  * @{
  */
#ifdef IOIF_LTC2944_ENABLED

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
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c4RxBuff"))) = {0};
#endif /* L30_CM_ENABLED */

#ifdef L30_CM_ENABLED
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c4RxBuff"))) = {0};
#endif /* L30_CM_ENABLED */

#ifdef L30_MD_REV06_ENABLED
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c2RxBuff"))) = {0};
#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MINICM_ENABLED
static uint8_t ltcDmaRxBuff[IOIF_LTC2944_BUFF_SIZE] __attribute__((section(".i2c4RxBuff"))) = {0};
#endif /* SUIT_MINICM_ENABLED */

#ifdef SUIT_MD_ENABLED

#endif /* SUIT_MD_ENABLED */

#ifdef SUIT_WIDM_ENABLED 

#endif /* SUIT_WIDM_ENABLED */

static LTC2944Obj_t ltc2944Obj;
static LTC2944IOctx_t ltc2944IOctx;
static IOIF_I2C_t i2cHandle;


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static uint8_t IsDevReady(uint16_t devAddr);
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Initialize the 6-axis IMU sensor.
 * @param i2c Enumeration representing the I2C channel to use.
 * @return Status of the initialization operation.
 */
IOIF_BatState_t IOIF_InitBat(IOIF_I2C_t i2c)
{
    ltc2944IOctx.IsDevReady = IsDevReady;
    ltc2944IOctx.ReadReg = ReadReg;
    ltc2944IOctx.WriteReg = WriteReg;
    
    i2cHandle = i2c;
    // for DMA Read
    ltc2944Obj.dataBuff = ltcDmaRxBuff;

    uint8_t status = LTC2944_SetIoctx(&ltc2944Obj, &ltc2944IOctx);
    if (status != LTC2944_STATUS_OK) {
        return status; // Error handling
    }

    // Initialize the LTC2944 object
    // You should have a function for initialization
	status = LTC2944_Init(&ltc2944Obj);
    if (status != LTC2944_STATUS_OK) {
        return status; // Error handling
    }

    return status;
}

/**
 * @brief Retrieve the current values from the 6-axis IMU sensor.
 * @param batData Pointer to a structure to store the retrieved data.
 * @return Status of the data retrieval operation.
 */
IOIF_BatState_t IOIF_GetBatValue(IOIF_BatData_t* batData)
{
    // Check for NULL pointer
    if (batData == NULL) {
        return IOIF_BAT_STATUS_ERROR;
    }

    // Initialize acceleration and gyroscope data
	memset(&ltc2944Obj.batData, 0, sizeof(ltc2944Obj.batData));

    // Get the value from the hardware object and check for errors
    uint8_t status = LTC2944_GetValue(&ltc2944Obj);
    if (status != IOIF_BAT_STATUS_OK) {
        return status;
    }

    memcpy(batData, &ltc2944Obj.batData, sizeof(ltc2944Obj.batData));

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

// Checks if the Battery Monitor Device is ready
static uint8_t IsDevReady(uint16_t devAddr)
{
    return BSP_IsDevReadyI2C((BSP_I2C_t)i2cHandle, devAddr, IOIF_LTC2944_TRIALS, IOIF_LTC2944_TIMEOUT);
}

// Reads a register from the Battery Monitor Device using DMA
static uint8_t ReadReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
    return BSP_I2CMasterDMA((BSP_I2C_t)i2cHandle, devAddr, pData, size, BSP_I2C_MASTER_RECEIVE_DMA);
}

// Writes to a register on the Battery Monitor Device using blocking method
static uint8_t WriteReg(uint16_t devAddr, uint16_t regAddr, uint8_t* pData, uint16_t size)
{
	return BSP_I2CMemBlock((BSP_I2C_t)i2cHandle, devAddr, regAddr, BSP_I2C_MEMADD_SIZE_8BIT, pData, size, IOIF_LTC2944_TIMEOUT, BSP_I2C_MEM_WRITE);
}


#endif /* IOIF_LTC2944_ENABLED */
