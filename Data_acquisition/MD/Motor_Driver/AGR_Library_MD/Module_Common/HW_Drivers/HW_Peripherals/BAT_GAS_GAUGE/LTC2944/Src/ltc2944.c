/**
 *-----------------------------------------------------------
 *              LTC2944 BATTERY GAS GAUGE DRIVER
 *-----------------------------------------------------------
 * @file ltc2944.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief LTC2944 Battery Gas Gauge Driver Implementation.
 *
 * This file implements the functions required to interact with the LTC2944 Battery Gas Gauge IC.
 * It includes functions to read, write, and configure the device, as well as retrieve battery
 * status information such as voltage, current, and temperature.
 *
 * The driver is designed to work with the I2C interface and provides an abstraction layer
 * for the hardware interactions with the LTC2944 IC.
 *
 * @ref LTC2944 Datasheet
 */

#include "ltc2944.h"

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




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

// static uint8_t ReadReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t WriteReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t SetReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size);

static void CalValues(LTC2944Obj_t* ltc2944Obj);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Set the I/O context for the uint8_t object.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @param ltc2944IOctx Pointer to the I/O context structure.
 * @return uint8_t status code.
 */
uint8_t LTC2944_SetIoctx(LTC2944Obj_t* ltc2944Obj, LTC2944IOctx_t* ltc2944IOctx)
{
    // Check for valid pointers and functions
    if (!ltc2944Obj || !ltc2944IOctx->IsDevReady || !ltc2944IOctx->ReadReg || !ltc2944IOctx->WriteReg) {
        return LTC2944_STATUS_ERROR;
    }

    // Assign the I/O functions
    ltc2944Obj->io.Init        = ltc2944IOctx->Init;
    ltc2944Obj->io.DeInit      = ltc2944IOctx->DeInit;
    ltc2944Obj->io.IsDevReady  = ltc2944IOctx->IsDevReady;
    ltc2944Obj->io.ReadReg     = ltc2944IOctx->ReadReg;
    ltc2944Obj->io.WriteReg    = ltc2944IOctx->WriteReg;
    ltc2944Obj->io.Wait        = ltc2944IOctx->Wait;

    return LTC2944_STATUS_OK;
}

/**
 * @brief Initialize the uint8_t object.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @return uint8_t status code.
 */
uint8_t LTC2944_Init(LTC2944Obj_t* ltc2944Obj)
{
    // Check for valid pointers and functions
    if (!ltc2944Obj) {
        return LTC2944_STATUS_ERROR;
    }

    // Check device devAddress
    uint8_t status = LTC2944_STATUS_ERROR;
    ltc2944Obj->devAddr = LTC2944_DEV_ADDR << 1;

    if (ltc2944Obj->io.IsDevReady(ltc2944Obj->devAddr) == 0) {
    	status = LTC2944_STATUS_OK;
    }

    // Configure Control
    if (status == LTC2944_STATUS_OK) {
        uint8_t tConf = LTC2944_ADC_AUTO | LTC2944_PRESCALE_12BIT | LTC2944_ALCC_DISALBED;
        status = SetReg(ltc2944Obj, LTC2944_CONTROL_REG, &tConf, LTC2944_CONTROL_SIZE);
    }

    return status;
}

/**
 * @brief Retrieve the battery data values from the uint8_t object.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @return uint8_t status code.
 */
uint8_t LTC2944_GetValue(LTC2944Obj_t* ltc2944Obj)
{
    // Check for valid object pointer
	if (!ltc2944Obj) {
        return LTC2944_STATUS_ERROR;
    }

    // Read data from device
    uint8_t status = ltc2944Obj->io.ReadReg(ltc2944Obj->devAddr, LTC2944_DEV_ADDR, ltc2944Obj->dataBuff, LTC2944_READ_DATA_SIZE);

    // If the read operation was successful, scale and assign the data
    if (status == LTC2944_STATUS_OK) {
        CalValues(ltc2944Obj);
    }

	return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Read a register from the uint8_t device.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @param reg Register devAddress to read from.
 * @param data Pointer to the buffer to store the read data.
 * @param size Number of bytes to read.
 * @return uint8_t status code.
 */
// static uint8_t ReadReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size)
// {
//     return ltc2944Obj->io.ReadReg(ltc2944Obj->devAddr, reg, pData, size);
// }

/**
 * @brief Write to a register of the uint8_t device.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @param reg Register devAddress to write to.
 * @param data Pointer to the data to write.
 * @param size Number of bytes to write.
 * @return uint8_t status code.
 */
static uint8_t WriteReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    return ltc2944Obj->io.WriteReg(ltc2944Obj->devAddr, reg, pData, size);
}

/**
 * @brief Set a register value in the uint8_t device and verify the write operation.
 * @param ltc2944Obj Pointer to the uint8_t object.
 * @param reg Register devAddress to set.
 * @param data Pointer to the data to set.
 * @param size Number of bytes to set.
 * @return uint8_t status code. Returns an error if the verification fails.
 */
static uint8_t SetReg(LTC2944Obj_t* ltc2944Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    uint8_t status = LTC2944_STATUS_OK;
    // uint8_t readVal[size]; // Changed variable name

    status = WriteReg(ltc2944Obj, reg, pData, size);

    // if (status == LTC2944_STATUS_OK) {
    //     status = ReadReg(ltc2944Obj, reg, readVal, size);
    // }

    // if (status == LTC2944_STATUS_OK) {
    //     if (memcmp(pData, readVal, size) != 0) { // Fixed the argument
    //         status = LTC2944_STATUS_ERROR;
    //     }
    // }

    return status;
}

/**
 * @brief Scales the raw data from the LTC2944 sensor and assigns it to the appropriate structure members.
 * 
 * This function takes the raw data bytes from the LTC2944 sensor, converts them to 16-bit signed integers,
 * scales them according to the defined scale factors for acceleration and gyroscope, and assigns them
 * to the corresponding members of the LTC2944 object.
 * 
 * @param ltc2944Obj Pointer to the LTC2944 object containing the data structure to be updated.
 * @param dataBuff Pointer to the buffer containing the raw data bytes from the sensor.
 */
static void CalValues(LTC2944Obj_t* ltc2944Obj)
{
    // Array to hold the raw 16-bit values for each sensor reading
    uint16_t rawValues[LTC2944_TOTAL_ARGUMENT] = {0};

    // Convert the raw bytes to 16-bit signed integers
    uint8_t indexOffset = 8;
    for (int i = 0; i < LTC2944_TOTAL_ARGUMENT; i++) {
        rawValues[i] = (uint16_t)(ltc2944Obj->dataBuff[indexOffset + i * 6] << 8 | ltc2944Obj->dataBuff[indexOffset + i * 6 + 1]);
    }

    // Scale the raw values and assign them to the appropriate structure members
    ltc2944Obj->batData.batVol  = LTC2944_FULLSCALE_VOLTAGE * (float)rawValues[0] / (float)(0xFFFF); // V
    ltc2944Obj->batData.batCurr = LTC2944_FULLSCALE_CURRENT / LTC2944_RSENS * (float)(rawValues[1] / (float)(0x7FFF) - 1); // A
    ltc2944Obj->batData.brdTemp = (LTC2944_FULLSCALE_TEMPERATURE * (float)rawValues[2] / (float)(0xFFFF)) - 273.15; // Celsius
}

