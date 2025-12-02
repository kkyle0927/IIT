/**
 *-----------------------------------------------------------
 *       PCA9531 8-BIT I2C-BUS LED DIMMER DRIVER
 *-----------------------------------------------------------
 * @file pca9531.c
 * @date Created on: Aug 8, 2023
 * @author AngelRobotics HW Team
 * @brief Driver code for the PCA9531 8-bit I2C-bus LED dimmer.
 *
 * This source file provides functionality to interface
 * with the PCA9531 device, including initialization,
 * PWM control, and LED state configurations.
 * 
 * Refer to the PCA9531 datasheet and related documents for more information.
 *
 * @ref PCA9531 Datasheet
 */

#include "pca9531.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




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
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes for this module.
 */

static PCA9531 PCA9531_ReadReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size);
static PCA9531 PCA9531_WriteReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size);
//static PCA9531 PCA9531_SetReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Set the I/O context for the PCA9531 object.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @param pca9531_ioctx Pointer to the I/O context structure.
 * @return PCA9531 status code.
 */
PCA9531 PCA9531_SetIoctx(PCA9531Object_t *pca9531_obj, PCA9531IOctx_t *pca9531_ioctx)
{
    if (!pca9531_obj || !pca9531_ioctx->IsDevReady || !pca9531_ioctx->WriteReg) {
        return PCA9531_STATUS_ERROR;
    }

    pca9531_obj->io.Init       = pca9531_ioctx->Init;
    pca9531_obj->io.DeInit     = pca9531_ioctx->DeInit;
    pca9531_obj->io.IsDevReady = pca9531_ioctx->IsDevReady;
    pca9531_obj->io.ReadReg    = pca9531_ioctx->ReadReg;
    pca9531_obj->io.WriteReg   = pca9531_ioctx->WriteReg;

    return PCA9531_STATUS_OK;
}

/**
 * @brief Initialize the PCA9531 object by setting control registers.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @return PCA9531 status code.
 */
PCA9531 PCA9531_Init(PCA9531Object_t *pca9531_obj)
{
    if (!pca9531_obj) {
        return PCA9531_STATUS_ERROR;
    }

	PCA9531 status = PCA9531_STATUS_OK;

    // Init IO Context
    if (!pca9531_obj->isInitialized) {
        if (pca9531_obj->io.Init != 0) {
            pca9531_obj->io.Init();
        }
    }
    
    // Find Address
    status = PCA9531_STATUS_ADDRESS_ERROR;
    for (uint16_t hwid = 0; hwid <= PCA9531_DEV_ADDR_MAX_HW_ID; ++hwid) {
        uint16_t targetDevAddr = (PCA9531_DEV_ADDR_PREFIX | hwid) << 1;
        if (pca9531_obj->io.IsDevReady(targetDevAddr, 10, PCA9531_TIMEOUT) == 0) {
        	pca9531_obj->devAddr = targetDevAddr;
            status = PCA9531_STATUS_OK;
            break;
        }
    }

    return status;
}

/**
 * @brief De-Initialize the PCA9531 object by setting control registers.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @return PCA9531 status code.
 */
PCA9531 PCA9531_DeInit(PCA9531Object_t *pca9531_obj)
{
    PCA9531 status = PCA9531_STATUS_OK;

    // Reset Register
    // if (status == PCA9531_STATUS_OK) {
    //     PCA9531 dummy = 0;
    //     status = PCA9531_WriteReg(pca9531_obj, PCA9531_RESET_REG, &dummy, 1);
    // }

    return status;
}

/**
 * @brief Set PWM0 value.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pPwm Pointer to PWM value.
 * @param num Number of PWM values to write.
 * @return Status of the operation.
 */
PCA9531 PCA9531_SetPwm0(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num)
{
    return PCA9531_WriteReg(pca9531_obj, PCA9531_PWM0_REG, (PCA9531*)pPwm, PCA9531_PWM0_SIZE*num);
}

/**
 * @brief Get PWM0 value.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pPwm Pointer to store read PWM value.
 * @param num Number of PWM values to read.
 * @return Status of the operation.
 */
PCA9531 PCA9531_GetPwm0(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num)
{
	return PCA9531_ReadReg(pca9531_obj, PCA9531_PWM0_REG,(PCA9531*)pPwm, PCA9531_PWM0_SIZE*num);
}

/**
 * @brief Set PWM1 value.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pPwm Pointer to PWM value.
 * @param num Number of PWM values to write.
 * @return Status of the operation.
 */
PCA9531 PCA9531_SetPwm1(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num)
{
    return PCA9531_WriteReg(pca9531_obj, PCA9531_PWM1_REG,(PCA9531*)pPwm, PCA9531_PWM1_SIZE*num);
}

/**
 * @brief Get PWM1 value.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pPwm Pointer to store read PWM value.
 * @param num Number of PWM values to read.
 * @return Status of the operation.
 */
PCA9531 PCA9531_GetPwm1(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num)
{
    return PCA9531_ReadReg(pca9531_obj, PCA9531_PWM1_REG, (PCA9531*)pPwm, PCA9531_PWM1_SIZE*num);
}

/**
 * @brief Get LS0 (LED0~3) state.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pData Pointer to store read LED state data.
 * @param num Number of LED state values to read.
 * @return Status of the operation.
 */
PCA9531 PCA9531_GetLs0(PCA9531Object_t *pca9531_obj, uint16_t *pData, uint32_t num)
{
    return PCA9531_ReadReg(pca9531_obj, PCA9531_LS0_REG, (PCA9531*)pData, PCA9531_LS0_SIZE*num);
}

/**
 * @brief Set LS0 (LED0~3) state.
 * @param pca9531_obj Pointer to PCA9531 object.
 * @param pData Pointer to LED state data.
 * @param num Number of LED state values to write.
 * @return Status of the operation.
 */
PCA9531 PCA9531_SetLs0(PCA9531Object_t *pca9531_obj, uint16_t *pData, uint32_t num)
{
    return PCA9531_WriteReg(pca9531_obj, PCA9531_LS0_REG, (PCA9531*)pData, PCA9531_LS0_SIZE*num);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Read data from the specified register of the PCA9531 device.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @param reg Register devAddress to read.
 * @param data Pointer to store the read data.
 * @param size Size of the data to be read.
 * @return PCA9531 status code.
 */
static PCA9531 PCA9531_ReadReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size)
{
    if (pca9531_obj->io.ReadReg(pca9531_obj->devAddr, reg, pData, size) < 0) {
        return PCA9531_STATUS_READ_ERROR;
    } else {
        return PCA9531_STATUS_OK;
    }
}

/**
 * @brief Write data to the specified register of the PCA9531 device.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @param reg Register devAddress to write.
 * @param data Pointer to the data to write.
 * @param size Size of the data to be written.
 * @return PCA9531 status code.
 */
static PCA9531 PCA9531_WriteReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size)
{
    if (pca9531_obj->io.WriteReg(pca9531_obj->devAddr, reg, pData, size) < 0) { //i2c's writereg ioif
        return PCA9531_STATUS_WRITE_ERROR;
    } else {
        return PCA9531_STATUS_OK;
    }
}

/**
 * @brief Set and verify the specified register of the PCA9531 device.
 * @param pca9531_obj Pointer to the PCA9531 object.
 * @param reg Register devAddress to set.
 * @param data Pointer to the data to set.
 * @param size Size of the data to be set.
 * @return PCA9531 status code.
 */
//static PCA9531 PCA9531_SetReg(PCA9531Object_t *pca9531_obj, uint16_t reg, PCA9531 *pData, uint32_t size)
//{
//	PCA9531 status = PCA9531_STATUS_OK;
//    PCA9531 readval[size];
//
//    status = PCA9531_WriteReg(pca9531_obj, reg, pData, size);
//
//    if (status == PCA9531_STATUS_OK) {
//        status = PCA9531_ReadReg(pca9531_obj, reg, readval, size);
//    }
//
//    if (status == PCA9531_STATUS_OK) {
//        if (memcmp(pData, &readval, size) != 0) {
//            status = PCA9531_STATUS_ERROR;
//        }
//    }
//
//    return status;
//}


