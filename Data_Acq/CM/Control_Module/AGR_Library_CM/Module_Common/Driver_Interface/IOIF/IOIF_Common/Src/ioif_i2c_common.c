

#include "ioif_i2c_common.h"

/** @defgroup I2C I2C
 * @brief I2C BSP module driver
 * @{
 */
#ifdef BSP_I2C_MODULE_ENABLED

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
 * @brief Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

uint8_t IOIF_SetI2CCB(IOIF_I2C_t i2c, IOIF_I2CCBType_t callbackType, IOIF_I2CCBPtr_t callback, void* params)
{
	// Check the validity of the i2c
	if (i2c >= IOIF_I2C_COUNT) {
		return IOIF_I2C_STATUS_ERROR; // Invalid i2c
	}

	// Check the validity of the callbackType
	if (callbackType >= IOIF_I2C_CALLBACK_TYPE_COUNT) {
		return IOIF_I2C_STATUS_ERROR; // Invalid callbackType
	}

	// Check if the callback pointer is valid
	if (!callback) {
		return IOIF_I2C_STATUS_ERROR; // Invalid callback pointer
	}

	// Map IOIF enums, parameters, callback to BSP equivalents if necessary
	BSP_SetI2CCB((BSP_I2C_t)(i2c), (BSP_I2CCBType_t)(callbackType), (BSP_I2CCBPtr_t)(callback), params);

	return IOIF_I2C_STATUS_OK;
}

IOIF_I2CState_t IOIF_I2C_InitRecovery(IOIF_I2C_t i2c,
                                        IOIF_GPIOPort_t sda_port, uint16_t sda_pin,
                                        IOIF_GPIOPort_t scl_port, uint16_t scl_pin)
{
	uint32_t i2c_port = (uint32_t)i2c;
	I2C_HandleTypeDef* hi2c = BSP_I2C_FindHandle(i2c);
	if(hi2c == NULL) { return IOIF_I2C_STATUS_HANDLE_ERROR;}

	IOIF_SYS_BlockingDelay(IOIF_I2C_RECOVERY_TIME_INTERVAL);

	__HAL_I2C_DISABLE(hi2c);
	switch(i2c_port) {
	    case 1: __HAL_RCC_I2C1_CLK_DISABLE(); break;
	    case 2: __HAL_RCC_I2C2_CLK_DISABLE(); break;
	    case 3: __HAL_RCC_I2C3_CLK_DISABLE(); break;
	    case 4: __HAL_RCC_I2C4_CLK_DISABLE(); break;
	    default: return IOIF_I2C_STATUS_ERROR;
	}

	if(BSP_I2C_DeInit((BSP_I2C_t)i2c) != BSP_OK) 	{return IOIF_I2C_STATUS_ERROR;}
	BSP_I2C_MspDeInit((BSP_I2C_t)i2c);

	BSP_I2C_BusReset(i2c, (BSP_GPIOPort_t)sda_port, sda_pin, (BSP_GPIOPort_t)scl_port, scl_pin);

	if(BSP_I2C_Init((BSP_I2C_t)i2c) != BSP_OK) 		{return IOIF_I2C_STATUS_ERROR;}
	BSP_I2C_MspInit((BSP_I2C_t)i2c);

	switch((uint32_t)i2c) {
	case 1: __HAL_RCC_I2C1_CLK_ENABLE(); break;
	case 2: __HAL_RCC_I2C2_CLK_ENABLE(); break;
	case 3: __HAL_RCC_I2C3_CLK_ENABLE(); break;
	case 4: __HAL_RCC_I2C4_CLK_ENABLE(); break;
	default: return IOIF_I2C_STATUS_ERROR;
	}
	__HAL_I2C_ENABLE(hi2c);

	return (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY) ?
			IOIF_I2C_STATUS_OK : IOIF_I2C_STATUS_ERROR;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* BSP_I2C_MODULE_ENABLED */
