

#include "ioif_spi_common.h"

/** @defgroup SPI SPI
  * @brief SPI BSP module driver
  * @{
  */
#ifdef BSP_SPI_MODULE_ENABLED

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

//code example for SPI DMA buffer - should be fixed
//uint8_t spi3DmaTxBuff[256] __attribute__((section(".spi3TxBuff"))) = {0}; // AM SPI Communication Tx
//uint8_t spi3DmaTxTestBuff[256] __attribute__((section(".spi3TxTestBuff"))) = {0}; // AM SPI Communication Tx Test


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

uint8_t IOIF_SetSPICB(IOIF_SPI_t spi, IOIF_SPICBType_t callbackType, IOIF_SPICBPtr_t callback, void* params)
{
    // Check the validity of the spi
    if (spi >= IOIF_SPI_COUNT) {
        return IOIF_SPI_STATUS_ERROR; // Invalid spi
    }

    // Check the validity of the callbackType
    if (callbackType >= IOIF_SPI_CALLBACK_TYPE_COUNT) {
        return IOIF_SPI_STATUS_ERROR; // Invalid callbackType
    }

    // Check if the callback pointer is valid
    if (!callback) {
        return IOIF_SPI_STATUS_ERROR; // Invalid callback pointer
    }

    // Map IOIF enums, parameters, callback to BSP equivalents if necessary
    BSP_SetSPICB((BSP_SPI_t)(spi), (BSP_SPICBType_t)(callbackType), (BSP_SPICBPtr_t)(callback), params);

    return IOIF_SPI_STATUS_OK;
}


void IOIF_SPI_InitFailRecovery()
{

	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, LED_DRIVE_SPI_NSS_Pin, IOIF_GPIO_PIN_RESET);		// cs low

	__HAL_RCC_SPI1_CLK_DISABLE();
	__HAL_SPI_DISABLE(&hspi1);
	HAL_SPI_DeInit(&hspi1);


#ifdef _USE_OS_RTOS
	IOIF_SYS_BlockingDelay(IOIF_SPI_RECOVERY_TIME_INTERVAL);
#else
	IOIF_msDelay(I2C_REVOVERY_TIME_INTERVAL);
#endif /*_USE_OS_RTOS*/


	HAL_SPI_Init(&hspi1);
	__HAL_I2C_ENABLE(&hspi1);
	__HAL_RCC_SPI1_CLK_DISABLE();

}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* BSP_SPI_MODULE_ENABLED */
