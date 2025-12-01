/**
 * @file bsp_sdmmc.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for SDMMC functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "bsp_sdmmc.h"

/** @defgroup SDMMC SDMMC
  * @brief SDMMC HAL BSP module driver
  * @{
  */
#ifdef HAL_SD_MODULE_ENABLED

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

static BSP_SDMap_t bsp_sdMap[BSP_SD_COUNT] = {
    {BSP_SD_COUNT, NULL},    // Dummy entry for index 0
    {BSP_SD1, &hsd1},         // SD not defined
    {BSP_SD2, NULL},         // SD 2
};

static BSP_SDCB_t bsp_sdCB[BSP_SD_COUNT];


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static SD_HandleTypeDef* FindSDHandle(BSP_SD_t sd);
//static BSP_SD_t FindSDEnum(SD_HandleTypeDef* hsd);
//static void ExecuteSDCB(SD_HandleTypeDef* hsd, BSP_SDCBType_t callbackType);
static uint8_t BSP_SD_IsDetected(void);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
/**
 * @brief Initializes the SD card.
 * @param sd The SD card identifier.
 * @return BSP_StatusTypeDef_t Status of the initialization process.
 */
BSP_SD_Status_t BSP_InitSD(BSP_SD_t sd)
{
	BSP_SD_Status_t status = BSP_MSD_OK;
	uint8_t sd_state = MSD_OK;

    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    //return status = BSP_SD_Init();		// call 'bsp_driver_sd.c' driver init.

    /* HAL driver initialized with Bus width : 1bit */
    hsd->Init.BusWide = SDMMC_BUS_WIDE_1B;

    if (BSP_SD_IsDetected() != SD_PRESENT)										// Card detection check if sd card is inserted or not
   	 {
    	status = BSP_MSD_ERROR_SD_NOT_PRESENT;
   	 }

    sd_state = HAL_SD_Init(hsd);

    hsd->Init.BusWide = SDMMC_BUS_WIDE_4B;

    if (sd_state == MSD_OK)
    {
    	// Enable wide operation
    	if (HAL_SD_ConfigWideBusOperation(hsd, SDMMC_BUS_WIDE_4B) != HAL_OK)
    	{
    		status = BSP_MSD_ERROR;
    	}
    }

    return status;
}

/**
 * @brief Initializes the specified SD card.
 * @param sd The SD card identifier.
 * @return BSP_StatusTypeDef_t Status of the initialization process.
 */
BSP_StatusTypeDef_t BSP_InitSDCard(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    return (BSP_StatusTypeDef_t)HAL_SD_InitCard(hsd);
}

/**
 * @brief Deinitializes the specified SD card.
 * @param sd The SD card identifier.
 * @return BSP_StatusTypeDef_t Status of the deinitialization process.
 */
BSP_StatusTypeDef_t BSP_DeInitSD(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    return (BSP_StatusTypeDef_t)HAL_SD_DeInit(hsd);
}

/* ------------------- SD CHECK ------------------- */
/**
 * @brief Gets the current state of the SD card.
 * @param sd The SD card identifier.
 * @return HAL_SD_StateTypeDef Current state of the SD card.
 */
BSP_SD_StateTypeDef_t BSP_GetStateSD(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    return (BSP_SD_StateTypeDef_t)HAL_SD_GetState(hsd);
}

/**
 * @brief Gets the current error state of the SD card.
 * @param sd The SD card identifier.
 * @return uint32_t The error code.
 */
uint32_t BSP_SD_GetError(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    return HAL_SD_GetError(hsd);
}

/* ------------------- PERIPHERAL CONTROL FUNCTIONS ------------------- */
/**
 * @brief Returns information the information of the card which are stored on the CID register.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pCID Pointer to a HAL_SD_CardCIDTypeDef structure that contains all CID register parameters.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_GetSDCID(BSP_SD_t sd, BSP_SD_CardCIDTypeDef_t* pCID)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to get the card CID
    return (BSP_StatusTypeDef_t)HAL_SD_GetCardCID(hsd, (HAL_SD_CardCIDTypeDef*)pCID);
}

/**
 * @brief Returns information of the card which is stored on the CSD register.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pCSD Pointer to a BSP_SD_CardCSDTypeDef_t structure that contains all CSD register parameters.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_GetSDCSD(BSP_SD_t sd, BSP_SD_CardCSDTypeDef_t* pCSD)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to get the card CSD
    return (BSP_StatusTypeDef_t)HAL_SD_GetCardCSD(hsd, (HAL_SD_CardCSDTypeDef*)pCSD);
}

/**
 * @brief Gets the SD status info.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pStat Pointer to the BSP_SD_CardStatusTypeDef_t structure that will contain the SD card status information.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_GetSDStat(BSP_SD_t sd, BSP_SD_CardStatusTypeDef_t* pStat)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to get the card status
    return (BSP_StatusTypeDef_t)HAL_SD_GetCardStatus(hsd, (HAL_SD_CardStatusTypeDef*)pStat);
}

/**
 * @brief Gets the SD card info.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pCDInfo Pointer to the BSP_SD_CardInfoTypeDef_t structure that will contain the SD card status information.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_GetSDInfo(BSP_SD_t sd, BSP_SD_CardInfoTypeDef_t* pCDInfo)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to get the card information
    return (BSP_StatusTypeDef_t)HAL_SD_GetCardInfo(hsd, (HAL_SD_CardInfoTypeDef*)pCDInfo);
}

/**
 * @brief Enables wide bus operation for the requested card if supported by card.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param WideMode Specifies the SD card wide bus mode. Can be one of the following values:
 *                 - SDMMC_BUS_WIDE_8B: 8-bit data transfer
 *                 - SDMMC_BUS_WIDE_4B: 4-bit data transfer
 *                 - SDMMC_BUS_WIDE_1B: 1-bit data transfer
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_ConfigSDWB(BSP_SD_t sd, uint32_t WideMode)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to configure wide bus operation
    return (BSP_StatusTypeDef_t)HAL_SD_ConfigWideBusOperation(hsd, WideMode);
}

/**
 * @brief Configure the speed bus mode.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param SpeedMode Specifies the SD card speed bus mode. Can be one of the following values:
 *                  - SDMMC_SPEED_MODE_AUTO: Max speed mode supported by the card
 *                  - SDMMC_SPEED_MODE_DEFAULT: Default Speed/SDR12 mode
 *                  - SDMMC_SPEED_MODE_HIGH: High Speed/SDR25 mode
 *                  - SDMMC_SPEED_MODE_ULTRA: Ultra high speed mode
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_ConfigSDSB(BSP_SD_t sd, uint32_t SpeedMode)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to configure speed bus operation
    return (BSP_StatusTypeDef_t)HAL_SD_ConfigSpeedBusOperation(hsd, SpeedMode);
}

/**
 * @brief Gets the current SD card data state.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @return Card state.
 */
BSP_SD_CardStateTypeDef BSP_GetSDState(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    return (((BSP_SD_CardStateTypeDef)HAL_SD_GetCardState(hsd) == HAL_SD_CARD_TRANSFER ) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
 * @brief Abort the current transfer and disable the SD.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_AbortSDOp(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to abort the current transfer
    return (BSP_StatusTypeDef_t)HAL_SD_Abort(hsd);
}

/**
 * @brief Abort the current transfer and disable the SD using Interrupt.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_AbortSDIT(BSP_SD_t sd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to abort the current transfer with interrupt
    return (BSP_StatusTypeDef_t)HAL_SD_Abort_IT(hsd);
}

/* ------------------- BLOCKING MODE ------------------- */
/**
 * @brief Reads block(s) from a specified address in a card.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the received data.
 * @param blockAdd Block Address from where data is to be read.
 * @param numBlock Number of SD blocks to read.
 * @param timeout Specify timeout value. in millisecond.
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 */
BSP_StatusTypeDef_t BSP_ReadSDBlock(BSP_SD_t sd, uint32_t* pData, uint32_t blockAdd, uint32_t numBlock, uint32_t timeout)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to read blocks from SD card
    return (BSP_StatusTypeDef_t)HAL_SD_ReadBlocks(hsd, (uint8_t*) pData, blockAdd, numBlock, timeout);
}

/**
 * @brief Allows to write block(s) to a specified address in a card.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the data to transmit.
 * @param blockAdd Block Address where data will be written.
 * @param numBlock Number of SD blocks to write.
 * @param timeout Specify timeout value. in millisecond.
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 */
BSP_StatusTypeDef_t BSP_WriteSDBlock(BSP_SD_t sd, const uint32_t* pData, uint32_t blockAdd, uint32_t numBlock, uint32_t timeout)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to write blocks from SD card
    return (BSP_StatusTypeDef_t)HAL_SD_WriteBlocks(hsd, (const uint8_t*) pData, blockAdd, numBlock, timeout);
}

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
/**
 * @brief Reads block(s) from a specified address in a card using Interrupt.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the received data.
 * @param blockAdd Block Address from where data is to be read.
 * @param numBlock Number of blocks to read.
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 * You could also check the IT transfer process through the SD Rx interrupt event.
 */
BSP_StatusTypeDef_t BSP_ReadSDBlockIT(BSP_SD_t sd, uint8_t* pData, uint32_t blockAdd, uint32_t numBlock)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to read blocks from SD card with interrupt
    return (BSP_StatusTypeDef_t)HAL_SD_ReadBlocks_IT(hsd, pData, blockAdd, numBlock);
}

/**
 * @brief Writes block(s) to a specified address in a card using Interrupt.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the data to transmit.
 * @param blockAdd Block Address where data will be written.
 * @param numBlock Number of blocks to write.
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 * You could also check the IT transfer process through the SD Tx interrupt event.
 */
BSP_StatusTypeDef_t BSP_WriteSDBlockIT(BSP_SD_t sd, const uint8_t* pData, uint32_t blockAdd, uint32_t numBlock)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to write blocks to SD card with interrupt
    return (BSP_StatusTypeDef_t)HAL_SD_WriteBlocks_IT(hsd, pData, blockAdd, numBlock);
}

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
/**
 * @brief Reads block(s) from a specified address in a card using DMA.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the received data.
 * @param blockAdd Block Address from where data is to be read.
 * @param numBlock Number of blocks to read.
 * ! dma_buff logic is only for Normal mode
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 * You could also check the DMA transfer process through the SD Rx interrupt event.
 */
BSP_StatusTypeDef_t BSP_ReadSDBlockDMA(BSP_SD_t sd, uint32_t* pData, uint32_t blockAdd, uint32_t numBlock)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to read blocks from SD card with DMA
    return (BSP_StatusTypeDef_t)HAL_SD_ReadBlocks_DMA(hsd, (uint8_t*)pData, blockAdd, numBlock);
}

/**
 * @brief Writes block(s) to a specified address in a card using DMA.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param pData Pointer to the buffer that will contain the data to transmit.
 * @param blockAdd Block Address where data will be written.
 * @param numBlock Number of blocks to write.
 * ! dma_buff logic is only for Normal mode
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 * You could also check the DMA transfer process through the SD Tx interrupt event.
 */
BSP_StatusTypeDef_t BSP_WriteSDBlockDMA(BSP_SD_t sd, const uint32_t* pData, uint32_t blockAdd, uint32_t numBlock)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to write blocks to SD card with DMA
    return (BSP_StatusTypeDef_t)HAL_SD_WriteBlocks_DMA(hsd, (const uint8_t*)pData, blockAdd, numBlock);
}

/* ------------------- ERASE ------------------- */
/**
 * @brief Erases the specified memory area of the given SD card.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD module.
 * @param blockStartAdd Start Block address.
 * @param blockEndAdd End Block address.
 * @return HAL status.
 * @note This API should be followed by a check on the card state through BSP_GetSDState().
 */
BSP_StatusTypeDef_t BSP_EraseSDCard(BSP_SD_t sd, uint32_t blockStartAdd, uint32_t blockEndAdd)
{
    SD_HandleTypeDef* hsd = FindSDHandle(sd);

    // Call the HAL function to erase blocks from SD card
    return (BSP_StatusTypeDef_t)HAL_SD_Erase(hsd, blockStartAdd, blockEndAdd);
}

/* ------------------- SD CALLBACKS ------------------- */
/**
 * @brief Registers a custom callback for a specified SD port and callback type.
 *
 * @param sd Enum value representing the SD port.
 * @param callbackType Enum value representing the type of callback (e.g., transmit complete, receive complete).
 * @param callback Function pointer to the custom callback function.
 * @param params Pointer to any parameters that the custom callback function may need.
 */
void BSP_SetSDCB(BSP_SD_t sd, BSP_SDCBType_t callbackType, BSP_SDCBPtr_t callback, void* params)
{
    if (sd < BSP_SD_COUNT && callbackType < BSP_SD_CALLBACK_TYPE_COUNT && callback) {
        bsp_sdCB[sd].callbacks[callbackType] = callback;
        bsp_sdCB[sd].params[callbackType] = params;
    }
}

/**
 * @brief Callback function for SD card transmit complete event.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD card.
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef* hsd)
{
    //UNUSED(hsd);

    //ExecuteSDCB(hsd, BSP_SD_TX_CPLT_CALLBACK);
    BSP_SD_WriteCpltCallback();
}

/**
 * @brief Callback function for SD card receive complete event.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD card.
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef * hsd)
{
    //UNUSED(hsd);

    //ExecuteSDCB(hsd, BSP_SD_RX_CPLT_CALLBACK);
    BSP_SD_ReadCpltCallback();
}

/**
 * @brief Callback function for SD card error event.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD card.
 */
void HAL_SD_ErrorCallback(SD_HandleTypeDef * hsd)
{
    UNUSED(hsd);

    //ExecuteSDCB(hsd, BSP_SD_ERROR_CALLBACK);
}

/**
 * @brief Callback function for SD card abort event.
 * @param hsd Pointer to SD_HandleTypeDef structure that contains the configuration information for the SD card.
 */
void HAL_SD_AbortCallback(SD_HandleTypeDef * hsd)
{
    //UNUSED(hsd);

    //ExecuteSDCB(hsd, BSP_SD_ABORT_CALLBACK);
    //BSP_SD_AbortCallback();
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Executes a custom callback for a specified SD port and callback type.
 *
 * @param hsd HAL SD handle, used to find the corresponding SD port enum value.
 * @param callbackType Enum value representing the type of callback to execute.
 */
//static void ExecuteSDCB(SD_HandleTypeDef* hsd, BSP_SDCBType_t callbackType)
//{
//    BSP_SD_t sd = FindSDEnum(hsd);
//    if (sd < BSP_SD_COUNT) {
//        BSP_SDCBPtr_t callback = bsp_sdCB[sd].callbacks[callbackType];
//        void* params = bsp_sdCB[sd].params[callbackType];
//        if (callback) {
//            callback(params); // Executes the custom callback with the specified parameters
//        }
//    }
//}

/**
 * @brief Find the SD handle corresponding to the specified SD enumeration
 * 
 * @param sd SD enumeration
 * @return Pointer to SD handle or NULL if not found
 */
static SD_HandleTypeDef* FindSDHandle(BSP_SD_t sd) 
{
    for (int i = 0; i < ARRAY_SIZE(bsp_sdMap); i++) {
        if (bsp_sdMap[i].sd == sd) {
            return bsp_sdMap[i].handle;
        }
    }
    return NULL;
}

//static BSP_SD_t FindSDEnum(SD_HandleTypeDef* hsd)
//{
//	for (int i = 0; i < ARRAY_SIZE(bsp_sdMap); i++) {
//        if (bsp_sdMap[i].handle == hsd) {
//            return bsp_sdMap[i].sd;
//        }
//    }
//    return BSP_SD_COUNT; // TODO: Change Status
//}

//BSP_SD_Transfer_Status_t BSP_SD_IsDetected(void)
//{
//  __IO BSP_SD_Transfer_Status_t status = SD_PRESENT;
//
//  if (BSP_PlatformIsDetected() == SD_NOT_PRESENT)
//  {
//    status = SD_NOT_PRESENT;
//  }
//
//  return status;
//}

static uint8_t BSP_SD_IsDetected(void)
{
  __IO uint8_t status = SD_PRESENT;
  uint8_t IsCardDetected = BSP_PlatformIsDetected();

    if (IsCardDetected != SD_PRESENT)
    {
      status = SD_NOT_PRESENT;
    }

//  if (BSP_PlatformIsDetected() != SD_PRESENT)
//  {
//    status = SD_NOT_PRESENT;
//  }

  return status;
}


#endif /* HAL_SD_MODULE_ENABLED */
