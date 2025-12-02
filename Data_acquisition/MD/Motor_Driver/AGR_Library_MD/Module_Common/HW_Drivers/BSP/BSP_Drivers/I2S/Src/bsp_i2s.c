/**
 * @file bsp_i2s.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for I2S functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "bsp_i2s.h"

/** @defgroup I2S I2S
  * @brief I2S HAL BSP module driver
  * @{
  */
#ifdef HAL_I2S_MODULE_ENABLED

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

#endif /* WALKON5_CM_ENABLED */

#ifdef L30_CM_ENABLED

#endif /* L30_CM_ENABLED */

#ifdef SUIT_MINICM_ENABLED
static BSP_I2SMap_t bsp_i2sMap[BSP_I2S_COUNT] = {
    {BSP_I2S_COUNT, NULL},    // Dummy entry for index 0
    {BSP_I2S1, NULL},         // I2S not defined
    {BSP_I2S2, &hi2s2},       // I2S 2
    {BSP_I2S3, NULL},         // I2S not defined
};
#endif /* SUIT_MINICM_ENABLED */

#ifdef L30_MD_REV06_ENABLED

#endif /* L30_MD_REV06_ENABLED */

#ifdef L30_MD_REV07_ENABLED

#endif /* L30_MD_REV07_ENABLED */

#ifdef L30_MD_REV08_ENABLED

#endif /* L30_MD_REV08_ENABLED */

#ifdef SUIT_MD_ENABLED

#endif /* SUIT_MD_ENABLED */

#ifdef WIDM_ENABLED

#endif /* WIDM_ENABLED */


static BSP_I2SCB_t bsp_i2sCB[BSP_I2S_COUNT];


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static I2S_HandleTypeDef* FindI2SHandle(BSP_I2S_t i2s);
static BSP_I2S_t FindI2SEnum(I2S_HandleTypeDef* hi2s);
static void ExecuteI2SCB(I2S_HandleTypeDef* hi2s, BSP_I2SCBType_t callbackType);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
/**
 * @brief Initialize the I2S peripheral.
 * @param i2s Identifier for the I2S peripheral.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_InitI2S(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_StatusTypeDef_t)HAL_I2S_Init(hi2s);
}

/**
 * @brief Deinitialize the I2S peripheral.
 * @param i2s Identifier for the I2S peripheral.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_DeInitI2S(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_StatusTypeDef_t)HAL_I2S_DeInit(hi2s);
}

/* ------------------- I2S DEVICE CHECK ------------------- */
/**
 * @brief Retrieve the current I2S state.
 * @param i2s Identifier for the I2S peripheral.
 * @return HAL status.
 */
BSP_I2SStateTypeDef_t BSP_GetStateI2S(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_I2SStateTypeDef_t)HAL_I2S_GetState(hi2s);
}

/**
 * @brief Retrieve the current I2S error state.
 * @param i2s Identifier for the I2S peripheral.
 * @return HAL status.
 */
uint32_t BSP_GetErrorI2S(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return HAL_I2S_GetError(hi2s);
}

/* ------------------- BLOCKING MODE ------------------- */
/**
 * @brief Transmit/Receive an amount of data in blocking mode.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @param pTxData A 16-bit pointer to the data buffer.
 * @param pRxData A 16-bit pointer to the data buffer.
 * @param size Number of data samples to be sent. When a 16-bit or 24-bit data frame is selected, 
 *             the size parameter means the number of 16-bit data length; when a 32-bit data frame is selected, 
 *             it means the number of 16-bit data length.
 * @param timeout timeout duration. in millisecond.
 * @param operation I2S operation type.
 * @return HAL status.
 * @note The I2S is kept enabled at the end of the transaction to avoid the clock de-synchronization between Master and Slave (e.g., audio streaming).
 */
BSP_StatusTypeDef_t BSP_RunI2SBlock(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, uint32_t timeout, BSP_I2SOP_t operation)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_I2S_TRANSMIT:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Transmit(hi2s, pTxData, size, timeout);
            break;
        case BSP_I2S_RECEIVE:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Receive(hi2s, pRxData, size, timeout);
            break;
        case BSP_I2SEX_TRANSMITRECEIVE:
            status = (BSP_StatusTypeDef_t)HAL_I2SEx_TransmitReceive(hi2s, pTxData, pRxData, size, timeout);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
/**
 * @brief Transmit/Receive an amount of data in non-blocking mode with Interrupt.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @param pTxData A 16-bit pointer to the data buffer.
 * @param pRxData A 16-bit pointer to the data buffer.
 * @param size Number of data samples to be sent. When a 16-bit or 24-bit data frame is selected, 
 *             the size parameter means the number of 16-bit data length; when a 32-bit data frame is selected, 
 *             it means the number of 16-bit data length.
 * @param operation I2S operation type.
 * @return HAL status.
 * @note The I2S is kept enabled at the end of the transaction to avoid the clock de-synchronization between Master and Slave (e.g., audio streaming).
 */
BSP_StatusTypeDef_t BSP_RunI2SIT(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, BSP_I2SOP_t operation)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_I2S_TRANSMIT_IT:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Transmit_IT(hi2s, pTxData, size);
            break;
        case BSP_I2S_RECEIVE_IT:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Receive_IT(hi2s, pRxData, size);
            break;
        case BSP_I2SEX_TRANSMITRECEIVE_IT:
            status = (BSP_StatusTypeDef_t)HAL_I2SEx_TransmitReceive_IT(hi2s, pTxData, pRxData, size);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
/**
 * @brief Transmit an amount of data in non-blocking mode with DMA.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @param pTxData A 16-bit pointer to the data buffer.
 * @param pRxData A 16-bit pointer to the data buffer.
 * @param size Number of data samples to be sent. When a 16-bit or 24-bit data frame is selected, 
 *              the size parameter means the number of 16-bit data length; when a 32-bit data frame is selected,
 *              it means the number of 16-bit data length.
 * @param operation I2S operation type.
 * ! dma_buff logic is only for Normal mode
 * @return HAL status.
 * @note The I2S is kept enabled at the end of the transaction to avoid the clock de-synchronization between Master and Slave (e.g., audio streaming).
 */
BSP_StatusTypeDef_t BSP_RunI2SDMA(BSP_I2S_t i2s, const uint16_t* pTxData, uint16_t* pRxData, uint16_t size, BSP_I2SOP_t operation)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_I2S_TRANSMIT_DMA:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Transmit_DMA(hi2s, pTxData, size);
            break;
        case BSP_I2S_RECEIVE_DMA:
            status = (BSP_StatusTypeDef_t)HAL_I2S_Receive_DMA(hi2s, pRxData, size);
            break;
        case BSP_I2SEX_TRANSMITRECEIVE_DMA:
            status = (BSP_StatusTypeDef_t)HAL_I2SEx_TransmitReceive_DMA(hi2s, pTxData, pRxData, size);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/**
 * @brief Pauses the audio DMA Stream/Channel playing from the Media.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_PauseI2SDMA(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_StatusTypeDef_t)HAL_I2S_DMAPause(hi2s);
}

/**
 * @brief Resumes the audio DMA Stream/Channel playing from the Media.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_ResumeI2SDMA(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_StatusTypeDef_t)HAL_I2S_DMAResume(hi2s);
}

/**
 * @brief Stops the audio DMA Stream/Channel playing from the Media.
 * @param hi2s Pointer to a I2S_HandleTypeDef structure that contains the configuration information for I2S module.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_StopI2SDMA(BSP_I2S_t i2s)
{
    I2S_HandleTypeDef* hi2s = FindI2SHandle(i2s);

    return (BSP_StatusTypeDef_t)HAL_I2S_DMAStop(hi2s);
}

/* ------------------- I2S CALLBACKS ------------------- */
/**
 * @brief Registers a custom callback for a specified I2S port and callback type.
 *
 * @param i2s Enum value representing the I2S port.
 * @param callbackType Enum value representing the type of callback (e.g., transmit complete, receive complete).
 * @param callback Function pointer to the custom callback function.
 * @param params Pointer to any parameters that the custom callback function may need.
 */
void BSP_SetI2SCB(BSP_I2S_t i2s, BSP_I2SCBType_t callbackType, BSP_I2SCBPtr_t callback, void* params)
{
    if (i2s < BSP_I2S_COUNT && callbackType < BSP_I2S_CALLBACK_TYPE_COUNT && callback) {
        bsp_i2sCB[i2s].callbacks[callbackType] = callback;
        bsp_i2sCB[i2s].params[callbackType] = params;
    }
}

/**
 * @brief Transmit half complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_TX_HALF_CPLT_CALLBACK);
}

/**
 * @brief Transmit complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_TX_CPLT_CALLBACK);
}

/**
 * @brief Receive half complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_RX_HALF_CPLT_CALLBACK);
}

/**
 * @brief Receive complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_RX_CPLT_CALLBACK);
}

/**
 * @brief Transmit and receive half complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_TXRX_HALF_CPLT_CALLBACK);
}

/**
 * @brief Transmit and receive complete callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_TXRX_CPLT_CALLBACK);
}

/**
 * @brief Error callback for I2S.
 * @param hi2s Pointer to the I2S_HandleTypeDef structure that contains the configuration information for I2S.
 */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef* hi2s)
{
    UNUSED(hi2s);

    ExecuteI2SCB(hi2s, BSP_I2S_ERROR_CALLBACK);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Executes a custom callback for a specified I2S port and callback type.
 *
 * @param hi2s HAL I2S handle, used to find the corresponding I2S port enum value.
 * @param callbackType Enum value representing the type of callback to execute.
 */
static void ExecuteI2SCB(I2S_HandleTypeDef* hi2s, BSP_I2SCBType_t callbackType)
{
    BSP_I2S_t i2s = FindI2SEnum(hi2s);
    if (i2s < BSP_I2S_COUNT) {
        BSP_I2SCBPtr_t callback = bsp_i2sCB[i2s].callbacks[callbackType];
        void* params = bsp_i2sCB[i2s].params[callbackType];
        if (callback) {
            callback(params); // Executes the custom callback with the specified parameters
        }
    }
}

/**
 * @brief Find the I2S handle corresponding to the specified I2S enumeration
 * 
 * @param i2s I2S enumeration
 * @return Pointer to I2S handle or NULL if not found
 */
static I2S_HandleTypeDef* FindI2SHandle(BSP_I2S_t i2s) 
{
    for (int i = 0; i < ARRAY_SIZE(bsp_i2sMap); i++) {
        if (bsp_i2sMap[i].i2s == i2s) {
            return bsp_i2sMap[i].handle;
        }
    }
    return NULL;
}

static BSP_I2S_t FindI2SEnum(I2S_HandleTypeDef* hi2s)
{
    for (int i = 0; i < ARRAY_SIZE(bsp_i2sMap); i++) {
        if (bsp_i2sMap[i].handle == hi2s) {
            return bsp_i2sMap[i].i2s;
        }
    }
    return BSP_I2S_COUNT; // TODO: Change Status
}


#endif /* HAL_I2S_MODULE_ENABLED */
