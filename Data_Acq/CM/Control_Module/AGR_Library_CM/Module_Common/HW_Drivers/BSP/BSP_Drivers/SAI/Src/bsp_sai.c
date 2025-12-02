/**
 * @file bsp_sai.c
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for SAI functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#include "bsp_sai.h"

/** @defgroup FDCAN FDCAN
  * @brief FDCAN HAL BSP module driver
  * @{
  */
#ifdef HAL_SAI_MODULE_ENABLED

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
static BSP_SAIMap_t bsp_saiMap[BSP_SAI_COUNT] = {
    {BSP_SAI_COUNT, NULL},      // Dummy entry for index 0
    {BSP_SAI1, &hsai_BlockA1},  // SAI 1
    {BSP_SAI2, NULL},           // SAI not defined
    {BSP_SAI3, NULL},           // SAI not defined
    {BSP_SAI4, NULL},           // SAI not defined
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


static BSP_SAICB_t bsp_saiCB[BSP_SAI_COUNT];


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static SAI_HandleTypeDef* FindSAIHandle(BSP_SAI_t sai);
static BSP_SAI_t FindSAIEnum(SAI_HandleTypeDef* hsai);
static void ExecuteSAICB(SAI_HandleTypeDef* hsai, BSP_SAICBType_t callbackType);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
/**
 * @brief  Initialize the structure FrameInit, SlotInit, 
 *          and the low part of Init according to the specified parameters 
 *          and call the function BSP_InitSAI to initialize the SAI block.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the SAI module.
 * @param  protocol: One of the supported protocols (see SAI Supported protocol).
 * @param  datasize: One of the supported data sizes (see SAI protocol data size).
 * @param  nbSlot: Number of slots.
 * @return BSP_StatusTypeDef_t: Status of the initialization process.
 */
BSP_StatusTypeDef_t BSP_InitSAIProtocol(BSP_SAI_t sai, uint32_t protocol, uint32_t datasize, uint32_t nbSlot)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_InitProtocol(hsai, protocol, datasize, nbSlot);
}

/**
 * @brief Initialize the SAI peripheral.
 * @param sai Identifier for the SAI peripheral.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_InitSAI(BSP_SAI_t sai) 
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_Init(hsai);
}

/**
 * @brief Deinitialize the SAI peripheral.
 * @param sai Identifier for the SAI peripheral.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_DeInitSAI(BSP_SAI_t sai) 
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DeInit(hsai);
}

/* ------------------- SAI DEVICE CHECK ------------------- */
/**
 * @brief Retrieve the current SAI state.
 * @param sai Identifier for the SAI peripheral.
 * @return HAL status.
 */
BSP_SAIStateTypeDef_t BSP_GetStateSAI(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_SAIStateTypeDef_t)HAL_SAI_GetState(hsai);
}

/**
 * @brief Retrieve the current SAI error state.
 * @param sai Identifier for the SAI peripheral.
 * @return HAL status.
 */
uint32_t BSP_GetErrorSAI(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return HAL_SAI_GetError(hsai);
}

/* ------------------- BLOCKING MODE ------------------- */
/**
 * @brief Perform a blocking SAI operation.
 * @param sai SAI device identifier.
 * @param pTxData Pointer to transmission data.
 * @param pRxData Pointer to receive data buffer.
 * @param size Amount of data to be sent/received.
 * @param timeout timeout duration. in millisecond.
 * @param operation SAI operation type.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_RunSAIBlock(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeout, BSP_SAIOP_t operation)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_SAI_TRANSMIT:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Transmit(hsai, pTxData, size, timeout);
            break;
        case BSP_SAI_RECEIVE:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Receive(hsai, pRxData, size, timeout);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/**
 * @brief Abort a blocking SAI operation.
 * @param sai SAI device identifier.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_AbortSAIBlock(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    BSP_StatusTypeDef_t status = BSP_OK;
    status = (BSP_StatusTypeDef_t)HAL_SAI_Abort(hsai);

    return status;
}

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
/**
 * @brief Perform a non-blocking SAI operation with interrupts.
 * @param sai SAI device identifier.
 * @param pTxData Pointer to transmission data.
 * @param pRxData Pointer to receive data buffer.
 * @param size Amount of data to be sent/received.
 * @param operation SAI operation type.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_RunSAIIT(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, BSP_SAIOP_t operation)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_SAI_TRANSMIT_IT:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Transmit_IT(hsai, pTxData, size);
            break;
        case BSP_SAI_RECEIVE_IT:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Receive_IT(hsai, pRxData, size);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
/**
 * @brief Perform a non-blocking SAI operation with DMA.
 * @param sai SAI device identifier.
 * @param pTxData Pointer to transmission data.
 * @param pRxData Pointer to receive data buffer.
 * @param size Amount of data to be sent/received.
 * @param operation SAI operation type.
 * ! dma_buff logic is only for Normal mode
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_RunSAIDMA(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, BSP_SAIOP_t operation)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    BSP_StatusTypeDef_t status = BSP_OK;
    switch (operation) {
        case BSP_SAI_TRANSMIT_DMA:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Transmit_DMA(hsai, pTxData, size);
            break;
        case BSP_SAI_RECEIVE_DMA:
            status = (BSP_StatusTypeDef_t)HAL_SAI_Receive_DMA(hsai, pRxData, size);
            break;
        default:
            return BSP_ERROR; // Invalid operation
    }

    return status;
}

/**
 * @brief Pause a DMA-based SAI operation.
 * @param sai SAI device identifier.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_PauseSAIDMA(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DMAPause(hsai);
}

/**
 * @brief Resume a paused DMA-based SAI operation.
 * @param sai SAI device identifier.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_ResumeSAIDMA(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DMAResume(hsai);
}

/**
 * @brief Stop a DMA-based SAI operation.
 * @param sai SAI device identifier.
 * @return HAL status.
 */
BSP_StatusTypeDef_t BSP_StopSAIDMA(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DMAStop(hsai);
}

/**
 * @brief  Enables the transmit mute mode for a specific SAI module.
 * @param  sai: BSP_SAI_t type that identifies the SAI module to be operated on.
 * @param  val: Value to set the mute mode.
 * @return BSP_StatusTypeDef_t: The status of the operation.
 */
BSP_StatusTypeDef_t BSP_EnableSAITxMute(BSP_SAI_t sai, uint16_t val)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_EnableTxMuteMode(hsai, val);
}

/**
 * @brief  Disables the transmit mute mode for a specific SAI module.
 * @param  sai: BSP_SAI_t type that identifies the SAI module to be operated on.
 * @return BSP_StatusTypeDef_t: The status of the operation.
 */
BSP_StatusTypeDef_t BSP_DisableSAITxMute(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DisableTxMuteMode(hsai);
}

/**
 * @brief  Enables the receive mute mode for a specific SAI module.
 * @param  sai: BSP_SAI_t type that identifies the SAI module to be operated on.
 * @param  callback: BSP_SAICBPtr_t type that specifies the callback function to be invoked.
 * @param  counter: Value to set the mute mode counter.
 * @return BSP_StatusTypeDef_t: The status of the operation.
 */
BSP_StatusTypeDef_t BSP_EnableSAIRxMute(BSP_SAI_t sai, BSP_SAICBPtr_t callback, uint16_t counter)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_EnableRxMuteMode(hsai, (SAIcallback)callback, counter);
}

/**
 * @brief  Disables the receive mute mode for a specific SAI module.
 * @param  sai: BSP_SAI_t type that identifies the SAI module to be operated on.
 * @return BSP_StatusTypeDef_t: The status of the operation.
 */
BSP_StatusTypeDef_t BSP_DisableSAIRxMute(BSP_SAI_t sai)
{
    SAI_HandleTypeDef* hsai = FindSAIHandle(sai);

    return (BSP_StatusTypeDef_t)HAL_SAI_DisableRxMuteMode(hsai);
}

/* ------------------- SAI CALLBACKS ------------------- */
/**
 * @brief Registers a custom callback for a specified SAI port and callback type.
 *
 * @param sai Enum value representing the SAI port.
 * @param callbackType Enum value representing the type of callback (e.g., transmit complete, receive complete).
 * @param callback Function pointer to the custom callback function.
 * @param params Pointer to any parameters that the custom callback function may need.
 */
void BSP_SetSAICB(BSP_SAI_t sai, BSP_SAICBType_t callbackType, BSP_SAICBPtr_t callback, void* params)
{
    if (sai < BSP_SAI_COUNT && callbackType < BSP_SAI_CALLBACK_TYPE_COUNT && callback) {
        bsp_saiCB[sai].callbacks[callbackType] = callback;
        bsp_saiCB[sai].params[callbackType] = params;
    }
}

/**
 * @brief  Callback function to be executed upon the completion of an SAI transmit operation.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the specified SAI module.
 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef* hsai)
{
    UNUSED(hsai);

   	ExecuteSAICB(hsai, BSP_SAI_TX_CPLT_CALLBACK);
}

/**
 * @brief  Callback function to be executed upon the half completion of an SAI transmit operation.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the specified SAI module.
 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    UNUSED(hsai);

   	ExecuteSAICB(hsai, BSP_SAI_TX_HALF_CPLT_CALLBACK);
}

/**
 * @brief  Callback function to be executed upon the completion of an SAI receive operation.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the specified SAI module.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef* hsai)
{
    UNUSED(hsai);

   	ExecuteSAICB(hsai, BSP_SAI_RX_CPLT_CALLBACK);
}

/**
 * @brief  Callback function to be executed upon the half completion of an SAI receive operation.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the specified SAI module.
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef* hsai)
{
    UNUSED(hsai);

   	ExecuteSAICB(hsai, BSP_SAI_RX_HALF_CPLT_CALLBACK);
}

/**
 * @brief  Callback function to be executed when an error occurs during an SAI operation.
 * @param  hsai: Pointer to a SAI_HandleTypeDef structure that contains the configuration information for the specified SAI module.
 */
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef* hsai)
{
    UNUSED(hsai);

   	ExecuteSAICB(hsai, BSP_SAI_ERROR_CALLBACK);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Executes a custom callback for a specified SAI port and callback type.
 *
 * @param hsai HAL SAI handle, used to find the corresponding SAI port enum value.
 * @param callbackType Enum value representing the type of callback to execute.
 */
static void ExecuteSAICB(SAI_HandleTypeDef* hsai, BSP_SAICBType_t callbackType)
{
    BSP_SAI_t sai = FindSAIEnum(hsai);
    if (sai < BSP_SAI_COUNT) {
        BSP_SAICBPtr_t callback = bsp_saiCB[sai].callbacks[callbackType];
        void* params = bsp_saiCB[sai].params[callbackType];
        if (callback) {
            callback(params); // Executes the custom callback with the specified parameters
        }
    }
}

/**
 * @brief Find the SAI handle corresponding to the specified SAI enumeration
 * 
 * @param sai SAI enumeration
 * @return Pointer to SAI handle or NULL if not found
 */
static SAI_HandleTypeDef* FindSAIHandle(BSP_SAI_t sai) 
{
    for (int i = 0; i < ARRAY_SIZE(bsp_saiMap); i++) {
        if (bsp_saiMap[i].sai == sai) {
            return bsp_saiMap[i].handle;
        }
    }
    return NULL;
}

static BSP_SAI_t FindSAIEnum(SAI_HandleTypeDef* hsai)
{
    for (int i = 0; i < ARRAY_SIZE(bsp_saiMap); i++) {
        if (bsp_saiMap[i].handle == hsai) {
            return bsp_saiMap[i].sai;
        }
    }
    return BSP_SAI_COUNT; // TODO: Change Status
}


#endif /* HAL_SAI_MODULE_ENABLED */
