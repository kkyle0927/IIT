/**
 * @file bsp_sai.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for SAI functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef SAI_INC_BSP_SAI_H_
#define SAI_INC_BSP_SAI_H_

#include "main.h"
#include "module.h"

/** @defgroup SAI SAI
  * @brief SAI HAL BSP module driver
  * @
  */
#ifdef HAL_SAI_MODULE_ENABLED
#define BSP_SAI_MODULE_ENABLED

#include "sai.h"
#include "bsp_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */




/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Function pointer type for SAI callback functions.
 */
typedef void (*BSP_SAICBPtr_t)(void* params);

/**
  * @brief  HAL State structures definition
  */
typedef enum _BSP_SAIStateTypeDef_t{
    BSP_SAI_STATE_RESET   = HAL_SAI_STATE_RESET, /*!< SAI not yet initialized or disabled  */
    BSP_SAI_STATE_READY   = HAL_SAI_STATE_READY, /*!< SAI initialized and ready for use    */
    BSP_SAI_STATE_BUSY    = HAL_SAI_STATE_BUSY, /*!< SAI internal process is ongoing      */
    BSP_SAI_STATE_BUSY_TX = HAL_SAI_STATE_BUSY_TX, /*!< Data transmission process is ongoing */
    BSP_SAI_STATE_BUSY_RX = HAL_SAI_STATE_BUSY_RX, /*!< Data reception process is ongoing    */
} BSP_SAIStateTypeDef_t;

/**
 * @brief Enumeration for BSP SAI operations.
 *
 * Represents the various sai-related operations available
 * within the BSP layer, abstracting HAL functions.
 */
typedef enum _BSP_SAIOP_t {
    // SAI Blocking Mode
    BSP_SAI_TRANSMIT,
    BSP_SAI_RECEIVE,

    // SAI Non-Blocking Mode with IT
    BSP_SAI_TRANSMIT_IT,
    BSP_SAI_RECEIVE_IT,

    // SAI Non-Blocking Mode with DMA
    BSP_SAI_TRANSMIT_DMA,
    BSP_SAI_RECEIVE_DMA,

    // SAI Mute Mode
    BSP_SAI_ENABLE_TX_MUTE_MODE,
    BSP_SAI_DISABLE_TX_MUTE_MODE,
    BSP_SAI_ENABLE_RX_MUTE_MODE,
    BSP_SAI_DISABLE_RX_MUTE_MODE,
} BSP_SAIOP_t;

/**
 * @brief Enumeration for BSP SAI identifiers.
 * Starts from 1 to align with common STM32 naming (SAI1, SAI2, ...)
 */
typedef enum _BSP_SAI_t {
    BSP_SAI1 = 1,  ///< SAI 1 Identifier
    BSP_SAI2,      ///< SAI 2 Identifier
    BSP_SAI3,      ///< SAI 3 Identifier
    BSP_SAI4,      ///< SAI 4 Identifier
    BSP_SAI_COUNT
} BSP_SAI_t;

/**
 * @enum BSP_SAICBType_t
 * @brief SAI callback types for various sai events.
 */
typedef enum _BSP_SAICBType_t {
    BSP_SAI_TX_CPLT_CALLBACK,
    BSP_SAI_TX_HALF_CPLT_CALLBACK,
    BSP_SAI_RX_CPLT_CALLBACK,
    BSP_SAI_RX_HALF_CPLT_CALLBACK,
    BSP_SAI_ERROR_CALLBACK,
    BSP_SAI_CALLBACK_TYPE_COUNT,
} BSP_SAICBType_t;

/**
 * @struct BSP_SAIMap_t
 * @brief Maps BSP SAI enumerations to their corresponding HAL SAI handles.
 */
typedef struct _BSP_SAIMap_t {
    BSP_SAI_t sai;       ///< Enumeration of the saier
    SAI_HandleTypeDef* handle;  ///< Pointer to the HAL SAI handle
} BSP_SAIMap_t;

/**
 * @struct BSP_SAICB_t
 * @brief Manager BSP SAI Custom Callbacks and Parameters.
 */
typedef struct _BSP_SAICB_t {
    BSP_SAICBPtr_t callbacks[BSP_SAI_CALLBACK_TYPE_COUNT];
    void* params[BSP_SAI_CALLBACK_TYPE_COUNT];
} BSP_SAICB_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */




/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

/* ------------------- INITIALIZATION & DE-INITIALIZATION ------------------- */
BSP_StatusTypeDef_t BSP_InitSAIProtocol(BSP_SAI_t sai, uint32_t protocol, uint32_t dataSize, uint32_t nbSlot);
BSP_StatusTypeDef_t BSP_InitSAI(BSP_SAI_t sai);
BSP_StatusTypeDef_t BSP_DeInitSAI(BSP_SAI_t sai);

/* ------------------- SAI DEVICE CHECK ------------------- */
BSP_SAIStateTypeDef_t BSP_GetStateSAI(BSP_SAI_t sai);
uint32_t BSP_GetErrorSAI(BSP_SAI_t sai);

/* ------------------- BLOCKING MODE ------------------- */
BSP_StatusTypeDef_t BSP_RunSAIBlock(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeout, BSP_SAIOP_t operation);
BSP_StatusTypeDef_t BSP_AbortSAIBlock(BSP_SAI_t sai);

/* ------------------- NON-BLOCKING MODE WITH IT ------------------- */
BSP_StatusTypeDef_t BSP_RunSAIIT(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, BSP_SAIOP_t operation);

/* ------------------- NON-BLOCKING MODE WITH DMA ------------------- */
BSP_StatusTypeDef_t BSP_RunSAIDMA(BSP_SAI_t sai, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, BSP_SAIOP_t operation);
BSP_StatusTypeDef_t BSP_PauseSAIDMA(BSP_SAI_t sai);
BSP_StatusTypeDef_t BSP_ResumeSAIDMA(BSP_SAI_t sai);
BSP_StatusTypeDef_t BSP_StopSAIDMA(BSP_SAI_t sai);

/* ------------------- MUTE MODE ------------------- */
BSP_StatusTypeDef_t BSP_EnableSAITxMute(BSP_SAI_t sai, uint16_t val);
BSP_StatusTypeDef_t BSP_DisableSAITxMute(BSP_SAI_t sai);
BSP_StatusTypeDef_t BSP_EnableSAIRxMute(BSP_SAI_t sai, BSP_SAICBPtr_t callback, uint16_t counter);
BSP_StatusTypeDef_t BSP_DisableSAIRxMute(BSP_SAI_t sai);

/* ------------------- SAI CALLBACKS ------------------- */
void BSP_SetSAICB(BSP_SAI_t sai, BSP_SAICBType_t callbackType, BSP_SAICBPtr_t callback, void* params);


#endif /* HAL_SAI_MODULE_ENABLED */

#endif /* SAI_INC_BSP_SAI_H_ */
