/**
 *-----------------------------------------------------------
 *       PCA9531 8-BIT I2C LED DIMMER DRIVER HEADER           
 *-----------------------------------------------------------
 * @file pca9531.h
 * @date Created on: Aug 7, 2023
 * @author AngelRobotics HW Team
 * @brief Header file for the PCA9531 8-bit I2C LED dimmer.
 *
 * This header file provides type definitions, function prototypes,
 * macros, and other necessary information to interface with the
 * PCA9531 8-bit I2C LED dimmer.
 * 
 * Refer to the PCA9531 datasheet and related documents for more information.
 *
 * @ref PCA9531 Datasheet
 */

#ifndef PCA9531_INC_PCA9531_H_
#define PCA9531_INC_PCA9531_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pca9531_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define PCA9531_TIMEOUT     200

#define PCA9531_ONECOLOR    -1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef uint8_t PCA9531;

typedef uint8_t (*PCA9531_InitFunc)       (void);
typedef uint8_t (*PCA9531_DeInitFunc)     (void);
typedef uint8_t (*PCA9531_IsDevReadyFunc) (uint16_t, uint32_t, uint32_t); // DevAddr, Trials, Timeout
typedef uint8_t (*PCA9531_ReadRegFunc)    (uint16_t, uint16_t, uint8_t*, uint16_t); // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*PCA9531_WriteRegFunc)   (uint16_t, uint16_t, uint8_t*, uint16_t); // DevAddr, RegAddr, Data, DataSize
//typedef uint8_t (*PCA9531_WaitFunc)       (uint32_t); // ms to wait
//typedef uint8_t (*PCA9531_EnableFunc)     (uint8_t); // 0: disable, else: enable

typedef enum _PCA9531State_t {
    PCA9531_STATUS_OK = 0,
    PCA9531_STATUS_ERROR,
    PCA9531_STATUS_BUSY,
    PCA9531_STATUS_TIMEOUT,
    PCA9531_STATUS_ADDRESS_ERROR,
    PCA9531_STATUS_READ_ERROR,
    PCA9531_STATUS_WRITE_ERROR,
} PCA9531State_t;

// IO context
typedef struct _PCA9531IOctx_t {
    PCA9531_InitFunc       Init;         /**< Initialization function. */
    PCA9531_DeInitFunc     DeInit;       /**< De-initialization function. */
    PCA9531_IsDevReadyFunc IsDevReady;   /**< Check device readiness. */
    PCA9531_ReadRegFunc    ReadReg;      /**< Read register function. */
    PCA9531_WriteRegFunc   WriteReg;     /**< Write register function. */
} PCA9531IOctx_t;

// Object handle
typedef struct _PCA9531Object_t {
    uint32_t devAddr;               /**< Device address. */
    uint32_t isInitialized;         /**< Initialization status. */
    void *pData;                    /**< Pointer to additional data. */
    PCA9531IOctx_t io;              /**< IO context. */
} PCA9531Object_t;

typedef struct _PCA9531I2CMessage_t {
    uint16_t address;  /**< I2C address. */
    uint16_t nrBytes;  /**< Number of bytes. */
    void* pi2cData;    /**< Pointer to I2C data. */
} PCA9531I2CMessage_t;


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

PCA9531 PCA9531_SetIoctx(PCA9531Object_t *pca9531_obj, PCA9531IOctx_t *pca9531_ioctx);
PCA9531 PCA9531_Init(PCA9531Object_t *pca9531_obj);
PCA9531 PCA9531_DeInit(PCA9531Object_t *pca9531_obj);

PCA9531 PCA9531_SetPwm0(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num);
PCA9531 PCA9531_GetPwm0(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num);
PCA9531 PCA9531_SetPwm1(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num);
PCA9531 PCA9531_GetPwm1(PCA9531Object_t *pca9531_obj, uint16_t *pPwm, uint32_t num);

PCA9531 PCA9531_GetLs0(PCA9531Object_t *pca9531_obj, uint16_t *pData, uint32_t num);
PCA9531 PCA9531_SetLs0(PCA9531Object_t *pca9531_obj, uint16_t *pData, uint32_t num);

#endif /* PCA9531_INC_PCA9531_H_ */
