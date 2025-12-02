/**
 *-----------------------------------------------------------
 *               IOIF INCREMENTAL ENCODER INTERFACE HEADER
 *-----------------------------------------------------------
 * @file ioif_rmb20ic.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface header for the Incremental Encoder.
 * 
 * This header file provides the declarations for the interface functions 
 * used to read incremental encoders using timer encoder mode. The functions
 * allow for initialization, setting offset, reading encoder counts, and 
 * handling rollovers.
 * 
 * @ref Timer Encoder Mode Documentation or Datasheet
 */

#ifndef RMB20IC_INC_IOIF_RMB20IC_H_
#define RMB20IC_INC_IOIF_RMB20IC_H_

#include "module.h"

/** @defgroup TIM TIM
  * @brief TIM EMB20IC Incremental Encoder module driver
  * @{
  */
#ifdef IOIF_RMB20IC_ENABLED

#include <stdint.h>

#include "ioif_tim_common.h"
#include "bsp_gpio.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_INT32_THRESHOLD 2147483647  // uint32_t half 0~4,294,967,295

#define IOIF_INT32_MAX 2147483647
#define IOIF_INT32_MIN (-2147483647 - 1)

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_IncState_t {
    IOIF_INCENC_STATUS_OK = 0,
	IOIF_INCENC_STATUS_ERROR,
    IOIF_INCENC_TIM_ERROR,
    IOIF_INCENC_NULL_POINTER,
} IOIF_IncState_t;

#pragma pack(push,1)
typedef struct _IOIF_IncEnc_t {
	int32_t currCnt;
    int32_t prevCnt;
	int32_t userCnt;
	int32_t offset;
	uint16_t resolution;
} IOIF_IncEnc_t;
#pragma pack(pop)


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

IOIF_IncState_t IOIF_InitIncEnc(IOIF_Tim_t timer, uint32_t channel, IOIF_IncEnc_t* incEnc);
IOIF_IncState_t IOIF_SetIncOffset(IOIF_Tim_t timer, IOIF_IncEnc_t* incEnc);
IOIF_IncState_t IOIF_ReadIncCnt(IOIF_Tim_t timer, IOIF_IncEnc_t* incEnc);


#endif /* IOIF_RMB20IC_ENABLED */

#endif /* RMB20IC_INC_IOIF_RMB20IC_H_ */
