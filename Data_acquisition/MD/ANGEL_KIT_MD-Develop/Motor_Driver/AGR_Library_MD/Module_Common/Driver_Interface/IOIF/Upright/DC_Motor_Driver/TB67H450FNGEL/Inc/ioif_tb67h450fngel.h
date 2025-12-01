/**
 *-----------------------------------------------------------
 *            TB67H450FNGEL MOTOR CONTROL INTERFACE
 *-----------------------------------------------------------
 * @file ioif_tb67h450fngel.h
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface header for the TB67H450FNGEL DC Motor Driver.
 * 
 * This header file provides the interface definitions for the 
 * TB67H450FNGEL DC Motor Driver. The interface includes macros, 
 * type declarations, and function prototypes essential for controlling 
 * the L30 robot's leg length by adjusting a DC motor. It determines 
 * the motor's operation state based on button or length inputs and 
 * interfaces with the GPIO pins to control the DC motor driver.
 * 
 * @ref TB67H450FNGEL_DATA_SHEET_XXXXXXX.pdf
 */

#ifndef TB67H450FNGEL_INC_IOIF_TB67H450FNGEL_H_
#define TB67H450FNGEL_INC_IOIF_TB67H450FNGEL_H_

#include "module.h"

/** @defgroup GPIO GPIO
  * @brief GPIO Upright DC Motor module driver
  * @{
  */
#ifdef IOIF_TB67H450FNGEL_ENABLED

#include "ioif_gpio_common.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_UPRIGHT_DEAD_ZONE	0.5


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef enum _IOIF_UprightState_t {
    IOIF_UPRIGHT_STATUS_OK = 0,
    IOIF_UPRIGHT_STATUS_ERROR,
} IOIF_UprightState_t;

typedef enum _IOIF_UprightStateMachine_t {
    IOIF_UPRIGHT_POSITIVE = 0,
    IOIF_UPRIGHT_NEGATIVE,
    IOIF_UPRIGHT_STOP,
} IOIF_UprightStateMachine_t;

typedef enum {
    INPUT_NONE = 0,
    INPUT_LENGTH,
    INPUT_BUTTON,   // Highest priority
} IOIF_UprightInputPriority_t;

typedef enum _IOIF_UprightMoveDir_t {
    UPRIGHT_MOVE_POSITIVE,
    UPRIGHT_MOVE_NEGATIVE,
    UPRIGHT_MOVE_STOP,
    UPRIGHT_MOVE_NUM,
} IOIF_UprightMoveDir_t;

typedef struct _IOIF_UPRIGHT_t{
	uint8_t upright_id;

    float lengthRef;
    float lengthAct;

    uint16_t pDirSw;
    uint16_t nDirSw;

    uint16_t pDirMotorIn;
    uint16_t nDirMotorIn;

    IOIF_UprightMoveDir_t DirCmd;
    IOIF_UprightMoveDir_t DirAct;

	uint8_t DirMove;
	uint8_t DirButton;

	float_t slope;
	float_t offset;
} IOIF_UPRIGHT_t;


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

IOIF_UprightState_t IOIF_InitUpright(IOIF_UPRIGHT_t* uprightObj, uint8_t uprightId, uint8_t dirBut, uint8_t dirMov, uint16_t positiveDirSw, uint16_t negativeDirSw, uint16_t positiveMotorIn, uint16_t negativeMotorIn);
void IOIF_SetUprightBT(IOIF_UPRIGHT_t* uprightObj);
void IOIF_SetUprightLen(IOIF_UPRIGHT_t* uprightObj);
void IOIF_MoveUpright(IOIF_UPRIGHT_t* uprightObj);


#endif /* IOIF_TB67H450FNGEL_ENABLED */

#endif /* TB67H450FNGEL_INC_IOIF_TB67H450FNGEL_H_ */
