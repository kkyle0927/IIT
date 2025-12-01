/**
 *-----------------------------------------------------------
 *            TB67H450FNGEL MOTOR CONTROL INTERFACE
 *-----------------------------------------------------------
 * @file ioif_tb67h450fngel.c
 * @date Created on: Aug 24, 2023
 * @author AngelRobotics HW Team
 * @brief IO Interface implementation for the TB67H450FNGEL DC Motor Driver.
 * 
 * This source file provides the functionality to control the L30 robot's 
 * leg length by adjusting a DC motor. The interface is implemented as a 
 * state machine that determines the motor's operation state based on 
 * button or length inputs. The program controls the GPIO pins to signal 
 * the TB67H450FNGEL DC motor driver, enabling motor movements like 
 * positive, negative, and stop.
 * 
 * @ref TB67H450FNGEL_DATA_SHEET_XXXXXXX.pdf
 */
#include "ioif_tb67h450fngel.h"


/** @defgroup GPIO GPIO
  * @brief GPIO Upright DC Motor module driver
  * @{
  */
#ifdef IOIF_TB67H450FNGEL_ENABLED

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
 * @brief Static Function prototypes for this module.
 */

static void UpdateState(IOIF_UPRIGHT_t* uprightObj);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

IOIF_UprightState_t IOIF_InitUpright(IOIF_UPRIGHT_t* uprightObj, uint8_t uprightId, uint8_t dirBut, uint8_t dirMov, uint16_t positiveDirSw, uint16_t negativeDirSw, uint16_t positiveMotorIn, uint16_t negativeMotorIn)
{
	// TODO: Varies depending on Upright.
	uprightObj->upright_id = uprightId;
	uprightObj->DirButton=dirBut;
	uprightObj->DirMove=dirMov;

	if(uprightObj->DirButton==1){
		uprightObj->pDirSw = positiveDirSw;
		uprightObj->nDirSw = negativeDirSw;
	}
	else if(uprightObj->DirButton==2){
		uprightObj->pDirSw = negativeDirSw ;
		uprightObj->nDirSw = positiveDirSw;
	}

	if(uprightObj->DirMove==1){
		uprightObj->pDirMotorIn = positiveMotorIn;
		uprightObj->nDirMotorIn = negativeMotorIn;
	}
	else if(uprightObj->DirMove==2){
		uprightObj->pDirMotorIn = negativeMotorIn ;
		uprightObj->nDirMotorIn = positiveMotorIn ;
	}
	uprightObj->DirCmd = IOIF_UPRIGHT_STOP;
//	uprightObj->DirAct = IOIF_UPRIGHT_STOP;

	return IOIF_UPRIGHT_STATUS_OK;
}

void IOIF_SetUprightBT(IOIF_UPRIGHT_t* uprightObj)
{
	uint8_t tPos = BSP_ReadGPIOPin(BSP_GPIO_PORT_E, uprightObj->pDirSw);
	uint8_t tNeg = BSP_ReadGPIOPin(BSP_GPIO_PORT_E, uprightObj->nDirSw);

	if(tPos == BSP_GPIO_PIN_SET)		{	uprightObj->DirCmd = IOIF_UPRIGHT_POSITIVE;	}
	else if(tNeg == BSP_GPIO_PIN_SET) 	{	uprightObj->DirCmd = IOIF_UPRIGHT_NEGATIVE;	}
	else 								{	uprightObj->DirCmd = IOIF_UPRIGHT_STOP;		}
}

void IOIF_SetUprightLen(IOIF_UPRIGHT_t* uprightObj)
{
	static uint16_t tUpperLimit,tLowerLimit=0;

	tUpperLimit = uprightObj->lengthRef + (float)IOIF_UPRIGHT_DEAD_ZONE;
	tLowerLimit = uprightObj->lengthRef - (float)IOIF_UPRIGHT_DEAD_ZONE;

	if((uprightObj->lengthAct <= tUpperLimit) && (uprightObj->lengthAct >= tLowerLimit)) {
		uprightObj->DirCmd = IOIF_UPRIGHT_STOP;
	}else if(uprightObj->lengthAct > tUpperLimit){
		uprightObj->DirCmd = IOIF_UPRIGHT_NEGATIVE;
	}else if(uprightObj->lengthAct < tLowerLimit){
		uprightObj->DirCmd = IOIF_UPRIGHT_POSITIVE;
	}
}

void IOIF_MoveUpright(IOIF_UPRIGHT_t* uprightObj)
{
    UpdateState(uprightObj);  // Update the state based on the command

	switch(uprightObj->DirAct){
		case UPRIGHT_MOVE_POSITIVE:
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED)
			BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_SET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_RESET);
#endif
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
			BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_SET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_RESET);
#endif
			break;
		case UPRIGHT_MOVE_NEGATIVE:
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED)
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_RESET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_SET);
#endif
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
			BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_RESET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_SET);
#endif
			break;
		case UPRIGHT_MOVE_STOP:
#if defined(WALKON5_CM_ENABLED) || defined(L30_CM_ENABLED)
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_RESET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_RESET);
#endif
#if defined(L30_MD_REV06_ENABLED) || defined(L30_MD_REV07_ENABLED) || defined(L30_MD_REV08_ENABLED)
			BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->pDirMotorIn, BSP_GPIO_PIN_RESET);
            BSP_WriteGPIOPin(BSP_GPIO_PORT_B, uprightObj->nDirMotorIn, BSP_GPIO_PIN_RESET);
#endif
			break;
		default: 
            // TODO : Error Handling
            break;
	}
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void UpdateState(IOIF_UPRIGHT_t* uprightObj)
{
    switch (uprightObj->DirCmd) {
        case IOIF_UPRIGHT_POSITIVE:
            uprightObj->DirAct = UPRIGHT_MOVE_POSITIVE;
            break;

        case IOIF_UPRIGHT_NEGATIVE:
            uprightObj->DirAct = UPRIGHT_MOVE_NEGATIVE;
            break;

        case IOIF_UPRIGHT_STOP:
        default:
            uprightObj->DirAct = UPRIGHT_MOVE_STOP;
            break;
    }
}


#endif /* IOIF_TB67H450FNGEL_ENABLED */
