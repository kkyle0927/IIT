/**
 * @file bsp_wwdg.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics HW Team
 * @brief Board Support Package for WWDG functionalities.
 * @ref um2217-description-of-stm32h7-hal-and-lowlayer-drivers-stmicroelectronics.pdf
 */

#ifndef WWDG_INC_BSP_WWDG_H_
#define WWDG_INC_BSP_WWDG_H_

#include "main.h"

/** @defgroup WWDG WWDG
  * @brief WWDG HAL BSP module driver
  * @
  */
#ifdef HAL_WWDG_MODULE_ENABLED
#define BSP_WWDG_MODULE_ENABLED

#include "wwdg.h"
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




#endif /* HAL_WWDG_MODULE_ENABLED */

#endif /* WWDG_INC_BSP_WWDG_H_ */
