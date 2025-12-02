/**
 * @file system_ctrl_task.h
 * @date Created on: Jul 28, 2023
 * @author AngelRobotics FW Team
 * @brief 
 * @ref 
 */

#ifndef APPS_SYSTEM_CTRL_INC_SYSTEM_CTRL_H_
#define APPS_SYSTEM_CTRL_INC_SYSTEM_CTRL_H_

#include "ioif_tim_common.h"
#include "ioif_gpio_common.h"
#include "ioif_ltc2944.h"

#include "task_mngr.h"
#include "error_dictionary.h"

#include "msg_hdlr.h"

#include "../../../../Module_Common/Driver_Interface/CDI/DOP_Mngr/Inc/data_object_common.h"

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

extern TaskObj_t systemCtrlTask;
#if defined(L30_MD_REV06_ENABLED) || defined (L30_MD_REV07_ENABLED)
extern IOIF_BatData_t batData;
#endif
/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

void InitSysMngtTask(void);
void RunSysMngtTask(void* params);


#endif /* APPS_SYSTEM_CTRL_INC_SYSTEM_CTRL_H_ */
