/**
 *-----------------------------------------------------------
 *             System Control : LED & Power IC control
 *-----------------------------------------------------------
 * @file ioif_sysctrl.h
 * @date Created on: Dec 20, 2023
 * @author AngelRobotics HW Team
 * @brief
 *
 * @ref
 */

#ifndef IOIF_SYSTEMCTRL_IOIF_SYSCTRL_H_
#define IOIF_SYSTEMCTRL_IOIF_SYSCTRL_H_

#include "module.h"
#include "bsp_common.h"
#include "ioif_gpio_common.h"
#include "ioif_cpu_temperature_sensing.h"

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

typedef enum _IOIF_SysCtrl_t {
    IOIF_SYS_CTRL_OK = 0,
	IOIF_SYS_CTRL_ERROR,
} IOIF_SysCtrl_t;

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

IOIF_SysCtrl_t  IOIF_SYS_CtrlInit(void);
void 			IOIF_SYS_PwrOff(void);
bool 			IOIF_SYS_IsPwrBtnPressed(void);
void 			IOIF_SYS_BlockingDelay(uint32_t ms);
uint8_t 		IOIF_SYS_HwRevGPIO(void);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunSysCtrl(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /* IOIF_BSP_HEADER_BSP_CPU_TEMPERATURE_SENSING_H_ */
