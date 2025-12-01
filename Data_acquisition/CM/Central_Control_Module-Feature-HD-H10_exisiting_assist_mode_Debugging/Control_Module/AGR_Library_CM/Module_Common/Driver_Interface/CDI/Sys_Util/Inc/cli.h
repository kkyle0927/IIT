/**
 * @file cli.h
 * @date Created on: Apr 16, 2024
 * @author AngelRobotics FW Team
 * @brief
 * @ref
 */

#ifndef MODULE_COMMON_DRIVER_INTERFACE_CDI_SYS_UTIL_INC_CLI_H_
#define MODULE_COMMON_DRIVER_INTERFACE_CDI_SYS_UTIL_INC_CLI_H_

#include "module.h"

#ifdef _USE_DEBUG_CLI


#include "main.h"
#include "ioif_usb_common.h"				//output device : USB (default)

#include "stdint.h"
#include "stdbool.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */


#define CLI_CMD_LIST_MAX      32				// CLI 에 저장할 CMD list 의 최대값,  (determined by MCU SRAM size)
#define CLI_CMD_NAME_MAX      32				// CMD 이름 길이 최대값				(determined by MCU SRAM size)

#define CLI_LINE_HIS_MAX      10				// CLI Line history 의 최대 저장값	(determined by MCU SRAM size)
#define CLI_LINE_BUF_MAX      128				// CLI Line buffer 배열의 최대값		(determined by MCU SRAM size)



/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */


typedef struct
{
  uint16_t   argc;										// argument count
  char     **argv;										// argument string

  int32_t  (*getData)(uint8_t index);					// get integer data from char.
  float    (*getFloat)(uint8_t index);					// get float data from char.
  char    *(*getStr)(uint8_t index);					// get string
  bool     (*cmpStr)(uint8_t index, const char *p_str);	// compare string
} cli_args_t;



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


bool      CLI_Init(void);
bool      CLI_Run (void);

void      CLI_Printf(const char *fmt, ...);
bool      CLI_CMDAdd(const char *cmd_str, void (*p_func)(cli_args_t *));
bool      CLI_KeepLoop(void);

uint32_t  CLI_IsAvailable(void);
void 	  CLI_Delay(uint32_t ms);
bool      CLI_RunStr(const char *fmt, ...);

/* CLI Basic Functions */
void 	  cliClearScreen(cli_args_t *args);
void      cliShowList(cli_args_t *args);
void      cliMemoryDump(cli_args_t *args);
void 	  cliSystemOff();


#endif


#endif /* MODULE_COMMON_DRIVER_INTERFACE_CDI_SYS_UTIL_INC_CLI_H_ */

