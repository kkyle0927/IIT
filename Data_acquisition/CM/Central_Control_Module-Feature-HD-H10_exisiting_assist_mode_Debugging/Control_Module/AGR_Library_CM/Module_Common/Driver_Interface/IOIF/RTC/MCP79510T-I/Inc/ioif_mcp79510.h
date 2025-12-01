/**
 *-----------------------------------------------------------
 *                  MCP7510T-I/MS RTC driver
 *-----------------------------------------------------------
 * @file ioif_mcp79510.c
 * @date Created on: Nov 29, 2024
 * @author AngelRobotics FW Team
 * @brief Code for the MCP7510T-I/MS RTC driver.
 *
 *
 * Refer to the MCP7510T-I/MS datasheet and related documents for more information.
 * @ref MCP7510T-I/MS Datasheet
 */

#ifndef MODULE_COMMON_DRIVER_INTERFACE_IOIF_SYSTEMCTRL_RTC_MCP79510T_I_INC_IOIF_MCP795_H_
#define MODULE_COMMON_DRIVER_INTERFACE_IOIF_SYSTEMCTRL_RTC_MCP79510T_I_INC_IOIF_MCP795_H_

#include "module.h"

#ifdef IOIF_MCP79510_ENABLED

/** @defgroup SPI SPI
 * @brief SPI MCP7510T-I/MS RTC driver
 * @{
 */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include "ioif_spi_common.h"
#include "ioif_gpio_common.h"
#include "MCP79510T.h"

/**
*-----------------------------------------------------------
*              MACROS AND PREPROCESSOR DIRECTIVES
*-----------------------------------------------------------
* @brief Directives and macros for readability and efficiency.
*/


#define RTC_SPI_RX_BUFF_LENGTH		257
#define RTC_SPI_TX_BUFF_LENGTH		257


/**
*------------------------------------------------------------
*                     TYPE DECLARATIONS
*------------------------------------------------------------
* @brief Custom data types and structures for the module.
*/

typedef enum _IOIF_MCP79510_State_t {
    IOIF_MCP79510_STATE_OK = 0,
	IOIF_MCP79510_STATE_ERROR,
} IOIF_MCP79510_State_t;

typedef enum _IOIF_MCP79510_CS{
	IOIF_SPI_CS_LOW = CS_LOW,
	IOIF_SPI_CS_HIGH = CS_HIGH,
} IOIF_MCP79510_CS;

typedef enum _IOIF_RTC_SPIGPIO_t{
	IOIF_RTC_SPI_CS_PORT    = BSP_GPIO_PORT_B,
	IOIF_RTC_SPI_CS_PIN 	= BSP_GPIO_PIN_9,
	IOIF_RTC_SPI_PORT		= BSP_SPI2,
}IOIF_RTC_SPIGPIO_t;

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


IOIF_MCP79510_State_t IOIF_MCP79510_Init(void);

IOIF_MCP79510_State_t IOIF_MCP79510_SetTime(rtcTime_t* rtcValue);
IOIF_MCP79510_State_t IOIF_MCP79510_GetTime(rtcTime_t* rtcValue);

time_t IOIF_MCP79510_convert_to_linuxtime(rtcTime_t *rtcValue);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunMCP79510(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /* IOIF_MCP79510_ENABLED */

#endif /* MODULE_COMMON_DRIVER_INTERFACE_IOIF_SYSTEMCTRL_RTC_MCP79510T_I_INC_IOIF_MCP795_H_ */
