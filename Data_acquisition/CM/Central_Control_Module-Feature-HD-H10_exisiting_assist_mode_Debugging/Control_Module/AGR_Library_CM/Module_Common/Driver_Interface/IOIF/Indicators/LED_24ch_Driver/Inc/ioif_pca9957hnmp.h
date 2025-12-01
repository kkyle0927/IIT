/**
 *-----------------------------------------------------------
 *                 PCA9957HNMP LED DRIVER
 *-----------------------------------------------------------
 * @file ioif_pca9957hnmp.h
 * @date Created on: Sep 14, 2023
 * @author AngelRobotics FW Team
 * @brief Driver code for the PCA9957HNMP LED DRIVER.
 *
 * Todo: Add Annotation
 *
 * Refer to the PCA9957HNMP datasheet and related documents for more information.
 *
 * @ref PCA9957HNMP Datasheet
 */

#ifndef LED_24CH_DRIVER_INC_IOIF_PCA9957HNMP_H_
#define LED_24CH_DRIVER_INC_IOIF_PCA9957HNMP_H_

#include "module.h"
/** @defgroup SPI SPI
 * @brief SPI PCA9957HNMP module driver
 * @{
 */
#ifdef IOIF_PCA9957HNMP_ENABLED

#include <string.h>

#include "ioif_spi_common.h"
#include "ioif_gpio_common.h"
#include "pca9957hnmp.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define LED_NA 0 //Not applicable, 가독성을 위해 naming 규칙 예외

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef enum _IOIF_led_color_t {
	CNULL,
	RED,
	GREEN,
	BLUE,
	WHITE
} IOIF_LED24chColor_t;

typedef enum _IOIF_led_ctrl_t {
	OFF = 0,
	ON,
	BLINK
} IOIF_LED24chCtrl_t;

typedef enum _IOIF_StatusTypeDef_t {
    IOIF_OK 	 = BSP_OK,
    IOIF_ERROR   = BSP_ERROR,
    IOIF_BUSY	 = BSP_BUSY,
	IOIF_TIMEOUT = BSP_TIMEOUT
} IOIF_StatusTypeDef_t;

typedef enum _IOIF_LEDState_t {
    IOIF_LED_STATUS_OK = 0,
    IOIF_LED_STATUS_ERROR,
    IOIF_LED_STATUS_BUSY,
    IOIF_LED_STATUS_TIMEOUT
} IOIF_LEDState_t;

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
IOIF_LEDState_t IOIF_LED24chDriverInit(IOIF_SPI_t spiChannel);
bool IOIF_LED24chSPIStart(uint8_t reg, uint8_t tdata, uint8_t* rdata, uint8_t timeout, PCA9957HNMP_SPIOP_t operation);
bool IOIF_LED24chnOECtrl(bool state);
bool IOIF_LED24chnRSTCtrl(bool state);

/*LED Control*/
void IOIF_LED24chBluetooth(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t brightness);
void IOIF_LED24chError(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, IOIF_LED24chColor_t color, uint8_t brightness);
void IOIF_LED24chMode(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, IOIF_LED24chColor_t color, uint8_t brightness);
void IOIF_LED24chBattery(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t nth, IOIF_LED24chColor_t color, uint8_t brightness);
void IOIF_LED24chAssist(IOIF_LED24chCtrl_t ctrl, uint32_t blinking_period, uint8_t nth, uint8_t brightness);
void IOIF_LEDforAssistOnebyOne(uint8_t nth, uint8_t brightness);
void IOIF_LED24chAll(IOIF_LED24chCtrl_t ctrl, uint8_t brightness);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunPca9957hnmp(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /* IOIF_PCA9957HNMP_ENABLED */
#endif /* LED_24CH_DRIVER_INC_IOIF_PCA9957HNMP_H_ */
