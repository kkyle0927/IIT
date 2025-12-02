/**
*-----------------------------------------------------------
*                MDBT48Q-AT BLE Slave Device
*-----------------------------------------------------------
* @file ioif_mdbt42q-at.h
* @date Created on: Sep 19, 2023
* @author AngelRobotics HW Team
* @brief Code for the MDBT48Q-AT BLE Slave Device.
*
*
* Refer to the MDBT48Q-AT datasheet and related documents for more information.
* @ref MDBT48Q-AT Datasheet
*/

#ifndef BLE_INC_IOIF_BLEIF_MDBT42Q_AT_H_
#define BLE_INC_IOIF_BLEIF_MDBT42Q_AT_H_

#include "module.h"

/** @defgroup UART UART
 * @brief UART MDBT42Q-AT BLE module driver
 * @{
 */

#ifdef IOIF_MDBT42Q_AT_ENABLED

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "mdbt42q-at.h"
#include "bsp_common.h"
#include "bsp_uart.h"

#include "ioif_usb_common.h"
#include "ioif_uart_common.h"
#include "ioif_gpio_common.h"
#include "ioif_sysctrl.h"


/**
*-----------------------------------------------------------
*              MACROS AND PREPROCESSOR DIRECTIVES
*-----------------------------------------------------------
* @brief Directives and macros for readability and efficiency.
*/
#define IOIF_BLE_DATA_BUFFER_INDEX			1024
#define IOIF_BLE_CMD_BUFFER_INDEX			35

#define IOIF_BLE_NAME_LENGTH_MAX			27		//AT+NAME(7 bytes) + 20 bytes
#define IOIF_BLE_MAC_LENGTH_MAX				16		//AT+MACADDR(10 bytes) + 6 bytes
//#define IOIF_BLE_SERIAL_LENGTH_MAX			19		//AT+SERIALNO(11 bytes) + 8 bytes //Serial Num 미사용이지만 혹시 몰라 남겨둠..

#define IOIF_BLE_ACK_BAUDRATE9600			"0 baudrate9600"
#define IOIF_BLE_ACK_BAUDRATE115200			"4 baudrate115200"
#define IOIF_BLE_ACK_BAUD115200_SUCCESS		"4 baudrate115200reset success"

#define IOIF_BLE_BLE_INIT_TIMEOUT			1000	//10ms * BLE_INIT_TIMEOUT = 10second
#define IOIF_BLE_TEST_SEND_PACKET			"Send Test Packet transmit, BLE is connected successfully!!\r\n"

#define IOIF_BLE_DEFAULT_NAME				"Raytac AT-UART"
#define IOIF_BLE_DEFAULT_BAUDRATE			"0 baudrate9600"

/**
*------------------------------------------------------------
*                     TYPE DECLARATIONS
*------------------------------------------------------------
* @brief Custom data types and structures for the module.
*/
typedef enum _IOIF_BLEInit_t {
	IOIF_BLE_INIT_OK = 0,
	IOIF_BLE_INIT_NOT_READY = 0,
	IOIF_BLE_INIT_ACK_ERROR,
	IOIF_BLE_INIT_READ_ERROR,
	IOIF_BLE_INIT_WRITE_ERROR,
	IOIF_BLE_INIT_ERROR,
} IOIF_BLEInit_t;

typedef enum _IOIF_BLECommState_t {
	IOIF_BLE_IDLE, 			// 대기 상태 (=disconnect), disconnect 후 대기할 때
	IOIF_BLE_ADVERTISING,	// disconnect 상태에서 연결할 app을 찾을 때
	IOIF_BLE_CONNECTED,
	IOIF_BLE_ERROR,
} IOIF_BLECommState_t;

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
/* Init & Settings */
bool 			IOIF_BLE_Init(uint32_t baudrate);
IOIF_BLEInit_t 	IOIF_BLE_SetBaudrate115200(void);
IOIF_BLEInit_t 	IOIF_BLE_SetConfigParams(uint8_t* mac, uint8_t* name);
void 			IOIF_BLE_ResetHW();
void 			IOIF_BLE_SetToDefaultSW();
void 			IOIF_BLE_SetToDefaultHW();

/* Data */
uint32_t IOIF_BLE_ReadCommData(uint8_t* buffer, uint32_t length);
void 	 IOIF_BLE_TransmitCommData(uint8_t* dataBuff, uint32_t dataSize);

/* BLE Connect */
bool IOIF_BLE_IsConnected(void);
bool IOIF_BLE_StopPairing(void);
bool IOIF_BLE_StartPairing(void);
bool IOIF_BLE_Disconnect(void);

/* Test code */
void IOIF_BLE_AckTest(void);

/* BLE Callback Functions */
uint32_t 	IOIF_BLE_CB_CalDataLength(void);
bool 		IOIF_BLE_CB_CtrlPDpin (GPIO_State gpio_state);
bool 		IOIF_BLE_CB_CtrlHWdefaultpin(GPIO_State gpio_state);
uint8_t 	IOIF_BLE_CB_ReadData(void);
bool 		IOIF_BLE_CB_WriteData(uint8_t* data, uint32_t length);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunMdbt42qat(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /*IOIF_MDBT42Q_AT_ENABLED*/
#endif /* BLE_INC_IOIF_BLEIF_MDBT42Q_AT_H_ */
