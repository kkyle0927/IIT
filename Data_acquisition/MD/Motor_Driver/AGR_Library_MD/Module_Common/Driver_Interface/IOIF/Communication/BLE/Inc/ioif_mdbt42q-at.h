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
  * @brief I2C ICM20608G module driver
  * @{
  */

#ifdef IOIF_MDBT42Q_AT_ENABLED

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "bsp_uart.h"

#include "ioif_usb_common.h"
#include "ioif_uart_common.h"
#include "ioif_gpio_common.h"
#include "mdbt42q-at.h"


/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_BLE_DATA_BUFFER_INDEX			1024
#define IOIF_BLE_CMD_BUFFER_INDEX			30
#define IOIF_BLE_NAME_LENGTH_MAX			28		//AT+NAME + 20Bytes
#define IOIF_BLE_SERIAL_LENGTH_MAX			19		//AT+SERIALNO + 8bytes
#define IOIF_BLE_MAC_LENGTH_MAX				22		//AT+MACADDR + 12bytes
#define IOIF_BLE_ACK_BAUDRATE9600			"0 baudrate9600"
#define IOIF_BLE_ACK_BAUDRATE115200			"4 baudrate115200"
#define IOIF_BLE_ACK_BAUD115200_SUCCESS		"4 baudrate115200reset success"

#define IOIF_BLE_BLE_INIT_TIMEOUT			1000	//10ms * BLE_INIT_TIMEOUT = 10second
#define IOIF_BLE_TEST_SEND_PACKET			"Send Test Packet transmit, BLE is connected successfully!!\r\n"

/* H/W Definitions */
#define IOIF_BLE_SLAVE_NODENAME				"AngelSUIT_Test"	//Max length 20 characters
#define IOIF_BLE_SLAVE_SERIAL				"00000001"				//fixed 8-character width
#define IOIF_BLE_SLAVE_MAC					"10000"			//12byte(Hex), MAC address, Written order is from MSB byte to LSB

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */
typedef enum _IOIF_BLEInit_t {
	IOIF_BLE_INIT_OK = 0,
	IOIF_BLE_INIT_ACK_ERROR,
	IOIF_BLE_INIT_ERROR,
} IOIF_BLEInit_t;

typedef enum _IOIF_BLEConnect_t {
    IOIF_BLE_Connected = 0,
	IOIF_BLE_Disconnected,
} IOIF_BLEConnect_t;


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

void IOIF_BLE_ConnectGPIO_CB_Set(void);				// Gpio Callback 설정
bool IOIF_BLE_Ready(); 								// BLE Device 에 필요한 환경 변수 설정
bool IOIF_BLE_ConfigParameters_Set(void);			// BLE Configuration Parameter Setting 함수
int  IOIF_BLE_DataRead_Test(void);					// Routine 에 사용할 함수, BLE Data Read
int  IOIF_BLE_DataWrite_Test(void);					// Routine 에 사용할 함수, BLE Data Write (Test)
bool IOIF_IsBLEConnected(void);

/* BLE Callback Functions */
uint32_t 	IOIF_BLE_UART_IsReady(void);
bool 		IOIF_BLE_Write_GPIO_State (GPIO_State gpio_state);
uint32_t 	IOIF_BLE_UART_GetBaudrate (void);
bool 		IOIF_BLE_UART_SetBaudrate (uint32_t baudrate);
uint8_t 	IOIF_BLE_UART_Read(void);
bool 		IOIF_BLE_UART_Write(uint8_t* data, uint32_t length);
bool 		IOIF_BLE_HWDefault (GPIO_State gpio_state);

/*test code*/
void IOIF_BLE_Ack_Test(void);

#endif /*IOIF_MDBT42Q_AT_ENABLED*/
#endif /* BLE_INC_IOIF_BLEIF_MDBT42Q_AT_H_ */
