/**
 *-----------------------------------------------------------
 *                MDBT48Q-AT BLE Slave Device
 *-----------------------------------------------------------
 * @file mdbt42q-at.h
 * @date Created on: July 23, 2023
 * @author AngelRobotics HW Team
 * @brief Code for the MDBT48Q-AT BLE Slave Device.
 *
 *
 * Refer to the MDBT48Q-AT datasheet and related documents for more information.
 * @ref MDBT48Q-AT Datasheet
 */

#ifndef MDBT48Q_AT_INC_MDBT48Q_AT_H_
#define MDBT48Q_AT_INC_MDBT48Q_AT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

/* Write Command Definitions*/
#define BLE_SLAVE_WRITE_NAME_COMMAND 	"AT+NAME"
#define BLE_SLAVE_WRITE_NAME 			BLE_SLAVE_WRITE_NAME_COMMAND BLE_SLAVE_NODENAME
#define BLE_SLAVE_WRITE_RESET			"AT+RESET"
#define BLE_SLAVE_WRITE_ADV_START		"AT+ADVSTART"
#define BLE_SLAVE_WRITE_ADV_STOP		"AT+ADVSTOP"
#define BLE_SLAVE_WRITE_SLEEP			"AT+SLEEP"

#define BLE_SLAVE_WRITE_BAUD9600		"AT+BAUDRATE9600"
#define BLE_SLAVE_WRITE_BAUD57600		"AT+BAUDRATE57600"
#define BLE_SLAVE_WRITE_BAUD115200		"AT+BAUDRATE115200"

#define BLE_SLAVE_WRITE_PHY1MBPS		"AT+PHYMODE1MBPS"
#define BLE_SLAVE_WRITE_PHY2MBPS		"AT+PHYMODE2MBPS"

#define BLE_SLAVE_WRITE_SERIALNO		"AT+SERIALNO"
#define BLE_SLAVE_WRITE_DISCONNECT		"AT+DISCONNECT"
#define BLE_SLAVE_WRITE_DEFAULT			"AT+DEFAULT"
#define BLE_SLAVE_WRITE_MACADDR			"AT+MACADDR"

#define BLE_SLAVE_WRITE_ADV_LEDOFF		"AT+ADVPATTERN00000000"
#define BLE_SLAVE_WRITE_ADV_LEDON		"AT+ADVPATTERN01F401F4"			// 0.5msec on, 0.5msec off


/* Read Command Definitions */
#define BLE_SLAVE_READ_NAME				"AT?NAME"
#define BLE_SLAVE_READ_BAUDRATE			"AT?BAUDRATE"
#define BLE_SLAVE_READ_FW_VERSION		"AT?VERSION"
#define BLE_SLAVE_READ_SERIALNO			"AT?SERIALNO"
#define BLE_SLAVE_READ_MACADDR			"AT?MACADDR"

#define BLE_SLAVE_READ_PHYMODE			"AT?PHYMODE"

#define BLE_SLAVE_READ_CONN_INTV_MODE	"AT?CONNECTINTERVALMODE"	// To retrieve status of connection interval mode
#define BLE_SLAVE_READ_CONN_INDI		"AT?CONNECTINDICATOR"		// To retrieve logic of pin for BT-connecting indicator
#define BLE_SLAVE_READ_CONN_INTV_TIME	"AT?CONNECTINTERVALTIME"	// To retrieve value of connection interval time under Mode 2

#define BLE_SLAVE_READ_ALL_PARAM		"AT?ALLPARAMETERS"			// To retrieve value of all parameters


#define BLE_NAME_LENGTH_MAX			28		//AT+NAME + 20Bytes
#define BLE_SERIAL_LENGTH_MAX		19		//AT+SERIALNO + 8bytes
#define BLE_MAC_LENGTH_MAX			16		//AT+MACADDR + 6bytes


/* BLE IO Wait Time */
#define BLE_CMD_WAIT_50MS				50
#define BLE_CMD_WAIT_100MS				100
#define BLE_CMD_WAIT_150MS				150
#define BLE_CMD_WAIT_250MS				250
#define BLE_CMD_WAIT_300MS				300
#define BLE_CMD_WAIT_600MS				600
#define BLE_CMD_WAIT_700MS				700
#define BLE_CMD_WAIT_1000MS				1000

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/* BLE GPIO State */
typedef enum {
	MDBT42Q_GPIO_STATUS_LOW = 0,
	MDBT42Q_GPIO_STATUS_HIGH = 1
} GPIO_State;

/* BLE Setting Save*/
typedef enum {
	Save_Disable = 0,
	Save_Active = 1
} SettingSave_Option;


/* Callback Function Pointer */
typedef uint32_t	(*MDBT42QAT_CalDataLength)			(void);
typedef bool 		(*MDBT42QAT_IsConnected)			(void);
typedef uint8_t		(*MDBT42QAT_read)					(void);
//typedef bool		(*MDBT42QAT_read)					(uint8_t* readData, uint32_t length); //read burst 함수 검증 후 변경
typedef bool		(*MDBT42QAT_write)					(uint8_t* data, uint32_t length);
typedef bool		(*MDBT42QAT_WriteGPIO_HWdefault)	(GPIO_State gpio_state);
typedef bool 		(*MDBT42QAT_WriteGPIO_PD) 			(GPIO_State gpio_state);
typedef void		(*MDBT42QAT_IOwait)					(uint32_t ms_wait);

typedef struct {
	MDBT42QAT_CalDataLength			mdbt42qat_data_length;
	MDBT42QAT_IsConnected			mdbt42qat_connected;
	MDBT42QAT_read					mdbt42qat_read;
	MDBT42QAT_write					mdbt42qat_write;
	MDBT42QAT_WriteGPIO_HWdefault	mdbt42qat_hw_default;
	MDBT42QAT_WriteGPIO_PD 			mdbt42qat_uart_enable;
	MDBT42QAT_IOwait				mdbt42qat_IOwait;
} MDBT42QAT_CallbackStruct;


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

/* BLE Basic Function Definitions */

bool  		MDBT42Q_Init(MDBT42QAT_CallbackStruct* BLE_callback);		/* Not defined */
bool 		MDBT42Q_Deinit(void);	/* Not defined */

bool 		MDBT42Q_AT_ReadCMD(uint8_t* AT_READ_CMD, uint32_t size);
bool 		MDBT42Q_AT_WriteCMD(uint8_t * AT_WRITE_CMD, uint32_t size, SettingSave_Option save_option);
bool 		MDBT42Q_AT_WriteCMD(uint8_t * AT_WRITE_CMD, uint32_t size, SettingSave_Option save_option);
bool 		MDBT42Q_AT_WriteCMDwhenConnected(uint8_t *AT_WRITE_CMD, uint32_t size,  SettingSave_Option save_option);
uint32_t 	MDBT42Q_ReceivedData(uint8_t uart_ch, uint8_t* AT_data, uint32_t length);
bool 		MDBT42Q_TransmitData(uint8_t* data, uint32_t length);

void		MDBT42Q_SetNodeName(uint8_t* name, uint8_t* data);
void 		MDBT42Q_SetNodeSerial(uint8_t* serialno, uint8_t* data);
void 		MDBT42Q_SetNodeMac(uint8_t* macad, uint8_t* data);
void	  	MDBT42Q_HW_Default(void);

#endif /* MDBT48Q_AT_INC_MDBT48Q_AT_H_ */
