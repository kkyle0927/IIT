/**
 *-----------------------------------------------------------
 *                ESP32 WiFi - BT Combo Master Device
 *-----------------------------------------------------------
 * @file ioif_esp32.c
 * @date Created on: Nov 25, 2024
 * @author AngelRobotics FW Team
 * @brief Code for the ESP32 Wifi-BT Combo Master Device.
 *
 *
 * Refer to the ESP32 datasheet and related documents for more information.
 * @ref ESP32 Datasheet
 */

#ifndef MODULE_COMMON_DRIVER_INTERFACE_IOIF_COMMUNICATION_ESP32_INC_IOIF_ESP32_H_
#define MODULE_COMMON_DRIVER_INTERFACE_IOIF_COMMUNICATION_ESP32_INC_IOIF_ESP32_H_

#include "module.h"

/** @defgroup SPI SPI
 * @brief UART ESP32 Wifi-BT Combo module driver
 * @{
 */

#ifdef IOIF_ESP32_ENABLED

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ioif_uart_common.h"
#include "ioif_spi_common.h"
#include "ioif_gpio_common.h"
#include "ring_buffer.h"
#include "mdbt42q-at.h"

/**
*-----------------------------------------------------------
*              MACROS AND PREPROCESSOR DIRECTIVES
*-----------------------------------------------------------
* @brief Directives and macros for readability and efficiency.
*/

#define IOIF_ESP32_WAIT_100MS					100

#define IOIF_ESP32_SPI_TX_BUFF_LENGTH 			1000
#define IOIF_ESP32_SPI_RX_BUFF_LENGTH			1000
#define IOIF_ESP32_SPI3_RX_RING_BUFF_LENGTH		1024

#define IOIF_ESP32_BT_DATA_BUFFER_INDEX			1024
#define IOIF_ESP32_BT_CMD_BUFFER_INDEX			35

#define IOIF_ESP32_BT_TEST_SEND_PACKET			"Send Test Packet transmit, BLE is connected successfully!!\r\n"

/**
*------------------------------------------------------------
*                     TYPE DECLARATIONS
*------------------------------------------------------------
* @brief Custom data types and structures for the module.
*/

typedef enum _IOIF_ESP32_Init_t {
	IOIF_ESP32_INIT_OK = 0,
	IOIF_ESP32_INIT_ERROR,
} IOIF_ESP32_Init_t;

typedef enum _IOIF_ESP32_SPI_Comm_t {
	IOIF_ESP32_SPI_COMM_OK = 0,
	IOIF_ESP32_SPI_TX_ERROR,
	IOIF_ESP32_SPI_RX_ERROR,
	IOIF_ESP32_SPI_BUSY,
} IOIF_ESP32_SPI_Comm_t;

typedef enum _IOIF_ESP32_UART_Comm_t {
	IOIF_ESP32_UART_COMM_OK = 0,
	IOIF_ESP32_UART_TX_ERROR,
	IOIF_ESP32_UART_RX_ERROR,
	IOIF_ESP32_UART_BUSY,
} IOIF_ESP32_UART_Comm_t;

typedef enum _IOIF_ESP32_GPIO_t { //todo : gpio refactoring
	IOIF_ESP32_BT_CONNECT_PORT 	= IOIF_GPIO_PORT_E,
	IOIF_ESP32_BT_CONNECT_PIN 	= BLE_BT_CONNECT_Pin,
	IOIF_ESP32_BT_RESET_PORT 	= IOIF_GPIO_PORT_E,
	IOIF_ESP32_BT_RESET_PIN 	= BLE_RESET_Pin,
	IOIF_ESP32_BT_READY_PORT	= IOIF_GPIO_PORT_E,
	IOIF_ESP32_BT_READY_PIN		= BLE_BT_BLE_RDY_Pin,
	IOIF_ESP32_UART_PORT		= IOIF_UART1,
	IOIF_ESP32_SPI_CS_PORT  	= IOIF_GPIO_PORT_A,
	IOIF_ESP32_SPI_CS_PIN		= ESP32_SPI_NSS_Pin,
	IOIF_ESP32_SPI_PORT			= IOIF_SPI3,//BSP_SPI3,
} IOIF_ESP32_GPIO_t ;

typedef enum _IOIF_ESP32_BT_Comm_t {
	IOIF_ESP32_BT_IDLE, 			// 대기 상태 (=disconnect), disconnect 후 대기할 때
	IOIF_ESP32_BT_ADVERTISING,	// disconnect 상태에서 연결할 app을 찾을 때
	IOIF_ESP32_BT_CONNECTED,
	IOIF_ESP32_BT_ERROR,
} IOIF_ESP32_BT_Comm_t;

/* Callback Function Pointer */
typedef uint32_t	(*ESP32_BT_CalDataLength)			(void);
typedef bool 		(*ESP32_BT_IsConnected)				(void);
typedef uint8_t		(*ESP32_BT_read)					(void);
typedef bool		(*ESP32_BT_write)					(uint8_t* data, uint32_t length);
typedef bool		(*ESP32_BT_WriteGPIO_HWdefault)		(GPIO_PinState gpio_state);
typedef bool 		(*ESP32_BT_WriteGPIO_PD) 			(GPIO_PinState gpio_state);
typedef void		(*ESP32_BT_IOwait)					(uint32_t ms_wait);

typedef struct {
	ESP32_BT_CalDataLength			esp32_bt_data_length;
	ESP32_BT_IsConnected			esp32_bt_connected;
	ESP32_BT_read					esp32_bt_read;
	ESP32_BT_write					esp32_bt_write;
	ESP32_BT_WriteGPIO_PD 			esp32_bt_uart_enable;
	ESP32_BT_IOwait					esp32_bt_IOwait;
} ESP32_BT_CallbackStruct;

/**
*------------------------------------------------------------
*                      GLOBAL VARIABLES
*------------------------------------------------------------
* @brief Extern declarations for global variables.
*/

extern DMA_HandleTypeDef hdma_spi3_rx;
extern volatile bool esp32_spitxrx_cplt;
extern volatile bool esp32_spitx_cplt;
extern volatile bool esp32_spirx_cplt;

extern uint8_t SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH]  __attribute__((section(".spi3TxBuff")));
extern uint8_t SPI3_DMA_RX_BUFF[IOIF_ESP32_SPI_RX_BUFF_LENGTH]  __attribute__((section(".spi3RxBuff")));


/**
*------------------------------------------------------------
*                     FUNCTION PROTOTYPES
*------------------------------------------------------------
* @brief Function prototypes declaration for this module.
*/

bool 					IOIF_ESP32_Init(void);
void					IOIF_ESP32_GPIOCB(void);
/* UART - BT */
bool 					IOIF_ESP32_BT_IsConnected(void);
void 					IOIF_ESP32_BT_Disconnected(void);
void 					IOIF_ESP32_BT_StopPairing(void);
void 					IOIF_ESP32_BT_StartPairing(void);
void 					IOIF_ESP32_BT_ResetHW(void);
bool 					IOIF_ESP32_BT_IsReady(void);
bool 					IOIF_ESP32_BT_TransmitData(uint8_t* dataBuff, uint32_t dataSize);
uint32_t 				IOIF_ESP32_BT_ReadCommData(uint8_t* buffer, uint32_t length);


/* SPI - Wifi */
IOIF_ESP32_SPI_Comm_t  	IOIF_ESP32_SPI_TX(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl);
IOIF_ESP32_SPI_Comm_t  	IOIF_ESP32_SPI_RX(uint8_t *pRxData, uint16_t size, bool cs_autoctrl);

//IOIF_ESP32_SPI_Comm_t  IOIF_ESP32_StartReceiveBuff(void);
//IOIF_ESP32_SPI_Comm_t  IOIF_ESP32_RXBuffFlush(void);
//IOIF_ESP32_SPI_Comm_t  IOIF_ESP32_RingBufferUpdate(void);

/* Callback Functions */
uint32_t 				IOIF_ESP32_CB_CalDataLength(void);
bool 					IOIF_ESP32_CB_CtrlPDpin (GPIO_PinState gpio_state);
uint8_t 				IOIF_ESP32_CB_ReadData(void);

#ifdef _USE_DEBUG_CLI
#include "cli.h"
void CLI_RunEsp32(cli_args_t *args);
#endif /*_USE_DEBUG_CLI*/

#endif /*IOIF_ESP32_ENABLED*/

#endif /* MODULE_COMMON_DRIVER_INTERFACE_IOIF_COMMUNICATION_ESP32_INC_IOIF_ESP32_2_H_ */
