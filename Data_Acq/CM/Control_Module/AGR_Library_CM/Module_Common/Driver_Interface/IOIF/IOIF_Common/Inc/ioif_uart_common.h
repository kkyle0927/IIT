

#ifndef IOIF_COMMON_INC_IOIF_UART_COMMON_H_
#define IOIF_COMMON_INC_IOIF_UART_COMMON_H_

#include "bsp_uart.h"
#include "module.h"
#include "ring_buffer.h"

/** @defgroup UART UART
  * @brief UART BSP module driver
  * @{
  */
#ifdef BSP_UART_MODULE_ENABLED

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_UART_BUFFER_LENGTH		2056
#define IOIF_UART_TX_LENGTH 		244  //fdcan 프로토콜 크기랑 맞춤 -> 최대량

/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

typedef int (*IOIF_UARTUserRxCBPtr_t)(uint8_t*);
typedef void (*IOIF_UARTRxCpltPtr_t)(void* params);

typedef enum _IOIF_UART_t {
    IOIF_UART1 = BSP_UART1,
    IOIF_UART2 = BSP_UART2,
    IOIF_UART3 = BSP_UART3,
    IOIF_UART4 = BSP_UART4,
	IOIF_UART5 = BSP_UART5,
	IOIF_UART6 = BSP_UART6,
    IOIF_UART_COUNT
} IOIF_UART_t;

typedef enum _IOIF_UARTState_t {
    IOIF_UART_STATUS_OK = 0,
    IOIF_UART_STATUS_ERROR,
    IOIF_UART_STATUS_BUSY,
    IOIF_UART_STATUS_TIMEOUT,
} IOIF_UARTState_t;

typedef enum _IOIF_UARTMode_t {
	IOIF_UART_MODE_POLLING = 0,
	IOIF_UART_MODE_IT,
	IOIF_UART_MODE_DMA
} IOIF_UARTMode_t;

/**
 * @enum IOIF_UARTCallbackType_t
 * @brief UART callback types for various uart events.
 */
typedef enum _IOIF_UARTCBType_t {
    IOIF_UART_TX_CPLT_CALLBACK,
    IOIF_UART_TX_HALF_CPLT_CALLBACK,
    IOIF_UART_RX_CPLT_CALLBACK,
    IOIF_UART_RX_HALF_CPLT_CALLBACK,
    IOIF_UART_ERROR_CALLBACK,
    IOIF_UART_ABORT_CPLT_CALLBACK,
    IOIF_UART_ABORT_TRANSMIT_CPLT_CALLBACK,
    IOIF_UART_ABORT_RECEIVE_CPLT_CALLBACK,
    IOIF_UARTEX_RX_EVENT_CALLBACK,
    IOIF_UARTEX_WAKEUP_CALLBACK,
    IOIF_UARTEX_RX_FIFO_FULL_CALLBACK,
    IOIF_UARTEX_TX_FIFO_EMPTY_CALLBACK,
    IOIF_UART_CALLBACK_TYPE_COUNT,
} IOIF_UARTCBType_t;

typedef struct _IOIF_UARTStruct_t {
	IOIF_UART_t		  uart_port;
	uint32_t 		  baudrate;
	RingBufferStruct* uart_ringbufPtr;
} IOIF_UARTStruct_t;

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern uint8_t rx_packet[IOIF_UART_BUFFER_LENGTH];
extern uint8_t uart1dmaTxBuff[IOIF_UART_TX_LENGTH];
extern RingBufferStruct uart_rx_buff;

/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */

IOIF_UARTState_t IOIF_UART_Init(IOIF_UART_t ch, uint32_t baudrate);
IOIF_UARTState_t IOIF_UART_Deinit(IOIF_UART_t ch);

IOIF_UARTState_t IOIF_UART_StartReceive(IOIF_UART_t ch, uint32_t baudrate, IOIF_UARTMode_t mode);
IOIF_UARTState_t IOIF_UART_Stop_DMA(IOIF_UART_t ch);

IOIF_UARTState_t IOIF_UART_ReadData(IOIF_UART_t ch, uint8_t* readData, uint32_t length);
IOIF_UARTState_t IOIF_UART_Write(IOIF_UART_t ch, IOIF_UARTMode_t mode, uint8_t *data, uint32_t length);
uint8_t 		 IOIF_UART_Read(IOIF_UART_t ch);

void 			 IOIF_UART_UpdateHeadTail(void);
uint32_t 		 IOIF_UART_UpdateHead(void);
uint32_t 		 IOIF_UART_CalDataLength(IOIF_UART_t ch);
uint32_t 		 IOIF_UART_GetBaudrate(IOIF_UART_t ch);
IOIF_UARTState_t IOIF_UART_SetBaudrate(IOIF_UART_t ch, uint32_t baudrate);
IOIF_UARTState_t IOIF_UART_RX_BufferFlush(IOIF_UART_t ch);
void 			 IOIF_SetUARTRxCpltCB(IOIF_UART_t ch, IOIF_UARTCBType_t callbackType, IOIF_UARTUserRxCBPtr_t callback);


#endif /* BSP_UART_MODULE_ENABLED */

#endif /* IOIF_COMMON_INC_IOIF_UART_COMMON_H_ */
