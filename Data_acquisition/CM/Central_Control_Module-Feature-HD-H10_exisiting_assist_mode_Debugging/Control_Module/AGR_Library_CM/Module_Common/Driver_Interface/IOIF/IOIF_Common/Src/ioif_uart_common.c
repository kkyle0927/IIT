

#include "ioif_uart_common.h"

/** @defgroup UART UART
  * @brief UART BSP module driver
  * @{
  */
#ifdef BSP_UART_MODULE_ENABLED


//Todo: Separate the UART code specifically for BLE
/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

DMA_HandleTypeDef* UART1_RX_DMA = &hdma_usart1_rx;
DMA_HandleTypeDef* uart1_tx_dma = &hdma_usart1_tx;

RingBufferStruct uart_rx_buff __attribute__((section(".uart1dmaRxBuff")));
uint8_t rx_packet[IOIF_UART_BUFFER_LENGTH] __attribute__((section(".uart1dmaRxBuff")));
uint8_t uart1dmaTxBuff[IOIF_UART_TX_LENGTH] __attribute__((section(".uart1dmaTxBuff")));

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */


static IOIF_UARTUserRxCBPtr_t userRxCBPtrUART1;
static volatile bool uart1_dma_tx_ready = true;

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

static void IOIF_AllocateUART1UserRxCB(IOIF_UARTUserRxCBPtr_t funcPtr);
static void RunUART1CpltRxCB(void* params);
static void RunUART1CpltTxCB(void* params);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */


IOIF_UARTState_t IOIF_UART_Init(IOIF_UART_t ch, uint32_t baudrate)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;

	if(BSP_InitUART(ch, baudrate) != BSP_OK)
		ret = IOIF_UART_STATUS_ERROR;

	/* Set Registration TX Complete Callback */
	BSP_SetUARTCB((BSP_UART_t)ch, BSP_UART_TX_CPLT_CALLBACK, RunUART1CpltTxCB, NULL);

	return ret;
}

IOIF_UARTState_t IOIF_UART_Deinit(IOIF_UART_t ch)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;

	if(BSP_DeInitUART(ch) != BSP_OK)
		ret = IOIF_UART_STATUS_ERROR;

	return ret;
}


IOIF_UARTState_t IOIF_UART_StartReceive(IOIF_UART_t ch, uint32_t baudrate, IOIF_UARTMode_t mode)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_ERROR;
    uint8_t dummy_tx = 0;

	/* Ring Buffer 생성 시 buffer 용량은 충분히 크게 할 것!	\
		Push(인입) 와 pop(인출) 의 속도 차이에 따라 data overrap 현상이 발생할 수 있음! */
    memset(&rx_packet[0], 0, IOIF_UART_BUFFER_LENGTH);				//Receive Buffer initialization
    RingBufferCreate(&uart_rx_buff, &rx_packet[0], IOIF_UART_BUFFER_LENGTH);

    if(IOIF_UART_Init(ch, baudrate) != IOIF_UART_STATUS_OK)
    	return ret;

    switch(mode)
    {
    case IOIF_UART_MODE_POLLING:
    	/* 구현 필요 */
    	break;
    case IOIF_UART_MODE_IT:
    	/* Received IT 의 경우 수신받는 데이터가 많을 경우 지속적인 인터럽트가 발생함으로써 concurrent 성능을 저하시키고, \
           UART Transmit 할 때 Received IT disable 해주지 않으면 데이터 송신 시 장애를 일으킬 수 있음!*/
    	break;
    case IOIF_UART_MODE_DMA:
    	if(BSP_RunUARTDMA(ch, &dummy_tx, (uint8_t *)&rx_packet[0], IOIF_UART_BUFFER_LENGTH, BSP_UART_RECEIVE) == BSP_OK)
    		ret = IOIF_UART_STATUS_OK;

    	/* DMA buffer index 와 ring buffer index 의 synchronization 필요, DMA 의 CNTDR (STM32H743 의 경우 SxNDTR 임!) 의 경우 index 가 decrease 함!\
    			Normal 인 경우 index 가 0이 될 시 stop, circular mode 에서는 auto reloaded 되어 다시 전송을 시작한다 */
    	uart_rx_buff.head = uart_rx_buff.length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;
    	uart_rx_buff.tail = uart_rx_buff.head;	// 초기 수신 DMA 실행 시 dummy data 가 들어오면 flush 처리
    	break;
     default:
    	break;
    }

	return ret;
}

IOIF_UARTState_t IOIF_UART_Stop(IOIF_UART_t ch)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;

	return ret = IOIF_UART_Deinit(ch);
}

IOIF_UARTState_t IOIF_UART_ReadData(IOIF_UART_t ch, uint8_t* readData, uint32_t length)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;

	switch(ch)
	{
	case IOIF_UART1:
		if(readData != NULL)
			RingBufferPop(&uart_rx_buff, readData, length);
		break;

	default :
		break;
	}

	return ret;
}

uint8_t IOIF_UART_Read(IOIF_UART_t ch)
{
	uint8_t ret = 0;

	switch(ch)
	{
	case IOIF_UART1:
		RingBufferPop(&uart_rx_buff, &ret, 1);
		break;

	default:
		break;
	}

	return ret;
}

IOIF_UARTState_t IOIF_UART_Write(IOIF_UART_t ch, IOIF_UARTMode_t mode, uint8_t *data, uint32_t length)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;
	uint32_t time_out = 300;
	uint8_t dummy_rx = 0;

	//memcpy(uart1dmaTxBuff, data, length);

	for(uint16_t i=0; i<length ; i++)
	{
		uart1dmaTxBuff[i] = data[i];
	}

	switch(mode)
	{
		case IOIF_UART_MODE_POLLING:
			if (BSP_RunUARTBlock(ch, data, &dummy_rx, length, time_out, BSP_UART_TRANSMIT) != BSP_OK)
				ret = IOIF_UART_STATUS_ERROR;
			break;
		case IOIF_UART_MODE_IT:
			/* 구현 필요 */
			break;
		case IOIF_UART_MODE_DMA:
			//if (BSP_RunUARTDMA(ch, data, &dummy_rx, length, BSP_UART_TRANSMIT) != BSP_OK)
			if(uart1_dma_tx_ready == true)
			{
				uart1_dma_tx_ready = false;		//sending start
				if (BSP_RunUARTDMA(ch, uart1dmaTxBuff, &dummy_rx, length, BSP_UART_TRANSMIT) != BSP_OK)
					ret = IOIF_UART_STATUS_ERROR;
			}
			break;
		default :
			break;
	}

	return ret;
}

void IOIF_UART_UpdateHeadTail(void)
{
	// DMA를 통해 수신된 데이터의 위치를 업데이트
	uart_rx_buff.head = uart_rx_buff.length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;
	uart_rx_buff.tail = uart_rx_buff.head;
}

uint32_t IOIF_UART_UpdateHead(void)
{
    // DMA를 통해 수신된 데이터의 위치를 업데이트
    uart_rx_buff.head = uart_rx_buff.length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR; // index 업데이트

	return uart_rx_buff.head;
}

uint32_t IOIF_UART_CalDataLength(IOIF_UART_t ch)
{
	uint32_t buff_length = 0;

	switch(ch)
	{
	case IOIF_UART1:

		/* ring buffer head update */
		uart_rx_buff.head = uart_rx_buff.length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR; // index 업데이트

		/* return length, 현재 'head' 와 현재 'tail' 의 차이 , 즉 가용한 ring buffer 수) */
		buff_length = RingBufferIsAvailable(&uart_rx_buff);

		break;

	default:
		break;
	}

	return buff_length;
}

uint32_t IOIF_UART_GetBaudrate(IOIF_UART_t ch)
{
	return BSP_GetBaudRateUART(ch);
}

IOIF_UARTState_t IOIF_UART_SetBaudrate(IOIF_UART_t ch, uint32_t baudrate)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_ERROR;

	/* 1. UART De-init */
	IOIF_UART_Deinit(ch);

	/* 2. UART init with new baudrate */
	IOIF_UART_Init(ch, baudrate);

	/* 3. Check update baudrate */
	if(IOIF_UART_GetBaudrate(ch) == baudrate)
		ret = IOIF_UART_STATUS_OK;

	switch(ch)
	{
	case IOIF_UART1:

		uint8_t dummy_tx = 0;
		/* Re-init. 후 다시 DMA RX start */
		if(BSP_RunUARTDMA(ch, &dummy_tx, (uint8_t *)&rx_packet[0], sizeof(rx_packet), BSP_UART_RECEIVE) == BSP_OK)
			ret = IOIF_UART_STATUS_OK;

		/* ring-buffer sync. & flush */
		uart_rx_buff.head = uart_rx_buff.length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;
		uart_rx_buff.tail = uart_rx_buff.head;
		break;
	default:
		break;
	}

	return ret;
}

IOIF_UARTState_t IOIF_UART_RX_BufferFlush(IOIF_UART_t ch)
{
	IOIF_UARTState_t ret = IOIF_UART_STATUS_OK;

	switch(ch)
	{
	case IOIF_UART1:
		RingBufferFlush(&uart_rx_buff);
		break;
	default :
		break;
	}

	return ret;
}

void IOIF_SetUARTRxCpltCB(IOIF_UART_t ch, IOIF_UARTCBType_t callbackType, IOIF_UARTUserRxCBPtr_t callback)
{
    BSP_SetUARTCB((BSP_UART_t)ch, (BSP_UARTCBType_t)callbackType, RunUART1CpltRxCB, NULL);

	IOIF_AllocateUART1UserRxCB(callback);
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Allocating RxFifo0Callback function
 */
static void IOIF_AllocateUART1UserRxCB(IOIF_UARTUserRxCBPtr_t funcPtr)
{
	userRxCBPtrUART1 = funcPtr;
}

static void RunUART1CpltRxCB(void* params)
{
	if (userRxCBPtrUART1 != NULL) {
		userRxCBPtrUART1(rx_packet);
	}
}

static void RunUART1CpltTxCB(void* params)
{
	uart1_dma_tx_ready = true;
}


#endif /* BSP_UART_MODULE_ENABLED */
