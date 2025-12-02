

#include "ioif_uart_common.h"

/** @defgroup UART UART
  * @brief UART BSP module driver
  * @{
  */
#ifdef BSP_UART_MODULE_ENABLED

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
BSP_UARTMap_t UART_Handle;

/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */
static RingBufferStruct uart_rx_buff[IOIF_ASSIGN_UART_PORT_COUNT];
//uint8_t rx_packet[IOIF_UART_BUFFER_LENGTH] __attribute__((section(".uart1dmaRxBuff")));	// MPU area in RAM D3
//uint8_t rx_packet[IOIF_UART_BUFFER_LENGTH];
//uint8_t rx_packet[100];
uint8_t rx_packet[IOIF_UART_BUFFER_LENGTH] __attribute__((section(".uart1dmaRxBuff")));



/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */




/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

BSP_UART_t IOIF_UART_CH(IOIF_UART_t ch)
{

	BSP_UART_t uart_port;

//	UART_HandleTypeDef* ret = {0,};
//
	if (ch == 1) uart_port = BSP_UART1;
	else if (ch == 2) uart_port = BSP_UART2;
	else if (ch == 3) uart_port = BSP_UART3;
	else if (ch == 4) uart_port = BSP_UART4;
	else if (ch == 5) uart_port = BSP_UART5;
	else if (ch == 6) uart_port = BSP_UART6;
	else if (ch == 7) uart_port = BSP_UART7;

	return uart_port;
}

USART_TypeDef* IOIF_UART_CH_Inst(IOIF_UART_t ch)
{
	USART_TypeDef* ret = NULL;

	if (ch == 1) return USART1;
	if (ch == 2) return USART2;
	if (ch == 3) return USART3;
	if (ch == 4) return UART4;
	if (ch == 5) return UART5;

	return ret;
}

bool IOIF_UART_Init(IOIF_UART_t ch, IOIF_UARTBaudrate_t baudrate)
{
	bool ret = true;

#ifdef _USE_MCU_ST_H7

	if(BSP_InitUART((BSP_UART_t)ch,(uint32_t) baudrate) != BSP_OK)
		ret = false;
	else
		ret = true;
#endif

#ifdef _USE_MCU_ST_L5

	huart->Init.BaudRate = baudrate;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;
	huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	BSP_StatusTypeDef_t status = BSP_OK;

	if (HAL_UART_Init(huart) != HAL_OK)
	{
		status = BSP_ERROR;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		status = BSP_ERROR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		status = BSP_ERROR;
	}
	if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK)
	{
		status = BSP_ERROR;
	}

	    return status;
#endif

	return ret;
}

bool IOIF_UART_Deinit(IOIF_UART_t ch)
{
	bool ret = true;

	BSP_UART_t uart_port = IOIF_UART_CH(ch);
	BSP_DeInitUART(uart_port);

	return ret;
}


uint8_t IOIF_UART_Start(IOIF_UART_t ch, uint8_t mode, uint32_t baudrate)
{
	uint8_t 	ret = IOIF_UART_START_OK;
	uint32_t 	i=0;
	uint32_t 	ring_buff_index = 1; //Todo: ring_buff_index = IOIF_ASSIGN_UART_PORT_COUNT % ch -1; //사용중인 UART 의 최대 갯수와 실제 사용하는 UART channel 번호와 ring buffer index 맞추기
	BSP_UART_t 	uart_port = IOIF_UART_CH(ch);
    uint8_t 	tx_data  = 0;

//	UART_HandleTypeDef* bsp_uart_ch = _UART_CH(ch);

	memset(&rx_packet[0], 0, sizeof(rx_packet));				//Receive Buffer initialization

	for (i=0 ; i<=ring_buff_index; i++)							//UART 수신용 Ring Buffer 생성, UART DMA circular mode 를 사용함.
	{
		/* Ring Buffer 생성 시 buffer 용량은 충분히 크게 할 것!	\
			Push(인입) 와 pop(인출) 의 속도 차이에 따라 data overrap 현상이 발생할 수 있음! */
		RingBufferCreate(&uart_rx_buff[i], &rx_packet[0], IOIF_UART_BUFFER_LENGTH);
	}

	if(IOIF_UART_Init(ch, baudrate) != true)
	{
		return IOIF_UART_INIT_FAIL;
	}

	switch(mode)
	{
		case IOIF_UART_MODE_POLLING:
			/* TODO : implement, 하지만 polling 은 부적절함. */
		break;

		case IOIF_UART_MODE_IT:	/* Received IT 의 경우 수신받는 데이터가 많을 경우 지속적인 인터럽트가 발생함으로써 concurrent 성능을 저하시키고, \
						UART Transmit 를 할 때 Received IT disable 해주지 않으면 데이터 송신 시 장애를 일으킬 수 있음!*/
			if(BSP_RunUARTIT(uart_port, &tx_data, (uint8_t *)&rx_packet[0], sizeof(rx_packet), BSP_UART_RECEIVE_IT) != BSP_OK)							//1byte receive
				ret = IOIF_UART_START_FAIL;
		break;

		case IOIF_UART_MODE_DMA:
			if(BSP_RunUARTDMA(uart_port, &tx_data, (uint8_t *)&rx_packet[0], sizeof(rx_packet), BSP_UART_RECEIVE_DMA) != BSP_OK)
				ret = IOIF_UART_START_FAIL;
			//Todo:
//			if (HAL_UART_Receive_DMA(&huart7, (uint8_t *)&rx_packet[0], sizeof(rx_packet)) != HAL_OK) {ret = IOIF_UART_START_FAIL;}
			/* DMA buffer 와 ring buffer 의 synchronization 필요, DMA 의 CNTDR (STM32H743 의 경우 SxNDTR 임!) 의 경우 index 가 decrease 함!\
				Normal 인 경우 index 가 0이 될 시 stop, circular mode 에서는 auto reloaded 되어 다시 전송을 시작한다 */

			uart_rx_buff[ring_buff_index].head = uart_rx_buff[ring_buff_index].length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;
			uart_rx_buff[ring_buff_index].tail = uart_rx_buff[ring_buff_index].head;	// 초기 수신 DMA 실행 시 dummy data 가 들어오면 flush 처리
		break;
	}
	return ret;
}

uint8_t	IOIF_UART_Stop(uint8_t ch, uint8_t mode, uint32_t baudrate)
{
	uint8_t ret = 0;

	switch(mode)
		{
			case IOIF_UART_MODE_POLLING:
				/* TODO : implement */
			break;

			case IOIF_UART_MODE_IT:
				/* TODO : implement */
			break;

			case IOIF_UART_MODE_DMA:
				/* TODO : implement */
			break;
		}

	return ret;
}

uint8_t IOIF_UART_Read(uint8_t ch)
{
	uint8_t  ret = 0;
	uint32_t ring_buff_index = 0;//IOIF_ASSIGN_UART_PORT_COUNT % ch - 1;		//사용중인 UART 의 최대 갯수와 실제 사용하는 UART channel 번호와 ring buffer index 맞추기

	RingBufferPop(&uart_rx_buff[ring_buff_index], &ret, 1);		//ring buffer 로부터 1바이트 읽기

	return ret;
}


bool IOIF_UART_Write(uint8_t ch, uint8_t mode, uint8_t *data, uint32_t length)
{
	bool 	ret 	 = false;
	uint8_t time_out = 100;

//	UART_HandleTypeDef* bsp_uart_ch = _UART_CH(ch);
	BSP_UART_t uart_port = IOIF_UART_CH(ch);

	uint8_t rx_packet[0] = {};

	switch(mode)
	{
		case IOIF_UART_MODE_POLLING:
			if (BSP_RunUARTBlock(uart_port, data, (uint8_t *)&rx_packet[0], length, time_out, BSP_UART_TRANSMIT) == BSP_OK) {
				ret = true;}
			break;

		case IOIF_UART_MODE_IT:
			/* TODO : implement */
			break;

		case IOIF_UART_MODE_DMA:
			/* TODO : implement */
			if (BSP_RunUARTDMA(uart_port, data, (uint8_t *)&rx_packet[0], length, BSP_UART_TRANSMIT_DMA) == BSP_OK) {
				ret = true;}
			break;
	}

	return ret;
}

uint32_t IOIF_UART_IsReady(uint8_t ch)
{
	uint32_t ret = 0;

	uint32_t ring_buff_index = 0; //IOIF_ASSIGN_UART_PORT_COUNT % ch - 1;		//사용중인 UART 의 최대 갯수와 실제 사용하는 UART channel 번호와 ring buffer index 맞추기
	uint32_t buffer_length=0;

	uart_rx_buff[ring_buff_index].head = uart_rx_buff[ring_buff_index].length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;	// index 업데이트

	buffer_length = RingBufferIsAvailable(&uart_rx_buff[ring_buff_index]);

	if(buffer_length > 0)
			return ret = buffer_length;

	return ret;
}

uint32_t IOIF_UART_GetBaudrate(uint8_t ch)
{
	uint32_t ret = 0;

//	UART_HandleTypeDef* bsp_uart_ch = _UART_CH(ch);

	return ret = UART_Handle.handle->Init.BaudRate;

}

bool IOIF_UART_SetBaudrate(volatile uint8_t ch_, uint32_t baudrate)
{
	bool ret = true;
	volatile uint8_t ch = ch_;

	BSP_UART_t uart_port = IOIF_UART_CH(ch);
	uint32_t ring_buff_index= 0 ;//IOIF_ASSIGN_UART_PORT_COUNT % ch - 1;

	IOIF_UART_Deinit(ch);
	IOIF_UART_Init(ch, baudrate);

	uint8_t tx_data;

	/* Re-init. 후 다시 DMA RX start */

/*
	if(BSP_RunUARTDMA(uart_port, &tx_data, (uint8_t *)&rx_packet[0], 1, BSP_UART_RECEIVE_DMA) != HAL_OK)
			ret = IOIF_UART_START_FAIL;
*/
	BSP_RunUARTDMA(uart_port, &tx_data, (uint8_t *)&rx_packet[0], sizeof(rx_packet), BSP_UART_RECEIVE_DMA);

	uart_rx_buff[ring_buff_index].head = uart_rx_buff[ring_buff_index].length - ((DMA_Stream_TypeDef*)UART1_RX_DMA->Instance)->NDTR;
	uart_rx_buff[ring_buff_index].tail = uart_rx_buff[ring_buff_index].head;

	return ret;
}

bool IOIF_UART_RX_BufferFlush(uint8_t ch)
{
	bool ret = false;

	uint32_t ring_buff_index = 0; //IOIF_ASSIGN_UART_PORT_COUNT % ch - 1;

	if(RingBufferFlush(&uart_rx_buff[ring_buff_index]) == true)
		ret = true;

	return ret;
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */




#endif /* BSP_UART_MODULE_ENABLED */
