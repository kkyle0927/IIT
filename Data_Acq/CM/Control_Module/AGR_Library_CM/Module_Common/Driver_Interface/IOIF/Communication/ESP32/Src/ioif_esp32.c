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

#include "ioif_esp32.h"

/** @defgroup SPI SPI
 * @brief UART ESP32 Wifi-BT Combo module driver
 * @{
 */

#ifdef IOIF_ESP32_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */
bool bt_connection_status = false;

uint8_t SPI3_DMA_TX_BUFF[IOIF_ESP32_SPI_TX_BUFF_LENGTH]  __attribute__((section(".spi3TxBuff"))) = {0,};
uint8_t SPI3_DMA_RX_BUFF[IOIF_ESP32_SPI_RX_BUFF_LENGTH]  __attribute__((section(".spi3RxBuff"))) = {0,};
RingBufferStruct esp32_spi_rx_buff __attribute__((section(".esp32spiRxRingBuff")));
uint8_t spi3_rx_ring_buff[IOIF_ESP32_SPI3_RX_RING_BUFF_LENGTH] __attribute__((section(".esp32spiRxRingBuff")));

DMA_HandleTypeDef* SPI3_RX_DMA = &hdma_spi3_rx;

static uint8_t BT_DataBuff	 [IOIF_ESP32_BT_DATA_BUFFER_INDEX];
static uint8_t BT_ATCMDBuff	 [IOIF_ESP32_BT_CMD_BUFFER_INDEX];
static uint8_t BT_ACK		 [IOIF_ESP32_BT_CMD_BUFFER_INDEX];

ESP32_BT_CallbackStruct esp32_bt_CallbackStruct = {
		NULL, NULL, NULL, NULL, NULL, NULL
};

/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */



/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */

static volatile bool wifi_spitransmit_cs_autocontrol;
static volatile bool wifi_spireceive_cs_autocontrol;

volatile bool esp32_spitxrx_cplt = false;
volatile bool esp32_spitx_cplt = false;
volatile bool esp32_spirx_cplt = false;

volatile bool ESP32_IsWiFiConnected = false;
volatile bool ESP32_IsSPIReady = false;

#ifdef _USE_DEBUG_CLI
static bool moduleinit_res = false;
#endif /*_USE_DEBUG_CLI*/

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
static void ESP32_SPI_TxCB(void* params);
static void ESP32_SPI_RxCB(void* params);
static void ESP32_SPI_TxRxCB(void* params);

static void ESP32_WifiConnectCB(uint16_t gpioPin);
static void ESP32_SPIReadyCB(uint16_t gpioPin);
static bool ESP32_WIFI_SPI_StatusCheck(void);

static void updateBTStatus (uint16_t gpioPin);
static bool regBTCallbacks();
/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool IOIF_ESP32_Init(void)
{
	/* UART */
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_G, BLE_3V3_PWR_EN_Pin, IOIF_GPIO_PIN_SET); //device on
	IOIF_ESP32_BT_ResetHW();
	// GPIO Callback
	IOIF_SetGPIOCB(IOIF_ESP32_BT_CONNECT_PIN, IOIF_GPIO_EXTI_CALLBACK, updateBTStatus);

	/* SPI */
	// CS High
	IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
	// Transmit, Receive Callback
	BSP_SetSPICB(IOIF_ESP32_SPI_PORT, BSP_SPI_TX_CPLT_CALLBACK, ESP32_SPI_TxCB, NULL);
	BSP_SetSPICB(IOIF_ESP32_SPI_PORT, BSP_SPI_RX_CPLT_CALLBACK, ESP32_SPI_RxCB, NULL);
	BSP_SetSPICB(IOIF_ESP32_SPI_PORT, BSP_SPI_TXRX_CPLT_CALLBACK, ESP32_SPI_TxRxCB, NULL);

	return regBTCallbacks();
}

void IOIF_ESP32_GPIOCB(void)
{
	// ESP GPIO Callback
	IOIF_SetGPIOCB(WIFI_CONNECT_Pin, IOIF_GPIO_EXTI_CALLBACK, ESP32_WifiConnectCB);
	IOIF_SetGPIOCB(WIFI_SPI_RDY_Pin, IOIF_GPIO_EXTI_CALLBACK, ESP32_SPIReadyCB);
}

bool IOIF_ESP32_BT_IsConnected()
{
	return bt_connection_status;
}

void IOIF_ESP32_BT_Disconnected()
{
	//Todo : 구현
}

void IOIF_ESP32_BT_StopPairing()
{
	//Todo : 구현
}


void IOIF_ESP32_BT_StartPairing()
{
	//Todo : 구현
}

bool IOIF_ESP32_BT_IsReady(void)
{
	if(IOIF_ReadGPIOPin(IOIF_ESP32_BT_READY_PORT, IOIF_ESP32_BT_READY_PIN) == IOIF_GPIO_PIN_SET)
		return true;

	else return false;
}

bool IOIF_ESP32_BT_TransmitData(uint8_t* dataBuff, uint32_t dataSize)
{
	esp32_bt_CallbackStruct.esp32_bt_write(dataBuff, dataSize);

	return true;
}

uint32_t IOIF_ESP32_BT_ReadCommData(uint8_t* buffer, uint32_t length)
{	/* 링버퍼로부터 읽은 바이트 수를 return, buffer에 data copy */
	uint32_t ret = 0;
	uint32_t buffer_length = 0;

	buffer_length = esp32_bt_CallbackStruct.esp32_bt_data_length();			//UART RX ring buffer 에 읽어올 데이터가 있는지 확인
	if(buffer_length <= 0)
		return 0;

	for (uint32_t index = 0; index < length; index++ ) {
		buffer[index] = esp32_bt_CallbackStruct.esp32_bt_read();
	}

	//	/* burst read */ //Todo: burst 함수 검증 후 이 함수로 대체
	//	mdbt42qat_callback.mdbt42qat_read(AT_data, buffer_length);

	return ret = buffer_length;
}

void IOIF_ESP32_BT_ResetHW(void)
{
	IOIF_WriteGPIOPin(IOIF_ESP32_BT_RESET_PORT, IOIF_ESP32_BT_RESET_PIN, IOIF_GPIO_PIN_SET);
	BSP_CommonSysDelay(IOIF_ESP32_WAIT_100MS);
	IOIF_WriteGPIOPin(IOIF_ESP32_BT_RESET_PORT, IOIF_ESP32_BT_RESET_PIN, IOIF_GPIO_PIN_RESET);
}

IOIF_ESP32_SPI_Comm_t IOIF_ESP32_SPI_TX(uint8_t *pTxData, uint8_t *pRxData, uint16_t size, bool cs_autoctrl)
{
	IOIF_ESP32_SPI_Comm_t ret = IOIF_ESP32_SPI_COMM_OK;

	/* ESP32 SPI Status Check */
	if((ESP32_IsSPIReady != true) || (ESP32_WIFI_SPI_StatusCheck() != true))
		return ret = IOIF_ESP32_SPI_BUSY;

	/* CS Pin Control */
	if(cs_autoctrl == true)
		wifi_spitransmit_cs_autocontrol = true;
	else
		wifi_spitransmit_cs_autocontrol = false;

	/* 1. CS is Low */
	IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_RESET);

	/* 2. Send Data via SPI DMA */
	if(pRxData != NULL){
		esp32_spitxrx_cplt = false;
		if(BSP_RunSPIDMA(IOIF_ESP32_SPI_PORT, pTxData, pRxData, size, BSP_SPI_TRANSMIT_RECEIVE_DMA) != BSP_OK)  //SPI Transfer & Receive
			ret = IOIF_ESP32_SPI_TX_ERROR;
	}
	else if (pRxData == NULL){
		esp32_spitx_cplt = false;
		if(BSP_RunSPIDMA(IOIF_ESP32_SPI_PORT, pTxData, NULL, size, BSP_SPI_TRANSMIT_DMA) != BSP_OK)				//SPI Transfer only
			ret = IOIF_ESP32_SPI_TX_ERROR;
	}
	else if (pTxData == NULL || size < 0)
		ret = IOIF_ESP32_SPI_TX_ERROR;



	return ret;
}


IOIF_ESP32_SPI_Comm_t IOIF_ESP32_SPI_RX(uint8_t *pRxData, uint16_t size, bool cs_autoctrl)
{
	IOIF_ESP32_SPI_Comm_t ret = IOIF_ESP32_SPI_COMM_OK;

	/* ESP32 SPI Status Check */
	if((ESP32_IsSPIReady != true) || (ESP32_WIFI_SPI_StatusCheck() != true))
		return ret = IOIF_ESP32_SPI_BUSY;

	/* CS Pin Control */
	if(cs_autoctrl == true)
		wifi_spitransmit_cs_autocontrol = true;
	else
		wifi_spitransmit_cs_autocontrol = false;

	/* 1. CS is Low */
	IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_RESET); 						//CS is low,

	/* 2. Received Data */
	if(pRxData != NULL){
		if(BSP_RunSPIDMA(IOIF_ESP32_SPI_PORT, NULL, pRxData, size, BSP_SPI_RECEIVE_DMA) != BSP_OK)  //SPI Transfer & Receive
			ret = IOIF_ESP32_SPI_TX_ERROR;
	}
	else					//argument mistypo.
		ret = IOIF_ESP32_SPI_TX_ERROR;

	return ret;
}


//IOIF_ESP32_SPI_Comm_t  ESP32_WIFI_SPI_StartReceiveBuff(void)
//{
//	IOIF_ESP32_SPI_Comm_t ret = IOIF_ESP32_WIFI_SPI_COMM_OK;
//
//	/* 1. Ring buffer creation */
//	memset(&spi3_rx_ring_buff[0], 0, IOIF_ESP32_SPI3_RX_RING_BUFF_LENGTH);		//ring buffer init.
//	RingBufferCreate(&esp32_spi_rx_buff, &spi3_rx_ring_buff[0], IOIF_ESP32_SPI3_RX_RING_BUFF_LENGTH);		//ring buffer creation
//
//	/* test */
//	IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
//
//	/* 2. Run SPI RX DMA */
//	if(BSP_RunSPIDMA(IOIF_ESP32_WIFI_SPI_PORT, NULL, &spi3_rx_ring_buff[0], IOIF_ESP32_SPI3_RX_RING_BUFF_LENGTH, BSP_SPI_RECEIVE_DMA) != BSP_OK)
//		ret = IOIF_ESP32_WIFI_SPI_RX_ERROR;
//
//	/* 3. Ring buffer indicator init. */
//	esp32_spi_rx_buff.head = esp32_spi_rx_buff.length - ((DMA_Stream_TypeDef*)SPI3_RX_DMA->Instance)->NDTR;
//	esp32_spi_rx_buff.tail = esp32_spi_rx_buff.head;
//
//	return ret;
//}

//IOIF_ESP32_SPI_Comm_t  ESP32_WIFI_SPI_RXBuffFlush(void)
//{
//	IOIF_ESP32_SPI_Comm_t ret = IOIF_ESP32_WIFI_SPI_COMM_OK;
//
//	RingBufferFlush(&esp32_spi_rx_buff);
//
//	return ret;
//}
//
//IOIF_ESP32_SPI_Comm_t  ESP32_WIFI_SPI_RingBufferUpdate(void)
//{
//	IOIF_ESP32_SPI_Comm_t ret = IOIF_ESP32_WIFI_SPI_COMM_OK;
//
//	esp32_spi_rx_buff.head = esp32_spi_rx_buff.length - ((DMA_Stream_TypeDef*)SPI3_RX_DMA->Instance)->NDTR;
//	esp32_spi_rx_buff.tail = esp32_spi_rx_buff.head;
//
//	return ret;
//}

/* Callback Functions */
uint32_t IOIF_ESP32_CB_CalDataLength(void)
{
	return IOIF_UART_CalDataLength(IOIF_ESP32_UART_PORT);
}

bool IOIF_ESP32_CB_CtrlPDpin(GPIO_PinState gpio_state)
{
	bool ret = true;

	//	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, ESP32_UART_EN_Pin, gpio_state); //uart PD pin

	return ret;
}

uint8_t IOIF_ESP32_CB_ReadData(void)
{
	uint8_t rx_buffer;
	return rx_buffer = IOIF_UART_Read(IOIF_ESP32_UART_PORT);

	/* 검증 후 IOIF_BLE_CB_ReadData 대신 사용
	 * bool IOIF_BLE_UART_ReadBurst(uint8_t* readData, uint32_t length) -> 함수명 IOIF_BLE_ReadData로 수정
	{
		bool ret = true;

		if(IOIF_UART_ReadData(IOIF_ESP32_UART_PORT, readData, length) != IOIF_UART_STATUS_OK)
			ret = false;

		return ret;
	}*/

}

bool IOIF_ESP32_CB_WriteData(uint8_t* data, uint32_t length)
{
	bool ret = true;

	if(IOIF_UART_Write(IOIF_ESP32_UART_PORT, IOIF_UART_MODE_DMA, data, length) != IOIF_UART_STATUS_OK)
		return false;

	return ret;
}

/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */
static bool regBTCallbacks()
{
	bool ret = false;

	/* Register BLE Callback Functions */
	esp32_bt_CallbackStruct.esp32_bt_data_length		= IOIF_ESP32_CB_CalDataLength;
	esp32_bt_CallbackStruct.esp32_bt_connected			= IOIF_ESP32_BT_IsConnected;
	esp32_bt_CallbackStruct.esp32_bt_uart_enable 		= IOIF_ESP32_CB_CtrlPDpin;
	esp32_bt_CallbackStruct.esp32_bt_read		 		= IOIF_ESP32_CB_ReadData;
	esp32_bt_CallbackStruct.esp32_bt_write		 		= IOIF_ESP32_CB_WriteData;
	esp32_bt_CallbackStruct.esp32_bt_IOwait 			= BSP_CommonSysDelay;

	if (MDBT42Q_Init((MDBT42QAT_CallbackStruct*)&esp32_bt_CallbackStruct) == true)
		ret = true;

	/* Buffer Init. */
	memset(&BT_ACK[0], 		 0, IOIF_ESP32_BT_CMD_BUFFER_INDEX);
	memset(&BT_DataBuff[0],  0, IOIF_ESP32_BT_DATA_BUFFER_INDEX);
	memset(&BT_ATCMDBuff[0], 0, IOIF_ESP32_BT_CMD_BUFFER_INDEX);

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif /*_USE_DEBUG_CLI*/

	return ret;
}


static void updateBTStatus (uint16_t gpioPin)
{
	if ((gpioPin == IOIF_ESP32_BT_CONNECT_PIN) && (IOIF_ReadGPIOPin(IOIF_ESP32_BT_CONNECT_PORT, IOIF_ESP32_BT_CONNECT_PIN)) == IOIF_GPIO_PIN_SET) //BLE_BT_CONNECT_GPIO_Port
		bt_connection_status = true;			//BT connected
	else if ((gpioPin == IOIF_ESP32_BT_CONNECT_PIN) && (IOIF_ReadGPIOPin(IOIF_ESP32_BT_CONNECT_PORT, IOIF_ESP32_BT_CONNECT_PIN)) == IOIF_GPIO_PIN_RESET)
		bt_connection_status = false;			//BT disconnected
}


static void ESP32_SPI_TxCB(void* params)
{
	if(wifi_spitransmit_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_SET);

	esp32_spitx_cplt = true;
}

static void ESP32_SPI_RxCB(void* params)
{
	if(wifi_spitransmit_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_SET);
}

static void ESP32_SPI_TxRxCB(void* params)
{
	if(wifi_spitransmit_cs_autocontrol == true)
		IOIF_WriteGPIOPin(IOIF_ESP32_SPI_CS_PORT, IOIF_ESP32_SPI_CS_PIN, IOIF_GPIO_PIN_SET);

	esp32_spitxrx_cplt = true;
}

static void ESP32_WifiConnectCB(uint16_t gpioPin)
{
	if ((gpioPin == WIFI_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, WIFI_CONNECT_Pin)) == IOIF_GPIO_PIN_SET)
		ESP32_IsWiFiConnected = true;
	else if ((gpioPin == WIFI_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, WIFI_CONNECT_Pin)) == IOIF_GPIO_PIN_RESET)
		ESP32_IsWiFiConnected = false;
}

static void ESP32_SPIReadyCB(uint16_t gpioPin)
{
	if ((gpioPin == WIFI_SPI_RDY_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, WIFI_SPI_RDY_Pin)) == IOIF_GPIO_PIN_SET)
		ESP32_IsSPIReady = true;
	else if ((gpioPin == WIFI_SPI_RDY_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, WIFI_SPI_RDY_Pin)) == IOIF_GPIO_PIN_RESET)
		ESP32_IsSPIReady = false;
}

static bool ESP32_WIFI_SPI_StatusCheck(void)
{
	bool ret = false;

	if(IOIF_ReadGPIOPin(IOIF_GPIO_PORT_E, WIFI_SPI_RDY_Pin) == IOIF_GPIO_PIN_SET)
		ret = true;

	return ret;
}

#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */

void CLI_RunEsp32(cli_args_t *args)
{
	bool ret = false;

	/* BLUETOOTH */
	if (args->cmpStr(0, "bt") == true)
	{
		if(args->argc == 2 && args->cmpStr(1, "isconnected") == true)
		{
			bool state = bt_connection_status;

			const char* bleInitString[] = {
					"BT not connected",
					"BT connected"
			};
			CLI_Printf(" * Network Status : %s * \r\n", bleInitString[state ? 1 : 0]);

		} else if(args->argc == 2 && args->cmpStr(1, "writetest") == true)
		{
			uint32_t duration = 100; //(uint32_t) args->getData(1);

			if (bt_connection_status == false )
				CLI_Printf(" * The device is not connected to the app *\r\n");
			else
			{
				CLI_Printf(" * Check the app to see if the data is being received *\r\n");
				memcpy(uart1dmaTxBuff, IOIF_ESP32_BT_TEST_SEND_PACKET, sizeof(IOIF_ESP32_BT_TEST_SEND_PACKET));
//				memset(uart1dmaTxBuff,0,sizeof(uart1dmaTxBuff));

				while(CLI_KeepLoop())
				{
//					static uint8_t cnt = 0;
//					uart1dmaTxBuff[0] = cnt;
					if(	IOIF_ESP32_CB_WriteData(uart1dmaTxBuff, sizeof(IOIF_ESP32_BT_TEST_SEND_PACKET)) == false)
						CLI_Printf(" !! UART DMA ERROR!! \r\n");

//					if (cnt == 250) cnt = 0;
					CLI_Delay(duration);
//					cnt++;
				}
				ret = true;
			}

		} else if(args->argc == 2 && args->cmpStr(1, "readtest") == true)
		{
			uint8_t ble_rxbuff[30] = {0};
			uint32_t ble_length = 0;

			CLI_Printf(" * Check if the device is connected to the app *\r\n");

			if (bt_connection_status == false )
				CLI_Printf(" * The device is not connected to the app *\r\n");
			else
			{
				CLI_Printf(" \r\n");
				CLI_Printf(" * Send data from the APP in packets of less than 30 bytes *\r\n");
				CLI_Printf(" * Sampling rate : 50 ms *\r\n");
				CLI_Printf(" * Because of the super-fast task period, packets can be seperated during reception *\r\n");
				CLI_Printf(" \r\n");

				memset(ble_rxbuff, 0, 30);

				while(CLI_KeepLoop())
				{
					CLI_Delay(49); // The optimal second determined by experiment
					ble_length = IOIF_ESP32_BT_ReadCommData(ble_rxbuff, 30);
					if(ble_length > 0) // 수신된 바이트가 있으면,
					{
						CLI_Printf("received data : " );
						for (int i = 0; i < ble_length; i++) {
							CLI_Printf("%d ", ble_rxbuff[i]);
						}
						CLI_Printf(", byte : %d \r\n", ble_length);
					}
				}
				ret = true;
			}
		}
	}
	/* WIFI */
	else if (args->cmpStr(0, "wifi") == true)
	{
		uint8_t test_buff[IOIF_ESP32_SPI_TX_BUFF_LENGTH] = {0,};
		uint32_t esp32_spi_rx_error_cnt = 0;
		uint32_t esp32_spi_rx_cnt = 0;
		uint32_t esp32_spi_rx_error_count1 = 0;
		uint32_t esp32_spi_rx_success_trial = 0;

		if(args->argc == 2 && args->cmpStr(1, "isconnected") == true)
		{
			bool state = ESP32_IsWiFiConnected;

			const char* InitString[] = {
					"Wi-Fi not connected",
					"Wi-Fi connected"
			};
			CLI_Printf(" * Network Status : %s * \r\n", InitString[state ? 1 : 0]);

		} else if(args->argc == 2 && args->cmpStr(1, "writetest") == true)
		{
			if(IOIF_ESP32_SPI_TX(SPI3_DMA_TX_BUFF, SPI3_DMA_RX_BUFF, 40, true) == IOIF_ESP32_SPI_COMM_OK)
			{
				while(!esp32_spitxrx_cplt);
				memcpy(test_buff, SPI3_DMA_RX_BUFF, IOIF_ESP32_SPI_RX_BUFF_LENGTH);

				for(uint8_t i = 0; i < IOIF_ESP32_SPI_TX_BUFF_LENGTH; i++)
				{
					if(test_buff[i] != 0x96)
						esp32_spi_rx_error_count1++;
				}
				if(esp32_spi_rx_error_count1 == 0)
					esp32_spi_rx_success_trial++;

				esp32_spi_rx_error_cnt += esp32_spi_rx_error_count1;

				esp32_spi_rx_cnt++;
			}

		} else if(args->argc == 2 && args->cmpStr(1, "readtest") == true)
		{

		}
	}

	else if (ret == false)
	{
		CLI_Printf(" * Wifi & BT Comm. *\r\n  ");
		//			CLI_Printf("   Data exchange via low-energy Bluetooth. \r\n");
		//			CLI_Printf(" * Input data specification \r\n");
		//			CLI_Printf("   Name        : 20 characters without blank  \r\n");
		//			CLI_Printf("   Mac Address : 12byte HEX \r\n");
		CLI_Printf(" * Usage \r\n");
		CLI_Printf("   esp32 bt isconnected \r\n");
		CLI_Printf("   esp32 bt writetest \r\n");
		CLI_Printf("   esp32 bt readtest \r\n");
		CLI_Printf("   esp32 wifi isconnected \r\n");
		CLI_Printf("   esp32 wifi writetest \r\n");
		CLI_Printf("   esp32 wifi readtest \r\n");
		CLI_Printf(" * Command Example \r\n");
		CLI_Printf("   esp32 bt writetest 500 \r\n");

	}


}

#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_ESP32_ENABLED */
