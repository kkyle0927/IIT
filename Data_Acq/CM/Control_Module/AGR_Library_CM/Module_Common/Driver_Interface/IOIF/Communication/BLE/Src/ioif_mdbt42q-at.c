/**
 *-----------------------------------------------------------
 *                MDBT48Q-AT BLE Slave Device
 *-----------------------------------------------------------
 * @file ioif_mdbt42q-at.c
 * @date Created on: Sep 19, 2023
 * @author AngelRobotics HW Team
 * @brief Code for the MDBT48Q-AT BLE Slave Device.
 *
 *
 * Refer to the MDBT48Q-AT datasheet and related documents for more information.
 * @ref MDBT48Q-AT Datasheet
 */

#include "ioif_mdbt42q-at.h"

/** @defgroup UART UART
 * @brief UART MDBT42Q-AT BLE module driver
 * @{
 */
#ifdef IOIF_MDBT42Q_AT_ENABLED

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */

MDBT42QAT_CallbackStruct BLE_CallbackStruct = {
		NULL, NULL, NULL, NULL, NULL, NULL, NULL
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
static uint8_t uart_port = IOIF_UART1;

static uint8_t BLE_DataBuff	 	 [IOIF_BLE_DATA_BUFFER_INDEX];
static uint8_t BLE_ATCMDBuff	 [IOIF_BLE_CMD_BUFFER_INDEX];
static uint8_t BLE_ACK			 [IOIF_BLE_CMD_BUFFER_INDEX];

static uint8_t BLE_SlaveName	 [IOIF_BLE_NAME_LENGTH_MAX];
static uint8_t BLE_SlaveMacAd 	 [IOIF_BLE_MAC_LENGTH_MAX];

bool ble_connection_status __attribute__((section(".uart1dmaRxBuff"))); //cash 영향으로 메모리 영역 옮김

#ifdef _USE_BAREMETAL
static uint32_t* hdlr_loop_time = NULL;
#endif

#ifdef _USE_DEBUG_CLI
static bool moduleinit_res = false;
#endif /*_USE_DEBUG_CLI*/

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */
static IOIF_BLEInit_t readConfigSet(const char* at_cmd, const char* setting);
static IOIF_BLEInit_t writeConfigSet(const char* at_cmd, const char* setting, SettingSave_Option save_op);

/* BLE connect status GPIO Callback */
static void updateBLEStatus (uint16_t gpioPin);
static bool regBLECallbacks();

#ifdef _USE_BAREMETAL
void sync_time_counter(uint32_t* task_looptime);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

bool IOIF_BLE_Init(uint32_t baudrate)
{
	bool ret = false;
	ble_connection_status = false;

	IOIF_BLE_ResetHW();					// Reset Assert
	ret = regBLECallbacks();
	/* UART Start */
	if (IOIF_UART_StartReceive(uart_port, baudrate, IOIF_UART_MODE_DMA) != IOIF_UART_STATUS_OK)			// Transmit : Normal, Received : DMA
		ret = false;

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif /*_USE_DEBUG_CLI*/

	return ret;
}

IOIF_BLEInit_t IOIF_BLE_SetBaudrate115200(void)
{
	IOIF_BLEInit_t state = IOIF_BLE_INIT_OK;

	if(IOIF_UART_SetBaudrate(uart_port, 9600) == IOIF_UART_STATUS_ERROR) 						/* MCU uart 9600으로 설정*/
		return IOIF_BLE_INIT_ERROR;

	state = readConfigSet(BLE_SLAVE_READ_BAUDRATE, IOIF_BLE_ACK_BAUDRATE9600);  				/* 1. BLE Module 의 현재 baudrate 확인 */

	if(state == IOIF_BLE_INIT_OK) //IOIF_BLE_INIT_OK : 9600, IOIF_BLE_INIT_ACK_ERROR : 115200 	/* 2. Baud rate 가 9600 일 경우 BLE baudrate 를 115200 으로 세팅 */
	{
		state = writeConfigSet(BLE_SLAVE_WRITE_BAUD115200, IOIF_BLE_ACK_BAUD115200_SUCCESS, Save_Active);
		if (state != IOIF_BLE_INIT_OK)
			return IOIF_BLE_INIT_WRITE_ERROR;
	}

	if (IOIF_UART_SetBaudrate(uart_port, 115200) == IOIF_UART_STATUS_ERROR) 					/* 3. MCU 의 UART Baudrate 를 115200 으로 변경 */
		return IOIF_BLE_INIT_ERROR;

	memset(&rx_packet[0], 0, 100);
	BSP_CommonSysDelay(BLE_CMD_WAIT_150MS); //Essential for state 4: Under 150ms, ack operates unstably.

	state = readConfigSet(BLE_SLAVE_READ_BAUDRATE, IOIF_BLE_ACK_BAUDRATE115200);				/* 4. MCU - BLE Module 간 baudrate 가 115200 인지 확인 */

#ifdef _USE_DEBUG_CLI
	if (state == IOIF_BLE_INIT_OK) 	moduleinit_res = true;
	else							moduleinit_res = false;
#endif /*_USE_DEBUG_CLI*/

	return state;
}

IOIF_BLEInit_t IOIF_BLE_SetConfigParams(uint8_t* mac, uint8_t* name) {

	IOIF_BLEInit_t state = IOIF_BLE_INIT_OK;

	/* BLE Module Identifier */
	MDBT42Q_SetNodeName		(name, 	BLE_SlaveName);
	MDBT42Q_SetNodeMac		(mac,	BLE_SlaveMacAd);

	if(writeConfigSet((const char*)BLE_SlaveMacAd, (const char*)mac, Save_Disable) != IOIF_BLE_INIT_OK)
		state = IOIF_BLE_INIT_WRITE_ERROR ;
	if(writeConfigSet((const char*)BLE_SlaveName, (const char*)name, Save_Active) != IOIF_BLE_INIT_OK)
		state = IOIF_BLE_INIT_WRITE_ERROR ;

	return state;
}

void IOIF_BLE_ResetHW(void)
{
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_RESET_Pin, IOIF_GPIO_PIN_SET);
	BSP_CommonSysDelay(BLE_CMD_WAIT_100MS);
	IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_RESET_Pin, IOIF_GPIO_PIN_RESET);
}

void IOIF_BLE_SetToDefaultSW()
{
	MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SLAVE_WRITE_DEFAULT[0], sizeof(BLE_SLAVE_WRITE_DEFAULT), Save_Active);
	BSP_CommonSysDelay(BLE_CMD_WAIT_250MS);
}

void IOIF_BLE_SetToDefaultHW()
{
	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_nDEFAULT_Pin, BSP_GPIO_PIN_RESET);
	BSP_CommonSysDelay(BLE_CMD_WAIT_700MS); //ref : datasheet
	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_nDEFAULT_Pin, BSP_GPIO_PIN_SET);
}

uint32_t IOIF_BLE_ReadCommData(uint8_t* buffer, uint32_t length)
{	/* 링버퍼로부터 읽은 바이트 수를 return, buffer에 data copy */
	return MDBT42Q_ReceivedData(uart_port, , length);
}

void IOIF_BLE_TransmitCommData(uint8_t* dataBuff, uint32_t dataSize)
{

#ifdef _USE_BAREMETAL
	static uint32_t ctime=0;

	if(*hdlr_loop_time - ctime >= 200)  //!!주기 없이 데이터 전송 시 다른 task들 hold 상태됨
	{
		/* BT device 간 연결 상태가 아니라면 BLE data 송신하지 않음*/
		if(IOIF_BLE_IsConnected() != false)
			MDBT42Q_TransmitData((uint8_t*)TEST_SEND_PACKET, sizeof(TEST_SEND_PACKET));
		else
			return 0;				// BT 가 연결 중이 아니라면 데이터 송신 불가..
		ctime = *hdlr_loop_time;

	}
#else
	MDBT42Q_TransmitData(dataBuff, dataSize);

#endif /* _USE_BAREMETAL*/

}

bool IOIF_BLE_IsConnected()
{
	if (ble_connection_status == true)  IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_UART_EN_Pin, IOIF_GPIO_PIN_RESET);
	else 								IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_UART_EN_Pin, IOIF_GPIO_PIN_SET);

	return ble_connection_status;
}

bool IOIF_BLE_StopPairing()
{
	return MDBT42Q_AT_WriteCMD((uint8_t*)BLE_SLAVE_WRITE_ADV_STOP, strlen(BLE_SLAVE_WRITE_ADV_STOP), Save_Disable);
}


bool IOIF_BLE_StartPairing()
{
	return MDBT42Q_AT_WriteCMD((uint8_t*)BLE_SLAVE_WRITE_ADV_START, strlen(BLE_SLAVE_WRITE_ADV_START), Save_Disable);
}

bool IOIF_BLE_Disconnect(){

	return MDBT42Q_AT_WriteCMDwhenConnected((uint8_t*)BLE_SLAVE_WRITE_DISCONNECT, sizeof(BLE_SLAVE_WRITE_DISCONNECT), Save_Disable);
}


/* Test code */
void IOIF_BLE_AckTest()
{
	static uint8_t test_buff	[IOIF_BLE_CMD_BUFFER_INDEX];

	if ( MDBT42Q_ReceivedData(uart_port, &test_buff[0], IOIF_BLE_CMD_BUFFER_INDEX) > 0)
	{
		memcpy(&BLE_ACK[0], &test_buff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		MDBT42Q_TransmitData((uint8_t*)test_buff, sizeof(test_buff));
	}
}

/* Callback Functions */
uint32_t IOIF_BLE_CB_CalDataLength(void)
{
	return IOIF_UART_CalDataLength(uart_port);
}

bool IOIF_BLE_CB_CtrlPDpin(GPIO_State gpio_state)
{
	bool ret = true;

	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_UART_EN_Pin, gpio_state); //uart PD pin

	return ret;
}

bool IOIF_BLE_CB_CtrlHWdefaultpin(GPIO_State gpio_state)
{
	bool ret = true;

	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_nDEFAULT_Pin, gpio_state); //flash default

	return ret;
}

uint8_t IOIF_BLE_CB_ReadData(void)
{
	uint8_t rx_buffer;
	return rx_buffer = IOIF_UART_Read(uart_port);

	/* 검증 후 IOIF_BLE_CB_ReadData 대신 사용
	 * bool IOIF_BLE_UART_ReadBurst(uint8_t* readData, uint32_t length) -> 함수명 IOIF_BLE_ReadData로 수정
	{
		bool ret = true;

		if(IOIF_UART_ReadData(uart_port, readData, length) != IOIF_UART_STATUS_OK)
			ret = false;

		return ret;
	}*/

}

bool IOIF_BLE_CB_WriteData(uint8_t* data, uint32_t length)
{
	bool ret = true;

	if(IOIF_UART_Write(uart_port, IOIF_UART_MODE_DMA, data, length) != IOIF_UART_STATUS_OK)
		return false;

	return ret;
}


#ifdef _USE_BAREMETAL
/* Asynchronous Wait Timer init. */
void IOIF_BLE_Sync_IimeCounter(uint32_t* task_looptime)
{
	hdlr_loop_time = task_looptime;
}

#endif


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static IOIF_BLEInit_t readConfigSet(const char* at_cmd, const char* setting) {

	IOIF_BLEInit_t state = IOIF_BLE_INIT_ERROR;

	if (MDBT42Q_AT_ReadCMD((uint8_t*)at_cmd, IOIF_BLE_CMD_BUFFER_INDEX) == true)
	{
		BSP_CommonSysDelay(BLE_CMD_WAIT_250MS); //essential for stable configuration

		MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		memcpy(&BLE_ACK[0], &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		if (strncmp((char*)BLE_ACK, setting, strlen(setting)) == 0)
			state = IOIF_BLE_INIT_OK;
		else
			state = IOIF_BLE_INIT_ACK_ERROR;

	} else
	{
		state = IOIF_BLE_INIT_READ_ERROR;
	}

	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);

	return state;
}

static IOIF_BLEInit_t writeConfigSet(const char* at_cmd, const char* setting, SettingSave_Option save_op) {

	IOIF_BLEInit_t state = IOIF_BLE_INIT_ERROR;

	if(MDBT42Q_AT_WriteCMD((uint8_t*)at_cmd, IOIF_BLE_CMD_BUFFER_INDEX, save_op) == true)
	{
		MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		BSP_CommonSysDelay(BLE_CMD_WAIT_250MS); //Essential for a stable save: Under 150ms, settings are unstably saved.
		if (strncmp((char*)BLE_ATCMDBuff, setting, strlen(setting)) == 0)
			state = IOIF_BLE_INIT_OK;
		else
			state = IOIF_BLE_INIT_ACK_ERROR;

	} else
	{
		state = IOIF_BLE_INIT_WRITE_ERROR;
	}

	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);

	return state;
}


/* Init - Register CB */
static void updateBLEStatus (uint16_t gpioPin)
{
	if ((gpioPin == BLE_BT_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BLE_BT_CONNECT_Pin)) == IOIF_GPIO_PIN_SET) //BLE_BT_CONNECT_GPIO_Port
		ble_connection_status = false;			//BT disconnected
	else if ((gpioPin == BLE_BT_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BLE_BT_CONNECT_Pin)) == IOIF_GPIO_PIN_RESET)
		ble_connection_status = true;			//BT connected
}


static bool regBLECallbacks()
{
	bool ret = false;

	/* BLE connect status GPIO Callback Registration */
	IOIF_SetGPIOCB(BLE_BT_CONNECT_Pin, IOIF_GPIO_EXTI_CALLBACK, updateBLEStatus);

	/* Register BLE Callback Functions */
	BLE_CallbackStruct.mdbt42qat_data_length		= IOIF_BLE_CB_CalDataLength;
	BLE_CallbackStruct.mdbt42qat_connected			= IOIF_BLE_IsConnected;
	BLE_CallbackStruct.mdbt42qat_uart_enable 		= IOIF_BLE_CB_CtrlPDpin;
	BLE_CallbackStruct.mdbt42qat_hw_default		 	= IOIF_BLE_CB_CtrlHWdefaultpin;
	BLE_CallbackStruct.mdbt42qat_read		 		= IOIF_BLE_CB_ReadData;
	BLE_CallbackStruct.mdbt42qat_write		 		= IOIF_BLE_CB_WriteData;
	BLE_CallbackStruct.mdbu42qat_IOwait 			= BSP_CommonSysDelay;

	if (MDBT42Q_Init(&BLE_CallbackStruct) == true)
		ret = true;

	/* Buffer Init. */
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_DataBuff[0], 0, IOIF_BLE_DATA_BUFFER_INDEX);
	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);

#ifdef _USE_DEBUG_CLI
	moduleinit_res = ret;
#endif /*_USE_DEBUG_CLI*/

	return ret;
}


#ifdef _USE_DEBUG_CLI
/**
 *------------------------------------------------------------
 *                      CLI FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions are supported Commmand Line Interface (CLI).
 */
bool ReadAndPrintBLEConfig(const char* read_cmd, size_t cmd_size) {

	if (MDBT42Q_AT_ReadCMD((uint8_t*)read_cmd, cmd_size) == true) {
		BSP_CommonSysDelay(BLE_CMD_WAIT_250MS); //essential for stable configuration

		if (MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX) > 0) {
			memset(&BLE_ACK, 0, IOIF_BLE_CMD_BUFFER_INDEX);
			memcpy(&BLE_ACK[0], &BLE_ATCMDBuff[0], cmd_size);
			//            CLI_Printf(" * BLE %s : %s  \r\n", config_name, BLE_ATCMDBuff);
			return true;
		}
	}
	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	return false;
}

void CLI_RunMdbt42qat(cli_args_t *args)
{
	bool ret = false;

	if(moduleinit_res == false)
	{
		CLI_Printf(" * !! MDBT42Q-AT init FAILED !! * \r\n");
		CLI_Printf(" * !! Please RESTART MCU !! * \r\n");

		moduleinit_res = true;

	} else if(args->argc == 3 && args->cmpStr(0, "set") == true)
	{
		char* device_mac;
		char* device_name;

		IOIF_BLEInit_t config_check;

		device_name   = (char*)args->getStr(1);
		device_mac 	  = (char*)args->getStr(2);

		CLI_Printf(" * Setting up the BLE device... * \r\n");
		CLI_Printf(" \r\n");

		if (IOIF_BLE_SetConfigParams((uint8_t*)device_mac, (uint8_t*)device_name) == IOIF_BLE_INIT_OK)
		{
			BSP_CommonSysDelay(BLE_CMD_WAIT_250MS); //Essential for a stable ack

			config_check = readConfigSet(BLE_SLAVE_READ_NAME, device_name);
			IOIF_UART_UpdateHeadTail();
			if 		(config_check == IOIF_BLE_INIT_OK) 			CLI_Printf(" * BLE Device Name          : %s  \r\n", device_name);
			else if (config_check == IOIF_BLE_INIT_ACK_ERROR) 	CLI_Printf(" * Check the baudrate or settings *\r\n");
			else if (config_check == IOIF_BLE_INIT_READ_ERROR) 	CLI_Printf(" !! BLE Device NAME set FAIL !!\r\n");\

			config_check = readConfigSet(BLE_SLAVE_READ_MACADDR, device_mac);
			IOIF_UART_UpdateHeadTail();
			if 		(config_check == IOIF_BLE_INIT_OK) 			CLI_Printf(" * BLE device MAC address   : %s  \r\n", device_mac);
			else if (config_check == IOIF_BLE_INIT_ACK_ERROR) 	CLI_Printf(" * Check the baudrate or settings *\r\n");
			else if (config_check == IOIF_BLE_INIT_READ_ERROR) 	CLI_Printf(" !! BLE Device MAC ADDRESS set FAIL !! \r\n");

		} else
		{
			CLI_Printf(" !! Failed to set up the device  \r\n");
			CLI_Printf(" * Please reset the device and try again * \r\n");
		}

	} else if (args->argc == 2 && args->cmpStr(0, "default") == true)
	{
		CLI_Printf(" * Restoring the BLE device to default settings * \r\n\r\n");

		if (args->cmpStr(1, "hw") == true)
		{
			IOIF_BLE_SetToDefaultHW();
			IOIF_SysBlockingDelay(500); //delay 없이 사용하면 cmd 전송 안됨

			if (readConfigSet(BLE_SLAVE_READ_NAME, IOIF_BLE_DEFAULT_NAME) == IOIF_BLE_INIT_OK)
			{
				IOIF_UART_UpdateHeadTail();
				CLI_Printf(" * BLE Device name     : %s  \r\n", IOIF_BLE_DEFAULT_NAME);

			} else
				CLI_Printf(" !! BLE Device default set FAILED !!\r\n");

		} else if (args->cmpStr(1, "sw") == true)
		{
			IOIF_BLE_SetToDefaultSW();
			MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);

			if (readConfigSet(BLE_SLAVE_READ_NAME, IOIF_BLE_DEFAULT_NAME) == IOIF_BLE_INIT_OK)
			{
				IOIF_UART_UpdateHeadTail();
				CLI_Printf(" * BLE Device name     : %s  \r\n", IOIF_BLE_DEFAULT_NAME);

			} else
			{
				CLI_Printf(" !! BLE Device default set FAILED !!\r\n");
				CLI_Printf(" * Check if BLE is connected. If it is, please disconnect and try again * \r\n");
				CLI_Printf(" * If not, please use hw default * \r\n");
			}
		}


	} else if (args->argc == 2 && args->cmpStr(0, "get") == true)
	{
		if (ReadAndPrintBLEConfig(BLE_SLAVE_READ_NAME, sizeof(BLE_SLAVE_READ_NAME)) == true)
			CLI_Printf(" * BLE Device Name          : %s  \r\n", BLE_ATCMDBuff);
		else CLI_Printf(" !! Failed to get device NAME !! \r\n");

		if (ReadAndPrintBLEConfig(BLE_SLAVE_READ_MACADDR, sizeof(BLE_SLAVE_READ_MACADDR)) == true)
			CLI_Printf(" * BLE device MAC address   : %s  \r\n", BLE_ATCMDBuff);
		else CLI_Printf(" !! Failed to get device MAC ADDRESS !! \r\n ");

	} else if (args->argc == 2 && args->cmpStr(0, "restart") == true)
	{
		IOIF_BLE_ResetHW();
		CLI_Printf(" * *\r\n");
	}
	/*------------ Use after connected  to the app ------------*/
	else if(args->argc == 1 && args->cmpStr(0, "isconnected") == true)
	{
		bool state = ble_connection_status;

		const char* bleInitString[] = {
				"BLE not connected",
				"BLE connected"
		};
		CLI_Printf(" * Network Status : %s * \r\n", bleInitString[state ? 1 : 0]);

	} else if(args->argc == 2 && args->cmpStr(0, "writetest") == true)
	{
		uint32_t duration = (uint32_t) args->getData(1);

		CLI_Printf(" * Check if the device is connected to the app *\r\n");

		if (IOIF_BLE_IsConnected() == false )
			CLI_Printf(" * The device is not connected to the app *\r\n");
		else
		{
			CLI_Printf(" * Check the app to see if the data is being received *\r\n");
			memcpy(uart1dmaTxBuff, IOIF_BLE_TEST_SEND_PACKET, sizeof(IOIF_BLE_TEST_SEND_PACKET));
			while(CLI_KeepLoop())
			{
				//				test_time++;
				//				if(!(test_time % duration))
				//					IOIF_BLE_TransmitCommData((uint8_t*)IOIF_BLE_TEST_SEND_PACKET, sizeof(IOIF_BLE_TEST_SEND_PACKET));
				//					IOIF_BLE_TransmitCommData(&uart1dmaTxBuff, sizeof(IOIF_BLE_TEST_SEND_PACKET));
				//					IOIF_BLE_TransmitCommData(uart1dmaTxBuff, sizeof(IOIF_BLE_TEST_SEND_PACKET));

				IOIF_BLE_TransmitCommData(uart1dmaTxBuff, sizeof(IOIF_BLE_TEST_SEND_PACKET));
				CLI_Delay(duration);
			}
			ret = true;
		}

	} else if(args->argc == 1 && args->cmpStr(0, "readtest") == true)
	{
		uint8_t ble_rxbuff[30] = {0};
		uint32_t ble_length = 0;

		CLI_Printf(" * Check if the device is connected to the app *\r\n");

		if (IOIF_BLE_IsConnected() == false )
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
				ble_length = IOIF_BLE_ReadCommData(ble_rxbuff, 30);
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
	} else if(args->argc == 1 && args->cmpStr(0, "disconnect") == true)
	{
		if (IOIF_BLE_IsConnected() == false )
		{
			CLI_Printf(" * The device is not connected to the app *\r\n");
		} else
		{
			IOIF_BLE_Disconnect();

			if (IOIF_BLE_IsConnected() == false)
				CLI_Printf(" * BLE disconnect SUCCESS *\r\n");
			else
				CLI_Printf(" * BLE disconnect FAILED *\r\n");
		}
	}

	else if (ret == false)
	{
		CLI_Printf(" * BLE Comm. *\r\n  ");
		CLI_Printf("   Data exchange via low-energy Bluetooth. \r\n");
		CLI_Printf(" * Input data specification \r\n");
		CLI_Printf("   Name        : 20 characters without blank  \r\n");
		CLI_Printf("   Mac Address : 12byte HEX \r\n");
		CLI_Printf(" * Usage \r\n");
		CLI_Printf("   mdbt42qat set [name] [mac number]  \r\n");
		CLI_Printf("   mdbt42qat default [hw/sw] \r\n");
		CLI_Printf("   mdbt42qat get settings \r\n");
		CLI_Printf("   mdbt42qat restart hw \r\n");
		CLI_Printf("   -- Use after connected to the app -- \r\n");
		CLI_Printf("   mdbt42qat isconnected \r\n");
		CLI_Printf("   mdbt42qat writetest [duration(ms)] \r\n");
		CLI_Printf("   mdbt42qat readtest \r\n");
		CLI_Printf("   mdbt42qat disconnect \r\n");
		CLI_Printf(" * Command Example \r\n");
		CLI_Printf("   mdbt42qat isinit \r\n");
		CLI_Printf("   mdbt42qat set AngelSUIT_Test CBCDEF123456  \r\n");
		CLI_Printf("   mdbt42qat writetest 500 \r\n");


	}
}

#endif /*_USE_DEBUG_CLI*/
#endif /* IOIF_MDBT42Q_AT_ENABLED */


