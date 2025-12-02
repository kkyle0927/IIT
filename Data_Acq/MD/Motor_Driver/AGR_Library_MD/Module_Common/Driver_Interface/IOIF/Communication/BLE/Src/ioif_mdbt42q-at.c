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
  * @brief I2C ICM20608G module driver
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
static volatile bool ble_connection_status = false;
static uint8_t uart_port = IOIF_UART_1;

static uint8_t BLE_DataBuff	 	 [IOIF_BLE_DATA_BUFFER_INDEX];
static uint8_t BLE_ATCMDBuff	 [IOIF_BLE_CMD_BUFFER_INDEX];
static uint8_t BLE_ACK			 [IOIF_BLE_CMD_BUFFER_INDEX];

static uint8_t BLE_SlaveName	 [IOIF_BLE_NAME_LENGTH_MAX];
static uint8_t BLE_SlaveSerialNo [IOIF_BLE_SERIAL_LENGTH_MAX];
static uint8_t BLE_SlaveMacAd 	 [IOIF_BLE_MAC_LENGTH_MAX];

#ifdef _USE_BAREMETAL
static uint32_t* hdlr_loop_time = NULL;
#endif


/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* BLE connect status GPIO Callback */
static void SetBLEStatusCB (uint16_t gpioPin);

#ifdef _USE_BAREMETAL
void sync_time_counter(uint32_t* task_looptime);
#endif

/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* BLE connect status GPIO Callback Registration */
void IOIF_BLE_ConnectGPIO_CB_Set(void)
{
	IOIF_SetGPIOCB(BLE_BT_CONNECT_Pin, IOIF_GPIO_EXTI_CALLBACK, SetBLEStatusCB);
}

/* BLE Device Identifier */
bool IOIF_BLE_Ready()
{
	bool ret = false;

	/* Register BLE Callback Functions */
	BLE_CallbackStruct.mdbt42qat_ready		 		= IOIF_BLE_UART_IsReady;
	BLE_CallbackStruct.mdbt42qat_GPIO_PDpin_write 	= IOIF_BLE_Write_GPIO_State;
	BLE_CallbackStruct.mdbt42qat_baudrate_get 		= IOIF_BLE_UART_GetBaudrate;
	BLE_CallbackStruct.mdbt42qat_baudrate_set 		= IOIF_BLE_UART_SetBaudrate;
	BLE_CallbackStruct.mdbt42qat_read		 		= IOIF_BLE_UART_Read;
	BLE_CallbackStruct.mdbt42qat_write		 		= IOIF_BLE_UART_Write;
	BLE_CallbackStruct.mdbt42qat_HW_default		 	= IOIF_BLE_HWDefault;

	if (MDBT42Q_Init(&BLE_CallbackStruct) == true)
		ret = true;

	/* BLE Module Identifier : 이름 및 Serial Number 설정 */
	MDBT42Q_SetNodeName		((uint8_t*)IOIF_BLE_SLAVE_NODENAME, BLE_SlaveName);
	MDBT42Q_SetNodeSerial	((uint8_t*)IOIF_BLE_SLAVE_SERIAL,	BLE_SlaveSerialNo);
	MDBT42Q_SetNodeMac		((uint8_t*)IOIF_BLE_SLAVE_MAC,		BLE_SlaveMacAd);

	/* Buffer Init. */
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_DataBuff[0], 0, IOIF_BLE_DATA_BUFFER_INDEX);
	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);

	return ret;
}

bool IOIF_BLE_ConfigParameters_Set(void) {  //Todo : 9600, 115200 외에도 가능하도록 수정

	bool ret = false;

	uint8_t state = 0;
	volatile uint16_t Isbaudrate9600 = 0;					// Init 함수의 state machine 변수
	static uint8_t timeout_trial = 0;	// Timeout Trial 변수 (최대 3회)
	static uint32_t buffer_index = 0;	// UART RX Ring buffer 의 index



	/* 1. BLE Module 의 현재 baudrate 확인 */
	if (MDBT42Q_AT_ReadCMD((uint8_t*)&BLE_SLAVE_READ_BAUDRATE[0], sizeof(BLE_SLAVE_READ_BAUDRATE)) == true)
	{
		buffer_index = MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		if ((buffer_index - 1) > 0)
		{																		// AT CMD 를 다 읽으면,
			memcpy(&BLE_ACK[0], &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
			if(strncmp((char*)BLE_ATCMDBuff, (char*) IOIF_BLE_ACK_BAUDRATE9600, sizeof(IOIF_BLE_ACK_BAUDRATE9600)) == 0)
				Isbaudrate9600 = 1;
			else
				Isbaudrate9600 = 0;
		}
	} else
		state = IOIF_BLE_INIT_ERROR;

	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);



	/* 2. Baud rate 가 9600 일 경우 BLE baudrate 를 115200 으로 세팅 */
	if (Isbaudrate9600 == 1)
	{
		if(MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SLAVE_WRITE_BAUD115200[0], sizeof(BLE_SLAVE_WRITE_BAUD115200), Save_Active) == true)
		{
			buffer_index = MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
			if ((buffer_index - 1) > 0)																		// AT CMD 를 다 읽으면,
			{
				memcpy(&BLE_ACK[0], &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
				if(strncmp((char*)BLE_ATCMDBuff, (char*) IOIF_BLE_ACK_BAUD115200_SUCCESS, sizeof(IOIF_BLE_ACK_BAUD115200_SUCCESS)) == 0)
					state = IOIF_BLE_INIT_OK;
				else
					state = IOIF_BLE_INIT_ACK_ERROR;
			}
		} else
			state = IOIF_BLE_INIT_ERROR;
	}
	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);


	/* 3. MCU 의 UART Baudrate 를 115200 으로 변경 */
	if(IOIF_UART_SetBaudrate(uart_port, 115200) == false)
		state = IOIF_UART_INIT_FAIL;

	osDelay(BLE_CMD_WAIT_150MS); //Essential for state 4: Under 150ms, ack operates unstably.

	/* 4. MCU - BLE Module 간 baudrate 가 115200 인지 확인 */
	if(MDBT42Q_AT_ReadCMD((uint8_t*)&BLE_SLAVE_READ_BAUDRATE[0], sizeof(BLE_SLAVE_READ_BAUDRATE)) == true)
	{
		buffer_index = MDBT42Q_ReceivedData(uart_port, &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		if ((buffer_index - 1) > 0)
		{
			memcpy(&BLE_ACK[0], &BLE_ATCMDBuff[0], IOIF_BLE_CMD_BUFFER_INDEX);
			if(strncmp((char*)BLE_ATCMDBuff, (char*)IOIF_BLE_ACK_BAUDRATE115200, sizeof(IOIF_BLE_ACK_BAUDRATE115200)) == 0)
				state = IOIF_BLE_INIT_OK;
			else
				state = IOIF_BLE_INIT_ACK_ERROR;

		}
	} else
		state = IOIF_BLE_INIT_ERROR;

	memset(&BLE_ATCMDBuff[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);
	memset(&BLE_ACK[0], 0, IOIF_BLE_CMD_BUFFER_INDEX);


	/* 5. BLE Module 의 Name, Serial Number, Mac Address 설정 */
	if (state == IOIF_BLE_INIT_OK)
	{

		if(MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SlaveMacAd[0], sizeof(BLE_SlaveMacAd), Save_Active) == false )
			state = IOIF_BLE_INIT_ERROR;

		osDelay(BLE_CMD_WAIT_150MS); //Essential for a stable save: Under 150ms, settings are unstably saved.

		if(MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SlaveSerialNo[0], sizeof(BLE_SlaveSerialNo), Save_Active) == false )
			state = IOIF_BLE_INIT_ERROR;

		osDelay(BLE_CMD_WAIT_150MS); //Essential for a stable save: Under 150ms, settings are unstably saved.

		if(MDBT42Q_AT_WriteCMD((uint8_t*)&BLE_SlaveName[0], sizeof(BLE_SlaveName), Save_Active) == false )
			state = IOIF_BLE_INIT_ERROR;
	}


	/* 모든 Initialization 과정이 Timeout 내에 실행되지 않으면 Recovery, Default 로 복구 :
	 * 1. HW Default 수행, 2. MCU UART 를 9600 으로 변경
	 * 2. Timeout Trial 은 최대 3회까지
	 * 3. 3회 이후는 BLE device initialization fail 을 return 하고 BLE 기능을 사용하지 않는다 */

#ifdef _USE_BAREMETAL
	if (*hdlr_loop_time > BLE_INIT_TIMEOUT)
	{
		state = BLE_ERROR_STATE;												// Transition to Error state
		MDBT42Q_HW_Default();									// HW default 수행

//			_BLE_IO_State->_BLE_state = BLE_IO_INIT;
			if(IOIF_UART_SetBaudrate(uart_port, 9600) != true)
				timeout_trial++;
			state = 0;															// State Machine 초기화
			*hdlr_loop_time = 0;												// loop time count 초기화
			IOIF_UART_RX_BufferFlush(uart_port);									// UART RX Buffer Flush
			memset(&BLE_ATCMDBuff[0], 0, CMD_BUFFER_INDEX);						// CMD buffer clear
			memset(&BLE_ACK[0], 0, CMD_BUFFER_INDEX);							// CMD buffer clear

	}
#endif

	if (timeout_trial > 2)
		return false;

	return ret;
}

/* BLE connection check */
bool IOIF_IsBLEConnected(void)
{
	bool ret = false;

	if(ble_connection_status == true)
	{
		ret = true;  																//When BT is connected,
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_UART_EN_Pin, IOIF_GPIO_PIN_RESET);	//UART PD is low
	} else
	{
		ret = false;  																//When BT is disconnected,
		IOIF_WriteGPIOPin(IOIF_GPIO_PORT_E, BLE_UART_EN_Pin, IOIF_GPIO_PIN_SET);	//UART PD is high
	}

	return ret;
}


/* BLE Data_Read */
int IOIF_BLE_DataRead_Test(void)
{

//	MDBT42Q_ReceivedData(uart_port, &BLE_DataBuff[0], DATA_BUFFER_INDEX);
#ifdef _USE_BAREMETAL
	static uint32_t current_time=0;

	if(*hdlr_loop_time - current_time >= 5)		//통신 주기 handling
	{
		MDBT42Q_ReceivedData(uart_port, &BLE_DataBuff[0], DATA_BUFFER_INDEX);
		current_time = *hdlr_loop_time;
	}
#else
	MDBT42Q_ReceivedData(uart_port, &BLE_DataBuff[0], IOIF_BLE_DATA_BUFFER_INDEX);
#endif
	return 0;
}


int IOIF_BLE_DataWrite_Test(void)
{

#ifdef _USE_BAREMETAL
	static uint32_t ctime=0;

	if(*hdlr_loop_time - ctime >= 200)  //!!주기 없이 데이터 전송 시 다른 task들 hold 상태됨
	{
	/* BT device 간 연결 상태가 아니라면 BLE data 송신하지 않음*/
	if(IOIF_IsBLEConnected() != false)
		MDBT42Q_TransmitData((uint8_t*)TEST_SEND_PACKET, sizeof(TEST_SEND_PACKET));
	else
		return 0;				// BT 가 연결 중이 아니라면 데이터 송신 불가..
	ctime = *hdlr_loop_time;

	}
#else
	MDBT42Q_TransmitData((uint8_t*)IOIF_BLE_TEST_SEND_PACKET, sizeof(IOIF_BLE_TEST_SEND_PACKET));

	/*static uint8_t test[1] = {0};
	test[0]++;

	MDBT42Q_TransmitData((uint8_t*)test, sizeof(test));

	if (test[0] == 100) {test[0] = 0;}*/
#endif

	return 0;
}


void IOIF_BLE_Ack_Test()
{
	static uint8_t test_buff	[IOIF_BLE_CMD_BUFFER_INDEX];

	if ( MDBT42Q_ReceivedData(uart_port, &test_buff[0], IOIF_BLE_CMD_BUFFER_INDEX) > 0)
	{
		memcpy(&BLE_ACK[0], &test_buff[0], IOIF_BLE_CMD_BUFFER_INDEX);
		MDBT42Q_TransmitData((uint8_t*)test_buff, sizeof(test_buff));
	}

}



/* Callback Functions */

uint32_t IOIF_BLE_UART_IsReady(void)
{
	return IOIF_UART_IsReady(uart_port);
}

bool IOIF_BLE_Write_GPIO_State (GPIO_State gpio_state)
{
	bool ret = true;

	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_UART_EN_Pin, gpio_state);

	return ret;
}

uint32_t IOIF_BLE_UART_GetBaudrate (void)
{
	uint32_t baud;
	return baud = IOIF_UART_GetBaudrate(uart_port);
}


bool IOIF_BLE_UART_SetBaudrate (uint32_t baudrate)
{
	bool ret = true;

	if(IOIF_UART_SetBaudrate(uart_port, baudrate) != true)
		return false;

	return ret;
}


uint8_t IOIF_BLE_UART_Read(void)
{
	uint8_t rx_buffer;
	return rx_buffer = IOIF_UART_Read(uart_port);
}


bool IOIF_BLE_UART_Write(uint8_t* data, uint32_t length)
{
	bool ret = true;

	if(IOIF_UART_Write(uart_port, IOIF_UART_MODE_POLLING, data, length) != true)
		return false;

	return ret;
}

bool IOIF_BLE_HWDefault (GPIO_State gpio_state)
{
	bool ret = true;

	BSP_WriteGPIOPin(BSP_GPIO_PORT_E, BLE_nDEFAULT_Pin, gpio_state);

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
static void SetBLEStatusCB (uint16_t gpioPin)
{
	if ((gpioPin == BLE_BT_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BLE_BT_CONNECT_Pin)) == IOIF_GPIO_PIN_SET) //BLE_BT_CONNECT_GPIO_Port
		ble_connection_status = false;			//BT disconnected
	else if ((gpioPin == BLE_BT_CONNECT_Pin) && (IOIF_ReadGPIOPin(IOIF_GPIO_PORT_D, BLE_BT_CONNECT_Pin)) == IOIF_GPIO_PIN_RESET)
		ble_connection_status = true;			//BT connected
}

#endif /* IOIF_MDBT42Q_AT_ENABLED */


